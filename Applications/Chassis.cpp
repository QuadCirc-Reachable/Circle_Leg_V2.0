#include "Chassis.hpp"

#include "Matrixf.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "Quaternion.hpp"

namespace Applications
{
using namespace Core::Drivers;

// PID Parameters for Active Suspension (Comfort Mode)
// Adjust these based on actual tuning
// Output limit is now in METERS.
// Max travel is 2*r = 130mm = 0.13m. Set limit to 0.15m to allow full range.
static Core::Control::PID::Param roll_pid_param(0.015f, 0.0002f, 0.00012f, 1000.0f, 0.08f);
static Core::Control::PID::Param pitch_pid_param(0.015f, 0.00015f, 0.00012f, 1000.0f, 0.08f);

static Core::Control::PID roll_pid(roll_pid_param);
static Core::Control::PID pitch_pid(pitch_pid_param);

static inline float clampSym(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

// =====================================================================
// Bending Direction Convention
// =====================================================================
// bending_direction determines which angular solution Set_Leg_Height uses:
//   +1 → positive angle θ (0..+180°)
//   -1 → negative angle θ (0..-180°)
//
// Hardware-verified bending directions (May 2026 bringup, DEBUG mode):
//   With X button (standard stance, target=90°), all four legs need -1.
// "Inward" stance flips every leg.
static constexpr int BEND_STD_FL = -1, BEND_STD_FR = -1, BEND_STD_BL = -1, BEND_STD_BR = -1;
static constexpr int BEND_INV_FL = 1, BEND_INV_FR = 1, BEND_INV_BL = 1, BEND_INV_BR = 1;

// =====================================================================
// Global debug variables — watch these directly in Ozone
// =====================================================================
DbgIMU dbg_imu;
DbgLeveling dbg_leveling;
DbgClimbing dbg_climb;
DbgControl dbg_ctrl;
DbgGroundContact dbg_gc;
DbgImpedance dbg_imp;
DbgTorque dbg_torque;
DbgLeg dbg_leg;
DbgWheel dbg_wheel;
volatile uint8_t dbg_state = 0;

Chassis::Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br)
    : FL_WheelLegs_(fl),
      FR_WheelLegs_(fr),
      BL_WheelLegs_(bl),
      BR_WheelLegs_(br),
      R_m_(WHEEL_RADIUS_R / 1000.0f),
      r_m_(ECCENTRIC_OFFSET_r / 1000.0f),
      wb_m_(WHEEL_BASE / 1000.0f),
      wt_f_m_(WHEEL_TRACK_FRONT / 1000.0f)
{
    max_pitch_deg_  = rad2deg(atan2f(2.0f * r_m_, wb_m_));
    max_roll_deg_   = rad2deg(atan2f(2.0f * r_m_, wt_f_m_));
    float limit_cos = cosf(deg2rad(10.0f));
    h_max_          = R_m_ + r_m_ * limit_cos;
    h_min_          = R_m_ - r_m_ * limit_cos;
    // H = R + r * cos(theta). At 135 deg ≈ 0.114m.
    target_chassis_height_ = R_m_ + r_m_ * cosf(deg2rad(INITIAL_LEG_ANGLE));
}

void Chassis::Init()
{
    if (FL_WheelLegs_)
        FL_WheelLegs_->Init();
    if (FR_WheelLegs_)
        FR_WheelLegs_->Init();
    if (BL_WheelLegs_)
        BL_WheelLegs_->Init();
    if (BR_WheelLegs_)
        BR_WheelLegs_->Init();
}

void Chassis::Set_Mode(Chassis_State new_state)
{
    // --- Capture exit state when LEAVING COMFORT (for smooth Kp ramp-out) ---
    if (current_state_ == Chassis_State::COMFORT && new_state != Chassis_State::COMFORT)
    {
        for (int i = 0; i < 4; i++)
        {
            exit_kp_[i] = impedance_.getLegOutput(i).kp;
            exit_kd_[i] = impedance_.getLegOutput(i).kd;
        }
        mode_transition_timer_ = TRANSITION_FRAMES;
    }

    // --- Always ramp Kp from a safe low value when LEAVING IDLE ---
    // IDLE / disconnect drops the legs into low-Kp hold, which can leave them
    // displaced from 0°. Without a ramp, a sudden Kp=60 yank to target=0°
    // produces a violent slam (~90 Nm peak). Force every IDLE->non-IDLE
    // transition through the same ramp the COMFORT exit already uses.
    if (current_state_ == Chassis_State::IDLE && new_state != Chassis_State::IDLE)
    {
        for (int i = 0; i < 4; i++)
        {
            exit_kp_[i] = 0.0f;  // start ramp from gentle hold
            exit_kd_[i] = 0.5f;
        }
        mode_transition_timer_ = TRANSITION_FRAMES;
    }

    current_state_ = new_state;
    dbg_state      = static_cast<uint8_t>(new_state);
    // Reset vx slew-rate so a freshly-entered mode never starts mid-ramp.
    controller_.Reset_Velocity_Limiter();
    // Reset PIDs when entering modes that use body leveling
    if (new_state == Chassis_State::COMFORT || new_state == Chassis_State::CLIMBING)
    {
        roll_pid.reset();
        pitch_pid.reset();
        ground_contact_.reset();
    }
    // Reset mode-specific compensators
    if (new_state == Chassis_State::COMFORT)
    {
        impedance_.reset();  // Starts ramp from entry_kp → impedance values
    }
    if (new_state == Chassis_State::CLIMBING)
    {
        climbing_.reset();
        climbing_.startClimbAll();
        // Match body height to the PREP motor angle so BL/BR (IDLE) don't
        // create a pitch difference with FL/FR (PREP).
        // PREP motor angle = 180° - prep_theta_deg.  Height at that angle
        // via H = R + r·cos(angle) gives the matching height.
        // This also leaves a safe low height when returning to COMFORT,
        // avoiding the θ≈0° singularity (near full extension).
        target_chassis_height_ = CalculateHeightFromAngle(180.0f - climbing_.config().prep_theta_deg);
    }
}

void Chassis::Update(const Protocol::PC_Msg &cmd)
{
    // Always read IMU first so dbg_imu / chassis_roll_ / chassis_pitch_ /
    // chassis_accel_z_ stay live in IDLE, on disconnect, and during any mode.
    // Cheap (single sensor read + LPF) and required for safe-mode diagnostics.
    readAndTransformIMU();

    // Check connection first
    if (!Applications::Command_Task::Is_PC_Connected())
    {
        if (current_state_ != Chassis_State::IDLE)
        {
            Set_Mode(Chassis_State::IDLE);
        }

        // Force update of last button to prevent immediate jump on reconnect
        // Setting to 0xFF means all buttons are considered "previously pressed"
        // So a held button on reconnect (1) won't trigger a rising edge (1->1 no change, or 0->1 rising)
        // logic: rising = (current ^ last) & current
        // If held on reconnect: current=1, last=0xFF. changed=0xFE. rising=0. SAFE.
        last_button_status_ = 0xFF;

        // Stop motors immediately for safety
        Wheel_Leg_Params stop_params = {0};
        stop_params.state            = Chassis_State::IDLE;
        if (FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(stop_params);

        return;
    }

    // Extract buttons
    uint8_t current_buttons = cmd.button_status;
    uint8_t changing_edges  = current_buttons ^ last_button_status_;
    uint8_t rising_edges    = changing_edges & current_buttons;
    last_button_status_     = current_buttons;

    bool ml_pressed = (current_buttons & BTN_ML);
    bool mr_pressed = (current_buttons & BTN_MR);

    // --- TEMPORARY: only IDLE / ENERGY_SAVING / DEBUG enabled for live use ---
    // CALIBRATION combo (ML+MR) is disabled to prevent accidental DM flash erase.
    // COMFORT / CLIMBING / FREE_CONTROL are skipped while bringup is in progress.
    static const Chassis_State kAllowedStates[] = {
        Chassis_State::IDLE,
        Chassis_State::ENERGY_SAVING,
        Chassis_State::DEBUG,
    };
    constexpr int kNumAllowed = sizeof(kAllowedStates) / sizeof(kAllowedStates[0]);

    if (!(ml_pressed && mr_pressed))
    {
        // Find current index in allowed list (default to IDLE if not found)
        int idx = 0;
        for (int i = 0; i < kNumAllowed; i++)
        {
            if (current_state_ == kAllowedStates[i])
            {
                idx = i;
                break;
            }
        }

        if (rising_edges & BTN_ML)
        {
            idx = (idx - 1 + kNumAllowed) % kNumAllowed;
            Set_Mode(kAllowedStates[idx]);
        }
        else if (rising_edges & BTN_MR)
        {
            idx = (idx + 1) % kNumAllowed;
            Set_Mode(kAllowedStates[idx]);
        }
    }

    last_button_status_ = current_buttons;

    switch (current_state_)
    {
    case Chassis_State::CALIBRATION:
        handleCalibrationMode();
        break;
    case Chassis_State::IDLE:
    {
        Wheel_Leg_Params stop_params = {0};
        stop_params.state            = Chassis_State::IDLE;
        if (FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(stop_params);
        break;
    }
    case Chassis_State::ENERGY_SAVING:
        handleEnergySaving(cmd);
        break;
    case Chassis_State::COMFORT:
        handleComfortMode(cmd);
        break;
    case Chassis_State::CLIMBING:
        handleClimbingMode(cmd);
        break;
    case Chassis_State::FREE_CONTROL:
        handleFreeControl(cmd);
        break;
    case Chassis_State::DEBUG:
        handleDebugMode(cmd);
        break;
    default:
        break;
    }

    // --- Torque residual tracking (ALL modes) ---
    // LPF alpha for baseline (~1.6 Hz @ 500 Hz, same as Climbing_Dynamics)
    constexpr float torque_base_alpha = 0.02f;

    float t_fb[4]   = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegTorqueFeedback() : 0.0f,
                       FR_WheelLegs_ ? FR_WheelLegs_->Get_LegTorqueFeedback() : 0.0f,
                       BL_WheelLegs_ ? BL_WheelLegs_->Get_LegTorqueFeedback() : 0.0f,
                       BR_WheelLegs_ ? BR_WheelLegs_->Get_LegTorqueFeedback() : 0.0f};
    float g_comp[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegGravityTorque() : 0.0f,
                       FR_WheelLegs_ ? FR_WheelLegs_->Get_LegGravityTorque() : 0.0f,
                       BL_WheelLegs_ ? BL_WheelLegs_->Get_LegGravityTorque() : 0.0f,
                       BR_WheelLegs_ ? BR_WheelLegs_->Get_LegGravityTorque() : 0.0f};

    float *torque_arr[] = {&dbg_torque.torque_fl, &dbg_torque.torque_fr, &dbg_torque.torque_bl, &dbg_torque.torque_br};
    float *grav_arr[]   = {&dbg_torque.grav_fl, &dbg_torque.grav_fr, &dbg_torque.grav_bl, &dbg_torque.grav_br};
    float *res_arr[]    = {&dbg_torque.res_fl, &dbg_torque.res_fr, &dbg_torque.res_bl, &dbg_torque.res_br};
    float *base_arr[]   = {&dbg_torque.base_fl, &dbg_torque.base_fr, &dbg_torque.base_bl, &dbg_torque.base_br};
    float *dev_arr[]    = {&dbg_torque.dev_fl, &dbg_torque.dev_fr, &dbg_torque.dev_bl, &dbg_torque.dev_br};

    for (int i = 0; i < 4; i++)
    {
        *torque_arr[i] = t_fb[i];
        *grav_arr[i]   = g_comp[i];
        float res      = t_fb[i] - g_comp[i];
        *res_arr[i]    = res;
        *base_arr[i]   = torque_base_alpha * res + (1.0f - torque_base_alpha) * (*base_arr[i]);
        *dev_arr[i]    = fabsf(res - *base_arr[i]);
    }

    // --- Leg angle tracking (ALL modes) ---
    dbg_leg.fb_fl  = FL_WheelLegs_ ? FL_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_leg.fb_fr  = FR_WheelLegs_ ? FR_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_leg.fb_bl  = BL_WheelLegs_ ? BL_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_leg.fb_br  = BR_WheelLegs_ ? BR_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_leg.cmd_fl = FL_WheelLegs_ ? FL_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_fr = FR_WheelLegs_ ? FR_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_bl = BL_WheelLegs_ ? BL_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_br = BR_WheelLegs_ ? BR_WheelLegs_->Get_FinalLegCommand() : 0.0f;
}

void Chassis::handleCalibrationMode()
{
#if USE_HT_LEG_MOTOR
    // Re-enter motor mode (safe if already entered; recovers from lost ENTER_MOTOR)
    if (FL_WheelLegs_)
        FL_WheelLegs_->EnterMotorMode();
    if (FR_WheelLegs_)
        FR_WheelLegs_->EnterMotorMode();
    if (BL_WheelLegs_)
        BL_WheelLegs_->EnterMotorMode();
    if (BR_WheelLegs_)
        BR_WheelLegs_->EnterMotorMode();

    if (FL_WheelLegs_)
        FL_WheelLegs_->SetZero();
    if (FR_WheelLegs_)
        FR_WheelLegs_->SetZero();
    if (BL_WheelLegs_)
        BL_WheelLegs_->SetZero();
    if (BR_WheelLegs_)
        BR_WheelLegs_->SetZero();
#elif USE_DM_LEG_MOTOR
    // DM motors don't need ENTER_MOTOR ritual.
    //
    // SetZero() calls setZeroPosition(), which WRITES THE DM MOTOR'S FLASH and
    // permanently relocates the absolute encoder zero. Calling it on every
    // CALIBRATION entry is dangerous:
    //   - It silently invalidates DM_LEG_ID*_OFFSET in Robot_Params.hpp.
    //   - Repeated flash writes wear out the motor's flash.
    //   - Whatever pose the leg happens to be in becomes the new zero,
    //     usually NOT what you want.
    //
    // Calibrate manually via the DM Tool (or a one-shot debug build) instead.
    // Leave the calls disabled here so accidentally entering CALIBRATION never
    // corrupts the per-motor zero.
    //
    // if (FL_WheelLegs_) FL_WheelLegs_->SetZero();
    // if (FR_WheelLegs_) FR_WheelLegs_->SetZero();
    // if (BL_WheelLegs_) BL_WheelLegs_->SetZero();
    // if (BR_WheelLegs_) BR_WheelLegs_->SetZero();
#endif
    Set_Mode(Chassis_State::IDLE);
}

void Chassis::readAndTransformIMU()
{
    // 1. Read Euler Angles (IMU body frame)
    float euler[3];  // Z, Y, X (Yaw, Pitch, Roll)
    Core::Drivers::IMU::getEulerZYX(euler);

    float imu_pitch = rad2deg(euler[1]);
    float imu_roll  = rad2deg(euler[2]);

    // 2. Transform to Chassis Frame.
    // IMU is mounted with +90° yaw (CCW about Z, top-down view) relative to chassis.
    // For small attitudes:
    //     chassis_pitch (about chassis Y)  =  imu_roll       (about IMU X = chassis Y)
    //     chassis_roll  (about chassis X)  = -imu_pitch      (about IMU Y = chassis -X)
    //
    // Empirical observation on bench:
    //   • At level, imu_roll reads ≈ ±180° (i.e. the chip is also flipped 180° on its
    //     own X axis — "upside-down on the roll axis"), so we wrap that ±180° base to 0.
    //   • Tilting the chassis backward (nose-up) makes imu_roll go 175 → 174 (decrease),
    //     but the suspension PID expects "nose-up = +pitch". Therefore we also flip sign.
    //
    // Convention required by executeBodyControl():
    //   chassis_pitch_ > 0  ⇔  nose-up  ⇒ pitch_pid produces negative h_adj
    //                                    ⇒ front legs SHORTEN, back legs EXTEND
    //                                    ⇒ chassis levels back down
    float raw_pitch = imu_roll;
    if (raw_pitch > 90.0f)
        raw_pitch -= 180.0f;  // 175  → -5
    else if (raw_pitch < -90.0f)
        raw_pitch += 180.0f;  // -175 → 5
    raw_pitch = -raw_pitch;   // flip: nose-up (lean backward) → positive pitch

    // Roll convention required by executeBodyControl() leg-mixing matrix:
    //   chassis_roll_ > 0  ⇔  RIGHT side up
    //   (FL/BL get -roll_h_adj, FR/BR get +roll_h_adj; PID output is -Kp·roll
    //    so right-up → negative roll_h_adj → right legs shorten, left extend → level)
    // Hardware-verified: with raw = -imu_pitch, LEFT-up reads positive.
    // Flip again so RIGHT-up reads positive and matches the mixing matrix.
    float raw_roll = imu_pitch;

    // Normalize Roll to [-180, 180]
    if (raw_roll > 180.0f)
        raw_roll -= 360.0f;
    if (raw_roll < -180.0f)
        raw_roll += 360.0f;

    // Low-pass filter angles (cutoff ≈ 4 Hz @ 500 Hz)
    // Prevents motor vibration from feeding back through IMU into PID
    constexpr float imu_alpha = 0.02f;
    chassis_roll_             = imu_alpha * raw_roll + (1.0f - imu_alpha) * chassis_roll_;
    chassis_pitch_            = imu_alpha * raw_pitch + (1.0f - imu_alpha) * chassis_pitch_;

    // Update debug variables
    dbg_imu.pitch = chassis_pitch_;
    dbg_imu.roll  = chassis_roll_;

    // 3. Earth-frame linear acceleration (gravity-removed, AHRS-fused).
    // Earth frame is invariant under body-frame mounting yaw, so Z stays Z.
    float earth_accel[3];
    Core::Drivers::IMU::getEarthLinearAccel(earth_accel);
    chassis_accel_z_ = earth_accel[2];  // Z-up in earth frame

    // 4. Read & Transform Gyro (IMU body-frame rates -> chassis body-frame rates)
    // IMU yaw +90° CCW: gyro_x_imu = pitch rate (chassis), gyro_y_imu = roll rate (chassis)
    // Sign of roll rate must match sign of chassis_roll_ (right-up = positive),
    // otherwise rate damping in PID becomes positive feedback.
    float gyro[3];
    Core::Drivers::IMU::getCalibratedGyroTransformed(gyro);
    chassis_pitch_rate_ = gyro[0];
    chassis_roll_rate_  = gyro[1];

    dbg_imu.accel_z = chassis_accel_z_;
}

void Chassis::handleHeightButtons(const Protocol::PC_Msg &cmd)
{
    //==========================================================================================
    //============ Button Mappings for Target Heights + Bending Direction =======================
    //
    // Standard (BEND_STD): all legs same direction (outward)
    // Inverted (BEND_INV): front reversed, back normal (inward / 收缩)
    //==========================================================================================

    // X: 90 deg — standard stance
    if (cmd.button_status & BTN_X)
    {
        target_chassis_height_ = CalculateHeightFromAngle(90.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // Y: 145 deg — standard stance
    else if (cmd.button_status & BTN_Y)
    {
        target_chassis_height_ = CalculateHeightFromAngle(145.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // A: 45 deg — standard stance
    else if (cmd.button_status & BTN_A)
    {
        target_chassis_height_ = CalculateHeightFromAngle(45.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // B: 165 deg — standard stance
    else if (cmd.button_status & BTN_B)
    {
        target_chassis_height_ = CalculateHeightFromAngle(165.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // RB: 135 deg — inward stance (front reversed, back normal)
    else if (cmd.button_status & BTN_RB)
    {
        target_chassis_height_ = CalculateHeightFromAngle(135.0f);
        SetBendingDirection(BEND_INV_FL, BEND_INV_FR, BEND_INV_BL, BEND_INV_BR);
    }
    // LB: 90 deg — inward stance (front reversed, back normal)
    else if (cmd.button_status & BTN_LB)
    {
        target_chassis_height_ = CalculateHeightFromAngle(90.0f);
        SetBendingDirection(BEND_INV_FL, BEND_INV_FR, BEND_INV_BL, BEND_INV_BR);
    }
}

void Chassis::executeBodyControl(const Protocol::PC_Msg &cmd, const float mode_dh[4])
{
    // 1. Body Leveling PID
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(chassis_pitch_, max_pitch_deg_));

    // 2. Gyro Feedforward (disabled — set ff_gain > 0 to re-enable after tuning)
    static float filt_pitch_rate = 0.0f, filt_roll_rate = 0.0f;
    const float alpha = 0.05f, ff_gain = 0.0f;
    filt_pitch_rate  = alpha * chassis_pitch_rate_ + (1.0f - alpha) * filt_pitch_rate;
    filt_roll_rate   = alpha * chassis_roll_rate_ + (1.0f - alpha) * filt_roll_rate;
    float v_pitch_ff = (wb_m_ / 2.0f) * filt_pitch_rate * ff_gain;
    float v_roll_ff  = (wt_f_m_ / 2.0f) * filt_roll_rate * ff_gain;

    // 3. Per-leg height distribution: base + leveling + mode ΔH → clamp → send
    // FL (Front-Left): +Pitch, -Roll
    float h_fl = clampHeight(target_chassis_height_ + pitch_h_adj - roll_h_adj + mode_dh[0]);
    float v_fl = v_pitch_ff - v_roll_ff;
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl);

    // FR (Front-Right): +Pitch, +Roll
    float h_fr = clampHeight(target_chassis_height_ + pitch_h_adj + roll_h_adj + mode_dh[1]);
    float v_fr = v_pitch_ff + v_roll_ff;
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr);

    // BL (Back-Left): -Pitch, -Roll
    float h_bl = clampHeight(target_chassis_height_ - pitch_h_adj - roll_h_adj + mode_dh[2]);
    float v_bl = -v_pitch_ff - v_roll_ff;
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl);

    // BR (Back-Right): -Pitch, +Roll
    float h_br = clampHeight(target_chassis_height_ - pitch_h_adj + roll_h_adj + mode_dh[3]);
    float v_br = -v_pitch_ff + v_roll_ff;
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(h_br, v_br);

    dbg_leveling.h_fl = h_fl;
    dbg_leveling.h_fr = h_fr;
    dbg_leveling.h_bl = h_bl;
    dbg_leveling.h_br = h_br;

    // 4. Wheel Velocity Control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // 5. Execute
    executeMotorCommands();
}

// =====================================================================
// Impedance Body Control: PID leveling + per-leg Kp/Kd/FFW override
// =====================================================================
void Chassis::executeBodyControlImpedance(const Protocol::PC_Msg &cmd)
{
    // 1. Body Leveling PID (same as position-based path)
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(chassis_pitch_, max_pitch_deg_));

    // 2. Gyro Feedforward (disabled — set ff_gain > 0 to re-enable after tuning)
    static float filt_pitch_rate_i = 0.0f, filt_roll_rate_i = 0.0f;
    const float alpha = 0.05f, ff_gain = 0.0f;
    filt_pitch_rate_i = alpha * chassis_pitch_rate_ + (1.0f - alpha) * filt_pitch_rate_i;
    filt_roll_rate_i  = alpha * chassis_roll_rate_ + (1.0f - alpha) * filt_roll_rate_i;
    float v_pitch_ff  = (wb_m_ / 2.0f) * filt_pitch_rate_i * ff_gain;
    float v_roll_ff   = (wt_f_m_ / 2.0f) * filt_roll_rate_i * ff_gain;

    // 3. Per-leg height + impedance params → Set_Leg_Height(h, v, kp, kd, ffw)
    //    PID offsets for pitch/roll leveling are position commands; Kp/Kd/FFW handle compliance.
    auto &out_fl = impedance_.getLegOutput(0);
    auto &out_fr = impedance_.getLegOutput(1);
    auto &out_bl = impedance_.getLegOutput(2);
    auto &out_br = impedance_.getLegOutput(3);

    // FL (Front-Left): +Pitch, -Roll
    float h_fl = clampHeight(target_chassis_height_ + pitch_h_adj - roll_h_adj);
    float v_fl = v_pitch_ff - v_roll_ff;
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl, out_fl.kp, out_fl.kd, out_fl.ffw_torque);

    // FR (Front-Right): +Pitch, +Roll
    float h_fr = clampHeight(target_chassis_height_ + pitch_h_adj + roll_h_adj);
    float v_fr = v_pitch_ff + v_roll_ff;
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr, out_fr.kp, out_fr.kd, out_fr.ffw_torque);

    // BL (Back-Left): -Pitch, -Roll
    float h_bl = clampHeight(target_chassis_height_ - pitch_h_adj - roll_h_adj);
    float v_bl = -v_pitch_ff - v_roll_ff;
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl, out_bl.kp, out_bl.kd, out_bl.ffw_torque);

    // BR (Back-Right): -Pitch, +Roll
    float h_br = clampHeight(target_chassis_height_ - pitch_h_adj + roll_h_adj);
    float v_br = -v_pitch_ff + v_roll_ff;
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(h_br, v_br, out_br.kp, out_br.kd, out_br.ffw_torque);

    dbg_leveling.h_fl = h_fl;
    dbg_leveling.h_fr = h_fr;
    dbg_leveling.h_bl = h_bl;
    dbg_leveling.h_br = h_br;

    // 4. Wheel Velocity Control (same as position-based path)
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // 5. Execute
    executeMotorCommands();
}

// =====================================================================
// COMFORT MODE: Variable Impedance Control (Kp/Kd/FFW modulation)
// =====================================================================
void Chassis::handleComfortMode(const Protocol::PC_Msg &cmd)
{
    // IMU already updated at top of Update()

    // 2. Button → Target Height
    handleHeightButtons(cmd);

    // 3. Gather per-leg data
    float leg_currents[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             FR_WheelLegs_ ? FR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             BL_WheelLegs_ ? BL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             BR_WheelLegs_ ? BR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f};

    float leg_angles[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegPosition() : 90.0f,
                           FR_WheelLegs_ ? FR_WheelLegs_->Get_LegPosition() : 90.0f,
                           BL_WheelLegs_ ? BL_WheelLegs_->Get_LegPosition() : 90.0f,
                           BR_WheelLegs_ ? BR_WheelLegs_->Get_LegPosition() : 90.0f};

    const float dt = 0.002f;  // 500 Hz

    // 4. Update Impedance Controller → per-leg Kp, Kd, FFW
    impedance_.update(leg_currents, leg_angles, chassis_accel_z_, chassis_roll_rate_, chassis_pitch_rate_, dt);

    // 5. Update debug variables
    dbg_imp.kp_fl      = impedance_.getLegOutput(0).kp;
    dbg_imp.kp_fr      = impedance_.getLegOutput(1).kp;
    dbg_imp.kp_bl      = impedance_.getLegOutput(2).kp;
    dbg_imp.kp_br      = impedance_.getLegOutput(3).kp;
    dbg_imp.kd_fl      = impedance_.getLegOutput(0).kd;
    dbg_imp.kd_fr      = impedance_.getLegOutput(1).kd;
    dbg_imp.kd_bl      = impedance_.getLegOutput(2).kd;
    dbg_imp.kd_br      = impedance_.getLegOutput(3).kd;
    dbg_imp.ffw_fl     = impedance_.getLegOutput(0).ffw_torque;
    dbg_imp.ffw_fr     = impedance_.getLegOutput(1).ffw_torque;
    dbg_imp.ffw_bl     = impedance_.getLegOutput(2).ffw_torque;
    dbg_imp.ffw_br     = impedance_.getLegOutput(3).ffw_torque;
    dbg_imp.mass_est   = impedance_.getEstimatedMass();
    dbg_imp.warp_error = impedance_.getWarpError();
    dbg_imp.vz         = impedance_.getBodyVelZ();

    // 6. Execute body control with impedance parameters
    executeBodyControlImpedance(cmd);
}

void Chassis::handleFreeControl(const Protocol::PC_Msg &cmd)
{
    static float folded_angle = 180.0f;

    // Triggers Control Angle Interpolation
    // Left Trigger: FL & FR
    // Right Trigger: BL & BR
    // 0 (Released) -> folded_angle
    // 1000 (Pressed) -> 0 deg (Extended)
    float l_ratio = (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    float r_ratio = (float)cmd.Right_trigger_x1000_msg / 1000.0f;

    float fl_fr_angle = -folded_angle * (1.0f - l_ratio);
    float bl_br_angle = folded_angle * (1.0f - r_ratio);

    Wheel_Leg_Params params;
    params.state     = Chassis_State::FREE_CONTROL;
    params.Leg_Force = 0;
    params.Leg_RPM   = 0;
    // Use default stiff parameters for position control
    params.Leg_Kp = 50.0f;
    params.Leg_Kd = 1.0f;

    // Wheel control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        params.Leg_POS   = fl_fr_angle;
        params.Wheel_RPM = wheel_rpms[0];
        FL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (FR_WheelLegs_)
    {
        params.Leg_POS   = fl_fr_angle;
        params.Wheel_RPM = wheel_rpms[1];
        FR_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BL_WheelLegs_)
    {
        params.Leg_POS   = bl_br_angle;
        params.Wheel_RPM = wheel_rpms[2];
        BL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BR_WheelLegs_)
    {
        params.Leg_POS   = bl_br_angle;
        params.Wheel_RPM = wheel_rpms[3];
        BR_WheelLegs_->Set_Wheel_Leg(params);
    }

    // Wheel motor health (FREE_CONTROL uses Set_Wheel_Leg, not executeMotorCommands)
    updateWheelDebug();
}

// =====================================================================
// DEBUG MODE: Pure height + wheel control, NO pitch/roll leveling
// Same button decoding as COMFORT (handleHeightButtons), but no PID,
// no gyro feedforward, no active suspension.
// =====================================================================
void Chassis::handleDebugMode(const Protocol::PC_Msg &cmd)
{
    // ------------------------------------------------------------------
    // DEBUG: direct per-leg position targets (radians, raw feedback frame).
    // Bypasses Set_Leg_Height / inverse kinematics — values are sent straight
    // to DM via setMIT (after deg-conversion in Execute_Leg_Control).
    //
    // Reference values come from physical measurement (May 2026 bringup);
    // not all legs land at the same numeric angle because of mechanical
    // mounting / encoder offset differences.
    //
    // Order: {FL, BL, FR, BR}  (matches Wheel_Leg_Params layout)
    //
    //   X — wheels splayed outward, mid-stance
    //   A — wheels splayed outward, ~45° squat (lower)
    //   (others fall back to "hold last target")
    // ------------------------------------------------------------------

    static float last_target_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target_rad[4]             = {last_target_rad[0], last_target_rad[1], last_target_rad[2], last_target_rad[3]};

    if (cmd.button_status & BTN_X)
    {
        target_rad[0] = -1.3f;  // FL
        target_rad[1] = +1.3f;  // BL
        target_rad[2] = +1.3f;  // FR
        target_rad[3] = -1.3f;  // BR
    }
    else if (cmd.button_status & BTN_A)
    {
        target_rad[0] = -2.0f;  // FL
        target_rad[1] = +2.0f;  // BL
        target_rad[2] = +2.0f;  // FR
        target_rad[3] = -2.0f;  // BR
    }
    // Y / B currently unused — held at last_target_rad

    for (int i = 0; i < 4; i++)
        last_target_rad[i] = target_rad[i];

    // DM holding gains (same as ENERGY_SAVING)
    const float kp = 60.0f;
    const float kd = 2.5f;

    Wheel_Leg *legs[4] = {FL_WheelLegs_, BL_WheelLegs_, FR_WheelLegs_, BR_WheelLegs_};
    for (int i = 0; i < 4; i++)
    {
        if (legs[i])
            legs[i]->Set_Leg_Target(rad2deg(target_rad[i]), 0.0f, 0.0f, kp, kd);
    }

    dbg_leveling.h_fl = target_rad[0];
    dbg_leveling.h_fr = target_rad[2];
    dbg_leveling.h_bl = target_rad[1];
    dbg_leveling.h_br = target_rad[3];

    // Wheel velocity control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // Execute
    executeMotorCommands();
}

void Chassis::handleEnergySaving(const Protocol::PC_Msg &cmd)
{
    // --- Smooth Kp ramp when transitioning from COMFORT ---
    // Holding torque must resist gravity at the lowest pose AND the disturbance
    // from wheel acceleration / forward-backward jerk. Too soft → leg sways,
    // sway gets amplified by joystick input and the leg goes unstable.
    float target_kp = 60.0f;
    float target_kd = 2.5f;
    float kp_use, kd_use;
    if (mode_transition_timer_ > 0)
    {
        float alpha = 1.0f - (float)mode_transition_timer_ / (float)TRANSITION_FRAMES;
        // Use per-leg average of exit values for simplicity
        float avg_exit_kp = (exit_kp_[0] + exit_kp_[1] + exit_kp_[2] + exit_kp_[3]) * 0.25f;
        float avg_exit_kd = (exit_kd_[0] + exit_kd_[1] + exit_kd_[2] + exit_kd_[3]) * 0.25f;
        kp_use            = avg_exit_kp + alpha * (target_kp - avg_exit_kp);
        kd_use            = avg_exit_kd + alpha * (target_kd - avg_exit_kd);
        mode_transition_timer_--;
    }
    else
    {
        kp_use = target_kp;
        kd_use = target_kd;
    }

    Wheel_Leg_Params params;
    params.state     = Chassis_State::ENERGY_SAVING;
    params.Leg_POS   = 0.0f;
    params.Leg_Force = 0;
    params.Leg_RPM   = 0;
    params.Leg_Kp    = kp_use;
    params.Leg_Kd    = kd_use;

    // Wheel control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[0];
        FL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (FR_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[1];
        FR_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BL_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[2];
        BL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BR_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[3];
        BR_WheelLegs_->Set_Wheel_Leg(params);
    }

    // Wheel motor health (ENERGY_SAVING uses Set_Wheel_Leg, not executeMotorCommands)
    updateWheelDebug();
}

// =====================================================================
// CLIMBING MODE: Per-leg kinematic step climbing + body leveling
// =====================================================================
void Chassis::handleClimbingMode(const Protocol::PC_Msg &cmd)
{
    // IMU already updated at top of Update()

    // --- Trigger-based DIRECT ANGLE debug (verify climbing direction) ---
    // L trigger → FL/FR sweep from -(180-deadzone) toward 0° (climbing direction)
    // R trigger → BL/BR sweep from +(180-deadzone) toward 0° (climbing direction)
    // Released = PREP position (±165°),  Fully pressed = 0° (extended)
    float l_ratio_c    = (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    float r_ratio_c    = (float)cmd.Right_trigger_x1000_msg / 1000.0f;
    bool manual_climb  = (l_ratio_c > 0.05f || r_ratio_c > 0.05f);
    float deadzone_deg = climbing_.config().prep_theta_deg;
    float prep_angle   = 180.0f - deadzone_deg;  // e.g. 165°
    // Sweep: ratio=0 → ±prep_angle,  ratio=1 → 0°
    float front_angle = -prep_angle * (1.0f - l_ratio_c);  // FL/FR: negative, toward 0
    float back_angle  = +prep_angle * (1.0f - r_ratio_c);  // BL/BR: positive, toward 0

    // --- Smooth Kp/Kd ramp when transitioning from COMFORT ---
    float climb_target_kp = 12.0f;
    float climb_target_kd = 2.5f;
    float kp_use, kd_use;
    if (mode_transition_timer_ > 0)
    {
        float alpha       = 1.0f - (float)mode_transition_timer_ / (float)TRANSITION_FRAMES;
        float avg_exit_kp = (exit_kp_[0] + exit_kp_[1] + exit_kp_[2] + exit_kp_[3]) * 0.25f;
        float avg_exit_kd = (exit_kd_[0] + exit_kd_[1] + exit_kd_[2] + exit_kd_[3]) * 0.25f;
        kp_use            = avg_exit_kp + alpha * (climb_target_kp - avg_exit_kp);
        kd_use            = avg_exit_kd + alpha * (climb_target_kd - avg_exit_kd);
        mode_transition_timer_--;
    }
    else
    {
        kp_use = climb_target_kp;
        kd_use = climb_target_kd;
    }

    // Gather per-leg feedback
    LegClimbFeedback fb[4] = {};
    // Compute torque residual per leg: actual_torque - gravity_comp
    float tres[4] = {FL_WheelLegs_ ? (FL_WheelLegs_->Get_LegTorqueFeedback() - FL_WheelLegs_->Get_LegGravityTorque()) : 0.0f,
                     FR_WheelLegs_ ? (FR_WheelLegs_->Get_LegTorqueFeedback() - FR_WheelLegs_->Get_LegGravityTorque()) : 0.0f,
                     BL_WheelLegs_ ? (BL_WheelLegs_->Get_LegTorqueFeedback() - BL_WheelLegs_->Get_LegGravityTorque()) : 0.0f,
                     BR_WheelLegs_ ? (BR_WheelLegs_->Get_LegTorqueFeedback() - BR_WheelLegs_->Get_LegGravityTorque()) : 0.0f};
    if (FL_WheelLegs_)
        fb[0] = {FL_WheelLegs_->Get_LegPosition(), FL_WheelLegs_->Get_WheelRPM(), tres[0]};
    if (FR_WheelLegs_)
        fb[1] = {FR_WheelLegs_->Get_LegPosition(), FR_WheelLegs_->Get_WheelRPM(), tres[1]};
    if (BL_WheelLegs_)
        fb[2] = {BL_WheelLegs_->Get_LegPosition(), BL_WheelLegs_->Get_WheelRPM(), tres[2]};
    if (BR_WheelLegs_)
        fb[3] = {BR_WheelLegs_->Get_LegPosition(), BR_WheelLegs_->Get_WheelRPM(), tres[3]};

    climbing_.config().step_height_m        = dbg_ctrl.step_height_mm / 1000.0f;
    climbing_.config().torque_res_threshold = dbg_ctrl.torque_res_threshold;
    climbing_.config().climb_omega          = dbg_ctrl.climb_omega;
    const float dt                          = 0.002f;

    climbing_.update(fb, dt);

    // Ground Contact Warp Compensation (reuse leg currents from feedback)
    float leg_currents_c[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               FR_WheelLegs_ ? FR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               BL_WheelLegs_ ? BL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               BR_WheelLegs_ ? BR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f};
    ground_contact_.update(leg_currents_c, dt);

    dbg_gc.warp_error = ground_contact_.getWarpError();
    dbg_gc.warp_dh    = ground_contact_.getWarpDH();
    dbg_gc.dh_fl      = ground_contact_.getDeltaH(0);
    dbg_gc.dh_fr      = ground_contact_.getDeltaH(1);
    dbg_gc.dh_bl      = ground_contact_.getDeltaH(2);
    dbg_gc.dh_br      = ground_contact_.getDeltaH(3);

    // Debug variables
    dbg_climb.dh_fl    = climbing_.getTargetHeight(0);
    dbg_climb.dh_fr    = climbing_.getTargetHeight(1);
    dbg_climb.dh_bl    = climbing_.getTargetHeight(2);
    dbg_climb.dh_br    = climbing_.getTargetHeight(3);
    dbg_climb.phase_fl = static_cast<uint8_t>(climbing_.getPhase(0));
    dbg_climb.phase_fr = static_cast<uint8_t>(climbing_.getPhase(1));
    dbg_climb.phase_bl = static_cast<uint8_t>(climbing_.getPhase(2));
    dbg_climb.phase_br = static_cast<uint8_t>(climbing_.getPhase(3));
    // Torque residual step detection debug
    dbg_climb.tbase_fl = climbing_.getBaseline(0);
    dbg_climb.tbase_fr = climbing_.getBaseline(1);
    dbg_climb.tbase_bl = climbing_.getBaseline(2);
    dbg_climb.tbase_br = climbing_.getBaseline(3);
    dbg_climb.raw_fl   = tres[0];
    dbg_climb.raw_fr   = tres[1];
    dbg_climb.raw_bl   = tres[2];
    dbg_climb.raw_br   = tres[3];
    dbg_climb.tres_fl  = fabsf(tres[0] - dbg_climb.tbase_fl);
    dbg_climb.tres_fr  = fabsf(tres[1] - dbg_climb.tbase_fr);
    dbg_climb.tres_bl  = fabsf(tres[2] - dbg_climb.tbase_bl);
    dbg_climb.tres_br  = fabsf(tres[3] - dbg_climb.tbase_br);
    // Target height sent to FL (m) — check in Ozone to diagnose height issues
    dbg_climb.target_h = climbing_.isDirectControl(0) ? climbing_.getTargetHeight(0) : target_chassis_height_;

    // Pitch lean bias during climbing — gravitational "push" toward climbing wheels
    // Active during PREP/DETECT/CLIMBING (not just CLIMBING) so weight shifts early.
    float pitch_setpoint = 0.0f;
    auto phFL = climbing_.getPhase(0), phFR = climbing_.getPhase(1);
    auto phBL = climbing_.getPhase(2), phBR = climbing_.getPhase(3);
    bool front_active = (phFL == LegClimbPhase::PREP || phFL == LegClimbPhase::DETECT || phFL == LegClimbPhase::CLIMBING ||
                         phFR == LegClimbPhase::PREP || phFR == LegClimbPhase::DETECT || phFR == LegClimbPhase::CLIMBING);
    bool back_active  = (phBL == LegClimbPhase::PREP || phBL == LegClimbPhase::DETECT || phBL == LegClimbPhase::CLIMBING ||
                         phBR == LegClimbPhase::PREP || phBR == LegClimbPhase::DETECT || phBR == LegClimbPhase::CLIMBING);
    // Keep separate flags for pitch suppression (only suppress during actual CLIMBING)
    bool front_climbing = (phFL == LegClimbPhase::CLIMBING || phFR == LegClimbPhase::CLIMBING);
    bool back_climbing  = (phBL == LegClimbPhase::CLIMBING || phBR == LegClimbPhase::CLIMBING);
    if (front_active)
        pitch_setpoint = -dbg_ctrl.climb_pitch_bias;  // lean forward to load front wheels
    else if (back_active)
        pitch_setpoint = dbg_ctrl.climb_pitch_bias;  // lean backward to load back wheels

    // Body Leveling PID
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(pitch_setpoint, clampSym(chassis_pitch_, max_pitch_deg_));

    // Gyro Feedforward (disabled — set ff_gain_c > 0 to re-enable after tuning)
    static float filt_pitch_rate_c = 0.0f, filt_roll_rate_c = 0.0f;
    const float alpha_c = 0.05f, ff_gain_c = 0.0f;
    filt_pitch_rate_c = alpha_c * chassis_pitch_rate_ + (1.0f - alpha_c) * filt_pitch_rate_c;
    filt_roll_rate_c  = alpha_c * chassis_roll_rate_ + (1.0f - alpha_c) * filt_roll_rate_c;
    float v_pitch_ff  = (wb_m_ / 2.0f) * filt_pitch_rate_c * ff_gain_c;
    float v_roll_ff   = (wt_f_m_ / 2.0f) * filt_roll_rate_c * ff_gain_c;

    // Per-Leg Height Distribution (climbing-aware + warp compensation)
    // Near full extension (θ≈15°), dh/dθ is very small, so height corrections
    // get amplified into large angle changes. Scale down PID leveling to prevent oscillation.
    constexpr float lev_scale = 0.25f;

    // Per-leg height: trigger angle debug OR climbing state machine
    float h_targets[4], v_targets[4];
    float lev_signs[4][2]   = {{1, -1}, {1, 1}, {-1, -1}, {-1, 1}};  // {pitch_sign, roll_sign}
    float gv_signs[4][2]    = {{1, -1}, {1, 1}, {-1, -1}, {-1, 1}};  // same for gyro FF
    float trigger_angles[4] = {front_angle, front_angle, back_angle, back_angle};

    // Climbing angle sign: FL/FR negative, BL/BR positive.
    // BL/BR use a mirrored trajectory so angle increases during climbing (see below).
    float climb_sign[4] = {-1.0f, -1.0f, 1.0f, 1.0f};

    for (int i = 0; i < 4; i++)
    {
        // During climbing: keep pitch leveling on support legs (for pitch bias lean)
        // but disable warp compensation (designed for flat ground, misleading on step).
        float pitch_contrib = lev_signs[i][0] * pitch_h_adj;
        float warp_dh       = ground_contact_.getDeltaH(i);
        if (front_climbing && i >= 2)
            warp_dh = 0.0f;
        if (back_climbing && i < 2)
            warp_dh = 0.0f;

        float lev = lev_scale * (pitch_contrib + lev_signs[i][1] * roll_h_adj) + warp_dh;
        float gv  = gv_signs[i][0] * v_pitch_ff + gv_signs[i][1] * v_roll_ff;

        if (manual_climb)
        {
            // Trigger debug: direct angle control, bypass state machine
            // h_targets/v_targets unused — angle sent directly below
            h_targets[i] = 0.0f;
            v_targets[i] = 0.0f;
        }
        else if (climbing_.isDirectControl(i))
        {
            // Climbing direct control: use kinematic height directly, NO body leveling.
            // The 180°-acos() angle mapping creates a non-monotonic relationship
            // between height corrections and unwrapped motor angle, causing positive
            // feedback with the roll/pitch PID for legs with negative climb_sign.
            h_targets[i] = climbing_.getTargetHeight(i);
            v_targets[i] = climbing_.getTargetVelocity(i);
        }
        else
        {
            // Non-climbing legs: normal height pipeline with leveling
            h_targets[i] = target_chassis_height_ + lev;
            v_targets[i] = gv;
        }
    }

    // Constants for height→angle conversion (same as Set_Leg_Height)
    const float R_m = WHEEL_RADIUS_R / 1000.0f;
    const float r_m = ECCENTRIC_OFFSET_r / 1000.0f;

    Wheel_Leg *legs[4]     = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    float dbg_angle_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;

        if (manual_climb)
        {
            // --- Trigger debug: direct angle command, verify climbing direction ---
            float ffw = legs[i]->Get_LegGravityTorque();
            legs[i]->Set_Leg_Target(trigger_angles[i], 0.0f, ffw, kp_use, kd_use);
            dbg_angle_cmd[i] = trigger_angles[i];
        }
        else if (climbing_.isDirectControl(i))
        {
            // --- Direct angle control for climbing phases ---
            // Climbing_Dynamics outputs unsigned motor angle (180°=highest, 0°=lowest)
            // and angular velocity. theta_unsigned DECREASES during climbing (165→57.4).
            //
            // FL/FR (i<2): angle = -theta_unsigned → -165 → -57.4 (toward 0) ✓
            // BL/BR (i≥2): angle = +theta_unsigned → +165 → +57.4 (toward 0) ✓
            //   Both front and back are mirrors, converging toward 0°.
            float theta_unsigned = climbing_.getTargetThetaDeg(i);
            float omega_unsigned = climbing_.getTargetOmega(i);  // negative (theta decreasing)

            // climb_sign: FL/FR = -1, BL/BR = +1
            float angle_cmd = climb_sign[i] * theta_unsigned;
            float vel_cmd   = climb_sign[i] * omega_unsigned;

            // Pitch leveling for direct-control legs (PREP/DETECT/CLIMBING/COMPLETE).
            // The height pipeline can't reach them, so convert PID output (meters)
            // → angle offset (degrees) using H = R + r·cos(θ), dθ = -dH/(r·sin(θ)).
            {
                float dh      = lev_signs[i][0] * pitch_h_adj;  // full PID output
                float thu_rad = theta_unsigned * 3.14159265f / 180.0f;
                float sin_thu = sinf(thu_rad);
                if (fabsf(sin_thu) < 0.15f)
                    sin_thu = copysignf(0.15f, sin_thu);
                float dtheta_deg = -dh / (r_m * sin_thu) * (180.0f / 3.14159265f);
                // Clamp offset and final angle to safe range
                if (dtheta_deg > 50.0f)
                    dtheta_deg = 50.0f;
                if (dtheta_deg < -50.0f)
                    dtheta_deg = -50.0f;
                float new_theta = theta_unsigned + dtheta_deg;
                if (new_theta > 180.0f)
                    new_theta = 180.0f;
                if (new_theta < 0.0f)
                    new_theta = 0.0f;
                angle_cmd = climb_sign[i] * new_theta;
            }

            float ffw = legs[i]->Get_LegGravityTorque();
            legs[i]->Set_Leg_Target(angle_cmd, vel_cmd, ffw, kp_use, kd_use);
            dbg_angle_cmd[i] = angle_cmd;
        }
        else
        {
            // Normal height pipeline (COMFORT-like, or manual trigger)
            legs[i]->Set_Leg_Height(clampHeight(h_targets[i]), v_targets[i], kp_use, kd_use, legs[i]->Get_LegGravityTorque());
            // Show actual motor feedback for non-climbing legs (so BL/BR don't show 0)
            dbg_angle_cmd[i] = legs[i]->Get_LegPosition();
        }
    }

    dbg_leveling.h_fl = manual_climb ? 0.0f : (climbing_.isDirectControl(0) ? climbing_.getTargetHeight(0) : target_chassis_height_);
    dbg_leveling.h_fr = manual_climb ? 0.0f : (climbing_.isDirectControl(1) ? climbing_.getTargetHeight(1) : target_chassis_height_);
    dbg_leveling.h_bl = manual_climb ? 0.0f : (climbing_.isDirectControl(2) ? climbing_.getTargetHeight(2) : target_chassis_height_);
    dbg_leveling.h_br = manual_climb ? 0.0f : (climbing_.isDirectControl(3) ? climbing_.getTargetHeight(3) : target_chassis_height_);

    // Climbing kinematic debug: β₀, per-leg β, and commanded motor angle
    dbg_climb.beta0        = climbing_.getBeta0() * 180.0f / 3.14159265f;
    dbg_climb.beta_fl      = climbing_.getBeta(0) * 180.0f / 3.14159265f;
    dbg_climb.beta_fr      = climbing_.getBeta(1) * 180.0f / 3.14159265f;
    dbg_climb.beta_bl      = climbing_.getBeta(2) * 180.0f / 3.14159265f;
    dbg_climb.beta_br      = climbing_.getBeta(3) * 180.0f / 3.14159265f;
    dbg_climb.theta_fl     = dbg_angle_cmd[0];
    dbg_climb.theta_fr     = dbg_angle_cmd[1];
    dbg_climb.theta_bl     = dbg_angle_cmd[2];
    dbg_climb.theta_br     = dbg_angle_cmd[3];
    dbg_climb.raw_theta_fl = climbing_.getTargetThetaDeg(0) * climb_sign[0];
    dbg_climb.raw_theta_fr = climbing_.getTargetThetaDeg(1) * climb_sign[1];
    dbg_climb.raw_theta_bl = climbing_.getTargetThetaDeg(2) * climb_sign[2];
    dbg_climb.raw_theta_br = climbing_.getTargetThetaDeg(3) * climb_sign[3];

    // Wheel velocity + execute
    // Add forward RPM throughout entire climbing sequence (including all COMPLETE).
    // Only stops when user switches out of CLIMBING mode.
    float climb_base_rpm = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        LegClimbPhase ph = climbing_.getPhase(i);
        if (ph == LegClimbPhase::CLIMBING || ph == LegClimbPhase::COMPLETE)
        {
            climb_base_rpm = climbing_.config().climb_omega * (60.0f / (2.0f * 3.14159265f)) * dbg_ctrl.climb_wheel_scale;
            break;
        }
    }
    dbg_climb.wheel_rpm = climb_base_rpm;

    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    // Only add climb boost when user is pushing forward (vx > 0)
    if (vx > 0.5f)
        vx += climb_base_rpm;
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    executeMotorCommands();
}

void Chassis::executeMotorCommands()
{
    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Execute_Wheel_Control();
        FL_WheelLegs_->Execute_Leg_Control();
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Execute_Wheel_Control();
        FR_WheelLegs_->Execute_Leg_Control();
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Execute_Wheel_Control();
        BL_WheelLegs_->Execute_Leg_Control();
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Execute_Wheel_Control();
        BR_WheelLegs_->Execute_Leg_Control();
    }

    updateWheelDebug();
}

void Chassis::updateWheelDebug()
{
    if (FL_WheelLegs_)
    {
        dbg_wheel.out_fl  = FL_WheelLegs_->Get_WheelOutput();
        dbg_wheel.cur_fl  = FL_WheelLegs_->Get_WheelCurrentFeedback();
        dbg_wheel.rpm_fl  = FL_WheelLegs_->Get_WheelRPM();
        dbg_wheel.temp_fl = FL_WheelLegs_->Get_WheelTemperature();
        dbg_wheel.tgt_fl  = FL_WheelLegs_->Get_FinalWheelRPM();
        dbg_wheel.util_fl = fabsf(dbg_wheel.out_fl) / 16000.0f;
        dbg_wheel.err_fl  = dbg_wheel.tgt_fl - dbg_wheel.rpm_fl;
    }
    if (FR_WheelLegs_)
    {
        dbg_wheel.out_fr  = FR_WheelLegs_->Get_WheelOutput();
        dbg_wheel.cur_fr  = FR_WheelLegs_->Get_WheelCurrentFeedback();
        dbg_wheel.rpm_fr  = FR_WheelLegs_->Get_WheelRPM();
        dbg_wheel.temp_fr = FR_WheelLegs_->Get_WheelTemperature();
        dbg_wheel.tgt_fr  = FR_WheelLegs_->Get_FinalWheelRPM();
        dbg_wheel.util_fr = fabsf(dbg_wheel.out_fr) / 16000.0f;
        dbg_wheel.err_fr  = dbg_wheel.tgt_fr - dbg_wheel.rpm_fr;
    }
    if (BL_WheelLegs_)
    {
        dbg_wheel.out_bl  = BL_WheelLegs_->Get_WheelOutput();
        dbg_wheel.cur_bl  = BL_WheelLegs_->Get_WheelCurrentFeedback();
        dbg_wheel.rpm_bl  = BL_WheelLegs_->Get_WheelRPM();
        dbg_wheel.temp_bl = BL_WheelLegs_->Get_WheelTemperature();
        dbg_wheel.tgt_bl  = BL_WheelLegs_->Get_FinalWheelRPM();
        dbg_wheel.util_bl = fabsf(dbg_wheel.out_bl) / 16000.0f;
        dbg_wheel.err_bl  = dbg_wheel.tgt_bl - dbg_wheel.rpm_bl;
    }
    if (BR_WheelLegs_)
    {
        dbg_wheel.out_br  = BR_WheelLegs_->Get_WheelOutput();
        dbg_wheel.cur_br  = BR_WheelLegs_->Get_WheelCurrentFeedback();
        dbg_wheel.rpm_br  = BR_WheelLegs_->Get_WheelRPM();
        dbg_wheel.temp_br = BR_WheelLegs_->Get_WheelTemperature();
        dbg_wheel.tgt_br  = BR_WheelLegs_->Get_FinalWheelRPM();
        dbg_wheel.util_br = fabsf(dbg_wheel.out_br) / 16000.0f;
        dbg_wheel.err_br  = dbg_wheel.tgt_br - dbg_wheel.rpm_br;
    }
}

void Chassis::inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms)
{
    // =====================================================================
    // Skid-Steer Inverse Kinematics (asymmetric track: W_F ≠ W_B)
    // =====================================================================
    //
    // Model: Vehicle rotates around ICR at (0, R_turn) with angular vel ω.
    //   V_{x,i} = ω · (R_turn − y_i)
    //   V_center = ω · R_turn = Vx
    //
    // Each wheel:
    //   V_FL = Vx − ω · W_F/2     V_FR = Vx + ω · W_F/2
    //   V_BL = Vx − ω · W_B/2     V_BR = Vx + ω · W_B/2
    //
    // The controller outputs wz as differential RPM scaled to a reference.
    // We normalize by average track width so front/back get correct ratio.
    // =====================================================================

    (void)vy;

    // Track widths in mm (from Robot_Params.hpp)
    constexpr float W_F   = WHEEL_TRACK_FRONT;   // 531 mm
    constexpr float W_B   = WHEEL_TRACK_BACK;    // 395 mm
    constexpr float W_avg = (W_F + W_B) / 2.0f;  // Reference: average track

    // Differential RPM gain for each axle (wz is scaled to W_avg)
    constexpr float k_F = W_F / W_avg;  // > 1.0 (front wider → more differential)
    constexpr float k_B = W_B / W_avg;  // < 1.0 (back narrower → less differential)

    out_wheel_rpms[0] = vx - wz * k_F;  // FL
    out_wheel_rpms[1] = vx + wz * k_F;  // FR
    out_wheel_rpms[2] = vx - wz * k_B;  // BL
    out_wheel_rpms[3] = vx + wz * k_B;  // BR

    // Normalize: if any wheel exceeds MAX_WHEEL_RPM, scale all proportionally
    float max_rpm = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float abs_rpm = fabsf(out_wheel_rpms[i]);
        if (abs_rpm > max_rpm)
            max_rpm = abs_rpm;
    }
    if (max_rpm > MAX_WHEEL_RPM)
    {
        float scale = MAX_WHEEL_RPM / max_rpm;
        for (int i = 0; i < 4; i++)
            out_wheel_rpms[i] *= scale;
    }
}

float Chassis::CalculateHeightFromAngle(float angle_deg) { return R_m_ + r_m_ * cosf(deg2rad(angle_deg)); }

void Chassis::SetBendingDirection(int fl, int fr, int bl, int br)
{
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Bending_Direction(fl);
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Bending_Direction(fr);
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Bending_Direction(bl);
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Bending_Direction(br);
}

void Chassis::Get_Msg(Protocol::Reachable_Msg *msg)
{
    if (!msg)
        return;

    Wheel_Leg_Params info;
    if (FL_WheelLegs_)
    {
        info                               = FL_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[0] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[0]      = (int16_t)info.Wheel_RPM;
    }
    if (FR_WheelLegs_)
    {
        info                               = FR_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[1] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[1]      = (int16_t)info.Wheel_RPM;
    }
    if (BL_WheelLegs_)
    {
        info                               = BL_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[2] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[2]      = (int16_t)info.Wheel_RPM;
    }
    if (BR_WheelLegs_)
    {
        info                               = BR_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[3] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[3]      = (int16_t)info.Wheel_RPM;
    }
}

}  // namespace Applications
