#include "Chassis.hpp"

#include "Matrixf.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "Quaternion.hpp"

namespace Applications
{
using namespace Core::Drivers;

// PID Parameters for Active Suspension (Comfort Mode)
// 增益定义全部在 Robot_Params.hpp 的“COMFORT 调平”小节，这里只组装。
static Core::Control::PID::Param roll_pid_param(
    BODY_ROLL_PID_KP, BODY_ROLL_PID_KI, BODY_ROLL_PID_KD, BODY_ROLL_PID_INT_LIMIT, BODY_ROLL_PID_OUT_LIMIT);
static Core::Control::PID::Param pitch_pid_param(
    BODY_PITCH_PID_KP, BODY_PITCH_PID_KI, BODY_PITCH_PID_KD, BODY_PITCH_PID_INT_LIMIT, BODY_PITCH_PID_OUT_LIMIT);

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
// =====================================================================
// DM zero-point calibration switch.
// Set to 1 ONLY when re-zeroing the DM motors:
//   1. Set to 1, rebuild, flash.
//   2. Physically position every leg at the desired zero pose.
//   3. Hold ML+MR briefly to enter CALIBRATION (writes DM flash once),
//      then immediately release and return to IDLE.
//   4. Set back to 0, rebuild, flash again.
// Keep at 0 in normal builds to prevent accidental flash writes.
// =====================================================================
#define ENABLE_DM_CALIBRATION 0

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
DbgSummary dbg_summary __attribute__((used));
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
    max_pitch_deg_ = rad2deg(atan2f(2.0f * r_m_, wb_m_));
    max_roll_deg_  = rad2deg(atan2f(2.0f * r_m_, wt_f_m_));
    // Keep working range AWAY from kinematic singularity at θ=0,π.
    // At small |sin θ| the leg has near-zero mechanical advantage AND the
    // impedance Kp/Kd both saturate to their floors → no damping → violent
    // rocking. Restrict θ ∈ [30°, 150°] so sin² ≥ 0.25 always.
    float limit_cos = cosf(deg2rad(30.0f));
    h_max_          = R_m_ + r_m_ * limit_cos;
    h_min_          = R_m_ - r_m_ * limit_cos;
    // New calibration convention: H = R − r * cos(theta). theta=0 ⇔ lowest.
    target_chassis_height_  = R_m_ - r_m_ * cosf(deg2rad(INITIAL_LEG_ANGLE));
    target_height_setpoint_ = target_chassis_height_;
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
    // --- Intercept exit from CLIMBING: must home back to motor-frame 0°
    //     before actually switching modes. DM motors accumulate multi-turn
    //     position internally; switching to position-control mode while the
    //     legs are at ±170° (PREP/COMPLETE) and accumulated turns from
    //     velocity-mode crossings would cause the motor to unwind by full
    //     turns on the next position command. Returning to 0° guarantees a
    //     clean handoff regardless of accumulated turns. ---
    if (current_state_ == Chassis_State::CLIMBING && new_state != Chassis_State::CLIMBING)
    {
        if (climb_stage_ != ClimbStage::HOMING_OUT)
        {
            Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
            for (int i = 0; i < 4; i++)
                climb_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegPosition() : 0.0f;
            climb_homing_ticks_       = 0;
            climb_stage_              = ClimbStage::HOMING_OUT;
            climb_pending_exit_state_ = new_state;
        }
        // Stay in CLIMBING — handleClimbingMode will do the ramp and
        // re-call Set_Mode(pending) once HOMING_OUT completes.
        return;
    }

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
        impedance_.reset();  // Will start its kp ramp ONLY after homing completes

        // todo3: ES → COMFORT 软启动窗口。只在「上一模式 == ES」时开窗；
        // 其它模式进入 COMFORT 不触发，避免误伤正常切换响应。
        if (current_state_ == Chassis_State::ENERGY_SAVING)
            es_to_comfort_limit_ticks_ = ES2COMFORT_LIMIT_TICKS;

        // Capture the Kp/Kd the legs were holding under the previous mode so
        // the homing handoff doesn't release stiffness. ENERGY_SAVING uses
        // Pos_KP=80 Kd=4.0 to nail the leg at θ=0; any sudden drop to a lower
        // Kp releases gravity load and the chassis falls / kicks.
        float prev_kp = 80.0f, prev_kd = 4.0f;
        if (current_state_ == Chassis_State::ENERGY_SAVING)
        {
            prev_kp = 80.0f;
            prev_kd = 4.0f;
        }
        else if (current_state_ == Chassis_State::IDLE)
        {
            prev_kp = 0.0f;
            prev_kd = 0.5f;
        }
        for (int i = 0; i < 4; i++)
        {
            comfort_homing_kp_start_[i] = prev_kp;
            comfort_homing_kd_start_[i] = prev_kd;
        }

        comfort_phase_        = ComfortPhase::HOMING;
        comfort_homing_ticks_ = 0;
    }
    if (new_state == Chassis_State::ENERGY_SAVING)
    {
        // Capture per-leg motor-frame angle and prior Kp/Kd so HOMING can
        // smoothly walk every leg back to θ=0 without releasing stiffness
        // and without firing the Wheel_Compensation FF at homing speed.
        Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};

        // Decide entry Kp/Kd per source state (mirrors COMFORT homing logic).
        float prev_kp = 0.0f, prev_kd = 0.5f;
        if (current_state_ == Chassis_State::COMFORT)
        {
            // exit_kp_/exit_kd_ already populated above (per-leg).
        }
        else if (current_state_ == Chassis_State::IDLE)
        {
            prev_kp = 0.0f;
            prev_kd = 0.5f;
        }
        else
        {
            // Coming from any other mode: assume it was holding at ES values.
            prev_kp = ENERGY_HOMING_KP_END;
            prev_kd = ENERGY_HOMING_KD_END;
        }

        for (int i = 0; i < 4; i++)
        {
            energy_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegPosition() : 0.0f;
            if (current_state_ == Chassis_State::COMFORT)
            {
                energy_homing_kp_start_[i] = exit_kp_[i];
                energy_homing_kd_start_[i] = exit_kd_[i];
            }
            else
            {
                energy_homing_kp_start_[i] = prev_kp;
                energy_homing_kd_start_[i] = prev_kd;
            }
        }
        energy_phase_        = EnergyPhase::HOMING;
        energy_homing_ticks_ = 0;
        // HOMING runs its own Kp/Kd ramp; suppress the old global timer-based
        // ramp inside handleEnergySaving RUN so they don't fight.
        mode_transition_timer_ = 0;
    }
    if (new_state == Chassis_State::CLIMBING)
    {
        climbing_.reset();
        // DO NOT start climbing pipeline yet — first home all legs to 0°,
        // then wait for user BTN_X to begin climbing. This guarantees the
        // PREP angle (~±170°) is approached from 0° (single-direction
        // sweep, never crosses ±π wrap).
        Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
        for (int i = 0; i < 4; i++)
            climb_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegPosition() : 0.0f;
        climb_homing_ticks_ = 0;
        climb_stage_        = ClimbStage::HOMING_IN;
        climb_last_buttons_ = 0;
        // Match body height to the PREP motor angle so BL/BR (IDLE) don't
        // create a pitch difference with FL/FR (PREP).
        // PREP motor angle = 180° - prep_theta_deg.  Height at that angle
        // via H = R + r·cos(angle) gives the matching height.
        // This also leaves a safe low height when returning to COMFORT,
        // avoiding the θ≈0° singularity (near full extension).
        target_height_setpoint_ = CalculateHeightFromAngle(180.0f - climbing_.config().prep_theta_deg);
    }
}

void Chassis::slewTargetHeight()
{
    // Asymmetric slew: descending is HALF the rate of ascending.
    // Going down, gravity helps and θ shrinks toward the singularity (sin²→0,
    // Kp_MIT → mit_kp_min floor), so impedance authority is weakest exactly
    // when we are commanding the most aggressive motion. Slow it down so the
    // legs stay close to the slewing setpoint instead of free-falling into it.
    float diff      = target_height_setpoint_ - target_chassis_height_;
    float rate_up   = HEIGHT_SLEW_PER_CYCLE;
    float rate_down = HEIGHT_SLEW_PER_CYCLE * 0.5f;

    // todo3: ES → COMFORT 软启动窗口。窗口在 Set_Mode 中被开（置为
    // ES2COMFORT_LIMIT_TICKS），这里每 tick 递减一次。窗口内把上升速率压到
    // ES2COMFORT_MAX_H_DOT 以下，让乘客感觉抬升变柔。
    if (es_to_comfort_limit_ticks_ > 0)
    {
        constexpr float kCtrlHz    = 500.0f;
        const float capped_rate_up = ES2COMFORT_MAX_H_DOT / kCtrlHz;
        if (capped_rate_up < rate_up)
            rate_up = capped_rate_up;
        --es_to_comfort_limit_ticks_;
    }

    float dh_target = 0.0f;
    if (diff > rate_up)
        dh_target = rate_up;
    else if (diff < -rate_down)
        dh_target = -rate_down;
    else
        dh_target = diff;  // close enough: step exactly

    // Jerk-limit the rate itself: instead of stepping dh from 0 to rate_up in
    // one cycle (= infinite jerk), smooth dh with a one-pole LPF (~80 ms tau).
    // Effect: target_chassis_height_ profile becomes an S-curve, with finite
    // acceleration at start and stop. Ascent feels "smooth" instead of yank.
    static float dh_filt   = 0.0f;
    const float jerk_alpha = 0.06f;  // tau ≈ 80ms @ 500Hz
    dh_filt                = jerk_alpha * dh_target + (1.0f - jerk_alpha) * dh_filt;

    target_chassis_height_ += dh_filt;
    if (fabsf(dh_target) < 1e-7f && fabsf(dh_filt) < 1e-7f)
        target_chassis_height_ = target_height_setpoint_;  // snap when fully settled

    // Record signed rate (m/s) so executeBodyControlImpedance can feed it
    // forward into each leg's velocity command. Without this, MIT Kd resists
    // the commanded descent (target_vel=0 but actual_vel<0 → Kd pushes up),
    // accumulating position error until the loop "jumps" — the faster the
    // commanded descent, the bigger the jump.
    height_slew_rate_ = dh_filt * 500.0f;  // 500 Hz update
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

        // Keep DbgSummary live during disconnect so Ozone watch never freezes.
        updateDbgSummary();
        return;
    }

    // Extract buttons
    uint8_t current_buttons = cmd.button_status;
    uint8_t changing_edges  = current_buttons ^ last_button_status_;
    uint8_t rising_edges    = changing_edges & current_buttons;
    last_button_status_     = current_buttons;

    // Global driving speed tier from D-pad. Runs BEFORE mode dispatch so it
    // takes effect in every mode (all modes funnel vx/wz through
    // controller_.Map_Joystick_To_Velocity which reads the tier).
    controller_.Update_Speed_Tier(cmd.dpad_status);

    bool ml_pressed = (current_buttons & BTN_ML);
    bool mr_pressed = (current_buttons & BTN_MR);

    // --- TEMPORARY: only IDLE / ENERGY_SAVING / DEBUG enabled for live use ---
    // COMFORT / CLIMBING / FREE_CONTROL are skipped while bringup is in progress.
    // CALIBRATION combo (ML+MR) is gated by ENABLE_DM_CALIBRATION (see
    // handleCalibrationMode). When the macro is 0, the combo is also ignored.
#if ENABLE_DM_CALIBRATION
    if (ml_pressed && mr_pressed)
    {
        if (current_state_ != Chassis_State::CALIBRATION)
            Set_Mode(Chassis_State::CALIBRATION);
    }
#endif

    // Active state
    static const Chassis_State kAllowedStates[] = {
        Chassis_State::IDLE, Chassis_State::ENERGY_SAVING, Chassis_State::COMFORT, Chassis_State::CLIMBING};
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

    // Slew the consumed target height toward the discrete setpoint every cycle.
    // Done before mode dispatch so all modes see the smoothed value.
    slewTargetHeight();

    // Tell impedance mass estimator whether the chassis target is currently
    // slewing. While slewing, the body unloads (apparent weight drops as it
    // accelerates downward) → leg current drops → if we let it, M_est would
    // collapse → FFW collapses → deeper drop. Freeze the estimator instead.
    // Also hold the freeze for ~250ms AFTER slew stops, so the arrival ring
    // doesn't get baked into the warm-snap.
    static int settle_hold_ticks = 0;
    if (fabsf(height_slew_rate_) >= 1e-4f)
        settle_hold_ticks = 125;  // 250 ms @ 500 Hz
    else if (settle_hold_ticks > 0)
        settle_hold_ticks--;
    impedance_.setSettled(settle_hold_ticks == 0);

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

    updateDbgSummary();
}

void Chassis::updateDbgSummary()
{
    // Motor angle feedback (deg). Always fresh — Get_LegPosition reads
    // CAN feedback directly, no command pipeline needed.
    dbg_summary.angle_fl = FL_WheelLegs_ ? FL_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_summary.angle_fr = FR_WheelLegs_ ? FR_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_summary.angle_bl = BL_WheelLegs_ ? BL_WheelLegs_->Get_LegPosition() : 0.0f;
    dbg_summary.angle_br = BR_WheelLegs_ ? BR_WheelLegs_->Get_LegPosition() : 0.0f;

    // Per-leg current height (m): H = R - r * cos(theta_motor).
    // cos is even so bending_direction sign cancels out.
    auto angle_to_h        = [this](float deg) { return R_m_ - r_m_ * cosf(deg2rad(deg)); };
    dbg_summary.height_fl  = angle_to_h(dbg_summary.angle_fl);
    dbg_summary.height_fr  = angle_to_h(dbg_summary.angle_fr);
    dbg_summary.height_bl  = angle_to_h(dbg_summary.angle_bl);
    dbg_summary.height_br  = angle_to_h(dbg_summary.angle_br);
    dbg_summary.height_avg = 0.25f * (dbg_summary.height_fl + dbg_summary.height_fr + dbg_summary.height_bl + dbg_summary.height_br);

    dbg_summary.target_height = target_chassis_height_;

    // IMU snapshot (already filtered / transformed in readAndTransformIMU)
    dbg_summary.imu_pitch      = chassis_pitch_;
    dbg_summary.imu_roll       = chassis_roll_;
    dbg_summary.imu_pitch_rate = chassis_pitch_rate_;
    dbg_summary.imu_roll_rate  = chassis_roll_rate_;
    dbg_summary.imu_accel_z    = chassis_accel_z_;

    // Sprung-mass estimate from impedance controller
    dbg_summary.mass_est = impedance_.getEstimatedMass();

    dbg_summary.state = dbg_state;
}

void Chassis::handleCalibrationMode()
{
    // DM motors don't need ENTER_MOTOR ritual.
    //
    // SetZero() calls setZeroPosition(), which WRITES THE DM MOTOR'S FLASH and
    // permanently relocates the absolute encoder zero. Gated by
    // ENABLE_DM_CALIBRATION (file-scope macro near the top of this file).
#if ENABLE_DM_CALIBRATION
    if (FL_WheelLegs_)
        FL_WheelLegs_->SetZero();
    if (FR_WheelLegs_)
        FR_WheelLegs_->SetZero();
    if (BL_WheelLegs_)
        BL_WheelLegs_->SetZero();
    if (BR_WheelLegs_)
        BR_WheelLegs_->SetZero();
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

    // Level-trim: subtract mounting-tilt offsets measured on flat ground so
    // "perfectly level" reads (0, 0) in chassis frame. See Robot_Params.hpp.
    raw_pitch -= IMU_PITCH_OFFSET_DEG;
    raw_roll -= IMU_ROLL_OFFSET_DEG;

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
    // COMFORT button mappings: choose target chassis height (in meters).
    //
    // NOTE: COMFORT mode INTENTIONALLY does NOT touch bending_direction_. The
    // per-leg bending sign is configured statically in Robot_Config.cpp and
    // describes mechanical mounting — it should never be flipped at runtime.
    // Old code used to call SetBendingDirection() here; that was removed.
    //
    // theta convention: theta=0 ⇔ lowest pose, |theta|=180 ⇔ highest pose.
    //==========================================================================================

    // X: 90 deg — mid stance
    if (cmd.button_status & BTN_X)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(90.0f);
    }
    // Y: 145 deg — high stance
    else if (cmd.button_status & BTN_Y)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(145.0f);
    }
    // A: 45 deg — low stance
    else if (cmd.button_status & BTN_A)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(45.0f);
    }
    // B: 165 deg — highest stance
    else if (cmd.button_status & BTN_B)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(165.0f);
    }
    // RB: 135 deg
    else if (cmd.button_status & BTN_RB)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(135.0f);
    }
    // LB: 90 deg
    else if (cmd.button_status & BTN_LB)
    {
        target_height_setpoint_ = CalculateHeightFromAngle(90.0f);
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
    // 1. Body Leveling PID — LOW-BANDWIDTH version for COMFORT.
    //
    // The impedance loop already absorbs bumps (each leg has natural ω_n ≈
    // 9 Hz at θ=90°). Body leveling should only correct STEADY tilt (e.g.
    // weight shift, slope), NOT chase per-wheel bump transients.
    //
    // 拆轴 LPF：roll 摇晃带宽 > pitch（轨距短，模态频率高），
    // 具体 α 值见 Robot_Params.hpp::BODY_LPF_ALPHA_*。
    static float roll_lpf  = 0.0f;
    static float pitch_lpf = 0.0f;
    roll_lpf               = BODY_LPF_ALPHA_ROLL * chassis_roll_ + (1.0f - BODY_LPF_ALPHA_ROLL) * roll_lpf;
    pitch_lpf              = BODY_LPF_ALPHA_PITCH * chassis_pitch_ + (1.0f - BODY_LPF_ALPHA_PITCH) * pitch_lpf;

    float roll_h_adj  = roll_pid(0.0f, clampSym(roll_lpf, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(pitch_lpf, max_pitch_deg_));

    // PID 权威按轴独立标尺（参数见 BODY_PID_SCALE_*）。
    // roll 较高 → 压左右晃；pitch 较低 → 仅做慢漂移纠偏。
    roll_h_adj *= BODY_PID_SCALE_ROLL;
    pitch_h_adj *= BODY_PID_SCALE_PITCH;

    // Gate PID differential by impedance ramp_alpha (0→1 over ~0.4s) so we
    // never command a wide pitch/roll spread while legs are still at the
    // mode-entry pose and impedance Kp is climbing. Otherwise: entry @ θ≈0
    // (e.g. coming from ENERGY_SAVING which holds Leg_POS=0) + immediate
    // ±0.08m differential = legs slammed to ±70° with stiff Kp = explosion.
    float pid_gate = impedance_.getRampAlpha();
    roll_h_adj *= pid_gate;
    pitch_h_adj *= pid_gate;

    // 2. Gyro Feedforward — 角速度转高度差分速度，做"软阻尼器"。
    //    位置 PID 只对角度误差起作用，无法直接压住角速度。
    //    参数见 Robot_Params.hpp::BODY_GYRO_*。pitch 默认关闭。
    static float filt_pitch_rate_i = 0.0f, filt_roll_rate_i = 0.0f;
    filt_pitch_rate_i = BODY_GYRO_LPF_ALPHA * chassis_pitch_rate_ + (1.0f - BODY_GYRO_LPF_ALPHA) * filt_pitch_rate_i;
    filt_roll_rate_i  = BODY_GYRO_LPF_ALPHA * chassis_roll_rate_ + (1.0f - BODY_GYRO_LPF_ALPHA) * filt_roll_rate_i;
    float v_pitch_ff  = (wb_m_ / 2.0f) * filt_pitch_rate_i * BODY_GYRO_FF_GAIN_PITCH;
    float v_roll_ff   = (wt_f_m_ / 2.0f) * filt_roll_rate_i * BODY_GYRO_FF_GAIN_ROLL;

    // 2b. 自适应负载缩放 —— 解决"空载乱晃 / 满载需要的增益不同"问题。
    //     上面 SCALE/FF 是按 BODY_CTRL_NOMINAL_MASS_KG（满载）调出来的；
    //     乘上 (M_est / M_nominal) 后：空载自动变软、满载维持标定。
    //     钳到 [MIN, MAX] 防估计抖动把环开成 0 或拉飞。
    //     仅作用于位置/速度调平输出，不动 v_chassis（垂向 FFW 与 mass 解耦）。
    {
        const float m_est = impedance_.getEstimatedMass();
        float mass_scale  = m_est / BODY_CTRL_NOMINAL_MASS_KG;
        if (mass_scale < BODY_CTRL_MASS_SCALE_MIN)
            mass_scale = BODY_CTRL_MASS_SCALE_MIN;
        else if (mass_scale > BODY_CTRL_MASS_SCALE_MAX)
            mass_scale = BODY_CTRL_MASS_SCALE_MAX;
        roll_h_adj *= mass_scale;
        pitch_h_adj *= mass_scale;
        v_roll_ff *= mass_scale;
        v_pitch_ff *= mass_scale;
    }

    // Vertical velocity feedforward from the (jerk-limited) height slew. All
    // four legs share the same commanded vertical rate. Without this, MIT Kd
    // sees target_vel=0 while actual_vel≠0 during slew → Kd resists commanded
    // motion → position error accumulates → loop "snaps". height_slew_rate_
    // is already LPF-smoothed in slewTargetHeight() so no extra filter needed
    // here.
    float v_chassis = height_slew_rate_;

    // 3. Per-leg height + impedance params → Set_Leg_Height(h, v, kp, kd, ffw)
    //    PID offsets for pitch/roll leveling are position commands; Kp/Kd/FFW handle compliance.
    auto &out_fl = impedance_.getLegOutput(0);
    auto &out_fr = impedance_.getLegOutput(1);
    auto &out_bl = impedance_.getLegOutput(2);
    auto &out_br = impedance_.getLegOutput(3);

    // FL (Front-Left): +Pitch, -Roll
    float h_fl = clampHeight(target_chassis_height_ + pitch_h_adj - roll_h_adj + ground_contact_.getDeltaH(0));
    float v_fl = v_chassis + v_pitch_ff - v_roll_ff;
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl, out_fl.kp, out_fl.kd, out_fl.ffw_torque);

    // FR (Front-Right): +Pitch, +Roll
    float h_fr = clampHeight(target_chassis_height_ + pitch_h_adj + roll_h_adj + ground_contact_.getDeltaH(1));
    float v_fr = v_chassis + v_pitch_ff + v_roll_ff;
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr, out_fr.kp, out_fr.kd, out_fr.ffw_torque);

    // BL (Back-Left): -Pitch, -Roll
    float h_bl = clampHeight(target_chassis_height_ - pitch_h_adj - roll_h_adj + ground_contact_.getDeltaH(2));
    float v_bl = v_chassis - v_pitch_ff - v_roll_ff;
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl, out_bl.kp, out_bl.kd, out_bl.ffw_torque);

    // BR (Back-Right): -Pitch, +Roll
    float h_br = clampHeight(target_chassis_height_ - pitch_h_adj + roll_h_adj + ground_contact_.getDeltaH(3));
    float v_br = v_chassis - v_pitch_ff + v_roll_ff;
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
//
// Internal sub-state machine:
//   HOMING (first ~0.4s) — drive legs from arbitrary pose (esp. θ≈0 from
//     ENERGY_SAVING) to θ = ±COMFORT_HOMING_THETA away from the kinematic
//     singularity. Uses Set_Leg_Target with low Kp; wheels held at 0 RPM.
//     Impedance controller is NOT updated yet so its mass estimation and
//     PID gating ramp stay frozen.
//   RUN — normal variable-impedance + body-leveling control.
// =====================================================================
void Chassis::handleComfortMode(const Protocol::PC_Msg &cmd)
{
    Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};

    // --------- HOMING sub-state ---------
    if (comfort_phase_ == ComfortPhase::HOMING)
    {
        // Linear angle ramp from each leg's current angle (captured on tick 0)
        // toward target = ±COMFORT_HOMING_THETA, signed by bending direction.
        // Simultaneously ramp Kp/Kd from the previous mode's stiffness toward
        // COMFORT_HOMING_KP_END so legs are NEVER released — only made gentler.
        static float homing_start_angle[4] = {0};
        if (comfort_homing_ticks_ == 0)
        {
            for (int i = 0; i < 4; i++)
                homing_start_angle[i] = legs[i] ? legs[i]->Get_LegPosition() : 0.0f;
        }

        float alpha = (float)comfort_homing_ticks_ / (float)COMFORT_HOMING_FRAMES;
        if (alpha > 1.0f)
            alpha = 1.0f;
        // Smoothstep for gentler accel/decel at endpoints
        float s = alpha * alpha * (3.0f - 2.0f * alpha);

        // Sin-based gravity FFW with a guessed chassis mass keeps the static
        // equilibrium during HOMING aligned with what RUN converges to once
        // its FFW kicks in. Without this, kp=30+ffw=0 sags ~10° below the
        // commanded angle → handoff produces a visible second climb stage as
        // RUN's mass-aware FFW pushes the leg the remaining distance.
        const float r_m              = ECCENTRIC_OFFSET_r / 1000.0f;
        const float chassis_per_leg  = COMFORT_HOMING_CHASSIS_MASS_GUESS * 0.25f;
        const float ffw_load_per_leg = chassis_per_leg + LEG_MASS_kg;

        for (int i = 0; i < 4; i++)
        {
            if (!legs[i])
                continue;
            float target_deg = COMFORT_HOMING_THETA * (float)legs[i]->Get_Bending_Direction();
            float cmd_deg    = homing_start_angle[i] + s * (target_deg - homing_start_angle[i]);
            float kp_use     = comfort_homing_kp_start_[i] + alpha * (COMFORT_HOMING_KP_END - comfort_homing_kp_start_[i]);
            float kd_use     = comfort_homing_kd_start_[i] + alpha * (COMFORT_HOMING_KD_END - comfort_homing_kd_start_[i]);
            // Same convention as Impedance_Controller: ffw = +load·g·r·sin(θ_motor).
            // Use the COMMANDED angle (not feedback) so FFW grows smoothly with
            // the smoothstep angle ramp and is insensitive to sensor noise.
            float sin_cmd    = sinf(deg2rad(cmd_deg));
            float ffw_homing = ffw_load_per_leg * GRAVITY_g * r_m * sin_cmd;
            legs[i]->Set_Leg_Target(cmd_deg, 0.0f, ffw_homing, kp_use, kd_use);
            legs[i]->Set_Wheel_Target(0.0f);
        }

        comfort_homing_ticks_++;
        if (comfort_homing_ticks_ >= COMFORT_HOMING_FRAMES)
        {
            // Seed target_chassis_height_ to the post-homing height so the
            // first RUN cycle commands what the legs are already at.
            float h_homed           = CalculateHeightFromAngle(COMFORT_HOMING_THETA);
            target_chassis_height_  = clampHeight(h_homed);
            target_height_setpoint_ = target_chassis_height_;
            // Continuity: make impedance ramp start from HOMING's exit Kp/Kd,
            // not from the static defaults. Otherwise Kd jumps 2.0 → 1.5 at
            // the handoff and the legs lose damping for one cycle → twitch.
            impedance_.config().entry_kp = COMFORT_HOMING_KP_END;
            impedance_.config().entry_kd = COMFORT_HOMING_KD_END;
            // FFW continuity: seed the mass estimator with the same value used
            // for HOMING's FFW so the first RUN cycle's load_per_leg matches
            // HOMING's, AND mass_warmed_=true skips the snap (which used to
            // fire ~0.4 s into RUN as a second push). The real estimator will
            // then drift toward the true mass via the 0.8 Hz LPF — smooth.
            impedance_.seedMass(COMFORT_HOMING_CHASSIS_MASS_GUESS);
            comfort_phase_ = ComfortPhase::RUN;
        }

        executeMotorCommands();
        return;
    }

    // --------- RUN sub-state (normal impedance) ---------
    // 2. Button → Target Height
    handleHeightButtons(cmd);

    // 3. Gather per-leg data
    float leg_currents[4] = {legs[0] ? legs[0]->Get_LegCurrentFeedback() : 0.0f,
                             legs[1] ? legs[1]->Get_LegCurrentFeedback() : 0.0f,
                             legs[2] ? legs[2]->Get_LegCurrentFeedback() : 0.0f,
                             legs[3] ? legs[3]->Get_LegCurrentFeedback() : 0.0f};

    float leg_angles[4] = {legs[0] ? legs[0]->Get_LegPosition() : 0.0f,
                           legs[1] ? legs[1]->Get_LegPosition() : 0.0f,
                           legs[2] ? legs[2]->Get_LegPosition() : 0.0f,
                           legs[3] ? legs[3]->Get_LegPosition() : 0.0f};

    const float dt = 0.002f;  // 500 Hz

    // 4. Update Impedance Controller → per-leg Kp, Kd, FFW
    impedance_.update(leg_currents, leg_angles, chassis_accel_z_, chassis_roll_rate_, chassis_pitch_rate_, dt);

    // 4b. Update Ground Contact (warp) compensator. Roll+Pitch PID controls
    //     only 2 of 3 tilt DOFs; the diagonal "warp" mode is uncontrolled and
    //     lets one wheel lift off (e.g. FR on a step → FL+BR unloaded).
    //     Ground Contact senses current imbalance between diagonals and adds
    //     a small per-leg height bias orthogonal to roll/pitch.
    //
    //     IMPORTANT: leg_currents[] is the raw motor current, which is signed
    //     by bending_direction (front legs bend opposite to back legs). For a
    //     SAME downward ground reaction, FL motor torque has opposite sign to
    //     BL motor torque. We must normalize so that "more ground load" = more
    //     positive across all legs before computing the diagonal warp error.
    float i_norm[4];
    Wheel_Leg *legs_arr[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    for (int i = 0; i < 4; i++)
    {
        float bd  = legs_arr[i] ? (float)legs_arr[i]->Get_Bending_Direction() : 1.0f;
        i_norm[i] = leg_currents[i] * bd;
    }
    ground_contact_.update(i_norm, dt);

    dbg_gc.warp_error = ground_contact_.getWarpError();
    dbg_gc.warp_dh    = ground_contact_.getWarpDH();
    dbg_gc.dh_fl      = ground_contact_.getDeltaH(0);
    dbg_gc.dh_fr      = ground_contact_.getDeltaH(1);
    dbg_gc.dh_bl      = ground_contact_.getDeltaH(2);
    dbg_gc.dh_br      = ground_contact_.getDeltaH(3);

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
    // DEBUG: button-driven LOGICAL-frame angle setpoint for climbing tuning.
    //   X → +90°    A → +120°
    //   B → -90°    Y → -120°
    // Setpoint LATCHES until another button is pressed. Position control
    // (Pos_KP non-zero, vel=0). Each leg's raw motor target =
    // logical * bending_direction_, so all 4 legs bend the same physical way.
    // ------------------------------------------------------------------

    static float debug_target_deg = 0.0f;  // latched logical angle (deg)

    uint8_t btn = cmd.button_status;
    if (btn & BTN_X)
        debug_target_deg = 90.0f;
    else if (btn & BTN_A)
        debug_target_deg = 120.0f;
    else if (btn & BTN_B)
        debug_target_deg = -90.0f;
    else if (btn & BTN_Y)
        debug_target_deg = -120.0f;

    Wheel_Leg *legs[4]  = {FL_WheelLegs_, BL_WheelLegs_, FR_WheelLegs_, BR_WheelLegs_};
    float target_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; i++)
    {
        if (legs[i])
            target_deg[i] = debug_target_deg * (float)legs[i]->Get_Bending_Direction();
    }

    // Position control gains (MIT mode: Pos_KP * (target - pos) + Vel_KD * (0 - vel) + ffw)
    const float kp = 80.0f;
    const float kd = 4.0f;

    for (int i = 0; i < 4; i++)
    {
        if (legs[i])
            legs[i]->Set_Leg_Target(target_deg[i], 0.0f, 0.0f, kp, kd);
    }

    dbg_leveling.h_fl = target_deg[0] * (PI / 180.0f);
    dbg_leveling.h_fr = target_deg[2] * (PI / 180.0f);
    dbg_leveling.h_bl = target_deg[1] * (PI / 180.0f);
    dbg_leveling.h_br = target_deg[3] * (PI / 180.0f);

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
    Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};

    // --------- HOMING sub-state ---------
    // Walk every leg from its entry angle to motor-frame 0° via a smoothstep
    // trajectory; simultaneously ramp Kp/Kd from the previous mode's hold
    // values to the ES stiffness. CRUCIAL: wheels held at 0 RPM with NO
    // Wheel_Compensation — the decoupling FF (∝ leg_rpm) at homing speed
    // would otherwise drive the front and back axles in opposite directions
    // (the "rear wheels go forward / front legs flip back / car tilts then
    // snaps level" bug). The DM slew-rate limiter inside Execute_Leg_Control
    // still caps any cycle-to-cycle step, so this is safe even if the
    // entry angle is large.
    if (energy_phase_ == EnergyPhase::HOMING)
    {
        float alpha = (float)energy_homing_ticks_ / (float)ENERGY_HOMING_FRAMES;
        if (alpha > 1.0f)
            alpha = 1.0f;
        // Smoothstep: zero velocity at both endpoints → no jerk at start/end.
        float s = alpha * alpha * (3.0f - 2.0f * alpha);

        for (int i = 0; i < 4; i++)
        {
            if (!legs[i])
                continue;
            // Linear (smoothstepped) angle interpolation in motor frame
            // toward 0. Don't multiply by bending_direction — these are
            // already raw motor-frame angles captured in Set_Mode.
            float cmd_deg = energy_homing_start_angle_[i] * (1.0f - s);
            float kp_use  = energy_homing_kp_start_[i] + alpha * (ENERGY_HOMING_KP_END - energy_homing_kp_start_[i]);
            float kd_use  = energy_homing_kd_start_[i] + alpha * (ENERGY_HOMING_KD_END - energy_homing_kd_start_[i]);
            legs[i]->Set_Leg_Target(cmd_deg, 0.0f, 0.0f, kp_use, kd_use);
            legs[i]->Set_Wheel_Target(0.0f);
            // INTENTIONALLY no Add_Wheel_Compensation here.
        }

        energy_homing_ticks_++;
        if (energy_homing_ticks_ >= ENERGY_HOMING_FRAMES)
        {
            energy_phase_ = EnergyPhase::RUN;
        }

        executeMotorCommands();
        return;
    }

    // --------- RUN sub-state (normal ENERGY_SAVING) ---------
    // Legs are at θ≈0 and stationary, so Wheel_Compensation() (proportional
    // to leg_rpm) is ~0 and Set_Wheel_Leg's internal Add_Wheel_Compensation
    // call is harmless. Joystick now drives the wheels normally.
    float target_kp = ENERGY_HOMING_KP_END;
    float target_kd = ENERGY_HOMING_KD_END;
    float kp_use    = target_kp;
    float kd_use    = target_kd;

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

    // =================================================================
    // Sub-state machine: HOMING_IN → WAIT_START → ACTIVE → HOMING_OUT
    // HOMING_IN / HOMING_OUT and WAIT_START all hold legs near 0°. The
    // ACTIVE branch runs the original climbing pipeline (below).
    // =================================================================
    Wheel_Leg *legs_top[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};

    if (climb_stage_ == ClimbStage::HOMING_IN || climb_stage_ == ClimbStage::HOMING_OUT)
    {
        // Smoothstep all legs from their captured start angle → 0°.
        // Position control (kp=80, kd=4). Wheels follow joystick so the
        // robot can still be driven while legs ramp to/from 0°.
        float alpha = (float)climb_homing_ticks_ / (float)CLIMB_HOMING_FRAMES;
        if (alpha > 1.0f)
            alpha = 1.0f;
        float s    = alpha * alpha * (3.0f - 2.0f * alpha);
        float kp_h = CLIMB_HOMING_KP_END;
        float kd_h = CLIMB_HOMING_KD_END;
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            float cmd_deg = climb_homing_start_angle_[i] * (1.0f - s);
            legs_top[i]->Set_Leg_Target(cmd_deg, 0.0f, 0.0f, kp_h, kd_h);
        }

        // Wheel velocity control from joystick (with wheel-side compensation).
        // legs_top order: FL, FR, BL, BR — matches inverseKinematics output.
        float wheel_rpms_h[4], vx_h = 0.0f, wz_h = 0.0f;
        controller_.Map_Joystick_To_Velocity(cmd, vx_h, wz_h);
        inverseKinematics(vx_h, 0, wz_h, wheel_rpms_h);
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            legs_top[i]->Set_Wheel_Target(wheel_rpms_h[i]);
            legs_top[i]->Add_Wheel_Compensation(legs_top[i]->Wheel_Compensation());
        }

        climb_homing_ticks_++;
        if (climb_homing_ticks_ >= CLIMB_HOMING_FRAMES)
        {
            if (climb_stage_ == ClimbStage::HOMING_IN)
            {
                climb_stage_        = ClimbStage::WAIT_START;
                climb_last_buttons_ = cmd.button_status;
            }
            else  // HOMING_OUT done → perform deferred state change
            {
                Chassis_State target = climb_pending_exit_state_;
                // Force current_state_ to bypass our own intercept on next call.
                current_state_ = Chassis_State::IDLE;
                climb_stage_   = ClimbStage::HOMING_IN;  // reset for next entry
                Set_Mode(target);
                return;
            }
        }

        executeMotorCommands();
        return;
    }

    if (climb_stage_ == ClimbStage::WAIT_START)
    {
        // Hold all legs at 0°, wait for BTN_X rising edge to begin climbing.
        // Wheels follow joystick (robot stays drivable while parked at 0°).
        float kp_h = CLIMB_HOMING_KP_END;
        float kd_h = CLIMB_HOMING_KD_END;
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            legs_top[i]->Set_Leg_Target(0.0f, 0.0f, 0.0f, kp_h, kd_h);
        }

        float wheel_rpms_w[4], vx_w = 0.0f, wz_w = 0.0f;
        controller_.Map_Joystick_To_Velocity(cmd, vx_w, wz_w);
        inverseKinematics(vx_w, 0, wz_w, wheel_rpms_w);
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            legs_top[i]->Set_Wheel_Target(wheel_rpms_w[i]);
            legs_top[i]->Add_Wheel_Compensation(legs_top[i]->Wheel_Compensation());
        }

        bool x_now  = (cmd.button_status & BTN_X);
        bool x_prev = (climb_last_buttons_ & BTN_X);
        if (x_now && !x_prev)
        {
            climbing_.startClimbAll();
            climb_stage_ = ClimbStage::ACTIVE;
        }
        climb_last_buttons_ = cmd.button_status;

        executeMotorCommands();
        return;
    }

    // climb_stage_ == ACTIVE → fall through to original climbing pipeline

    // --- Trigger-based DIRECT ANGLE debug (verify climbing direction) ---
    // L trigger → FL/FR sweep from -(180-deadzone) toward 0° (climbing direction)
    // R trigger → BL/BR sweep from +(180-deadzone) toward 0° (climbing direction)
    // Released = PREP position (±165°),  Fully pressed = 0° (extended)
    float l_ratio_c    = (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    float r_ratio_c    = (float)cmd.Right_trigger_x1000_msg / 1000.0f;
    bool manual_climb  = (l_ratio_c > 0.05f || r_ratio_c > 0.05f);
    float deadzone_deg = climbing_.config().prep_theta_deg;
    float prep_angle   = 180.0f - deadzone_deg;  // e.g. 165°
    // Sweep magnitude: ratio=0 → prep_angle, ratio=1 → 0°. Per-leg sign is
    // applied below via climb_sign[] so all four legs bend the right way.
    float l_sweep = prep_angle * (1.0f - l_ratio_c);  // FL/FR magnitude
    float r_sweep = prep_angle * (1.0f - r_ratio_c);  // BL/BR magnitude

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
    float lev_signs[4][2] = {{1, -1}, {1, 1}, {-1, -1}, {-1, 1}};  // {pitch_sign, roll_sign}
    float gv_signs[4][2]  = {{1, -1}, {1, 1}, {-1, -1}, {-1, 1}};  // same for gyro FF
    // Per-leg trigger sweep magnitude (sign applied below via climb_sign[]).
    float trigger_mag[4] = {l_sweep, l_sweep, r_sweep, r_sweep};

    // Climbing angle sign per leg: picks one of the two ±θ solutions of the
    // inverse kinematics so each leg bends in the physically-correct direction.
    //   angle_cmd = climb_sign[i] * (180 - prep_theta)
    // Because the climbing path is pure velocity servo (Pos_KP=0), a leg can only
    // rest at the sign its climb_sign dictates: +1 -> +θ, -1 -> -θ.
    float climb_sign[4] = {1.0f, -1.0f, 1.0f, 1.0f};

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
            // climb_sign[i] picks the physically-correct ±θ branch per leg.
            float angle_cmd_dbg = climb_sign[i] * trigger_mag[i];
            float ffw           = legs[i]->Get_LegGravityTorque();
            legs[i]->Set_Leg_Target(angle_cmd_dbg, 0.0f, ffw, kp_use, kd_use);
            dbg_angle_cmd[i] = angle_cmd_dbg;
        }
        else if (climbing_.isDirectControl(i))
        {
            // --- Velocity-tracking control for climbing phases ---
            // Climbing_Dynamics gives us:
            //   theta_unsigned: per-tick trajectory target (180°=highest, 0°=lowest)
            //   omega_unsigned: trajectory angular velocity feed-forward (rad/s)
            //
            // We used to send the (signed) target angle as a position command with
            // the motor running Pos_KP closed-loop. Because the MIT frame wraps
            // position into [-π, π], a target that crosses the ±180° boundary —
            // e.g. when the leg starts on the opposite wrap side from the command —
            // makes the DM solver pick the long way round and the motor spins a
            // full turn before settling. The mitigation: keep the *target angle*
            // for visibility but actually drive the motor with a velocity command
            // computed from the shortest-path error to that target. Pos_KP is set
            // to 0 so the motor's internal position loop can't introduce the wrap
            // bug; the angular velocity loop (Vel_KD) does the work.
            float theta_unsigned = climbing_.getTargetThetaDeg(i);
            float omega_unsigned = climbing_.getTargetOmega(i);  // rad/s, negative when θ decreasing

            // climb_sign: FL/FR = -1, BL/BR = +1. Both mirrors converge toward 0°.
            float angle_cmd     = climb_sign[i] * theta_unsigned;
            float ffw_omega_rad = climb_sign[i] * omega_unsigned;  // trajectory feed-forward (rad/s)

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

            // Shortest-path angular error in (-180°, +180°] — kills the wraparound bug.
            float cur_deg = legs[i]->Get_LegPosition();
            float err_deg = angle_cmd - cur_deg;
            while (err_deg > 180.0f)
                err_deg -= 360.0f;
            while (err_deg <= -180.0f)
                err_deg += 360.0f;
            float err_rad = err_deg * (3.14159265f / 180.0f);

            // P-controller (position-error → velocity) + trajectory feed-forward.
            const float kp_vel    = climbing_.config().climb_pos_kp_vel;
            const float omega_lim = climbing_.config().climb_omega_max;
            float vel_cmd_rad     = kp_vel * err_rad + ffw_omega_rad;
            if (vel_cmd_rad > omega_lim)
                vel_cmd_rad = omega_lim;
            if (vel_cmd_rad < -omega_lim)
                vel_cmd_rad = -omega_lim;

            // Velocity-only MIT: Pos_KP = 0 so the motor ignores the position target
            // (no more ±π long-way-around). The Pos_KP-zero path means we don't care
            // what position we hand to Set_Leg_Target; send current position so any
            // ramp / slew limiter inside Wheel_Leg stays in sync with reality.
            float ffw = legs[i]->Get_LegGravityTorque();
            legs[i]->Set_Leg_Target(cur_deg, vel_cmd_rad, ffw, 0.0f, kd_use);
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

// NEW convention: theta=0 ⇔ lowest (H = R - r), |theta|=180 ⇔ highest (H = R + r).
// Matches Set_Leg_Height in Wheel_Leg.cpp (H = R - r*cos(theta)).
float Chassis::CalculateHeightFromAngle(float angle_deg) { return R_m_ - r_m_ * cosf(deg2rad(angle_deg)); }

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
