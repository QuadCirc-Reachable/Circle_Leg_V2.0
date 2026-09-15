/**
 * @file    Chassis.cpp
 * @brief   Chassis orchestrator: mode state machine, IMU attitude, body leveling,
 *          inverse kinematics and per-leg height distribution.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

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
DbgClimbTarget dbg_climb_tgt __attribute__((used));
DbgClimbPlot dbg_climb_plot __attribute__((used));
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

float Chassis::getAdaptiveLoadPerLeg() const
{
    // Source priority:
    //   1. Climbing PREP-phase mass estimator (climb_mass_estimate_kg_) once
    //      it has finalized this session -- this is the most accurate value
    //      for the CURRENT climb because it was measured by THIS climb's
    //      PREP sweep, capturing whatever load (empty or with rider) is
    //      actually on board RIGHT NOW.
    //   2. Impedance estimator's live mass (warmed from COMFORT RUN).
    //   3. Empty-chassis fallback if M_est too cold to trust.
    //
    // Why empty-chassis fallback (NOT loaded): if M_est is genuinely cold
    // (~0, e.g. IDLE -> CLIMBING with no COMFORT visit), the PREP estimator
    // will measure the truth within ~0.5 s of starting PREP. Using a LOADED
    // fallback (73 kg) for that brief window means FFW would push 3x too
    // hard on an empty chassis -> the leg overshoots target by ~12 deg ->
    // theta_motor crosses past the +/-180 seam -> multi-turn unwind risk.
    // Empty fallback (23 kg) errs the safe way for the leg seam; under a
    // loaded cold-start the legs would briefly sag (~12 deg under-shoot)
    // during the first sin-rich window, then catch up once PREP-mass
    // estimator finalizes. No multi-turn risk.
    if (climb_mass_estimated_)
        return climb_mass_estimate_kg_ * 0.25f + LEG_MASS_kg;

    float M_est = impedance_.getEstimatedMass();
    if (M_est < 10.0f)
        M_est = ROBOT_MASS_kg - 4.0f * LEG_MASS_kg;  // empty chassis ~ 23 kg
    return M_est * 0.25f + LEG_MASS_kg;
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
            {
                climb_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegAngleWrapped() : 0.0f;
                // Exit-from-CLIMBING uses Set_Leg_PD_Torque with PD gains
                // CLIMB_HOMING_KP_END / KD_END for the whole descent. Seed
                // the ramp start with the same values so Kp/Kd hold constant
                // (no stiffness change) while the angle smoothsteps to 0.
                climb_homing_kp_start_[i] = CLIMB_HOMING_KP_END;
                climb_homing_kd_start_[i] = CLIMB_HOMING_KD_END;
            }
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

        // SOFT START at COMFORT HOMING entry to prevent the transition-tick
        // torque spike that exceeded the motor's burst limit (observed
        // hardware "motor jitter" on ES->COMFORT). At entry the legs are at
        // theta~0 (folded), bearing NO chassis load (the body is on the
        // wheels). So Kp can safely start very low -- there is no support
        // role to preserve. The ramp climbs to COMFORT_HOMING_KP_END=30 over
        // the smoothstep, gaining stiffness as the legs reach loading angles.
        // Even if the very first tick has a few degrees of residual err
        // (from sensor noise or settling), Kp*err = 10 * 0.05 = 0.5 Nm is
        // gentle -- nowhere near burst torque.
        float prev_kp = 10.0f, prev_kd = 2.0f;
        if (current_state_ == Chassis_State::ENERGY_SAVING)
        {
            prev_kp = 10.0f;
            prev_kd = 2.0f;
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
        // Capture per-leg motor-frame angle and prior Kp/Kd so the ES descent
        // smoothstep starts from each leg's actual entry pose -- this is what
        // makes the transition jump-free (target = current at tick 0).
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
            energy_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegAngleWrapped() : 0.0f;
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
        //
        // Capture per-leg entry Kp/Kd so HOMING_IN can ramp stiffness
        // smoothly from the previous mode's gains (mirrors the ES descent
        // style). Without this, HOMING_IN jumps from soft IDLE-hold (or
        // COMFORT impedance) straight to kp=80 -> visible yank on entry.
        Wheel_Leg *legs[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};

        // Decide per-leg entry Kp/Kd by source state.
        float prev_kp_default = 0.0f, prev_kd_default = 0.5f;
        if (current_state_ == Chassis_State::ENERGY_SAVING)
        {
            prev_kp_default = ENERGY_HOMING_KP_END;  // ES holds at (80, 4) -- continuous
            prev_kd_default = ENERGY_HOMING_KD_END;
        }
        else if (current_state_ == Chassis_State::IDLE)
        {
            prev_kp_default = 0.0f;  // IDLE drops the legs to low-Kp hold
            prev_kd_default = 0.5f;
        }
        else
        {
            // Coming from any other mode (DEBUG, FREE, etc): start near end.
            prev_kp_default = CLIMB_HOMING_KP_END;
            prev_kd_default = CLIMB_HOMING_KD_END;
        }

        for (int i = 0; i < 4; i++)
        {
            climb_homing_start_angle_[i] = legs[i] ? legs[i]->Get_LegAngleWrapped() : 0.0f;
            if (current_state_ == Chassis_State::COMFORT)
            {
                // exit_kp_/exit_kd_ already populated above (per-leg) by the
                // "leaving COMFORT" capture block.
                climb_homing_kp_start_[i] = exit_kp_[i];
                climb_homing_kd_start_[i] = exit_kd_[i];
            }
            else
            {
                climb_homing_kp_start_[i] = prev_kp_default;
                climb_homing_kd_start_[i] = prev_kd_default;
            }
        }
        climb_homing_ticks_ = 0;
        climb_stage_        = ClimbStage::HOMING_IN;
        climb_last_buttons_ = 0;
        // HOMING_IN runs its own Kp/Kd ramp; suppress the old global timer
        // ramp inside the ACTIVE branch so they don't fight.
        mode_transition_timer_ = 0;
        // Reset PREP-phase mass estimator: each climb session starts a fresh
        // measurement of "what's currently on top of the chassis".
        for (int i = 0; i < 4; i++)
        {
            prep_mass_load_sum_[i] = 0.0f;
            prep_mass_count_[i]    = 0;
        }
        climb_mass_estimated_   = false;
        climb_mass_estimate_kg_ = 0.0f;
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
    float diff = target_height_setpoint_ - target_chassis_height_;
    // Symmetric rate: 0.5x HEIGHT_SLEW_PER_CYCLE both ways. Full-rate up
    // (0.2 m/s) under load caused motor stress / audible complaint per the
    // user. Halving brings it to 0.1 m/s -- same speed as descent -- making
    // in-COMFORT height button changes uniformly gentle.
    float rate_up   = HEIGHT_SLEW_PER_CYCLE * 0.5f;
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

    // ADAPTIVE gravity comp: uses impedance estimator's live mass so the
    // residual is "leg_torque - expected static" rather than "leg_torque -
    // (LEG_MASS only, cos formula)". The old `Get_LegGravityTorque()`
    // ignored the chassis + rider (only 4 kg compensated, ~70 kg ignored)
    // AND used a cos formula 90 deg out of phase with the actual eccentric
    // geometry, so the residual carried a large mass + angle dependent
    // baseline offset that the LPF had to absorb -- making detection
    // sensitive to rider weight, pitch transitions, mass shifts. With this
    // formula `residual ~= 0` in steady state regardless of rider weight,
    // and any spike is direct disturbance torque.
    const float ffw_load_per_leg_all = getAdaptiveLoadPerLeg();
    const float r_m_all              = ECCENTRIC_OFFSET_r / 1000.0f;

    Wheel_Leg *legs_all[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    float t_fb[4]   = {0, 0, 0, 0};
    float g_comp[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++)
    {
        if (!legs_all[i])
            continue;
        t_fb[i]         = legs_all[i]->Get_LegTorqueFeedback();
        float theta_rad = deg2rad(legs_all[i]->Get_LegAngleWrapped());
        // Correct eccentric-leg gravity torque: tau = m * g * r * sin(theta_motor)
        // (mass at lateral offset r*sin(theta), gravity creates motor-axis torque).
        g_comp[i] = ffw_load_per_leg_all * GRAVITY_g * r_m_all * sinf(theta_rad);
    }

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
    dbg_leg.fb_fl  = FL_WheelLegs_ ? FL_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_leg.fb_fr  = FR_WheelLegs_ ? FR_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_leg.fb_bl  = BL_WheelLegs_ ? BL_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_leg.fb_br  = BR_WheelLegs_ ? BR_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_leg.cmd_fl = FL_WheelLegs_ ? FL_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_fr = FR_WheelLegs_ ? FR_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_bl = BL_WheelLegs_ ? BL_WheelLegs_->Get_FinalLegCommand() : 0.0f;
    dbg_leg.cmd_br = BR_WheelLegs_ ? BR_WheelLegs_->Get_FinalLegCommand() : 0.0f;

    updateDbgSummary();
}

void Chassis::updateDbgSummary()
{
    // Motor angle feedback (deg). Always fresh — Get_LegAngleWrapped reads
    // CAN feedback directly, no command pipeline needed.
    dbg_summary.angle_fl = FL_WheelLegs_ ? FL_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_summary.angle_fr = FR_WheelLegs_ ? FR_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_summary.angle_bl = BL_WheelLegs_ ? BL_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
    dbg_summary.angle_br = BR_WheelLegs_ ? BR_WheelLegs_->Get_LegAngleWrapped() : 0.0f;

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

    // (Was: pid_gate = impedance_.getRampAlpha() to ramp PID effect in from 0
    //  at RUN start. After A4q, body PID is already gated by the COMFORT
    //  HOMING smoothstep `s` and reaches FULL effect by HOLD's end. Gating it
    //  again here from 0 would drop the PID action from full back to zero at
    //  the HOMING -> RUN handoff and ramp it up again over ~1 s -- this is
    //  the BIG IMPACT users felt at "switching to impedance control". Removed.
    //  PID is now continuously at full strength across HOMING + HOLD + RUN.)

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
                homing_start_angle[i] = legs[i] ? legs[i]->Get_LegAngleWrapped() : 0.0f;
        }

        float alpha = (float)comfort_homing_ticks_ / (float)COMFORT_HOMING_FRAMES;
        if (alpha > 1.0f)
            alpha = 1.0f;
        // Smoothstep for gentler accel/decel at endpoints
        float s         = alpha * alpha * (3.0f - 2.0f * alpha);
        float ds_dalpha = 6.0f * alpha * (1.0f - alpha);  // for FF wheel comp below

        // Sin-based gravity FFW with a guessed chassis mass keeps the static
        // equilibrium during HOMING aligned with what RUN converges to once
        // its FFW kicks in. Without this, kp=30+ffw=0 sags ~10° below the
        // commanded angle → handoff produces a visible second climb stage as
        // RUN's mass-aware FFW pushes the leg the remaining distance.
        const float r_m              = ECCENTRIC_OFFSET_r / 1000.0f;
        const float chassis_per_leg  = COMFORT_HOMING_CHASSIS_MASS_GUESS * 0.25f;
        const float ffw_load_per_leg = chassis_per_leg + LEG_MASS_kg;

        // --- Body PID active DURING HOMING so the chassis ends LEVEL at the
        //     end of the lift, not tilted because of an off-centre rider load
        //     that uniform per-leg targets cannot compensate. The "再抬一下"
        //     feel after HOMING was the body PID kicking in at RUN start and
        //     levelling a chassis that arrived tilted; now PID levels it
        //     CONTINUOUSLY during the lift so RUN handoff sees a level body.
        //     Scale PID output by s (smoothstep progress) so PID has no effect
        //     at t=0 (legs haven't moved, no real tilt error yet) and grows
        //     to full strength as the lift completes. The same h_adj signs
        //     (lev_signs) and SCALE constants are used as in RUN -> a tilt
        //     correction during HOMING is equivalent to one during RUN.
        static float roll_lpf_homing  = 0.0f;
        static float pitch_lpf_homing = 0.0f;
        if (comfort_homing_ticks_ == 0)
        {
            // Seed LPF with current IMU angle on the first tick so it doesn't
            // start at 0 and create a synthetic tilt error.
            roll_lpf_homing  = chassis_roll_;
            pitch_lpf_homing = chassis_pitch_;
        }
        roll_lpf_homing       = BODY_LPF_ALPHA_ROLL * chassis_roll_ + (1.0f - BODY_LPF_ALPHA_ROLL) * roll_lpf_homing;
        pitch_lpf_homing      = BODY_LPF_ALPHA_PITCH * chassis_pitch_ + (1.0f - BODY_LPF_ALPHA_PITCH) * pitch_lpf_homing;
        float roll_h_adj_hom  = roll_pid(0.0f, clampSym(roll_lpf_homing, max_roll_deg_)) * BODY_PID_SCALE_ROLL * s;
        float pitch_h_adj_hom = pitch_pid(0.0f, clampSym(pitch_lpf_homing, max_pitch_deg_)) * BODY_PID_SCALE_PITCH * s;

        // lev_signs: {pitch_sign, roll_sign} per leg, same as RUN's mixing.
        const float lev_signs_hom[4][2] = {{1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, -1.0f}, {-1.0f, 1.0f}};

        for (int i = 0; i < 4; i++)
        {
            if (!legs[i])
                continue;
            float target_deg = COMFORT_HOMING_THETA * (float)legs[i]->Get_Bending_Direction();
            float cmd_deg    = homing_start_angle[i] + s * (target_deg - homing_start_angle[i]);
            // Per-leg leveling: convert h_adj (meters) -> dtheta (deg) via
            //   dH = r*sin(theta)*dtheta -> dtheta = dH/(r*sin(theta))
            // Sign on theta carries through, so this works for both bending
            // directions (positive cmd_deg -> +sin, negative -> -sin -> sign
            // flips automatically).
            float dh        = lev_signs_hom[i][0] * pitch_h_adj_hom + lev_signs_hom[i][1] * roll_h_adj_hom;
            float sin_cmd_l = sinf(deg2rad(cmd_deg));
            if (fabsf(sin_cmd_l) < 0.15f)
                sin_cmd_l = copysignf(0.15f, sin_cmd_l);
            float dtheta = dh / (r_m * sin_cmd_l) * (180.0f / 3.14159265f);
            if (dtheta > 30.0f)
                dtheta = 30.0f;
            if (dtheta < -30.0f)
                dtheta = -30.0f;
            cmd_deg += dtheta;
            float kp_use     = comfort_homing_kp_start_[i] + alpha * (COMFORT_HOMING_KP_END - comfort_homing_kp_start_[i]);
            float kd_use     = comfort_homing_kd_start_[i] + alpha * (COMFORT_HOMING_KD_END - comfort_homing_kd_start_[i]);
            // Same convention as Impedance_Controller: ffw = +load·g·r·sin(θ_motor).
            // Use the COMMANDED angle (not feedback) so FFW grows smoothly with
            // the smoothstep angle ramp and is insensitive to sensor noise.
            float sin_cmd    = sinf(deg2rad(cmd_deg));
            float ffw_homing = ffw_load_per_leg * GRAVITY_g * r_m * sin_cmd;
            // Software PD-torque (Pos_KP = Vel_KD = 0 on the motor) so the DM
            // can never unwind multi-turn during the entry-pose ramp. tau_max
            // matches the DM hardware ceiling -- the loaded sin-FFW (~13 Nm at
            // theta=90) plus Kd*omega_ff during the smoothstep ramp can hit
            // 30+ Nm peak under a rider; a low clamp here would saturate the
            // lift and look like "legs sag during COMFORT entry".
            constexpr float HOMING_TAU_MAX = 200.0f;
            legs[i]->Set_Leg_PD_Torque(cmd_deg, 0.0f, kp_use, kd_use, ffw_homing, HOMING_TAU_MAX);
        }

        // Wheels: joystick velocity + FEEDFORWARD wheel compensation. The
        // earlier "no wheel comp here" stance applied to the FB (measured
        // leg rpm) variant -- its front/back-axle coupling could fight
        // itself when legs had asymmetric actual rates. The FF variant
        // takes the *commanded* leg omega (the smoothstep derivative is
        // shared across all four legs only in magnitude scaled by each
        // leg's start->target delta), so wheel comp directions are
        // self-consistent. Without this the leg arm rotates by 60 deg
        // during the lift, dragging the wheel center along an arc, but
        // the wheel motor sees only joystick rpm -> wheel scrubs ground
        // during the entire HOMING lift (felt as leg "fighting" the
        // ground/wheel friction).
        constexpr float COMFORT_HOMING_DURATION_S = COMFORT_HOMING_FRAMES * 0.002f;
        float wheel_rpms_h[4], vx_h = 0.0f, wz_h = 0.0f;
        controller_.Map_Joystick_To_Velocity(cmd, vx_h, wz_h);
        inverseKinematics(vx_h, 0, wz_h, wheel_rpms_h);
        for (int i = 0; i < 4; i++)
        {
            if (!legs[i])
                continue;
            float target_deg_w      = COMFORT_HOMING_THETA * (float)legs[i]->Get_Bending_Direction();
            float omega_ff_rad_wcmp = (target_deg_w - homing_start_angle[i]) * ds_dalpha
                                      * (3.14159265f / 180.0f) / COMFORT_HOMING_DURATION_S;
            legs[i]->Set_Wheel_Target(wheel_rpms_h[i]);
            legs[i]->Add_Wheel_Compensation(legs[i]->Wheel_Compensation(omega_ff_rad_wcmp));
        }

        comfort_homing_ticks_++;
        // Transition to RUN only after BOTH the lift smoothstep (FRAMES) AND
        // the steady-hold tail (HOLD_FRAMES) complete. While ticks are in the
        // HOLD region, alpha clamps to 1.0 above -> s=1 -> target stays at
        // COMFORT_HOMING_THETA, kp_use/kd_use stay at KP_END/KD_END, and ffw
        // stays on the open-loop sin-based form. The leg fully settles at the
        // target before impedance/body PID take over.
        if (comfort_homing_ticks_ >= COMFORT_HOMING_FRAMES + COMFORT_HOLD_FRAMES)
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

    float leg_angles[4] = {legs[0] ? legs[0]->Get_LegAngleWrapped() : 0.0f,
                           legs[1] ? legs[1]->Get_LegAngleWrapped() : 0.0f,
                           legs[2] ? legs[2]->Get_LegAngleWrapped() : 0.0f,
                           legs[3] ? legs[3]->Get_LegAngleWrapped() : 0.0f};

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
    // 175 deg (not 180) keeps the trigger-driven target 5 deg away from the
    // +/-180 wrap seam, so NearestEquivalentTarget never has to resolve a
    // boundary target (the same overshoot/wrap class of bugs we fought in
    // CLIMBING applies here too -- mechanical rotation is 360 deg unlimited).
    static const float folded_angle = 175.0f;

    // Triggers Control Angle Interpolation
    // Left Trigger : FL/FR target  (released = -folded_angle, pressed = 0 = extended)
    // Right Trigger: BL/BR target  (released = +folded_angle, pressed = 0 = extended)
    float l_ratio = (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    float r_ratio = (float)cmd.Right_trigger_x1000_msg / 1000.0f;

    float fl_fr_angle = -folded_angle * (1.0f - l_ratio);
    float bl_br_angle = folded_angle * (1.0f - r_ratio);

    // --- Unified PD-torque legs (Pos_KP=Vel_KD=0 on motor -> no multi-turn unwind) ---
    constexpr float FREE_KP      = 50.0f;
    constexpr float FREE_KD      = 1.0f;
    constexpr float FREE_TAU_MAX = 30.0f;
    Wheel_Leg *legs[4]   = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    float target_deg[4]  = {fl_fr_angle, fl_fr_angle, bl_br_angle, bl_br_angle};
    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;
        float ffw = legs[i]->Get_LegGravityTorque();
        legs[i]->Set_Leg_PD_Torque(target_deg[i], 0.0f, FREE_KP, FREE_KD, ffw, FREE_TAU_MAX);
    }

    // --- Wheels: joystick velocity + decoupling FF ---
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);
    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;
        legs[i]->Set_Wheel_Target(wheel_rpms[i]);
        legs[i]->Add_Wheel_Compensation(legs[i]->Wheel_Compensation());
    }

    executeMotorCommands();
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

    // Position control via software PD-torque (Pos_KP=Vel_KD=0 on motor side
    // -> no multi-turn unwind at the +/-pi seam). Same Kp/Kd semantics as MIT.
    constexpr float kp           = 80.0f;
    constexpr float kd           = 4.0f;
    constexpr float DBG_TAU_MAX  = 30.0f;
    for (int i = 0; i < 4; i++)
    {
        if (legs[i])
            legs[i]->Set_Leg_PD_Torque(target_deg[i], 0.0f, kp, kd, 0.0f, DBG_TAU_MAX);
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

    // === ES descent: simple smoothstep angle ramp + PD + gravity FFW ===
    // Per user direction "简单的速度和位置闭环就好了, 如果有前馈就加前馈,
    // 主打稳定 smooth": don't reuse the full impedance machinery (which jumped
    // on entry because it referenced target_chassis_height_ that didn't match
    // the leg's actual pose). Just smoothstep each leg's target angle from
    // its captured START angle (entry pose) to 0 over a fixed duration -- the
    // target STARTS at the current angle, so zero discontinuity on entry, no
    // possible jump.
    //
    // Control law (per leg, per tick):
    //   target_now  = start * (1 - smoothstep(t/T))                    // ramps to 0
    //   omega_ff    = -start * d(smoothstep)/dt                        // matched velocity FFW
    //   ffw_gravity = (M_est/4 + LEG_MASS) * g * r * sin(theta_now)     // grav comp
    //   tau         = Kp*(target_now - cur) + Kd*(omega_ff - omega_fb) + ffw_gravity
    // Set_Leg_PD_Torque handles the PD + FFW computation; we just compose
    // the trajectory + ffw inputs here.
    //
    // Set_Mode captures energy_homing_start_angle_[i] and zeroes
    // energy_homing_ticks_ on entry, so the handler picks up from tick 0.

    float alpha = (float)energy_homing_ticks_ / (float)ENERGY_HOMING_FRAMES;
    if (alpha > 1.0f)
        alpha = 1.0f;
    float s         = alpha * alpha * (3.0f - 2.0f * alpha);   // smoothstep, 0 at endpoints
    float ds_dalpha = 6.0f * alpha * (1.0f - alpha);           // smoothstep derivative

    // Stiffer position loop than the earlier 50/8. Differential turning loads
    // the legs sideways via the eccentric wheel coupling -- under that
    // reaction torque the previous Kp let the legs drift back-and-forth (user
    // observed wheels "扯前后" during left/right yaw). 120 Nm/rad holds the
    // theta=0 pose firmly through yaw transients. Kd raised to maintain the
    // damping ratio relative to the higher Kp.
    constexpr float ES_KP        = 120.0f;
    constexpr float ES_KD        = 12.0f;
    constexpr float ES_TAU_MAX   = 200.0f;
    constexpr float ES_DURATION_S = ENERGY_HOMING_FRAMES * 0.002f;  // ticks * dt = seconds

    // Gravity FFW: uniform M_est/4 per leg. M_est carries forward from the
    // last COMFORT phase (impedance doesn't update at theta~0 due to the sin
    // singularity gate, so it holds its last good value).
    const float M_est        = impedance_.getEstimatedMass();
    const float load_per_leg = M_est * 0.25f + LEG_MASS_kg;
    const float r_m          = ECCENTRIC_OFFSET_r / 1000.0f;

    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;
        float start         = energy_homing_start_angle_[i];
        float target_now    = start * (1.0f - s);              // deg, ramps start -> 0
        // d(target)/dt = -start * ds/dalpha * (1/duration), convert deg -> rad for omega_ff
        float omega_ff_rad  = -start * ds_dalpha * (3.14159265f / 180.0f) / ES_DURATION_S;
        float sin_now       = sinf(deg2rad(legs[i]->Get_LegAngleWrapped()));
        float ffw_gravity   = load_per_leg * GRAVITY_g * r_m * sin_now;
        legs[i]->Set_Leg_PD_Torque(target_now, omega_ff_rad, ES_KP, ES_KD, ffw_gravity, ES_TAU_MAX);
    }

    energy_homing_ticks_++;
    if (energy_homing_ticks_ > ENERGY_HOMING_FRAMES)
        energy_homing_ticks_ = ENERGY_HOMING_FRAMES;  // clamp so smoothstep stays at s=1 (target=0)

    // Wheels: joystick velocity + FEEDFORWARD wheel compensation. Earlier
    // versions intentionally skipped wheel comp here ("comp prop. to leg_rpm
    // would drive front/back apart"); that reasoning applied to the FB
    // (measured-rpm) variant which couples per-leg actual motion into wheel
    // commands and can fight itself when legs have different actual rates.
    // The FF variant takes the *commanded* leg omega per leg -- all four
    // share the same smoothstep, so the wheel comp directions are coherent
    // (each wheel rolls with its own leg's commanded motion). Without this
    // the wheel center traverses an arc while the wheel motor stays at the
    // joystick rpm -> wheel scrubs the ground during the descent.
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);
    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;
        float start             = energy_homing_start_angle_[i];
        float omega_ff_rad_wcmp = -start * ds_dalpha * (3.14159265f / 180.0f) / ES_DURATION_S;
        legs[i]->Set_Wheel_Target(wheel_rpms[i]);
        legs[i]->Add_Wheel_Compensation(legs[i]->Wheel_Compensation(omega_ff_rad_wcmp));
    }

    executeMotorCommands();
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
        // Smooth ES-style descent (HOMING_OUT) / entry (HOMING_IN):
        //   target_now   = start * (1 - smoothstep(t/T))      // ramps start -> 0
        //   omega_ff_rad = -start * d(smoothstep)/dt          // matched velocity FFW
        //   ffw_gravity  = (M_est/4 + LEG_MASS) * g * r * sin(theta_now)
        //   Kp/Kd        = ramp from per-leg start -> CLIMB_HOMING_KP/KD_END
        //   tau_max      = 200 (DM hardware ceiling)
        // Why this matters: under rider load the OLD path (kp=80, no FFW,
        // tau_max=30) lagged the smoothstep target by ~9 deg at theta=90 --
        // gravity won and the leg would "fall" the last few degrees into 0
        // when PD finally caught up. Matched omega_ff + gravity FFW makes the
        // PD do only error-correction; the leg now tracks the smoothstep
        // continuously, identical feel to the ES descent (COMFORT->ES).
        float alpha = (float)climb_homing_ticks_ / (float)CLIMB_HOMING_FRAMES;
        if (alpha > 1.0f)
            alpha = 1.0f;
        float s         = alpha * alpha * (3.0f - 2.0f * alpha);   // smoothstep
        float ds_dalpha = 6.0f * alpha * (1.0f - alpha);           // smoothstep derivative

        constexpr float CLIMB_HOMING_TAU_MAX = 200.0f;
        constexpr float CLIMB_HOMING_DURATION_S = CLIMB_HOMING_FRAMES * 0.002f;

        // Loaded-chassis gravity FFW. M_est is the impedance estimator's
        // last good value (carries forward from prior mode -- HOMING_IN from
        // COMFORT inherits the warmed estimate; HOMING_OUT inherits from the
        // ACTIVE branch's loaded seed). Same form as ES descent.
        const float M_est_local        = impedance_.getEstimatedMass();
        const float load_per_leg_local = M_est_local * 0.25f + LEG_MASS_kg;
        const float r_m_local          = ECCENTRIC_OFFSET_r / 1000.0f;

        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            float start         = climb_homing_start_angle_[i];
            float cmd_deg       = start * (1.0f - s);
            // d(target)/dt = -start * ds/dalpha * (1/duration) in deg/s;
            // convert deg -> rad for omega_ff (Set_Leg_PD_Torque expects rad/s).
            float omega_ff_rad  = -start * ds_dalpha * (3.14159265f / 180.0f) / CLIMB_HOMING_DURATION_S;
            // Use motor-feedback angle (not cmd_deg) for FFW so it tracks the
            // actual pose -- under load the leg can lag the smoothstep target.
            float sin_now       = sinf(deg2rad(legs_top[i]->Get_LegAngleWrapped()));
            float ffw_gravity   = load_per_leg_local * GRAVITY_g * r_m_local * sin_now;
            float kp_use        = climb_homing_kp_start_[i] + alpha * (CLIMB_HOMING_KP_END - climb_homing_kp_start_[i]);
            float kd_use        = climb_homing_kd_start_[i] + alpha * (CLIMB_HOMING_KD_END - climb_homing_kd_start_[i]);
            legs_top[i]->Set_Leg_PD_Torque(cmd_deg, omega_ff_rad, kp_use, kd_use, ffw_gravity, CLIMB_HOMING_TAU_MAX);
        }

        // Wheel velocity control from joystick + FEEDFORWARD wheel
        // compensation. The leg is being commanded at omega_ff_rad (the
        // smoothstep derivative) -- the FB variant of Wheel_Compensation()
        // would read leg_motor->getRPMFeedback() which lags this command
        // and lets the wheel motor trail the leg accel -> wheel scrubs the
        // ground during the homing sweep. Using the FF overload with the
        // commanded leg omega closes the loop ahead of time.
        // legs_top order: FL, FR, BL, BR -- matches inverseKinematics output.
        float wheel_rpms_h[4], vx_h = 0.0f, wz_h = 0.0f;
        controller_.Map_Joystick_To_Velocity(cmd, vx_h, wz_h);
        inverseKinematics(vx_h, 0, wz_h, wheel_rpms_h);
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            // Recompute the same omega_ff used for the leg PD above so the
            // wheel comp matches the leg's commanded angular velocity.
            float start             = climb_homing_start_angle_[i];
            float omega_ff_rad_wcmp = -start * ds_dalpha * (3.14159265f / 180.0f) / CLIMB_HOMING_DURATION_S;
            legs_top[i]->Set_Wheel_Target(wheel_rpms_h[i]);
            legs_top[i]->Add_Wheel_Compensation(legs_top[i]->Wheel_Compensation(omega_ff_rad_wcmp));
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
        constexpr float WAIT_TAU_MAX = 30.0f;
        for (int i = 0; i < 4; i++)
        {
            if (!legs_top[i])
                continue;
            legs_top[i]->Set_Leg_PD_Torque(0.0f, 0.0f, kp_h, kd_h, 0.0f, WAIT_TAU_MAX);
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

    // A4ar: MANUAL back-pair trigger on BTN_X rising during ACTIVE. If
    // auto-detection of back-wheel step-contact is unreliable, the operator
    // can position the back wheels at the step and press BTN_X again to
    // force-trigger BL/BR climb (bypasses all DETECT gates). Only allowed
    // when FL/FR are BOTH COMPLETE (back can't climb before front anyway).
    {
        bool x_now_active  = (cmd.button_status & BTN_X);
        bool x_prev_active = (climb_last_buttons_ & BTN_X);
        if (x_now_active && !x_prev_active)
        {
            bool front_both_complete =
                (climbing_.getPhase(0) == LegClimbPhase::COMPLETE &&
                 climbing_.getPhase(1) == LegClimbPhase::COMPLETE);
            if (front_both_complete)
            {
                float bl_deg = BL_WheelLegs_ ? BL_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
                float br_deg = BR_WheelLegs_ ? BR_WheelLegs_->Get_LegAngleWrapped() : 0.0f;
                climbing_.manualTriggerBackPair(bl_deg, br_deg);
            }
        }
        climb_last_buttons_ = cmd.button_status;
    }

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
    // Compute torque residual per leg using the ADAPTIVE gravity comp
    // (impedance-estimator-driven, full loaded-chassis sin formula). The
    // residual is then a direct measurement of the disturbance torque -- in
    // steady state ~= 0 regardless of rider weight, spikes only when the
    // wheel encounters an obstacle (step edge).
    const float climb_ffw_load_per_leg = getAdaptiveLoadPerLeg();
    const float climb_r_m              = ECCENTRIC_OFFSET_r / 1000.0f;
    Wheel_Leg *legs_climb[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    float tres[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++)
    {
        if (!legs_climb[i])
            continue;
        float theta_rad = deg2rad(legs_climb[i]->Get_LegAngleWrapped());
        float grav_static = climb_ffw_load_per_leg * GRAVITY_g * climb_r_m * sinf(theta_rad);
        tres[i] = legs_climb[i]->Get_LegTorqueFeedback() - grav_static;
    }
    // A4ag/A4ah: DEBOUNCED wheel-stall flag. Raw condition: user commanding
    // forward (left-stick past deadzone) AND wheel not rolling (|RPM| <
    // threshold). But the raw condition is TRUE for the first few ticks of
    // any forward push -- the wheel motor hasn't spun up yet (RPM still ~0).
    // Treating that startup transient as a stall would FALSE-TRIGGER climb
    // the instant the user pushes. So require the raw condition to PERSIST
    // for WHEEL_STALL_CONFIRM_S: a genuine stall (jammed at step) holds the
    // wheel at ~0 indefinitely; a spin-up transient clears within ~0.1 s
    // once the wheel starts rolling, resetting the timer.
    bool user_pushing_fwd = (cmd.left_joystick.r_x1000_msg > 200);
    // A4am: STATE-MACHINE wheel-stall detection.
    //
    // The simple "user pushing + low RPM" debounce (A4ag/A4ah/A4ak) fails at
    // startup: when the chassis is heavy and the user pushes from rest, the
    // wheel motor pulls high current to accelerate but takes 0.3-0.8 s to
    // cross the stall RPM threshold -- in that window the "raw stall" is
    // continuously true, and even a 0.2 s confirm fires falsely. User
    // observed: "起步很慢并且有大扭矩 直接判断堵转, 一动就直接 climb 了".
    //
    // Fix: per-leg "was_rolling" flag. The wheel must FIRST cross
    // WHEEL_ROLLING_RPM (= clearly rolling) before stall judgment is enabled.
    // - Spin-up from rest: RPM rising 0->target, never crossed ROLLING ->
    //   was_rolling = false -> stall judgment OFF -> no false trigger.
    // - Wheel rolled normally then hit step: RPM crossed ROLLING (= true)
    //   then dropped < STALL -> stall timer accumulates -> trigger.
    // Reset on joystick release (robot stopped, new attempt resets state).
    // A4an: split stall RPM threshold front vs back.
    //   Front (FL/FR): HT motor fully BRAKES against the step face -> the
    //     front wheel reaches near-0 RPM cleanly when blocked. Tight 5 RPM
    //     threshold detects this strict stall.
    //   Back (BL/BR): wheel decelerates from rolling but doesn't fully stop
    //     (edge friction/scrape, not face-jam). User observation: "BL BR
    //     climb 时会有减速现象，但不会堵转为 0". A higher 15 RPM threshold
    //     catches "rolling -> slowed past 15" as a stall, even if the
    //     wheel keeps slow-slipping at 5-12 RPM.
    // ROLLING threshold (25, common) is the was_rolling latch -- the wheel
    // must first spin clearly fast before stall judgment is enabled (so
    // heavy-load spin-up from rest doesn't false-trigger).
    constexpr float WHEEL_ROLLING_RPM       = 25.0f;
    constexpr float WHEEL_STALL_RPM_FRONT   = 5.0f;
    constexpr float WHEEL_STALL_RPM_BACK    = 15.0f;
    constexpr float WHEEL_STALL_CONFIRM_S   = 0.20f;
    // A4ao: VELOCITY RESIDUAL path (mirror of torque residual).
    //   The wheel RPM has its own LPF baseline (slow ~1.6 Hz). When the
    //   wheel suddenly decelerates (e.g. hits step edge), baseline lags
    //   the actual rpm and the "drop" = (baseline - rpm) reveals the
    //   deceleration EVENT, even if the wheel doesn't stop completely
    //   (catches "decelerated from 30 -> 12 RPM" which absolute stall at
    //   < 15 might only barely catch, and at < 5 wouldn't catch at all).
    //   Per user HW: BL/BR decelerates but doesn't reach 0 -- exactly the
    //   case this path handles. Combined OR with abs_stall in raw_stall.
    constexpr float WHEEL_RPM_LPF_ALPHA   = 0.02f;  // ~1.6 Hz tau ~ 100 ms
    // A4ap: drop threshold raised 10 -> 25 RPM after A4ao false-triggered
    // on normal joystick push variation (user squeezes/releases gently ->
    // wheel rpm wobbles 10-15 RPM -> false climb trigger). 25 RPM drop
    // requires a substantial deceleration event (e.g. wheel really jamming
    // at step), not normal speed variation.
    constexpr float WHEEL_DECEL_THRESHOLD = 25.0f;
    Wheel_Leg *legs_stall[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    bool wheel_blocked_arr[4] = {false, false, false, false};
    for (int i = 0; i < 4; i++)
    {
        if (!legs_stall[i])
            continue;
        float rpm_abs = fabsf(legs_stall[i]->Get_WheelRPM());

        // A4ao: LPF baseline of wheel RPM. Slow tracking (~1.6 Hz) so that
        // a sudden deceleration creates a large transient `drop`.
        wheel_rpm_baseline_[i] = WHEEL_RPM_LPF_ALPHA * rpm_abs +
                                 (1.0f - WHEEL_RPM_LPF_ALPHA) * wheel_rpm_baseline_[i];

        // Latch "was rolling" once the wheel actually spins (A4am).
        if (rpm_abs > WHEEL_ROLLING_RPM)
            wheel_was_rolling_[i] = true;

        // Joystick release -> robot stopped -> reset stall state for next attempt.
        if (!user_pushing_fwd)
        {
            wheel_was_rolling_[i]    = false;
            wheel_stall_timer_[i]    = 0.0f;
            wheel_rpm_baseline_[i]   = rpm_abs;  // fresh baseline on next push
        }

        // Per-leg stall threshold: front strict, back relaxed (A4an).
        float stall_thresh = (i < 2) ? WHEEL_STALL_RPM_FRONT : WHEEL_STALL_RPM_BACK;
        bool abs_stall    = (rpm_abs < stall_thresh);

        // A4aq: wheel_blocked now ONLY = abs_stall (with was_rolling). The
        // velocity-residual path moved to Climbing_Dynamics as part of the
        // sum-of-scores combined detection (avoids double-counting the
        // wheel drop signal in both Chassis OR and the combined score).
        // The wheel_rpm_baseline_ still tracks here -- we publish the
        // drop value via feedback for Climbing_Dynamics to use.
        bool raw_stall = user_pushing_fwd && wheel_was_rolling_[i] && abs_stall;
        if (raw_stall)
            wheel_stall_timer_[i] += 0.002f;  // 500 Hz tick
        else
            wheel_stall_timer_[i] = 0.0f;
        wheel_blocked_arr[i] = (wheel_stall_timer_[i] > WHEEL_STALL_CONFIRM_S);
    }
    // A4aq: compute wheel_drop per leg = baseline - |rpm| (positive when
    // wheel decelerates below its smoothed baseline). Pass to
    // Climbing_Dynamics for the combined sum-of-scores detection.
    auto compute_wheel_drop = [&](int idx) -> float {
        if (!legs_stall[idx]) return 0.0f;
        float rpm_abs_local = fabsf(legs_stall[idx]->Get_WheelRPM());
        float d = wheel_rpm_baseline_[idx] - rpm_abs_local;
        return (d > 0.0f) ? d : 0.0f;  // only report positive drops (decel)
    };
    if (FL_WheelLegs_)
        fb[0] = {FL_WheelLegs_->Get_LegAngleWrapped(), FL_WheelLegs_->Get_WheelRPM(), tres[0], FL_WheelLegs_->Get_LegVelocity(), wheel_blocked_arr[0], compute_wheel_drop(0)};
    if (FR_WheelLegs_)
        fb[1] = {FR_WheelLegs_->Get_LegAngleWrapped(), FR_WheelLegs_->Get_WheelRPM(), tres[1], FR_WheelLegs_->Get_LegVelocity(), wheel_blocked_arr[1], compute_wheel_drop(1)};
    if (BL_WheelLegs_)
        fb[2] = {BL_WheelLegs_->Get_LegAngleWrapped(), BL_WheelLegs_->Get_WheelRPM(), tres[2], BL_WheelLegs_->Get_LegVelocity(), wheel_blocked_arr[2], compute_wheel_drop(2)};
    if (BR_WheelLegs_)
        fb[3] = {BR_WheelLegs_->Get_LegAngleWrapped(), BR_WheelLegs_->Get_WheelRPM(), tres[3], BR_WheelLegs_->Get_LegVelocity(), wheel_blocked_arr[3], compute_wheel_drop(3)};

    // A4ao: publish velocity-residual diagnostics for Ozone plot.
    for (int k = 0; k < 4; k++)
    {
        dbg_climb_plot.wheel_rpm_baseline[k] = wheel_rpm_baseline_[k];
        if (legs_stall[k])
            dbg_climb_plot.wheel_rpm_drop[k] = wheel_rpm_baseline_[k] - fabsf(legs_stall[k]->Get_WheelRPM());
    }

    // A4as: publish detection diagnostic scores. These are populated by
    // Climbing_Dynamics in DETECT case; zero outside DETECT. With a leg in
    // DETECT, plot these alongside wheel_rpm_drop to see end-to-end signal
    // path and identify which gate (front_gate / back_settle / turning /
    // warmup) is blocking trigger when combined_score >= 1 but detect_allowed
    // = 0.
    for (int k = 0; k < 4; k++)
    {
        float ts = climbing_.getTorqueScore(k);
        float ws = climbing_.getWheelScore(k);
        dbg_climb_plot.t_score[k]        = ts;
        dbg_climb_plot.w_score[k]        = ws;
        dbg_climb_plot.combined_score[k] = ts + ws;
        dbg_climb_plot.detect_allowed[k] = climbing_.getDetectAllowed(k) ? 1 : 0;
        dbg_climb_plot.detect_timer_s[k] = climbing_.getDetectTimer(k);
    }

    climbing_.config().step_height_m                = dbg_ctrl.step_height_mm / 1000.0f;
    climbing_.config().torque_res_threshold         = dbg_ctrl.torque_res_threshold;       // front
    climbing_.config().torque_res_threshold_back    = dbg_ctrl.torque_res_threshold_back;  // A4al back
    climbing_.config().climb_omega                  = dbg_ctrl.climb_omega;
    climbing_.config().back_detect_hold_offset_deg  = dbg_ctrl.climb_back_lift_offset_deg;
    const float dt                          = 0.002f;

    // A4ad: inhibit step detection while turning. Read the RAW right-stick
    // (rotation) magnitude directly from the joystick -- NOT via
    // Map_Joystick_To_Velocity, which has a stateful slew limiter that must
    // be called exactly once per tick (the wheel block below owns that call).
    // A yaw maneuver differentially loads the eccentric legs and fakes a
    // step-contact torque residual, so detection is suppressed whenever the
    // operator is steering. r_x1000_msg is 0..1000 stick magnitude; the
    // controller deadzone is 200, so 200 here matches "any real turn input".
    {
        constexpr uint16_t DETECT_TURN_INHIBIT_R = 200;  // right-stick magnitude deadzone
        bool turning = (cmd.right_joystick.r_x1000_msg > DETECT_TURN_INHIBIT_R);
        climbing_.setDetectInhibit(turning);
        dbg_climb_plot.detect_inhibited = turning ? 1 : 0;
    }

    climbing_.update(fb, dt);

    // ---- PREP-phase mass estimation ----
    // While the legs sweep through theta_motor near 90 deg during PREP, the
    // motor torque feedback satisfies the static balance equation
    //   tau_motor = (M_per_leg + LEG_MASS) * g * r * sin(theta_motor)
    // (dynamic terms I_eff*theta_ddot and damping are small for the 3s
    // smoothstep). Inverting per-leg gives M_per_leg + LEG_MASS in real time.
    // Accumulate during the high-sin window per leg; finalize after enough
    // samples and seed impedance so the rest of CLIMBING (DETECT, CLIMBING,
    // COMPLETE) has accurate FFW.
    //
    // This lets cold-start IDLE -> CLIMBING measure load (empty vs. rider)
    // BEFORE accurate FFW is needed for DETECT / CLIMBING -- decouples
    // climb-mass calibration from having to enter COMFORT first.
    if (!climb_mass_estimated_)
    {
        Wheel_Leg *legs_mass[4] = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
        const float r_m_mass    = ECCENTRIC_OFFSET_r / 1000.0f;
        for (int i = 0; i < 4; i++)
        {
            if (!legs_mass[i] || climbing_.getPhase(i) != LegClimbPhase::PREP)
                continue;
            float theta_motor = legs_mass[i]->Get_LegAngleWrapped();
            float s_motor     = sinf(deg2rad(theta_motor));
            if (fabsf(s_motor) < PREP_MASS_SIN_THRESHOLD)
                continue;
            // m_per_leg_inst = tau / (g*r*sin) -- signed numerator and
            // denominator give a consistent +m_per_leg regardless of which
            // bending direction the leg uses (climb_sign cancels).
            float tau    = legs_mass[i]->Get_LegTorqueFeedback();
            float m_inst = tau / (GRAVITY_g * r_m_mass * s_motor);
            prep_mass_load_sum_[i] += m_inst;
            prep_mass_count_[i]++;
        }

        // Finalize when every leg has accumulated enough samples (in-sync
        // smoothstep means counts are similar). Average per leg, sum, subtract
        // 4*LEG_MASS to get the SPRUNG (chassis + rider) mass.
        int min_count = prep_mass_count_[0];
        for (int i = 1; i < 4; i++)
            if (prep_mass_count_[i] < min_count) min_count = prep_mass_count_[i];

        if (min_count >= PREP_MASS_MIN_SAMPLES)
        {
            float load_total = 0.0f;
            for (int i = 0; i < 4; i++)
                load_total += prep_mass_load_sum_[i] / (float)prep_mass_count_[i];
            // load_total = sum of (M/4 + LEG_MASS) per leg = M + 4*LEG_MASS
            float M_inst = load_total - 4.0f * LEG_MASS_kg;
            if (M_inst < 0.0f)
                M_inst = 0.0f;
            climb_mass_estimate_kg_ = M_inst;
            climb_mass_estimated_   = true;
            impedance_.seedMass(M_inst);  // sync impedance for future COMFORT entry
        }
    }

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

    // ---- A4tt: phase-aware pitch bias (unloads the wheels that are climbing) ----
    //
    // Stage 1: front not yet COMPLETE (PREP / DETECT / CLIMBING on FL/FR)
    //   target = +climb_pitch_front_deg (+15 deg nose UP)
    //   Shifts CoM backward, unloading front wheels for easier pivot.
    //
    // Stage 2: front COMPLETE, back still active
    //   target = climb_pitch_back_deg (-5 deg nose DOWN)
    //   Shifts CoM forward; back wheels lighter -> easier roll forward to
    //   contact step, easier pivot once they reach it.
    //
    // Stage 3: all COMPLETE
    //   target = 0 (level)
    //
    // LPF smooths the +15 -> -5 jump (20 deg setpoint swing) over ~1.2 s so
    // the chassis doesn't snap-transition; passenger feels each attitude
    // change as a gentle lean, not a jolt.
    auto phFL = climbing_.getPhase(0), phFR = climbing_.getPhase(1);
    auto phBL = climbing_.getPhase(2), phBR = climbing_.getPhase(3);
    // Convenience flags (kept names for other uses below, e.g. PID gating).
    bool front_active = (phFL == LegClimbPhase::PREP || phFL == LegClimbPhase::DETECT || phFL == LegClimbPhase::CLIMBING ||
                         phFR == LegClimbPhase::PREP || phFR == LegClimbPhase::DETECT || phFR == LegClimbPhase::CLIMBING);
    bool back_active  = (phBL == LegClimbPhase::PREP || phBL == LegClimbPhase::DETECT || phBL == LegClimbPhase::CLIMBING ||
                         phBR == LegClimbPhase::PREP || phBR == LegClimbPhase::DETECT || phBR == LegClimbPhase::CLIMBING);
    bool front_climbing = (phFL == LegClimbPhase::CLIMBING || phFR == LegClimbPhase::CLIMBING);
    bool back_climbing  = (phBL == LegClimbPhase::CLIMBING || phBR == LegClimbPhase::CLIMBING);
    bool front_all_complete = (phFL == LegClimbPhase::COMPLETE && phFR == LegClimbPhase::COMPLETE);

    float target_pitch_deg = 0.0f;
    if (!front_all_complete && front_active)
    {
        // Stage 1: front-active. Nose UP to unload front.
        target_pitch_deg = dbg_ctrl.climb_pitch_front_deg;
    }
    else if (front_all_complete && back_active)
    {
        // Stage 2: front done, back working. Nose DOWN to shift CoM forward.
        target_pitch_deg = dbg_ctrl.climb_pitch_back_deg;
    }
    // else: stage 3 (all done) or initial (no leg yet in any phase) -> 0

    // LPF for smooth passenger-friendly transitions.
    static float pitch_sp_filt = 0.0f;
    pitch_sp_filt = dbg_ctrl.climb_pitch_lpf_alpha * target_pitch_deg +
                    (1.0f - dbg_ctrl.climb_pitch_lpf_alpha) * pitch_sp_filt;
    float pitch_setpoint = pitch_sp_filt;

    // Expose to plot for tuning visibility
    dbg_climb_plot.pitch_setpoint_raw_deg  = target_pitch_deg;
    dbg_climb_plot.pitch_setpoint_filt_deg = pitch_setpoint;

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
    // Confirmed empirically (left/right mirror): FL/BL go positive, FR/BR
    // negative. With the unified torque-track this sign is the ONLY thing that
    // distinguishes the four legs' commanded targets (magnitude is identical).
    float climb_sign[4] = {1.0f, -1.0f, 1.0f, -1.0f};

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

    // A4aw: update FL/FR dtheta authority ramp once per tick (before the
    // per-leg loop). The authority [0..1] multiplies FL/FR's body-PID
    // dtheta. Target = 0 while BL/BR is in CLIMBING (kill the up-down
    // jerk from PID chasing the dynamic chassis pitch during back climb);
    // target = 1 otherwise (full PID leveling for static poses).
    // Smoothstep timer-based transition (matches the gentle-ramp pattern
    // user is comfortable with from PREP), tunable via climb_dtheta_ramp_s.
    {
        bool back_any_climbing = (climbing_.getPhase(2) == LegClimbPhase::CLIMBING ||
                                  climbing_.getPhase(3) == LegClimbPhase::CLIMBING);
        float new_target = back_any_climbing ? 0.0f : 1.0f;
        if (fabsf(new_target - climb_dtheta_authority_target_) > 0.01f)
        {
            // Rising/falling edge: start a fresh smoothstep transition
            // from the CURRENT authority value (catches the "interrupt
            // mid-ramp" case where back exits CLIMBING before authority
            // reached 0 -- ramp restarts back to 1 from wherever it was).
            climb_dtheta_authority_start_  = climb_dtheta_authority_;
            climb_dtheta_authority_target_ = new_target;
            climb_dtheta_authority_t_      = 0.0f;
        }
        const float dt_aw    = 0.002f;
        const float ramp_s   = (dbg_ctrl.climb_dtheta_ramp_s > 1e-3f) ? dbg_ctrl.climb_dtheta_ramp_s : 1e-3f;
        climb_dtheta_authority_t_ += dt_aw;
        float alpha_aw = climb_dtheta_authority_t_ / ramp_s;
        if (alpha_aw > 1.0f) alpha_aw = 1.0f;
        float s_aw = alpha_aw * alpha_aw * (3.0f - 2.0f * alpha_aw);  // smoothstep
        climb_dtheta_authority_ = climb_dtheta_authority_start_ +
                                  (climb_dtheta_authority_target_ - climb_dtheta_authority_start_) * s_aw;
    }

    // A4ax: per-leg smoothstep authorities for (a) the COMPLETE-phase
    // PID gate, and (b) the direct angle offsets (front_drop / back_extend).
    // Both eliminate the binary-snap symptoms user reported after A4aw:
    // FL/FR "突然变矮 再突然变高" was front_drop applying/releasing as a
    // step; "类似调整pitch一样的瞬间响应" was phase_gate jumping 0->1 on
    // COMPLETE entry, letting body-PID dtheta hit the leg as an impulse.
    {
        bool front_all_complete = (climbing_.getPhase(0) == LegClimbPhase::COMPLETE &&
                                   climbing_.getPhase(1) == LegClimbPhase::COMPLETE);
        bool back_all_complete  = (climbing_.getPhase(2) == LegClimbPhase::COMPLETE &&
                                   climbing_.getPhase(3) == LegClimbPhase::COMPLETE);
        const float dt_x   = 0.002f;
        const float ramp_x = (dbg_ctrl.climb_dtheta_ramp_s > 1e-3f) ? dbg_ctrl.climb_dtheta_ramp_s : 1e-3f;
        for (int j = 0; j < 4; j++)
        {
            LegClimbPhase ph_j = climbing_.getPhase(j);

            // Phase-complete authority: target=1 when leg in COMPLETE, else 0.
            float pc_target = (ph_j == LegClimbPhase::COMPLETE) ? 1.0f : 0.0f;
            if (fabsf(pc_target - phase_complete_authority_target_[j]) > 0.01f)
            {
                phase_complete_authority_start_[j]  = phase_complete_authority_[j];
                phase_complete_authority_target_[j] = pc_target;
                phase_complete_authority_t_[j]      = 0.0f;
            }
            phase_complete_authority_t_[j] += dt_x;
            float a_pc = phase_complete_authority_t_[j] / ramp_x;
            if (a_pc > 1.0f) a_pc = 1.0f;
            float s_pc = a_pc * a_pc * (3.0f - 2.0f * a_pc);
            phase_complete_authority_[j] = phase_complete_authority_start_[j] +
                                           (phase_complete_authority_target_[j] - phase_complete_authority_start_[j]) * s_pc;

            // Leg-offset authority: target=1 when the front_drop or
            // back_extend offset should apply. Front (i<2): leg COMPLETE
            // && back not done. Back (i>=2): front both COMPLETE && leg
            // in DETECT/CLIMBING/COMPLETE && back not done.
            float lo_target = 0.0f;
            if (j < 2)
            {
                if (ph_j == LegClimbPhase::COMPLETE && !back_all_complete)
                    lo_target = 1.0f;
            }
            else
            {
                bool back_in_active = (ph_j == LegClimbPhase::DETECT ||
                                       ph_j == LegClimbPhase::CLIMBING ||
                                       ph_j == LegClimbPhase::COMPLETE);
                if (front_all_complete && back_in_active && !back_all_complete)
                    lo_target = 1.0f;
            }
            if (fabsf(lo_target - leg_offset_authority_target_[j]) > 0.01f)
            {
                leg_offset_authority_start_[j]  = leg_offset_authority_[j];
                leg_offset_authority_target_[j] = lo_target;
                leg_offset_authority_t_[j]      = 0.0f;
            }
            leg_offset_authority_t_[j] += dt_x;
            float a_lo = leg_offset_authority_t_[j] / ramp_x;
            if (a_lo > 1.0f) a_lo = 1.0f;
            float s_lo = a_lo * a_lo * (3.0f - 2.0f * a_lo);
            leg_offset_authority_[j] = leg_offset_authority_start_[j] +
                                       (leg_offset_authority_target_[j] - leg_offset_authority_start_[j]) * s_lo;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if (!legs[i])
            continue;

        if (manual_climb)
        {
            // --- Trigger debug: direct angle command, verify climbing direction ---
            // climb_sign[i] picks the physically-correct ±θ branch per leg.
            float angle_cmd_dbg            = climb_sign[i] * trigger_mag[i];
            float ffw                      = legs[i]->Get_LegGravityTorque();
            constexpr float MANUAL_TAU_MAX = 30.0f;
            legs[i]->Set_Leg_PD_Torque(angle_cmd_dbg, 0.0f, kp_use, kd_use, ffw, MANUAL_TAU_MAX);
            dbg_angle_cmd[i] = angle_cmd_dbg;
        }
        else if (climbing_.isDirectControl(i))
        {
            // --- Unified computed-torque control for ALL climbing phases ---
            // One continuous law (no velocity<->position switch, no motor Pos_KP).
            // Wheel_Leg::Set_Leg_Torque_Track computes gravity + software PD and
            // sends it as FFW torque only, tracking the target in the continuous
            // (unwrapped) frame so the leg never unwinds across the +/-180 seam.
            //
            // Climbing_Dynamics gives the unsigned trajectory; climb_sign[i]
            // picks the physical bending solution (and direction of motion).
            float theta_unsigned = climbing_.getTargetThetaDeg(i);
            float omega_unsigned = climbing_.getTargetOmega(i);  // rad/s, neg when θ decreasing
            float ffw_omega_rad  = climb_sign[i] * omega_unsigned;

            // Pitch leveling for direct-control legs: PID output (m) -> leg
            // angle (deg) via a CONSTANT gain dH/r, no 1/sin(theta) kinematic
            // transform.
            //
            // Why no 1/sin: dtheta = dH / (r*sin(theta)) is the kinematically
            // exact "how much do I rotate the leg to deliver dH chassis-height
            // change", but it diverges as theta -> ±180 (sin(165°)=0.26,
            // sin(170°)=0.17, ...). At PREP angles the leg has REDUCED authority
            // to correct chassis tilt -- that's geometric truth, not a bug. The
            // 1/sin transform tells the leg "rotate 250 deg to deliver the 8 cm
            // the PID asked for", and the saturating ±50 deg clamp then traps
            // the loop in an asymmetric relaxation oscillator against the
            // new_theta [15, 165] cap: PID demands +dtheta -> clamped to 165 ->
            // no effect; PID demands -dtheta -> leg drops 50 deg -> overshoots
            // -> repeat. That's "climb to PREP 抖个不停".
            //
            // Principled fix: accept the geometric truth. Command angle
            // adjustment PROPORTIONAL to the PID's height output via the
            // single calibration constant 1/r (= the same gain COMFORT
            // happens to deliver at theta=90 where sin=1). lev_scale and
            // BODY_PID_SCALE_PITCH match the effective authority used in the
            // non-direct branch and in COMFORT respectively, so leveling
            // behavior is consistent across modes. The 10 deg final clamp is
            // a safety bound well below the 15 deg new_theta-clamp headroom
            // (no saturation against the boundary).
            // PHASE GATE for PID dtheta:
            // PREP / DETECT: NO body-PID dtheta. Legs MUST sit cleanly at the
            //   smoothstep target with zero PID interference, so (a) PREP->DETECT
            //   transition criterion (|fb - prep_motor_angle| < 3 deg) is reachable,
            //   and (b) DETECT can read true torque-residual disturbance without
            //   PID-induced motion creating false spikes.
            // CLIMBING / COMPLETE: PID dtheta active for active body leveling
            //   during/after the kinematic climb.
            // Without this gate, PID dtheta of -5 deg on BL/BR during DETECT
            // generates continuous Kp*err output -> false residual spikes ->
            // BL/BR false-trigger DETECT->CLIMBING before FL/FR actually contact.
            // A4ae: PID dtheta applied ONLY in COMPLETE phase.
            //   - PREP / DETECT: gated off (clean lift + clean detection).
            //   - CLIMBING: gated off too -- the trajectory IS the leg
            //     command during climb; adding a +10deg nose-up dtheta
            //     would FIGHT the climbing motion (climb drives theta DOWN
            //     165->62, pitch dtheta pushes it back UP). So no pitch
            //     during the active pivot.
            //   - COMPLETE: applied. This is the post-front nose-down
            //     (CoM-forward) phase. Front legs are at ~62deg (end-climb)
            //     where sin(theta) is large (0.88) -> good pitch authority,
            //     and far from the seam -> the larger +/-15deg clamp below
            //     is safe. Lowering the front legs tilts the chassis
            //     nose-down to shift CoM forward for the back climb.
            LegClimbPhase ph_i_gate = climbing_.getPhase(i);
            // A4ax: smoothstep-ramped COMPLETE-phase gate (replaces the
            // binary A4uu gate). authority is 0 outside COMPLETE, ramps
            // 0->1 on COMPLETE entry over climb_dtheta_ramp_s. Eliminates
            // the dtheta impulse that hit the leg as PID went live at
            // CLIMBING->COMPLETE transition ("类似调整pitch一样的瞬间响应").
            float phase_gate = phase_complete_authority_[i];
            // A4aw: multiply FL/FR's gate by the smoothstep authority that
            // ramps 1->0 while BL/BR is in CLIMBING (kills PID-driven jerk
            // during the dynamic back-climb window). BL/BR's gate is not
            // affected: their CLIMBING/DETECT phase already zeros phase_gate
            // via the phase_complete_authority above.
            if (i < 2)
                phase_gate *= climb_dtheta_authority_;
            float dh = lev_scale * BODY_PID_SCALE_PITCH * lev_signs[i][0] * pitch_h_adj * phase_gate;
            float dtheta_deg = dh / r_m * (180.0f / 3.14159265f);
            // Clamp +/- 5 deg: well below the 15 deg new_theta-boundary
            // headroom on both sides, so the PID-driven dtheta can never
            // hit the [15, 165] clamp. This prevents the asymmetric-
            // saturation relaxation oscillator (one polarity clamped to 0
            // effect, the other polarity full +/-5 deg) that drove the
            // previous shake. With Kp=80 in the flat PD, 5 deg position
            // error -> Kp*err = 80 * 0.087 = ~7 Nm of correction torque,
            // an order of magnitude smaller than the gravity FFW (~15 Nm
            // at theta=90) -- so PID leveling is a true MICRO-adjustment
            // on top of FFW's static balance, not a competing controller.
            // A4ae: clamp raised +/-5 -> +/-15 deg. Now that dtheta only
            // applies in COMPLETE (legs at ~62deg, far from seam), a larger
            // swing is safe (new_theta clamp [15,165] still catches it) and
            // gives meaningful pitch authority: at 62deg, 15deg of front-leg
            // lowering -> ~15mm chassis drop -> ~3deg nose-down (vs the ~1deg
            // the old +/-5 clamp produced -- user saw "+-1deg basically no
            // change"). Tune via the dtheta clamp if more/less tilt wanted.
            if (dtheta_deg > 15.0f)
                dtheta_deg = 15.0f;
            if (dtheta_deg < -15.0f)
                dtheta_deg = -15.0f;
            float new_theta = theta_unsigned + dtheta_deg;
            // Clamp the commanded motor angle to [prep_margin, 180-prep_margin]
            // = [15, 165] by default. Two things this guards against:
            //   1) NearestEquivalentTarget wrap-flip near the +/-180 seam (tiny
            //      HOMING residual in pos_cont could otherwise flip the target
            //      to the opposite hemisphere and send the leg the LONG way).
            //   2) Position-loop OVERSHOOT crossing +/-180. The PD is somewhat
            //      underdamped, so the leg overshoots the commanded angle by
            //      ~10-20 deg. Keeping the target at most 15 deg from the seam
            //      means even a 15 deg overshoot lands at <180 and the leg
            //      settles cleanly at the target without ever crossing.
            // Leveling can push theta DOWN from the prep angle (raising the
            // chassis on that leg) but never past the prep pose itself.
            //
            // A4ab: relax upper clamp for back legs after FL/FR COMPLETE so
            // Climbing_Dynamics can drive BL/BR's DETECT hold angle past 165
            // (e.g. to 170-175) via back_detect_hold_offset_deg, achieving
            // a back-lift / CoM-shift before BL/BR's own contact phase. 5 deg
            // margin still keeps the seam clear (overshoot from the leg PD
            // is bounded since DETECT uses Kp=200 -> tight tracking).
            const float prep_margin = climbing_.config().prep_theta_deg;
            float upper_clamp = 180.0f - prep_margin;  // default 165
            {
                bool is_back_local = (i >= 2);
                bool front_all_complete_local =
                    (climbing_.getPhase(0) == LegClimbPhase::COMPLETE &&
                     climbing_.getPhase(1) == LegClimbPhase::COMPLETE);
                if (is_back_local && front_all_complete_local)
                    upper_clamp = 175.0f;  // 5 deg seam margin
            }
            if (new_theta > upper_clamp)
                new_theta = upper_clamp;
            if (new_theta < prep_margin)
                new_theta = prep_margin;

            // A4af: DIRECT front-leg drop to shift CoM forward for the back
            // climb. After FL/FR reach COMPLETE (front wheels on the step),
            // the chassis tends to sit back-heavy ("重心全压在后轮") and the
            // back wheels can't pivot up. Lowering the FRONT legs by a fixed
            // angle drops the chassis front -> nose-down -> CoM shifts
            // forward -> back wheels unloaded. This is the EFFECTIVE
            // mechanism (front legs at ~62deg have strong geometric
            // authority, 1.08 mm/deg) vs. the near-useless back-leg
            // extension at 165deg (0.32 mm/deg). It's a DIRECT, visible
            // angle command (not the weak attenuated body-PID dtheta), so
            // climb_front_drop_deg degrees of front lowering reliably
            // produces the tilt. Only while back legs still need to climb;
            // once back legs COMPLETE too, the drop is released (climb done).
            // A4ax: front_drop now multiplied by leg_offset_authority_[i]
            // (smoothstep 0..1). The previous binary application snapped
            // the leg by climb_front_drop_deg (~15 deg) on COMPLETE entry
            // and again on back-COMPLETE release -- "突然变矮 再突然变高".
            // Authority handles BOTH transitions: ramps 0->1 over
            // climb_dtheta_ramp_s when activation conditions become true,
            // ramps 1->0 over the same window when they become false.
            {
                bool is_front_leg = (i < 2);
                if (is_front_leg)
                {
                    new_theta -= dbg_ctrl.climb_front_drop_deg * leg_offset_authority_[i];
                    if (new_theta < prep_margin)
                        new_theta = prep_margin;
                }
            }
            // A4av (mechanism) + A4ax (smoothing): BL/BR DIRECT angle
            // extend, multiplied by leg_offset_authority_[i] so the
            // offset eases in/out over climb_dtheta_ramp_s (no snap).
            // Sign convention (per A4av):
            //   > 0  motor angle larger -> back leg extends -> back of
            //        chassis rises -> nose-down -> CoM forward -> back
            //        wheels unloaded (less weight, less grip)
            //   < 0  motor angle smaller -> back leg shortens -> back of
            //        chassis drops -> nose-up -> CoM backward -> back
            //        wheels loaded (more weight, more grip)
            // For CLIMBING phase the offset rides on top of the kinematic
            // trajectory -- accept small deviation from the pure beta-
            // pivot path in exchange for the chassis pose adjustment.
            {
                bool is_back_leg = (i >= 2);
                if (is_back_leg)
                {
                    new_theta += dbg_ctrl.climb_back_extend_deg * leg_offset_authority_[i];
                    // Re-clamp against the upper_clamp computed earlier
                    // (175 for back during this window, 5 deg seam margin)
                    // and prep_margin (15 deg).
                    if (new_theta > upper_clamp) new_theta = upper_clamp;
                    if (new_theta < prep_margin) new_theta = prep_margin;
                }
            }
            float angle_cmd = climb_sign[i] * new_theta;

            // Body-mass-aware sin-based gravity FFW. Uses ADAPTIVE load
            // from `getAdaptiveLoadPerLeg()` (impedance-estimator-driven,
            // self-calibrating to actual rider weight). This is the same
            // load source used for the residual computation -- so in steady
            // state, the leg torque feedback exactly cancels with the FFW
            // and the residual is ~0 (clean direct measurement of any
            // disturbance).
            //
            // Critical: use theta_unsigned (smoothstep target) signed by
            // climb_sign, NOT angle_cmd which includes the body-PID dtheta.
            // If FFW followed angle_cmd, PID swings would step-change the
            // gravity FFW (sin(165)=0.26 vs sin(115)=0.91 -> 3.5x torque jump
            // for a 50 deg dtheta swing) -- amplifying the body-PID loop's
            // output into a leg-torque disturbance that re-tilts the chassis.
            float ffw_climb = climb_ffw_load_per_leg * GRAVITY_g * climb_r_m
                              * sinf(deg2rad(climb_sign[i] * theta_unsigned));

            // Flat PD-as-torque (matches the architecture the user described
            // and the rest of the modes -- COMFORT / HOMING_IN/OUT / FREE all
            // use Set_Leg_PD_Torque). Switched from the cascade controller
            // (Set_Leg_Torque_Track, kp_pos -> omega_clamp -> kd_vel) because
            // the cascade's outer-loop bandwidth (~1.3 Hz with kp_pos=8) was
            // close to the body-PID LPF bandwidth (~1.6 Hz), causing resonance
            // when the body-PID's per-leg dtheta acted as a position
            // disturbance into the cascade. The flat PD has a higher natural
            // frequency than the body-PID LPF, no internal velocity-clamp
            // saturation, so the body-PID loop can't excite a resonant mode.
            //
            // Control law:  tau = Kp*(angle_cmd - actual) + Kd*(omega_ff - omega_actual) + ffw
            //   - ffw    : balances rider weight at this leg angle (sin formula)
            //   - Kp*P   : holds position firmly against disturbances
            //   - Kd*V   : damps motion / softly limits speed
            //   - omega_ff: matched smoothstep derivative -> Kd doesn't fight commanded motion
            // All computed in software with NearestEquivalentTarget and motor
            // Pos_KP=Vel_KD=0 -> no multi-turn unwind, no DM internal-loop
            // surprises.
            //
            // PHASE-DEPENDENT Kp/Kd: each climbing phase has different
            // physical demands, so a single stiffness compromises somewhere.
            //
            //   PREP     (40/2): soft smoothstep lift -- avoids "rigid"
            //                    feel at lift start, gentle on passenger.
            //   DETECT  (200/5): RIGID hold against horizontal step force.
            //                    Wheel pressed against step face pushes
            //                    chassis forward -> leg arm rotates back ->
            //                    soft Kp lets leg yield 30 deg before
            //                    generating enough torque to trigger
            //                    threshold. WORSE: detect_settle_omega=0.4
            //                    gates off detection while leg is yielding,
            //                    so the spike that should fire DETECT->
            //                    CLIMBING is suppressed. Kp=200 keeps leg
            //                    within ~2 deg of target -> motor delivers
            //                    full reaction force as torque (not yield)
            //                    -> residual spikes cleanly, leg vel stays
            //                    below settle gate -> detection fires.
            //   CLIMBING (80/4): firm trajectory tracking, mid-stiffness.
            //   COMPLETE (80/4): firm hold at end-of-climb pose.
            float CLIMB_PD_KP, CLIMB_PD_KD;
            switch (climbing_.getPhase(i))
            {
                case LegClimbPhase::PREP:
                    CLIMB_PD_KP = 40.0f;
                    CLIMB_PD_KD = 2.0f;
                    break;
                case LegClimbPhase::DETECT:
                    CLIMB_PD_KP = 200.0f;
                    CLIMB_PD_KD = 5.0f;
                    break;
                case LegClimbPhase::CLIMBING:
                case LegClimbPhase::COMPLETE:
                default:
                    // A4aq: raised 80 -> 200 to match DETECT stiffness. The
                    // wheel motor's reaction torque through the eccentric
                    // coupling was overwhelming Kp=80 (or 120), pushing the
                    // leg past its trajectory target -> dangerous snap-back.
                    // 200 holds the leg firmly on trajectory regardless of
                    // wheel coupling. omega_ff (matched trajectory derivative)
                    // keeps Kd from fighting commanded motion, so Kp=200
                    // doesn't make tracking feel stiff.
                    CLIMB_PD_KP = 200.0f;
                    CLIMB_PD_KD = 5.0f;
                    break;
            }
            constexpr float CLIMB_PD_TAU_MAX = 200.0f;
            legs[i]->Set_Leg_PD_Torque(angle_cmd, ffw_omega_rad, CLIMB_PD_KP, CLIMB_PD_KD, ffw_climb, CLIMB_PD_TAU_MAX);
            dbg_angle_cmd[i] = angle_cmd;

            // Per-leg TARGET diagnostics. All four legs run the IDENTICAL
            // program logic, so theta_unsigned should match across legs and
            // angle_cmd should differ only by climb_sign + a small leveling
            // offset. If any leg diverges here, the bug is upstream of the
            // motor / torque control (Ozone: watch dbg_climb_tgt).
            dbg_climb_tgt.theta_unsigned[i] = theta_unsigned;
            dbg_climb_tgt.angle_cmd[i]      = angle_cmd;
            dbg_climb_tgt.angle_fb[i]       = legs[i]->Get_LegAngleWrapped();
            dbg_climb_tgt.pos_cont[i]       = legs[i]->Get_LegAngleUnwrapped();
            // (TT_* fields are vestigial -- cascade was replaced by flat PD in A4jj.
            //  Kept for HW-debug rollback only; will be removed in A5.)
            dbg_climb_tgt.omega_cmd[i] = legs[i]->Get_TT_OmegaCmd();
            dbg_climb_tgt.omega_fb[i]  = legs[i]->Get_TT_OmegaFb();
            dbg_climb_tgt.tau_total[i] = legs[i]->Get_TT_TauTotal();

            // ---- Plot-friendly aggregated signals (DbgClimbPlot) ----
            dbg_climb_plot.target_theta_deg[i] = angle_cmd;
            dbg_climb_plot.angle_fb_deg[i]     = legs[i]->Get_LegAngleWrapped();
            dbg_climb_plot.dtheta_pid_deg[i]   = dtheta_deg;
            dbg_climb_plot.target_omega_rad[i] = ffw_omega_rad;
            dbg_climb_plot.actual_omega_rad[i] = legs[i]->Get_LegVelocity();
            dbg_climb_plot.ffw_grav_nm[i]      = ffw_climb;
            dbg_climb_plot.tau_fb_nm[i]        = legs[i]->Get_LegTorqueFeedback();
            dbg_climb_plot.residual_nm[i]      = tres[i];  // = tau_fb - grav_at_angle_fb
        }
        else
        {
            // Normal height pipeline (COMFORT-like, or manual trigger)
            legs[i]->Set_Leg_Height(clampHeight(h_targets[i]), v_targets[i], kp_use, kd_use, legs[i]->Get_LegGravityTorque());
            // Show actual motor feedback for non-climbing legs (so BL/BR don't show 0)
            dbg_angle_cmd[i] = legs[i]->Get_LegAngleWrapped();
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

    // ---- DbgClimbPlot: chassis-wide signals + per-leg phase/beta/baseline ----
    for (int k = 0; k < 4; k++)
    {
        LegClimbPhase ph_k                     = climbing_.getPhase(k);
        dbg_climb_plot.phase[k]                = static_cast<uint8_t>(ph_k);
        dbg_climb_plot.beta_rad[k]             = climbing_.getBeta(k);
        dbg_climb_plot.wheel_blocked[k]        = fb[k].wheel_blocked ? 1 : 0;
        dbg_climb_plot.residual_baseline_nm[k] = climbing_.getBaseline(k);
        // Threshold-compare deviation (matches the value Climbing_Dynamics tests).
        float dev_k                            = fabsf(dbg_climb_plot.residual_nm[k] - climbing_.getBaseline(k));
        dbg_climb_plot.residual_dev_nm[k]      = dev_k;

        // Peak hold: reset on phase change, then accumulate max.
        if (ph_k != residual_peak_last_phase_[k])
        {
            residual_peak_[k]              = 0.0f;
            residual_peak_last_phase_[k]   = ph_k;
        }
        if (dev_k > residual_peak_[k])
            residual_peak_[k] = dev_k;
        dbg_climb_plot.residual_peak_nm[k] = residual_peak_[k];
    }
    dbg_climb_plot.torque_res_threshold_nm      = climbing_.config().torque_res_threshold;
    dbg_climb_plot.torque_res_threshold_back_nm = climbing_.config().torque_res_threshold_back;
    dbg_climb_plot.beta0_rad              = climbing_.getBeta0();
    // pitch_setpoint_raw_deg / pitch_setpoint_filt_deg already populated
    // earlier in A4tt phase-bias block; only fill the remaining IMU + PID
    // state here.
    dbg_climb_plot.pitch_fb_deg           = chassis_pitch_;
    dbg_climb_plot.pitch_h_adj_m          = pitch_h_adj;
    // PREP smoothstep progress for plot (recomputed standalone since the
    // per-leg ramp_progress variable was removed when A4uu gated PID dtheta
    // by phase). Same value across all 4 legs (synchronized smoothstep).
    {
        float theta_unsigned_fl = climbing_.getTargetThetaDeg(0);
        float prep_full_fl      = 180.0f - climbing_.config().prep_theta_deg;
        dbg_climb_plot.prep_ramp_progress = (prep_full_fl > 1.0f)
                                            ? fminf(theta_unsigned_fl / prep_full_fl, 1.0f)
                                            : 1.0f;
    }
    dbg_climb_plot.M_est_climb_kg      = impedance_.getEstimatedMass();
    dbg_climb_plot.ffw_load_per_leg_kg = climb_ffw_load_per_leg;

    // A4yy: back-settle countdown for plot visibility. While > 0, BL/BR are
    // blocked from DETECT->CLIMBING regardless of residual. Watch this drop
    // to 0 ~1 sec after FL/FR enter CLIMBING.
    dbg_climb_plot.back_settle_remaining_s = climbing_.getBackSettleRemaining();

    // PREP-phase mass estimator state -- watch in Ozone to see the live
    // measurement converge: sample_count rises during PREP, then once
    // prep_mass_estimated flips to 1, prep_mass_estimate_kg shows the
    // measured sprung mass (~23 empty, ~73 fully loaded with 50 kg rider).
    dbg_climb_plot.prep_mass_estimate_kg = climb_mass_estimate_kg_;
    dbg_climb_plot.prep_mass_estimated   = climb_mass_estimated_ ? 1 : 0;
    for (int k = 0; k < 4; k++)
        dbg_climb_plot.prep_mass_sample_count[k] = static_cast<uint16_t>(prep_mass_count_[k]);

    // Wheel velocity + execute. Old logic auto-boosted wheel rpm by
    // `climb_omega * climb_wheel_scale` whenever any leg was in CLIMBING or
    // COMPLETE -- a band-aid for under-powered old motors that needed extra
    // wheel speed to actually crawl up steps. With the new higher-torque
    // motors that boost is unnecessary AND counterproductive: it makes the
    // chassis suddenly accelerate the moment ONE leg flips to COMPLETE,
    // even with no joystick input. Now wheels follow joystick + FF wheel
    // compensation only (the FF comp from A4gg keeps the wheels rolling in
    // sync with leg motion during PREP/CLIMBING, so the leg doesn't fight
    // the wheel). The natural Kd*V damping on the leg PD takes the motor to
    // whatever output is required -- it's no longer artificially capped.
    dbg_climb.wheel_rpm = 0.0f;

    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);

    // A4ai: AUTO CHASSIS-ADVANCE during CLIMBING.
    //
    // The user reported: BL/BR triggers (phase=CLIMBING) but "爬不动" --
    // back legs run the trajectory but the wheel doesn't climb. Most
    // likely cause: the chassis can't move forward.
    //
    // Physics: for the BACK wheel to pivot up the step, the chassis MUST
    // advance forward (geometric requirement of the pivot trajectory).
    // The chassis advance requires the FRONT wheels (on the step surface)
    // to roll forward. But HT wheel motors BRAKE when commanded 0 RPM --
    // if the user isn't pushing the joystick hard enough during the back
    // climb, the front wheels stay locked, the chassis can't advance, and
    // the back leg motor stalls (commanded to rotate but the chassis is
    // pinned). Result: back climb FAILS even though the trajectory is
    // running. The front climb didn't have this issue because the back
    // wheels were on flat ground and could skid/drag.
    //
    // Fix: drive ALL wheels forward automatically during CLIMBING at the
    // KINEMATIC chassis-advance rate (cos(beta) * phi_w), so the chassis
    // advances independently of user push. phi_w is smoothstep-ramped
    // (A4ww), so the drive ramps in smoothly -- no surge like the old
    // climb_base_rpm (removed in A4ll). climb_advance_scale > 1.0 adds a
    // grip margin so the climbing wheel is also pushed against the step
    // edge to help it climb.
    {
        float climb_adv_rpm = 0.0f;
        for (int k = 0; k < 4; k++)
        {
            if (climbing_.getPhase(k) == LegClimbPhase::CLIMBING)
            {
                float phi_w_k = climbing_.getEffectivePhiW(k);
                float beta_k  = climbing_.getBeta(k);
                // chassis-advance wheel angular velocity = cos(beta) * phi_w
                float adv_k   = fabsf(cosf(beta_k)) * phi_w_k *
                                (60.0f / (2.0f * 3.14159265f)) *
                                dbg_ctrl.climb_advance_scale;
                if (adv_k > climb_adv_rpm)
                    climb_adv_rpm = adv_k;
            }
        }
        vx += climb_adv_rpm;  // add forward drive; cap below limits total
        dbg_climb_plot.climb_advance_rpm = climb_adv_rpm;
    }

    inverseKinematics(vx, 0, wz, wheel_rpms);

    // A4xx: wheel speed cap during CLIMBING.
    //
    // During CLIMBING, the chassis moves forward at v_chassis ~= R *
    // cos(beta) * phi_w (wheel pivoting around step edge). With
    // climb_omega = 0.5 rad/s and R = 0.1 m, peak chassis speed is ~0.05
    // m/s. If user pushes joystick to full (~0.5 m/s), wheel motor is
    // commanded 10x the chassis-matching rate -> wheel slips at the
    // step contact / drags forward, the leg motor sees reaction torque,
    // and CLIMBING tracking degrades.
    //
    // Fix: cap |wheel_rpm| by phi_w-derived chassis-matching rate when
    // any leg is in CLIMBING. phi_w starts at 0 (smoothstep ramp from
    // A4ww), grows to climb_omega over climb_ramp_s -> cap ramps from
    // 0 to ~chassis_max naturally. After CLIMBING, no cap (PREP/DETECT/
    // COMPLETE all return phi_w = 0; the loop below skips them).
    //
    // Wheel RPM for matched rolling on flat ground:
    //   v_wheel_surface = omega_wheel * R = v_chassis
    //   omega_wheel = v_chassis / R = phi_w * cos(beta)
    //   rpm = omega_wheel * 60 / (2*pi)
    // Use cos(beta) ~ 1 for small beta -- worst case is a slight
    // over-cap as beta grows, which still permits chassis-matching
    // motion (margin shrinks but never goes negative since beta < pi/2).
    // SAFETY ceiling on wheel RPM during CLIMBING regardless of how the user
    // tunes climb_omega / climb_wheel_speed_ratio.
    //   A4aq: kept at 40 RPM per user feedback. Safety addressed via
    //   raising CLIMBING Kp to 200 (resists wheel-coupling deflection)
    //   rather than throttling the wheel.
    constexpr float CLIMB_ABSOLUTE_MAX_WHEEL_RPM = 40.0f;
    float climb_max_rpm = 1.0e9f;  // effectively no cap unless overridden below
    for (int i = 0; i < 4; i++)
    {
        float phi_w = climbing_.getEffectivePhiW(i);
        dbg_climb_plot.climb_effective_phi_w[i] = phi_w;
        if (phi_w > 1e-4f)
        {
            // chassis-matching wheel rate * user-tunable multiplier (A4yy):
            // ratio=1.0 strict, 2.0 allow assistive joystick push, etc.
            // Then capped by the absolute safety ceiling above.
            float leg_cap = phi_w * (60.0f / (2.0f * 3.14159265f)) * dbg_ctrl.climb_wheel_speed_ratio;
            if (leg_cap > CLIMB_ABSOLUTE_MAX_WHEEL_RPM)
                leg_cap = CLIMB_ABSOLUTE_MAX_WHEEL_RPM;
            if (leg_cap < climb_max_rpm)
                climb_max_rpm = leg_cap;
        }
    }
    if (climb_max_rpm < 1.0e8f)
    {
        for (int i = 0; i < 4; i++)
        {
            if (wheel_rpms[i] > climb_max_rpm)
                wheel_rpms[i] = climb_max_rpm;
            if (wheel_rpms[i] < -climb_max_rpm)
                wheel_rpms[i] = -climb_max_rpm;
        }
        dbg_climb_plot.climb_max_wheel_rpm = climb_max_rpm;
    }
    else
    {
        dbg_climb_plot.climb_max_wheel_rpm = 0.0f;  // no cap active
    }

    // Wheel compensation gating by climbing phase:
    //
    //   PREP     -> FF wheel comp (rolling on flat ground while leg lifts
    //               chassis; commanded omega from smoothstep, no FB lag).
    //   DETECT   -> NO comp (leg static; target_omega=0 -> comp would be 0
    //               anyway, but explicit exclusion is clearer + robust).
    //   CLIMBING -> NO comp.  Critical: the trajectory model has the wheel
    //               PIVOTING around the step edge E (constraint cos(theta)
    //               = (R+L-h-R*sin(beta))/L), NOT rolling on flat ground.
    //               The wheel-comp formula leg_rpm * (1 + r/R*cos(theta))
    //               assumes flat-ground rolling -- applying it here
    //               commands the wheel to spin at the rolling rate, which
    //               conflicts with the pivot-around-E geometry. The
    //               reaction torque feeds back into the leg motor and
    //               disturbs trajectory tracking.
    //   COMPLETE -> NO comp (leg static, same as DETECT).
    //   non-direct (IDLE, manual_climb) -> FB comp (normal rolling).
    Wheel_Leg *legs_w[4]  = {FL_WheelLegs_, FR_WheelLegs_, BL_WheelLegs_, BR_WheelLegs_};
    for (int i = 0; i < 4; i++)
    {
        if (!legs_w[i])
            continue;
        legs_w[i]->Set_Wheel_Target(wheel_rpms[i]);

        if (manual_climb || !climbing_.isDirectControl(i))
        {
            // IDLE leg (climbing pipeline not active here) or manual trigger:
            // standard FB wheel comp (slow joystick-driven sweep, lag OK).
            legs_w[i]->Add_Wheel_Compensation(legs_w[i]->Wheel_Compensation());
        }
        else if (climbing_.getPhase(i) == LegClimbPhase::PREP)
        {
            // PREP only: wheel rolls on flat ground as leg lifts chassis.
            float omega_ff_wcmp = climb_sign[i] * climbing_.getTargetOmega(i);
            legs_w[i]->Add_Wheel_Compensation(legs_w[i]->Wheel_Compensation(omega_ff_wcmp));
        }
        // DETECT / CLIMBING / COMPLETE: deliberately no wheel comp (see above).
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
