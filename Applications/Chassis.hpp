#pragma once
#include "Climbing_Dynamics.hpp"
#include "Comm_Msg.hpp"
#include "Controller.hpp"
#include "Cust_Types.hpp"
#include "Ground_Contact.hpp"
#include "IMU.hpp"
#include "Impedance_Controller.hpp"
#include "Math.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
using namespace Core::Drivers;

// =========================================================================
// Debug Global Variables — for direct Ozone watch
// =========================================================================

struct DbgIMU
{
    float pitch   = 0.0f;
    float roll    = 0.0f;
    float accel_z = 0.0f;
};

struct DbgLeveling
{
    float h_fl = 0.0f;
    float h_fr = 0.0f;
    float h_bl = 0.0f;
    float h_br = 0.0f;
};

struct DbgClimbing
{
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
    uint8_t phase_fl = 0;
    uint8_t phase_fr = 0;
    uint8_t phase_bl = 0;
    uint8_t phase_br = 0;
    // Torque residual step detection debug
    float tres_fl = 0.0f, tres_fr = 0.0f, tres_bl = 0.0f, tres_br = 0.0f;      // |residual - baseline| (Nm)
    float tbase_fl = 0.0f, tbase_fr = 0.0f, tbase_bl = 0.0f, tbase_br = 0.0f;  // LPF baseline (Nm)
    float raw_fl = 0.0f, raw_fr = 0.0f, raw_bl = 0.0f, raw_br = 0.0f;          // raw torque residual (Nm)
    float target_h = 0.0f;                                                     // Height target being sent to FL (m) — for Ozone debug
    // Climbing kinematic debug
    float beta0    = 0.0f;  // β₀ at DETECT→CLIMBING transition (rad)
    float beta_fl  = 0.0f;  // current β for FL (rad)
    float beta_fr  = 0.0f;
    float beta_bl  = 0.0f;
    float beta_br  = 0.0f;
    float theta_fl = 0.0f;  // final motor angle cmd (deg, with climb_sign/mirror)
    float theta_fr = 0.0f;
    float theta_bl = 0.0f;
    float theta_br = 0.0f;
    // Raw unsigned theta from Climbing_Dynamics (deg, 180=highest, 0=lowest)
    float raw_theta_fl = 0.0f, raw_theta_fr = 0.0f, raw_theta_bl = 0.0f, raw_theta_br = 0.0f;
    float wheel_rpm = 0.0f;  // climbing forward RPM being commanded
};

struct DbgControl
{
    int state_cmd              = -1;
    float step_height_mm       = 100.0f;
    float torque_res_threshold = 4.0f;  // Ozone-tunable: step detection threshold (Nm)
    float climb_omega          = 0.5f;  // Ozone-tunable: climbing trajectory speed (rad/s) -- 1.0->0.5 (A4ww safety)
    float climb_wheel_scale    = 5.0f;  // (DEPRECATED post-A4ll, kept for backward Ozone watch)
    float climb_pitch_bias     = 3.0f;  // (DEPRECATED post-A4tt, use climb_pitch_front_deg)

    // ---- A4tt: phase-aware pitch bias ----
    // Sign convention: chassis_pitch > 0 = nose UP.
    //
    // Front climbing (PREP/DETECT/CLIMBING on FL/FR) -> nose UP shifts CoM
    // backward, unloading the front wheels for easier pivot over step edge.
    //
    // Front COMPLETE + back active (BL/BR in PREP/DETECT/CLIMBING) -> nose
    // DOWN shifts CoM forward; back wheels get lighter so they roll
    // forward into step contact easily, and during their CLIMBING they
    // have less load to lift.
    //
    // All COMPLETE -> 0 (level).
    //
    // LPF smooths the setpoint transitions so passengers feel the chassis
    // ease into each new attitude over ~1.2 s instead of snapping.
    float climb_pitch_front_deg = 15.0f;   // nose-up target during front-active phases (+ = nose UP)
    float climb_pitch_back_deg  = -5.0f;   // nose-down target after front COMPLETE  (- = nose DOWN)
    float climb_pitch_lpf_alpha = 0.004f;  // ~0.32 Hz LPF -- 3 deg step settles in ~1.2 s

    // A4yy: wheel speed cap multiplier during CLIMBING.
    //
    // 1.0 = strict chassis-matching rate (zero slip allowed). With this
    //     cap, joystick can't push the chassis any faster than the leg
    //     kinematic trajectory advances -- which felt too restrictive
    //     ("上不去了") because it blocked the user's natural ability
    //     to push the chassis through the step.
    // 2.0 = allow 2x the chassis-matching rate -- some wheel slip on the
    //     ground at the back/step edge at the front, but provides
    //     assistive forward push so the user can help the climb.
    // 3.0+ = significant slip, jerks the leg motor via wheel coupling.
    //
    // The cap STILL ramps with the climb-entry smoothstep (A4ww), so
    // there's no jerk at CLIMBING entry regardless of this ratio.
    float climb_wheel_speed_ratio = 2.0f;

    // A4ab: BL/BR DETECT hold-angle offset after FL/FR COMPLETE.
    //   > 0 deg: BL/BR target larger (more extension, back of chassis rises
    //            -> nose-down -> CoM forward). Clamped to 175 deg (5 deg
    //            seam margin) by the leg-angle clamp in handleClimbingMode.
    //   < 0 deg: BL/BR target smaller (back legs shorter, back wheels
    //            lifted off ground for clean forward approach on FL/FR).
    //   = 0    : no change from default 165 deg PREP angle.
    // Only takes effect while BL/BR are in DETECT AND FL/FR are both
    // COMPLETE. Live-tunable from Ozone; default 0 (no behavior change).
    float climb_back_lift_offset_deg = 0.0f;
};

// 接地补偿 (warp mode)
struct DbgGroundContact
{
    float warp_error = 0.0f;  // 对角电流差 (A)
    float warp_dh    = 0.0f;  // 补偿量 (m)
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
};

// 阻抗控制器 (variable impedance)
struct DbgImpedance
{
    float kp_fl = 0.0f, kp_fr = 0.0f, kp_bl = 0.0f, kp_br = 0.0f;
    float kd_fl = 0.0f, kd_fr = 0.0f, kd_bl = 0.0f, kd_br = 0.0f;
    float ffw_fl = 0.0f, ffw_fr = 0.0f, ffw_bl = 0.0f, ffw_br = 0.0f;
    float mass_est   = 0.0f;
    float warp_error = 0.0f;
    float vz         = 0.0f;
};

// Torque residual monitor — updated in ALL modes at end of Update()
// Use this in Ozone to observe step-contact torque spikes in any mode.
struct DbgTorque
{
    // Raw torque feedback (Nm)
    float torque_fl = 0.0f, torque_fr = 0.0f, torque_bl = 0.0f, torque_br = 0.0f;
    // Gravity compensation torque (Nm)
    float grav_fl = 0.0f, grav_fr = 0.0f, grav_bl = 0.0f, grav_br = 0.0f;
    // Torque residual (Nm) = torque - gravity  (spikes on step contact)
    float res_fl = 0.0f, res_fr = 0.0f, res_bl = 0.0f, res_br = 0.0f;
    // LPF baseline of residual (Nm) — slow-moving average
    float base_fl = 0.0f, base_fr = 0.0f, base_bl = 0.0f, base_br = 0.0f;
    // Deviation = |residual - baseline| (Nm) — this is the step-detection signal
    float dev_fl = 0.0f, dev_fr = 0.0f, dev_bl = 0.0f, dev_br = 0.0f;
};

// Leg angle monitor — updated in ALL modes at end of Update()
struct DbgLeg
{
    float cmd_fl = 0.0f, cmd_fr = 0.0f, cmd_bl = 0.0f, cmd_br = 0.0f;  // Final motor command (deg, post-slew)
    float fb_fl = 0.0f, fb_fr = 0.0f, fb_bl = 0.0f, fb_br = 0.0f;      // Motor feedback (deg)
};

// Wheel motor health monitor — updated in ALL modes
struct DbgWheel
{
    float out_fl = 0.0f, out_fr = 0.0f, out_bl = 0.0f, out_br = 0.0f;      // Motor output command
    float cur_fl = 0.0f, cur_fr = 0.0f, cur_bl = 0.0f, cur_br = 0.0f;      // Current feedback (A)
    float rpm_fl = 0.0f, rpm_fr = 0.0f, rpm_bl = 0.0f, rpm_br = 0.0f;      // RPM feedback
    float temp_fl = 0.0f, temp_fr = 0.0f, temp_bl = 0.0f, temp_br = 0.0f;  // Temperature (°C)
    // Torque utilization: |output| / 16000 (0~1, 1=saturated)
    float util_fl = 0.0f, util_fr = 0.0f, util_bl = 0.0f, util_br = 0.0f;
    // Target RPM (final, after compensation)
    float tgt_fl = 0.0f, tgt_fr = 0.0f, tgt_bl = 0.0f, tgt_br = 0.0f;
    // RPM error = target - actual (positive = stalling, negative = spinning free/slipping)
    float err_fl = 0.0f, err_fr = 0.0f, err_bl = 0.0f, err_br = 0.0f;
};

// Per-leg TARGET diagnostics for climbing — focus on "what target is the
// program giving each leg, and what does the leg actually read?". Hardware is
// identical across the four legs, so all four entries should match each other
// up to a sign flip from climb_sign[]. If a single leg diverges here, the bug
// is upstream in the target-computation, not in the motor or torque control.
// Index FL=0, FR=1, BL=2, BR=3.
struct DbgClimbTarget
{
    float theta_unsigned[4] = {0, 0, 0, 0};  // deg, Climbing_Dynamics output BEFORE climb_sign/leveling
    float angle_cmd[4]      = {0, 0, 0, 0};  // deg, FINAL target after climb_sign + pitch leveling
    float angle_fb[4]       = {0, 0, 0, 0};  // deg, Get_LegAngleWrapped (single-turn feedback)
    float pos_cont[4]       = {0, 0, 0, 0};  // deg, Get_LegAngleUnwrapped (multi-turn feedback)
    // TEMPORARY (FL-slow diagnosis): if tau_total[FL] ~= tau_total[FR] but
    // omega_fb[FL] tracks omega_cmd worse than FR -> hardware/mech. If
    // tau_total[FL] < tau_total[FR] -> software bug upstream of motor.
    float omega_cmd[4] = {0, 0, 0, 0};  // rad/s, velocity setpoint after omega_max clamp
    float omega_fb[4]  = {0, 0, 0, 0};  // rad/s, filtered leg velocity feedback
    float tau_total[4] = {0, 0, 0, 0};  // Nm, final torque sent to motor (gravity + tau_fb)
};

// =====================================================================
// Climbing plot-friendly aggregated signals
// =====================================================================
// One struct, watch in Ozone's Sampling viewer to plot anything you need
// during a climb. Per-leg signals are indexed [FL=0, FR=1, BL=2, BR=3];
// scalars are chassis-frame.
//
// Recommended Ozone plots:
//   Trajectory  : target_theta_deg[i] vs angle_fb_deg[i]
//   Velocity    : target_omega_rad[i] vs actual_omega_rad[i]
//   Disturbance : residual_nm[i], residual_dev_nm[i] (compare vs threshold)
//   Body PID    : pitch_setpoint_filt_deg, pitch_fb_deg, pitch_h_adj_m
//   Mass adapt  : M_est_climb_kg over time
//   Phases      : phase[i] (steps 0=IDLE 1=PREP 2=DETECT 3=CLIMBING 4=COMPLETE)
//   FFW health  : ffw_grav_nm[i] vs tau_fb_nm[i] (should overlap in steady)
struct DbgClimbPlot
{
    // Per-leg phase (LegClimbPhase enum value: IDLE=0, PREP=1, DETECT=2, CLIMBING=3, COMPLETE=4)
    uint8_t phase[4] = {0, 0, 0, 0};

    // Per-leg target/feedback angles (deg, signed motor frame)
    float target_theta_deg[4] = {0, 0, 0, 0};  // smoothstep / trajectory + PID dtheta + climb_sign
    float angle_fb_deg[4]     = {0, 0, 0, 0};  // motor encoder feedback
    float dtheta_pid_deg[4]   = {0, 0, 0, 0};  // body-PID per-leg leveling correction (deg)

    // Per-leg velocity (rad/s, signed motor frame)
    float target_omega_rad[4] = {0, 0, 0, 0};
    float actual_omega_rad[4] = {0, 0, 0, 0};

    // Per-leg torque (Nm)
    float ffw_grav_nm[4]          = {0, 0, 0, 0};  // adaptive gravity FFW commanded
    float tau_fb_nm[4]            = {0, 0, 0, 0};  // motor torque feedback
    float residual_nm[4]          = {0, 0, 0, 0};  // tau_fb - ffw_grav (direct disturbance)
    float residual_baseline_nm[4] = {0, 0, 0, 0};  // LPF baseline (slow drift)
    float residual_dev_nm[4]      = {0, 0, 0, 0};  // |residual - baseline| (threshold compare)

    // Per-leg trajectory (rad)
    float beta_rad[4] = {0, 0, 0, 0};  // beta (rotation around step edge)
    float beta0_rad   = 0.0f;          // beta_0 at DETECT->CLIMBING

    // Chassis / body PID
    float pitch_setpoint_raw_deg  = 0.0f;  // per-phase target before LPF
    float pitch_setpoint_filt_deg = 0.0f;  // after LPF (what PID sees)
    float pitch_fb_deg            = 0.0f;  // chassis_pitch_ after IMU LPF
    float pitch_h_adj_m           = 0.0f;  // body PID output (m)

    // PREP smoothstep progress
    float prep_ramp_progress = 0.0f;  // [0..1]

    // Adaptive gravity (key indicator for "is grav comp matching reality")
    float M_est_climb_kg      = 0.0f;  // mass currently used in ffw_grav (kg)
    float ffw_load_per_leg_kg = 0.0f;  // M_est/4 + LEG_MASS (kg)

    // CLIMBING wheel-speed cap (A4xx): cap |wheel_rpm| during active CLIMBING
    // so user joystick doesn't outrun the chassis kinematic forward velocity.
    float climb_max_wheel_rpm       = 0.0f;  // current per-tick cap (0 = no cap active)
    float climb_effective_phi_w[4]  = {0, 0, 0, 0};  // per-leg phi_w currently used (rad/s)

    // Back-leg settle countdown (A4yy): seconds remaining before BL/BR can
    // trigger DETECT->CLIMBING after FL/FR first entered CLIMBING. While > 0,
    // BL/BR detection is gated off to let the deceleration impulse dissipate.
    float back_settle_remaining_s   = 0.0f;

    // PREP-phase mass estimator state
    float prep_mass_estimate_kg        = 0.0f;          // M (sprung) measured during PREP (kg)
    uint8_t prep_mass_estimated        = 0;             // 1 once estimator has finalized this climb session
    uint16_t prep_mass_sample_count[4] = {0, 0, 0, 0};  // per-leg sample count in high-sin window

    // Residual peak hold (for threshold tuning).
    //
    // Tracks max(residual_dev_nm[i]) since the last per-leg phase transition.
    // Pattern of use:
    //   1. Enter CLIMBING, press X -> PREP -> DETECT.
    //   2. While in DETECT (no step contact yet), residual_peak_nm[i] shows
    //      the NOISE FLOOR -- highest disturbance from sensor/motor noise.
    //      Typical: 0.5 - 1.5 Nm.
    //   3. Push joystick forward, wheel hits step -> residual_dev_nm spikes.
    //      residual_peak_nm[i] captures the max spike value (e.g. 5 - 10 Nm).
    //   4. Set torque_res_threshold ~= 0.5 * (peak - noise_floor) + noise_floor
    //      e.g. peak=8, floor=1 -> threshold = 4.5 Nm. Comfortable SNR.
    // Peak resets every time the leg changes phase, so you read a fresh value
    // for each segment (PREP / DETECT / CLIMBING / COMPLETE).
    float residual_peak_nm[4]     = {0, 0, 0, 0};
    float torque_res_threshold_nm = 0.0f;  // current threshold (mirrored from dbg_ctrl for plot overlay)
};

// Aggregated summary — one struct to watch in Ozone for the most useful signals
// across all modes (motor angles, leg heights, IMU state, mass estimate, target).
struct DbgSummary
{
    // Motor angle feedback (deg, normalized to [-180, 180])
    float angle_fl = 0.0f, angle_fr = 0.0f, angle_bl = 0.0f, angle_br = 0.0f;

    // Per-leg current height (m), computed from motor angle via
    //   H = R - r * cos(theta_motor)
    float height_fl = 0.0f, height_fr = 0.0f, height_bl = 0.0f, height_br = 0.0f;
    float height_avg = 0.0f;  // average of the four legs

    // Target chassis height (m) — what Set_Leg_Height is being asked for
    float target_height = 0.0f;

    // IMU state (chassis frame, after mounting transform + level trim)
    float imu_pitch      = 0.0f;  // deg, nose-up positive
    float imu_roll       = 0.0f;  // deg, right-up positive
    float imu_pitch_rate = 0.0f;  // deg/s (gyro)
    float imu_roll_rate  = 0.0f;
    float imu_accel_z    = 0.0f;  // m/s², earth frame Z-up, gravity removed

    // Sprung-mass / impedance estimator
    float mass_est = 0.0f;  // kg

    // Mode (mirrors dbg_state) for one-stop visibility
    uint8_t state = 0;
};

extern DbgIMU dbg_imu;
extern DbgLeveling dbg_leveling;
extern DbgClimbing dbg_climb;
extern DbgControl dbg_ctrl;
extern DbgGroundContact dbg_gc;
extern DbgImpedance dbg_imp;
extern DbgTorque dbg_torque;
extern DbgLeg dbg_leg;
extern DbgWheel dbg_wheel;
extern DbgClimbTarget dbg_climb_tgt;
extern DbgClimbPlot dbg_climb_plot;
extern DbgSummary dbg_summary;
// Values: 0=CALIBRATION,1=IDLE,2=ENERGY_SAVING,3=COMFORT,4=CLIMBING,5=FREE_CONTROL,6=DEBUG,7=ERROR
extern volatile uint8_t dbg_state;

// =========================================================================

class Chassis
{
   private:
    Chassis_State current_state_ = Chassis_State::CALIBRATION;
    Controller controller_;
    Wheel_Leg *FL_WheelLegs_;
    Wheel_Leg *FR_WheelLegs_;
    Wheel_Leg *BL_WheelLegs_;
    Wheel_Leg *BR_WheelLegs_;

    // Precomputed geometry (meters), init in constructor
    float R_m_;            // wheel radius
    float r_m_;            // eccentric offset (leg length)
    float wb_m_;           // wheelbase
    float wt_f_m_;         // front track width
    float max_pitch_deg_;  // max PID input clamp for pitch
    float max_roll_deg_;   // max PID input clamp for roll
    float h_max_;          // height upper bound
    float h_min_;          // height lower bound

    float target_chassis_height_                 = 0.0f;     // Filtered (slew-rate-limited) height in meters — what executors consume
    float target_height_setpoint_                = 0.0f;     // Step setpoint written by buttons / modes; slewed into target_chassis_height_
    float height_slew_rate_                      = 0.0f;     // m/s; current slew speed (signed), used as Kd feedforward for legs
    static constexpr float HEIGHT_SLEW_PER_CYCLE = 0.0004f;  // 0.0004 m/cycle × 500 Hz = 0.2 m/s ramp

    // todo3: ES → COMFORT 软启动窗口。Set_Mode 在 ES → COMFORT 切换瞬间把
    // 这个计数器置为 ES2COMFORT_LIMIT_TICKS（窗口长度），slewTargetHeight
    // 在每个 tick 内消费（递减）一次；窗口内上升速率被压低到
    // ES2COMFORT_MAX_H_DOT 以下，窗口外恢复正常 HEIGHT_SLEW_PER_CYCLE。
    int es_to_comfort_limit_ticks_ = 0;

    // Ground Contact Warp Compensator (COMFORT / CLIMBING modes)
    GroundContact ground_contact_;

    // Impedance Controller (COMFORT mode — alternative to position-based suspension)
    Impedance_Controller impedance_;

    // Climbing Dynamics Controller (CLIMBING mode)
    Climbing_Dynamics climbing_;

    // IMU-derived values (updated each cycle by readAndTransformIMU)
    float chassis_pitch_      = 0.0f;  // deg
    float chassis_roll_       = 0.0f;  // deg
    float chassis_accel_z_    = 0.0f;  // m/s² (gravity removed)
    float chassis_pitch_rate_ = 0.0f;  // rad/s
    float chassis_roll_rate_  = 0.0f;  // rad/s

    uint8_t last_button_status_ = 0;

    // Mode transition: smooth Kp ramp when leaving COMFORT
    static constexpr int TRANSITION_FRAMES = 200;  // 0.4s @500Hz
    int mode_transition_timer_             = 0;
    float exit_kp_[4]                      = {35.0f, 35.0f, 35.0f, 35.0f};  // Last impedance Kp per leg
    float exit_kd_[4]                      = {1.5f, 1.5f, 1.5f, 1.5f};      // Last impedance Kd per leg

    // ---- COMFORT internal sub-state machine ----
    // HOMING: drive each leg from whatever pose ENERGY_SAVING (or any prior
    //   mode) left it in to a safe θ ∈ [+/-]COMFORT_HOMING_THETA, away from
    //   the kinematic singularity at θ=0,π. Smoothly hands off Kp from the
    //   previous mode's stiffness to COMFORT_HOMING_KP_END, so legs never
    //   get released. Wheels held at 0 RPM.
    // RUN: normal impedance + body-leveling control.
    enum class ComfortPhase : uint8_t
    {
        HOMING = 0,
        RUN    = 1
    };
    ComfortPhase comfort_phase_                = ComfortPhase::HOMING;
    int comfort_homing_ticks_                  = 0;
    float comfort_homing_kp_start_[4]          = {60.0f, 60.0f, 60.0f, 60.0f};
    float comfort_homing_kd_start_[4]          = {2.5f, 2.5f, 2.5f, 2.5f};
    static constexpr int COMFORT_HOMING_FRAMES = 500;  // 1.0s @500Hz — gentler lift, kills ES->COMFORT 顿挫
    // After the lift smoothstep finishes, hold at target with the same fixed
    // PD (no impedance, no body PID) for this many ticks so the leg fully
    // settles BEFORE the impedance/leveling layer engages. Eliminates the
    // "阶段性" feel where the lift transition into impedance was perceptible.
    static constexpr int COMFORT_HOLD_FRAMES     = 250;    // 0.5s @500Hz
    static constexpr float COMFORT_HOMING_THETA  = 90.0f;  // deg — middle of workspace (sin²=1)
    static constexpr float COMFORT_HOMING_KP_END = 30.0f;  // Final Kp at end of homing
    static constexpr float COMFORT_HOMING_KD_END = 2.0f;
    // Sin-based gravity-FFW during HOMING uses this chassis mass guess so the
    // static equilibrium during HOMING matches the equilibrium RUN converges
    // to after impedance ramps in. Without it, sag during HOMING (FFW=0,
    // Kp=30) is ~10° → leg lands at 81° instead of 90°, then RUN's FFW pushes
    // the leg the remaining 9° → visible "second-stage" climb. The same value
    // is also seeded into Impedance_Controller::seedMass at handoff to skip
    // the mass-warmed snap.
    //
    // Semantics match Impedance_Controller's M: SPRUNG mass only (total robot
    // minus the four legs, which are unsprung at the wheel end). Derive from
    // Robot_Params so this tracks edits to ROBOT_MASS_kg / LEG_MASS_kg
    // automatically. With ROBOT_MASS=39, LEG_MASS=4, RIDER=50 → 73 kg.
    //
    // Includes the expected RIDER_MASS_kg because the dominant use case is
    // ridden — guessing the loaded value avoids ~6–7° sag at HOMING when a
    // rider is on. Running empty causes mild overshoot (~2°) that the on-line
    // estimator corrects within ~1 s of entering RUN; either way the visible
    // second-stage motion is gone.
    // Mass seed used for the open-loop FFW during COMFORT HOMING and as the
    // initial value handed to Impedance_Controller::seedMass() at the
    // HOMING->RUN handoff. SPRUNG mass only (chassis minus the four legs).
    //
    // Sized for the LOADED case (chassis + rider) because under-seeding the
    // loaded case starves FFW and the legs cannot lift the rider -- observed
    // as "COMFORT cannot reach target height, lacks force, wobbles". The
    // unloaded case oscillates with this seed (M_est LPF takes ~1 s to drop
    // to actual ~10 kg) -- that is a separate bug to be fixed differently
    // (faster early LPF / load detection), NOT by under-seeding here.
    static constexpr float COMFORT_HOMING_CHASSIS_MASS_GUESS = (ROBOT_MASS_kg - 4.0f * LEG_MASS_kg) + RIDER_MASS_kg;

    // ---- ENERGY_SAVING internal sub-state machine ----
    // HOMING: drive each leg from whatever pose the previous mode left it in
    //   back to motor-frame θ=0 (the folded ES stance). Uses a smoothstep
    //   angle ramp + simultaneous Kp/Kd ramp from the previous mode's
    //   stiffness to the ES hold values. During HOMING we DO NOT add
    //   Wheel_Compensation: the decoupling FF is proportional to leg_rpm
    //   and at homing speeds it would drive the wheels noticeably (front
    //   and back axles in opposite directions) — that was the "rear wheels
    //   creeping forward / front legs flipping back" symptom.
    // RUN: normal ENERGY_SAVING — wheels follow joystick, legs hold θ=0.
    enum class EnergyPhase : uint8_t
    {
        HOMING = 0,
        RUN    = 1
    };
    // Per-leg snapshot at COMFORT->ES transition: captures the motor torque
    // and sin(theta) each leg was holding when COMFORT exited. During ES
    // descent the ffw is then `tau_entry * sin_now / sin_entry` -- i.e. the
    // ACTUAL per-leg load scaled to the current pose. Cancels per-leg gravity
    // exactly, so descent velocity is set by Kd*(omega_cmd - omega_fb) alone
    // and all four legs descend in lockstep regardless of weight distribution
    // (rider forward/back/centred).
    float es_entry_tau_[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
    float es_entry_sin_[4]  = {0.0f, 0.0f, 0.0f, 0.0f};
    bool es_snapshot_valid_ = false;

    EnergyPhase energy_phase_                   = EnergyPhase::RUN;
    int energy_homing_ticks_                    = 0;
    float energy_homing_kp_start_[4]            = {0.0f, 0.0f, 0.0f, 0.0f};
    float energy_homing_kd_start_[4]            = {0.5f, 0.5f, 0.5f, 0.5f};
    float energy_homing_start_angle_[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr int ENERGY_HOMING_FRAMES   = 500;  // 1.0s @500Hz — symmetric with COMFORT_HOMING_FRAMES (the ES->COMFORT lift)
    static constexpr float ENERGY_HOMING_KP_END = 80.0f;
    static constexpr float ENERGY_HOMING_KD_END = 4.0f;

    // ---- CLIMBING internal sub-state machine ----
    // Wraps the leg-level Climbing_Dynamics state machine with two zero-pose
    // stages so leg motors NEVER need to cross the ±π wrap boundary during
    // mode entry/exit (DM motors accumulate multi-turn internally even with
    // P_MAX=π, so position-control commands across the wrap would unwind
    // accumulated turns on the next mode switch — see Chassis.cpp notes).
    //
    // HOMING_IN  : smoothstep all legs from entry angle → motor-frame 0°.
    //              No climbing logic active. Wait at 0° for user to start.
    // WAIT_START : hold all legs at 0°. Press X to enter ACTIVE.
    // ACTIVE     : normal climbing pipeline (PREP/DETECT/CLIMBING/COMPLETE).
    // HOMING_OUT : user pressed a mode-switch button OR finished climb;
    //              smoothstep legs from current angle → 0° before performing
    //              the actual Chassis_State transition.
    enum class ClimbStage : uint8_t
    {
        HOMING_IN  = 0,
        WAIT_START = 1,
        ACTIVE     = 2,
        HOMING_OUT = 3
    };
    ClimbStage climb_stage_            = ClimbStage::HOMING_IN;
    int climb_homing_ticks_            = 0;
    float climb_homing_start_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // Per-leg entry stiffness captured on Set_Mode(CLIMBING) so HOMING_IN
    // can ramp Kp/Kd smoothly from the previous mode's gains (mirrors the
    // ES descent style). HOMING_OUT seeds these with CLIMB_HOMING_KP_END so
    // the stiffness ramp is a no-op during exit (only the angle ramps).
    float climb_homing_kp_start_[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
    float climb_homing_kd_start_[4]         = {0.5f, 0.5f, 0.5f, 0.5f};
    Chassis_State climb_pending_exit_state_ = Chassis_State::IDLE;
    uint8_t climb_last_buttons_             = 0;
    // 1000 ticks = 2.0 s @ 500 Hz. Climb -> other-mode return-to-0 sweep
    // covers a much larger angle (up to ±165° from PREP) than the COMFORT
    // smoothstep does, so it needs more time to feel gentle. Was 400 (0.8 s)
    // which felt too fast to the rider on the way down.
    static constexpr int CLIMB_HOMING_FRAMES   = 1000;
    static constexpr float CLIMB_HOMING_KP_END = 80.0f;
    static constexpr float CLIMB_HOMING_KD_END = 4.0f;

    // ---- PREP-phase mass estimator ----
    // While the legs sweep through theta_motor ~ 90 deg during PREP, sin(theta)
    // is large (~1.0), and the motor torque feedback satisfies the static
    // balance equation tau_motor = (M/4 + LEG_MASS) * g * r * sin(theta) +
    // small_dynamics. Sampling tau_fb / (g*r*sin(theta)) per leg over the
    // high-sin window gives us a direct measurement of "what is on top of
    // the chassis" -- empty vs. rider -- BEFORE we need accurate FFW for
    // DETECT / CLIMBING. This decouples climb-mass calibration from having
    // to enter COMFORT first.
    //
    // Reset in Set_Mode(CLIMBING). Sampled in handleClimbingMode while
    // climbing_.getPhase(i) == PREP and sin(theta_motor) > 0.7. Once each
    // leg has >= MIN samples, finalize M = sum(per_leg) - 4*LEG_MASS and
    // seed the impedance estimator.
    static constexpr float PREP_MASS_SIN_THRESHOLD = 0.7f;          // ~ theta in [44, 136] deg
    static constexpr int PREP_MASS_MIN_SAMPLES     = 50;            // ~ 0.1 s of samples in window
    float prep_mass_load_sum_[4]                   = {0, 0, 0, 0};  // sum of m_per_leg samples (kg)
    int prep_mass_count_[4]                        = {0, 0, 0, 0};
    bool climb_mass_estimated_                     = false;
    float climb_mass_estimate_kg_                  = 0.0f;  // M (sprung) once estimated

    // Residual peak hold per leg (reset on phase change). Used for tuning
    // torque_res_threshold via dbg_climb_plot.residual_peak_nm.
    LegClimbPhase residual_peak_last_phase_[4] = {LegClimbPhase::IDLE, LegClimbPhase::IDLE, LegClimbPhase::IDLE, LegClimbPhase::IDLE};
    float residual_peak_[4]                    = {0.0f, 0.0f, 0.0f, 0.0f};

    /**
     * @brief Adaptive per-leg load mass for gravity FFW.
     *
     * Returns `M_est/4 + LEG_MASS_kg` where `M_est` is the impedance
     * estimator's live sprung-mass estimate (warmed up during COMFORT) --
     * the most accurate "actual rider + chassis" figure available. If
     * `M_est` is suspiciously low (< 10 kg, e.g. cold start before COMFORT
     * was used), falls back to the loaded default
     * `COMFORT_HOMING_CHASSIS_MASS_GUESS/4 + LEG_MASS_kg` so FFW is at
     * least sized for the rider.
     *
     * Used by every mode that needs an accurate gravity comp:
     *   - Climbing ACTIVE direct-control: ffw_climb = load * g * r * sin(theta)
     *   - Climbing torque-residual computation (DETECT step contact)
     *   - All-modes dbg_torque residual baseline
     *
     * Why adaptive: with fixed `LEG_MASS_kg=4`, gravity comp was off by
     * (chassis + rider) ~ 70 kg of "weight on each leg" -- the LPF baseline
     * had to absorb this offset, which made detection sensitive to any
     * mass shift (rider movement, pitch_bias changes, etc.).
     */
    float getAdaptiveLoadPerLeg() const;

   public:
    Chassis() = delete;
    /**
     * @brief Chassis
     * @param fl Front-left Wheel_Leg pointer
     * @param fr Front-right Wheel_Leg pointer
     * @param bl Back-left Wheel_Leg pointer
     * @param br Back-right Wheel_Leg pointer
     */
    Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br);

    //===================//
    //==== User API =====//
    //===================//

    /**
     * @brief Initialize the chassis subsystem
     */
    void Init();

    /**
     * @brief Main update loop
     * @param cmd Command from PC or remote controller
     */
    void Update(const Protocol::PC_Msg &cmd);

    /**
     * @brief Get the Reachable_Msg for feedback
     * @param msg Pointer to the Reachable_Msg to be filled
     */
    void Get_Msg(Protocol::Reachable_Msg *msg);
    //---------------------------------------------------------------------------------------------//

    //======================//
    //==== Internal API ====//
    //======================//
    /**
     * @brief Set the Chassis Mode
     * @param new_state New chassis state
     * */
    void Set_Mode(Chassis_State new_state);

    /**
     * @brief Get the current Chassis Mode
     * @return Current chassis state
     */
    Chassis_State Get_Mode() const { return current_state_; }

    //---------------------------------------------------------------------------------------------//

    //===

   private:
    // --- 各种模式的处理函数 ---
    void handleCalibrationMode();
    void handleEnergySaving(const Protocol::PC_Msg &cmd);
    void handleFreeControl(const Protocol::PC_Msg &cmd);
    void handleDebugMode(const Protocol::PC_Msg &cmd);

    // 舒适模式：核心是主动悬挂算法
    // 输入：IMU数据 (Roll, Pitch, Z-accel)
    // 输出：调整腿的角度和轮子的力矩
    void handleComfortMode(const Protocol::PC_Msg &cmd);

    // 攀爬模式：可能涉及到重心调整或特殊的步态
    void handleClimbingMode(const Protocol::PC_Msg &cmd);

    // ===== 共享管线 (Shared Pipeline) =====
    // 读取IMU并转换到底盘坐标系
    void readAndTransformIMU();
    // 按钮 → 目标高度映射 (可在各模式中覆盖)
    void handleHeightButtons(const Protocol::PC_Msg &cmd);
    // 共享的车身控制管线：自动平衡PID + 高度分配 + 轮速控制
    // mode_dh[4]: 由模式特定算法提供的额外高度补偿 (FL, FR, BL, BR)
    void executeBodyControl(const Protocol::PC_Msg &cmd, const float mode_dh[4]);
    // Impedance variant: uses per-leg Kp/Kd/FFW from Impedance_Controller
    void executeBodyControlImpedance(const Protocol::PC_Msg &cmd);
    void executeMotorCommands();
    void updateWheelDebug();
    // Refresh DbgSummary aggregate (angles, heights, IMU, mass, state).
    // Cheap; safe to call every cycle including during PC disconnect.
    void updateDbgSummary();
    float clampHeight(float h) const { return h < h_min_ ? h_min_ : (h > h_max_ ? h_max_ : h); }
    // Slew-rate-limit target_chassis_height_ toward target_height_setpoint_.
    // Prevents instantaneous height jumps from button presses or mode changes
    // from producing huge MIT command angle steps × high Kp = violent slams.
    void slewTargetHeight();

    // 运动学解算：将底盘整体速度(Vx, Vy, Wz)分解为4个轮子的速度
    void inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms);

    // Helper functions
    float CalculateHeightFromAngle(float angle_deg);
    void SetBendingDirection(int fl, int fr, int bl, int br);
};

}  // namespace Applications