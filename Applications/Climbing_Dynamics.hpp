#pragma once

#include <cmath>
#include <cstdint>

#include "Robot_Params.hpp"

namespace Applications
{

// =====================================================================
// Per-leg climbing state machine phases
// =====================================================================
enum class LegClimbPhase : uint8_t
{
    IDLE = 0,  // Normal operation — standard height pipeline
    PREP,      // Pre-bending leg to prep_theta (avoid θ=0 singularity)
    DETECT,    // At prep angle, monitoring wheel current for step contact
    CLIMBING,  // Active kinematic compensation
    COMPLETE   // Climb finished, returning to IDLE
};

// =====================================================================
// Per-leg sensor feedback for climbing algorithm
// =====================================================================
struct LegClimbFeedback
{
    float leg_pos_deg;          // Current leg angle (deg)
    float wheel_rpm;            // Wheel motor RPM
    float leg_torque_residual;  // Leg torque - gravity comp (Nm) — step detection
    float leg_vel_radps;        // Leg angular velocity (rad/s) — detection settle gate
};

/**
 * @class Climbing_Dynamics
 * @brief Single-wheel step-climbing kinematics with per-leg state machine
 *
 * ============================================================
 * Kinematic Model (constant chassis height Oy = R + L)
 * ============================================================
 *
 * A wheel (radius R) connected via an eccentric leg (length L)
 * climbs a step of height h by rotating around the step edge E.
 *
 *   Constraint:  cos(θ) = (R + L − h − R·sin(β)) / L
 *
 *                √( R² − [R − h + L·(1 − cosθ)]² )
 *   θ̇  =  ───────────────────────────────────────── · φ_w
 *                        L · sin(θ)
 *
 * Start: θ = θ_prep,  β = β₀ = arcsin((R−h)/R)
 * End:   β = 90°,     θ_end = arccos((L−h)/L)
 *
 * Per-leg state machine:
 *   IDLE → PREP → DETECT → CLIMBING → COMPLETE → IDLE
 *
 * Output per-leg:
 *   target_h_  — absolute target height for Set_Leg_Height (m)
 *   target_v_  — velocity feedforward (m/s)
 * ============================================================
 */
class Climbing_Dynamics
{
   public:
    struct Config
    {
        // --- Step geometry ---
        float step_height_m = 0.100f;  // Step height h (m), must be < R

        // --- Prep phase ---
        float prep_theta_deg     = 15.0f;  // Preparatory angle (deg) away from singularity
        float prep_tolerance_deg = 3.0f;   // Angle tolerance for PREP→DETECT transition
        // PREP ramp duration: target_theta_deg smoothsteps from 0 to (180-prep_theta_deg)
        // over this time. Shared by all legs so they LIFT IN LOCKSTEP -- removes the
        // chassis wobble that comes from each leg's PD racing independently to omega_max
        // (real-world friction/load differences make their actual speeds diverge).
        // 3.0 s instead of 1.5 s: rider load asymmetry (heavier side = slower
        // PD response) made the lift visibly desynchronized at 1.5 s (front
        // legs reached target ~0.3 s before back legs -> chassis tilted
        // backward / forward during the lift). Slowing the synchronized
        // smoothstep gives the lagging side time to catch up while the leading
        // side waits at the ramping target, so all four legs reach prep
        // together. Combined with climb_kd_vel bumped 6 -> 12 for tighter
        // velocity tracking.
        float prep_ramp_s = 3.0f;

        // --- Step detection (leg torque residual) ---
        float torque_res_threshold = 4.0f;    // |residual - baseline| to trigger (Nm)
        float baseline_alpha       = 0.02f;   // LPF rate for baseline (~1.6 Hz @ 500 Hz)
        float detect_confirm_s     = 0.020f;  // Confirmation duration (s)
        // Detection is only valid while the leg is (nearly) stationary: the
        // closed-loop drive torque during motion shows up in the torque residual
        // and would false-trigger CLIMBING. Gate on leg speed below this.
        float detect_settle_omega = 0.4f;  // rad/s; arm detection only when |leg vel| < this

        // A4yy: BACK-LEG DETECT post-front-CLIMBING delay.
        //
        // When the robot encounters a step, it necessarily decelerates --
        // front wheels jam at the step face, chassis sees a forward-
        // deceleration impulse. This impulse propagates through chassis
        // dynamics to BL/BR leg motors as a torque transient that takes
        // ~0.5 - 1.0 s to dissipate (depends on rider mass, friction).
        //
        // After FL/FR transition to CLIMBING, the A4vv front-first gate
        // opens for BL/BR. Without an additional delay, BL/BR's residual
        // (already elevated by the deceleration impulse) immediately
        // crosses threshold -> false trigger.
        //
        // Solution: even AFTER front_first gate opens, BL/BR DETECT is
        // suppressed for this additional window. On entry to that window,
        // BL/BR's torque_baseline is also snap-reset to the current
        // residual so the LPF tracks the new operating point cleanly.
        float back_settle_s       = 1.0f;  // delay after FL/FR enter CLIMBING before BL/BR can trigger

        // A4ab: BACK-LIFT during back-settle window.
        //
        // After FL/FR transition to CLIMBING (and eventually COMPLETE), the
        // chassis sits with front on step and back on flat ground.  Tilting
        // chassis nose-down (CoM forward) helps the BL/BR pivot more
        // efficiently when they reach the step.  A4tt already sets
        // pitch_setpoint to -5 deg for body PID to do this, but the dtheta
        // clamp (+/-5 deg) limits the achievable pitch swing.
        //
        // This direct mechanism: during the back_settle window AND after
        // FL/FR are in COMPLETE, override the BL/BR DETECT hold angle from
        // the default 165 deg to (165 + back_detect_hold_offset_deg).
        //   offset > 0  -> BL/BR target larger (more extension toward 180,
        //                  back of chassis rises slightly, nose down)
        //   offset < 0  -> BL/BR target smaller (back legs shorter, back
        //                  wheels lifted off ground for clean approach)
        //   offset = 0  -> no change (default behaviour)
        // Range clamped to safe bounds [15, 175] so the seam is never near.
        float back_detect_hold_offset_deg = 0.0f;  // tune via Ozone; default = no change

        // --- Climbing kinematics ---
        // Virtual β rate (rad/s). Tuning history:
        //   1.0 -> 0.5 (A4ww): cut jerk on FL/FR CLIMBING entry
        //   0.5 -> 0.8 (A4yy): 0.5 was too slow, user reported "上不去了"
        //                      and the strict wheel cap removed user
        //                      joystick assistance. 0.8 with the wheel cap
        //                      ratio relaxation below gives faster climb
        //                      while still controlling the entry jerk
        //                      via climb_ramp_s.
        // At climb_omega=0.8, leg motor theta_dot at theta_prep=15deg is
        // R*cos(beta0)*omega/(L*sin(theta_prep)) = 0.1*1*0.8/(0.07*0.26)
        //   = 4.4 rad/s = 252 deg/s. Combined with the 0.5s smoothstep
        // ramp (A4ww), the leg starts gently and reaches peak velocity
        // smoothly.
        float climb_omega   = 0.8f;
        // Smoothstep ramp duration at DETECT->CLIMBING entry. phi_w (the
        // effective β rate) starts at 0 and smoothsteps to climb_omega
        // over this time, eliminating the step-on jerk that previously
        // sent target_omega from 0 to ~5.5 rad/s in one tick. The jerk
        // through wheel-leg coupling and chassis dynamics created torque
        // transients on BL/BR that crossed the residual threshold and
        // false-triggered their DETECT.
        float climb_ramp_s  = 0.5f;
        float theta_min_deg = 3.0f;  // Min θ for singularity clamping (deg)

        // --- Robot geometry ---
        float wheel_radius_m = WHEEL_RADIUS_R / 1000.0f;      // R (m)
        float leg_length_m   = ECCENTRIC_OFFSET_r / 1000.0f;  // L = eccentric offset (m)

        // --- Safety ---
        float max_target_v = 0.5f;  // Max velocity feedforward magnitude (m/s)

        // --- Computed-torque tracking (used by Chassis to drive the leg) ---
        // Climbing/prep/complete drive each leg through Wheel_Leg::
        // Set_Leg_Torque_Track, a speed-limited software velocity loop sent as
        // FFW only (Pos_KP = Vel_KD = 0):
        //   omega_cmd = clamp(climb_pos_kp*err + omega_ff, +/-climb_omega_max)
        //   tau       = gravity + clamp(climb_kd_vel*(omega_cmd - omega), +/-climb_tau_max)
        // The DM never runs its internal loops -> no +/-pi unwind; the unwrapped
        // angle + nearest-equivalent target keep the leg single-turn safe; the
        // omega_max clamp bounds leg speed (passenger-safe + protects detection).
        float climb_pos_kp = 8.0f;  // position->velocity gain (1/s)
        // Bumped 3 -> 5 rad/s: with a 3 rad/s ceiling and a heavy rider, the
        // PD's omega_cmd saturated immediately whenever the leg fell behind the
        // smoothstep target (kp_pos*err quickly exceeds 3 once err > 0.375 rad
        // = 21 deg). Once clipped, no matter how large the position error
        // grew, the velocity loop could only command 3 rad/s -- and 18 Nm.s
        // /rad * (3 - actual_omega) was not enough to accelerate the loaded
        // leg through the mid-PREP gravity peak (theta=90). 5 rad/s leaves
        // headroom for the velocity loop to keep pumping torque while the
        // leg catches up. Passenger safety preserved: the smoothstep target's
        // own derivative still peaks at ~1.5 rad/s during a 3 s synchronized
        // lift, so 5 rad/s is only ever reached during catch-up transients.
        float climb_omega_max = 5.0f;
        // Damping: zeta = kd_vel / (2*sqrt(kp_pos*kd_vel*I^-1)). With I_leg ~ 0.01,
        // kd_vel=6 gives zeta~0.45 (~20% overshoot) -- combined with the +/-165
        // clamp's 15deg headroom, the leg never crosses the +/-180 seam during
        // overshoot. Higher kd_vel = stiffer, but velocity-feedback LPF noise gets
        // amplified, so tune carefully.
        // Bumped 12 -> 18 Nm.s/rad: paired with omega_max 3 -> 5, gives the
        // velocity loop ~90 Nm headroom (was 36) to push through the loaded
        // mid-PREP gravity peak. User reported "PREP起来在负载情况下扭矩不够
        // 到达target" after the tau_max 20 -> 200 raise -- meaning the bottleneck
        // wasn't the tau_max clamp, it was the velocity-loop authority itself.
        float climb_kd_vel = 18.0f;  // velocity->torque gain (Nm.s/rad)
        // Bumped 20 -> 200 Nm in A4cc: gravity FFW handles the static torque,
        // but the PD has no headroom to push the lift through friction + inertia
        // without this; 200 Nm matches the DM hardware ceiling already used by
        // COMFORT and HOMING.
        float climb_tau_max = 200.0f;  // clamp on velocity-loop torque, excl. gravity (Nm)
    };

    Climbing_Dynamics() = default;
    explicit Climbing_Dynamics(const Config &cfg);

    void init(const Config &cfg);
    void reset();

    /** Trigger climbing sequence for one leg (IDLE → PREP) */
    void startClimb(int idx);

    /** Start climbing directly from current angle (skip PREP/DETECT) */
    void startClimbDirect(int idx, float current_unsigned_deg);

    /** Trigger climbing sequence for all legs */
    void startClimbAll();

    /**
     * @brief Main update — call every control cycle (dt ≈ 2 ms)
     * @param feedback  Per-leg feedback [4]: FL, FR, BL, BR
     * @param dt        Time step (seconds)
     */
    void update(const LegClimbFeedback feedback[4], float dt);

    // ===== Per-leg output =====

    /** Current climbing phase */
    LegClimbPhase getPhase(int idx) const { return legs_[idx].phase; }

    /** True if this leg is actively climbing (PREP/DETECT/CLIMBING) */
    bool isDirectControl(int idx) const;

    /** Absolute target height for Set_Leg_Height (m). Valid when isDirectControl(). */
    float getTargetHeight(int idx) const { return target_h_[idx]; }

    /** Velocity feedforward for Set_Leg_Height (m/s). Valid when isDirectControl(). */
    float getTargetVelocity(int idx) const { return target_v_[idx]; }

    /** Direct motor angle output (deg, unsigned — apply climb_sign in caller). */
    float getTargetThetaDeg(int idx) const { return target_theta_deg_[idx]; }

    /** Motor angular velocity output (rad/s, unsigned — apply climb_sign in caller). */
    float getTargetOmega(int idx) const { return target_omega_[idx]; }

    /** LPF baseline of torque residual (Nm) — for debug */
    float getBaseline(int idx) const { return legs_[idx].torque_baseline; }

    /** Current β angle (rad) — for debug */
    float getBeta(int idx) const { return legs_[idx].beta; }

    /** β₀ at DETECT→CLIMBING transition — for debug */
    float getBeta0() const { return computeBeta0(); }

    /**
     * Effective β rate phi_w currently driving the climbing trajectory for leg
     * idx. Zero unless leg is in CLIMBING phase; during CLIMBING it ramps from
     * 0 to climb_omega via the climb-entry smoothstep (climb_ramp_s window),
     * then holds at climb_omega. Chassis uses this to cap wheel motor speed
     * during CLIMBING so the wheel doesn't outrun the chassis kinematic
     * forward velocity (chassis forward speed ~= R * cos(β) * phi_w).
     */
    float getEffectivePhiW(int idx) const { return legs_[idx].climb_phi_w_current; }

    Config &config() { return cfg_; }

   private:
    struct LegState
    {
        LegClimbPhase phase     = LegClimbPhase::IDLE;
        float beta              = 0.0f;  // Climbing angle β (rad)
        float detect_timer_s    = 0.0f;  // Detection confirmation timer
        float torque_baseline   = 0.0f;  // LPF baseline of torque residual (Nm)
        float detect_theta_deg  = 0.0f;  // Hold angle for DETECT (0 = use prep angle)
        float baseline_warmup_s = 0.0f;  // Warmup timer: fast LPF convergence before detection
        float prep_ramp_t       = 0.0f;  // Seconds since PREP entry; drives the synchronized lift ramp
        float climb_ramp_t      = 0.0f;  // Seconds since DETECT->CLIMBING transition; ramps phi_w from 0 to climb_omega
        float climb_phi_w_current = 0.0f;  // Effective β rate (rad/s) being applied this tick -- 0 outside CLIMBING, ramped value during CLIMBING. Chassis caps wheel speed by this.
    };

    float computeThetaDot(float theta_rad, float phi_w) const;
    float computeThetaEnd() const;
    float computeBeta0() const;
    float computeBetaFromTheta(float theta_rad) const;
    float heightFromTheta(float theta_rad) const;

    Config cfg_;
    LegState legs_[4];
    float target_h_[4]         = {0};
    float target_v_[4]         = {0};
    float target_theta_deg_[4] = {0};  // unsigned motor angle (deg): 180 = highest, 0 = lowest
    float target_omega_[4]     = {0};  // unsigned angular velocity (rad/s, negative = toward 0°)

    // A4ac: back-leg detect delay state. Triggered on FL/FR BOTH-COMPLETE
    // transition (replaces A4yy's any-front-climbing trigger -- see CLIMBING
    // case comment for why). While back_settle_remaining_s_ > 0 OR FL/FR
    // not both COMPLETE, BL/BR cannot enter CLIMBING.
    float back_settle_remaining_s_ = 0.0f;
    bool  front_was_complete_      = false;

   public:
    /** Remaining seconds in the post-front-CLIMBING back-detect delay (A4yy). 0 = window expired. */
    float getBackSettleRemaining() const { return back_settle_remaining_s_; }
};

}  // namespace Applications
