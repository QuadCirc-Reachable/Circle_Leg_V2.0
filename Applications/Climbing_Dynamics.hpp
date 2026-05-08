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

        // --- Step detection (leg torque residual) ---
        float torque_res_threshold = 3.0f;    // |residual - baseline| to trigger (Nm)
        float baseline_alpha       = 0.02f;   // LPF rate for baseline (~1.6 Hz @ 500 Hz)
        float detect_confirm_s     = 0.020f;  // Confirmation duration (s)

        // --- Climbing kinematics ---
        float climb_omega   = 1.0f;  // Virtual wheel angular velocity for trajectory (rad/s)
        float theta_min_deg = 3.0f;  // Min θ for singularity clamping (deg)

        // --- Robot geometry ---
        float wheel_radius_m = WHEEL_RADIUS_R / 1000.0f;      // R (m)
        float leg_length_m   = ECCENTRIC_OFFSET_r / 1000.0f;  // L = eccentric offset (m)

        // --- Safety ---
        float max_target_v = 0.5f;  // Max velocity feedforward magnitude (m/s)
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
};

}  // namespace Applications
