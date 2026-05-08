#pragma once

#include <cmath>

#include "Robot_Params.hpp"

namespace Applications
{

/**
 * @class Impedance_Controller
 * @brief Variable-impedance active suspension via MIT motor Kp/Kd/FFW modulation
 *
 * ============================================================
 * Derivation: Cartesian Impedance → Joint-Space MIT Parameters
 * ============================================================
 *
 * Eccentric Linkage Kinematics:
 *   H(θ) = R + r·cos(θ)
 *   J(θ) = dH/dθ = −r·sin(θ)        [Jacobian]
 *
 * Desired leg-tip virtual impedance (spring-damper in Cartesian space):
 *   F = kᵥ·(H_des − H) + cᵥ·(Ḣ_des − Ḣ) + F_gravity
 *
 * Motor output torque from leg-tip force (via Jacobian transpose):
 *   τ = Jᵀ · F = −r·sin(θ) · F
 *
 * Linearised position/velocity mapping around operating point:
 *   δH ≈ −r·sin(θ) · δθ
 *   δḢ ≈ −r·sin(θ) · δθ̇
 *
 * Substituting:
 *   τ = r²·sin²(θ)·kᵥ·(θ_des − θ_fb) + r²·sin²(θ)·cᵥ·(θ̇_des − θ̇_fb) − F_g·r·sin(θ)
 *
 * Matching to MIT control law:
 *   τ = Kp·(θ_des − θ_fb) + Kd·(θ̇_des − θ̇_fb) + τ_ff
 *
 * ─────────────────────────────────────────────────────
 *  Result:
 *    Kp_MIT = r² · sin²(θ) · kᵥ
 *    Kd_MIT = r² · sin²(θ) · cᵥ
 *    τ_ff   = −F_load · r · sin(θ)
 * ─────────────────────────────────────────────────────
 *
 * Numerical Example (θ=90°, r=0.065m):
 *   r²·sin²(90°) = 0.004225
 *   kᵥ = 5000 N/m  →  Kp = 21.1 N·m/rad   (within HT8115 range 0-500)
 *   cᵥ = 150 N·s/m →  Kd = 0.63 N·m·s/rad  (within HT8115 range 0-5)
 *
 * ============================================================
 * Active Suspension: Skyhook Damping via Kd Modulation
 * ============================================================
 *
 * Per-leg velocity from rigid-body kinematics:
 *   V_FL = V_z − ωᵣ·W_F/2 + ωₚ·L/2
 *   V_FR = V_z + ωᵣ·W_F/2 + ωₚ·L/2
 *   V_BL = V_z − ωᵣ·W_B/2 − ωₚ·L/2
 *   V_BR = V_z + ωᵣ·W_B/2 − ωₚ·L/2
 *
 * Skyhook: increase damping when body velocity is high.
 *   cᵥ_i = c_base + c_sky · |V_leg_i|
 *
 * Effect: high-frequency road disturbances → large V_leg → high Kd → absorbed.
 *
 * ============================================================
 * Ground Contact: Warp Compensation via Kp Modulation
 * ============================================================
 *
 * Warp error (diagonal load imbalance):
 *   e_warp = mean(I_FL, I_BR) − mean(I_FR, I_BL)
 *
 * Warp sign pattern: {FL:+1, FR:−1, BL:−1, BR:+1}
 *
 *   kᵥ_i = kᵥ_base · (1 − γ · warp_sign_i · clamp(e_warp))
 *
 * Overloaded diagonal → softer → compresses → redistributes load.
 * Underloaded diagonal → stiffer → maintains position → holds ground.
 *
 * ============================================================
 * FFW: Dynamic Load Estimation
 * ============================================================
 *
 * Total mass estimated from motor currents (slow LPF):
 *   M_est = LPF(Σ|I_i| · KA · GR) / g
 *
 * Per-leg gravity compensation torque:
 *   τ_ff_i = −(M_est/4 + m_leg) · g · r · sin(θ_i)
 *
 * This adapts to loaded/unloaded conditions over ~1 second.
 * ============================================================
 */
class Impedance_Controller
{
   public:
    struct Config
    {
        // --- Eccentric Geometry ---
        float r = ECCENTRIC_OFFSET_r / 1000.0f;  // m

        // --- Virtual Impedance (Cartesian space) ---
        float kv_base = 800.0f;  // Base virtual stiffness (N/m)  → Kp≈3.4 @θ=90°
        float cv_base = 250.0f;  // Fixed virtual damping (N·s/m) → Kd≈1.06

        // --- Skyhook Kd Modulation (DISABLED — accel_z noise too high) ---
        float cv_sky_gain = 0.0f;    // Disabled: accel-based velocity estimate is pure noise
        float cv_max      = 400.0f;  // Ceiling for cv

        // --- Warp Kp/FFW Modulation (Ground Contact) ---
        float kv_warp_gain  = 0.5f;    // Kp redistribution: overloaded diagonal softens, underloaded stiffens
        float kv_min        = 100.0f;  // Floor for virtual stiffness (N/m) → Kp min≈0.4
        float warp_deadband = 0.02f;   // Current deadband (A) — low for soft-leg regime
        float warp_clamp    = 3.0f;    // Max warp error used for modulation (A)
        float warp_ffw_gain = 0.25f;   // FFW offset (Nm/A): -warp_sign pushes overloaded diagonal DOWN

        // --- FFW Load Estimation ---
        float ffw_lpf_alpha = 0.01f;  // Faster LPF (~0.8 Hz @ 500 Hz) for quicker adaptation
        float ka_times_gr   = 3.5f;   // KA_motor × GearRatio (HT8115: 0.583 × 6)
        float leg_mass      = LEG_MASS_kg;

        // --- MIT Parameter Limits (hardware) ---
        float mit_kp_min = 2.0f;
        float mit_kp_max = 100.0f;
        float mit_kd_min = 0.3f;
        float mit_kd_max = 4.5f;

        // --- Mode Transition Ramp ---
        float entry_kp  = 35.0f;   // Kp to blend FROM on mode entry (matches default MIT / ENGSAV)
        float entry_kd  = 1.5f;    // Kd to blend FROM
        float ramp_rate = 0.005f;  // Ramp increment per cycle (0→1 in 200 cycles = 0.4s @500Hz)

        // --- Chassis Geometry (for rigid-body kinematics) ---
        float W_F = WHEEL_TRACK_FRONT / 1000.0f;
        float W_B = WHEEL_TRACK_BACK / 1000.0f;
        float L   = WHEEL_BASE / 1000.0f;

        // --- Body Velocity Estimation ---
        float vel_lpf_alpha  = 0.01f;   // HPF cutoff for DC removal (~0.8 Hz)
        float vel_decay      = 0.995f;  // Leaky integrator decay (faster reset)
        float accel_deadband = 0.05f;   // Ignore small accel (m/s²)
    };

    struct LegOutput
    {
        float kp;          // MIT Kp (N·m/rad)
        float kd;          // MIT Kd (N·m·s/rad)
        float ffw_torque;  // FFW torque (Nm) — same convention as Get_LegGravityTorque()
    };

    Impedance_Controller() = default;
    explicit Impedance_Controller(const Config &cfg);

    void init(const Config &cfg);
    void reset();

    /**
     * @brief Main update — call every control cycle (dt ≈ 2 ms)
     * @param leg_currents   [FL, FR, BL, BR] motor currents (A)
     * @param leg_angles_deg [FL, FR, BL, BR] kinematic angles (deg, from Get_LegPosition)
     * @param accel_z        Body vertical acceleration (m/s²), gravity removed
     * @param omega_roll     Roll angular velocity (rad/s)
     * @param omega_pitch    Pitch angular velocity (rad/s)
     * @param dt             Time step (s)
     */
    void update(const float leg_currents[4], const float leg_angles_deg[4], float accel_z, float omega_roll, float omega_pitch, float dt);

    const LegOutput &getLegOutput(int idx) const { return outputs_[idx]; }
    float getEstimatedMass() const { return estimated_mass_; }
    float getBodyVelZ() const { return vel_z_hp_; }
    float getWarpError() const { return warp_error_; }
    Config &config() { return cfg_; }

   private:
    Config cfg_;
    LegOutput outputs_[4] = {};

    // Mass estimation
    float I_filter_       = 0.0f;
    float estimated_mass_ = 1.0f;

    // Body velocity (skyhook)
    float raw_vel_z_ = 0.0f;
    float vel_z_lpf_ = 0.0f;
    float vel_z_hp_  = 0.0f;

    // Warp
    float warp_error_ = 0.0f;

    // Mode transition ramp (0=entry, 1=full impedance)
    float ramp_alpha_ = 0.0f;
};

}  // namespace Applications
