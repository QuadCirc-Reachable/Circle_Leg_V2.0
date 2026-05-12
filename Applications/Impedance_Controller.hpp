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
        // Sized for human-payload (≈70 kg) operation. At θ=90°:
        //   Kp_MIT = r²·sin²·kv = 0.0081·kv  →  kv=14000 ⇒ Kp≈113 N·m/rad
        //   Kd_MIT = r²·sin²·cv = 0.0081·cv  →  cv=1500  ⇒ Kd≈12 (clamped to 5)
        // Tuning history:
        //   kv_base = 20000 (Kp≈162) → under heavy load, transient disturbances
        //     (ground impact / wheel reaction / warp dh step) excite a brief
        //     high-frequency limit cycle in the leg position loop (current
        //     spike 25A→60A with ringing). DM Kd is already saturated at 5,
        //     so we can't add more damping; the only way to recover damping
        //     ratio ζ = Kd / (2√(Kp·J)) is to reduce Kp.
        //   kv_base = 14000 → Kp≈113. Gravity hold comes from FFW (mass-est
        //     × g × r × sinθ), so position loop only tracks small deviations;
        //     a lower Kp is fine.
        float kv_base = 14000.0f;  // Base virtual stiffness (N/m)
        float cv_base = 1500.0f;   // Base virtual damping (N·s/m).

        // --- Skyhook Kd Modulation (DISABLED — accel_z noise too high) ---
        float cv_sky_gain = 0.0f;     // Disabled: accel-based velocity estimate is pure noise
        float cv_max      = 1500.0f;  // Ceiling for cv

        // --- Warp Kp/FFW Modulation (DISABLED — moved to Ground_Contact) ---
        // The raw_warp formula here computes (I_FL+I_BR)/2 − (I_FR+I_BL)/2 on
        // RAW motor currents, but front legs have bending_direction=-1 vs back
        // legs +1, so equal ground load produces opposite-sign currents and
        // the "diagonal" formula actually measures pitch-front-vs-back, not
        // warp. Worse: feeding this signal back through kv_warp_gain (±50%
        // Kp) and warp_ffw_gain (direct torque) creates a positive-feedback
        // loop with the noisy current → "越抖越大" oscillation in COMFORT.
        //
        // The corrected warp control now lives in Ground_Contact.cpp (which
        // normalizes by bending_direction in Chassis.cpp before calling).
        // Keep these zero here.
        float kv_warp_gain  = 0.0f;     // was 0.5
        float kv_min        = 5000.0f;  // Floor for virtual stiffness (N/m)
        float warp_deadband = 0.02f;
        float warp_clamp    = 3.0f;
        float warp_ffw_gain = 0.0f;  // was 0.25

        // --- FFW Load Estimation ---
        // NOTE: `leg_currents` passed to update() is actually DM output torque in Nm
        // (DMMotor decodes the τ-field with tMax=200 Nm). Mass is estimated from
        // per-leg balance |τ| = (M/4+m_leg)·g·r·|sin θ|; ka_times_gr is unused.
        float ffw_lpf_alpha = 0.01f;          // ~0.8 Hz LPF on M_est. Slower than mechanical
                                              // response → breaks self-feedback limit cycle.
        float mass_update_v_thresh = 0.04f;   // m/s; only update M_est when ALL legs settled
        float ka_times_gr          = 1.263f;  // DM J10010L_2EC: KA(0.1263 Nm/A) × GR(10) — kept for reference, unused
        float leg_mass             = LEG_MASS_kg;

        // --- MIT Parameter Limits (hardware) ---
        // DM J10010L MIT mode accepts Kp ∈ [0, 500], Kd ∈ [0, 5]; we leave headroom.
        // Kd floor must be substantial: at small |sin θ| (near workspace limits)
        // the impedance-computed Kd drops to nearly zero. Without a real floor,
        // legs lose damping and rock chaotically.
        float mit_kp_min = 50.0f;
        float mit_kp_max = 250.0f;
        float mit_kd_min = 2.5f;
        float mit_kd_max = 5.0f;

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
    float getRampAlpha() const { return ramp_alpha_; }
    Config &config() { return cfg_; }

    // External settle flag: caller (Chassis) sets to false whenever the
    // commanded height is slewing. Mass estimator will refuse to update while
    // false, so transient unloading (apparent weight loss during commanded
    // descent) cannot collapse M_est → FFW → support → deeper drop.
    void setSettled(bool s) { external_settled_ = s; }

   private:
    Config cfg_;
    LegOutput outputs_[4] = {};

    // Mass estimation
    float I_filter_        = 0.0f;
    float estimated_mass_  = 1.0f;
    bool mass_warmed_      = false;  // false until first valid sample → snap to it
    bool external_settled_ = true;   // set false by Chassis during slew

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
