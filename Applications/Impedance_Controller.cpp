#include "Impedance_Controller.hpp"

namespace Applications
{

static inline float clampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float absF(float v) { return v < 0.0f ? -v : v; }

static constexpr float DEG2RAD = 3.14159265f / 180.0f;
static constexpr float GRAVITY = 9.81f;

Impedance_Controller::Impedance_Controller(const Config &cfg) : cfg_(cfg) { reset(); }

void Impedance_Controller::init(const Config &cfg)
{
    cfg_ = cfg;
    reset();
}

void Impedance_Controller::reset()
{
    I_filter_       = 0.0f;
    estimated_mass_ = 1.0f;
    raw_vel_z_      = 0.0f;
    vel_z_lpf_      = 0.0f;
    vel_z_hp_       = 0.0f;
    warp_error_     = 0.0f;
    ramp_alpha_     = 0.0f;  // Start with entry Kp/Kd, ramp toward impedance values
    for (int i = 0; i < 4; i++)
        outputs_[i] = {cfg_.entry_kp, cfg_.entry_kd, 0.0f};
}

void Impedance_Controller::update(
    const float leg_currents[4], const float leg_angles_deg[4], float accel_z, float omega_roll, float omega_pitch, float dt)
{
    const float r  = cfg_.r;
    const float r2 = r * r;

    // --- Advance mode-entry ramp ---
    ramp_alpha_ += cfg_.ramp_rate;
    if (ramp_alpha_ > 1.0f)
        ramp_alpha_ = 1.0f;

    // ================================================================
    // 1. Mass Estimation from total motor current (slow LPF)
    //    M_est = LPF(Σ|Iᵢ|) × KA × GR / g
    // ================================================================
    float I_total = absF(leg_currents[0]) + absF(leg_currents[1]) + absF(leg_currents[2]) + absF(leg_currents[3]);

    I_filter_ = cfg_.ffw_lpf_alpha * I_total + (1.0f - cfg_.ffw_lpf_alpha) * I_filter_;

    estimated_mass_ = (I_filter_ * cfg_.ka_times_gr) / GRAVITY;
    if (estimated_mass_ < 1.0f)
        estimated_mass_ = 1.0f;

    float load_per_leg = estimated_mass_ / 4.0f + cfg_.leg_mass;

    // ================================================================
    // 2. Body Vertical Velocity (for Skyhook damping)
    //    Leaky integrator + HPF to remove DC bias
    // ================================================================
    float az   = (accel_z > -cfg_.accel_deadband && accel_z < cfg_.accel_deadband) ? 0.0f : accel_z;
    raw_vel_z_ = raw_vel_z_ * cfg_.vel_decay + az * dt;

    vel_z_lpf_ = cfg_.vel_lpf_alpha * raw_vel_z_ + (1.0f - cfg_.vel_lpf_alpha) * vel_z_lpf_;
    vel_z_hp_  = raw_vel_z_ - vel_z_lpf_;

    float V_z = vel_z_hp_;

    // Per-leg velocities from rigid-body kinematics
    float half_Wf = cfg_.W_F * 0.5f;
    float half_Wb = cfg_.W_B * 0.5f;
    float half_L  = cfg_.L * 0.5f;

    float V_leg[4];
    V_leg[0] = V_z - omega_roll * half_Wf + omega_pitch * half_L;  // FL
    V_leg[1] = V_z + omega_roll * half_Wf + omega_pitch * half_L;  // FR
    V_leg[2] = V_z - omega_roll * half_Wb - omega_pitch * half_L;  // BL
    V_leg[3] = V_z + omega_roll * half_Wb - omega_pitch * half_L;  // BR

    // ================================================================
    // 3. Warp Error (diagonal load imbalance for ground contact)
    //    e_warp = mean(I_FL, I_BR) − mean(I_FR, I_BL)
    // ================================================================
    float raw_warp = (leg_currents[0] + leg_currents[3]) * 0.5f - (leg_currents[1] + leg_currents[2]) * 0.5f;

    if (raw_warp > -cfg_.warp_deadband && raw_warp < cfg_.warp_deadband)
        raw_warp = 0.0f;
    warp_error_ = raw_warp;

    // Clamp warp error for modulation
    float warp_clamped = clampF(warp_error_, -cfg_.warp_clamp, cfg_.warp_clamp);

    // Warp sign pattern: {FL:+1, FR:−1, BL:−1, BR:+1}
    // Overloaded diagonal gets softer (lower Kp), underloaded gets stiffer
    static const float warp_sign[4] = {1.0f, -1.0f, -1.0f, 1.0f};

    // ================================================================
    // 4. Per-Leg Impedance Computation
    //    Kp_MIT = r²·sin²(θ) · kᵥ
    //    Kd_MIT = r²·sin²(θ) · cᵥ
    //    τ_ff   = −F_load · r · sin(θ)
    // ================================================================
    for (int i = 0; i < 4; i++)
    {
        float theta_rad = leg_angles_deg[i] * DEG2RAD;
        float sin_theta = sinf(theta_rad);
        float sin2      = sin_theta * sin_theta;

        // --- Virtual stiffness with warp modulation ---
        // No bend_sign needed: warp error sign naturally flips with bending direction
        // (inward legs have positive current, outward have negative → warp auto-adapts)
        float warp_norm   = (cfg_.warp_clamp > 0.01f) ? (warp_clamped / cfg_.warp_clamp) : 0.0f;
        float warp_factor = 1.0f - cfg_.kv_warp_gain * warp_sign[i] * warp_norm;
        float kv_i        = cfg_.kv_base * warp_factor;
        if (kv_i < cfg_.kv_min)
            kv_i = cfg_.kv_min;

        // --- Virtual damping with skyhook modulation ---
        // cᵥ_i = c_base + c_sky · |V_leg_i|
        float cv_i = cfg_.cv_base + cfg_.cv_sky_gain * absF(V_leg[i]);
        if (cv_i > cfg_.cv_max)
            cv_i = cfg_.cv_max;

        // --- Map to MIT parameters ---
        float raw_kp = clampF(r2 * sin2 * kv_i, cfg_.mit_kp_min, cfg_.mit_kp_max);
        float raw_kd = clampF(r2 * sin2 * cv_i, cfg_.mit_kd_min, cfg_.mit_kd_max);

        // --- Ramp blend: smooth entry from default Kp/Kd ---
        outputs_[i].kp = cfg_.entry_kp + ramp_alpha_ * (raw_kp - cfg_.entry_kp);
        outputs_[i].kd = cfg_.entry_kd + ramp_alpha_ * (raw_kd - cfg_.entry_kd);

        // --- FFW: dynamic gravity compensation + warp ground-seeking offset ---
        // Gravity comp: τ_ff = −load · g · r · sin(θ)  (auto-correct for both bending dirs)
        // Warp FFW: −warp_sign pushes underloaded legs toward ground.
        //   No bend_sign: current signs already encode bending direction.
        float warp_ffw_offset  = -warp_sign[i] * warp_clamped * cfg_.warp_ffw_gain;
        outputs_[i].ffw_torque = -(load_per_leg * GRAVITY * r * sin_theta) + warp_ffw_offset;
    }
}

}  // namespace Applications
