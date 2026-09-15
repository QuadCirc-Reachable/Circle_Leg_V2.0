/**
 * @file    Ground_Contact.cpp
 * @brief   Warp (diagonal twist) compensator: PI loop on the diagonal leg-current
 *          imbalance -> per-leg height offset, keeps all four wheels grounded.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#include "Ground_Contact.hpp"

namespace Applications
{

static inline float clampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void GroundContact::reset()
{
    warp_integral_ = 0.0f;
    warp_error_    = 0.0f;
    warp_dh_       = 0.0f;
    dh_[0] = dh_[1] = dh_[2] = dh_[3] = 0.0f;
}

void GroundContact::update(const float leg_currents[4], float dt)
{
    // Diagonal load imbalance:
    //   positive → FL+BR carry more than FR+BL
    float raw_error = (leg_currents[0] + leg_currents[3]) * 0.5f - (leg_currents[1] + leg_currents[2]) * 0.5f;

    // LPF the error before deadband — leg currents at COMFORT Kp ≈ 250 are
    // VERY noisy (PWM ripple, chassis vibration), and any high-frequency
    // content fed back as a height bias goes through impedance Kp → torque →
    // chassis pitch/roll → more current noise → positive feedback shake.
    // Cutoff ≈ 3 Hz: slow enough to ignore vibration, fast enough to follow
    // a step terrain transient (<1 Hz).
    static float err_lpf = 0.0f;
    const float alpha    = 0.04f;  // ~3 Hz @ 500Hz
    err_lpf              = alpha * raw_error + (1.0f - alpha) * err_lpf;

    // Deadband: ignore small errors (noise when unloaded / airborne)
    float gated = err_lpf;
    if (gated > -cfg_.deadband && gated < cfg_.deadband)
        gated = 0.0f;
    warp_error_ = gated;

    // PI controller → drive warp_error towards zero
    warp_integral_ += warp_error_ * dt;

    // Decay integral towards zero when error is in deadband (prevents residual drift)
    // Also always apply mild decay to prevent slow bias accumulation
    if (warp_error_ == 0.0f)
        warp_integral_ *= 0.990f;
    else
        warp_integral_ *= 0.999f;

    // Anti-windup
    float int_limit = (cfg_.ki > 1e-6f) ? (cfg_.max_warp_dh / cfg_.ki) : 1e6f;
    warp_integral_  = clampF(warp_integral_, -int_limit, int_limit);

    warp_dh_ = cfg_.kp * warp_error_ + cfg_.ki * warp_integral_;
    warp_dh_ = clampF(warp_dh_, -cfg_.max_warp_dh, cfg_.max_warp_dh);

    // Warp pattern: orthogonal to heave, pitch, roll
    // If FL+BR diagonal is overloaded (positive error) → shorten them, extend FR+BL
    dh_[0] = -warp_dh_;  // FL
    dh_[1] = +warp_dh_;  // FR
    dh_[2] = +warp_dh_;  // BL
    dh_[3] = -warp_dh_;  // BR
}

float GroundContact::getDeltaH(int leg_idx) const
{
    if (leg_idx < 0 || leg_idx > 3)
        return 0.0f;
    return dh_[leg_idx];
}

}  // namespace Applications
