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

    // Deadband: ignore small errors (noise when unloaded / airborne)
    if (raw_error > -cfg_.deadband && raw_error < cfg_.deadband)
        raw_error = 0.0f;
    warp_error_ = raw_error;

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
