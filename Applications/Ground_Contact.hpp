#pragma once

#include <cstdint>

namespace Applications
{

/**
 * @class GroundContact
 * @brief Warp-mode compensator to ensure all 4 wheels maintain ground contact
 *
 * ============================================================
 * Problem
 * ============================================================
 * A rigid chassis with 4 contact points on uneven ground is over-constrained.
 * The Pitch/Roll PID controls 2 of the 3 tilting DOFs, leaving the
 * "warp" (diagonal twist) DOF uncontrolled — one wheel can lift off.
 *
 * ============================================================
 * Solution — Warp Decomposition
 * ============================================================
 * The 4 per-leg height adjustments decompose into 4 orthogonal modes:
 *
 *   Heave:  (+1, +1, +1, +1)   ← controlled by target_chassis_height_
 *   Pitch:  (+1, +1, -1, -1)   ← controlled by pitch PID
 *   Roll:   (-1, +1, -1, +1)   ← controlled by roll PID
 *   Warp:   (+1, -1, -1, +1)   ← **THIS MODULE**
 *
 * We detect warp imbalance via leg motor current (proxy for ground force):
 *   warp_error = mean(I_FL, I_BR) − mean(I_FR, I_BL)
 *
 * A PI controller drives warp_error → 0 by adjusting the warp-mode ΔH.
 * Because the warp pattern is orthogonal to heave/pitch/roll, this
 * controller CANNOT fight the existing PID loops.
 *
 * Output ΔH per leg:
 *   FL: −warp_dh,  FR: +warp_dh,  BL: +warp_dh,  BR: −warp_dh
 * ============================================================
 */
class GroundContact
{
   public:
    struct Config
    {
        float kp          = 0.002f;   // Proportional gain (m / A)
        float ki          = 0.0005f;  // Integral gain     (m / (A·s)) — low to avoid bias drift
        float max_warp_dh = 0.020f;   // Max compensation  (m) — 20 mm
        float deadband    = 0.15f;    // Ignore warp_error below this (A) — reject static current bias
    };

    void reset();
    void update(const float leg_currents[4], float dt);

    float getDeltaH(int leg_idx) const;
    float getWarpError() const { return warp_error_; }
    float getWarpDH() const { return warp_dh_; }
    Config &config() { return cfg_; }

   private:
    Config cfg_;
    float warp_integral_ = 0.0f;
    float warp_error_    = 0.0f;
    float warp_dh_       = 0.0f;
    float dh_[4]         = {};
};

}  // namespace Applications
