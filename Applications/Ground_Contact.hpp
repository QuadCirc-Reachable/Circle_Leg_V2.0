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
        // Tuning history:
        //   kp=0.002, ki=0.0005, db=0.15 → "站起来后疯狂晃动"
        //   kp=0.0005, ki=0.0001, db=0.6 → still 越抖越大 even with impedance warp
        //     gains zeroed and 3Hz current LPF.
        //   kp=0, ki=0 → isolated: GC was NOT the source, IMU PID was.
        //   After PID rework (lvl_alpha=0.02, scale=0.7) → carefully re-enable
        //   GC at kp=0.0001 (5× smaller than the last attempt) to handle the
        //   diagonal warp DOF that pitch/roll PID can't reach.
        //   0.0001 → felt too weak when one leg was lifted, slight residual sway.
        //   0.0003 + ki 0.00003 → stable but warp_dh still <1mm/A, can't pull
        //     unloaded legs to ground in time.
        //   → kp=0.001, ki=0.0001. Per amp: ~2.5 mm dh, hits 20 mm at ~8 A.
        float kp          = 0.001f;   // m / A
        float ki          = 0.0001f;  // m / (A·s)
        float max_warp_dh = 0.030f;   // raise from 20mm to 30mm headroom
        float deadband    = 0.6f;
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
