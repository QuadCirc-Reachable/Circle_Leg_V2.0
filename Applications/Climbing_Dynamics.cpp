#include "Climbing_Dynamics.hpp"

namespace Applications
{

Climbing_Dynamics::Climbing_Dynamics(const Config &cfg) : cfg_(cfg) { reset(); }

void Climbing_Dynamics::init(const Config &cfg)
{
    cfg_ = cfg;
    reset();
}

void Climbing_Dynamics::reset()
{
    for (int i = 0; i < 4; i++)
    {
        legs_[i]     = LegState{};
        target_h_[i] = 0.0f;
        target_v_[i] = 0.0f;
    }
}

void Climbing_Dynamics::startClimb(int idx)
{
    if (idx < 0 || idx > 3)
        return;
    legs_[idx].phase          = LegClimbPhase::PREP;
    legs_[idx].beta           = 0.0f;
    legs_[idx].detect_timer_s = 0.0f;
}

void Climbing_Dynamics::startClimbDirect(int idx, float current_unsigned_deg)
{
    if (idx < 0 || idx > 3)
        return;
    // Skip PREP: go directly to DETECT at current angle, wait for step contact.
    legs_[idx].phase             = LegClimbPhase::DETECT;
    legs_[idx].beta              = 0.0f;
    legs_[idx].detect_timer_s    = 0.0f;
    legs_[idx].detect_theta_deg  = current_unsigned_deg;  // hold at this angle, not prep
    legs_[idx].torque_baseline   = 0.0f;                  // fast-converge during warmup
    legs_[idx].baseline_warmup_s = 0.0f;                  // reset warmup counter
}

void Climbing_Dynamics::startClimbAll()
{
    // Front-first: only start FL(0)/FR(1).
    // BL(2)/BR(3) auto-start when both front legs reach COMPLETE.
    startClimb(0);
    startClimb(1);
}

bool Climbing_Dynamics::isDirectControl(int idx) const
{
    LegClimbPhase p = legs_[idx].phase;
    // All active phases use climbing-controlled angle (PREP/DETECT/CLIMBING/COMPLETE).
    // COMPLETE holds the final climbing angle to prevent snap-back.
    return p == LegClimbPhase::PREP || p == LegClimbPhase::DETECT || p == LegClimbPhase::CLIMBING || p == LegClimbPhase::COMPLETE;
}

// =====================================================================
// Kinematic helpers
// =====================================================================

float Climbing_Dynamics::computeBeta0() const
{
    float prep_rad = cfg_.prep_theta_deg * PI / 180.0f;
    return computeBetaFromTheta(prep_rad);
}

float Climbing_Dynamics::computeBetaFromTheta(float theta_rad) const
{
    // From constraint: cos(θ) = (R + L − h − R·sin(β)) / L
    // → sin(β) = [ R + L·(1 − cos(θ)) − h ] / R
    float R   = cfg_.wheel_radius_m;
    float L   = cfg_.leg_length_m;
    float h   = cfg_.step_height_m;
    float arg = (R + L * (1.0f - cosf(theta_rad)) - h) / R;
    if (arg > 1.0f)
        arg = 1.0f;
    if (arg < -1.0f)
        arg = -1.0f;
    return asinf(arg);
}

float Climbing_Dynamics::computeThetaEnd() const
{
    // θ_end = arccos((L - h) / L)
    float arg = (cfg_.leg_length_m - cfg_.step_height_m) / cfg_.leg_length_m;
    if (arg > 1.0f)
        arg = 1.0f;
    if (arg < -1.0f)
        arg = -1.0f;
    return acosf(arg);
}

float Climbing_Dynamics::computeThetaDot(float theta_rad, float phi_w) const
{
    //            sqrt(R² - (R - h + L·(1 - cosθ))²)
    // θ̇ =  ─────────────────────────────────────────── · φ_w
    //                    L · sinθ

    float R = cfg_.wheel_radius_m;
    float L = cfg_.leg_length_m;
    float h = cfg_.step_height_m;

    float inner = R - h + L * (1.0f - cosf(theta_rad));
    float disc  = R * R - inner * inner;
    if (disc < 0.0f)
        disc = 0.0f;

    float numerator = sqrtf(disc);

    // Singularity protection: clamp sin(θ) away from zero
    float sin_theta = sinf(theta_rad);
    float min_sin   = sinf(cfg_.theta_min_deg * PI / 180.0f);
    if (min_sin < 1e-4f)
        min_sin = 1e-4f;
    if (sin_theta < min_sin)
        sin_theta = min_sin;

    float denominator = L * sin_theta;
    return (numerator / denominator) * phi_w;
}

float Climbing_Dynamics::heightFromTheta(float theta_rad) const
{
    // Pipeline model: h = R + L·cos(θ)
    return cfg_.wheel_radius_m + cfg_.leg_length_m * cosf(theta_rad);
}

// =====================================================================
// Main update — per-leg state machine
// =====================================================================

void Climbing_Dynamics::update(const LegClimbFeedback feedback[4], float dt)
{
    float theta_end = computeThetaEnd();
    float prep_rad  = cfg_.prep_theta_deg * PI / 180.0f;
    float h_prep    = heightFromTheta(prep_rad);

    for (int i = 0; i < 4; i++)
    {
        LegState &leg              = legs_[i];
        const LegClimbFeedback &fb = feedback[i];

        switch (leg.phase)
        {
        // ---------------------------------------------------------
        case LegClimbPhase::IDLE:
            target_h_[i]         = 0.0f;
            target_v_[i]         = 0.0f;
            target_theta_deg_[i] = 0.0f;
            target_omega_[i]     = 0.0f;
            break;

        // ---------------------------------------------------------
        case LegClimbPhase::PREP:
        {
            // Command leg to prep angle (near-extended) to avoid singularity
            target_h_[i]         = h_prep;
            target_v_[i]         = 0.0f;
            target_theta_deg_[i] = 180.0f - cfg_.prep_theta_deg;  // e.g. 165°
            target_omega_[i]     = 0.0f;

            // Warm up torque baseline during PREP so it's stable for DETECT
            leg.torque_baseline = cfg_.baseline_alpha * fb.leg_torque_residual + (1.0f - cfg_.baseline_alpha) * leg.torque_baseline;

            // Check if leg has reached prep angle (within tolerance)
            // Motor angle for prep = 180° - prep_theta_deg (e.g. 165° for 15° deadzone)
            float current_theta    = fabsf(fb.leg_pos_deg);
            float prep_motor_angle = 180.0f - cfg_.prep_theta_deg;
            if (fabsf(current_theta - prep_motor_angle) < cfg_.prep_tolerance_deg)
            {
                leg.phase          = LegClimbPhase::DETECT;
                leg.detect_timer_s = 0.0f;
                // Baseline is already warm from LPF above
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::DETECT:
        {
            // Hold at detect angle while monitoring torque for step contact.
            // detect_theta_deg > 0 means we entered via startClimbDirect (skip PREP).
            float hold_angle = (leg.detect_theta_deg > 0.0f) ? leg.detect_theta_deg : (180.0f - cfg_.prep_theta_deg);
            float hold_h     = heightFromTheta((180.0f - hold_angle) * PI / 180.0f);

            target_h_[i]         = hold_h;
            target_v_[i]         = 0.0f;
            target_theta_deg_[i] = hold_angle;
            target_omega_[i]     = 0.0f;

            // Warmup: use fast LPF (alpha=0.3) for first 0.1s to converge baseline,
            // then switch to slow LPF. Detection disabled during warmup.
            constexpr float warmup_duration = 0.1f;  // 50 frames @ 500Hz
            constexpr float fast_alpha      = 0.3f;
            bool warmed_up                  = (leg.baseline_warmup_s >= warmup_duration);
            float alpha_use                 = warmed_up ? cfg_.baseline_alpha : fast_alpha;
            leg.torque_baseline             = alpha_use * fb.leg_torque_residual + (1.0f - alpha_use) * leg.torque_baseline;
            leg.baseline_warmup_s += dt;

            // Step detection: only after warmup
            float deviation = fabsf(fb.leg_torque_residual - leg.torque_baseline);
            if (warmed_up && deviation > cfg_.torque_res_threshold)
            {
                leg.detect_timer_s += dt;
                if (leg.detect_timer_s >= cfg_.detect_confirm_s)
                {
                    // Step confirmed — begin climbing from hold angle
                    leg.phase             = LegClimbPhase::CLIMBING;
                    float theta_model_rad = (180.0f - hold_angle) * PI / 180.0f;
                    if (theta_model_rad < cfg_.theta_min_deg * PI / 180.0f)
                        theta_model_rad = cfg_.theta_min_deg * PI / 180.0f;
                    leg.beta = computeBetaFromTheta(theta_model_rad);
                }
            }
            else
            {
                leg.detect_timer_s = 0.0f;  // Reset if spike subsides
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::CLIMBING:
        {
            // Integrate β at fixed rate (independent of actual wheel speed).
            // Using actual wheel RPM fails because the wheel stalls against
            // the step edge, giving phi_w ≈ 0 and freezing the trajectory.
            float phi_w = cfg_.climb_omega;  // constant virtual angular velocity (rad/s)
            leg.beta += phi_w * dt;

            // Compute target θ from constraint:
            //   cos(θ) = (R + L − h − R·sin(β)) / L
            float R   = cfg_.wheel_radius_m;
            float L   = cfg_.leg_length_m;
            float h   = cfg_.step_height_m;
            float cth = (R + L - h - R * sinf(leg.beta)) / L;
            if (cth > 1.0f)
                cth = 1.0f;
            if (cth < -1.0f)
                cth = -1.0f;
            float theta_rad = acosf(cth);
            float theta_deg = theta_rad * 180.0f / PI;

            // Correct θ̇ for β-driven trajectory:
            //   d(cosθ)/dt = −R·cos(β)·β̇ / L
            //   θ̇ = R·cos(β)·β̇ / (L·sin(θ))
            float sin_theta = sinf(theta_rad);
            float min_sin   = 0.05f;  // singularity guard
            if (sin_theta < min_sin)
                sin_theta = min_sin;
            float theta_dot = R * cosf(leg.beta) * phi_w / (L * sin_theta);

            // Direct angle output for Chassis.cpp (bypass height→angle mapping)
            // Motor convention: 180° = body highest, 0° = body lowest
            // As θ_model increases, motor unsigned angle = 180°-θ DECREASES
            target_theta_deg_[i] = 180.0f - theta_deg;
            // Motor angular velocity: d(180°-θ)/dt = -θ̇ (negative = toward 0°)
            target_omega_[i] = -theta_dot;  // rad/s, unsigned frame

            // Also keep height output for debug/display
            target_h_[i] = heightFromTheta(theta_rad);
            target_v_[i] = -L * sinf(theta_rad) * theta_dot;

            // End condition: β ≥ 90° or θ ≥ θ_end
            if (leg.beta >= PI / 2.0f || theta_rad >= theta_end)
            {
                leg.phase = LegClimbPhase::COMPLETE;
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::COMPLETE:
        {
            // Hold at the end-of-climb angle: wheel is on step, leg stays bent.
            //   H_complete = R + L - h
            //   cos(θ_model) = (L - h) / L
            float L   = cfg_.leg_length_m;
            float h   = cfg_.step_height_m;
            float cth = (L - h) / L;
            if (cth > 1.0f)
                cth = 1.0f;
            if (cth < -1.0f)
                cth = -1.0f;
            float theta_complete_deg = acosf(cth) * 180.0f / PI;

            target_theta_deg_[i] = 180.0f - theta_complete_deg;
            target_omega_[i]     = 0.0f;
            target_h_[i]         = heightFromTheta(acosf(cth));
            target_v_[i]         = 0.0f;
            // Stay in COMPLETE — caller must explicitly reset to IDLE
            break;
        }
        }

        // Clamp velocity output
        if (target_v_[i] > cfg_.max_target_v)
            target_v_[i] = cfg_.max_target_v;
        if (target_v_[i] < -cfg_.max_target_v)
            target_v_[i] = -cfg_.max_target_v;
    }

    // Front-first gating: auto-start back legs when both front legs reach COMPLETE.
    // Skip PREP/DETECT — start CLIMBING directly from current leg angle.
    if (legs_[0].phase == LegClimbPhase::COMPLETE && legs_[1].phase == LegClimbPhase::COMPLETE)
    {
        for (int i = 2; i < 4; i++)
        {
            if (legs_[i].phase == LegClimbPhase::IDLE)
            {
                // Convert signed feedback to unsigned: motor angle = |feedback|
                float unsigned_deg = fabsf(feedback[i].leg_pos_deg);
                startClimbDirect(i, unsigned_deg);
            }
        }
    }
}

}  // namespace Applications
