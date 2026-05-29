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
    legs_[idx].prep_ramp_t    = 0.0f;  // reset shared lift-ramp clock
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
    // All four legs PREP together so they share the synchronized lift ramp
    // (see prep_ramp_s) -- otherwise BL/BR fall back to the normal Set_Leg_Height
    // pipeline (slewed at 0.2 m/s) and would rise much faster than FL/FR's
    // 1.5 s ramp, tilting the chassis. After PREP each leg transitions to DETECT
    // and waits there for its own wheel's step-contact torque spike before
    // proceeding to CLIMBING -- so the front-first climbing sequence is still
    // enforced naturally by physics (back wheels are not on the step yet).
    startClimb(0);
    startClimb(1);
    startClimb(2);
    startClimb(3);
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
    // The body starts at the prep pose (θ = prep_theta), so its Y coordinate is
    //   H_prep = R + L·cos(prep_theta)
    // After climbing one step the body has descended by exactly one step height:
    //   H_complete = H_prep − h
    // Inverting H = R + L·cos(θ_model) gives:
    //   cos(θ_end) = cos(prep_theta) − h / L
    // (Previously the formula assumed H_prep = R + L, i.e. prep_theta = 0,
    //  which leaves a small steady-state height offset of L·(1 − cos(prep)).)
    float prep_rad = cfg_.prep_theta_deg * PI / 180.0f;
    float arg      = cosf(prep_rad) - cfg_.step_height_m / cfg_.leg_length_m;
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

    // A4ac: Detect FL/FR BOTH-COMPLETE transition to start back-leg
    // settle window. The chassis is in motion throughout the entire
    // FL/FR CLIMBING phase (beta advancing, chassis moving forward and
    // up via the trajectory) -- back legs see varying load and dynamic
    // torque transients during that whole window. So only arm BL/BR
    // detection after FL/FR are fully at rest in COMPLETE phase.
    //
    // Previously (A4yy): armed on first FL/FR -> CLIMBING. That gave
    // a 1 s window inside an active ~2.5 s climbing dynamic, which was
    // insufficient -- false triggers persisted.
    bool both_front_complete_now = (legs_[0].phase == LegClimbPhase::COMPLETE &&
                                    legs_[1].phase == LegClimbPhase::COMPLETE);
    if (!front_was_complete_ && both_front_complete_now)
    {
        // Just-now both-complete transition: arm the delay window.
        back_settle_remaining_s_ = cfg_.back_settle_s;
        // Snap-reset BL/BR baseline so the LPF starts tracking the new
        // (post-climb-settle) operating point cleanly rather than slowly
        // catching up to a load-shift that accumulated during FL/FR climb.
        legs_[2].torque_baseline   = feedback[2].leg_torque_residual;
        legs_[3].torque_baseline   = feedback[3].leg_torque_residual;
        legs_[2].baseline_warmup_s = 0.0f;  // re-arm fast LPF for re-convergence
        legs_[3].baseline_warmup_s = 0.0f;
        legs_[2].detect_timer_s    = 0.0f;  // clear any partial confirm timer
        legs_[3].detect_timer_s    = 0.0f;
    }
    front_was_complete_ = both_front_complete_now;
    if (back_settle_remaining_s_ > 0.0f)
    {
        back_settle_remaining_s_ -= dt;
        if (back_settle_remaining_s_ < 0.0f)
            back_settle_remaining_s_ = 0.0f;
    }

    for (int i = 0; i < 4; i++)
    {
        LegState &leg              = legs_[i];
        const LegClimbFeedback &fb = feedback[i];

        // Default: no climb-driven chassis motion. Only the CLIMBING case below
        // overrides this with the current ramped phi_w. Chassis reads this via
        // getEffectivePhiW() to cap wheel-motor speed during active climbing.
        leg.climb_phi_w_current = 0.0f;

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
            // Synchronized lift ramp: target_theta_deg smoothsteps from 0 to
            // (180 - prep_theta_deg) over prep_ramp_s. ALL legs share dt and
            // start prep_ramp_t at 0 in startClimb -> they lift in lockstep,
            // so per-leg friction/load differences can't make one side fast
            // and another slow (which was tilting the chassis).
            leg.prep_ramp_t += dt;
            float alpha = leg.prep_ramp_t / cfg_.prep_ramp_s;
            if (alpha > 1.0f)
                alpha = 1.0f;
            float s              = alpha * alpha * (3.0f - 2.0f * alpha);  // smoothstep
            float prep_angle_deg = 180.0f - cfg_.prep_theta_deg;
            target_theta_deg_[i] = s * prep_angle_deg;
            // FFW omega = d(target)/dt = ds/dalpha * (prep_angle/prep_ramp_s), in rad/s.
            // ds/dalpha for smoothstep is 6*alpha*(1-alpha); zero at both ends so
            // accel/decel are gentle.
            if (alpha >= 1.0f)
            {
                target_omega_[i] = 0.0f;
            }
            else
            {
                float ds_dalpha  = 6.0f * alpha * (1.0f - alpha);
                target_omega_[i] = ds_dalpha * prep_angle_deg * (PI / 180.0f) / cfg_.prep_ramp_s;
            }
            target_h_[i] = heightFromTheta(target_theta_deg_[i] * PI / 180.0f);
            target_v_[i] = 0.0f;

            // Warm up torque baseline during PREP so it's stable for DETECT
            leg.torque_baseline = cfg_.baseline_alpha * fb.leg_torque_residual + (1.0f - cfg_.baseline_alpha) * leg.torque_baseline;

            // Transition to DETECT only after the ramp has finished AND the leg
            // is at the prep angle (within tolerance). Without the ramp-complete
            // guard the leg could enter DETECT while still moving.
            float current_theta    = fabsf(fb.leg_pos_deg);
            float prep_motor_angle = prep_angle_deg;
            if (alpha >= 1.0f && fabsf(current_theta - prep_motor_angle) < cfg_.prep_tolerance_deg)
            {
                leg.phase          = LegClimbPhase::DETECT;
                leg.detect_timer_s = 0.0f;
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::DETECT:
        {
            // Hold at detect angle while monitoring torque for step contact.
            // detect_theta_deg > 0 means we entered via startClimbDirect (skip PREP).
            float hold_angle;
            bool is_back_for_hold = (i >= 2);
            bool front_all_complete_for_hold = (legs_[0].phase == LegClimbPhase::COMPLETE &&
                                                legs_[1].phase == LegClimbPhase::COMPLETE);
            if (leg.detect_theta_deg > 0.0f)
            {
                hold_angle = leg.detect_theta_deg;
            }
            else if (is_back_for_hold && front_all_complete_for_hold)
            {
                // A4ab: After FL/FR COMPLETE, override BL/BR hold angle to
                // shift CoM forward (via chassis pitch tilt). offset > 0
                // extends; < 0 shortens (lifts wheels). Safe-range clamped.
                hold_angle = (180.0f - cfg_.prep_theta_deg) + cfg_.back_detect_hold_offset_deg;
                if (hold_angle > 175.0f) hold_angle = 175.0f;
                if (hold_angle <  15.0f) hold_angle =  15.0f;
            }
            else
            {
                hold_angle = 180.0f - cfg_.prep_theta_deg;
            }
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

            // FRONT-BOTH-COMPLETE GATE (A4ac, replaces A4vv's any-front-
            // climbing gate):
            // Back legs (BL=2, BR=3) cannot enter CLIMBING until:
            //   1. BOTH FL and FR are in COMPLETE phase, AND
            //   2. back_settle_remaining_s_ has counted down to 0
            //
            // Why stricter than A4vv: during the entire FL/FR CLIMBING phase
            // (~2.5 s with climb_omega=0.8), the chassis is in motion via the
            // trajectory -- back legs experience time-varying load and
            // dynamic torque transients that easily exceed the residual
            // threshold even with baseline LPF. Earlier "any front climbing"
            // gate plus 1 s settle window proved insufficient. Requiring
            // BOTH front legs at rest in COMPLETE removes the window of
            // vulnerability entirely; back detection only proceeds when the
            // system is genuinely static after front climb finishes.
            bool is_back_leg = (i >= 2);
            bool both_front_complete = (legs_[0].phase == LegClimbPhase::COMPLETE &&
                                        legs_[1].phase == LegClimbPhase::COMPLETE);
            bool back_settle_done    = (back_settle_remaining_s_ <= 0.0f);
            bool front_gate_pass     = !is_back_leg || (both_front_complete && back_settle_done);

            // Step detection: only after warmup AND while the leg is settled.
            // The closed-loop drive torque during motion appears in the residual
            // and would false-trigger CLIMBING, so require low leg speed.
            float deviation = fabsf(fb.leg_torque_residual - leg.torque_baseline);
            bool settled    = (fabsf(fb.leg_vel_radps) < cfg_.detect_settle_omega);
            if (warmed_up && settled && deviation > cfg_.torque_res_threshold && front_gate_pass)
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
                    leg.climb_ramp_t = 0.0f;  // start CLIMBING-entry smoothstep
                }
            }
            else
            {
                leg.detect_timer_s = 0.0f;  // Reset if spike subsides OR front not ready
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::CLIMBING:
        {
            // CLIMBING-entry smoothstep: phi_w starts at 0 and ramps to
            // climb_omega over climb_ramp_s. Avoids step-on jerk that
            // previously sent target_omega from 0 to ~5.5 rad/s in one
            // tick at DETECT->CLIMBING transition, shaking the chassis
            // and false-triggering BL/BR via inertial coupling.
            leg.climb_ramp_t += dt;
            float climb_alpha = (cfg_.climb_ramp_s > 1e-4f) ? (leg.climb_ramp_t / cfg_.climb_ramp_s) : 1.0f;
            if (climb_alpha > 1.0f) climb_alpha = 1.0f;
            float climb_s = climb_alpha * climb_alpha * (3.0f - 2.0f * climb_alpha);  // smoothstep [0..1]
            float phi_w = cfg_.climb_omega * climb_s;  // effective β rate, ramps gently
            leg.climb_phi_w_current = phi_w;  // expose to chassis for wheel-speed capping
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
            //   H_prep     = R + L·cos(prep_theta)
            //   H_complete = H_prep − h          (body descended by one step)
            //   cos(θ_model_complete) = cos(prep_theta) − h / L
            //
            // The unsigned motor angle θ_motor = 180° − θ_model is positive; the
            // caller (Chassis) applies climb_sign[i] to pick the correct branch
            // of the two solutions (FL/FR negative, BL/BR positive — matches the
            // mechanical mounting of the leg motors).
            float theta_complete_rad = computeThetaEnd();
            float theta_complete_deg = theta_complete_rad * 180.0f / PI;

            target_theta_deg_[i] = 180.0f - theta_complete_deg;
            target_omega_[i]     = 0.0f;
            target_h_[i]         = heightFromTheta(theta_complete_rad);
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

    // (No front-first auto-start gate any more: all four legs were already
    //  started in PREP by startClimbAll() so they share the lift ramp; back legs
    //  sit in DETECT after their PREP completes and only enter CLIMBING when
    //  their own wheel hits the step -- so the front-first sequence is still
    //  enforced naturally, without an explicit gate.)
}

}  // namespace Applications
