/**
 * @file    Climbing_Dynamics.cpp
 * @brief   Per-leg step-climbing state machine (IDLE -> PREP -> DETECT -> CLIMBING ->
 *          COMPLETE): torque-residual step detection and kinematic climbing trajectory.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

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

void Climbing_Dynamics::beginClimbing(int idx, float current_motor_deg)
{
    if (idx < 0 || idx > 3)
        return;
    if (legs_[idx].phase != LegClimbPhase::DETECT)
        return;  // only transition from DETECT (don't disturb already-climbing legs)
    legs_[idx].phase = LegClimbPhase::CLIMBING;
    // Seed beta from this leg's CURRENT motor angle (model theta = 180 - |motor|).
    float theta_model_rad = (180.0f - fabsf(current_motor_deg)) * PI / 180.0f;
    if (theta_model_rad < cfg_.theta_min_deg * PI / 180.0f)
        theta_model_rad = cfg_.theta_min_deg * PI / 180.0f;
    legs_[idx].beta        = computeBetaFromTheta(theta_model_rad);
    legs_[idx].climb_ramp_t = 0.0f;  // start CLIMBING-entry smoothstep
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

    // A4au: trigger back_settle on FL/FR FIRST-ENTER-CLIMBING (replaces
    // A4ac's BOTH-COMPLETE trigger).
    //
    // User HW data revealed TWO BL/BR wheel_drop peaks during a front
    // climb:
    //   (1) ~0 to 500 ms after FL/FR DETECT->CLIMBING: FALSE peak --
    //       the climb-entry smoothstep (climb_ramp_s = 0.5 s) ramps
    //       phi_w from 0 to 0.8 rad/s, producing a wheel-leg coupling
    //       transient that decelerates BL/BR wheels (looks like step
    //       contact but is just dynamics).
    //   (2) ~1.5 to 2.5 s after FL/FR DETECT->CLIMBING: REAL peak --
    //       FL/FR climb has advanced the chassis ~3-4 cm forward, BL/BR
    //       wheels reach the step face, true step contact.
    //
    // A4ac's COMPLETE-trigger missed peak (2) entirely (gate opened at
    // ~3.5 s, by which time the LPF baseline had caught up and the
    // signal was gone). A4at's CLIMBING-OR-COMPLETE relaxation without
    // a settle would false-trigger on peak (1).
    //
    // The fix: arm back_settle (1 s) on the CLIMBING-entry edge. The
    // settle blocks peak (1) entirely; once it expires (~1 s into
    // CLIMBING), BL/BR DETECT is enabled and catches peak (2) at
    // ~1.5-1.8 s into CLIMBING. Baseline reset accompanies the trigger
    // so the LPF starts fresh after FL/FR's coupling dynamics settle.
    bool both_front_climbing_or_complete_now =
        (legs_[0].phase == LegClimbPhase::CLIMBING || legs_[0].phase == LegClimbPhase::COMPLETE) &&
        (legs_[1].phase == LegClimbPhase::CLIMBING || legs_[1].phase == LegClimbPhase::COMPLETE);
    if (!front_was_complete_ && both_front_climbing_or_complete_now)
    {
        // Rising edge of "both front legs entered CLIMBING (or COMPLETE)"
        // -- arm the settle window to mask climb-entry coupling transient.
        back_settle_remaining_s_ = cfg_.back_settle_s;
        // Snap-reset BL/BR baseline so the LPF starts tracking the new
        // (post-climb-entry) operating point cleanly rather than slowly
        // catching up to dynamics accumulated during FL/FR's ramp-up.
        legs_[2].torque_baseline   = feedback[2].leg_torque_residual;
        legs_[3].torque_baseline   = feedback[3].leg_torque_residual;
        legs_[2].baseline_warmup_s = 0.0f;  // re-arm fast LPF for re-convergence
        legs_[3].baseline_warmup_s = 0.0f;
        legs_[2].detect_timer_s    = 0.0f;  // clear any partial confirm timer
        legs_[3].detect_timer_s    = 0.0f;
    }
    front_was_complete_ = both_front_climbing_or_complete_now;  // var name kept for ABI stability; semantics: "both front in CLIMBING-or-COMPLETE"
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

        // A4as: clear detection diagnostic outputs by default; DETECT case
        // overwrites with live values. Outside DETECT they read as zero / false
        // in Ozone (helpful: a non-zero score on a leg NOT in DETECT would
        // indicate a logic bug).
        per_leg_t_score_[i]        = 0.0f;
        per_leg_w_score_[i]        = 0.0f;
        per_leg_detect_allowed_[i] = false;

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

            // FRONT-GATE for BL/BR (A4at, relaxed from A4ac):
            // Back legs (BL=2, BR=3) cannot enter CLIMBING until BOTH front
            // legs are in CLIMBING OR COMPLETE phase. Previously required
            // BOTH COMPLETE + 1 s back_settle -- but user data showed BL/BR
            // step contact happens ~1.8 s INTO FL/FR's CLIMBING phase
            // (chassis advances during front climb -> back wheels roll
            // forward and hit the step face). The old gate, which only
            // opened ~3.5 s post-FL-trigger (FL/FR climb time 2.5 s + 1 s
            // settle), missed this signal entirely (LPF baseline caught up
            // by then). Now: as soon as FL/FR start climbing the gate opens,
            // so when BL/BR's real step contact signal arises 1.8 s later
            // it can fire. back_settle still applies for the COMPLETE
            // transition (existing behavior, no-op for the common case where
            // BL/BR triggers during FL/FR CLIMBING and exits DETECT first).
            //
            // Why CLIMBING-OR-COMPLETE (not just CLIMBING): if BL/BR signal
            // arrives slightly after FL/FR COMPLETE, we still want to trigger.
            //
            // False-trigger risk during the initial 0-0.5 s of FL/FR CLIMBING
            // (smoothstep ramp): coupling transients could mimic step
            // contact. Mitigated by combined-score threshold (wheel_drop
            // alone needs >= 12 RPM, which is well above coupling noise).
            bool is_back_leg = (i >= 2);
            auto front_in_climbing_or_complete = [&](int idx) {
                LegClimbPhase p = legs_[idx].phase;
                return p == LegClimbPhase::CLIMBING || p == LegClimbPhase::COMPLETE;
            };
            bool both_front_climbing_or_complete =
                front_in_climbing_or_complete(0) && front_in_climbing_or_complete(1);
            bool back_settle_done    = (back_settle_remaining_s_ <= 0.0f);
            bool front_gate_pass     = !is_back_leg || (both_front_climbing_or_complete && back_settle_done);
            // A4ad: global turning inhibit -- suppress detection while the
            // robot is yawing (differential wheel load fakes a step-contact
            // residual). Applies to both front and back legs.
            bool detect_allowed      = front_gate_pass && !detect_inhibit_;

            // Step detection: TWO independent contact signals (A4ag).
            //
            //  (a) Torque-residual path: leg torque deviates from gravity
            //      baseline by > threshold, while the leg is settled (low
            //      vel). Sensitive but confounded by leg motion / scraping.
            //  (b) Wheel-stall path: the user is commanding the wheel
            //      forward but the wheel RPM is ~0 -> it's jammed against the
            //      step face. Unambiguous "wheel at step" signal, independent
            //      of leg torque AND of the settle gate (a stalled wheel
            //      doesn't require the leg to be still). This is what catches
            //      the back wheel "蹭台阶边缘" case where the leg vibrates
            //      from scraping (failing the settle gate) and/or the
            //      residual is too weak to cross threshold.
            //
            // Either path, sustained for detect_confirm_s while detection is
            // allowed (front-gate + not-turning), confirms contact.
            float deviation = fabsf(fb.leg_torque_residual - leg.torque_baseline);
            bool settled    = (fabsf(fb.leg_vel_radps) < cfg_.detect_settle_omega);
            // A4al + A4aq: per-leg thresholds for torque AND wheel-drop.
            // Front: stronger contact, higher thresholds. Back: weaker
            // residuals, lower thresholds.
            float t_thresh = (i < 2) ? cfg_.torque_res_threshold : cfg_.torque_res_threshold_back;
            float w_thresh = (i < 2) ? cfg_.wheel_drop_threshold : cfg_.wheel_drop_threshold_back;

            // A4aq: COMBINED sum-of-scores detection. Each residual signal
            // is normalized to its threshold (1.0 = at threshold). Trigger
            // when the SUM crosses 1.0. This lets either signal alone fire
            // (if it crosses its own threshold), OR both moderate signals
            // fire together (e.g. each at 0.6 of threshold -> sum 1.2 ->
            // trigger). Per user: "在遇到门槛时候的速度骤降很明显" --
            // combining the obvious wheel decel with the weaker back-leg
            // torque residual gives robust back detection.
            float t_score = settled ? (deviation / t_thresh) : 0.0f;
            float w_score = (fb.wheel_drop > 0.0f) ? (fb.wheel_drop / w_thresh) : 0.0f;
            bool combined_hit = (t_score + w_score >= 1.0f);
            // Plus the wheel-blocked (absolute stall, was_rolling-gated)
            // path as an additional OR fallback for clean full-stall cases.
            bool detect_fires = combined_hit || fb.wheel_blocked;

            // A4as: publish per-leg detection diagnostics (Ozone plot).
            // Reads:
            //   t_score        -- torque residual normalized to threshold (>=1 alone fires)
            //   w_score        -- wheel-drop normalized to threshold       (>=1 alone fires)
            //   combined_score (= t+w) -- crosses 1.0 to trigger (combined_hit)
            //   detect_allowed -- TRUE iff gate (front_gate + !inhibit) open
            // If t+w >= 1 but detect_allowed = false, a gate is blocking.
            per_leg_t_score_[i]        = t_score;
            per_leg_w_score_[i]        = w_score;
            per_leg_detect_allowed_[i] = detect_allowed && warmed_up;
            if (warmed_up && detect_allowed && detect_fires)
            {
                leg.detect_timer_s += dt;
                if (leg.detect_timer_s >= cfg_.detect_confirm_s)
                {
                    // A4ae: COUPLED-PAIR trigger. The two front wheels (or two
                    // back wheels) hit the same step edge together physically.
                    // Per-leg friction / approach-angle differences mean one
                    // leg's residual often crosses threshold noticeably before
                    // the other -- and sometimes the laggard never crosses
                    // ("一个触发一个不触发"). So when EITHER leg of a pair
                    // confirms contact, start BOTH legs of that pair climbing
                    // together. Front pair = {0,1}, back pair = {2,3}. Each
                    // leg's beta is seeded from ITS OWN current motor angle.
                    int pair0 = (i < 2) ? 0 : 2;
                    beginClimbing(pair0,     feedback[pair0].leg_pos_deg);
                    beginClimbing(pair0 + 1, feedback[pair0 + 1].leg_pos_deg);
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
