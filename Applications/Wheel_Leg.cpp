#include "Wheel_Leg.hpp"

#include <cmath>

namespace Applications
{

Wheel_Leg::Wheel_Leg(Motors::HT8115 *wheel_motor_,
                     Motors::J10010L_2EC *leg_motor_,
                     MIT_Params mit_pid_,
                     float leg_offset_,
                     int bending_direction,
                     float wheel_coupling_sign)
    : wheel_motor(wheel_motor_),
      leg_motor(leg_motor_),
      leg_offset(leg_offset_),
      bending_direction_(bending_direction),
      wheel_coupling_sign_(wheel_coupling_sign)
{
    // For DM motors, FFW_Current field is repurposed to carry feed-forward TORQUE (Nm)
    // because DM's MIT frame already encodes torque directly (no current→torque conversion).
    mit_set.Position    = 0.0f;
    mit_set.Velocity    = 0.0f;
    mit_set.Pos_KP      = mit_pid_.Pos_KP;
    mit_set.Vel_KD      = mit_pid_.Vel_KD;
    mit_set.FFW_Current = 0.0f;

    default_mit_set = mit_set;

    info.Wheel_RPM = 0.0f;
    info.Leg_POS   = 0.0f;
    info.Leg_Force = 0.0f;
    info.Leg_RPM   = 0.0f;
}

void Wheel_Leg::Init()
{
    if (wheel_motor)
    {
        wheel_motor->enable();
        // HT8115 wheels also need ENTER_MOTOR ritual to accept MIT frames
        for (int i = 0; i < 20; i++)
        {
            wheel_motor->sendCommand(Motors::HT8115::SpecialCommands::ENTER_MOTOR);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    if (leg_motor)
    {
        // DM motors enter MIT mode directly via enable() (sends 0xFFFFFFFFFFFFFFFC)
        leg_motor->enable();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // Initialize Slew Rate Limiter to current position
    if (leg_motor)
    {
        // Wait a bit for feedback to update
        vTaskDelay(pdMS_TO_TICKS(10));
        prev_leg_pos_cmd = Get_LegPosition();
    }
}

float Wheel_Leg::Wheel_Compensation()
{
    float leg_rpm = 0.0f;
    float leg_pos = 0.0f;

    /**
     * @brief Decoupling Compensation Formula
     *
     *                               r_offset
     * n_Wheel = n_Leg * ( 1.0 +  -------------- * cos(theta_Leg) )
     *                              R_Wheel
     */

    if (leg_motor)
    {
        leg_rpm = leg_motor->getRPMFeedback();
        leg_pos = rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
    }

    float compensation_rpm = leg_rpm * (1.0f + (ECCENTRIC_OFFSET_r / WHEEL_RADIUS_R) * cosf(deg2rad(leg_pos)));
    return compensation_rpm * wheel_coupling_sign_;
}

float Wheel_Leg::VMC_Calculation(float F_z)
{
    float leg_pos   = Get_LegPosition();  // Degrees
    float theta_rad = deg2rad(leg_pos);

    // Jacobian: Torque = F_z * r * cos(theta)
    // ECCENTRIC_OFFSET_r is in mm, convert to m
    float torque = F_z * (ECCENTRIC_OFFSET_r / 1000.0f) * cosf(theta_rad);

    return torque;
}

float Wheel_Leg::Get_WheelRPM()
{
    if (wheel_motor)
        return wheel_motor->getRPMFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegPosition()
{
    if (leg_motor)
        return rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
    return 0.0f;
}

float Wheel_Leg::Get_WheelCurrentFeedback()
{
    if (wheel_motor)
        return wheel_motor->getCurrentFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_WheelTemperature()
{
    if (wheel_motor)
        return wheel_motor->getTemperatureFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_WheelOutput()
{
    // HT8115 is MIT-only and does not support getOutput().
    // Use current feedback as a proxy for "output magnitude".
    if (wheel_motor)
        return wheel_motor->getCurrentFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegCurrentFeedback()
{
    if (leg_motor)
        return leg_motor->getCurrentFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegTorqueFeedback()
{
    if (leg_motor)
        return leg_motor->getTorqueFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegGravityTorque()
{
    // Calculate torque needed to hold the leg against gravity
    // Torque = - (m * g * r * cos(theta))
    // Assuming Positive Torque pushes DOWN (extends).
    // Gravity pushes DOWN.
    // So to hold it UP, we need Negative Torque.

    float pos_deg   = Get_LegPosition();
    float theta_rad = deg2rad(pos_deg);
    float r_meter   = ECCENTRIC_OFFSET_r / 1000.0f;

    // Gravity Torque (Effect of gravity) = m * g * r * cos(theta)
    // This is Positive (pushes down).
    // Compensation Torque (Motor Output) = - Gravity Torque

    return -(LEG_MASS_kg * GRAVITY_g * r_meter * cosf(theta_rad));
}

float Wheel_Leg::Get_LegForce()
{
    if (leg_motor)
        return leg_motor->getTorqueFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegVelocity()
{
    if (leg_motor)
        return leg_motor->getRPMFeedback() * 2.0f * M_PI / 60.0f;
    return 0.0f;
}

Wheel_Leg_Params Wheel_Leg::Get_Info()
{
    info.Wheel_RPM = Get_WheelRPM();
    info.Leg_POS   = Get_LegPosition();
    info.Leg_Force = Get_LegForce();
    info.Leg_RPM   = Get_LegVelocity();
    return info;
}

// === Wheel Pipeline ===

void Wheel_Leg::Set_Wheel_Target(float rpm_cmd)
{
    target_wheel_rpm       = rpm_cmd;
    wheel_compensation_rpm = 0.0f;  // Reset compensation for new cycle
}

void Wheel_Leg::Add_Wheel_Compensation(float comp_rpm) { wheel_compensation_rpm += comp_rpm; }

void Wheel_Leg::Execute_Wheel_Control()
{
    final_wheel_rpm = target_wheel_rpm + wheel_compensation_rpm;

    if (wheel_motor)
    {
        // HT8115 wheel control: pure velocity mode via internal velocity loop.
        // setMIT(posCmd=0, velCmd=target_rad_s, Kp=0, Kd=WHEEL_KD, ffw=0)
        // Kp=0 disables position tracking; Kd alone tracks velocity.
        //
        // Deadzone: only suppress torque when BOTH the command and the actual
        // wheel speed are essentially zero. We keep the threshold small (2 RPM)
        // so a slewed startup ramps the velocity command continuously through
        // small values — no sudden step from 0 to a 15 RPM target. The
        // velocity-feedback term in the OR avoids torque disable while the
        // wheel is still coasting.
        constexpr float WHEEL_RPM_DEADZONE = 2.0f;
        constexpr float WHEEL_KD           = 2.0f;  // N·m·s/rad. Tuning history:
                                                    //   1.5 → static friction stall in turns
                                                    //   3.0 → strong brake torque on release
                                                    //         excites chassis pitch (inertia
                                                    //         carries wheel forward → big
                                                    //         neg error → big brake → reaction
                                                    //         pitches chassis → loop)
                                                    //   2.0 → compromise. HT8115 Kd_max=5.0.
        constexpr float RPM_TO_RADS = 2.0f * (float)M_PI / 60.0f;

        const float vel_fb_rpm = wheel_motor->getRPMFeedback();

        if (fabsf(final_wheel_rpm) < WHEEL_RPM_DEADZONE && fabsf(vel_fb_rpm) < WHEEL_RPM_DEADZONE)
        {
            // Idle: zero MIT command (no torque)
            wheel_motor->setMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        else
        {
            const float vel_target_rad_s = final_wheel_rpm * RPM_TO_RADS;
            wheel_motor->setMIT(0.0f, vel_target_rad_s, 0.0f, WHEEL_KD, 0.0f);
        }
        wheel_motor->transmit();
    }
}

// === Leg Pipeline ===

void Wheel_Leg::Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd, float kp, float kd)
{
    target_leg_pos   = pos_cmd;
    target_leg_vel   = vel_cmd;
    target_leg_force = for_cmd;

    mit_set.Pos_KP = kp;
    mit_set.Vel_KD = kd;

    leg_compensation_pos   = 0.0f;
    leg_compensation_vel   = 0.0f;
    leg_compensation_force = 0.0f;
}

void Wheel_Leg::Set_Leg_Height(float h_meters, float v_meters_s)
{
    // Convert Height (m) -> Angle (deg).
    //
    // NEW convention (after DM zero-point was calibrated at the LOWEST pose):
    //   theta = 0       ⇒ LOWEST  (H = R - r)
    //   theta = ±180°    ⇒ HIGHEST (H = R + r)
    // Model:  H = R - r * cos(theta)
    //
    // bending_direction_ then chooses whether to express theta as positive or
    // negative when sending to the motor (mechanical mounting may flip the
    // sign of one or more legs).

    float R = WHEEL_RADIUS_R / 1000.0f;      // m
    float r = ECCENTRIC_OFFSET_r / 1000.0f;  // m

    // Reachable range (with 10° margin away from singularities at theta=0/180).
    //   H_min = R - r * cos(10°)   (very low, but not at the singularity)
    //   H_max = R + r * cos(10°)
    float limit_angle_deg = 10.0f;
    float limit_cos       = cosf(deg2rad(limit_angle_deg));
    float max_h           = R + r * limit_cos;
    float min_h           = R - r * limit_cos;

    bool is_clamped_max = false;
    bool is_clamped_min = false;
    if (h_meters > max_h)
    {
        h_meters       = max_h;
        is_clamped_max = true;
    }
    if (h_meters < min_h)
    {
        h_meters       = min_h;
        is_clamped_min = true;
    }
    if (is_clamped_max && v_meters_s > 0.0f)
        v_meters_s = 0.0f;
    if (is_clamped_min && v_meters_s < 0.0f)
        v_meters_s = 0.0f;

    // Solve  cos(theta) = (R - H) / r   (flip vs old convention).
    float cos_theta = (R - h_meters) / r;
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;

    float theta_rad = acosf(cos_theta);  // [0, pi], 0 = lowest
    float theta_deg = rad2deg(theta_rad);
    float sin_theta = sinf(theta_rad);

    // dH/dtheta = +r*sin(theta)  ⇒  dtheta/dt = (dH/dt) / (r*sin(theta))
    // (sign flipped vs old formula).
    float damping_val      = 0.1f;
    float target_vel_rad_s = v_meters_s / (r * (sin_theta + damping_val));

    // Per-leg bending direction handles mechanical mounting sign.
    float target_angle = theta_deg * (float)bending_direction_;
    target_vel_rad_s *= (float)bending_direction_;

    float gravity_comp = Get_LegGravityTorque();

    Set_Leg_Target(target_angle, target_vel_rad_s, gravity_comp, default_mit_set.Pos_KP, default_mit_set.Vel_KD);
}

#if (USE_HT_LEG_MOTOR || USE_DM_LEG_MOTOR)
void Wheel_Leg::Set_Leg_Height(float h_meters, float v_meters_s, float kp, float kd, float ffw_torque)
{
    // Same height→angle conversion as the default overload,
    // but uses caller-provided Kp, Kd, and FFW instead of defaults.

    float R = WHEEL_RADIUS_R / 1000.0f;
    float r = ECCENTRIC_OFFSET_r / 1000.0f;

    float limit_angle_deg = 10.0f;
    float limit_cos       = cosf(deg2rad(limit_angle_deg));
    float max_h           = R + r * limit_cos;
    float min_h           = R - r * limit_cos;

    if (h_meters > max_h)
    {
        h_meters = max_h;
        if (v_meters_s > 0.0f)
            v_meters_s = 0.0f;
    }
    if (h_meters < min_h)
    {
        h_meters = min_h;
        if (v_meters_s < 0.0f)
            v_meters_s = 0.0f;
    }

    // Solve  cos(theta) = (R - H) / r  (new convention, theta=0 ⇔ lowest)
    float cos_theta = (R - h_meters) / r;
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;

    float theta_rad = acosf(cos_theta);
    float theta_deg = rad2deg(theta_rad);
    float sin_theta = sinf(theta_rad);

    float damping_val      = 0.1f;
    float target_vel_rad_s = v_meters_s / (r * (sin_theta + damping_val));

    float target_angle = theta_deg * (float)bending_direction_;
    target_vel_rad_s *= (float)bending_direction_;

    Set_Leg_Target(target_angle, target_vel_rad_s, ffw_torque, kp, kd);
}
#endif

void Wheel_Leg::Add_Leg_Compensation(float comp_pos, float comp_vel, float comp_force)
{
    leg_compensation_pos += comp_pos;
    leg_compensation_vel += comp_vel;
    leg_compensation_force += comp_force;
}

void Wheel_Leg::Execute_Leg_Control()
{
    // All leg angles are kept inside [-180°, 180°] throughout the pipeline.
    // (Project convention: any 2π equivalent collapses to the single-turn
    // representative. Mismatches between accumulated prev_leg_pos_cmd and the
    // wrapped feedback used to cause ±180° boundary stalls / direction flips.)

    auto wrap180 = [](float a)
    {
        while (a > 180.0f)
            a -= 360.0f;
        while (a <= -180.0f)
            a += 360.0f;
        return a;
    };

    float raw_target_pos = wrap180(target_leg_pos + leg_compensation_pos);
    float prev           = wrap180(prev_leg_pos_cmd);

    // Shortest signed diff in (-180, 180]
    float diff = wrap180(raw_target_pos - prev);

    // Slew Rate Limiter (Ramp) — 2 ms tick (500 Hz)
    float dt        = 0.002f;
    float max_delta = LEG_MAX_SPEED * dt;
    if (diff > max_delta)
        diff = max_delta;
    else if (diff < -max_delta)
        diff = -max_delta;

    float new_cmd    = wrap180(prev + diff);
    prev_leg_pos_cmd = new_cmd;
    final_leg_pos    = new_cmd;

    final_leg_vel   = target_leg_vel + leg_compensation_vel;
    final_leg_force = target_leg_force + leg_compensation_force;

    if (leg_motor)
    {
        // DM J10010L_2EC MIT command position range is [-pMax, +pMax] = [-π, +π]
        // (single-turn). DM internally has its own multi-turn absolute encoder
        // and resolves the wrapped command correctly. We MUST NOT do our own
        // "multi-turn rewinding" using the wrapped feedback — when leg_offset
        // sits near ±π and feedback naturally wraps across the boundary, that
        // logic flips the command between +π and −π every loop, causing the
        // motor to spin half a turn back and forth (observed on all four legs
        // in ENERGY_SAVING mode).
        //
        // Fix: just normalize the target into [-π, π] and send it once.
        float target_pos_rad = deg2rad(final_leg_pos) + leg_offset;
        target_pos_rad       = normalizeAngle(target_pos_rad);

        mit_set.Position = target_pos_rad;
        mit_set.Velocity = final_leg_vel;
        // DM ffw is feed-forward TORQUE (Nm) directly — no current conversion.
        mit_set.FFW_Current = final_leg_force;

        leg_motor->setMIT(mit_set.Position, mit_set.Velocity, mit_set.Pos_KP, mit_set.Vel_KD, mit_set.FFW_Current);
        leg_motor->transmit();
    }
}

void Wheel_Leg::SetZero()
{
    // WARNING: DM setZeroPosition writes flash. Calibrate sparingly via DM Tool
    // or one-shot at bring-up; not every boot.
    if (leg_motor)
    {
        leg_motor->setZeroPosition();
    }
}

void Wheel_Leg::Set_Wheel_Leg(Wheel_Leg_Params cmd)
{
    if (cmd.state == Chassis_State::IDLE)
    {
        // Safe IDLE / disconnect hold:
        //   - Wheel target 0 (Execute_Wheel_Control deadzone kills torque once
        //     stopped; light Kd damping coasts wheels to a smooth stop).
        //   - Legs: PURE DAMPING (Kp = 0, Kd > 0).
        //       • No position hold ⇒ user can freely reposition the leg by hand
        //         (calibration, repair, manual move) without fighting the motor.
        //       • Avoids the ±π boundary fight: with Kp>0, target=cur_pos
        //         crossing ±π produces an ambiguous wrap (DM may see ~2π error)
        //         and the leg shakes near the boundary.
        //       • Kd alone damps gravity-driven swing, so the leg doesn't flop
        //         freely.
        //   - Re-entry into ES/COMFORT from IDLE is protected by the Kp ramp
        //     in Chassis::Set_Mode, so 0 → 60 is smooth.
        cmd.Wheel_RPM = 0.0f;
        cmd.Leg_POS   = 0.0f;  // unused when Kp=0
        cmd.Leg_RPM   = 0.0f;
        cmd.Leg_Force = 0.0f;
        cmd.Leg_Kp    = 0.0f;  // no position hold
        cmd.Leg_Kd    = 3.0f;  // heavy damping → leg slowly settles to lowest pose under gravity

        // Keep slew-rate state in sync with the physical pose so that when we
        // leave IDLE the first ramped command starts from "where the leg is",
        // not from a stale value.
        if (leg_motor)
            prev_leg_pos_cmd = Get_LegPosition();
    }

    // 1. Set Targets
    Set_Wheel_Target(cmd.Wheel_RPM);
    Set_Leg_Target(cmd.Leg_POS, cmd.Leg_RPM, cmd.Leg_Force, cmd.Leg_Kp, cmd.Leg_Kd);

    // 2. Add Compensations
    // Internal Decoupling Compensation
    float decouple_comp = Wheel_Compensation();
    Add_Wheel_Compensation(decouple_comp);

    // 3. Execute Control
    Execute_Wheel_Control();
    Execute_Leg_Control();
}
}  // namespace Applications