/**
 * @file    Wheel_Leg.hpp
 * @brief   One CircLeg corner (wheel motor + eccentric leg motor): height <-> angle
 *          conversion, MIT leg command pipeline, wheel velocity loop and wheel-leg
 *          decoupling compensation.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#pragma once
#include "Cust_Types.hpp"
#include "HT8115.hpp"
#include "Helper.hpp"
#include "PID.hpp"
#include "Robot_Params.hpp"

#ifndef USE_6020_LEG_MOTOR
#define USE_6020_LEG_MOTOR 0
#endif

#ifndef USE_HT_LEG_MOTOR
#define USE_HT_LEG_MOTOR 0
#endif

#ifndef USE_DM_LEG_MOTOR
#define USE_DM_LEG_MOTOR 0
#endif

#if (USE_6020_LEG_MOTOR + USE_HT_LEG_MOTOR + USE_DM_LEG_MOTOR) > 1
#error "Cannot enable more than one leg motor type!"
#endif
#if (USE_6020_LEG_MOTOR + USE_HT_LEG_MOTOR + USE_DM_LEG_MOTOR) == 0
#error "Must enable at least one leg motor type!"
#endif

#if USE_6020_LEG_MOTOR
#include "GM6020.hpp"
#elif USE_HT_LEG_MOTOR
// HT8115 already included above for the wheel
#elif USE_DM_LEG_MOTOR
#include "J10010L_2EC.hpp"
#endif

namespace Applications
{

using namespace Core::Drivers;
using namespace Core::Control;
class Wheel_Leg
{
   private:
    //===Wheel Motor Selection===
    // Wheels are HT8115 (MIT-only). Internal velocity loop is used; no external PID.
    Motors::HT8115 *wheel_motor;
    //===Leg Motor Selection===
    Motors::J10010L_2EC *leg_motor;
    MIT_Params mit_set;
    MIT_Params default_mit_set;

    //=== Info Struct===
    Wheel_Leg_Params info;
    float leg_offset = 0.0f;

    //=== Pipeline Variables ===
    float target_wheel_rpm       = 0.0f;
    float wheel_compensation_rpm = 0.0f;
    float final_wheel_rpm        = 0.0f;

    // Leg Control (P-V-F)
    float target_leg_pos   = 0.0f;
    float target_leg_vel   = 0.0f;
    float target_leg_force = 0.0f;

    // Leg Impedance (Stiffness & Damping) - For MIT Mode
    float target_leg_kp = 0.0f;
    float target_leg_kd = 0.0f;

    // Leg Compensation
    float leg_compensation_pos   = 0.0f;
    float leg_compensation_vel   = 0.0f;
    float leg_compensation_force = 0.0f;

    // Final Execution
    float final_leg_pos   = 0.0f;
    float final_leg_vel   = 0.0f;
    float final_leg_force = 0.0f;
    float final_leg_kp    = 0.0f;
    float final_leg_kd    = 0.0f;

    // Slew Rate Limiter State
    float prev_leg_pos_cmd = 0.0f;

    // Continuous (unwrapped) leg-position tracking — A1 infra for the unified
    // computed-torque control. Updated every tick in Execute_Leg_Control by
    // accumulating the wrapped feedback delta, so it never jumps at the +/-180
    // boundary. Lets us (a) report a multi-turn-safe position and (b) pick the
    // nearest 360-equivalent of an absolute target so the leg never unwinds.
    float leg_pos_continuous_   = 0.0f;    // deg, unwrapped (may exceed +/-180)
    float prev_leg_pos_fb_      = 0.0f;    // deg, last wrapped feedback sample
    bool  leg_cont_initialized_ = false;   // seed on first Execute_Leg_Control
    float leg_vel_filt_         = 0.0f;    // rad/s, LPF state for torque-track damping

    // TEMPORARY (FL-slow diagnosis): last Set_Leg_Torque_Track snapshot. Can be
    // removed once FL-vs-FR speed asymmetry is root-caused.
    float tt_omega_cmd_  = 0.0f;           // rad/s, clamped velocity setpoint
    float tt_tau_total_  = 0.0f;           // Nm, gravity + tau_fb

    // Configuration
    int bending_direction_     = 1;  // 1 for Positive Angle solution, -1 for Negative Angle solution
    float wheel_coupling_sign_ = 1.0f;

   public:
    Wheel_Leg(Motors::HT8115 *wheel_motor_,
              Motors::J10010L_2EC *leg_motor_,
              MIT_Params mit_pid_,
              float leg_offset_         = 0.0f,
              int bending_direction     = 1,
              float wheel_coupling_sign = 1.0f);

    //====================//
    //==== Initialize ====//
    //====================//
    /**
     * @brief Initialize Wheel_Leg controller
     */
    void Init();
    //---------------------------------------------------------------------------------------------//

    //==================//
    //====Calculate ====//
    //==================//
    /**
     * @brief Calculate wheel compensation based on leg position and velocity
     * @param leg_current_rpm Current leg motor RPM
     * @param leg_current_pos Current leg motor position in radians
     * @return Compensation rpm to be added to wheel motor control
     */
    float Wheel_Compensation();

    /**
     * @brief Feedforward wheel compensation from a COMMANDED leg angular
     *        velocity (rad/s). Use this when the caller already has the
     *        commanded leg omega in hand (e.g. climbing trajectory FFW or a
     *        smoothstep derivative); it avoids the encoder-feedback lag of
     *        the no-arg Wheel_Compensation() that reads leg_motor->getRPMFeedback().
     *        The lag would let the wheel motor trail the leg's acceleration
     *        during PREP startup -> ground scrub. Position factor cos(theta)
     *        still uses the actual motor angle (true instantaneous geometry).
     *
     * @param leg_omega_radps Commanded leg angular velocity in motor frame
     *                        (signed rad/s, same sign convention as the
     *                        omega_ff passed to Set_Leg_Torque_Track).
     * @return Compensation rpm to be added to wheel motor control.
     */
    float Wheel_Compensation(float leg_omega_radps);

    /**
     * @brief Calculate motor torque required for a given vertical force (VMC)
     * @param F_z Vertical force in Newtons
     * @return Required motor torque in N-m
     */
    float VMC_Calculation(float F_z);

    //---------------------------------------------------------------------------------------------//

    //=================//
    //==== Getters ====//
    //=================//
    /**
     * @brief Get wheel RPM command
     * @return Wheel RPM command
     */
    float Get_WheelRPM();
    /**
     * @brief Leg angle wrapped to the single-turn range (-180, 180] degrees.
     * This is the raw encoder angle (leg_offset removed, then normalized). It
     * JUMPS at the +/-180 boundary, so do NOT difference it across the seam --
     * use Get_LegAngleUnwrapped() for any control error near the top pose.
     */
    float Get_LegAngleWrapped();

    /**
     * @brief Continuous, multi-turn leg angle in degrees (NEVER wraps).
     * Maintained in software by accumulating the per-tick wrapped-feedback
     * delta in Execute_Leg_Control(), so it is safe to difference anywhere,
     * including across the +/-180 seam. Use this for the unified torque control.
     */
    float Get_LegAngleUnwrapped() const { return leg_pos_continuous_; }

    /**
     * @brief Nearest 360-equivalent of an absolute target angle, in the
     * continuous frame. Leg height H = R - r*cos(theta) is 360-periodic, so any
     * theta+/-360k is the same height; returning the representative within +/-180
     * of the current continuous position lets a caller command a pose without
     * ever forcing a multi-turn unwind (passenger-safety critical).
     * @param target_deg Desired absolute angle (deg, any range)
     * @return target expressed in the continuous frame
     */
    float NearestEquivalentTarget(float target_deg) const;

    // TEMPORARY (FL-slow diagnosis): last torque-track snapshot accessors.
    float Get_TT_OmegaCmd() const { return tt_omega_cmd_; }
    float Get_TT_OmegaFb()  const { return leg_vel_filt_; }
    float Get_TT_TauTotal() const { return tt_tau_total_; }

    /**
     * @brief Get wheel motor raw current feedback (Amps)
     * @return Wheel motor current in Amps
     */
    float Get_WheelCurrentFeedback();

    /**
     * @brief Get wheel motor temperature (°C)
     */
    float Get_WheelTemperature();

    /**
     * @brief Get wheel motor output command (raw CAN value)
     */
    float Get_WheelOutput();

    /**
     * @brief Get final wheel RPM target (after compensation)
     */
    float Get_FinalWheelRPM() const { return final_wheel_rpm; }

    /**
     * @brief Get final leg command angle (deg) after slew rate limiter
     */
    float Get_FinalLegCommand() const { return final_leg_pos; }

    /**
     * @brief Get leg motor raw current feedback (Amps)
     * @return Motor current in Amps
     */
    float Get_LegCurrentFeedback();

    /**
     * @brief Get leg torque feedback from motor
     * @return Leg torque in N-m
     */
    float Get_LegTorqueFeedback();

    /**
     * @brief Get the torque required to compensate for leg gravity
     * @return Gravity compensation torque in N-m (usually negative if holding leg up)
     */
    float Get_LegGravityTorque();

    /**
     * @brief Get leg force command in N-m
     * @return Leg force command in N-m
     */
    float Get_LegForce();
    /**
     * @brief Get leg velocity command in rad/s
     * @return Leg velocity command in rad/s
     */
    float Get_LegVelocity();
    /**
     * @brief Get both wheel and leg commands
     * @return Struct containing wheel and leg commands
     */
    Wheel_Leg_Params Get_Info();

    //---------------------------------------------------------------------------------------------//

    //=================//
    //==== Setters ====//
    //=================//
    /**
     * @brief Set target wheel RPM (Stage 1 of Pipeline)
     * @param rpm_cmd Target RPM
     */
    void Set_Wheel_Target(float rpm_cmd);

    /**
     * @brief Add compensation to wheel RPM (Stage 2 of Pipeline)
     * @param comp_rpm Compensation RPM to add
     */
    void Add_Wheel_Compensation(float comp_rpm);

    /**
     * @brief Execute wheel control loop (Stage 3 of Pipeline)
     */
    void Execute_Wheel_Control();

#if USE_6020_LEG_MOTOR
    /**
     * @brief Set leg position command in degrees (Stage 1 of Pipeline)
     * @param pos_cmd Target position in degrees
     * @param vel_cmd Target velocity in rad/s (Feedforward)
     * @param for_cmd Target force in N-m (Feedforward)
     */
    void Set_Leg_Target(float pos_cmd, float vel_cmd = 0.0f, float for_cmd = 0.0f);

    /**
     * @brief Set leg height (Active Suspension)
     * @param h_meters Target height in meters (relative to BASE_LEG_POS level)
     *                 Positive = Up (Extend), Negative = Down (Retract)
     * @param v_meters_s Target vertical velocity in m/s (Feedforward)
     */
    void Set_Leg_Height(float h_meters, float v_meters_s = 0.0f);
#elif USE_HT_LEG_MOTOR
    /**
     * @brief Set leg MIT command (Stage 1 of Pipeline)
     * @param pos_cmd Target position in degrees
     * @param vel_cmd Target velocity in rad/s
     * @param for_cmd Target force in N-m
     * @param kp Position Gain
     * @param kd Velocity Gain
     */
    void Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd, float kp = 0.0f, float kd = 0.0f);

    /**
     * @brief Set leg height (Active Suspension)
     * @param h_meters Target height in meters (relative to BASE_LEG_POS level)
     *                 Positive = Up (Extend), Negative = Down (Retract)
     * @param v_meters_s Target vertical velocity in m/s (Feedforward)
     */
    void Set_Leg_Height(float h_meters, float v_meters_s = 0.0f);

    /**
     * @brief Set leg height with impedance override (Variable Impedance Control)
     * @param h_meters    Target height in meters
     * @param v_meters_s  Target vertical velocity in m/s
     * @param kp          MIT Kp override (N·m/rad)
     * @param kd          MIT Kd override (N·m·s/rad)
     * @param ffw_torque  FFW torque override (Nm) — replaces default gravity comp
     */
    void Set_Leg_Height(float h_meters, float v_meters_s, float kp, float kd, float ffw_torque);

    /**
     * @brief Set current position as zero
     */
    void SetZero();

    /**
     * @brief Re-send ENTER_MOTOR to HT leg motor (idempotent, safe to call repeatedly)
     */
    void EnterMotorMode();
#elif USE_DM_LEG_MOTOR
    /**
     * @brief Set leg MIT command (Stage 1 of Pipeline) for DM J10010L_2EC
     * @param pos_cmd Target position in degrees
     * @param vel_cmd Target velocity in rad/s
     * @param for_cmd Target feed-forward torque in N·m (passed directly to DM ffw)
     * @param kp Position gain (N·m/rad), clamped [0, 500]
     * @param kd Velocity damping (N·m·s/rad), clamped [0, 5]
     */
    void Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd, float kp = 0.0f, float kd = 0.0f);

    /**
     * @brief Set leg height (Active Suspension)
     */
    void Set_Leg_Height(float h_meters, float v_meters_s = 0.0f);

    /**
     * @brief Set leg height with impedance override
     */
    void Set_Leg_Height(float h_meters, float v_meters_s, float kp, float kd, float ffw_torque);

    /**
     * @brief Set current position as zero (writes flash on DM — calibrate sparingly)
     */
    void SetZero();
#endif

    /**
     * @brief Add compensation to leg command (Stage 2 of Pipeline)
     * @param comp_pos Position compensation in degrees
     * @param comp_vel Velocity compensation in rad/s
     * @param comp_force Force compensation in N-m
     */
    void Add_Leg_Compensation(float comp_pos, float comp_vel, float comp_force);

    /**
     * @brief Unified computed-torque leg control (passenger-safe, wrap-free).
     *
     * Software velocity loop: continuous-frame position error -> speed-limited
     * velocity setpoint -> torque, sent ENTIRELY as MIT FFW with Pos_KP = 0 and
     * Vel_KD = 0. The DM never runs its internal loops, so it can never unwind
     * multiple turns at the +/-180 seam. The error uses the unwrapped angle vs.
     * the nearest 360-equivalent target, so the leg always takes the short way
     * to an equal-height pose. The omega_max clamp bounds leg speed, which both
     * protects the passenger (no slamming) and keeps the motion torque from
     * swamping step detection. Only cost vs. the motor's internal loop is the
     * 500 Hz software rate.
     *
     * @param target_deg  Desired absolute leg angle (deg). Caller applies any
     *                    per-leg bending sign before calling.
     * @param omega_ff    Trajectory angular-velocity feed-forward (rad/s).
     * @param kp_pos      Position->velocity gain (1/s).
     * @param omega_max   Hard clamp on the velocity setpoint (rad/s) -- speed limit.
     * @param kd_vel      Velocity->torque gain (Nm.s/rad).
     * @param tau_max     Clamp on the velocity-loop torque term (Nm), excl. ffw.
     * @param ffw_torque  Feed-forward torque (Nm) -- gravity / load comp.
     *                    Pass 0 to disable. Caller is responsible for sign
     *                    (this primitive adds it as-is to the velocity-loop tau).
     */
    void Set_Leg_Torque_Track(float target_deg, float omega_ff, float kp_pos, float omega_max, float kd_vel, float tau_max, float ffw_torque);

    /**
     * @brief Direct PD-as-torque leg control (impedance-friendly).
     *
     * Computes the same control law as the motor's internal MIT loop:
     *   tau = kp*(target_cont - theta_cont) + kd*(omega_ff - omega_fb) + ffw
     * but in software and sends it via FFW only (Pos_KP = Vel_KD = 0), so the
     * DM never runs its position loop and can never unwind multi-turn. Units
     * are identical to MIT (Nm/rad, Nm.s/rad), so impedance gains drop in 1:1.
     * NearestEquivalentTarget keeps the leg single-turn safe.
     *
     * Use this when you want STIFF position holding with caller-supplied Kp/Kd
     * (COMFORT impedance, HOMING ramps). For SPEED-LIMITED trajectory tracking
     * (CLIMBING) use Set_Leg_Torque_Track instead.
     *
     * @param target_deg Desired absolute leg angle (deg). Caller applies any
     *                   per-leg bending sign.
     * @param omega_ff   Velocity feed-forward (rad/s).
     * @param kp         Position stiffness (Nm/rad) -- holds steady state.
     * @param kd         Velocity damping (Nm.s/rad) -- limits speed naturally
     *                   via the kd*(omega_ff - omega_fb) term. With tau_max
     *                   raised to the motor's hardware ceiling, this kd*omega
     *                   restoring term is the ONLY thing that keeps a big
     *                   position error from snapping the leg violently, so kd
     *                   must be tuned for the desired damping ratio.
     * @param ffw        Feed-forward torque (Nm) -- gravity comp, warp, etc.
     * @param tau_max    Safety clamp on the total commanded torque (Nm).
     */
    void Set_Leg_PD_Torque(float target_deg, float omega_ff, float kp, float kd, float ffw, float tau_max);

    /**
     * @brief Execute leg control loop (Stage 3 of Pipeline)
     */
    void Execute_Leg_Control();

    /**
     * @brief Set both wheel and leg commands and execute immediately (Legacy/Convenience)
     * @param cmd Struct containing wheel and leg commands
     */
    void Set_Wheel_Leg(Wheel_Leg_Params cmd);

    void Set_Bending_Direction(int dir) { bending_direction_ = dir; }
    int Get_Bending_Direction() const { return bending_direction_; }

    //---------------------------------------------------------------------------------------------//
};
}  // namespace Applications
