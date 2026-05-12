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
     * @brief Get leg position command in degrees
     * @return Leg position command in degrees
     */
    float Get_LegPosition();

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
