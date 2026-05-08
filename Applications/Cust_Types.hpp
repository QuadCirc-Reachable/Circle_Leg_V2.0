#pragma once
namespace Applications
{

enum class Chassis_State
{
    CALIBRATION,    // Calibration State (State 0)
    IDLE,           // Idle State
    ENERGY_SAVING,  // Energy Saving Mode
    COMFORT,        // Comfort/Active Suspension Mode
    CLIMBING,       // Climbing Mode
    FREE_CONTROL,   // Free Control/Debug Mode
    DEBUG,          // Debug Mode: X/Y/A/B → leg position, no suspension
    ERROR           // Error State
};

struct MIT_Params
{
    float Position;     // -95.5 to 95.5 rad
    float Velocity;     // -45 to 45 rad/s
    float Pos_KP;       // 0 to 500 N-m/rad
    float Vel_KD;       // 0 to 5 N-m/rad/s
    float FFW_Current;  // -18 to 18 Amps
};

/**
 * @struct Wheel_Leg_State
 * @brief Struct to hold the state information of wheel and leg
 */
struct Wheel_Leg_Params
{
    /**
     * @brief Wheel RPM
     * -Get: Actual wheel RPM feedback
     * -Set: Target wheel RPM
     */
    float Wheel_RPM;
    /**
     * @brief Leg position in degrees
     * -Get: Actual leg position feedback
     * -Set: Target leg position
     */
    float Leg_POS;
    /**
     * @brief Leg force in N-m
     * -Set: Feedforward force
     * -Get: Actual force feedback
     */
    float Leg_Force;
    /**
     * @brief Leg velocity in rad/s
     * -Set: Target velocity
     * -Get: Actual velocity feedback
     */
    float Leg_RPM;
    /**
     * @brief Leg Position Gain (for HT8115)
     * -Set: Position Gain
     */
    float Leg_Kp;
    /**
     * @brief Leg Velocity Gain (for HT8115)
     * -Set: Velocity Gain
     */
    float Leg_Kd;

    Chassis_State state;
};

}  // namespace Applications
