#pragma once
#include "Comm_Msg.hpp"
#include "Cust_Types.hpp"

namespace Applications
{

class Controller
{
   public:
    Controller() = default;

    /**
     * @brief Map Joystick to Velocity
     * @param msg The received PC message
     * @param vx Reference to output linear velocity
     * @param wz Reference to output angular velocity
     */
    void Map_Joystick_To_Velocity(const Protocol::PC_Msg &msg, float &vx, float &wz);

    /**
     * @brief Reset internal slew-rate state (call when re-entering a mode that
     *        uses Map_Joystick_To_Velocity from a stop).
     */
    void Reset_Velocity_Limiter();

   private:
    // Slew-rate state for vx only. wz passes through unbounded.
    float prev_vx_ = 0.0f;
    float prev_wz_ = 0.0f;
};

}  // namespace Applications
