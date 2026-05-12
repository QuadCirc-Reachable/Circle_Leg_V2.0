#include "Controller.hpp"

#include <cmath>

#include "Robot_Params.hpp"

namespace Applications
{

void Controller::Map_Joystick_To_Velocity(const Protocol::PC_Msg &msg, float &vx, float &wz)
{
    vx = 0.0f;
    wz = 0.0f;

    // Deadzone check
    const uint16_t DEADZONE = 200;

    // Left Stick: Forward/Backward (Vx)
    if (msg.left_joystick.r_x1000_msg > DEADZONE)
    {
        uint16_t angle_l = msg.left_joystick.angle_x10_msg;

        if (angle_l >= 300 && angle_l <= 1500)
        {
            vx = (float)msg.left_joystick.r_x1000_msg / 1000.0f * MAX_FORWARD_RPM;
        }
        else if (angle_l >= 2100 && angle_l <= 3300)
        {
            vx = -(float)msg.left_joystick.r_x1000_msg / 1000.0f * MAX_FORWARD_RPM;
        }
    }

    // Right Stick: Rotation (Wz)
    if (msg.right_joystick.r_x1000_msg > DEADZONE)
    {
        uint16_t angle_r = msg.right_joystick.angle_x10_msg;

        if (angle_r >= 1000 && angle_r <= 2600)
        {
            wz = (float)msg.right_joystick.r_x1000_msg / 1000.0f * MAX_TURN_RPM;
        }
        else if (angle_r <= 800 || angle_r >= 2800)
        {
            wz = -(float)msg.right_joystick.r_x1000_msg / 1000.0f * MAX_TURN_RPM;
        }
    }

    // ----- Slew-rate (acceleration) limits -----
    // Both vx and wz are limited to keep wheel-acceleration bounded.
    // Symmetric accel/decel. (Asymmetric decel was tried to mask the
    // wheel-release shake but the real cause was leg→wheel decoupling
    // positive-feedback in ES; fixed in Wheel_Leg.cpp.)
    // Assumes ~500Hz control loop (vTaskDelay(2)).
    constexpr float DT           = 0.002f;
    const float max_dvx_per_tick = MAX_FWD_ACCEL_RPM_PER_S * DT;
    const float max_dwz_per_tick = MAX_TURN_ACCEL_RPM_PER_S * DT;

    float dvx = vx - prev_vx_;
    if (dvx > max_dvx_per_tick)
        vx = prev_vx_ + max_dvx_per_tick;
    else if (dvx < -max_dvx_per_tick)
        vx = prev_vx_ - max_dvx_per_tick;
    prev_vx_ = vx;

    float dwz = wz - prev_wz_;
    if (dwz > max_dwz_per_tick)
        wz = prev_wz_ + max_dwz_per_tick;
    else if (dwz < -max_dwz_per_tick)
        wz = prev_wz_ - max_dwz_per_tick;
    prev_wz_ = wz;
}

void Controller::Reset_Velocity_Limiter()
{
    prev_vx_ = 0.0f;
    prev_wz_ = 0.0f;
}

}  // namespace Applications
