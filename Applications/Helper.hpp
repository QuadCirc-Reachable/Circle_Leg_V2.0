#pragma once
#include "Comm_Msg.hpp"
#include "Robot_Params.hpp"
#define M_PI_F 3.1415926f

// helper functions
inline float normalizeAngle(float angle)
{
    angle -= (int)(angle / ((2.0f * M_PI_F))) * (2.0f * M_PI_F);

    if (angle > M_PI_F)
        angle -= 2 * M_PI_F;
    else if (angle < -M_PI_F)
        angle += 2 * M_PI_F;

    return angle;
}

inline float deg2rad(float deg) { return deg * M_PI_F / 180.0f; }

inline float rad2deg(float rad) { return rad * 180.0f / M_PI_F; }

inline float Calculate_Wheel_RPM(const Protocol::Joystick_Info &joy)
{
    // Decode angle
    float angle_deg = static_cast<float>(joy.angle_x10_msg) / 10.0f;

    // Decode radius
    float r_val = static_cast<float>(joy.r_x1000_msg);

    // Direction
    int8_t direction = 0;

    // NA
    if (joy.angle_x10_msg < 0 || joy.r_x1000_msg < JOYSTICK_DEADZONE)
    {
        return 0;
    }
    // Forward [30, 150]
    else if (angle_deg >= ANGLE_FWD_MIN && angle_deg <= ANGLE_FWD_MAX)
    {
        direction = 1;
    }
    // Backward [210, 330]
    else if (angle_deg >= ANGLE_BWD_MIN && angle_deg <= ANGLE_BWD_MAX)
    {
        direction = -1;
    }
    else
    {
        return 0;
    }

    // Limit
    if (r_val > JOYSTICK_MAX_R)
        r_val = JOYSTICK_MAX_R;
    if (r_val < 0.0f)
        r_val = 0.0f;

    // Calculate RPM
    return (r_val / JOYSTICK_MAX_R) * MAX_WHEEL_RPM * direction;
}

inline void Set_Leg_Pos_by_Buttons(Protocol::PC_Msg *pc_msg_chassis, float *leg_set_pos, bool &is_free_mode, uint8_t &last_button_status)
{
    if (pc_msg_chassis->button_status & BTN_A)
    {
        if (!(last_button_status & BTN_A))
        {
            is_free_mode = !is_free_mode;  // Toggle free mode
        }
    }
    else if (pc_msg_chassis->button_status & BTN_B)
    {
        leg_set_pos[0] = 90.0f;  // ID1
        leg_set_pos[1] = 90.0f;  // ID2
        leg_set_pos[2] = 90.0f;  // ID3
        leg_set_pos[3] = 90.0f;  // ID4
    }
    else if (pc_msg_chassis->button_status & BTN_Y)
    {
        leg_set_pos[0] = 180.0f;  // ID1
        leg_set_pos[1] = 180.0f;  // ID2
        leg_set_pos[2] = 180.0f;  // ID3
        leg_set_pos[3] = 180.0f;  // ID4
    }
    else if (pc_msg_chassis->button_status & BTN_X)
    {
        leg_set_pos[0] = -90.0f;  // ID1
        leg_set_pos[1] = -90.0f;  // ID2
        leg_set_pos[2] = -90.0f;  // ID3
        leg_set_pos[3] = -90.0f;  // ID4
    }
    else if (pc_msg_chassis->button_status & BTN_LB)
    {
        leg_set_pos[0] = -90.0f;  // ID1
        leg_set_pos[1] = 90.0f;   // ID2
        leg_set_pos[2] = -90.0f;  // ID3
        leg_set_pos[3] = 90.0f;   // ID4
    }
    else if (pc_msg_chassis->button_status & BTN_RB)
    {
        leg_set_pos[0] = 90.0f;   // ID1
        leg_set_pos[1] = -90.0f;  // ID2
        leg_set_pos[2] = 90.0f;   // ID3
        leg_set_pos[3] = -90.0f;  // ID4
    }
    else
    {
        // Update last_button_status
        last_button_status = pc_msg_chassis->button_status;
    }
}