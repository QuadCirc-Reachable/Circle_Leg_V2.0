#pragma once
#include <cstdint>
// Button Bitmasks
#define BTN_LB (1 << 0)
#define BTN_RB (1 << 1)
#define BTN_X (1 << 2)
#define BTN_A (1 << 3)
#define BTN_B (1 << 4)
#define BTN_Y (1 << 5)
#define BTN_ML (1 << 6)
#define BTN_MR (1 << 7)

// D-Pad Bitmasks (lives in PC_Msg.dpad_status). Used to pick one of
// 4 global speed tiers — see Controller::SpeedTier.
#define DPAD_UP (1 << 0)
#define DPAD_DOWN (1 << 1)
#define DPAD_LEFT (1 << 2)
#define DPAD_RIGHT (1 << 3)

// Maximum Size 64 bytes
namespace Protocol
{
struct Reachable_Msg
{
    // GM6020 Motors
    int16_t GM6020_Current_Pos_x10_msg[4];  // 8 bytes
    int16_t GM6020_Target_Pos_x10_msg[4];   // 8 bytes
    int8_t GM6020_Current_Tem_msg[4];       // 4 bytes

    // M3508 Motors
    int16_t M3508_Current_RPM_msg[4];  // 8 bytes
    int16_t M3508_Target_RPM_msg[4];   // 8 bytes
    int8_t M3508_Current_Tem_msg[4];   // 4 bytes

    void reset()
    {
        for (int i = 0; i < 4; i++)
        {
            GM6020_Current_Pos_x10_msg[i] = 0;
            GM6020_Target_Pos_x10_msg[i]  = 0;
            GM6020_Current_Tem_msg[i]     = 0;
            M3508_Current_RPM_msg[i]      = 0;
            M3508_Target_RPM_msg[i]       = 0;
            M3508_Current_Tem_msg[i]      = 0;
        }
    }
} __attribute__((packed));
// Total: 40 bytes

/* --------------------------------------------------------- */
/* PC -> MCU */

struct Joystick_Info
{
    int16_t angle_x10_msg;  // Range 0 ~ 360 degrees -1 NA
    uint16_t r_x1000_msg;   // Range 0-1000 (Mapped from 0.0 ~ 1.0)
} __attribute__((packed));

// Buttons: | LB | RB | X | A | B | Y | ML | MR |

struct PC_Msg
{
    Joystick_Info left_joystick;       // 4 bytes
    Joystick_Info right_joystick;      // 4 bytes
    uint16_t Left_trigger_x1000_msg;   // 2 bytes (Mapped from 0.0 ~ 1.0)
    uint16_t Right_trigger_x1000_msg;  // 2 bytes (Mapped from 0.0 ~ 1.0)
    uint8_t button_status;             // 1 byte
    uint8_t dpad_status;               // 1 byte — D-pad bitmask (DPAD_UP/DOWN/LEFT/RIGHT)

    void reset()
    {
        left_joystick.angle_x10_msg  = -1;
        left_joystick.r_x1000_msg    = 0;
        right_joystick.angle_x10_msg = -1;
        right_joystick.r_x1000_msg   = 0;
        Left_trigger_x1000_msg       = 0;
        Right_trigger_x1000_msg      = 0;
        button_status                = 0;
        dpad_status                  = 0;
    }
} __attribute__((packed));
// Total: 14 bytes
}  // namespace Protocol