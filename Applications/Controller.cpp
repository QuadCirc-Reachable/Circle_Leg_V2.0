/**
 * @file    Controller.cpp
 * @brief   Joystick -> chassis velocity mapping (Vx, Wz) with deadzone, D-pad speed
 *          tiers, acceleration limit and low-pass smoothing.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#include "Controller.hpp"

#include <cmath>

#include "Robot_Params.hpp"

namespace Applications
{

// Out-of-class storage for the constexpr tier tables (needed pre-C++17 for
// odr-use; harmless on C++17+).
constexpr float Controller::kTierForwardRPM[4];
constexpr float Controller::kTierTurnRPM[4];
constexpr float Controller::kTierFwdAccel[4];
constexpr float Controller::kTierTurnAccel[4];

void Controller::Map_Joystick_To_Velocity(const Protocol::PC_Msg &msg, float &vx, float &wz)
{
    vx = 0.0f;
    wz = 0.0f;

    // Resolve caps from current speed tier (D-pad selectable; persists across modes).
    const uint8_t tier_idx = static_cast<uint8_t>(speed_tier_);
    const float fwd_cap    = kTierForwardRPM[tier_idx];
    const float turn_cap   = kTierTurnRPM[tier_idx];
    const float fwd_accel  = kTierFwdAccel[tier_idx];
    const float turn_accel = kTierTurnAccel[tier_idx];

    // Deadzone check
    const uint16_t DEADZONE = 200;

    // Left Stick: Forward/Backward (Vx)
    if (msg.left_joystick.r_x1000_msg > DEADZONE)
    {
        uint16_t angle_l = msg.left_joystick.angle_x10_msg;

        if (angle_l >= 300 && angle_l <= 1500)
        {
            vx = (float)msg.left_joystick.r_x1000_msg / 1000.0f * fwd_cap;
        }
        else if (angle_l >= 2100 && angle_l <= 3300)
        {
            vx = -(float)msg.left_joystick.r_x1000_msg / 1000.0f * fwd_cap;
        }
    }

    // Right Stick: Rotation (Wz)
    if (msg.right_joystick.r_x1000_msg > DEADZONE)
    {
        uint16_t angle_r = msg.right_joystick.angle_x10_msg;

        if (angle_r >= 1000 && angle_r <= 2600)
        {
            wz = (float)msg.right_joystick.r_x1000_msg / 1000.0f * turn_cap;
        }
        else if (angle_r <= 800 || angle_r >= 2800)
        {
            wz = -(float)msg.right_joystick.r_x1000_msg / 1000.0f * turn_cap;
        }
    }

    // ===== 1) 硬加速度上限（永不被突破）=====
    // 控制周期 500Hz (vTaskDelay(2)); accel 单位 RPM/s。
    constexpr float DT           = 0.002f;
    const float max_dvx_per_tick = fwd_accel * DT;
    const float max_dwz_per_tick = turn_accel * DT;

    auto slew_limit = [](float prev, float target, float max_delta)
    {
        float delta = target - prev;
        if (delta > max_delta)
            return prev + max_delta;
        if (delta < -max_delta)
            return prev - max_delta;
        return target;
    };
    prev_vx_ = slew_limit(prev_vx_, vx, max_dvx_per_tick);
    prev_wz_ = slew_limit(prev_wz_, wz, max_dwz_per_tick);

    // ===== 2) 一阶 LPF 平滑（软化推背感，参数见 Robot_Params.hpp）=====
    // 串接在加速度限幅之后：因为输入已经被硬截断，LPF 的输出绝对不会
    // 比硬上限更激进 —— 只会更柔。
    const float a = SPEED_SMOOTH_LPF_ALPHA;
    smooth_vx_    = a * prev_vx_ + (1.0f - a) * smooth_vx_;
    smooth_wz_    = a * prev_wz_ + (1.0f - a) * smooth_wz_;

    vx = smooth_vx_;
    wz = smooth_wz_;
}

void Controller::Reset_Velocity_Limiter()
{
    prev_vx_   = 0.0f;
    prev_wz_   = 0.0f;
    smooth_vx_ = 0.0f;
    smooth_wz_ = 0.0f;
}

void Controller::Update_Speed_Tier(uint8_t dpad_status)
{
    // Rising-edge detection so a held button doesn't repeatedly re-assert the
    // tier (allows the user to also implement +/- semantics later by reusing
    // the same edge detector).
    const uint8_t rising = (dpad_status ^ last_dpad_) & dpad_status;
    last_dpad_           = dpad_status;

    // Priority order if multiple D-pad keys are pressed in the same tick:
    // HIGH > LOW > MID_HIGH > MID_LOW (UP/DOWN take precedence as the
    // "extremes" \u2014 most likely user intent when fumbling).
    if (rising & DPAD_UP)
        speed_tier_ = SpeedTier::HIGH;
    else if (rising & DPAD_DOWN)
        speed_tier_ = SpeedTier::LOW;
    else if (rising & DPAD_RIGHT)
        speed_tier_ = SpeedTier::MID_HIGH;
    else if (rising & DPAD_LEFT)
        speed_tier_ = SpeedTier::MID_LOW;
}

}  // namespace Applications
