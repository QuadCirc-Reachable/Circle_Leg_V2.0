/**
 * @file    Controller.hpp
 * @brief   Joystick -> chassis velocity mapping (Vx, Wz) with deadzone, D-pad speed
 *          tiers, acceleration limit and low-pass smoothing.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#pragma once
#include <cstdint>

#include "Comm_Msg.hpp"
#include "Cust_Types.hpp"
#include "Robot_Params.hpp"  // SPEED_TIER_* / SPEED_TIER_DEFAULT / SPEED_SMOOTH_LPF_ALPHA

namespace Applications
{

// =====================================================================
//  Controller — 摇杆/D-Pad 到底盘速度命令的统一入口
// =====================================================================
//  - Map_Joystick_To_Velocity(): 把左/右摇杆映射成 vx / wz，并施加
//      1) 当前 SpeedTier 的速度上限
//      2) 当前 SpeedTier 的加速度上限（硬截断）
//      3) 一阶 LPF 平滑（推背感软化，不会突破加速度上限）
//  - Update_Speed_Tier(): 每个 tick 读 D-Pad，做上升沿检测后切档。
//  所有数值参数都在 Robot_Params.hpp，方便统一调参。
// =====================================================================
class Controller
{
   public:
    // 四档全局驾驶档位，由 D-Pad 切换，对所有底盘模式生效。
    enum class SpeedTier : uint8_t
    {
        LOW      = 0,  // 慢速 / 维修
        MID_LOW  = 1,  // 默认
        MID_HIGH = 2,  // 较快
        HIGH     = 3,  // 最快（仍低于 HT 最大转速）
    };

    // 档位查找表（数值定义见 Robot_Params.hpp）。
    static constexpr float kTierForwardRPM[4] = SPEED_TIER_FWD_RPM;
    static constexpr float kTierTurnRPM[4]    = SPEED_TIER_TURN_RPM;
    static constexpr float kTierFwdAccel[4]   = SPEED_TIER_FWD_ACCEL;
    static constexpr float kTierTurnAccel[4]  = SPEED_TIER_TURN_ACCEL;

    Controller() = default;

    /**
     * @brief 把摇杆消息映射成 vx / wz（带档位上限 + 加速度限幅 + LPF 平滑）。
     */
    void Map_Joystick_To_Velocity(const Protocol::PC_Msg &msg, float &vx, float &wz);

    /**
     * @brief 重置 slew/LPF 内部状态。模式切换从静止进入时调用，避免接续到
     *        上一模式末态产生跳变。
     */
    void Reset_Velocity_Limiter();

    /**
     * @brief 由 D-Pad 上升沿切换全局速度档位。映射：
     *          DPAD_DOWN  -> LOW
     *          DPAD_LEFT  -> MID_LOW
     *          DPAD_RIGHT -> MID_HIGH
     *          DPAD_UP    -> HIGH
     */
    void Update_Speed_Tier(uint8_t dpad_status);

    SpeedTier Get_Speed_Tier() const { return speed_tier_; }

   private:
    // 加速度限制 + LPF 平滑的内部状态。
    float prev_vx_   = 0.0f;
    float prev_wz_   = 0.0f;
    float smooth_vx_ = 0.0f;  // LPF 输出 (= 实际下发给底盘的 vx)
    float smooth_wz_ = 0.0f;

    // 全局速度档位 — 跨模式保留。
    SpeedTier speed_tier_ = static_cast<SpeedTier>(SPEED_TIER_DEFAULT);
    uint8_t last_dpad_    = 0;
};

}  // namespace Applications
