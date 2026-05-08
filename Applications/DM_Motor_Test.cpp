// ============================================================================
//  DM J10010L-2EC 驱动全功能验证 —— 双电机版
//  --------------------------------------------------------------------------
//  目标：同时驱动两台 DM 电机 (默认 master 0x301 / 0x302)，验证：
//        1. 两台电机各自独立的反馈解码与控制 (motors[CAN][ID] 数组路由)
//        2. 共享 transmitMutex 下并发发送的安全性
//        3. 两路指令之间不串扰 (m1 跟正向轨迹，m2 跟反向轨迹)
//
//  watch 命名约定：g_m1_xxx / g_m2_xxx，对照看就能确认两路独立。
// ============================================================================
#include "DM_Motor_Test.hpp"

#if USE_DM_MOTOR_TEST

#include <cmath>

#include "FreeRTOS.h"
#include "J10010L_2EC.hpp"
#include "task.h"

namespace Applications::DM_Motor_Test
{
using Core::Drivers::Motors::DMMotor;
using Core::Drivers::Motors::J10010L_2EC;

// ----- 测试目标电机 (两台) -----
// 各自在 ctor 里登记到 motors[CAN_INDEX][构造ID]。J10010L_2EC 的 ctor 会
// 写入 pMax=π / vMax=25 / tMax=200，必须与 DM 上位机一致。
static J10010L_2EC s_m1(DM_TEST_MOTOR1_ID, DM_TEST_MOTOR_CAN_INDEX, DMMotor::EMotorMode::eModeMIT, DM_TEST_MOTOR1_REVERSE != 0);
static J10010L_2EC s_m2(DM_TEST_MOTOR2_ID, DM_TEST_MOTOR_CAN_INDEX, DMMotor::EMotorMode::eModeMIT, DM_TEST_MOTOR2_REVERSE != 0);

// ----- 调试器观测变量 (volatile 防止被优化) -----
volatile Phase g_phase = Phase::eWaitConnect;  // 当前测试阶段

// 电机 1 (master 0x301)
volatile uint8_t g_m1_lastError    = 0;
volatile uint32_t g_m1_recvCounter = 0;
volatile float g_m1_posFb_rad      = 0.0f;
volatile float g_m1_velFb_rpm      = 0.0f;
volatile float g_m1_torqueFb_Nm    = 0.0f;
volatile float g_m1_posCmd_rad     = 0.0f;
volatile float g_m1_velCmd_rad_s   = 0.0f;
volatile float g_m1_torqueCmd_Nm   = 0.0f;
volatile float g_m1_home_rad       = 0.0f;
volatile float g_m1_accPos_rad     = 0.0f;
volatile bool g_m1_connected       = false;
volatile bool g_m1_homed           = false;

// 电机 2 (master 0x302)
volatile uint8_t g_m2_lastError    = 0;
volatile uint32_t g_m2_recvCounter = 0;
volatile float g_m2_posFb_rad      = 0.0f;
volatile float g_m2_velFb_rpm      = 0.0f;
volatile float g_m2_torqueFb_Nm    = 0.0f;
volatile float g_m2_posCmd_rad     = 0.0f;
volatile float g_m2_velCmd_rad_s   = 0.0f;
volatile float g_m2_torqueCmd_Nm   = 0.0f;
volatile float g_m2_home_rad       = 0.0f;
volatile float g_m2_accPos_rad     = 0.0f;
volatile bool g_m2_connected       = false;
volatile bool g_m2_homed           = false;

// ----- FreeRTOS 任务静态资源 -----
static StackType_t s_stack[1024];
static StaticTask_t s_tcb;

/**
 * @brief 把两台电机的反馈分别同步到 g_m1_* / g_m2_* watch 全局。
 */
static inline void sampleFeedback()
{
    g_m1_connected   = s_m1.isConnected();
    g_m1_lastError   = static_cast<uint8_t>(s_m1.getError());
    g_m1_posFb_rad   = s_m1.getPositionFeedback();
    g_m1_velFb_rpm   = s_m1.getRPMFeedback();
    g_m1_torqueFb_Nm = s_m1.getTorqueFeedback();
    g_m1_recvCounter = s_m1.getReceiveCounter();
    g_m1_accPos_rad  = s_m1.getAccumulatedPosition();

    g_m2_connected   = s_m2.isConnected();
    g_m2_lastError   = static_cast<uint8_t>(s_m2.getError());
    g_m2_posFb_rad   = s_m2.getPositionFeedback();
    g_m2_velFb_rpm   = s_m2.getRPMFeedback();
    g_m2_torqueFb_Nm = s_m2.getTorqueFeedback();
    g_m2_recvCounter = s_m2.getReceiveCounter();
    g_m2_accPos_rad  = s_m2.getAccumulatedPosition();
}

/**
 * @brief 同周期内对两台电机依次 transmit()。
 *        DMMotor::transmit() 内部用静态 transmitMutex 串行化，多电机安全。
 */
static inline void transmitBoth()
{
    s_m1.transmit();
    s_m2.transmit();
}

/**
 * @brief 测试主任务
 *        按顺序覆盖驱动的全部关键功能。每一阶段切换 g_phase，
 *        所有运动阶段都同时给两台电机下发指令。
 */
static void test_task(void *)
{
    // 启动后等 CANManager 与 DMMotor::init() 内部的后台任务起来再开始
    vTaskDelay(pdMS_TO_TICKS(500));

    // ============================================================
    //  阶段 1：等待两台电机的首次反馈
    //  验证：rxCallback 中按 (canIndex, rxBuffer[0]&0x0F) 的电机分发是否正确。
    //  失败诊断：只有一台 isConnected -> DM 上位机里两台 master ID
    //  是否冲突 / 配置 ID 是否分别 = MOTOR1_ID/MOTOR2_ID。
    // ============================================================
    g_phase = Phase::eWaitConnect;
    {
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        while (!(s_m1.isConnected() && s_m2.isConnected()))
        {
            sampleFeedback();
            if ((int32_t)(xTaskGetTickCount() - deadline) > 0)
            {
                g_phase = Phase::eFault;
                vTaskDelete(nullptr);
                return;
            }
            s_m1.enable();
            s_m2.enable();
            transmitBoth();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // ============================================================
    //  阶段 2：使能并清除错误 (两台并行)
    //  验证：transmit() 状态机各自独立工作；transmitMutex 在并发下安全。
    //  期望：两台 getError() 都变成 eFeedbackEnabled (=1)。
    // ============================================================
    g_phase = Phase::eEnabling;
    s_m1.enable();
    s_m2.enable();
    {
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
        while (s_m1.getError() != DMMotor::EFeedbackERR::eFeedbackEnabled || s_m2.getError() != DMMotor::EFeedbackERR::eFeedbackEnabled)
        {
            sampleFeedback();
            if ((int32_t)(xTaskGetTickCount() - deadline) > 0)
            {
                g_phase = Phase::eFault;
                vTaskDelete(nullptr);
                return;
            }
            transmitBoth();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // ============================================================
    //  阶段 3：两台分别置零位 (绝对多圈计数清零)
    // ============================================================
    s_m1.setZeroPosition();
    s_m2.setZeroPosition();
    {
        const uint32_t base1 = s_m1.getReceiveCounter();
        const uint32_t base2 = s_m2.getReceiveCounter();
        TickType_t deadline  = xTaskGetTickCount() + pdMS_TO_TICKS(500);
        while ((s_m1.getReceiveCounter() - base1 < 3) || (s_m2.getReceiveCounter() - base2 < 3))
        {
            transmitBoth();
            sampleFeedback();
            if ((int32_t)(xTaskGetTickCount() - deadline) > 0)
                break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    const float home1_rad = s_m1.getPositionFeedback();
    const float home2_rad = s_m2.getPositionFeedback();
    g_m1_home_rad         = home1_rad;
    g_m2_home_rad         = home2_rad;

    TickType_t tick0    = xTaskGetTickCount();
    TickType_t lastWake = tick0;

    // ============================================================
    //  阶段 4：HoldZero —— 锁定零位 2 s
    //  验证：MIT 帧的 (pos, vel, Kp, Kd, ffw) 全字段编码 + 位置环。
    //  期望：电机静止在 0，g_posFb_rad ≈ 0；轻微外力推动会回弹。
    // ============================================================
    g_phase = Phase::eHoldZero;
    tick0   = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(2000))
    {
        g_m1_posCmd_rad   = home1_rad;
        g_m2_posCmd_rad   = home2_rad;
        g_m1_velCmd_rad_s = g_m2_velCmd_rad_s = 0.0f;
        g_m1_torqueCmd_Nm = g_m2_torqueCmd_Nm = 0.0f;
        s_m1.setMIT(g_m1_posCmd_rad, g_m1_velCmd_rad_s, DM_TEST_KP, DM_TEST_KD, g_m1_torqueCmd_Nm);
        s_m2.setMIT(g_m2_posCmd_rad, g_m2_velCmd_rad_s, DM_TEST_KP, DM_TEST_KD, g_m2_torqueCmd_Nm);
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 5：PositionSine —— m1 跟 +sin，m2 跟 -sin (反相)
    //  验证：两路指令独立不串扰；watch 中 g_m1_posFb 跟 g_m1_posCmd、
    //        g_m2_posFb 跟 g_m2_posCmd。两台 home 近 ±π 时各自缩幅。
    // ============================================================
    g_phase = Phase::ePositionSine;
    tick0   = xTaskGetTickCount();
    {
        const float pMaxLim   = 3.141593f - 0.10f;
        const float headroom1 = fminf(pMaxLim - home1_rad, pMaxLim + home1_rad);
        const float headroom2 = fminf(pMaxLim - home2_rad, pMaxLim + home2_rad);
        const float A1        = fminf(0.8f, fmaxf(0.0f, headroom1));
        const float A2        = fminf(0.8f, fmaxf(0.0f, headroom2));
        const float fHz       = 1.0f;
        const float w         = 2.0f * (float)M_PI * fHz;
        while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(5000))
        {
            float t           = (xTaskGetTickCount() - tick0) / (float)configTICK_RATE_HZ;
            float s           = sinf(w * t);
            float c           = cosf(w * t);
            g_m1_posCmd_rad   = home1_rad + A1 * s;
            g_m1_velCmd_rad_s = A1 * w * c;
            g_m2_posCmd_rad   = home2_rad - A2 * s;  // 反相
            g_m2_velCmd_rad_s = -A2 * w * c;
            g_m1_torqueCmd_Nm = g_m2_torqueCmd_Nm = 0.0f;
            s_m1.setMIT(g_m1_posCmd_rad, g_m1_velCmd_rad_s, DM_TEST_KP, DM_TEST_KD, g_m1_torqueCmd_Nm);
            s_m2.setMIT(g_m2_posCmd_rad, g_m2_velCmd_rad_s, DM_TEST_KP, DM_TEST_KD, g_m2_torqueCmd_Nm);
            transmitBoth();
            sampleFeedback();
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
        }
    }

    // ============================================================
    //  阶段 6：VelocityStep —— m1 ±3 rad/s，m2 反号 ∓ 3 rad/s
    // ============================================================
    g_phase = Phase::eVelocityStep;
    tick0   = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(4000))
    {
        TickType_t dt     = xTaskGetTickCount() - tick0;
        float v           = (dt < pdMS_TO_TICKS(2000)) ? 3.0f : -3.0f;
        g_m1_velCmd_rad_s = v;
        g_m2_velCmd_rad_s = -v;
        g_m1_posCmd_rad   = s_m1.getPositionFeedback();
        g_m2_posCmd_rad   = s_m2.getPositionFeedback();
        g_m1_torqueCmd_Nm = g_m2_torqueCmd_Nm = 0.0f;
        s_m1.setMIT(g_m1_posCmd_rad, g_m1_velCmd_rad_s, 0.0f, 3.0f, g_m1_torqueCmd_Nm);
        s_m2.setMIT(g_m2_posCmd_rad, g_m2_velCmd_rad_s, 0.0f, 3.0f, g_m2_torqueCmd_Nm);
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 7：TorqueStep —— m1 ±3 Nm，m2 反号 ∓3 Nm
    // ============================================================
    g_phase = Phase::eTorqueStep;
    tick0   = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(2000))
    {
        TickType_t dt     = xTaskGetTickCount() - tick0;
        float tq          = (dt < pdMS_TO_TICKS(1000)) ? 3.0f : -3.0f;
        g_m1_torqueCmd_Nm = tq;
        g_m2_torqueCmd_Nm = -tq;
        g_m1_posCmd_rad   = s_m1.getPositionFeedback();
        g_m2_posCmd_rad   = s_m2.getPositionFeedback();
        g_m1_velCmd_rad_s = g_m2_velCmd_rad_s = 0.0f;
        s_m1.setMIT(g_m1_posCmd_rad, g_m1_velCmd_rad_s, 0.0f, 0.0f, g_m1_torqueCmd_Nm);
        s_m2.setMIT(g_m2_posCmd_rad, g_m2_velCmd_rad_s, 0.0f, 0.0f, g_m2_torqueCmd_Nm);
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 8：FreeSpin —— 两台都零阻抗，手转任意一台都应被独立累加
    // ============================================================
    g_phase = Phase::eFreeSpin;
    tick0   = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(DM_TEST_FREESPIN_MS))
    {
        s_m1.setMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        s_m2.setMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        g_m1_posCmd_rad = g_m2_posCmd_rad = 0.0f;
        g_m1_velCmd_rad_s = g_m2_velCmd_rad_s = 0.0f;
        g_m1_torqueCmd_Nm = g_m2_torqueCmd_Nm = 0.0f;
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 9：ReturnHome —— 两台同时回到各自零位
    // ============================================================
    g_phase = Phase::eReturnHome;
    tick0   = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(DM_TEST_RETURN_MS))
    {
        s_m1.setMIT(0.0f, 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f);
        s_m2.setMIT(0.0f, 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f);
        g_m1_posCmd_rad = g_m2_posCmd_rad = 0.0f;
        g_m1_velCmd_rad_s = g_m2_velCmd_rad_s = 0.0f;
        g_m1_torqueCmd_Nm = g_m2_torqueCmd_Nm = 0.0f;
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 10：HomeReached —— 终态判定 (g_m1_homed / g_m2_homed) + 持续观测
    // ============================================================
    sampleFeedback();
    g_m1_homed = (fabsf(g_m1_posFb_rad) < 0.10f) && (fabsf(g_m1_accPos_rad) < 0.10f);
    g_m2_homed = (fabsf(g_m2_posFb_rad) < 0.10f) && (fabsf(g_m2_accPos_rad) < 0.10f);
    g_phase    = Phase::eHomeReached;
    tick0      = xTaskGetTickCount();
    while ((xTaskGetTickCount() - tick0) < pdMS_TO_TICKS(DM_TEST_HOMEHOLD_MS))
    {
        s_m1.setMIT(0.0f, 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f);
        s_m2.setMIT(0.0f, 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f);
        transmitBoth();
        sampleFeedback();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
    }

    // ============================================================
    //  阶段 11：Park & Disable —— 两台都阻尼停车 + 发停机帧
    // ============================================================
    g_phase = Phase::eDisabled;
    {
        TickType_t dampStart = xTaskGetTickCount();
        while ((xTaskGetTickCount() - dampStart) < pdMS_TO_TICKS(200))
        {
            s_m1.setMIT(s_m1.getPositionFeedback(), 0.0f, 0.0f, DM_TEST_KD, 0.0f);
            s_m2.setMIT(s_m2.getPositionFeedback(), 0.0f, 0.0f, DM_TEST_KD, 0.0f);
            transmitBoth();
            sampleFeedback();
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
        }

        s_m1.disable();
        s_m2.disable();
        for (int i = 0; i < 10; ++i)
        {
            transmitBoth();  // 各自的 transmit() 状态机会发 0xFF..FD
            sampleFeedback();
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    while (true)
    {
        sampleFeedback();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief 创建测试任务，由 startUserTasks() 在 CAN + DMMotor::init() 之后调用。
 */
void init() { xTaskCreateStatic(test_task, "DM_Test", sizeof(s_stack) / sizeof(s_stack[0]), nullptr, 0, s_stack, &s_tcb); }

}  // namespace Applications::DM_Motor_Test

#endif  // USE_DM_MOTOR_TEST
