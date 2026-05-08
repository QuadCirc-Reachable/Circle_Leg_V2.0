#include "Chassis_Task.hpp"

#include "Robot_Config.hpp"
#include "fdcan.h"

// Bring-up diagnostics. Turn ON only when debugging CAN/motor connectivity.
// All globals (canX_*, HT_*_rx, HT_*_connected, DM_*_pos, DM_*_connected) and
// the Ozone wheel-test bypass become no-ops when set to 0.
#ifndef CHASSIS_DEBUG_SNAPSHOT
#define CHASSIS_DEBUG_SNAPSHOT 1
#endif

namespace Applications::Chassis_Task
{
using namespace Core::Drivers;
using namespace Core::Control;
using namespace Applications::Command_Task;

// Global Message Buffers
Protocol::Reachable_Msg reachable_msg_chassis = {};
Protocol::PC_Msg pc_msg_chassis               = {};

#if CHASSIS_DEBUG_SNAPSHOT
// =====================================================================
// Ozone-controllable wheel test bypass (debug builds only).
//   1. Watch  HT_FL_rx / HT_FL_connected / Wheel_Motor_FL.rpmFeedback
//   2. Set    dbg_wheel_test_enable = 1
//   3. Set    dbg_wheel_test_rpm    = 10  (try small first, e.g. 5-20)
//   4. Set back to 0 to stop.
// =====================================================================
volatile uint8_t dbg_wheel_test_enable = 0;
volatile float dbg_wheel_test_rpm      = 0.0f;

// CAN bus + motor connectivity snapshot (refreshed every loop).
volatile uint32_t can1_PSR = 0, can1_ECR = 0;
volatile uint32_t can2_PSR = 0, can2_ECR = 0;
volatile uint32_t can3_PSR = 0, can3_ECR = 0;
volatile uint8_t can2_TEC = 0, can2_REC = 0, can2_LEC = 0, can2_BO = 0;
volatile uint8_t can1_TEC = 0, can1_REC = 0, can1_LEC = 0, can1_BO = 0;
volatile uint8_t can3_TEC = 0, can3_REC = 0, can3_LEC = 0, can3_BO = 0;
volatile uint32_t can2_rxFifoLevel = 0;
volatile uint32_t can3_rxFifoLevel = 0;
volatile uint32_t HT_FL_rx = 0, HT_FR_rx = 0, HT_BL_rx = 0, HT_BR_rx = 0;
volatile uint8_t HT_FL_connected = 0, HT_FR_connected = 0, HT_BL_connected = 0, HT_BR_connected = 0;

// DM leg raw position feedback (rad) — for offset calibration in Ozone.
volatile float DM_FL_pos = 0.0f, DM_BL_pos = 0.0f, DM_FR_pos = 0.0f, DM_BR_pos = 0.0f;
volatile uint8_t DM_FL_connected = 0, DM_BL_connected = 0, DM_FR_connected = 0, DM_BR_connected = 0;
#endif  // CHASSIS_DEBUG_SNAPSHOT

StackType_t uxChassisTaskStack[2048];
StaticTask_t xChassisTaskTCB;

void Chassis_Task(void *pvPara)
{
    // Wait for HT motors to boot after power-on
    // HT8115 internal MCU needs ~500ms to initialize CAN interface
    // Without this delay, ENTER_MOTOR commands sent during Init() are lost
    // (This is why it works under Ozone but not standalone: the debugger adds implicit delay)
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Initialize Chassis (Motors, PIDs, etc.)
    chassis.Init();

    while (true)
    {
#if CHASSIS_DEBUG_SNAPSHOT
        // -------- CAN diagnostics snapshot --------
        can1_PSR         = hfdcan1.Instance->PSR;
        can1_ECR         = hfdcan1.Instance->ECR;
        can1_TEC         = (uint8_t)(can1_ECR & 0xFF);
        can1_REC         = (uint8_t)((can1_ECR >> 8) & 0x7F);
        can1_LEC         = (uint8_t)(can1_PSR & 0x7);
        can1_BO          = (uint8_t)((can1_PSR >> 7) & 0x1);
        can2_PSR         = hfdcan2.Instance->PSR;
        can2_ECR         = hfdcan2.Instance->ECR;
        can2_TEC         = (uint8_t)(can2_ECR & 0xFF);
        can2_REC         = (uint8_t)((can2_ECR >> 8) & 0x7F);
        can2_LEC         = (uint8_t)(can2_PSR & 0x7);
        can2_BO          = (uint8_t)((can2_PSR >> 7) & 0x1);
        can2_rxFifoLevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
        can3_PSR         = hfdcan3.Instance->PSR;
        can3_ECR         = hfdcan3.Instance->ECR;
        can3_TEC         = (uint8_t)(can3_ECR & 0xFF);
        can3_REC         = (uint8_t)((can3_ECR >> 8) & 0x7F);
        can3_LEC         = (uint8_t)(can3_PSR & 0x7);
        can3_BO          = (uint8_t)((can3_PSR >> 7) & 0x1);
        can3_rxFifoLevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0);
        // HT8115 wheels live on CAN1 (canIndex=0). IDs per Robot_Params diagram.
        auto *htFL      = Core::Drivers::Motors::HT8115::getMotor(1, 0);
        auto *htBL      = Core::Drivers::Motors::HT8115::getMotor(2, 0);
        auto *htFR      = Core::Drivers::Motors::HT8115::getMotor(3, 0);
        auto *htBR      = Core::Drivers::Motors::HT8115::getMotor(4, 0);
        HT_FL_rx        = htFL ? htFL->getReceiveCounter() : 0;
        HT_BL_rx        = htBL ? htBL->getReceiveCounter() : 0;
        HT_FR_rx        = htFR ? htFR->getReceiveCounter() : 0;
        HT_BR_rx        = htBR ? htBR->getReceiveCounter() : 0;
        HT_FL_connected = (htFL && htFL->isConnected()) ? 1 : 0;
        HT_BL_connected = (htBL && htBL->isConnected()) ? 1 : 0;
        HT_FR_connected = (htFR && htFR->isConnected()) ? 1 : 0;
        HT_BR_connected = (htBR && htBR->isConnected()) ? 1 : 0;

        // DM legs live on CAN2 (canIndex=1). Slave IDs 1..4 map to FL/BL/FR/BR.
        auto *dmFL      = Core::Drivers::Motors::DMMotor::getMotor(1, 1);
        auto *dmBL      = Core::Drivers::Motors::DMMotor::getMotor(1, 2);
        auto *dmFR      = Core::Drivers::Motors::DMMotor::getMotor(1, 3);
        auto *dmBR      = Core::Drivers::Motors::DMMotor::getMotor(1, 4);
        DM_FL_pos       = dmFL ? dmFL->getPositionFeedback() : 0.0f;
        DM_BL_pos       = dmBL ? dmBL->getPositionFeedback() : 0.0f;
        DM_FR_pos       = dmFR ? dmFR->getPositionFeedback() : 0.0f;
        DM_BR_pos       = dmBR ? dmBR->getPositionFeedback() : 0.0f;
        DM_FL_connected = (dmFL && dmFL->isConnected()) ? 1 : 0;
        DM_BL_connected = (dmBL && dmBL->isConnected()) ? 1 : 0;
        DM_FR_connected = (dmFR && dmFR->isConnected()) ? 1 : 0;
        DM_BR_connected = (dmBR && dmBR->isConnected()) ? 1 : 0;

        // -------- Ozone wheel-test bypass --------
        if (dbg_wheel_test_enable)
        {
            float rpm = dbg_wheel_test_rpm;
            FL_Leg.Set_Wheel_Target(rpm);
            FR_Leg.Set_Wheel_Target(rpm);
            BL_Leg.Set_Wheel_Target(rpm);
            BR_Leg.Set_Wheel_Target(rpm);
            FL_Leg.Execute_Wheel_Control();
            FR_Leg.Execute_Wheel_Control();
            BL_Leg.Execute_Wheel_Control();
            BR_Leg.Execute_Wheel_Control();
            vTaskDelay(2);
            continue;
        }
#endif  // CHASSIS_DEBUG_SNAPSHOT

        // Get PC Command
        Get_PC_Msg(&pc_msg_chassis);

        // Update Chassis Control Loop
        chassis.Update(pc_msg_chassis);

        // Transmit CAN Messages
        // HT8115 wheels and DM J10010L_2EC legs both transmit immediately inside
        // Execute_Wheel_Control / Execute_Leg_Control (one frame per motor),
        // so no group transmit call is needed here. DJI motors removed.

        // Update Feedback Messages
        chassis.Get_Msg(&reachable_msg_chassis);

        // Send feedback to PC
        Set_Reachable_Msg(&reachable_msg_chassis);

        vTaskDelay(2);
    }
}

void init() { xTaskCreateStatic(Chassis_Task, "Chassis_Task", 2048, NULL, 0, uxChassisTaskStack, &xChassisTaskTCB); }
}  // namespace Applications::Chassis_Task