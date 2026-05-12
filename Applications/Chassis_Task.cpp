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