#include "PC_Comm.hpp"

namespace Applications::Command_Task
{
using namespace Core::Communication::RosComm;

// Create global message objects
static Protocol::PC_Msg g_pc_msg               = {};
static Protocol::Reachable_Msg g_reachable_msg = {};

// Init PC_Comm object
PC_Comm pc_comm(g_pc_msg, g_reachable_msg);

// Callback function to handle received PC_Msg
void PC_RxCallback(uint8_t *data, uint16_t len, UART_HandleTypeDef *handle)
{
    if (handle != Core::Communication::RosComm::RosManager::managers[0].getUARTHandle())
    {
        return;
    }
    if (len != sizeof(Protocol::PC_Msg))
    {
        return;
    }
    memcpy((void *)&g_pc_msg, data, sizeof(Protocol::PC_Msg));

    // Update timestamp
    pc_comm.Update_Timestamp();

    // If not connected previously, maybe we want to log or anything, but logic is handled via Is_Connected check
}

void PC_Comm::Update_Timestamp()
{
    // Use xTaskGetTickCountFromISR() if inside ISR to avoid assertion failure
    // Note: If xPortIsInsideInterrupt() is not available, consider using HAL_GetTick() for both.
    // Assuming standard FreeRTOS port for ARM Cortex-M
    if (xPortIsInsideInterrupt())
    {
        last_msg_time_ms = xTaskGetTickCountFromISR();
    }
    else
    {
        last_msg_time_ms = xTaskGetTickCount();
    }
    is_connected     = true;
    has_received_msg = true;
}

bool PC_Comm::Is_Connected()
{
    // Check if Last Msg ever received
    if (!has_received_msg)
    {
        is_connected = false;
        return false;
    }

    // Check if last message was within 500ms
    // If no message for > 500ms, consider disconnected
    TickType_t current_tick = xTaskGetTickCount();
    if ((current_tick - last_msg_time_ms) > pdMS_TO_TICKS(500))
    {
        is_connected = false;
    }
    else
    {
        is_connected = true;
    }
    return is_connected;
}

// Getter for PC_Msg
void PC_Comm::Fetch_PC_Msg(Protocol::PC_Msg *pc_msg)
{
    if (pc_msg == nullptr)
        return;

    ATOMIC_ENTER_CRITICAL();
    // Copy the data
    memcpy(pc_msg, (void *)&pv_pc_msg, sizeof(Protocol::PC_Msg));
    ATOMIC_EXIT_CRITICAL();
}

// Setter for Reachable_Msg
void PC_Comm::Update_Reachable_Msg(Protocol::Reachable_Msg *reachable_msg)
{
    if (reachable_msg == nullptr)
        return;

    ATOMIC_ENTER_CRITICAL();
    memcpy((void *)&pv_reachable_msg, reachable_msg, sizeof(Protocol::Reachable_Msg));
    ATOMIC_EXIT_CRITICAL();
}

// PC_Comm Task
void PC_Comm::PC_CommTask(void *pvPara)
{
    // Init Frame Header (PC_Msg Protocol ID: 0xFF)
    FrameHeader PC_RxHeader;
    PC_RxHeader.sof        = START_BYTE;                // 0xAA
    PC_RxHeader.protocolID = 0xFF;                      // Protocol ID for PC Comm
    PC_RxHeader.dataLen    = sizeof(Protocol::PC_Msg);  // Data Length

    // Init Frame Header (Reachable_Msg Protocol ID: 0xFE)
    FrameHeader Reachable_TxHeader;
    Reachable_TxHeader.sof        = START_BYTE;                       // 0xAA
    Reachable_TxHeader.protocolID = 0xFE;                             // Protocol ID for Reachable Msg
    Reachable_TxHeader.dataLen    = sizeof(Protocol::Reachable_Msg);  // Data Length

    // Rest the messages
    pc_comm.pv_reachable_msg.reset();
    pc_comm.pv_pc_msg.reset();

    TickType_t xLastWakeTime    = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(pc_comm.tx_freq);

    // Register RX Callback
    RosManager::managers[0].registerFrameCallback(0xFF, PC_RxCallback);

    while (true)
    {
        // --- transmit ---
        if (RosManager::managers[0].isInitialized())
        {
            RosManager::managers[0].transmit(Reachable_TxHeader, (uint8_t *)&pc_comm.pv_reachable_msg);
        }

        // Absolute delay
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Get_PC_Msg(Protocol::PC_Msg *pc_msg) { pc_comm.Fetch_PC_Msg(pc_msg); }
void Set_Reachable_Msg(Protocol::Reachable_Msg *reachable_msg) { pc_comm.Update_Reachable_Msg(reachable_msg); }
bool Is_PC_Connected() { return pc_comm.Is_Connected(); }

StackType_t uxPC_CommTaskStack[2048];
StaticTask_t xPC_CommTaskTCB;
void init() { xTaskCreateStatic(PC_Comm::PC_CommTask, "PC_Comm_Task", 2048, NULL, 0, uxPC_CommTaskStack, &xPC_CommTaskTCB); }
}  // namespace Applications::Command_Task
