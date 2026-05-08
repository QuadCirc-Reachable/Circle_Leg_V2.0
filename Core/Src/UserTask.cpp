/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2023 (EthenJ@outlook.sg)
 * @brief Create user tasks with cpp support
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 */

#include "CANManager.hpp"
#include "Chassis_Task.hpp"
#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "DM_Motor_Test.hpp"
#include "FreeRTOS.h"
#include "HT8115.hpp"
#include "IMU.hpp"
#include "PC_Comm.hpp"
#include "RosComm.hpp"
#include "gpio.h"
#include "main.h"
#include "task.h"
#include "usart.h"

StackType_t uxBlinkTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xBlinkTaskTCB;

void blink(void *pvPara)
{
    HAL_GPIO_WritePin(LED_ACT_GPIO_Port, LED_ACT_Pin, GPIO_PIN_RESET);

    while (true)
    {
        HAL_GPIO_TogglePin(LED_ACT_GPIO_Port, LED_ACT_Pin);
        HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
        vTaskDelay(500);
    }
}

/**
 * @brief Create user tasks
 */
void startUserTasks()
{
    xTaskCreateStatic(blink, "blink", configMINIMAL_STACK_SIZE, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB);
    Core::Drivers::CANManager::managers[0].init(&hfdcan1);
    Core::Drivers::CANManager::managers[1].init(&hfdcan2);
    Core::Drivers::CANManager::managers[2].init(&hfdcan3);
#if !USE_DM_MOTOR_TEST
    // Skip HT init while running the standalone DM motor bring-up test
    // so the test owns the bus and nothing else fights for it.
    Core::Drivers::Motors::HT8115::init();
#endif
#if USE_DM_MOTOR
    Core::Drivers::Motors::DMMotor::init();
#endif
    Core::Drivers::IMU::init();
    Core::Communication::RosComm::RosManager::managers[0].init(&huart2);
#if USE_DM_MOTOR_TEST
    // Standalone DM motor bring-up test. Skips Chassis/Command/HT/DJI init
    // by design — run the test in isolation, then disable the macro.
    Applications::DM_Motor_Test::init();
#else
    Applications::Command_Task::init();
    Applications::Chassis_Task::init();
#endif
}
