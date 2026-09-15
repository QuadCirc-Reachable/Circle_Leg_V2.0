/**
 * @file    Chassis_Task.hpp
 * @brief   FreeRTOS chassis task: 500 Hz control loop (PC command in ->
 *          Chassis::Update -> motor commands -> feedback out).
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#pragma once
#include "Comm_Msg.hpp"
#include "DJIMotor.hpp"
#include "FreeRTOS.h"
#include "GM6020.hpp"
#include "Helper.hpp"
#include "IMU.hpp"
#include "M3508.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "main.h"
#include "task.h"

namespace Applications::Chassis_Task
{
void Chassis_Task(void *pvPara);
void init();
}  // namespace Applications::Chassis_Task