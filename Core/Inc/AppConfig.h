/**
 * @file AppConfig_tempelate.h
 * @author my name (my@domain.com)
 * @brief Configuration file for the whole project
 * @version 0.2
 * @date 2025-4-17
 *
 * @copyright Copyright (c) 2025
 *
 */

/* @how to use this file
 * 1. Copy this file as `AppConfig.h`
 * 2. put it into your template directory
 * 3. modify the content of this file
 * 4. include this file in your project
 */

/* clang-format off */
#if 1 /*Set it to "1" to enable content*/


#ifndef APPCONFIG_H
#define APPCONFIG_H
#endif

#include "Config.h"


//1 for use, 0 for not use
/*==============*
   DEBUG CONFIG
 *==============*/
/*if use debug, some debug watch variables could be seen*/
#define USE_DEBUG 0


/*============*
   IMU CONFIG
 *============*/
/*Config which IMU you use BEGIN*/
#define USE_IMU 1

#if USE_IMU
    #define USE_BMI088 0 // RoboMaster C Board
    #define USE_ICM42688 1 // G/F Board
    /*Config which IMU you use END*/
    #if USE_BMI088
        #define BMI088_SPI hspi1
    #endif // USE_BMI088

    #define IMU_ORIEN_X 0.0f
    #define IMU_ORIEN_Y 0.0f
    #define IMU_ORIEN_Z 0.0f

    #define USE_IMU_HEAT 0

    #if USE_IMU_HEAT
        #define IMU_HEAT_TIM htim20
        #define IMU_HEAT_CHL TIM_CHANNEL_3
        #define IMU_HEAT_TARGET_TEMP 40.0f
        // Config PID params
        #define IMU_HEAT_KP 10.0f
        #define IMU_HEAT_KI 2.0f
        #define IMU_HEAT_KD 5.0f
        #define IMU_HEAT_I_MAX 20.0f // Max integral PWM duty cycle
        #define IMU_HEAT_OUTPUT_MAX 100.0f // Max output PWM duty cycle
    #endif // USE_IMU_HEAT
#endif // USE_IMU

/*Config which Magnetometer you use BEGIN*/
#define USE_MAG 0

#if USE_MAG
    #define USE_LIS3MDL 1  // Select MAG type
#endif // USE_MAG

#if USE_MAG || USE_IMU
    #if defined(STM32F407xx)
        #define SENSOR_SPI hspi1
    #elif defined(STM32G473xx)
        #define SENSOR_SPI hspi1
    #elif defined(STM32G431xx)
        #error "If you are using internal board, MPU6500 driver is not supported here yet XD"
    #else
        #error "Chips are not defined"
    #endif // defined (chips & SENSOR_SPI)
#endif // USE_MAG || USE_IMU


/*================*
   CHASSIS CONFIG
 *================*/
#define USE_CHASSIS_CALCULATOR 0

#if USE_CHASSIS_CALCULATOR
    #define USE_OMNI_CHASSIS 0
    #define USE_MECANUM_CHASSIS 0
    #define USE_SWERVE_CHASSIS 1
#endif // USE_CHASSIS_CALCULATOR


/*=========================*
   POWER CONTROLLER CONFIG
 *=========================*/
#define USE_POWER_CONTROLLER 0

#if USE_POWER_CONTROLLER
/**
 * @note This config indicates whether super cap is connected in your chassis electronic circuit,
 *       but not whether you enable super cap module or not. Make sure you ACTIVATE THE MODULE CORRECTLY,
 *       otherwise power controller would not behave well
 */
#define USE_SUPER_CAPACITOR 0

#if USE_SUPER_CAPACITOR
    #define SUPER_CAPACITOR_CAN_INDEX 0 // 0 for CAN1, 1 for CAN2, 2 for CAN3
    #define SUPER_CAPACITOR_MODEL 2025 // 2024 for RM2024, 2025 for RM2025
    #define CUSTOM_ENERGY_LIMIT 0
    #if CUSTOM_ENERGY_LIMIT
        #define IN_GAME_ENERGY_LIMIT 230
        #define PURE_CAP_ENERGY_LIMIT 50
    #endif // CUSTOM_ENERGY_LIMIT
#endif // USE_SUPER_CAPACITOR
#endif // USE_POWER_CONTROLLER


/*============*
   PID CONFIG
 *============*/
#define PID_TIMEOUT_CUSTOM 0

#if PID_TIMEOUT_CUSTOM
    #define PID_DEFAULT_TIMEOUT pdMS_TO_TICKS(100)
#endif // PID_TIMEOUT_CUSTOM

#define PID_ALPHA_CUSTOM 0

#if PID_ALPHA_CUSTOM
    #define PID_DEFAULT_ALPHA 0.2f
#endif // PID_ALPHA_CUSTOM

#define PID_MAX_OUTPUT_CUSTOM 0

#if PID_MAX_OUTPUT_CUSTOM
    #define PID_DEFAULT_MAX_OUTPUT 10000.0f
#endif // PID_MAX_OUTPUT_CUSTOM


/*==================*
   DJI MOTOR CONFIG
 *==================*/
// NOTE: M3508 is still referenced by Wheel_Leg.hpp / Robot_Config.cpp, which
// are compiled regardless of USE_DM_MOTOR_TEST (Makefile globs Applications/*.cpp).
// DJI motors fully removed from this build. Wheels = HT8115 on CAN2; Legs = DM J10010L_2EC on CAN1.
#define USE_DJI_MOTOR 0

#if USE_DJI_MOTOR
    #define DJI_MOTOR_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

    #define DJI_MOTOR_USE_CAN1 0
    #define DJI_MOTOR_USE_CAN2 0
    #define DJI_MOTOR_USE_CAN3 0

    #define USE_DJI_GM6020 0
    #define USE_DJI_M2006 0
    #define USE_DJI_M3508 0
#endif // USE_DJI_MOTOR


/*=================*
   DM MOTOR CONFIG
 *=================*/
#define USE_DM_MOTOR 1

#if USE_DM_MOTOR
    #define DM_MOTOR_USE_CAN1 0
    #define DM_MOTOR_USE_CAN2 1
    #define DM_MOTOR_USE_CAN3 0

    /* DM Motor id setting, must be the same as configrator*/
    // Default DM Motor Slave ID range: 0x300 - 0x307
    #define DM_MOTOR_ID_CUSTOM 1

    #if DM_MOTOR_ID_CUSTOM
        #define DM_MOTOR_MASTER_ID_START 0x300
        #define DM_MOTOR_MASTER_ID_END 0x307
    #endif // DM_MOTOR_ID_CUSTOM

    // DM Motor MIT default parameters configurationS
    #define DM_MOTOR_UINT_CUSTOM 1
    
    #if DM_MOTOR_UINT_CUSTOM
        #define DM_MOTOR_DEFAULT_P_MAX 3.141593f
        #define DM_MOTOR_DEFAULT_V_MAX 25.0f
        #define DM_MOTOR_DEFAULT_T_MAX 200.0f
        // Defaultly, set KP and KD to 0 for only using the current loop
        #define DM_MOTOR_DEFAULT_KP 0.0f
        #define DM_MOTOR_DEFAULT_KD 0.0f
    #endif // DM_MOTOR_UINT_CUSTOM

    #define DM_MOTOR_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

    #define USE_DM_J3507_2EC 0
    #define USE_DM_J4310_2EC 0
    #define USE_DM_J4340_2EC 0
    #define USE_DM_J10010L_2EC 1
#endif // USE_DM_MOTOR


/*=========================*
   DM MOTOR BRING-UP TEST
 *=========================*/
// Set to 1 to run the standalone DM J10010 bring-up test (Applications/DM_Motor_Test.*).
// While enabled, HT8115::init / DJIMotor::init / Chassis_Task / Command_Task are skipped
// (see UserTask.cpp) so the test owns the bus. Set back to 0 for normal operation.
#define USE_DM_MOTOR_TEST 0

#if USE_DM_MOTOR_TEST
    // The DM driver uses one ID for both TX and RX:
    //   - TX frame CAN ID = mode_offset + DM_TEST_MOTOR_ID  (e.g. MIT -> 0x000 + ID = slave CAN ID)
    //   - RX dispatch indexes motors[canIdx][rxBuffer[0] & 0x0F] = slave CAN ID
    // So set DM_TEST_MOTOR_ID to the slave "CAN ID" you see in DM Tool, NOT the Master ID.
    // Example for this motor: CAN ID = 0x01, Master ID = 0x301  ->  DM_TEST_MOTOR_ID = 1
    #define DM_TEST_MOTOR_ID        1
    #define DM_TEST_MOTOR_CAN_INDEX 0       // 0 = CAN1 (DM bus), 1 = CAN2, 2 = CAN3
    #define DM_TEST_MOTOR_REVERSE   0
    #define DM_TEST_KP              1.0f    // diagnostic: minimal stiffness, ~no current
    #define DM_TEST_KD              0.5f    // diagnostic: minimal damping
    #define DM_TEST_AUTO_ZERO       0       // 1 -> setZeroPosition() once on first connect (writes flash; normally calibrate once via DM Tool)
#endif // USE_DM_MOTOR_TEST


/*=================*
   LK MOTOR CONFIG
 *=================*/
#define USE_LK_MOTOR 0

#if USE_LK_MOTOR
    #define LK_MOTOR_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

    #define LK_MOTOR_USE_CAN1 1
    #define LK_MOTOR_USE_CAN2 0
    #define LK_MOTOR_USE_CAN3 0

    #define USE_LK_MF7015 1
    #define USE_LK_MF9015 0
    #define USE_LK_MF9025 0
    #define USE_LK_MG4005E_I10 0
    #define USE_LK_MG4010E_I10 0
    #define USE_LK_MG5010E_I36 0
    #define USE_LK_MG6010E_I6 0
    #define USE_LK_MG8010E_I36 0
    #define USE_LK_MG8016E_I6 0
#endif // USE_LK_MOTOR

/*=================*
   HT MOTOR CONFIG
 *=================*/
#define USE_HT_MOTOR 1
#if USE_HT_MOTOR
    #define HT_MOTOR_CONNECTION_TIMEOUT pdMS_TO_TICKS(200)

    // HT8115 wheel motors live on CAN3 (try CAN3 since CAN2 transceiver/wiring not responding)
    #define HT_MOTOR_USE_CAN1 1
    #define HT_MOTOR_USE_CAN2 0
    #define HT_MOTOR_USE_CAN3 0
    #define USE_HT8115_MOTOR 1

#endif // USE_HT_MOTOR

/*====================*
   SP15D MOTOR CONFIG
  ====================*/
#define USE_SP15D 0

#if USE_SP15D
    // ID from 0 - 15, maximum number of 16 motors
    #define SP15D_MAX_MOTOR_NUM 16
    // TIMEOUT for the master to wait for the response of the motor
    #define SP15D_TIMEOUT_TICK 10
#endif // USE_SP15D


/*=============*
   DR16 CONFIG
 *=============*/
#define USE_DR16 0

#if USE_DR16
/*UART CONFIG*/
    #if defined(STM32F407xx)
        #define DR16_UART huart4
    #elif defined(STM32G473xx)
        #define DR16_UART huart1
    #elif defined(STM32G431xx)
        #define DR16_UART huart3
    #else
        #error "Chips are not defined"
    #endif // define chips
#endif // USE_DR16


/*=================*
   FS-iA10B CONFIG
 *=================*/
#define USE_FSiA10B 0

#if USE_FSiA10B
/*UART CONFIG*/
    #if defined(STM32F407xx)
        #define FSiA10B_UART huart4
    #elif defined(STM32G473xx)
        #define FSiA10B_UART huart1
    #elif defined(STM32G431xx)
        #define FSiA10B_UART huart3
    #else
        #error "Chips are not defined"
    #endif // define chips
#endif // USE_FSiA10B


/*======================*
   FDCAN MANAGER CONFIG
 *======================*/
#define USE_CAN_MANAGER 1

#if USE_CAN_MANAGER 
    #define CAN_CUSTOM 1

    #if CAN_CUSTOM
        #define CAN_NUM 3
        #define CAN_FILTER_NUM 8
        #define CAN_TX_MAX_MESSAGE_NUM_PER_TICK 7

    #if defined(HAL_CAN_MODULE_ENABLED)
        #define CAN_FILTER_SLAVE_START 14
    #endif
    #endif // CAN_CUSTOM
#endif // USE_CAN_MANAGER


/*======================================*
   SERIAL INTERBOARD (MODBUS485) CONFIG
 *======================================*/
#define USE_MODBUS485 0

#if USE_MODBUS485
    // Master Definition
    #define USE_MODBUS485_MASTER 1

    #if USE_MODBUS485_MASTER
        // Slaves number on the RS485 bus
        #define MODBUS485_SLAVE_COMM_NUM 1
        // Configurate the number of the master handle, on different UART port
        #define MODBUS485_MASTER_HANDLE_NUM 1
        // Timeout(ms) for the master to justify the slave is disconnected
        #define MODBUS485_MASTER_DISCONNECT_TIMEOUT 200
        // master tx, rx buffer size
        #define MODBUS485_MASTER_TX_PAYLOAD_LENGTH 8
        #define MODBUS485_MASTER_RX_PAYLOAD_LENGTH 8
    #endif // USE_MODBUS485_MASTER

    // Slave Definition
    #define USE_MODBUS485_SLAVE 0

    #if USE_MODBUS485_SLAVE
        // Configurate the number of the slave handle, on different UART port
        #define MODBUS485_SLAVE_HANDLE_NUM 1
        // Timeout(ms) for the slave to justify the master is disconnected
        #define MODBUS485_SLAVE_DISCONNECT_TIMEOUT 200

        // slave tx, rx buffer size
        #define MODBUS485_SLAVE_TX_PAYLOAD_LENGTH 8
        #define MODBUS485_SLAVE_RX_PAYLOAD_LENGTH 8
    #endif // USE_MODBUS485_SLAVE 
#endif // USE_MODBUS485


/*=========================*
   FDCAN INTERBOARD CONFIG
 *=========================*/
#define USE_FDCAN_INTERBOARD 0

#if USE_FDCAN_INTERBOARD
    // Configurate the number of the interboard handle, on different CAN port
    #define FDCAN_INTERBOARD_HANDLE_NUM 1
    // Configurate the number of the nodes on the CAN-Bus to communicate with
    #define FDCAN_INTERBOARD_COMM_NUM 1
    // Timeout(ms) for the node to justify the interboard is disconnected
    #define FDCAN_INTERBOARD_DISCONNECT_TIMEOUT 200
#endif // USE_FDCAN_INTERBOARD


/*============================*
   REFEREE SYSTEM COMM CONFIG
 *============================*/
#define USE_REFEREE_SYSTEM_COMM 0

#if USE_REFEREE_SYSTEM_COMM
    #if defined(STM32F407xx)
        #define REFEREE_COMM_UART huart2
    #elif defined(STM32G473xx)
        #define REFEREE_COMM_UART huart5
    #elif defined(STM32G431xx)
        #define REFEREE_COMM_UART huart2
    #else
        #error "Chips are not defined"
    #endif // define chips
#endif // USE_REFEREE_SYSTEM_COMM 


/*=================*
   ROS COMM CONFIG
 *=================*/
#define USE_ROS_COMM 1

#if USE_ROS_COMM 
    // The number of ROS communication channels
    #define ROS_NUM 1 
    // The size of the rx circular buffer, recommended to be the same with the ROS host tx buffer size
    #define ROS_RX_BUF_SIZE 1024
    // tx buffer size
    #define ROS_TX_BUF_SIZE 512
    // Maximum number of the message callback function
    #define ROS_MAX_FRAME_CALLBACK_NUM 4
    // Maximum size for one package, containing CRC and header
    #define ROS_MAX_PACKAGE_SIZE 64
    // RX timeout threshold
    #define ROS_RX_TIMEOUT 50

    #define ROS_CAMERA_SYNC 0
#endif // USE_ROS_COMM


/*============*
   KEY CONFIG
 *============*/
#define USE_KEY 0

#if USE_KEY
    #define LONG_PRESS_INTERVAL 800
    #define DOUBLE_PRESS_INTERVAL 500
    #define TRIPLE_PRESS_INTERVAL 500
    #define PRESS_STATUS_RESET_INTERVAL 1000
    #define MAX_KEYCOMBO 10
#endif


/*=================*
   VTM COMM CONFIG
 *=================*/
#define USE_VTM_COMM 0

#if USE_VTM_COMM
    #if defined(STM32F407xx)
        #define VTM_COMM_UART huart2
    #elif defined(STM32G473xx)
        #define VTM_COMM_UART huart5
    #elif defined(STM32G431xx)
        #define VTM_COMM_UART huart2
    #else
        #error "Chips are not defined"
    #endif // define chips
    #define VTM_COMM_CUSTOMIZED_CONTROLLER_TIMEOUT pdMS_TO_TICKS(200)
    #define VTM_COMM_KEYBOARD_MOUSE_TIMEOUT pdMS_TO_TICKS(200)
    #define VTM_COMM_RC_TIMEOUT pdMS_TO_TICKS(200)
    #define USE_NEW_VTM 0
    #if USE_NEW_VTM
        #define VTM_MAX_BUFF_SIZE 256
    #else
        #define VTM_MAX_BUFF_SIZE 80
    #endif // USE_NEW_VTM
#endif // USE_VTM_COMM


/*===============*
   BUZZER CONFIG
 *===============*/
#define USE_BUZZER 0

#if USE_BUZZER
    #define BUZZER_TIM htim20
    #define BUZZER_TIM_CHANNEL TIM_CHANNEL_2
    #define BUZZER_TIM_CLOCK 170000000.0f
    #define BUZZER_QUEUE_LENGTH 20
#endif // USE_BUZZER


#endif // Content enable