/**
 * @file DM_Motor_Test.hpp
 * @brief Standalone bring-up / sanity test for a DM J10010L-2EC motor.
 *
 * Exercises the driver in isolation:
 *   1. Wait for CAN feedback (verifies wiring + master/slave ID match).
 *   2. Enable the motor and confirm error code becomes eFeedbackEnabled.
 *   3. Optionally call setZeroPosition() once on first connect.
 *   4. Run a sequence of MIT commands: hold-zero, position sine, velocity step,
 *      torque step. Each phase logs to volatile telemetry globals so they can
 *      be watched live in Ozone/STM32CubeMonitor.
 *
 * The whole file is gated by USE_DM_MOTOR_TEST (define =1 in AppConfig.h to
 * enable). When disabled, nothing is compiled in and the test has zero cost.
 *
 * Call DM_Motor_Test::init() from startUserTasks() AFTER:
 *   - CANManager::managers[...].init(...)
 *   - Core::Drivers::Motors::DMMotor::init()
 *
 * @note This test owns its own J10010L_2EC instance; do NOT also instantiate
 *       one with the same (CAN, ID) pair elsewhere, the driver asserts that.
 */
#pragma once

#include "AppConfig.h"

#ifndef USE_DM_MOTOR_TEST
#define USE_DM_MOTOR_TEST 0
#endif

#if USE_DM_MOTOR_TEST

#if !USE_DM_MOTOR || !USE_DM_J10010L_2EC
#error "USE_DM_MOTOR_TEST requires USE_DM_MOTOR && USE_DM_J10010L_2EC"
#endif

// ---- Test target configuration (override in AppConfig.h if needed) ----
// 测试两路电机，构造 ID = master ID - DM_MOTOR_MASTER_ID_START。
// 当前 AppConfig.h 把 START 设为 0x300，所以构造 ID=1 对应 master 0x301、ID=2 对应 0x302。
#ifndef DM_TEST_MOTOR1_ID
#define DM_TEST_MOTOR1_ID 1
#endif
#ifndef DM_TEST_MOTOR2_ID
#define DM_TEST_MOTOR2_ID 2
#endif

#ifndef DM_TEST_MOTOR_CAN_INDEX
#define DM_TEST_MOTOR_CAN_INDEX 0  // 0 -> CAN1, 1 -> CAN2, 2 -> CAN3
#endif

#ifndef DM_TEST_MOTOR1_REVERSE
#define DM_TEST_MOTOR1_REVERSE 0
#endif
#ifndef DM_TEST_MOTOR2_REVERSE
#define DM_TEST_MOTOR2_REVERSE 0
#endif

// Default impedance used during the test sequence (very soft, safe to spin freely)
#ifndef DM_TEST_KP
#define DM_TEST_KP 5.0f
#endif
#ifndef DM_TEST_KD
#define DM_TEST_KD 0.5f
#endif

// Force-free phase duration: user hand-rotates the motor to verify
// multi-turn / absolute encoder accumulation in g_accPos_rad.
#ifndef DM_TEST_FREESPIN_MS
#define DM_TEST_FREESPIN_MS 10000
#endif

#ifndef DM_TEST_RETURN_MS
#define DM_TEST_RETURN_MS 5000
#endif

// How long to keep holding the motor at the origin in eHomeReached so you can
// inspect g_homed / g_accPos_rad in the watch panel before the test parks.
#ifndef DM_TEST_HOMEHOLD_MS
#define DM_TEST_HOMEHOLD_MS 10000
#endif

namespace Applications::DM_Motor_Test
{
/**
 * @brief Create the test FreeRTOS task. Call once from startUserTasks().
 */
void init();

/**
 * @brief Test phase the task is currently executing (for live watch).
 */
enum class Phase : uint8_t
{
    eWaitConnect = 0,
    eEnabling,
    eHoldZero,
    ePositionSine,
    eVelocityStep,
    eTorqueStep,
    eFreeSpin,     ///< Force-free, user spins motor by hand
    eReturnHome,   ///< Driving back to absolute zero
    eHomeReached,  ///< Origin reached, idle
    eDisabled,
    eFault
};

}  // namespace Applications::DM_Motor_Test

#endif  // USE_DM_MOTOR_TEST
