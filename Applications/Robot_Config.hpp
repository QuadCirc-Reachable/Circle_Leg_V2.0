/**
 * @file    Robot_Config.hpp
 * @brief   Hardware instantiation: HT8115 wheel motors, DM J10010L-2EC leg motors,
 *          CAN bus / ID mapping, per-leg MIT parameters and Wheel_Leg pairing.
 *
 * Circle_Leg_V2 - REACHABLE (QuadCirc) full-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#pragma once
#include "Chassis.hpp"
#include "HT8115.hpp"
#include "J10010L_2EC.hpp"
#include "PID.hpp"
#include "Robot_Params.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
// Expose the Chassis instance
extern Chassis chassis;

// Expose individual components if needed for debugging
extern Wheel_Leg FL_Leg;
extern Wheel_Leg FR_Leg;
extern Wheel_Leg BL_Leg;
extern Wheel_Leg BR_Leg;

}  // namespace Applications
