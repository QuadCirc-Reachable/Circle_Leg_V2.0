#include "Robot_Config.hpp"

namespace Applications
{
using namespace Core::Drivers;
using namespace Core::Control;

// ==========================================
//              Motor Definitions
// ==========================================

// --- Wheel Motors (HT8115 on CAN1) ---
// HT8115 wheels use MIT velocity mode (internal velocity loop via Kd).
// Constructor: HT8115(motorID, canIndex, reverse). canIndex=0 -> CAN1.
// Per chassis diagram in Robot_Params.hpp: FL=1, BL=2, FR=3, BR=4.
// Reverse flags: left side (FL/BL) reversed so positive RPM = forward for all corners.
Motors::HT8115 Wheel_Motor_FL(1, 0, true);   // ID 1, CAN 1
Motors::HT8115 Wheel_Motor_FR(3, 0, false);  // ID 3, CAN 1
Motors::HT8115 Wheel_Motor_BL(2, 0, true);   // ID 2, CAN 1
Motors::HT8115 Wheel_Motor_BR(4, 0, false);  // ID 4, CAN 1

// --- Leg Motors (DM J10010L_2EC on CAN1) ---
// Constructor: J10010L_2EC(id, canIndex, mode, reverse). canIndex=0 -> CAN1, MIT mode.
#if USE_6020_LEG_MOTOR
// (DJI GM6020 leg motors removed in this build)
#elif USE_HT_LEG_MOTOR
// (HT8115 leg motors removed in this build \u2014 HT now drives wheels)
#elif USE_DM_LEG_MOTOR
// Per chassis diagram: FL=ID1, BL=ID2, FR=ID3, BR=ID4 (matches HT wheel IDs).
// Reverse flags initially false; flip per corner during bring-up if rotation
// direction is opposite of expected (positive Kp·error -> extend leg).
// canIndex=1 -> CAN2.
Motors::J10010L_2EC Leg_Motor_FL(1, 1, Motors::DMMotor::EMotorMode::eModeMIT, false);
Motors::J10010L_2EC Leg_Motor_FR(3, 1, Motors::DMMotor::EMotorMode::eModeMIT, false);
Motors::J10010L_2EC Leg_Motor_BL(2, 1, Motors::DMMotor::EMotorMode::eModeMIT, false);
Motors::J10010L_2EC Leg_Motor_BR(4, 1, Motors::DMMotor::EMotorMode::eModeMIT, false);
#endif

// ==========================================
//               PID / MIT Definitions
// ==========================================

// Wheel control: HT internal velocity loop \u2014 no external PID needed.

// --- Leg PIDs / MIT Params ---
#if USE_6020_LEG_MOTOR
PID::Param LEG_VEL_PID_PARAM(18.0f, 1.0f, 0.0f, 5000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param LEG_POS_PID_PARAM(2500.0f, 0.0f, 120.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID Leg_Vel_PID_FL(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_FL(LEG_POS_PID_PARAM);
PID Leg_Vel_PID_FR(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_FR(LEG_POS_PID_PARAM);
PID Leg_Vel_PID_BL(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_BL(LEG_POS_PID_PARAM);
PID Leg_Vel_PID_BR(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_BR(LEG_POS_PID_PARAM);
#elif USE_HT_LEG_MOTOR
MIT_Params Leg_MIT_Param_FL = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_FR = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BL = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BR = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
#elif USE_DM_LEG_MOTOR
// DM J10010L_2EC has 10:1 reduction & much higher torque output.
// Start conservative; tune up after first bring-up.
// FFW_Current field carries TORQUE (Nm) for DM (no current conversion).
MIT_Params Leg_MIT_Param_FL = {.Position = 0, .Velocity = 0, .Pos_KP = 5.0f, .Vel_KD = 0.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_FR = {.Position = 0, .Velocity = 0, .Pos_KP = 5.0f, .Vel_KD = 0.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BL = {.Position = 0, .Velocity = 0, .Pos_KP = 5.0f, .Vel_KD = 0.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BR = {.Position = 0, .Velocity = 0, .Pos_KP = 5.0f, .Vel_KD = 0.5f, .FFW_Current = 0};
#endif

// ==========================================
//           Wheel_Leg Instantiation
// ==========================================

#if USE_6020_LEG_MOTOR
Wheel_Leg FL_Leg(&Wheel_Motor_FL, &Leg_Motor_FL, &Leg_Vel_PID_FL, &Leg_Pos_PID_FL, GM6020_ID3_OFFSET, 1, -1.0f);
Wheel_Leg FR_Leg(&Wheel_Motor_FR, &Leg_Motor_FR, &Leg_Vel_PID_FR, &Leg_Pos_PID_FR, GM6020_ID1_OFFSET, 1, -1.0f);
Wheel_Leg BL_Leg(&Wheel_Motor_BL, &Leg_Motor_BL, &Leg_Vel_PID_BL, &Leg_Pos_PID_BL, GM6020_ID2_OFFSET, -1, 1.0f);
Wheel_Leg BR_Leg(&Wheel_Motor_BR, &Leg_Motor_BR, &Leg_Vel_PID_BR, &Leg_Pos_PID_BR, GM6020_ID4_OFFSET, -1, 1.0f);
#elif USE_HT_LEG_MOTOR
Wheel_Leg FL_Leg(&Wheel_Motor_FL, &Leg_Motor_FL, Leg_MIT_Param_FL, HT8115_ID3_OFFSET, 1, -1.0f);
Wheel_Leg FR_Leg(&Wheel_Motor_FR, &Leg_Motor_FR, Leg_MIT_Param_FR, HT8115_ID1_OFFSET, 1, -1.0f);
Wheel_Leg BL_Leg(&Wheel_Motor_BL, &Leg_Motor_BL, Leg_MIT_Param_BL, HT8115_ID2_OFFSET, 1, 1.0f);
Wheel_Leg BR_Leg(&Wheel_Motor_BR, &Leg_Motor_BR, Leg_MIT_Param_BR, HT8115_ID4_OFFSET, 1, 1.0f);
#elif USE_DM_LEG_MOTOR
// Offsets indexed by motor CAN ID per Robot_Params diagram: FL=1, BL=2, FR=3, BR=4.
Wheel_Leg FL_Leg(&Wheel_Motor_FL, &Leg_Motor_FL, Leg_MIT_Param_FL, DM_LEG_ID1_OFFSET, 1, 1.0f);
Wheel_Leg FR_Leg(&Wheel_Motor_FR, &Leg_Motor_FR, Leg_MIT_Param_FR, DM_LEG_ID3_OFFSET, 1, -1.0f);
Wheel_Leg BL_Leg(&Wheel_Motor_BL, &Leg_Motor_BL, Leg_MIT_Param_BL, DM_LEG_ID2_OFFSET, 1, 1.0f);
Wheel_Leg BR_Leg(&Wheel_Motor_BR, &Leg_Motor_BR, Leg_MIT_Param_BR, DM_LEG_ID4_OFFSET, 1, -1.0f);
#endif

// ==========================================
//           Chassis Instantiation
// ==========================================

Chassis chassis(&FL_Leg, &FR_Leg, &BL_Leg, &BR_Leg);

}  // namespace Applications
