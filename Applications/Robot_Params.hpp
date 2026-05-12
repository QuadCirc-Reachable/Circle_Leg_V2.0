#pragma once

#ifndef PI
#define PI 3.1415926535f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * PI)
#endif

#define LEG_MOTOR_NUM 4
#define WHEEL_MOTOR_NUM 4

/*
                                    WHEEL_TRACK BACK
                HT8115 ID4 ------------------------- HT8115 ID2
        DM10010 ID0x04 (BR) ------------------------- DM10010 ID0x02 (BL)
                            |                     |
                            |                     |
                            |                     |
                            |                     |
                            |                     |
                            |                     |
        HT8115 ID3 ------------------------------------ HT8115 ID1
DM10010 ID0x03 (FR) ------------------------------------ DM10010 ID0x01 (FL)
                                WHEEL_TRACK FRONT


*/

// ==========================================
//        Leg Motor Configuration
// ==========================================

// Active leg motor selection. Exactly one must be 1.
// In this build: legs are DM J10010L_2EC on CAN1, wheels are HT8115 on CAN2.
#define USE_DM_LEG_MOTOR 1    // 1: use DM J10010L_2EC as leg motor;
#define USE_HT_LEG_MOTOR 0    // 1: use HT8115 as leg motor;
#define USE_6020_LEG_MOTOR 0  // 1: use DJI GM6020 as leg motor;

#if USE_6020_LEG_MOTOR
#define GM6020_ID1_OFFSET -2.050f
#define GM6020_ID2_OFFSET -0.070f
#define GM6020_ID3_OFFSET -0.040f
#define GM6020_ID4_OFFSET 1.000f

#elif USE_HT_LEG_MOTOR
#define HT8115_ID1_OFFSET 0.0f
#define HT8115_ID2_OFFSET 0.0f
#define HT8115_ID3_OFFSET 0.0f
#define HT8115_ID4_OFFSET 0.0f

#elif USE_DM_LEG_MOTOR
// ------------------------------------------------------------
// DM J10010L_2EC absolute-position offsets (radians).
//
// Definition:
//   The value is the RAW encoder reading getPositionFeedback() (rad)
//   when the leg is at its mechanical LOWEST / fully-EXTENDED pose
//   (i.e. when our logical leg_pos == 0°, see INITIAL_LEG_ANGLE).
//
// Used by Wheel_Leg as:
//   leg_pos[deg] = rad2deg( normalizeAngle( fb_rad - leg_offset ) )
//
// Calibration procedure (one-time, no need to SetZero on each power-up):
//   1. Power on, keep DM motors disabled.
//   2. Manually push each leg DOWN to its mechanical hard stop (lowest).
//   3. Read getPositionFeedback() (rad) for that motor.
//   4. Paste the value into the corresponding macro below.
//
// Unit helper: define in degrees if more convenient, then convert.
//   #define DM_LEG_ID1_OFFSET (deg2radf(123.4f))
// ------------------------------------------------------------

// Pre-measured raw encoder values at lowest pose (rad).
// All four motors had setZeroPosition() called at the mechanical lowest pose
// (DM flash already stores that as zero), so getPositionFeedback() ≈ 0 there.
#define DM_LEG_ID1_OFFSET 0.0f  // FL
#define DM_LEG_ID2_OFFSET 0.0f  // BL
#define DM_LEG_ID3_OFFSET 0.0f  // FR
#define DM_LEG_ID4_OFFSET 0.0f  // BR
#endif
// ==========================================
//        Mechanical Parameters
// ==========================================

// Wheel-Leg Geometry
#define ECCENTRIC_OFFSET_r 90.0f
#define WHEEL_RADIUS_R 177.5f

// Chassis Dimensions
#define WHEEL_TRACK_FRONT 620.0f  // Distance between Front Left and Front Right
#define WHEEL_TRACK_BACK 480.0f   // Distance between Back Left and Back Right
#define WHEEL_BASE 385.0f         // Distance between Front Axle and Back Axle

// IMU Mounting Position (Relative to Chassis Geometric Center)
// Positive X: Forward, Positive Y: Left, Positive Z: Up
#define IMU_MOUNT_X 0.0f
#define IMU_MOUNT_Y 0.0f
#define IMU_MOUNT_Z 0.0f

// IMU Mounting Orientation (Relative to Chassis Frame)
// Unit: Degrees
// Current physical mount: IMU is rotated +90° CCW about chassis Z (top-down view),
// otherwise level (no roll/pitch offset).
#define IMU_MOUNT_ROLL_DEG 0.0f
#define IMU_MOUNT_PITCH_DEG 0.0f
#define IMU_MOUNT_YAW_DEG 90.0f

// IMU level-trim offsets (degrees, in chassis frame AFTER mount transform).
// Measured while the chassis sits perfectly level on a flat surface; we subtract
// these from the transformed pitch/roll so "level" reads (0, 0).
//   Reading on flat ground (May 2026): pitch = -4°, roll = +1.3°
#define IMU_PITCH_OFFSET_DEG -4.0f
#define IMU_ROLL_OFFSET_DEG 1.3f

// Robot Physical Properties
#define ROBOT_MASS_kg 39.0f  // Total mass of the robot in kg
#define LEG_MASS_kg 4.0f     // Mass of the moving part of the leg (Motor + Wheel)
#define GRAVITY_g 9.81f      // Gravity acceleration in m/s^2

// Active Suspension Parameters
#define DLS_LAMBDA 0.1f  // Damping factor for Damped Least Squares

// Distances from IMU to Wheel Centers (Calculated or Measured)
// X-axis (Longitudinal)
#define DIST_IMU_TO_FRONT_AXLE (WHEEL_BASE / 2.0f - IMU_MOUNT_X)
#define DIST_IMU_TO_BACK_AXLE (WHEEL_BASE / 2.0f + IMU_MOUNT_X)

// Y-axis (Lateral)
#define DIST_IMU_TO_LEFT_WHEEL (WHEEL_TRACK_FRONT / 2.0f - IMU_MOUNT_Y)  // Assuming symmetric track for now
#define DIST_IMU_TO_RIGHT_WHEEL (WHEEL_TRACK_FRONT / 2.0f + IMU_MOUNT_Y)

// ==========================================
//           Control Parameters
// ==========================================

// Maximum wheel RPM
// Ground speed = RPM * 2π * R / 60.  With R=160mm, these give ~1.3 m/s fwd.
#define MAX_WHEEL_RPM 80.0f
#define MAX_FORWARD_RPM 40.0f
#define MAX_TURN_RPM 40.0f

// Forward/backward acceleration limit (RPM per second).
// Applied to vx only inside Controller::Map_Joystick_To_Velocity. Turning (wz)
// is left unbounded because chassis friction already damps it adequately.
// Lower → smoother but laggier; higher → more responsive but more jerky.
//
// Note interaction with WHEEL_RPM_DEADZONE (15 RPM) in Wheel_Leg.cpp:
// time-to-first-motion ≈ 15 / MAX_FWD_ACCEL_RPM_PER_S seconds.
//   200 RPM/s ⇒ ~75 ms before wheels start, ~0.2 s to MAX_FORWARD_RPM=40
//   100 RPM/s ⇒ ~150 ms / ~0.4 s
//    40 RPM/s ⇒ ~375 ms / ~1 s   (felt unresponsive)
#define MAX_FWD_ACCEL_RPM_PER_S 100.0f
#define MAX_TURN_ACCEL_RPM_PER_S 100.0f

// Deceleration limits (RPM per second), used when |target| < |prev| i.e. when
// slowing down or releasing the joystick. Set lower than ACCEL to soften the
// braking reaction torque that the wheel Kd loop dumps into the chassis — at
// ES θ≈0 a hard wheel brake kicks the leg and excites pitch oscillation.
#define MAX_FWD_DECEL_RPM_PER_S 40.0f
#define MAX_TURN_DECEL_RPM_PER_S 40.0f

// Maximum Leg Speed (Degrees per second)
// Used for Slew Rate Limiter to prevent violent movements
#define LEG_MAX_SPEED 400.0f

// Safety Thresholds
#define MAX_TILT_ANGLE 40.0f          // Max allowed Roll/Pitch in degrees
#define FORCE_LIFT_THRESHOLD 15.0f    // Force threshold (Newtons) for ground detection
#define LEG_EXTENDED_THRESHOLD 45.0f  // Max allowed deviation from BASE_LEG_POS before detecting "Lifted"

// Initial Configuration
#define INITIAL_LEG_ANGLE 0.0f  // Nominal operating angle (Degrees). 0=Extended, 180=Retracted.

// Joystick Parameters
#define JOYSTICK_MAX_R 1000.0f
#define JOYSTICK_DEADZONE 50.0f

// Direction Angle Ranges
// Forward range: [30, 150]
#define ANGLE_FWD_MIN 30.0f
#define ANGLE_FWD_MAX 150.0f

// Backward range: [210, 330]
#define ANGLE_BWD_MIN 210.0f
#define ANGLE_BWD_MAX 330.0f
