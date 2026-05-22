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
#define RIDER_MASS_kg \
    50.0f                // Expected rider mass (kg). Added to the cold-start sprung-mass guess
                         // so HOMING FFW matches the loaded-equilibrium that RUN converges to,
                         // eliminating the second-stage settle on COMFORT entry under load.
#define GRAVITY_g 9.81f  // Gravity acceleration in m/s^2

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

// ----------------------------------------------------------
//  D-Pad 四档速度配置（全模式共享，运行时可切换）
// ----------------------------------------------------------
// 数组顺序固定：{LOW, MID_LOW, MID_HIGH, HIGH}
// 单位：RPM（电机减速后轮端）。HT 电机参考：额定 140，最大 330。
// 顶档保留余量到 280，避免长时间贴近最大转速发热。
#define SPEED_TIER_FWD_RPM {20.0f, 40.0f, 80.0f, 120.0f}
#define SPEED_TIER_TURN_RPM {20.0f, 40.0f, 80.0f, 120.0f}

// 同档加速度上限（RPM/s）。这是「绝对加速度天花板」，永远不会被打破。
// 平滑感由下面的 SPEED_SMOOTH_LPF_ALPHA 单独调节。
#define SPEED_TIER_FWD_ACCEL {40.0f, 80.0f, 120.0f, 120.0f}
#define SPEED_TIER_TURN_ACCEL {40.0f, 80.0f, 120.0f, 120.0f}

// 开机默认档位：0=LOW, 1=MID_LOW, 2=MID_HIGH, 3=HIGH
#define SPEED_TIER_DEFAULT 1

// ----------------------------------------------------------
//  推背感平滑（一阶 LPF，加在加速度限制之后）
// ----------------------------------------------------------
// 让初段加速度从 0 平滑爬升而不是阶跃，主观上「推背感变软」。
//   - alpha = 1.0  → 不平滑，等价于线性 slew
//   - alpha = 0.30 → 推荐起点，明显柔和
//   - alpha < 0.1  → 反应会明显发懒
// 该 LPF 永远不会突破上面 SPEED_TIER_*_ACCEL 的硬上限。
#define SPEED_SMOOTH_LPF_ALPHA 0.30f

// ----------------------------------------------------------
//  ES → COMFORT 抬升速度限制（todo3）
// ----------------------------------------------------------
// 从 ENERGY_SAVING 切入 COMFORT 时，腿要从近 0° 拉到 ~90°，对应底盘
// 抬升较快、乘客感觉冲。这里给一段固定时长的"软启动"窗口：
//   - 窗口内：上升速率被压到 ES2COMFORT_MAX_H_DOT 以下
//   - 窗口外：恢复正常 HEIGHT_SLEW_PER_CYCLE
// 下降方向不受影响（避免影响落地/换姿态响应）。
#define ES2COMFORT_MAX_H_DOT 0.03f   // m/s，窗口内的上升速率上限
#define ES2COMFORT_LIMIT_TICKS 2500  // 2500 tick @500Hz = 5.0 s

// ----------------------------------------------------------
//  其他保留参数（仍被 Chassis / Helper 引用）
// ----------------------------------------------------------
#define MAX_WHEEL_RPM 280.0f  // inverseKinematics 用于轮速等比缩放
#define LEG_MAX_SPEED 400.0f  // 腿角度 slew 上限 (deg/s)

// ----------------------------------------------------------
//  COMFORT 模式 — 车体调平（Roll / Pitch PID + LPF + Gyro 阻尼）
// ----------------------------------------------------------
// 该组参数集中决定"乘客主观稳不稳"。改动前先理解层级：
//   IMU 原始角 ─► (LPF) ─► PID(角度环) ─► × SCALE ─► 高度差分命令
//                                       ─► (gyro rate FF) ─► 高度差速命令
//
// 调试入口（按从安全到激进的顺序）：
//   ① 摇晃感差 → 先加 GYRO_FF_GAIN_ROLL（rate damping，不会发散）
//   ② 仍不够刚 → 加 BODY_PID_SCALE_ROLL
//   ③ 反应迟钝 → 提高 BODY_LPF_ALPHA_ROLL（最多 ~0.08，过高会激发腿共振）
//   ④ 最后才考虑改 BODY_ROLL_PID_* / BODY_PITCH_PID_*
//
// LPF α 与截止频率：fc ≈ α / (2π·DT) = α × 500 / (2π) [Hz]
//   0.02 → ~1.6 Hz   0.04 → ~3.2 Hz   0.06 → ~5 Hz   0.08 → ~6.4 Hz
//   腿模态 ω_n ≈ 9 Hz，α 不应超过 0.08，否则会与 impedance 自激。

// — Roll / Pitch PID 增益（Kp, Ki, Kd, IntLim, OutLim） —
// OutLim 单位：米（高度差分上限）。Max travel = 2r = 0.13m，留余量取 0.08。
#define BODY_ROLL_PID_KP 0.015f
#define BODY_ROLL_PID_KI 0.0002f
#define BODY_ROLL_PID_KD 0.00012f
#define BODY_ROLL_PID_INT_LIMIT 1000.0f
#define BODY_ROLL_PID_OUT_LIMIT 0.08f

#define BODY_PITCH_PID_KP 0.015f
#define BODY_PITCH_PID_KI 0.00015f
#define BODY_PITCH_PID_KD 0.00012f
#define BODY_PITCH_PID_INT_LIMIT 1000.0f
#define BODY_PITCH_PID_OUT_LIMIT 0.08f

// — IMU 角度 LPF（拆轴，roll 带宽高于 pitch） —
// roll 轨距短、模态频率高，LPF 截止需高一些才能压住摇晃；
// pitch 轴距长、本身就稳，LPF 截止低、纯做慢漂移纠偏。
#define BODY_LPF_ALPHA_ROLL 0.06f   // ~5   Hz 截止
#define BODY_LPF_ALPHA_PITCH 0.02f  // ~1.6 Hz 截止

// — PID 输出标尺（COMFORT 下 PID 权威 = PID(out) × SCALE） —
#define BODY_PID_SCALE_ROLL 1.6f   // 调参史：0.7 → 1.0 → 1.3 → 1.6（载人压屁股摇晃）
#define BODY_PID_SCALE_PITCH 0.7f  // 调参史：0.5 → 0.8(自激) → 0.7 稳定

// — Gyro 速率前馈（角速度 → 高度差分速度命令，做"虚拟阻尼"） —
// 推荐：起步 0.002，嫌晃逐步 +0.001 加到 0.005；pitch 默认关闭。
#define BODY_GYRO_LPF_ALPHA 0.05f       // 角速度 LPF
#define BODY_GYRO_FF_GAIN_ROLL 0.0035f  // 0.002 → 0.0035（载人 roll 阻尼不够）
#define BODY_GYRO_FF_GAIN_PITCH 0.0f

// — 自适应负载缩放（解决"空载乱晃 / 满载需要的增益不同"问题） —
// 上面 BODY_PID_SCALE_* 与 BODY_GYRO_FF_GAIN_* 是按"满载（带人）"调出来的。
// 空载时簧载惯量缩到 ~1/3，相同力矩导致角加速度变 3 倍 → 必然摇晃。
//   做法：用 impedance 在线估计的实际簧载质量 / 标定质量，得到运行时缩放
//        系数，乘到 PID 输出与 gyro FF 上；
//   效果：空载自动变软、满载维持标定，无需手动切档或重新调参。
// 调参口径：
//   - BODY_CTRL_NOMINAL_MASS_KG    = 当前 SCALE/FF 是在"多少 kg 簧载"下调的
//     （= ROBOT_MASS - 4×LEG_MASS + RIDER_MASS = 39 - 16 + 50 = 73）
//   - BODY_CTRL_MASS_SCALE_MIN/MAX = 缩放安全带，防止估计抖动把环开成 0 或拉飞
//     默认 0.4 ~ 1.2：空载 23/73 ≈ 0.31 会被钳到 0.4（保留最小调平能力），
//     大块头乘客 88/73 ≈ 1.20 也会被封顶。
#define BODY_CTRL_NOMINAL_MASS_KG 73.0f
#define BODY_CTRL_MASS_SCALE_MIN 0.40f
#define BODY_CTRL_MASS_SCALE_MAX 1.20f

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
