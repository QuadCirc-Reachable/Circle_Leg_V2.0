#pragma once
#include "Climbing_Dynamics.hpp"
#include "Comm_Msg.hpp"
#include "Controller.hpp"
#include "Cust_Types.hpp"
#include "Ground_Contact.hpp"
#include "IMU.hpp"
#include "Impedance_Controller.hpp"
#include "Math.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
using namespace Core::Drivers;

// =========================================================================
// Debug Global Variables — for direct Ozone watch
// =========================================================================

struct DbgIMU
{
    float pitch   = 0.0f;
    float roll    = 0.0f;
    float accel_z = 0.0f;
};

struct DbgLeveling
{
    float h_fl = 0.0f;
    float h_fr = 0.0f;
    float h_bl = 0.0f;
    float h_br = 0.0f;
};

struct DbgClimbing
{
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
    uint8_t phase_fl = 0;
    uint8_t phase_fr = 0;
    uint8_t phase_bl = 0;
    uint8_t phase_br = 0;
    // Torque residual step detection debug
    float tres_fl = 0.0f, tres_fr = 0.0f, tres_bl = 0.0f, tres_br = 0.0f;      // |residual - baseline| (Nm)
    float tbase_fl = 0.0f, tbase_fr = 0.0f, tbase_bl = 0.0f, tbase_br = 0.0f;  // LPF baseline (Nm)
    float raw_fl = 0.0f, raw_fr = 0.0f, raw_bl = 0.0f, raw_br = 0.0f;          // raw torque residual (Nm)
    float target_h = 0.0f;                                                     // Height target being sent to FL (m) — for Ozone debug
    // Climbing kinematic debug
    float beta0    = 0.0f;  // β₀ at DETECT→CLIMBING transition (rad)
    float beta_fl  = 0.0f;  // current β for FL (rad)
    float beta_fr  = 0.0f;
    float beta_bl  = 0.0f;
    float beta_br  = 0.0f;
    float theta_fl = 0.0f;  // final motor angle cmd (deg, with climb_sign/mirror)
    float theta_fr = 0.0f;
    float theta_bl = 0.0f;
    float theta_br = 0.0f;
    // Raw unsigned theta from Climbing_Dynamics (deg, 180=highest, 0=lowest)
    float raw_theta_fl = 0.0f, raw_theta_fr = 0.0f, raw_theta_bl = 0.0f, raw_theta_br = 0.0f;
    float wheel_rpm = 0.0f;  // climbing forward RPM being commanded
};

struct DbgControl
{
    int state_cmd              = -1;
    float step_height_mm       = 100.0f;
    float torque_res_threshold = 3.2f;  // Ozone-tunable: step detection threshold (Nm)
    float climb_omega          = 1.0f;  // Ozone-tunable: climbing trajectory speed (rad/s)
    float climb_wheel_scale    = 5.0f;  // Ozone-tunable: wheel RPM multiplier vs theoretical (>1 = faster)
    float climb_pitch_bias     = 3.0f;  // Ozone-tunable: forward pitch lean during front climbing (deg)
};

// 接地补偿 (warp mode)
struct DbgGroundContact
{
    float warp_error = 0.0f;  // 对角电流差 (A)
    float warp_dh    = 0.0f;  // 补偿量 (m)
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
};

// 阻抗控制器 (variable impedance)
struct DbgImpedance
{
    float kp_fl = 0.0f, kp_fr = 0.0f, kp_bl = 0.0f, kp_br = 0.0f;
    float kd_fl = 0.0f, kd_fr = 0.0f, kd_bl = 0.0f, kd_br = 0.0f;
    float ffw_fl = 0.0f, ffw_fr = 0.0f, ffw_bl = 0.0f, ffw_br = 0.0f;
    float mass_est   = 0.0f;
    float warp_error = 0.0f;
    float vz         = 0.0f;
};

// Torque residual monitor — updated in ALL modes at end of Update()
// Use this in Ozone to observe step-contact torque spikes in any mode.
struct DbgTorque
{
    // Raw torque feedback (Nm)
    float torque_fl = 0.0f, torque_fr = 0.0f, torque_bl = 0.0f, torque_br = 0.0f;
    // Gravity compensation torque (Nm)
    float grav_fl = 0.0f, grav_fr = 0.0f, grav_bl = 0.0f, grav_br = 0.0f;
    // Torque residual (Nm) = torque - gravity  (spikes on step contact)
    float res_fl = 0.0f, res_fr = 0.0f, res_bl = 0.0f, res_br = 0.0f;
    // LPF baseline of residual (Nm) — slow-moving average
    float base_fl = 0.0f, base_fr = 0.0f, base_bl = 0.0f, base_br = 0.0f;
    // Deviation = |residual - baseline| (Nm) — this is the step-detection signal
    float dev_fl = 0.0f, dev_fr = 0.0f, dev_bl = 0.0f, dev_br = 0.0f;
};

// Leg angle monitor — updated in ALL modes at end of Update()
struct DbgLeg
{
    float cmd_fl = 0.0f, cmd_fr = 0.0f, cmd_bl = 0.0f, cmd_br = 0.0f;  // Final motor command (deg, post-slew)
    float fb_fl = 0.0f, fb_fr = 0.0f, fb_bl = 0.0f, fb_br = 0.0f;      // Motor feedback (deg)
};

// Wheel motor health monitor — updated in ALL modes
struct DbgWheel
{
    float out_fl = 0.0f, out_fr = 0.0f, out_bl = 0.0f, out_br = 0.0f;      // Motor output command
    float cur_fl = 0.0f, cur_fr = 0.0f, cur_bl = 0.0f, cur_br = 0.0f;      // Current feedback (A)
    float rpm_fl = 0.0f, rpm_fr = 0.0f, rpm_bl = 0.0f, rpm_br = 0.0f;      // RPM feedback
    float temp_fl = 0.0f, temp_fr = 0.0f, temp_bl = 0.0f, temp_br = 0.0f;  // Temperature (°C)
    // Torque utilization: |output| / 16000 (0~1, 1=saturated)
    float util_fl = 0.0f, util_fr = 0.0f, util_bl = 0.0f, util_br = 0.0f;
    // Target RPM (final, after compensation)
    float tgt_fl = 0.0f, tgt_fr = 0.0f, tgt_bl = 0.0f, tgt_br = 0.0f;
    // RPM error = target - actual (positive = stalling, negative = spinning free/slipping)
    float err_fl = 0.0f, err_fr = 0.0f, err_bl = 0.0f, err_br = 0.0f;
};

// Aggregated summary — one struct to watch in Ozone for the most useful signals
// across all modes (motor angles, leg heights, IMU state, mass estimate, target).
struct DbgSummary
{
    // Motor angle feedback (deg, normalized to [-180, 180])
    float angle_fl = 0.0f, angle_fr = 0.0f, angle_bl = 0.0f, angle_br = 0.0f;

    // Per-leg current height (m), computed from motor angle via
    //   H = R - r * cos(theta_motor)
    float height_fl = 0.0f, height_fr = 0.0f, height_bl = 0.0f, height_br = 0.0f;
    float height_avg = 0.0f;  // average of the four legs

    // Target chassis height (m) — what Set_Leg_Height is being asked for
    float target_height = 0.0f;

    // IMU state (chassis frame, after mounting transform + level trim)
    float imu_pitch      = 0.0f;  // deg, nose-up positive
    float imu_roll       = 0.0f;  // deg, right-up positive
    float imu_pitch_rate = 0.0f;  // deg/s (gyro)
    float imu_roll_rate  = 0.0f;
    float imu_accel_z    = 0.0f;  // m/s², earth frame Z-up, gravity removed

    // Sprung-mass / impedance estimator
    float mass_est = 0.0f;  // kg

    // Mode (mirrors dbg_state) for one-stop visibility
    uint8_t state = 0;
};

extern DbgIMU dbg_imu;
extern DbgLeveling dbg_leveling;
extern DbgClimbing dbg_climb;
extern DbgControl dbg_ctrl;
extern DbgGroundContact dbg_gc;
extern DbgImpedance dbg_imp;
extern DbgTorque dbg_torque;
extern DbgLeg dbg_leg;
extern DbgWheel dbg_wheel;
extern DbgSummary dbg_summary;
// Values: 0=CALIBRATION,1=IDLE,2=ENERGY_SAVING,3=COMFORT,4=CLIMBING,5=FREE_CONTROL,6=DEBUG,7=ERROR
extern volatile uint8_t dbg_state;

// =========================================================================

class Chassis
{
   private:
    Chassis_State current_state_ = Chassis_State::CALIBRATION;
    Controller controller_;
    Wheel_Leg *FL_WheelLegs_;
    Wheel_Leg *FR_WheelLegs_;
    Wheel_Leg *BL_WheelLegs_;
    Wheel_Leg *BR_WheelLegs_;

    // Precomputed geometry (meters), init in constructor
    float R_m_;            // wheel radius
    float r_m_;            // eccentric offset (leg length)
    float wb_m_;           // wheelbase
    float wt_f_m_;         // front track width
    float max_pitch_deg_;  // max PID input clamp for pitch
    float max_roll_deg_;   // max PID input clamp for roll
    float h_max_;          // height upper bound
    float h_min_;          // height lower bound

    float target_chassis_height_                 = 0.0f;     // Filtered (slew-rate-limited) height in meters — what executors consume
    float target_height_setpoint_                = 0.0f;     // Step setpoint written by buttons / modes; slewed into target_chassis_height_
    float height_slew_rate_                      = 0.0f;     // m/s; current slew speed (signed), used as Kd feedforward for legs
    static constexpr float HEIGHT_SLEW_PER_CYCLE = 0.0004f;  // 0.0004 m/cycle × 500 Hz = 0.2 m/s ramp

    // todo3: ES → COMFORT 软启动窗口。Set_Mode 在 ES → COMFORT 切换瞬间把
    // 这个计数器置为 ES2COMFORT_LIMIT_TICKS（窗口长度），slewTargetHeight
    // 在每个 tick 内消费（递减）一次；窗口内上升速率被压低到
    // ES2COMFORT_MAX_H_DOT 以下，窗口外恢复正常 HEIGHT_SLEW_PER_CYCLE。
    int es_to_comfort_limit_ticks_ = 0;

    // Ground Contact Warp Compensator (COMFORT / CLIMBING modes)
    GroundContact ground_contact_;

    // Impedance Controller (COMFORT mode — alternative to position-based suspension)
    Impedance_Controller impedance_;

    // Climbing Dynamics Controller (CLIMBING mode)
    Climbing_Dynamics climbing_;

    // IMU-derived values (updated each cycle by readAndTransformIMU)
    float chassis_pitch_      = 0.0f;  // deg
    float chassis_roll_       = 0.0f;  // deg
    float chassis_accel_z_    = 0.0f;  // m/s² (gravity removed)
    float chassis_pitch_rate_ = 0.0f;  // rad/s
    float chassis_roll_rate_  = 0.0f;  // rad/s

    uint8_t last_button_status_ = 0;

    // Mode transition: smooth Kp ramp when leaving COMFORT
    static constexpr int TRANSITION_FRAMES = 200;  // 0.4s @500Hz
    int mode_transition_timer_             = 0;
    float exit_kp_[4]                      = {35.0f, 35.0f, 35.0f, 35.0f};  // Last impedance Kp per leg
    float exit_kd_[4]                      = {1.5f, 1.5f, 1.5f, 1.5f};      // Last impedance Kd per leg

    // ---- COMFORT internal sub-state machine ----
    // HOMING: drive each leg from whatever pose ENERGY_SAVING (or any prior
    //   mode) left it in to a safe θ ∈ [+/-]COMFORT_HOMING_THETA, away from
    //   the kinematic singularity at θ=0,π. Smoothly hands off Kp from the
    //   previous mode's stiffness to COMFORT_HOMING_KP_END, so legs never
    //   get released. Wheels held at 0 RPM.
    // RUN: normal impedance + body-leveling control.
    enum class ComfortPhase : uint8_t
    {
        HOMING = 0,
        RUN    = 1
    };
    ComfortPhase comfort_phase_                  = ComfortPhase::HOMING;
    int comfort_homing_ticks_                    = 0;
    float comfort_homing_kp_start_[4]            = {60.0f, 60.0f, 60.0f, 60.0f};
    float comfort_homing_kd_start_[4]            = {2.5f, 2.5f, 2.5f, 2.5f};
    static constexpr int COMFORT_HOMING_FRAMES   = 250;    // 0.5s @500Hz
    static constexpr float COMFORT_HOMING_THETA  = 90.0f;  // deg — middle of workspace (sin²=1)
    static constexpr float COMFORT_HOMING_KP_END = 30.0f;  // Final Kp at end of homing
    static constexpr float COMFORT_HOMING_KD_END = 2.0f;
    // Sin-based gravity-FFW during HOMING uses this chassis mass guess so the
    // static equilibrium during HOMING matches the equilibrium RUN converges
    // to after impedance ramps in. Without it, sag during HOMING (FFW=0,
    // Kp=30) is ~10° → leg lands at 81° instead of 90°, then RUN's FFW pushes
    // the leg the remaining 9° → visible "second-stage" climb. The same value
    // is also seeded into Impedance_Controller::seedMass at handoff to skip
    // the mass-warmed snap.
    //
    // Semantics match Impedance_Controller's M: SPRUNG mass only (total robot
    // minus the four legs, which are unsprung at the wheel end). Derive from
    // Robot_Params so this tracks edits to ROBOT_MASS_kg / LEG_MASS_kg
    // automatically. With ROBOT_MASS=39, LEG_MASS=4, RIDER=50 → 73 kg.
    //
    // Includes the expected RIDER_MASS_kg because the dominant use case is
    // ridden — guessing the loaded value avoids ~6–7° sag at HOMING when a
    // rider is on. Running empty causes mild overshoot (~2°) that the on-line
    // estimator corrects within ~1 s of entering RUN; either way the visible
    // second-stage motion is gone.
    static constexpr float COMFORT_HOMING_CHASSIS_MASS_GUESS = (ROBOT_MASS_kg - 4.0f * LEG_MASS_kg) + RIDER_MASS_kg;

    // ---- ENERGY_SAVING internal sub-state machine ----
    // HOMING: drive each leg from whatever pose the previous mode left it in
    //   back to motor-frame θ=0 (the folded ES stance). Uses a smoothstep
    //   angle ramp + simultaneous Kp/Kd ramp from the previous mode's
    //   stiffness to the ES hold values. During HOMING we DO NOT add
    //   Wheel_Compensation: the decoupling FF is proportional to leg_rpm
    //   and at homing speeds it would drive the wheels noticeably (front
    //   and back axles in opposite directions) — that was the "rear wheels
    //   creeping forward / front legs flipping back" symptom.
    // RUN: normal ENERGY_SAVING — wheels follow joystick, legs hold θ=0.
    enum class EnergyPhase : uint8_t
    {
        HOMING = 0,
        RUN    = 1
    };
    EnergyPhase energy_phase_                   = EnergyPhase::RUN;
    int energy_homing_ticks_                    = 0;
    float energy_homing_kp_start_[4]            = {0.0f, 0.0f, 0.0f, 0.0f};
    float energy_homing_kd_start_[4]            = {0.5f, 0.5f, 0.5f, 0.5f};
    float energy_homing_start_angle_[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr int ENERGY_HOMING_FRAMES   = 400;  // 0.8s @500Hz — slow & smooth
    static constexpr float ENERGY_HOMING_KP_END = 80.0f;
    static constexpr float ENERGY_HOMING_KD_END = 4.0f;

    // ---- CLIMBING internal sub-state machine ----
    // Wraps the leg-level Climbing_Dynamics state machine with two zero-pose
    // stages so leg motors NEVER need to cross the ±π wrap boundary during
    // mode entry/exit (DM motors accumulate multi-turn internally even with
    // P_MAX=π, so position-control commands across the wrap would unwind
    // accumulated turns on the next mode switch — see Chassis.cpp notes).
    //
    // HOMING_IN  : smoothstep all legs from entry angle → motor-frame 0°.
    //              No climbing logic active. Wait at 0° for user to start.
    // WAIT_START : hold all legs at 0°. Press X to enter ACTIVE.
    // ACTIVE     : normal climbing pipeline (PREP/DETECT/CLIMBING/COMPLETE).
    // HOMING_OUT : user pressed a mode-switch button OR finished climb;
    //              smoothstep legs from current angle → 0° before performing
    //              the actual Chassis_State transition.
    enum class ClimbStage : uint8_t
    {
        HOMING_IN  = 0,
        WAIT_START = 1,
        ACTIVE     = 2,
        HOMING_OUT = 3
    };
    ClimbStage climb_stage_                    = ClimbStage::HOMING_IN;
    int climb_homing_ticks_                    = 0;
    float climb_homing_start_angle_[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
    Chassis_State climb_pending_exit_state_    = Chassis_State::IDLE;
    uint8_t climb_last_buttons_                = 0;
    static constexpr int CLIMB_HOMING_FRAMES   = 400;  // same as ENERGY_HOMING_FRAMES
    static constexpr float CLIMB_HOMING_KP_END = 80.0f;
    static constexpr float CLIMB_HOMING_KD_END = 4.0f;

   public:
    Chassis() = delete;
    /**
     * @brief Chassis
     * @param fl Front-left Wheel_Leg pointer
     * @param fr Front-right Wheel_Leg pointer
     * @param bl Back-left Wheel_Leg pointer
     * @param br Back-right Wheel_Leg pointer
     */
    Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br);

    //===================//
    //==== User API =====//
    //===================//

    /**
     * @brief Initialize the chassis subsystem
     */
    void Init();

    /**
     * @brief Main update loop
     * @param cmd Command from PC or remote controller
     */
    void Update(const Protocol::PC_Msg &cmd);

    /**
     * @brief Get the Reachable_Msg for feedback
     * @param msg Pointer to the Reachable_Msg to be filled
     */
    void Get_Msg(Protocol::Reachable_Msg *msg);
    //---------------------------------------------------------------------------------------------//

    //======================//
    //==== Internal API ====//
    //======================//
    /**
     * @brief Set the Chassis Mode
     * @param new_state New chassis state
     * */
    void Set_Mode(Chassis_State new_state);

    /**
     * @brief Get the current Chassis Mode
     * @return Current chassis state
     */
    Chassis_State Get_Mode() const { return current_state_; }

    //---------------------------------------------------------------------------------------------//

    //===

   private:
    // --- 各种模式的处理函数 ---
    void handleCalibrationMode();
    void handleEnergySaving(const Protocol::PC_Msg &cmd);
    void handleFreeControl(const Protocol::PC_Msg &cmd);
    void handleDebugMode(const Protocol::PC_Msg &cmd);

    // 舒适模式：核心是主动悬挂算法
    // 输入：IMU数据 (Roll, Pitch, Z-accel)
    // 输出：调整腿的角度和轮子的力矩
    void handleComfortMode(const Protocol::PC_Msg &cmd);

    // 攀爬模式：可能涉及到重心调整或特殊的步态
    void handleClimbingMode(const Protocol::PC_Msg &cmd);

    // ===== 共享管线 (Shared Pipeline) =====
    // 读取IMU并转换到底盘坐标系
    void readAndTransformIMU();
    // 按钮 → 目标高度映射 (可在各模式中覆盖)
    void handleHeightButtons(const Protocol::PC_Msg &cmd);
    // 共享的车身控制管线：自动平衡PID + 高度分配 + 轮速控制
    // mode_dh[4]: 由模式特定算法提供的额外高度补偿 (FL, FR, BL, BR)
    void executeBodyControl(const Protocol::PC_Msg &cmd, const float mode_dh[4]);
    // Impedance variant: uses per-leg Kp/Kd/FFW from Impedance_Controller
    void executeBodyControlImpedance(const Protocol::PC_Msg &cmd);
    void executeMotorCommands();
    void updateWheelDebug();
    // Refresh DbgSummary aggregate (angles, heights, IMU, mass, state).
    // Cheap; safe to call every cycle including during PC disconnect.
    void updateDbgSummary();
    float clampHeight(float h) const { return h < h_min_ ? h_min_ : (h > h_max_ ? h_max_ : h); }
    // Slew-rate-limit target_chassis_height_ toward target_height_setpoint_.
    // Prevents instantaneous height jumps from button presses or mode changes
    // from producing huge MIT command angle steps × high Kp = violent slams.
    void slewTargetHeight();

    // 运动学解算：将底盘整体速度(Vx, Vy, Wz)分解为4个轮子的速度
    void inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms);

    // Helper functions
    float CalculateHeightFromAngle(float angle_deg);
    void SetBendingDirection(int fl, int fr, int bl, int br);
};

}  // namespace Applications