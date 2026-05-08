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

extern DbgIMU dbg_imu;
extern DbgLeveling dbg_leveling;
extern DbgClimbing dbg_climb;
extern DbgControl dbg_ctrl;
extern DbgGroundContact dbg_gc;
extern DbgImpedance dbg_imp;
extern DbgTorque dbg_torque;
extern DbgLeg dbg_leg;
extern DbgWheel dbg_wheel;

// Current chassis state (mirrors Chassis::current_state_) for Ozone watch.
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

    float target_chassis_height_ = 0.0f;  // Nominal height in meters

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
    float clampHeight(float h) const { return h < h_min_ ? h_min_ : (h > h_max_ ? h_max_ : h); }

    // 运动学解算：将底盘整体速度(Vx, Vy, Wz)分解为4个轮子的速度
    void inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms);

    // Helper functions
    float CalculateHeightFromAngle(float angle_deg);
    void SetBendingDirection(int fl, int fr, int bl, int br);
};

}  // namespace Applications