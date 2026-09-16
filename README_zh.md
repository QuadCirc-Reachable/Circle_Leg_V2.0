# Circle_Leg_V2 — REACHABLE（QuadCirc）全尺寸轮腿轮椅固件

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32G473-03234B?logo=stmicroelectronics&logoColor=white" alt="STM32G473">
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-5CB85C" alt="FreeRTOS">
  <img src="https://img.shields.io/badge/C%2B%2B-GNU%2B%2B14-00599C?logo=cplusplus&logoColor=white" alt="C++">
  <img src="https://img.shields.io/badge/Control-500%20Hz-orange" alt="500 Hz">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License">
</p>

<p align="center">
  <a href="README.md">English</a> | <b>中文</b>
</p>

<p align="center">
  <img src="docs/figures/cover.jpg" width="360" alt="全尺寸 REACHABLE 原型在港科大 ISD 展出">
</p>
<p align="center"><sub>全尺寸原型在港科大 ISD 展出</sub></p>

**REACHABLE（QuadCirc）全尺寸电动轮椅** 的嵌入式控制固件。整车由四个 **CircLeg 偏心轮腿模块** 组成：
在平地上它就是普通的轮子；遇到轮椅使用者每天都会碰到的 5–18 cm 路沿、门槛和单级台阶时，它变成一条
腿，把底盘抬过障碍。香港科技大学毕业设计项目 **SL05a-25**（2025–2026）。

---

## 项目概述

每个 CircLeg 把轮毂放在半径为 *R* 的轮子内、偏离轮心 *r* 的位置。转动轮毂即可升降该角的底盘，而且
车轮全程不离地。每个角只用 **两个电机**（一个轮电机、一个腿电机），固件实现了：

- **行驶**：与普通轮椅一样——针对梯形底盘的差速（skid-steer）逆运动学、D-pad 四档速度、加速度限幅与平滑。
- **舒适模式（COMFORT）**：变阻抗主动悬挂，把虚拟笛卡尔弹簧阻尼映射到每条腿的 MIT 增益；IMU 车身调平；
  在线质量估计；以及保证四轮着地的 **Warp（对角扭转）补偿**。
- **攀爬模式（CLIMBING）**：逐腿状态机，用电机 **力矩残差** 加轮速骤降检测台阶接触，再按纯几何轨迹
  爬升；前轮对先爬，后轮对随后。
- **安全**：上位机断连检测（500 ms）、平滑的模式切换、奇异点限幅，以及可在 SEGGER Ozone 中实时调参的调试结构体。

相关仓库：

| 仓库 | 内容 |
|------|------|
| **Circle_Leg_V2**（本仓库） | 全尺寸原型固件（最终交付），以及 PCB 与结构件 |
| [Circle_Leg_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V1.0) | 半尺寸概念验证原型固件 |
| [Circle_Leg_Host_V2.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V2.0) | 上位机软件：手柄链路 + 可选 RealSense 台阶视觉（Python 包） |
| [Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0) | 旧版单文件手柄 → 串口桥接 |

---

## 演示

<p align="center">
  <a href="https://youtu.be/onJCvx1d8Sw">
    <img src="docs/figures/demo_video.jpg" width="640" alt="REACHABLE 项目视频（YouTube）">
  </a>
</p>

<p align="center">
  ▶ <a href="https://youtu.be/onJCvx1d8Sw">在 YouTube 观看 REACHABLE 项目视频</a>
</p>

---

## 主要结果

全尺寸原型，数据引自团队最终项目报告（2026 年 6 月）：

| 目标 | 指标 | 实测 |
|------|------|------|
| 基本行驶 | ≥ 1.2 m/s，差速转向 | 工作速度 **1.49 m/s**（最高约 2.2 m/s），断连急停 |
| CircLeg 驱动 | ±180° 在 3 s 内完成 | **约 0.8 s**，临界阻尼无超调 |
| 台阶攀爬 | 5–18 cm，座椅俯仰在 ±15° 内 | **5–18 cm**，座椅俯仰在 **±5°** 内 |
| 姿态与悬挂 | 坡面上调平在 ±10° 内 | 底盘调平在 **±2–3°** 内；Warp 补偿正常工作 |

> 以上数据由车载遥测（电机编码器、IMU、CAN 上报电流）和调试器观测得到，未使用外部测量设备。

---

## 系统架构

```
┌──────────────────────────┐  UART 2 Mbit/s (CH343)   ┌──────────────────────────────────────────────┐
│ Host (Jetson Orin Nano)  │ ── PC_Msg @ 30 Hz ─────► │ STM32G473 · FreeRTOS                         │
│ Circle_Leg_Host_V2       │ ◄── Reachable_Msg ────── │                                              │
│ Xbox gamepad → frames    │                          │  PC_Comm (link watchdog 500 ms)              │
└──────────────────────────┘                          │     │                                        │
                                                      │     ▼                                        │
┌──────────────────────────┐        SPI               │  Chassis_Task  (500 Hz, 2 ms)                │
│ IMU  ICM-42688-P         │ ───────────────────────► │  ┌────────────────────────────────────────┐  │
└──────────────────────────┘                          │  │ Chassis state machine                  │  │
                                                      │  │ IDLE · ENERGY_SAVING · COMFORT · CLIMB │  │
                                                      │  ├────────────────────────────────────────┤  │
                                                      │  │ Impedance · GroundContact (warp) ·     │  │
                                                      │  │ Climbing FSM · Body PID · Inverse kin. │  │
                                                      │  └───────────────────┬────────────────────┘  │
                                                      │        Wheel_Leg ×4 (FL · FR · BL · BR)       │
                                                      └──────────┬─────────────────────┬─────────────┘
                                                        CAN 1 Mbit/s            CAN 1 Mbit/s
                                                                 ▼                     ▼
                                                  4× HT8115 wheel motors   4× DM J10010L-2EC leg motors
                                                  (MIT velocity mode)      (MIT impedance mode)
```

<p align="center">
  <img src="docs/figures/system_architecture.png" width="760" alt="系统架构">
</p>

底盘任务每 2 ms 读取一次最新的手柄指令和 IMU 姿态，运行当前模式的控制器，把每条腿的 **高度** 目标
换算成腿角度和 MIT 指令 `{θ, ω, Kp, Kd, τ_ff}`，为每个电机各发一帧 CAN，并把遥测排队发回上位机。

---

## 控制设计

### 偏心轮腿运动学

<p align="center">
  <img src="docs/figures/circleg_geometry.png" width="360" alt="CircLeg 几何关系">
</p>

轮毂偏心距 *r* = 90 mm，轮半径 *R* = 177.5 mm。V2 固件的腿角度从 **最低位姿**（θ = 0°）量到
**最高位姿**（θ = 180°）：

$$H(\theta) = R - r\cos\theta, \qquad \theta = \arccos\frac{R - H}{r}, \qquad \frac{\partial H}{\partial \theta} = r\sin\theta$$

雅可比在 0° 和 180° 处为零，因此每一层控制器都会把工作范围限制在奇异点之外。

### 底盘模式

| 模式 | 车轮 | 腿 | 说明 |
|------|------|----|------|
| **IDLE** | 停止 | 低刚度保持 | 安全状态；上位机断连时也进入该状态 |
| **ENERGY_SAVING** | 跟随摇杆 | 接近 0°（最低位姿），高刚度保持 | 最省电；从 COMFORT 退出时平滑过渡 |
| **COMFORT** | 跟随摇杆 | 变阻抗悬挂 + 调平 + Warp 补偿 | 高度预设在 A/X/LB/RB/Y/B |
| **CLIMBING** | 摇杆 + 自动前进 | 逐腿攀爬状态机 | 前轮对先爬，后轮对随后 |

`ML` / `MR` 在 IDLE → ENERGY_SAVING → COMFORT → CLIMBING 之间向前 / 向后切换。CALIBRATION、
FREE_CONTROL 和 DEBUG 状态仍在代码中，但不在运行时的切换循环里。DM 腿电机的零点已写入 flash
（最低位姿），因此上电后无需标定。

### COMFORT — 变阻抗主动悬挂

在每个轮子的接地点设一个虚拟笛卡尔弹簧阻尼 $(k_v, c_v)$，通过偏心机构的雅可比映射到关节空间：

$$K_{p,i} = \left(\frac{\partial H}{\partial \theta_i}\right)^2 k_{v,i}, \qquad K_{d,i} = \left(\frac{\partial H}{\partial \theta_i}\right)^2 c_v, \qquad \tau_{ff,i} = \left(\frac{\hat M}{4} + m_\text{leg}\right) g \,\frac{\partial H}{\partial \theta_i} + \tau_{\text{warp},i}$$

- **质量估计** $\hat M$：各腿电流之和经低通滤波得到。高度目标正在变化时冻结估计，避免卸载瞬态把前馈拉塌。
- **车身调平**：Roll / Pitch PID 输出每条腿的高度偏移。陀螺角速度阻尼加上随负载自适应的增益缩放，
  使同一套参数在空载和载人时都稳定。
- **Warp**：见下一节。受力偏大的对角变软，受力偏小的一侧被压向地面。

### Warp（对角扭转）补偿

<p align="center">
  <img src="docs/figures/four_dof_modes.png" width="380" alt="Heave / Pitch / Roll / Warp 四个模态">
</p>

四点接地的刚性底盘有四个高度模态：heave、pitch、roll 和 **warp**。Pitch 和 Roll 的 PID 看不到 warp，
所以在不平地面上会出现某个轮子悬空。固件不加任何额外传感器，直接用对角电流差来测量 warp：

$$e_\text{warp} = \tfrac12\left(I_{FL} + I_{BR}\right) - \tfrac12\left(I_{FR} + I_{BL}\right)$$

并为它单独闭环：COMFORT 下调节 Kp 与前馈，CLIMBING 下对每条腿的高度偏移做 PI 控制。

### CLIMBING — 攀爬流程

```
HOMING_IN ──► WAIT_START ──(X)──► PREP ──► DETECT ──► CLIMBING ──► COMPLETE ──► HOMING_OUT
```

| 阶段 | 动作 |
|------|------|
| `HOMING_IN` | 所有腿平滑回到 0° |
| `WAIT_START` | 保持 0°，等待按下 X |
| `PREP` | 抬到 165°，同时车头上仰 |
| `DETECT` | 力矩残差 + 轮速骤降的合并得分 ≥ 1 即判定接触 |
| `CLIMBING` | 按纯几何轨迹爬升 |
| `COMPLETE` | 前轮对完成 → 车头下压，后轮对重复上述流程 |
| `HOMING_OUT` | 收腿退出攀爬模式 |

- **检测**：力矩残差 $\tau_\text{res} = \tau_\text{fb} - \tau_\text{gravity}$ 与自适应基线比较，
  再与轮速骤降的得分合并。前腿和后腿使用各自的阈值，转向过程中禁止触发。
- **轨迹**：腿角度按车轮翻越台阶边缘的几何关系推进，台阶高度 *h* 作为参数。前轮压在台阶上时，
  底盘自动前进推着车身越过。
- **重心转移**：前轮对爬升时车头上仰；完成后前腿下压，把重心前移以卸载后轮。所有偏移都用 smoothstep
  平滑地加入和退出。
- 前面两条腿都进入 COMPLETE 后，再按一次 **X** 可以跳过检测条件，强制后轮对开始爬，用于自动检测没有触发的情况。

完整的设计记录，包括 `dbg_ctrl` 里每个参数背后的调参过程，见
[docs/UNIFIED_CONTROL_REFACTOR.md](docs/UNIFIED_CONTROL_REFACTOR.md)。

### 行驶

梯形底盘按前后轴分别设差速增益，直线行驶时不会跑偏：

$$V_{FL/BL} = V_x - \omega_z k_{F/B}, \qquad V_{FR/BR} = V_x + \omega_z k_{F/B}, \qquad k_{F} = \frac{W_F}{W_\text{avg}},\; k_{B} = \frac{W_B}{W_\text{avg}}$$

轮腿解耦前馈会抵消轮毂转动带来的车轮位移，所以改变底盘高度时车子不会往前一窜。

---

## 目录结构

```
Circle_Leg_V2/
├── Applications/                   # 应用层（本项目代码）
│   ├── Chassis.hpp/.cpp            # 模式状态机、调平、逆运动学、攀爬调度
│   ├── Chassis_Task.hpp/.cpp       # FreeRTOS 500 Hz 控制任务
│   ├── Wheel_Leg.hpp/.cpp          # 单个 CircLeg：高度↔角度、MIT 腿指令链、轮速环
│   ├── Impedance_Controller.*      # 变阻抗悬挂、质量估计、Warp 调节
│   ├── Ground_Contact.*            # Warp PI 补偿器
│   ├── Climbing_Dynamics.*         # 逐腿攀爬状态机、台阶检测、轨迹
│   ├── Controller.*                # 摇杆 → (Vx, Wz)、D-pad 速度档、限幅 + 低通
│   ├── PC_Comm.*                   # 上位机串口链路（RosComm 帧）、断连看门狗
│   ├── Robot_Config.*              # 电机对象、CAN ID、每条腿的参数
│   ├── Robot_Params.hpp            # 几何、质量、IMU 安装、增益、限位
│   ├── Comm_Msg.hpp                # PC_Msg / Reachable_Msg 结构、按键位定义
│   ├── Cust_Types.hpp, Helper.hpp  # 公共类型与辅助函数
│   └── DM_Motor_Test.*             # DM J10010L 独立点动测试（USE_DM_MOTOR_TEST）
├── Core/                           # CubeMX 外设初始化 + UserTask.cpp（创建任务）
├── Drivers/, Middlewares/          # STM32G4 HAL、CMSIS、CMSIS-DSP（第三方）
├── RM2025-Core/                    # 子模块 → 私有 RM2025-Core（电机、CAN、IMU、RTOS）
├── hardware/
│   ├── power-distribution-board/   # KiCad 9 工程：双电池 24 V 电源分配板
│   └── interface-cad/              # SolidWorks 结构件装配体与零件
├── docs/
│   ├── TECHNICAL_REPORT.md         # 控制系统技术报告（含推导）
│   ├── UNIFIED_CONTROL_REFACTOR.md # 攀爬 / 腿控设计记录
│   ├── CODE_STRUCTURE.txt          # 类结构与控制流说明
│   └── figures/
├── Makefile, Core.mk               # GNU Make 构建（arm-none-eabi-gcc）
├── RM2024-Template-G473.ioc        # STM32CubeMX 工程
├── stm32g473vetx_flash.ld          # 链接脚本
└── startup_stm32g473xx.s
```

---

## 快速开始

### 环境要求

- `arm-none-eabi-gcc`（用 MSYS2 的 GCC 12.2 验证过）和 GNU Make
- SWD 调试器：SEGGER J-Link（Ozone / J-Flash）或 ST-Link（STM32CubeProgrammer）
- 私有仓库 [QuadCirc-Reachable/RM2025-Core](https://github.com/QuadCirc-Reachable/RM2025-Core) 的读取权限

> **RM2025-Core** 是香港科技大学 ENTERPRIZE RoboMaster 战队的内部库。本仓库以 git 子模块的形式引用
> 该私有仓库，**其代码不包含在本仓库内**。

### 克隆与编译

```bash
git clone --recurse-submodules https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0.git
cd Circle_Leg_V2.0
# 已经克隆但没带子模块？  git submodule update --init

make -j8
# → build/RM2024-Template-G473.elf / .hex / .bin
```

每个提交都锁定了它当时使用的 RM2025-Core 快照，所以检出任意历史提交后执行 `git submodule update`，
都能编出完全一致的固件。

### 烧录与调试

用 Ozone、J-Flash 或 STM32CubeProgrammer 通过 SWD 烧录 `build/RM2024-Template-G473.elf`（或 `.hex`）。
需要实时调参时，在 SEGGER Ozone 中打开 ELF，观察或修改这些全局变量：

| 全局变量 | 用途 |
|----------|------|
| `dbg_ctrl` | 可实时调整的攀爬参数（阈值、俯仰偏置、平滑时间、速度比例） |
| `DbgSummary` / `dbg_climb_plot` | 最常用的信号，可按 500 Hz 绘图 |
| `dbg_wheel_test_enable` / `dbg_wheel_test_rpm` | 轮子单独测试通道（`CHASSIS_DEBUG_SNAPSHOT`） |

### 配置

| 文件 | 可修改的内容 |
|------|--------------|
| `Applications/Robot_Params.hpp` | 几何、质量、IMU 安装与水平修正、速度档、调平增益、限位 |
| `Applications/Robot_Config.cpp` | 电机 ID、CAN 总线、方向标志 |
| `Core/Inc/AppConfig.h` | RM2025-Core 模块开关（IMU、DM / HT 电机、RosComm、`USE_DM_MOTOR_TEST`） |

---

## 操作说明

上位机上的 Xbox 手柄（见 [Circle_Leg_Host_V2.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V2.0)）：

| 输入 | 功能 |
|------|------|
| 左摇杆 | 前进 / 后退速度 |
| 右摇杆 | 转向角速度 |
| `ML` / `MR`（View / Menu） | 上一个 / 下一个底盘模式 |
| D-pad ↓ ← → ↑ | 速度档 LOW 20 / MID_LOW 40（默认）/ MID_HIGH 80 / HIGH 120 轮端 RPM |
| `A` `X`/`LB` `RB` `Y` `B` | COMFORT 高度预设：腿角度 45° / 90° / 135° / 145° / 165° |
| CLIMBING 中的 `X` | 开始攀爬（WAIT_START → PREP）；前轮对完成后再按一次可强制后轮对开始 |

---

## 通信协议

USART2，**2 Mbit/s**，RosComm 帧格式，带 CRC16：

```
| SOF 0xAA | len | ID 0xFF | CRC16(header) | payload (14 B) | CRC16(frame) |   → 21 bytes
```

`PC_Msg`（上位机 → MCU，小端、紧凑排列）：

| 字段 | 类型 | 范围 / 含义 |
|------|------|-------------|
| `left_joystick.angle_x10_msg` | int16 | 0–3600（角度 × 10），−1 = 回中 |
| `left_joystick.r_x1000_msg` | uint16 | 0–1000（摇杆幅度） |
| `right_joystick.angle_x10_msg` | int16 | 0–3600，−1 = 回中 |
| `right_joystick.r_x1000_msg` | uint16 | 0–1000 |
| `Left_trigger_x1000_msg` / `Right_trigger_x1000_msg` | uint16 | 0–1000 |
| `button_status` | uint8 | bit 0–7：LB、RB、X、A、B、Y、ML、MR |
| `dpad_status` | uint8 | bit 0–3：上、下、左、右 |

`Reachable_Msg`（MCU → 上位机，40 字节）回传每个角的腿角度（度 × 10）和轮速 RPM。字段名里仍写着
GM6020 / M3508，是半尺寸时期留下来的。

Host V2.0 还可以发送 RealSense 台阶识别结果（`0xFC`）并接收视觉控制心跳（`0xFD`）。**本固件只处理
`0xFF` 手柄链路**；要打通视觉闭环攀爬所需的 MCU 侧改动，见 Host V2.0 的
[MCU 集成指南](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V2.0/blob/main/docs/MCU_Integration_Guide.md)。

---

## 硬件

<div align="center">
<table>
  <tr>
    <td align="center"><img src="docs/figures/dev_side_view.jpg" height="340" alt="开发阶段的全尺寸原型侧视图"><br><sub>开发阶段侧视图</sub></td>
    <td align="center"><img src="docs/figures/fullsize_prototype.jpg" height="340" alt="轮腿模块正视图"><br><sub>轮腿模块正视图</sub></td>
  </tr>
</table>
</div>

| 项目 | 规格 |
|------|------|
| 主控 | STM32G473VET6，Cortex-M4F @ 170 MHz（ENTERPRIZE G4 控制板） |
| 实时系统 | FreeRTOS，1 kHz tick；底盘任务 500 Hz |
| 轮电机 | 4× HT8115，MIT 速度模式 |
| 腿电机 | 4× DM J10010L-2EC，MIT 阻抗模式 |
| 总线 | 2 路经典 CAN @ 1 Mbit/s（FDCAN 外设）；USART2 @ 2 Mbit/s 连上位机 |
| IMU | ICM-42688-P（SPI） |
| 几何 | *R* = 177.5 mm，*r* = 90 mm；轮距 620 / 480 mm（前 / 后），轴距 385 mm |
| 整车质量 | 约 39 kg |
| 电源 | 2× DJI TB48（22.8 V）；24→19 V 与 24→5 V 变换 |
| 上位机 | Jetson Orin Nano + Intel RealSense D435（台阶识别） |

### 电源分配板

`hardware/power-distribution-board/` 是一个 KiCad 9 工程。两路电池输入经 LM74700 理想二极管控制器配合
BSC070N10NS5 MOSFET 并联，带 TVS 保护和保险丝。24 V 母线用 XT30 连接器分配到八个电机接口（四个腿、
四个底盘）、迷你主机、控制板和风扇。自定义封装在 `Library.pretty/` 中。这块板由 Jason Chan 协助开发。

### 结构件

`hardware/interface-cad/` 存放 SolidWorks 装配体 `Assem3.SLDASM` 及其零件。

---

## 文档

| 文档 | 内容 |
|------|------|
| [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) | 控制系统报告：运动学、调平、阻抗、Warp、攀爬、RTOS |
| [docs/UNIFIED_CONTROL_REFACTOR.md](docs/UNIFIED_CONTROL_REFACTOR.md) | 攀爬流程与腿控设计记录，含调参历史 |
| [docs/CODE_STRUCTURE.txt](docs/CODE_STRUCTURE.txt) | 类层次、各模式控制流、关键公式 |

---

## 团队

<p align="center">
  <img src="docs/figures/team.jpg" width="560" alt="REACHABLE 团队在港科大 ISD Class of 2026 活动上">
</p>

**REACHABLE（QuadCirc）**，香港科技大学毕业设计项目 SL05a-25：

- **LIU Hualin** — 嵌入式控制负责人：电机与主控选型、双 CAN 电路，以及本仓库中的全部固件和控制算法
- **FANG Ruoyun** — 感知（RealSense 台阶识别）与人机交互
- **WU Ziyao** — 机械架构、上位机与 MCU 通信
- **XU Jusen** — 机械负责人（CircLeg 偏心支架、车架、DFM/DFA）

指导老师：Prof. Chi-Ying TSUI、Prof. Winnie Suk Wai LEUNG；协同指导：Prof. SHI Ling。

---

## 许可证

本仓库的应用层代码以 [MIT 许可证](LICENSE) 发布。第三方组件保留各自的许可证：`Drivers/` 与
`Middlewares/` 中的 STM32 HAL / CMSIS，以及来自 ENTERPRIZE RoboMaster 战队模板的文件
（`Core.mk`、`Core/Src/UserTask.cpp`）。`RM2025-Core` 子模块为私有仓库，不在本许可证覆盖范围内。

## 致谢

- 香港科技大学 ENTERPRIZE RoboMaster 战队，提供 RM2025-Core 和 G4 工程模板
- Jason GAN（RM2024），HT8115 电机驱动的原作者
- Jason Chan，协助开发电源分配板
- STMicroelectronics（HAL、CMSIS）与 FreeRTOS 项目
- 港科大 ISD 的朋友和同学们

<p align="center">
  <img src="docs/figures/isd_friends.jpg" width="480" alt="和港科大 ISD 的朋友们合影">
</p>
<p align="center"><sub>和 ISD 的朋友们合影</sub></p>
