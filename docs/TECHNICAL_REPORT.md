# Circle Leg V1 — Final Year Project Technical Report

> **Scope note (Circle_Leg_V2):** this report was written for the half-size prototype (Circle_Leg_V1: M3508 wheels, HT8115 legs) and uses V1's angle convention $H(	heta) = R + r\cos	heta$. The control architecture carries over to V2 unchanged; see the [README](../README.md) for the full-size hardware, parameters and V2's convention $H(	heta) = R - r\cos	heta$ (legs zeroed at the lowest pose).

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Configuration](#3-hardware-configuration)
4. [Eccentric Wheel-Leg Kinematics](#4-eccentric-wheel-leg-kinematics)
5. [Chassis State Machine](#5-chassis-state-machine)
6. [Body Leveling — Pitch/Roll PID](#6-body-leveling--pitchroll-pid)
7. [Inverse Kinematics — Skid-Steer Model](#7-inverse-kinematics--skid-steer-model)
8. [Wheel–Leg Decoupling Compensation](#8-wheel-leg-decoupling-compensation)
9. [Virtual Model Control (VMC)](#9-virtual-model-control-vmc)
10. [Impedance Controller — Active Suspension](#10-impedance-controller--active-suspension)
11. [Ground Contact — Warp Compensation](#11-ground-contact--warp-compensation)
12. [Climbing Dynamics — Step Climbing](#12-climbing-dynamics--step-climbing)
13. [Motor Control Pipeline](#13-motor-control-pipeline)
14. [Communication Protocol](#14-communication-protocol)
15. [RTOS Task Structure](#15-rtos-task-structure)
16. [Summary of Key Formulas](#16-summary-of-key-formulas)
17. [File Reference](#17-file-reference)

---

## 1. Project Overview

This project implements a **4-wheeled eccentric-leg robot** (Circle Leg) on an **STM32G473** microcontroller, running **FreeRTOS** at a **500 Hz** control loop. The robot features:

- **Four independent wheel-leg units**, each consisting of a **wheel motor** (DJI M3508) and a **leg motor** (HT8115, MIT control mode).
- **Variable-impedance active suspension** (Comfort Mode) for terrain adaptation.
- **Kinematic step-climbing** (Climbing Mode) using torque-residual step detection and geometric trajectory planning.
- **Ground contact warp compensation** to maintain 4-wheel ground contact on uneven surfaces.
- **PC remote control** via UART with joystick and button inputs.

---

## 2. System Architecture

```
┌──────────────────────────────────────────────────────┐
│                    PC Controller                     │
│           (Joystick + Buttons via UART)              │
└──────────────┬───────────────────────────────────────┘
               │  PC_Msg (13 bytes)
               ▼
┌──────────────────────────────────────────────────────┐
│              Chassis_Task (500 Hz)                   │
│  ┌────────────────────────────────────────────────┐  │
│  │              Chassis (State Machine)            │  │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────────┐   │  │
│  │  │ Pitch/   │ │Impedance │ │  Climbing    │   │  │
│  │  │ Roll PID │ │Controller│ │  Dynamics    │   │  │
│  │  └──────────┘ └──────────┘ └──────────────┘   │  │
│  │  ┌──────────┐ ┌──────────┐                    │  │
│  │  │  Ground  │ │  Inverse │                    │  │
│  │  │  Contact │ │Kinematics│                    │  │
│  │  └──────────┘ └──────────┘                    │  │
│  └────────────────────────────────────────────────┘  │
│          │            │           │          │        │
│      ┌───┴───┐   ┌───┴───┐  ┌───┴───┐  ┌───┴───┐   │
│      │FL Leg │   │FR Leg │  │BL Leg │  │BR Leg │   │
│      │M3508+ │   │M3508+ │  │M3508+ │  │M3508+ │   │
│      │HT8115 │   │HT8115 │  │HT8115 │  │HT8115 │   │
│      └───────┘   └───────┘  └───────┘  └───────┘   │
└──────────────────────────────────────────────────────┘
```

---

## 3. Hardware Configuration

### 3.1 Mechanical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Wheel Radius | $R$ | 160 | mm |
| Eccentric Offset (Leg Length) | $r$ | 65 | mm |
| Wheelbase | $L$ | 270 | mm |
| Front Track Width | $W_F$ | 531 | mm |
| Back Track Width | $W_B$ | 395 | mm |
| Robot Mass | $M$ | 10.0 | kg |
| Leg Mass | $m_{leg}$ | 0.5 | kg |

### 3.2 Motor Configuration

| Motor | Type | Quantity | Bus | Role |
|-------|------|----------|-----|------|
| M3508 | Brushless DC (DJI) | 4 | CAN 2 | Wheel drive, PID velocity control |
| HT8115 | Torque motor | 4 | CAN 3 | Leg actuation, MIT impedance control |

### 3.3 IMU

- On-board IMU providing Euler angles (Yaw, Pitch, Roll), calibrated gyro, and earth-frame linear acceleration (gravity removed).
- IMU mounting roll offset: **182.91°** (compensated in software).
- Low-pass filter on angles: $\alpha = 0.02$ (~4 Hz cutoff at 500 Hz).

---

## 4. Eccentric Wheel-Leg Kinematics

Each wheel-leg unit uses an **eccentric linkage**: a motor rotates the leg by angle $\theta$, changing the chassis height.

### 4.1 Height–Angle Relationship

$$H(\theta) = R + r \cdot \cos(\theta)$$

Where:
- $H$ = ground-to-motor-center height (m)
- $R$ = wheel radius (m)
- $r$ = eccentric offset / leg length (m)
- $\theta$ = leg angle (0° = fully extended/highest, 180° = fully retracted/lowest)

### 4.2 Inverse: Height to Angle

Given a target height $H_{target}$:

$$\cos(\theta) = \frac{H_{target} - R}{r}$$

$$\theta = \arccos\left(\frac{H_{target} - R}{r}\right)$$

**Safety:** clamped to $[10°, 170°]$ to avoid singularities at 0° and 180°.

### 4.3 Velocity Feedforward (Height Rate to Angular Velocity)

From $\dot{H} = -r \sin(\theta) \cdot \dot{\theta}$:

$$\dot{\theta} = \frac{-\dot{H}}{r \cdot (\sin(\theta) + \epsilon)}$$

Where $\epsilon = 0.1$ is a damping term to avoid division-by-zero near singularities.

### 4.4 Bending Direction

Each leg has a `bending_direction` parameter ($+1$ or $-1$) that selects between the positive and negative angle solution, allowing the robot to switch between "outward" and "inward" leg configurations.

---

## 5. Chassis State Machine

The system operates through the following states, controlled by gamepad buttons:

| State | ID | Description |
|-------|----|-------------|
| **CALIBRATION** | 0 | Re-enters HT motor mode, sets zero position, then transitions to IDLE |
| **IDLE** | 1 | All motors stopped, zero output |
| **ENERGY_SAVING** | 2 | Legs at 0° (folded), stiff position hold ($K_p=20$, $K_d=1.5$), basic wheel driving |
| **COMFORT** | 3 | **Variable-impedance active suspension** with IMU-based body leveling and warp compensation |
| **CLIMBING** | 4 | **Per-leg kinematic step climbing** with torque-residual detection, front-first then rear-legs |
| **FREE_CONTROL** | 5 | Direct trigger-to-angle mapping for manual leg positioning |
| **DEBUG** | 6 | Height control via buttons, no PID leveling (for testing) |

### Mode Transition

- **ML + MR** simultaneously → CALIBRATION
- **ML** (rising edge) → Previous state
- **MR** (rising edge) → Next state

### Smooth Kp Ramp on COMFORT Exit

When leaving COMFORT mode, the impedance Kp/Kd values are captured and linearly ramped to the target mode's values over **200 frames (0.4 s)** to prevent sudden torque changes:

$$K_p(t) = K_{p,\text{exit}} + \alpha(t) \cdot (K_{p,\text{target}} - K_{p,\text{exit}})$$

where $\alpha(t) = t / T_{ramp}$ increases from 0 to 1.

---

## 6. Body Leveling — Pitch/Roll PID

Two independent PID controllers maintain chassis level using IMU feedback:

| PID | $K_p$ | $K_i$ | $K_d$ | Integral Limit | Output Limit |
|-----|--------|--------|--------|----------------|--------------|
| Roll | 0.015 | 0.0002 | 0.00012 | 1000 | 0.08 m |
| Pitch | 0.015 | 0.00015 | 0.00012 | 1000 | 0.08 m |

**PID output** is a height offset in meters. Distribution to four legs:

| Leg | Pitch Sign | Roll Sign |
|-----|------------|-----------|
| FL (Front-Left)  | $+1$ | $-1$ |
| FR (Front-Right) | $+1$ | $+1$ |
| BL (Back-Left)   | $-1$ | $-1$ |
| BR (Back-Right)  | $-1$ | $+1$ |

Per-leg target height:

$$H_i = H_{target} + s_{pitch,i} \cdot \Delta H_{pitch} + s_{roll,i} \cdot \Delta H_{roll} + \Delta H_{mode,i}$$

Where:
- $\Delta H_{pitch} = \text{PID}_{pitch}(0, \theta_{pitch})$
- $\Delta H_{roll} = \text{PID}_{roll}(0, \theta_{roll})$
- $\Delta H_{mode,i}$ = mode-specific compensation (warp, climbing, etc.)

---

## 7. Inverse Kinematics — Skid-Steer Model

The robot uses **asymmetric skid-steer** kinematics ($W_F \neq W_B$):

$$V_{FL} = V_x - \omega_z \cdot k_F, \qquad V_{FR} = V_x + \omega_z \cdot k_F$$

$$V_{BL} = V_x - \omega_z \cdot k_B, \qquad V_{BR} = V_x + \omega_z \cdot k_B$$

Where the differential gain per axle:

$$k_F = \frac{W_F}{W_{avg}}, \qquad k_B = \frac{W_B}{W_{avg}}, \qquad W_{avg} = \frac{W_F + W_B}{2}$$

**Normalization:** If any wheel RPM exceeds `MAX_WHEEL_RPM` (80 RPM), all four are proportionally scaled down.

### Joystick Mapping

- **Left stick** → Forward/backward velocity $V_x$ (max ±40 RPM)
- **Right stick** → Rotational velocity $\omega_z$ (max ±40 RPM)
- Deadzone: 200/1000 on radius magnitude

---

## 8. Wheel–Leg Decoupling Compensation

When the leg motor rotates, it mechanically couples into the wheel, causing unintended wheel spin. This is compensated by:

$$n_{wheel,comp} = n_{leg} \cdot \left(1 + \frac{r}{R} \cdot \cos(\theta_{leg})\right) \cdot k_{coupling}$$

Where:
- $n_{leg}$ = leg motor RPM
- $r / R$ = eccentric offset / wheel radius ratio
- $\theta_{leg}$ = current leg angle
- $k_{coupling}$ = coupling sign ($\pm 1$ depending on mounting geometry)

This compensation RPM is added to the wheel motor target before PID execution.

---

## 9. Virtual Model Control (VMC)

Converts a desired vertical force $F_z$ to motor torque via the mechanism Jacobian:

$$\tau = F_z \cdot r \cdot \cos(\theta)$$

Where $\tau$ is motor torque (Nm), $r$ is eccentric offset (m), and $\theta$ is leg angle (rad).

### Gravity Compensation

Each leg compensates for gravitational torque:

$$\tau_{gravity} = -(m_{leg} \cdot g \cdot r \cdot \cos(\theta))$$

This is continuously applied as **feed-forward torque** in the MIT control command.

---

## 10. Impedance Controller — Active Suspension

The core of COMFORT mode. Maps desired **Cartesian impedance** (spring-damper at the wheel contact point) to **joint-space MIT motor parameters**.

### 10.1 Theoretical Derivation

**Eccentric Linkage Jacobian:**

$$J(\theta) = \frac{dH}{d\theta} = -r \cdot \sin(\theta)$$

**Desired Cartesian impedance:**

$$F = k_v \cdot (H_{des} - H) + c_v \cdot (\dot{H}_{des} - \dot{H}) + F_{gravity}$$

**Mapping to MIT control law** ($\tau = K_p \cdot \delta\theta + K_d \cdot \delta\dot{\theta} + \tau_{ff}$):

$$\boxed{K_{p,MIT} = r^2 \cdot \sin^2(\theta) \cdot k_v}$$

$$\boxed{K_{d,MIT} = r^2 \cdot \sin^2(\theta) \cdot c_v}$$

$$\boxed{\tau_{ff} = -F_{load} \cdot r \cdot \sin(\theta)}$$

**Numerical Example** ($\theta=90°$, $r=0.065$ m):
- $r^2 \sin^2(90°) = 0.004225$
- $k_v = 800$ N/m → $K_p = 3.38$ Nm/rad
- $c_v = 250$ Ns/m → $K_d = 1.06$ Nms/rad

### 10.2 Warp Kp Modulation (Ground Contact)

Diagonal load imbalance detection:

$$e_{warp} = \frac{I_{FL} + I_{BR}}{2} - \frac{I_{FR} + I_{BL}}{2}$$

Warp sign pattern: $\{FL: +1, FR: -1, BL: -1, BR: +1\}$

$$k_{v,i} = k_{v,base} \cdot \left(1 - \gamma \cdot s_{warp,i} \cdot \text{clamp}(e_{warp})\right)$$

**Effect:** Overloaded diagonal becomes softer → compresses → redistributes load to the underloaded diagonal.

### 10.3 Dynamic Load Estimation (FFW)

Total mass estimated from motor currents (slow LPF):

$$M_{est} = \frac{\text{LPF}(\sum |I_i|) \cdot K_A \cdot GR}{g}$$

Per-leg gravity compensation:

$$\tau_{ff,i} = -\left(\frac{M_{est}}{4} + m_{leg}\right) \cdot g \cdot r \cdot \sin(\theta_i) + \tau_{warp,i}$$

### 10.4 Mode Entry Ramp

On entering COMFORT, the Kp/Kd values ramp from entry defaults ($K_p=35$, $K_d=1.5$) to the computed impedance values over ~0.4 s:

$$K_p(t) = K_{p,entry} + \alpha(t) \cdot (K_{p,impedance} - K_{p,entry})$$

Where $\alpha$ increments by 0.005 per cycle until reaching 1.0.

---

## 11. Ground Contact — Warp Compensation

### 11.1 Problem

A rigid chassis on 4 contact points has **one over-constrained DOF** (the "warp" or diagonal twist). Pitch and Roll PIDs control 2 of 3 tilting DOFs, leaving warp uncontrolled — one wheel can lift off.

### 11.2 Warp Decomposition

The 4 per-leg height offsets decompose into 4 orthogonal modes:

| Mode | Pattern (FL, FR, BL, BR) | Controller |
|------|--------------------------|------------|
| Heave | $(+1, +1, +1, +1)$ | `target_chassis_height_` |
| Pitch | $(+1, +1, -1, -1)$ | Pitch PID |
| Roll  | $(-1, +1, -1, +1)$ | Roll PID |
| **Warp** | $(+1, -1, -1, +1)$ | **GroundContact PI** |

### 11.3 PI Controller

$$\Delta H_{warp} = K_p \cdot e_{warp} + K_i \cdot \int e_{warp} \, dt$$

| Parameter | Value |
|-----------|-------|
| $K_p$ | 0.002 m/A |
| $K_i$ | 0.0005 m/(A·s) |
| Max $\Delta H$ | ±20 mm |
| Deadband | 0.15 A |

Output per leg:

$$\Delta H_{FL} = -\Delta H_{warp}, \quad \Delta H_{FR} = +\Delta H_{warp}$$
$$\Delta H_{BL} = +\Delta H_{warp}, \quad \Delta H_{BR} = -\Delta H_{warp}$$

**Anti-windup:** Integral clamped and decayed (×0.990 when in deadband, ×0.999 otherwise).

---

## 12. Climbing Dynamics — Step Climbing

### 12.1 Per-Leg State Machine

```
IDLE → PREP → DETECT → CLIMBING → COMPLETE → (back to IDLE)
```

| Phase | Description |
|-------|-------------|
| **IDLE** | Normal height pipeline |
| **PREP** | Move leg to preparatory angle ($180° - \theta_{prep}$, default 165°) to avoid singularity |
| **DETECT** | Hold at prep angle; monitor **leg torque residual** for step contact |
| **CLIMBING** | Execute **kinematic trajectory** — compensate leg angle as wheel rolls over step |
| **COMPLETE** | Hold at end-of-climb angle; await reset |

**Sequencing:** Front legs (FL, FR) climb first. Rear legs (BL, BR) start automatically when both front legs reach COMPLETE.

### 12.2 Step Detection — Torque Residual Method

The torque residual isolates step-contact forces:

$$\tau_{residual} = \tau_{feedback} - \tau_{gravity\_comp}$$

A slow LPF baseline tracks the DC component:

$$\tau_{baseline}(k) = \alpha \cdot \tau_{residual}(k) + (1-\alpha) \cdot \tau_{baseline}(k-1)$$

Step detection triggers when:

$$|\tau_{residual} - \tau_{baseline}| > \tau_{threshold}$$

held for $t_{confirm} = 20$ ms. Default threshold: **3.0 Nm**.

**Warmup:** First 0.1 s uses fast LPF ($\alpha=0.3$) to converge baseline, then switches to slow ($\alpha=0.02$).

### 12.3 Climbing Kinematic Model

A wheel (radius $R$) connected via an eccentric leg (length $L$) climbs a step of height $h$. The wheel rotates around the step edge point $E$.

**Constraint equation:**

$$\cos(\theta) = \frac{R + L - h - R \cdot \sin(\beta)}{L}$$

Where:
- $\theta$ = leg angle in kinematic model (small $\theta$ = extended)
- $\beta$ = wheel rotation angle around step edge ($\beta_0$ → $90°$)

**Angular velocity coupling:**

$$\dot{\theta} = \frac{R \cdot \cos(\beta) \cdot \dot{\beta}}{L \cdot \sin(\theta)}$$

**Boundary conditions:**
- Start: $\theta = \theta_{prep}$, $\beta = \beta_0 = \arcsin\left(\frac{R - h}{R}\right)$  (from constraint at prep angle)
- End: $\beta = 90°$, $\theta_{end} = \arccos\left(\frac{L - h}{L}\right)$

### 12.4 Trajectory Generation

The climbing angle $\beta$ is integrated at a **constant virtual angular velocity** $\omega_{climb}$ (default 1.0 rad/s), independent of actual wheel speed (which stalls against the step):

$$\beta(k+1) = \beta(k) + \omega_{climb} \cdot \Delta t$$

At each step:
1. Compute $\theta$ from constraint: $\cos(\theta) = \frac{R + L - h - R \sin(\beta)}{L}$
2. Motor angle: $\theta_{motor} = 180° - \theta$ (convention: 180° = highest)
3. Apply climb_sign: FL/FR use $-\theta_{motor}$, BL/BR use $+\theta_{motor}$

### 12.5 Climbing Wheel Forward Drive

During CLIMBING and COMPLETE phases, a forward RPM boost is added:

$$n_{climb} = \omega_{climb} \cdot \frac{60}{2\pi} \cdot k_{scale}$$

where $k_{scale}$ (default 5.0) is the wheel RPM multiplier. The boost is active only when the user pushes forward ($V_x > 0.5$).

### 12.6 Pitch Bias During Climbing

A pitch lean angle biases weight toward the climbing wheels:
- **Front legs climbing** → lean forward ($-3°$ pitch setpoint)
- **Rear legs climbing** → lean backward ($+3°$ pitch setpoint)

---

## 13. Motor Control Pipeline

### 13.1 Wheel Motor Pipeline (M3508)

```
Target RPM → + Decoupling Compensation → Deadzone Check → PID → Motor Output
```

**Wheel Velocity PID** parameters (per motor):
- $K_p = 200$, $K_i = 180$, $K_d = 1.0$
- Integral limit: 15000, Output limit: 16000

**Deadzone:** When target and feedback are both below 15 RPM, output is zeroed and PID is reset to prevent integral windup.

### 13.2 Leg Motor Pipeline (HT8115 — MIT Mode)

```
Target Height → Height-to-Angle → Slew Rate Limiter → MIT Command
```

**MIT Control Law (HT8115):**

$$\tau = K_p \cdot (\theta_{target} - \theta_{fb}) + K_d \cdot (\dot{\theta}_{target} - \dot{\theta}_{fb}) + I_{ff} \cdot K_A$$

Where:
- $K_p$ = position stiffness (Nm/rad, range 0–500)
- $K_d$ = velocity damping (Nms/rad, range 0–5)
- $I_{ff}$ = feed-forward current (A)
- $K_A$ = motor torque constant

**Default MIT parameters:** $K_p = 35$, $K_d = 1.5$ (stiff position hold).

### 13.3 Slew Rate Limiter

Prevents violent leg movements by clamping angle change per cycle:

$$|\Delta\theta_{cmd}| \leq \dot{\theta}_{max} \cdot \Delta t = 400°/\text{s} \times 2\text{ms} = 0.8°/\text{cycle}$$

Includes **angle unwrapping** to handle ±180° wrapping correctly.

---

## 14. Communication Protocol

### 14.1 PC → MCU (PC_Msg, 13 bytes)

| Field | Type | Description |
|-------|------|-------------|
| `left_joystick` | `{angle_x10, r_x1000}` | Left stick: forward/backward |
| `right_joystick` | `{angle_x10, r_x1000}` | Right stick: rotation |
| `Left_trigger_x1000` | `uint16` | Left trigger (0–1000) |
| `Right_trigger_x1000` | `uint16` | Right trigger (0–1000) |
| `button_status` | `uint8` | 8 buttons: LB, RB, X, A, B, Y, ML, MR |

### 14.2 MCU → PC (Reachable_Msg, 40 bytes)

- Motor positions (×10), target positions (×10), temperatures
- Motor RPMs, target RPMs, temperatures

### 14.3 Connection Detection

- Timeout-based: if no message received within threshold, robot enters IDLE.
- On reconnect, button state is forced to 0xFF to prevent false rising edges.

---

## 15. RTOS Task Structure

| Task | Stack | Priority | Rate | Function |
|------|-------|----------|------|----------|
| `Chassis_Task` | 2048 words | 0 | 500 Hz (2 ms delay) | Main control loop |
| `PC_CommTask` | — | — | 100 ms TX | UART communication |

### Startup Sequence

1. Wait 3000 ms for HT8115 motor boot
2. Call `chassis.Init()` — enables all motors, sends ENTER_MOTOR ×20 to HT motors
3. Enter main loop: read PC command → update chassis → transmit CAN → send feedback

---

## 16. Summary of Key Formulas

### Kinematics

| Formula | Equation | Used In |
|---------|----------|---------|
| Height from angle | $H = R + r\cos\theta$ | `Set_Leg_Height`, `CalculateHeightFromAngle` |
| Angle from height | $\theta = \arccos\left(\frac{H-R}{r}\right)$ | `Set_Leg_Height` |
| Velocity feedforward | $\dot\theta = \frac{-\dot H}{r(\sin\theta + \epsilon)}$ | `Set_Leg_Height` |
| Wheel compensation | $n_w = n_l \left(1 + \frac{r}{R}\cos\theta\right)$ | `Wheel_Compensation` |
| VMC (Jacobian) | $\tau = F_z \cdot r \cdot \cos\theta$ | `VMC_Calculation` |
| Gravity torque | $\tau_g = -m \cdot g \cdot r \cdot \cos\theta$ | `Get_LegGravityTorque` |

### Impedance Control

| Formula | Equation |
|---------|----------|
| MIT Kp mapping | $K_{p} = r^2 \sin^2\theta \cdot k_v$ |
| MIT Kd mapping | $K_{d} = r^2 \sin^2\theta \cdot c_v$ |
| FFW torque | $\tau_{ff} = -F_{load} \cdot r \cdot \sin\theta$ |
| Warp stiffness modulation | $k_{v,i} = k_{v,base}(1 - \gamma \cdot s_i \cdot \hat{e}_{warp})$ |
| Mass estimation | $M = \text{LPF}(\sum|I_i|) \cdot K_A \cdot GR / g$ |

### Climbing

| Formula | Equation |
|---------|----------|
| Constraint | $\cos\theta = \frac{R + L - h - R\sin\beta}{L}$ |
| Angle rate | $\dot\theta = \frac{R\cos\beta \cdot \dot\beta}{L\sin\theta}$ |
| End angle | $\theta_{end} = \arccos\left(\frac{L-h}{L}\right)$ |
| Start angle | $\beta_0 = \arcsin\left(\frac{R + L(1 - \cos\theta_{prep}) - h}{R}\right)$ |

### Inverse Kinematics (Skid-Steer)

| Wheel | Formula |
|-------|---------|
| FL | $V_x - \omega_z \cdot k_F$ |
| FR | $V_x + \omega_z \cdot k_F$ |
| BL | $V_x - \omega_z \cdot k_B$ |
| BR | $V_x + \omega_z \cdot k_B$ |

Where $k_F = W_F / W_{avg}$, $k_B = W_B / W_{avg}$.

---

## 17. File Reference

| File | Description |
|------|-------------|
| `Chassis.hpp / .cpp` | Main chassis state machine, mode handlers, body leveling, inverse kinematics |
| `Wheel_Leg.hpp / .cpp` | Per-leg motor control: height→angle conversion, MIT command, wheel PID, decoupling |
| `Impedance_Controller.hpp / .cpp` | Variable-impedance active suspension: Cartesian→MIT mapping, warp modulation |
| `Climbing_Dynamics.hpp / .cpp` | Per-leg climbing state machine, kinematic trajectory, torque-residual step detection |
| `Ground_Contact.hpp / .cpp` | Warp-mode PI compensator for 4-wheel ground contact |
| `Controller.hpp / .cpp` | Joystick-to-velocity mapping |
| `Robot_Params.hpp` | All mechanical/control constants |
| `Robot_Config.cpp` | Motor/PID/Wheel_Leg instantiation and wiring |
| `Chassis_Task.cpp` | FreeRTOS task: main 500 Hz control loop |
| `PC_Comm.hpp / .cpp` | UART communication with PC controller |
| `Comm_Msg.hpp` | Protocol message structures (PC_Msg, Reachable_Msg) |
| `Cust_Types.hpp` | Enum/struct definitions (Chassis_State, MIT_Params, Wheel_Leg_Params) |
| `Helper.hpp` | Utility functions (angle normalization, deg/rad conversion) |
