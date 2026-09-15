# Unified Leg-Control Refactor — Plan & Memory

> Wheelchair project (passenger on board). **Safety-critical:** the leg motors
> allow unlimited 360° rotation, so the controller must NEVER let a leg unwind
> multiple turns (passenger bobs up/down = unsafe). This whole refactor exists
> to make multi-turn unwind *structurally impossible*, not just unlikely.

---

## 0. CLIMBING pipeline (consolidated, current state)

The climb sequence is a per-leg state machine (`Climbing_Dynamics`) wrapped by
a chassis-level stage machine (`Chassis::handleClimbingMode`). Front legs
(FL=0, FR=1) climb first; back legs (BL=2, BR=3) follow. Each leg's
`LegClimbPhase`: IDLE→PREP→DETECT→CLIMBING→COMPLETE.

### Step 1 — enter at theta=0 (transition pose) [HOMING_IN / WAIT_START]
- On `Set_Mode(CLIMBING)`, `climb_stage_ = HOMING_IN`. All legs smoothstep to
  motor-frame 0° (ES-style: matched omega_ff + gravity FFW + Kp/Kd ramp +
  FF wheel comp, A4dd/A4ee). Wheels follow joystick.
- At 0° → `WAIT_START`: hold at 0°, wait for BTN_X. Approaching the PREP
  angle from 0° guarantees a single-direction sweep that never crosses the
  ±180° seam.
- BTN_X → `startClimbAll()` → all 4 legs enter PREP, `climb_stage_=ACTIVE`.

### Step 2 — PREP (synchronized lift + nose-up lean) [phase PREP]
- All 4 legs smoothstep `target_theta` 0 → (180-prep_theta_deg)=165° over
  `prep_ramp_s`=3 s, IN LOCKSTEP (shared ramp clock → no chassis wobble).
  Driven by flat PD (Set_Leg_PD_Torque, Kp=40/Kd=2, A4kk) + gravity FFW
  (adaptive mass, A4oo) + matched omega_ff.
- **Pitch lean-back +10° (nose UP):** `climb_pitch_front_deg=10` shifts CoM
  backward to unload the front wheels for easier step pivot. Applied via
  body PID → per-leg dtheta, LPF-smoothed (A4tt). **GEOMETRIC CAVEAT:**
  at 165° the leg has almost no pitch authority (dH/dθ = r·sin(165°) =
  0.32 mm/°); with the ±5° dtheta clamp the achievable chassis tilt is only
  ~0.3-0.6°, NOT 10°. The setpoint is +10° but the realized tilt is small.
  To get real lean, lower prep angle or raise the dtheta clamp (TODO/open).
- **PREP mass measurement (A4qq):** while sweeping through sin(θ)>0.7,
  sample tau_fb/(g·r·sinθ) per leg → measure on-board load (empty vs rider)
  → seed impedance, so FFW is accurate for the rest of the climb.
- PREP→DETECT when smoothstep done (alpha≥1) AND leg within prep_tolerance.

### Step 3 — front DETECT (FL/FR step-contact detection) [phase DETECT]
- FL/FR hold at prep angle (Kp=200 RIGID, A4ss) so step-contact force shows
  up as a clean torque spike instead of letting the leg yield.
- Residual = tau_fb − adaptive_gravity (A4oo). In steady state ≈ 0
  regardless of rider weight; spikes on real contact. Detection: deviation
  from LPF baseline > torque_res_threshold for detect_confirm_s, while leg
  velocity < detect_settle_omega.
- **ISSUE — turning false-trigger:** a yaw maneuver differentially loads the
  eccentric legs → torque residual mimics step contact. **Fix (A4ad):**
  `setDetectInhibit(true)` whenever the right-stick (rotation) is deflected
  past deadzone → all detection suppressed while turning. Detect only when
  driving (nearly) straight.
- Contact confirmed → FL/FR enter CLIMBING.

### Step 4a — front CLIMBING (kinematic step pivot) [phase CLIMBING]
- β integrates at `climb_omega`=0.8 rad/s (A4yy), ramped in via a 0.5 s
  smoothstep at entry (A4ww, kills the omega step-on jerk). θ from the
  constraint `cos(θ)=(R+L−h−R·sinβ)/L`; motor target = 180−θ. Flat PD
  Kp=80/Kd=4 follows.
- Wheel speed capped to chassis-matching rate × `climb_wheel_speed_ratio`
  (default 2.0, for joystick assist), hard-ceilinged at 25 RPM (A4xx/A4zz)
  → wheel never outruns the chassis kinematic forward velocity (no slip
  runaway) and never exceeds ~1/4 walking pace (passenger safety).
- Pitch held at +10° nose-up throughout front climb.

### Step 4b — front COMPLETE → nose-down lean → back DETECT
- FL/FR reach β=90° (or θ_end) → COMPLETE, hold at end-climb pose (front
  wheels now on top of step).
- **Re-tilt nose-DOWN −3° (lean forward, A4tt `climb_pitch_back_deg=-3`):**
  shifts CoM forward, reducing back-wheel pressure for easier back pivot.
  Optionally `climb_back_lift_offset_deg` (A4ab) directly biases the BL/BR
  hold angle. (Same geometric caveat applies near 165°.)
- **ISSUE — back legs near-stall during front trigger:** when the robot
  hits the step, it decelerates; that impulse plus the whole FL/FR CLIMBING
  trajectory motion loads the (essentially stalled) back legs, faking a
  step-contact residual on BL/BR. **Fix (A4ac + A4yy):**
    - BL/BR DETECT→CLIMBING gated until BOTH FL+FR are COMPLETE (not just
      climbing) — removes the entire FL/FR-climbing window of vulnerability.
    - Plus a `back_settle_s`=1 s delay after that, during which BL/BR
      baselines are snap-reset and re-converged via fast LPF.
    - Plus the A4ad turning inhibit.
- After settle, BL/BR detect their own step contact (user drives forward) →
  BL/BR CLIMBING → COMPLETE. All four COMPLETE → pitch returns to 0
  (level), climb done.

### Exit — HOMING_OUT
- Switching out of CLIMBING is intercepted: legs smoothstep back to 0°
  (ES-style, A4ee) before the actual mode change, so no leg is left near
  the seam.

### Open items / tuning knobs (Ozone `dbg_ctrl` unless noted)
- `climb_pitch_front_deg`=10, `climb_pitch_back_deg`=-3, `climb_pitch_lpf_alpha`
- `climb_omega`=0.8, `climb_ramp_s`=0.5 (config), `climb_wheel_speed_ratio`=2.0
- `torque_res_threshold`, `detect_confirm_s`(cfg), `detect_settle_omega`(cfg)
- `back_settle_s`(cfg)=1.0, `climb_back_lift_offset_deg`=0
- **Geometric pitch authority** at extended leg angles is the main
  limitation on Steps 2 & 4b: meaningful CoM shift needs either a lower
  prep angle (less reach) or a larger dtheta clamp (closer to seam). Decide
  on HW whether the small achievable tilt is sufficient.

---

## 1. Problem background

- Leg pose: `H = R - r·cos(θ)`. "Highest / climbing-prep" pose ≈ motor **±170°**,
  i.e. right at the encoder **±180° wrap seam**.
- The DM J10010L motor runs an **internal multi-turn absolute** position loop.
  When a position command crosses ±π, the DM may resolve it the long way and
  **accumulate a turn**; a later "return to 0°" then unwinds 1+ turns → bob.
- Old climbing code drove the leg with a **velocity command from shortest-path
  wrapped error** (`Pos_KP=0`, `Vel_KD` only). Near the seam the shortest-path
  error flips sign, so legs went the wrong way / inconsistent direction
  (observed: FL commanded +170 but settled −170; FR direction unstable).
- `HOMING_OUT` "ramp every leg back to 0° before any mode switch" was a
  **band-aid**, not acceptable: any accumulated turns unwind through it → bob.

### Per-leg facts observed (for reference)
- `bending_direction` (Robot_Config.cpp): FL=−1, FR=+1, BL=+1, BR=−1.
- `climb_sign` currently `{FL:+1, FR:−1, BL:+1, BR:+1}` (FL was flipped by user).
- A pure sign flip cannot create a *magnitude* offset → BR's −142° (vs ~166°)
  was NOT a sign issue; at that snapshot BR was almost certainly NOT in a
  climbing direct-control phase (back legs only start after both fronts
  COMPLETE), so it was in the normal `Set_Leg_Height` pipeline governed by
  `bending_direction`. Re-evaluate per-leg `climb_sign` empirically AFTER the
  direction mechanism is fixed (it now actually takes effect).

---

## 2. Target architecture — the real fix

**Maintain a continuous (unwrapped) leg angle in software and command the
nearest 360°-equivalent target; drive the leg with a single computed-torque law
sent as FFW only (`Pos_KP = Vel_KD = 0`) in ALL modes.**

Why this is safe & consistent:
1. We own the turn-resolution (software continuous angle), not the DM. With
   `Pos_KP=0` the DM never does position resolution → it can't unwind.
2. `H` is 360°-periodic, so `NearestEquivalentTarget()` always sends the leg the
   short way to an equal-height pose → no multi-turn motion ever.
3. Everything is one continuous torque law (no velocity↔position mode switch),
   so there is no switch discontinuity. PREP/hold → `ω_des=0`, the PD spring +
   gravity FFW holds; CLIMBING → `ω_des` = trajectory rate.
4. All feedback (pos + vel) is motor telemetry in consistent units. Only cost
   vs. the motor's internal loop is the 500 Hz software rate (acceptable; COMFORT
   already runs impedance at 500 Hz).

Control law (per leg, per tick) — **speed-limited software velocity loop**:
```
target_cont = NearestEquivalentTarget(angle_cmd_deg)        // wrap-free
err_rad     = deg2rad(target_cont - theta_continuous)
omega_cmd   = clamp(kp_pos*err_rad + omega_ff, ±omega_max)   // SPEED LIMIT here
omega_fb    = LPF(Get_LegVelocity())                         // rad/s, alpha=0.3
tau         = Get_LegGravityTorque()
            + clamp(kd_vel*(omega_cmd - omega_fb), ±tau_max)
Set_Leg_Target(Get_LegAngleWrapped(), 0, tau, Pos_KP=0, Vel_KD=0)
```
The `omega_max` clamp is essential: a plain position-PD would slam the leg on a
large step error (PREP 0→±170) — unsafe for the passenger AND the resulting
drive torque would swamp step detection (the torque shows up in the
`residual = torque - gravity` signal). Bounding speed keeps motion gentle and
the residual clean.
- Torque sign is correct: HOMING with `Pos_KP>0` already holds correctly, so the
  DM's "+torque increases position" convention matches `+kp_tau*err` (negative
  feedback). FFW is added in the same frame.
- Position arg = current wrapped angle so `Execute_Leg_Control`'s slew-limiter
  state stays synced (ignored by the motor since `Pos_KP=0`).

### Mode transition safety
Once ALL modes use this law + continuous frame, there is no ±180 seam crossing
and no accumulated turns, so **`HOMING_OUT`/return-to-0 band-aid can be removed**
(A5). Until then it stays.

---

## 3. Phased plan

### Track A — control unification (safety-critical; do first, validate each)

- **A1 — DONE.** Non-breaking continuous-position infra in `Wheel_Leg`:
  - Renamed `Get_LegPosition()` → **`Get_LegAngleWrapped()`** (wrapped (−180,180]).
    All ~26 call sites updated (Chassis.cpp, Wheel_Leg.cpp/.hpp).
  - Added **`Get_LegAngleUnwrapped()`** (continuous, never wraps).
  - Added accumulator in `Execute_Leg_Control()` top: `leg_pos_continuous_ +=
    wrap180(fb - prev_fb)`; self-seeds; pure state, no behavior change. Members:
    `leg_pos_continuous_`, `prev_leg_pos_fb_`, `leg_cont_initialized_`.
  - Added **`NearestEquivalentTarget(target_deg)`** → continuous-frame target
    within ±180° of current continuous pos.
  - Naming rule: **`Wrapped`** = single-turn / jumps; **`Unwrapped`** = continuous / safe.

- **A2 — DONE.** Added `Wheel_Leg::Set_Leg_Torque_Track(target_deg, omega_ff,
  kp_pos, omega_max, kd_vel, tau_max)` implementing the speed-limited velocity
  loop in §2. Added member `leg_vel_filt_` (vel LPF, alpha=0.3). Non-breaking.

- **A3 — DONE in code, NEEDS HARDWARE VALIDATION.** Migrated CLIMBING:
  - `Climbing_Dynamics::Config`: removed `climb_pos_kp_vel`, `climb_omega_max`;
    added `climb_pos_kp=8 (1/s)`, `climb_omega_max=3 (rad/s, speed clamp)`,
    `climb_kd_vel=2 (Nm·s/rad)`, `climb_tau_max=20 (Nm)`.
  - Rewrote the climbing direct-control branch in `Chassis::handleClimbingMode`
    to call `Set_Leg_Torque_Track(...)`. Pitch-leveling offset (`dtheta_deg`)
    preserved. Removed the old shortest-path velocity code.
  - **Step-detection fix (the "leg too fast → false phase switch" bug):** the
    closed-loop drive torque during motion contaminates the
    `residual = torque - gravity` signal and false-triggered DETECT→CLIMBING.
    Two-part fix: (a) `climb_omega_max` bounds leg speed so motion torque stays
    small; (b) detection is **velocity-gated** — added `leg_vel_radps` to
    `LegClimbFeedback` (filled from `Get_LegVelocity()` in Chassis) and
    `Config::detect_settle_omega=0.4 rad/s`; DETECT only accepts a torque
    deviation while `|leg vel| < detect_settle_omega`.
  - **Validate:** low gains first. FL now 0→+170 (correct direction), FR stable,
    firm hold, no oscillation, no false phase switch during PREP motion, no
    unwind on PREP/CLIMBING/COMPLETE. Tune `climb_pos_kp / climb_omega_max /
    climb_kd_vel` via config. Re-check per-leg `climb_sign` (direction now
    actually takes effect).
  - **KNOWN (expected):** CLIMBING→other-mode transition still buggy — that is
    the not-yet-unified modes (fixed in A4), not a regression here.
  - **FL single-leg direction bug — diagnosed & fixed.** Diagnosed via the
    `DbgClimbTarget dbg_climb_tgt` instrumentation (see `Chassis.hpp` —
    per-leg arrays of `theta_unsigned`, `angle_cmd`, `angle_fb`, `pos_cont`).
    Data showed `angle_cmd[FL]=+180` and `angle_cmd[FR]=−180` — both saturated
    against the `[0,180]` clamp because pitch-leveling added +15° on top of
    `theta_unsigned=165`. **±180 is the wrap seam:** at exactly that target,
    `NearestEquivalentTarget` resolves differently depending on tiny HOMING
    residual in `pos_cont`, so legs whose residual is slightly positive vs.
    slightly negative end up driving toward OPPOSITE equivalents (`+180` vs
    `−180`). FR happened to resolve consistently (+190 cont); FL flipped to
    the negative side. **Fix:** in `Chassis::handleClimbingMode`, clamp
    `new_theta` to `[5, 175]` (5° seam margin) instead of `[0, 180]`, so
    `angle_cmd` is always at least 5° away from ±180. This keeps the wrap
    math's `diff` strictly inside (−180, 180) regardless of HOMING residual.
  - **Per-leg diagnostic struct (kept; useful for A4 too):** `DbgClimbTarget`
    in `Chassis.hpp`, externed as `dbg_climb_tgt`, filled in the climbing
    branch after each leg's `Set_Leg_Torque_Track`. Fields per leg (index
    FL=0,FR=1,BL=2,BR=3): `theta_unsigned[4]`, `angle_cmd[4]`, `angle_fb[4]`,
    `pos_cont[4]`. Compare across legs in Ozone — should match each other up
    to the climb_sign pattern and small leveling offsets.

- **A3c — IN PROGRESS (FL-slow vs FR).** After the seam fix, FL goes the
  correct direction (+170) but **moves visibly slower than FR** even though
  travel distance is identical. Software pipeline inspection found no per-leg
  asymmetry between FL and FR (climb_sign just flips sign, lev_signs pitch
  is `+1/+1` for FL/FR so leveling is identical magnitude, gravity comp uses
  even `cos(θ)`). To localize whether the asymmetry is software or motor/mech,
  added **TEMPORARY** per-leg diagnostics to `dbg_climb_tgt`:
    - `omega_cmd[4]` — clamped velocity setpoint (rad/s)
    - `omega_fb[4]` — filtered velocity feedback (rad/s, alpha=0.3)
    - `tau_total[4]` — final torque sent to motor (Nm, gravity + PD)

  Sourced from new TEMPORARY snapshot fields in `Wheel_Leg` (`tt_omega_cmd_`,
  `tt_tau_total_`) + reused `leg_vel_filt_`, exposed via `Get_TT_*` accessors.
  All marked TEMPORARY — remove in A5 once FL-slow is root-caused.

  Decision rule when reading Ozone during PREP:
    - If `tau_total[FL] ≈ tau_total[FR]` but `omega_fb[FL]` tracks `omega_cmd`
      worse than FR → hardware/mech (friction, cable tension, motor params).
    - If `tau_total[FL] < tau_total[FR]` (magnitude) → software bug upstream
      of the motor; trace from tau back through the pipeline.

- **A3d — Seam crossing from position-loop overshoot.** After A3c diagnostics
  hardware test, observed that FL traveled `0 -> 165 -> -165` — i.e. the leg
  correctly went the SHORT way to +165 but then OVERSHOT, crossed +180 into
  the wrap seam, and settled at -165 (= +195 cont). FR was clean
  `0 -> -165`. Root cause is the cascaded PD's underdamped response: with
  `kp_pos=8, kd_vel=2`, damping ratio zeta ~0.25 -> ~45% overshoot beyond the
  commanded target. The previous 5deg seam margin (clamp to +/-175) left no
  room for overshoot; commanded target +175 + ~20deg overshoot = +195 crosses
  +180. **Fix (two-part):**
  1. Tighter clamp: `new_theta` clamped to `[prep_theta_deg, 180-prep_theta_deg]`
     = `[15, 165]` by default. This is the user's stated "ideal 0->165"
     range, leaves 15deg of seam headroom for overshoot.
  2. Bump `climb_kd_vel` from 2.0 to 6.0 (Nm.s/rad). Drops overshoot from
     ~45% to ~20%, which combined with the 15deg headroom safely stays inside
     +/-180 during the transient. Higher kd_vel adds noise sensitivity but at
     6.0 the velocity-feedback LPF (alpha=0.3) handles it.

- **A3e — Synchronized PREP lift ramp (chassis wobble fix).** After A3d closed
  the seam-crossing issue, the four wheel-legs lifted to PREP at visibly
  DIFFERENT speeds (per-leg friction/load differences let the faster legs win
  the race to `omega_max`), tilting the chassis and wobbling the passenger.
  Identical software cannot equalise real-world physical differences, so the
  fix is to **drive all legs from a shared time-based master trajectory**
  instead of letting each race independently:
  - `Climbing_Dynamics::Config::prep_ramp_s = 1.5f` — duration of the
    synchronized lift.
  - `LegState::prep_ramp_t` — per-leg ramp clock, reset to 0 in `startClimb`.
  - PREP case now smoothsteps `target_theta_deg` from 0 to
    `180 - prep_theta_deg` over `prep_ramp_s`, with `target_omega` set to the
    derivative of the smoothstep so the velocity-loop FFW keeps the PD
    tracking the ramp tightly. PREP -> DETECT transition gated on
    `alpha >= 1.0 AND |current - prep| < prep_tolerance_deg`.
  - All four legs share `dt` and start `prep_ramp_t=0` in the same
    `startClimbAll` call -> lockstep lift. Slow legs no longer lag, fast legs
    no longer pull ahead. Max ramp velocity ~2.88 rad/s (just under
    `omega_max=3`), within all legs' capability.

- **A3f — All four legs in synchronized PREP.** After A3e, FL/FR lifted in
  lockstep but BL/BR went up MUCH faster -- because the climbing FSM only put
  FL/FR in PREP (`startClimbAll` started indices 0,1 only); BL/BR were IDLE in
  the climbing FSM, so they fell back to the normal `Set_Leg_Height` pipeline
  driven by `target_chassis_height_` and slewed at `HEIGHT_SLEW_PER_CYCLE
  = 0.2 m/s`. That rate finishes the lift in ~0.65 s while FL/FR's `prep_ramp_s
  = 1.5 s` ramp takes more than twice as long -> visible asymmetry, chassis
  wobble. **Fix:**
  - `startClimbAll()` now starts ALL FOUR legs in PREP, so they all share the
    same `prep_ramp_t` clock and the same smoothstep -> the chassis rises
    perfectly level.
  - Removed the bottom-of-`update()` "front-first auto-start" gate that used
    to fire `startClimbDirect` on BL/BR once both FL/FR reached COMPLETE -- no
    longer needed because BL/BR are already in the climbing FSM. They now
    just sit in DETECT after their PREP finishes, waiting for their own wheel
    to hit the step. **Front-first climbing sequence is still preserved** --
    enforced naturally by physics (back wheels are not on the step yet, so
    their DETECT torque-spike condition only fires later, after the chassis
    has driven forward).
  - `startClimbDirect` is now unused; will be deleted in A5 cleanup.

- **A3g — BR climb_sign corrected.** With all 4 legs now in PREP via the
  shared ramp (A3f), BR was observed driving to +120 instead of -120. Final
  `climb_sign` confirmed empirically: **`{FL:+1, FR:-1, BL:+1, BR:-1}`** --
  left-right mirror pattern. The earlier `{+1, -1, +1, +1}` was leftover from
  before BR ever participated in the climbing FSM (BR was on the normal
  pipeline back then so its climb_sign was effectively unused, and the value
  defaulted incorrectly).

- **A4d — ES->COMFORT transition: sudden-impact + empty-chassis roll
  divergence.** Two pre-existing bugs that A4a/b made more visible:
  1. **Sudden impact at the transition tick.** `Set_Mode` was assuming the
     leaving-ES Kp/Kd was 80/4 (the OLD ES motor-Kp position-hold value), but
     after A4a ES uses torque-track with no motor Kp; the effective stiffness
     is the cascade product `climb_pos_kp * climb_kd_vel = 8 * 6 = 48 Nm/rad`
     and damping `climb_kd_vel = 6 Nm.s/rad`. With the stale 80/4 starting
     point the leg saw a stiff jolt at the first COMFORT HOMING tick.
     **Fix:** `prev_kp = 48`, `prev_kd = 6` for the ES->COMFORT path -- HOMING
     now gently RAMPS DOWN to `COMFORT_HOMING_KP_END=30, KD_END=2` instead of
     down from 80.
  2. **Empty-chassis roll oscillation diverges.**
     `COMFORT_HOMING_CHASSIS_MASS_GUESS` was set to
     `(ROBOT_MASS - 4*LEG_MASS) + RIDER_MASS_kg` (~70-80 kg). When the robot
     is UNLOADED, FFW = `M_est * g * r * sin(theta)` is ~7x too large at the
     HOMING->RUN handoff -- legs over-actuate, body rises, IMU sees roll, the
     roll PID's correction excites the over-strong FFW further -> positive
     feedback before the `M_est` LPF (tau~1 s) can converge to the true mass.
     **Fix:** `COMFORT_HOMING_CHASSIS_MASS_GUESS = ROBOT_MASS_kg - 4*LEG_MASS_kg`
     (chassis only, no rider). Trade-off: with a rider on, there is a brief
     ~6-7 deg sag at HOMING that the LPF pulls up within ~1 s. This is
     **strictly safer** than divergence: brief droop is recoverable, an
     oscillating wheelchair is not. Future improvement candidates: detect
     load state via accel-z or button, OR temporarily boost LPF alpha
     during the first second of RUN.

- **A4e — M_est seed REVERTED back to LOADED.** A4d's reduction to unloaded
  chassis broke the loaded COMFORT case: legs lacked FFW to lift the rider,
  observed as "cannot reach target height, lacks force, wobbles". Reverted to
  `(ROBOT_MASS - 4*LEG_MASS) + RIDER_MASS_kg`. **Trade-off accepted:** empty-
  chassis ES->COMFORT may oscillate during the M_est LPF settle (~1 s); to be
  fixed via a different mechanism (faster early-RUN LPF, or load detection)
  in A4g. Under-seeding is unsafe (can't lift); over-seeding is uncomfortable
  but recoverable.

- **A4f — Climbing body-mass FFW added.** PREP pitch wobble (front/back legs
  rising at visibly DIFFERENT rates after A3e/f synced their timing) was
  caused by `Set_Leg_Torque_Track` adding only `Get_LegGravityTorque()`,
  which counts the 0.5 kg leg only and ignores body weight. Under load,
  front/back asymmetric load -> per-pair PD lag differs -> pitch oscillates.
  **Fix:** `Set_Leg_Torque_Track` signature extended with a `ffw_torque`
  parameter (replaces the internal `Get_LegGravityTorque()`). Climbing now
  computes
  `ffw = (M_chassis/4 + LEG_MASS_kg) * g * r * sin(angle_cmd)`
  using the same `COMFORT_HOMING_CHASSIS_MASS_GUESS` (loaded), so the legs
  get accurate sin-based gravity compensation throughout PREP/DETECT/CLIMB.
  ES passes `ffw = 0` (target at theta=0, sin=0 -> no FFW needed anyway).

- **A4h — Raised software tau_max to match DM hardware ceiling.** Loaded
  COMFORT could not lift / hold the rider after A4b. Root cause: the OLD
  motor-internal MIT loop was only bounded by the DM's hardware torque
  ceiling (200 Nm per host config); the first software port artificially
  clamped at 30 Nm, and the loaded case routinely needed 30-60 Nm
  (FFW ~13 Nm + impedance Kp~60 * err of a few deg = easily 30+). PD
  saturated -> legs sagged -> the same loop tried to track rider sway but
  could not -> "lacks force, wobbles" symptom. **Fix:**
  - 5-arg `Set_Leg_Height` (COMFORT RUN): `COMFORT_TAU_MAX 30 -> 200`.
  - COMFORT HOMING (`handleComfortMode`): `HOMING_TAU_MAX 30 -> 200`.
  - 200 Nm matches the DM hardware ceiling configured by the host -- the
    software now never restricts what the OLD MIT loop could deliver.

  **Design note on speed limiting under high tau_max** (per user direction):
  the PD law has speed control built in via the `Kd*(omega_ff - omega_fb)`
  term -- as the leg accelerates under `Kp*err`, `Kd*omega_fb` grows and
  opposes motion. This is the natural mechanism and we should NOT layer an
  artificial position-error clamp on top (an earlier draft did this; the user
  correctly noted it duplicates what Kd already provides). If a particular
  mode feels too "snappy" under high tau, the lever is `cv_base` /
  `mit_kd_max` in `Impedance_Controller::Config`, not a software-side speed
  clamp. The cascaded `Set_Leg_Torque_Track` (CLIMBING) is a different case:
  its omega_max bounds the SHARED trajectory speed across legs for sync,
  not a damping concern.

- **A4i — ES<->COMFORT transition tuning.** After A4h's tau_max raise, three
  remaining issues:
  1. **ES->COMFORT still has slight 顿挫** at the transition. Cause: the old
     `COMFORT_HOMING_FRAMES = 250` (0.5 s lift) implies a peak smoothstep
     velocity of ~4.7 rad/s -- brisk for a rider-on entry. **Fix:** raised to
     `500` (1.0 s lift, peak ~2.4 rad/s) -- gentle, still snappy enough.
  2. **COMFORT->ES descent too fast** -- ES was using climbing's
     `climb_omega_max = 3 rad/s`, so the ~90 deg drop took ~0.5 s. **Fix:**
     ES now uses its own local `ES_OMEGA_MAX = 1.5 rad/s` -> ~1 s descent,
     smoother for the passenger.
  3. **Empty-chassis roll oscillation** -- user-requested PID modification.
     `BODY_ROLL_PID_INT_LIMIT` reduced `1000 -> 100`. The old huge limit let
     `Ki` accumulate over the ~1 s M_est LPF settle window during empty
     ES->COMFORT, pushing roll command into OUT_LIMIT saturation and
     diverging. The 100 cap bounds the integral contribution to ~0.02 m of
     h_adj (Ki * INT_LIMIT), still enough to nudge out steady-state offset
     but no longer winds up under transient mass misestimation. **NOTE:** the
     real root cause (M_est slow to converge for empty) is still A4g
     territory; this is a palliative that bounds the WORST-CASE windup, not
     a complete fix.

- **A4j — Targeted fixes for the three remaining COMFORT complaints.**
  1. **HOMING -> RUN "卡顿" (velocity-loop -> impedance jolt).** User identified
     the transition point. The Kp ramp from `entry_kp` to full impedance Kp
     was running at `ramp_rate = 0.005 / cycle` -> 0.4 s. Slowed to
     `0.002 / cycle` -> 1.0 s. The leg now eases into impedance-style
     compliance instead of snapping.
  2. **COMFORT -> ES descent still too fast.** ES_OMEGA_MAX 1.5 -> 1.0 rad/s
     -> the ~90 deg drop takes ~1.6 s, matching the COMFORT HOMING lift
     duration. Mirror symmetry between entry and exit.
  3. **Empty-chassis roll oscillation -- ROOT CAUSE FIX (A4g closed).** Added
     a fast-LPF window to `Impedance_Controller`:
     - New config: `ffw_lpf_alpha_fast = 0.1f` (~30 ms tau) and
       `fast_lpf_duration_s = 1.5f`.
     - New state: `fast_lpf_remaining_s_`, reset on `reset()` to the
       configured duration, decremented by `dt` each update tick.
     - Mass-update LPF picks `ffw_lpf_alpha_fast` while the timer is > 0,
       else the normal slow `ffw_lpf_alpha = 0.01`.
     - With the loaded seed (~70-80 kg) and empty actual (~10 kg), M_est
       converges in ~100-300 ms instead of ~1 s. Mass_scale then collapses
       the body-PID gains before oscillation can build. The slow steady-
       state alpha is preserved for noise filtering once converged.

  Combined with the previous A4i changes (slower HOMING smoothstep, smaller
  roll PID INT_LIMIT), the ES<->COMFORT transition should now be smooth in
  both directions and stable on both empty and loaded chassis. If empty
  oscillation still appears, next levers are `BODY_PID_SCALE_ROLL` (1.6 ->
  1.2) or further shortening `fast_lpf_duration_s`.

- **A4k — COMFORT->ES feel + impedance damping bump.**
  1. **"失重感" at COMFORT->ES descent + asymmetric front-vs-back fall when
     rider leans forward.** Same root cause: ES handler was passing `ffw=0`,
     so once the leg started descending the only restoring torque was the
     velocity damping `kd*(omega_cmd - omega_fb)` -- gravity ripped the leg
     down faster than omega_max would suggest, especially on the heavier
     (rider-forward) front legs. **Fix:** ES descent now sets
     `ffw = (M_est/4 + LEG_MASS) * g * r * sin(theta_now)` per leg, using
     the live impedance M_est. With gravity fully cancelled, descent rate is
     now set by `ES_OMEGA_MAX` alone (1 rad/s) and is uniform across legs
     regardless of weight distribution.
  2. **A/X (mid-height) roll wobble vs Y (top) robust.** At mid heights the
     impedance Kp is at its peak (sin² maxes around theta=90), making the
     leg/body coupling stiffest -- exactly where the body PID was most able
     to excite ringing. Past mode could not damp enough because the OLD
     motor-MIT Kd was hardware-capped at 5. With the unified software path
     there is NO hardware Kd ceiling -- `kd*v` is just a software multiplier.
     **Fix:**
     - `cv_base 1500 -> 3000` (doubled virtual damping coefficient).
     - `mit_kd_max 5 -> 10` (software cap; documented that old "MIT
       hardware" reason no longer applies).
     - Peak Kd at theta=90 now 10 Nm.s/rad (was 5). zeta ~2.2 loaded, ~7
       unloaded -- heavily overdamped, kills the wobble per user's
       "可以继续增大阻尼抑制" request.

- **A4l — Per-leg torque snapshot for symmetric ES descent.** With A4k's
  M_est-based uniform FFW, EVEN load (rider centred) descends well, but
  off-centre rider revealed the next layer:
  - uniform FFW = M_est/4 per leg
  - rider on back -> back legs need MORE support than M_est/4
  - rider on front -> front legs need MORE
  In whichever direction is "lighter than M_est/4", the FFW OVER-supports,
  the leg can't descend, and the asymmetric drop returns (just opposite of
  the original ffw=0 case).
  **Fix:** snapshot each leg's actual motor torque at the COMFORT->ES
  transition, scale by `sin(theta_now)/sin(theta_entry)` during descent. The
  snapshot reflects the impedance-shaped per-leg torque distribution that
  COMFORT had been running, which DOES capture the rider weight asymmetry
  (impedance pushed back-side legs harder when rider sat back). Scaling
  by sin tracks the changing pose as the leg descends. Gravity now cancels
  exactly per leg -> descent velocity is set by `Kd*(omega_cmd - omega_fb)`
  alone -> ALL four legs descend in lockstep regardless of weight
  distribution.
  - Members: `es_entry_tau_[4]`, `es_entry_sin_[4]`, `es_snapshot_valid_`.
  - Set in `Set_Mode` when entering ES from COMFORT (other source states
    have no impedance history -> fall back to A4k's uniform M_est/4 ffw).
  - Used in `handleEnergySaving` ffw computation. Snapshot sin clamped to
    ±0.3 to guard against the theta~0/pi singularity (won't fire in
    practice since COMFORT pose is ~theta=90).

- **A4m — ES architectural rewrite: "COMFORT pinned at lowest".** Per user
  direction "用严格速度环以及ffw进行控制, 而不是用阻尼下降". The previous
  `Set_Leg_Torque_Track` + `omega_max` clamp + ffw approach was damping-based:
  the velocity loop only BRAKED against gravity rather than tracking a real
  velocity reference. Uniform M_est/4 ffw under-supported heavy-loaded legs
  and over-supported light-loaded legs -> asymmetric descent.
  **Fix:** ES now reuses the COMFORT control law entirely:
  ```
  target_height_setpoint_ = h_min_   // pin the height target at the lowest pose
  impedance_.update(...)             // keep per-leg Kp/Kd/FFW live
  executeBodyControlImpedance(cmd)   // same path COMFORT uses on button height switches
  ```
  COMFORT's button-driven height changes were already "丝滑" (per user) for
  the same reason: `slewTargetHeight` produces a jerk-limited height
  profile + a real `height_slew_rate_` velocity FFW, fed to each leg via
  `Set_Leg_Height(h, v, kp_imp, kd_imp, ffw_imp)`. Inside Set_Leg_Height
  this becomes `Set_Leg_PD_Torque` -> `tau = Kp*err + Kd*(v_ff - v_fb) + ffw`.
  With impedance's per-leg ffw and the v_ff term, all four legs track the
  slewed target uniformly regardless of rider weight distribution. Descent
  velocity is the slew rate, not "whatever gravity wins against Kd*omega".
  - Stale members `es_entry_tau_[4]`, `es_entry_sin_[4]`, `es_snapshot_valid_`
    are now unused; A5 will delete them.
  - Descent speed governed by `slewTargetHeight`'s `rate_down =
    HEIGHT_SLEW_PER_CYCLE * 0.5`. From mid COMFORT height to h_min (~0.064 m
    range) takes ~0.65 s + ~0.15 s LPF tail ~0.8 s. If user wants exact
    symmetry with the 1 s lift, tune `rate_down` factor here later.

- **A4n — KNOWN OPEN: climbing PREP per-leg sync under rider.** After A3f,
  prep_ramp_t is shared across legs so the COMMANDED target is identical, but
  per-leg ACTUAL tracking lags differ under rider load asymmetry (heavier
  leg's PD can't keep up with the ramp; lighter leg may even overshoot).
  Same root cause as the ES asymmetric-descent that A4m just solved: uniform
  M_est/4 ffw can't compensate per-leg load distribution. The ES fix
  (impedance-shaped per-leg ffw via Kp*err closed loop in COMFORT) is not
  available in climbing because we usually enter CLIMBING from IDLE/ES, not
  COMFORT -- no Kp*err history to inherit. Two paths to consider:
  - **Slow ramp / boost damping:** `prep_ramp_s 1.5 -> 3.0 s`, +
    `climb_kd_vel 6 -> 10`. Doesn't fix asymmetry mathematically but gives
    each leg more time to converge -> visible synchronization tightens.
  - **Pre-PREP impedance settle:** insert a brief (~0.5 s) impedance phase
    BEFORE the prep ramp where each leg holds at its current angle with
    impedance Kp*err loop closed. This lets impedance shape per-leg torques
    to match actual load; snapshot those torques and use as the per-leg ffw
    basis throughout PREP. Same idea as A4l's ES snapshot but earned within
    climbing.

- **A4m REVERTED, A4o — Simple smoothstep-angle ES descent.** A4m's
  "COMFORT pinned at lowest" approach JUMPED on entry: feeding the
  impedance/leveling machinery target_height_setpoint_=h_min when the
  previous state's target_chassis_height_ was different (and leg angle
  unrelated) produced a step in commanded leg target -> Kp*err burst ->
  visible jump. **Per user direction "简单的速度和位置闭环就好了, 主打稳
  定 smooth":** scrapped the impedance reuse, dropped the snapshot scheme,
  and implemented the simplest correct thing -- a smoothstep angle ramp.
  ```
  on entry:  energy_homing_start_angle_[i] = current angle      // captured in Set_Mode
             energy_homing_ticks_ = 0
  each tick: alpha = ticks / FRAMES                              // 0 -> 1 over 1 s
             s     = alpha*alpha*(3-2*alpha)                     // smoothstep
             target_now = start * (1 - s)                        // from start -> 0
             omega_ff   = -start * d(s)/dt                        // matched velocity FFW
             ffw_grav   = (M_est/4 + LEG_MASS) * g * r * sin(now) // gravity comp
             Set_Leg_PD_Torque(target_now, omega_ff,
                               ES_KP=50, ES_KD=8, ffw_grav, 200)
  ```
  Why no jump: target STARTS at the leg's actual entry angle, so err=0 at
  tick 0 -> no Kp*err burst. The smoothstep ramps target smoothly to 0 with
  zero velocity at both endpoints (matched by omega_ff also zero at
  endpoints), so the start and end of descent are jerk-free. Gravity FFW
  cancels per-leg gravity using the M_est carried over from COMFORT.
  - `ENERGY_HOMING_FRAMES 400 -> 500` (1 s) for symmetry with the
    COMFORT_HOMING_FRAMES=500 lift.
  - Stale snapshot capture removed from Set_Mode. The members
    `es_entry_tau_[4]/es_entry_sin_[4]/es_snapshot_valid_` are now unused
    (will delete in A5).

- **A4p — COMFORT HOLD phase + symmetric slew rate.**
  1. **"阶段性" between lift and impedance.** Even with smooth Kp/Kd
     continuity at HOMING->RUN handoff, impedance + body-PID engagement was
     perceptible immediately after the lift. User wants the lift to FULLY
     SETTLE before impedance kicks in. **Fix:** added a fixed-PD HOLD phase
     after the smoothstep: `COMFORT_HOLD_FRAMES = 250` (0.5 s). During the
     hold the smoothstep alpha is already clamped at 1 (target stays at
     `COMFORT_HOMING_THETA`), Kp/Kd stay at `KP_END/KD_END`, ffw stays on
     the open-loop sin-based form. Transition to RUN only when
     `ticks >= FRAMES + HOLD_FRAMES`. Then impedance ramp_alpha (still
     ramp_rate = 0.002, ~1 s) smoothly takes over with the leg fully
     stationary at target. Total time from button press to full impedance:
     1 s lift + 0.5 s hold + 1 s impedance ramp = ~2.5 s. Stable, no
     perceptible "stage" change.
  2. **COMFORT button-height up too fast (motor strain).** `rate_up` was at
     full `HEIGHT_SLEW_PER_CYCLE = 0.0004 m/cycle` -> 0.2 m/s while
     `rate_down` was 0.5x. The asymmetric full-rate up under load stressed
     the motors. **Fix:** `rate_up = HEIGHT_SLEW_PER_CYCLE * 0.5f` (matches
     `rate_down`). Both directions now 0.1 m/s. In-COMFORT height changes
     take ~1.3 s for full 0.13 m range. Quieter, kinder to the motors.

- **A4q — Body PID active during COMFORT HOMING.** Even after A4p's HOLD
  phase eliminated the impedance-engagement edge, an off-centre rider still
  felt a "再抬一下" at HOMING -> RUN. Root cause: HOMING commanded all four
  legs to the SAME angle (uniform smoothstep). With an off-centre load the
  per-leg PD's Kp*err couldn't perfectly equalise heights -> chassis arrived
  TILTED at end of HOMING. RUN then started body PID from rest, which
  proceeded to LEVEL the tilted chassis -> visible secondary lift on the
  heavy side. **Fix:** run roll/pitch body PID DURING HOMING (and HOLD),
  scaled by the smoothstep progress `s`:
  ```
  roll_h_adj_hom  = roll_pid(0, roll_lpf)  * BODY_PID_SCALE_ROLL  * s
  pitch_h_adj_hom = pitch_pid(0, pitch_lpf) * BODY_PID_SCALE_PITCH * s
  per leg: cmd_deg += dtheta_from_per_leg_h_adj   // via dH = r*sin*dtheta
  ```
  At s=0 (lift start) the PID contribution is 0 -> no jump on entry. As s
  grows the PID effect ramps in, levelling the chassis CONTINUOUSLY through
  the lift. By s=1 (lift complete + HOLD) the body is already level and
  RUN's impedance handoff has nothing to correct. The lev_signs and
  BODY_PID_SCALE_* are the same as RUN, so a tilt correction during HOMING
  is mathematically identical to one during RUN. Independent LPF state
  (`roll_lpf_homing`, `pitch_lpf_homing`) seeded from `chassis_roll_/pitch_`
  on tick 0 so the filter doesn't start at 0 and create a synthetic error.

- **A4r ATTEMPTED, REVERTED.** Tried to fix COMFORT-entry "顶屁股" by
  measuring chassis+rider mass at HOMING end (via per-leg motor torque) and
  seeding M_est with it. **Why it fails:** the measurement is circular under
  the unified torque path. Motor torque = software-computed `tau =
  Kp*err + Kd*derr + FFW`. At HOLD steady state, err≈0 and derr≈0, so
  tau ≈ FFW = M_guess-based. Reading tau back via
  `tau_i / (g*r*sin_i)` recovers M_guess, NOT the true load. No new info.
  Need a non-circular signal (test pulse during HOMING, accel-based
  weighing, etc.) for a real fix. Deferred to a future pass.

- **A4s ATTEMPTED, REVERTED.** Smoothstep ramp on climbing pitch_setpoint
  (0 -> ±`climb_pitch_bias` over 1.5 s) caused hardware to "突然单边挑起"
  (sudden one-side lift) -- very dangerous. Two suspected root causes
  (still to investigate before a redesign):
  (a) **Sin singularity at PREP start.** After A3f, `theta_unsigned`
      smoothsteps from 0 to ~165 over `prep_ramp_s`. The climbing pitch-
      leveling formula `dtheta = -dh/(r*sin(theta_unsigned))` divides by
      sin which is clamped at 0.15 near theta=0. Any non-zero
      `pitch_h_adj` early in PREP gets amplified to a dtheta ≥ 50° (clip
      cap) -> huge instant leg motion. The smoothstep ramp on pitch_setpoint
      doesn't help if the resulting pitch_h_adj after PID still produces
      a large dh / small sin division.
  (b) **Front -> back pitch_setpoint switch.** When the last front leg
      reaches COMPLETE, `front_active` becomes false and `back_active`
      stays true -> setpoint flips from -bias to +bias (a 6° step at
      bias=3°). The ramp only affected entry; the front->back switch
      stays abrupt. Pre-existing issue, but the ramp made it more
      visible. Need either a separate front->back transition ramp or a
      single unified setpoint trajectory.
  Deferred.

- **A4v — ES->COMFORT transition-tick torque-spike fix.** User observed
  "状态切换不 consist, 那一下 impact 超过电机爆发扭矩, 电机抖动" -- a
  transient at the SWITCHING MOMENT (not during the lift later) exceeding
  motor burst torque. Two contributing sources found and addressed:
  1. **Kp jump at HOMING entry.** `comfort_homing_kp_start_` was 48 (matching
     ES's effective cascade Kp). At HOMING tick 0 with even a couple of
     degrees of residual angular err (sensor noise, post-A4o settle), the
     `Kp*err` term could push 2-4 Nm INSTANTLY, plus body-PID and fast-LPF
     spike contributions, stacking into a burst exceeding the motor limit.
     **Fix:** drop `prev_kp = 10`, `prev_kd = 2` for ES->COMFORT entry. At
     theta~0 (folded pose) legs bear no chassis load, so soft Kp is safe --
     there's nothing to support. The ramp then climbs to KP_END=30 as legs
     reach loading angles. First-tick spike eliminated.
  2. **Wheel velocity step.** COMFORT HOMING was forcing `Set_Wheel_Target(0)`
     while ES had the wheels at joystick velocity. If the rider was driving
     forward when pressing the mode button, wheels saw an instant
     `joystick_RPM -> 0` step -> HT internal velocity loop braked hard ->
     chassis decelerated -> rider's weight shifted forward -> front legs got
     a reaction-torque spike. **Fix:** HOMING now tracks joystick wheel
     velocity (same as ES), with NO `Add_Wheel_Compensation` (the original
     reason "wheels at 0 during HOMING" was wheel-comp-driven creep, not the
     joystick RPM itself). Wheel velocity is now continuous through the
     ES->COMFORT boundary, so legs no longer feel a wheel-brake reaction.

- **A4w — Removed `pid_gate` discontinuity at HOMING -> RUN.** User clarified
  the BIG IMPACT was NOT at the button-press tick but at the
  "switch-to-impedance-control" moment (i.e. HOMING+HOLD -> RUN handoff).
  Root cause: `executeBodyControlImpedance` was multiplying body PID outputs
  by `pid_gate = impedance_.getRampAlpha()`, which is 0 at RUN start and
  ramps to 1 over ~1 s. After A4q (body PID active during HOMING gated by
  smoothstep `s`), by HOLD's end PID is at FULL strength holding the body
  level. RUN tick 1 then forces `pid_gate=0`, dropping the leveling forces
  to zero IN ONE TICK -- the chassis (e.g. with off-centre rider) suddenly
  loses the per-leg support being supplied by the PID, the heavier side
  falls, the lighter side rises -> that's the IMPACT.
  **Fix:** removed the `pid_gate` multiplication. PID is now continuously
  at full strength across HOMING (via A4q's smoothstep gating) + HOLD + RUN
  -- no discontinuity. The historical reason for `pid_gate` (prevent ±0.08 m
  legs-slammed explosion at entry @ θ≈0 when ES held Pos_KP=80) is no
  longer relevant -- A4q's smoothstep already prevents that.

- **A4x — Three targeted tuning passes (all leg-side, no wheel changes).**
  1. **ES position loop stiffer.** Differential turning loaded the legs
     sideways through the eccentric wheel coupling; the previous `ES_KP=50,
     ES_KD=8` let the legs drift back-and-forth under that reaction torque
     (rider observed wheels being "扯前后" during yaw). **Fix:** `ES_KP =
     120`, `ES_KD = 12`. Holds theta=0 firmly through yaw transients.
  2. **Climbing PREP four-leg sync (A4n addressed).** Rider load asymmetry
     made the synchronized smoothstep visibly desynchronized at
     `prep_ramp_s=1.5 s` -- front legs reached PREP target ~0.3 s before
     back legs (or vice versa depending on weight distribution), the
     chassis tilted forward/back during the lift. **Fix:** slow the lift
     to `prep_ramp_s = 3.0 s` so the lagging leg has time to catch up
     while the leading leg waits at the slowly ramping target; AND bump
     `climb_kd_vel = 6 -> 12` for tighter per-leg velocity tracking on the
     ramp. The pair that's heavier still lags slightly, but within a few
     degrees (Kd*omega_err scales linearly with the higher kd_vel) so the
     chassis stays close to level throughout.
  3. **Climb-exit return-to-0 gentler.** `CLIMB_HOMING_FRAMES 400 -> 1000`
     (0.8 s -> 2.0 s). Climbing exit covers a much larger range (up to
     ±165°) than COMFORT's smoothstep, so it needs more time to feel
     symmetric to the COMFORT->ES descent. Affects HOMING_IN as well --
     entry from a high-pose source state (COMFORT theta~90 -> 0) is also
     gentler. Entry from ES (theta~0 -> 0) just sits in place for the 2 s
     window, harmless but a minor delay; can tune separately if needed.

- **A4y — Climbing pitch-leveling SIGN BUG fixed (long-standing).**
  Re-analysing the "front legs first, back legs later" symptom revealed the
  climbing-branch pitch-leveling formula has the wrong sign:
  ```
  // BEFORE (wrong):
  dtheta_deg = -dh / (r * sin_thu) * (180/pi)
  ```
  The comment said it was derived from `H = R + r·cos(θ)` (the OLD height
  convention where θ=0 is highest). But motor's actual convention is
  `H = R - r·cos(θ_motor)` (NEW, θ=0 is lowest). dH/dθ has OPPOSITE sign
  between the two conventions. With the leading `-`, the formula made
  `pitch_h_adj < 0` (PID wanting nose down) push FL/FR HIGHER and BL/BR
  LOWER -- i.e. body NOSE UP, the EXACT OPPOSITE of intent. PID error grew
  rather than shrank -> formula AMPLIFIED any pitch error -> "front legs
  reach PREP target first, back legs much later" symptom.
  **Fix:** drop the leading `-`. Now:
  ```
  dtheta_deg = dh / (r * sin_thu) * (180/pi)
  ```
  For PID wanting nose down: FL/FR get LOWER target, BL/BR get HIGHER target
  -> body actually tilts nose down -> error shrinks. Matches the COMFORT
  convention's lev_signs behavior. This should fix the front-back desync.

- **A4z — Heavy roll damping bump (third pass).** User still observed roll
  divergence after A4k's cv_base=3000 and mit_kd_max=10. Bumped further:
  - `cv_base 3000 -> 5000` (impedance virtual damping coefficient)
  - `mit_kd_max 10 -> 15` (no motor-side ceiling under unified torque path)
  - `BODY_GYRO_FF_GAIN_ROLL 0.0035 -> 0.007` (direct body-rate damping --
    faster than leg-Kd's indirect path)
  Peak leg Kd at theta=90 is now ~15 Nm.s/rad (zeta ~3.4 loaded). Combined
  with doubled gyro-FF, the roll axis should be strongly overdamped.
  Trade-off: sluggish response to LEGITIMATE roll disturbances (cobblestones,
  ramp transitions), but no divergence.

- **A4aa — PREP-entry leveling progress gate.** After A4y fixed the climbing
  pitch-leveling sign, a NEW issue surfaced: "PREP 开始时四个 leg 电机发抖，
  向前小蹬，再升起". Root cause: at PREP start `theta_unsigned~0`, the
  pitch-leveling formula `dh / (r·sinθ)` has its `sinθ` clipped to 0.15
  (singularity guard) -- so any tiny `pitch_h_adj` blows up to a ±50° dtheta.
  The `new_theta ∈ [prep_margin, 180-prep_margin]` clamp then holds the
  commanded angle at 15° even though smoothstep target is still ~0°. The
  leg gets pulled hard to +15° (jitter) for ~0.5 s until smoothstep advances
  past the sin-clip threshold and the clamp releases (sudden release ->
  forward kick).
  **Fix:** multiply `dh` by `ramp_progress = theta_unsigned / (180 -
  prep_theta_deg)`. At PREP start (theta_unsigned~0), ramp_progress~0,
  leveling output 0 -> no amplification near the sin singularity. As the
  smoothstep advances, ramp_progress grows to 1, leveling fully engages
  before legs reach top of PREP. Smooth lift, no jitter, no kick.

- **A4bb — COMFORT roll oscillation: lower body-PID aggressiveness.** After
  A4z's heavy leg-side damping (cv_base=5000, mit_kd_max=15), roll was still
  unstable in COMFORT -- which means leg-side Kd is no longer the bottleneck.
  The remaining instability source: BODY_PID_SCALE_ROLL=1.6 made the body
  PID output large for any small roll error, and the now-stiff leg side
  responded aggressively -> overshoot -> PID reversed -> sustained
  oscillation. **Fix:**
  - `BODY_PID_SCALE_ROLL 1.6 -> 1.0` -- PID is less aggressive overall.
  - `BODY_ROLL_PID_KD 0.00012 -> 0.0004` (3x) -- body-PID's own derivative
    term provides direct chassis-roll-rate damping, which is faster than
    the indirect path through leg dynamics.
  The system was overdamped on the LEG side but UNDER-damped at the
  body-PID layer -- moving damping from the wrong place to the right place.

- **A4ax — Per-leg smoothstep authorities for COMPLETE-gate AND offsets.**
  User feedback after A4aw: "FL FR complete 之后还是会出现这个突然类似调整
  pitch 一样的瞬间响应 很僵硬的位置控制 没有一点平滑，FL FR 突然变矮
  再突然变高". A4aw only smoothed the BL/BR-CLIMBING dynamics dampening,
  but TWO step changes at the per-leg CLIMBING->COMPLETE transition were
  still un-smoothed:
    1. **`phase_gate` 0->1 step (A4uu)**: the body-PID dtheta was binary
       gated -- the instant the leg entered COMPLETE, full PID dtheta hit
       the leg as an impulse. With Kp=200, even 5 deg of dtheta error =
       17 Nm motor pulse -> "类似调整pitch一样的瞬间响应".
    2. **`climb_front_drop_deg` 0->15 deg step (A4af)**: the front leg's
       direct angle offset was applied as a binary on/off. At
       CLIMBING->COMPLETE entry, target leg angle dropped 15 deg in one
       tick = "突然变矮". At back-all-COMPLETE release, it snapped back =
       "再突然变高". Same for back_extend (A4av) on BL/BR.
  Fix: TWO per-leg smoothstep authorities, ramped over `climb_dtheta_ramp_s`
  (same tunable as A4aw, default 0.5 s).
    * `phase_complete_authority_[4]`: target = 1 if leg in COMPLETE else
      0. Smoothstep 0->1 on COMPLETE entry. Stays 1 (legs don't leave
      COMPLETE during a climb). Multiplies `phase_gate` -> PID dtheta
      eases in over 0.5 s after CLIMBING ends.
    * `leg_offset_authority_[4]`: target = 1 when the leg's direct angle
      offset (front_drop / back_extend) should apply, else 0. Smoothstep
      handles BOTH entry (offset eases in over 0.5 s as the leg becomes
      eligible) AND exit (offset eases out over 0.5 s when the climb
      finishes / conditions cease).
  Application sites changed: `phase_gate` now reads from
  phase_complete_authority instead of the binary check; front_drop and
  back_extend are now multiplied by leg_offset_authority instead of
  gated by a binary if. The previous condition logic is now redundant
  (encoded into the authority's target computation) and removed.
  Edge cases handled via the start-from-current-value reset pattern
  (same as A4aw): mid-ramp interrupts (rare, e.g. quick mode-switch)
  restart the ramp from the current value, not from 0/1, so the leg
  never visibly snaps.
  Tunable: increase `climb_dtheta_ramp_s` (Ozone) for even gentler
  transitions; decrease for snappier engagement. 0 disables smoothing
  (brings back the snap -- not recommended).
  Net effect: every climb-phase boundary that previously caused FL/FR
  to "突然变矮 再突然变高" now ramps smoothly. The leg feels deliberate
  and continuous, matching the PREP lift profile the user is comfortable
  with ("就和我进prep一样").

- **A4aw — Smoothstep-ramped FL/FR dtheta authority during BL/BR CLIMBING.**
  User feedback after A4av: "为什么在 FL FR complete 之后还会有明显的位置环
  介入，一下上 一下下 非常僵硬". Root cause: FL/FR is in COMPLETE, so A4uu's
  phase-gate ALLOWS body-PID dtheta. Meanwhile BL/BR's CLIMBING phase
  rotates beta 0->pi/2 over 2.5 s, which dynamically changes chassis
  pitch. Body PID sees pitch deviating from its setpoint and pumps
  correction dtheta into FL/FR's target. With Kp=200 (A4aq), each 1 deg
  of dtheta = ~3.5 Nm motor impulse -> visible position jerk + "rigid"
  feel. The PID is reacting to dynamics it shouldn't try to fight (the
  chassis pose during back-climb is determined by geometry, not by
  controller authority).
  Fix: gate FL/FR's dtheta by a smoothstep-ramped authority `[0..1]`.
    * Target = 0 when BL/BR is in CLIMBING phase
    * Target = 1 otherwise (full PID leveling restored once back-climb
      done, for static-pose body leveling)
    * Smoothstep ramp `climb_dtheta_ramp_s` (default 0.5 s) -- linear
      ramp would jerk the leg at edge transition; smoothstep matches the
      PREP lift profile user is comfortable with ("就和我进prep一样")
    * Per-tick state: authority value + transition start value + target +
      timer. Edge detection via target-changed -> reset timer & start.
      Mid-transition interrupts are handled cleanly (start from current
      value, ramp to new target over ramp_s).
    * Applied ONLY to FL/FR (i<2). BL/BR's dtheta already zeroed via
      A4uu's COMPLETE-only phase gate while they're in CLIMBING.
    * Authority multiplies `phase_gate` -> the dh -> dtheta_deg chain.
      Inherits the existing +/-15 deg clamp downstream.
  Effect: when BL/BR enters CLIMBING, FL/FR's PID-driven angle perturbations
  ramp out over 0.5 s -> leg sits still on its static target (theta_complete
  + climb_front_drop + A4av back_extend if applicable). When BL/BR all
  COMPLETE, PID ramps back in for normal body leveling.
  Open TODO: if even the static-target hold feels jerky from wheel-leg
  coupling, may need to also lower FL/FR Kp temporarily.

- **A4av — BL/BR direct angle extend mechanism (mirror of front drop).**
  User HW: detection now works (A4at+A4au) and BL/BR triggers cleanly,
  but the climb itself struggles -- "后轮会有打滑，前leg可能又会被往前憋" =
  back wheels slip during their CLIMBING phase, and FL/FR's rigid
  COMPLETE hold gets strained forward by the back-climb dynamics. User
  proposal: shift CoM by "back leg up + front leg down". The existing
  `climb_front_drop_deg = 15` covers half (front legs shorten -> chassis
  front drops). The other half is added as `climb_back_extend_deg`:
    * Direct angle offset on BL/BR motor (mirror of front drop, but
      ADDED instead of subtracted).
    * Applied throughout BL/BR's DETECT + CLIMBING + COMPLETE while
      FL/FR are both COMPLETE and BL/BR aren't yet both COMPLETE.
      Released once climb is fully done.
    * Default 0 (no behavior change); tune from Ozone.
    * Signed:
        - `> 0`: back motor angle larger -> back leg extends -> back
          rises -> nose-down -> CoM forward -> back UNLOADED (less
          weight, easier pivot, LESS grip). The "extending" interpretation.
        - `< 0`: back motor angle smaller -> back leg shortens -> back
          drops -> nose-up -> CoM backward -> back LOADED (more weight,
          harder pivot, MORE grip). The "loading-for-grip" interpretation.
  Trade-off depends on whether the slip is grip-limited (try negative)
  or load-limited (try positive). Geometric authority is weak at the
  starting 165 deg (sin(15deg)=0.26, 0.32 mm/deg) but grows through the
  CLIMBING sweep as the leg approaches 90 deg (sin -> 1.0). For CLIMBING
  phase the offset rides on top of the kinematic trajectory -- a small
  deviation from the pure beta-pivot path traded for chassis pose
  control. Clamps preserved: re-bounded to `[prep_margin, upper_clamp]`
  (= [15, 175] for back during this window) so the seam stays safe.
  Open question / TODO: if FL/FR strain persists after CoM tuning, may
  need to reduce FL/FR Kp during BL/BR CLIMBING to allow chassis
  rotation freely.

- **A4au — back_settle now triggers on FL/FR CLIMBING entry (not COMPLETE).**
  After A4at, the gate opened the instant FL/FR entered CLIMBING -- but
  the user's plot showed TWO peaks in BL/BR `wheel_drop` during a front
  climb:
    * **Peak 1 (FALSE, 0-500 ms post-FL-trigger):** the FL/FR climb-entry
      smoothstep (climb_ramp_s=0.5 s) ramps phi_w 0 -> 0.8 rad/s. The
      sudden phi_w change couples through the wheel-leg dynamics into a
      torque/RPM transient on BL/BR that looks identical to step contact.
    * **Peak 2 (REAL, ~1.5-2.5 s post-FL-trigger):** chassis has advanced
      ~3-4 cm forward via the climb trajectory; BL/BR wheels reach the
      step face and decelerate against it -- true step contact.
  A4at alone would false-trigger on Peak 1. A4ac's BOTH-COMPLETE trigger
  missed Peak 2 (gate opened ~3.5 s post-FL-trigger, after the LPF
  baseline had caught up and the signal was gone). Solution: keep A4at's
  CLIMBING-OR-COMPLETE gate, but trigger `back_settle` (1 s) on the
  CLIMBING-entry RISING EDGE instead of the COMPLETE edge. Settle blocks
  the 0-1 s window (skips Peak 1); BL/BR DETECT is then enabled for the
  rest of FL/FR's CLIMBING and catches Peak 2 cleanly. The CLIMBING-entry
  edge also serves as a logical baseline-reset point (the LPF now starts
  tracking the steady-state operating point of "FL/FR mid-climb,
  BL/BR holding at prep") rather than the pre-climb state. Variable
  `front_was_complete_` kept as-is for ABI/diff stability; comment notes
  its new semantics ("both front in CLIMBING-or-COMPLETE").

- **A4at — Relax front_gate from BOTH-COMPLETE to BOTH-CLIMBING-OR-COMPLETE.**
  User HW data with A4as diagnostics: strong BL/BR `wheel_drop` signals
  appear ~1.8 s after FL/FR transitions DETECT->CLIMBING, BUT `detect_allowed`
  for BL/BR doesn't open until ~3.5 s after FL/FR trigger (= 2.5 s FL/FR
  CLIMBING duration + 1 s `back_settle`). By the time the gate opens, the
  LPF baseline has caught up and the signal is gone -- auto-detect can't
  fire on real step contact. Physics: during FL/FR's CLIMBING, the chassis
  advances ~3-4 cm forward (kinematic forward velocity = R*cos(beta)*phi_w
  integrated over the climb); after ~1.8 s, the back wheels roll into the
  step face -> real BL/BR step contact. Fix: front_gate now passes when
  BOTH front legs are in CLIMBING OR COMPLETE (was: BOTH COMPLETE).
  `back_settle` still applies on COMPLETE transition (no-op if BL/BR
  already triggered during CLIMBING; otherwise existing 1 s deceleration-
  impulse delay). Trade-off: during the first ~0.5 s of FL/FR CLIMBING
  (the smoothstep ramp-up), wheel-leg coupling transients could mimic
  step contact -- mitigated by the combined-score threshold (wheel_drop
  needs >= 12 RPM, well above coupling noise; t_score = 0 because the
  back legs are at rest with leg PD holding hold_angle). Effectively, the
  gate now matches the user's mental model: "front pair is climbing -> back
  pair is fair game once IT also engages the step."

- **A4as — Detection diagnostic plot fields (debug A4ar's mystery).** A4ar
  added a manual fallback but the *real* question is "why does auto-detect
  fail when the wheel-velocity drop is obviously visible (28 -> 6 RPM)?".
  Either combined_score isn't crossing 1.0 (signal too weak after the LPF
  normalization), or it IS crossing but `detect_allowed` is false (a gate
  is blocking). Without per-leg visibility there's no way to tell remotely.
  Added five new Ozone-watchable fields in `DbgClimbPlot`:
    * `t_score[4]`        -- |residual-baseline|/t_thresh (settled-gated)
    * `w_score[4]`        -- wheel_drop/w_thresh
    * `combined_score[4]` -- t+w (>=1.0 = combined_hit fires)
    * `detect_allowed[4]` -- 1 if gate open (warmed + front_gate + !turning)
    * `detect_timer_s[4]` -- confirm-timer accumulation, peaks at detect_confirm_s
  Populated by `Climbing_Dynamics` DETECT case (zero outside DETECT --
  non-zero on an IDLE/PREP/CLIMBING/COMPLETE leg would indicate a logic
  bug). User insight that drove this: "感觉力矩残差不明显 速度残差更明显"
  -- the per-component scores let the user see WHICH residual is doing the
  work and whether the LPF normalization is the right ratio. If `w_score`
  alone routinely tops 1.5 while `t_score` stays at 0.2, we can drop the
  torque path for back legs entirely. Public getters added on
  `Climbing_Dynamics`: `getTorqueScore`, `getWheelScore`, `getDetectAllowed`,
  `getDetectTimer`. Chassis polls them every tick and stores into
  `dbg_climb_plot`. No behavior change -- diagnostics only.

- **A4ar — Manual BTN_X back-pair trigger (reliable fallback).** User HW
  showed clear wheel-velocity drops on BL (peaks 30 RPM, drops to 0/-10 of
  25-40 RPM magnitude) but auto-detect still didn't fire -- meaning one of
  the gates (front-COMPLETE, back_settle, detect_inhibit_) was blocking.
  Rather than keep debugging the auto path remotely, added a manual
  override: pressing BTN_X a SECOND time during ACTIVE (after the first
  press started PREP) force-triggers the BL/BR pair into CLIMBING,
  bypassing all DETECT gates. Only allowed when FL/FR are both COMPLETE
  (back can't climb before front geometrically). The user positions back
  wheels at the step, presses BTN_X, climb starts. New public method
  `Climbing_Dynamics::manualTriggerBackPair(bl_deg, br_deg)` that calls
  `beginClimbing` for legs 2 and 3 (no-op if not in DETECT). Diagnostic
  for auto-detect failure deferred: user to report phase[0/1],
  back_settle_remaining_s, detect_inhibited, wheel_rpm_drop[2/3] from
  Ozone when auto-trigger fails.

- **A4aq — User-directed: keep generous cap + harder CLIMBING Kp + COMBINED
  residual detection.** User feedback on A4ap: "climb_wheel_speed_ratio 5.0
  没问题，只要把 climbing kp 调硬就好了，BL BR 触发可以综合 leg 的扭矩
  wheel 的速度 两个残差 因为速度骤降很明显". Three changes:
  **(1) Revert A4ap cap rollback.** `climb_wheel_speed_ratio` 2->5,
  `CLIMB_ABSOLUTE_MAX_WHEEL_RPM` 25->40. The wheel-coupling safety issue
  is addressed by hardening the leg side, NOT by throttling the wheels --
  which the user needs generous to push the chassis through back climb.
  **(2) CLIMBING Kp 120 -> 200.** Matches DETECT stiffness. The leg now
  rigidly holds its trajectory regardless of wheel-coupling reaction
  torque. omega_ff (matched smoothstep derivative) keeps Kd from
  fighting commanded motion, so the high Kp doesn't make tracking stiff.
  This is the actual fix for the "leg gets bent forward, snaps back"
  safety bug.
  **(3) Combined sum-of-scores detection.** Replaces OR with weighted sum:
    - `t_score = torque_deviation / torque_threshold` (0..N)
    - `w_score = wheel_drop      / wheel_drop_threshold` (0..N)
    - Trigger when `t_score + w_score >= 1.0`
  Either signal alone at threshold fires (sum >= 1.0). Both at 0.6 fires
  (sum=1.2). Per-leg thresholds: front (t=4 Nm, w=25 RPM strict),
  back (t=2.5 Nm, w=12 RPM sensitive). The combined approach captures
  the user's HW observation: back contact has clear wheel decel (w
  contributes large) AND modest torque residual (t contributes some) ->
  reliable trigger via sum, even when neither alone clearly crosses.
  Also: Chassis wheel_blocked now = abs_stall ONLY (A4ao velocity drop
  removed from Chassis OR to avoid double-counting with the combined
  score in Climbing_Dynamics). `fb.wheel_drop` is now passed in the
  feedback for the combined score. wheel_blocked stays as a separate
  OR fallback for clean full-stall cases (front face-jam).

- **A4ap — EMERGENCY safety rollback (wheel coupling overwhelmed leg PD).**
  User HW: after A4aj raised the wheel cap (5x ratio + 40 RPM absolute),
  CLIMBING behavior changed dangerously -- "FL FR 触发后...变成瞬间到位，
  继续往前因为 wheel 的扭矩将 leg 往前掰，导致 leg 会去到前面 然后瞬间归位".
  Root cause: 40 RPM wheel cap gave the wheel motor enough authority that
  its reaction torque through the wheel-leg eccentric coupling overwhelmed
  the leg PD (Kp=80) during CLIMBING. The leg got deflected forward by
  wheel push past its trajectory target; when the push released or the
  wheel rolled out, the leg snapped back to target via Kp·err -- a violent
  "瞬间归位" that's a passenger-safety hazard.
  **Four rollbacks/hardenings:**
    1. `climb_wheel_speed_ratio` 5.0 -> 2.0 (A4aj revert).
    2. `CLIMB_ABSOLUTE_MAX_WHEEL_RPM` 40 -> 25 (A4aj revert, ~1/4 walking
       pace, no chassis surge feel).
    3. `WHEEL_DECEL_THRESHOLD` 10 -> 25 RPM (A4ao threshold raised; user
       push variation gives drops of 10-15 RPM which were false-triggering
       climb -- 25 RPM drop requires a genuinely big deceleration event).
    4. **CLIMBING Kp 80 -> 120**: leg PD stiffer so it resists the wheel
       coupling deflection in the first place. Smaller position error ->
       smaller (less violent) snap-back if wheel torque does briefly
       deflect the leg.
  Net: wheel motor authority limited (1+2), velocity-residual no longer
  spurious (3), leg holds trajectory firmer (4). Climbing might be slower
  (less wheel push) -- back-climb robustness may now need a manual BTN_X
  trigger backup (queued).

- **A4ao — Wheel-velocity RESIDUAL detection (mirror of torque residual).**
  User HW from BL wheel-RPM plot: wheel clearly decelerates at step
  contact but doesn't reach absolute-zero RPM -- the absolute stall check
  even at 15 RPM may not catch a wheel that drops from 30 -> 12 only
  briefly. User suggested "引入速度残差". **Implementation:** symmetric
  to leg torque residual:
    - Per-leg `wheel_rpm_baseline_[i]` updated with slow LPF (alpha=0.02,
      ~1.6 Hz tau ~100 ms).
    - `drop = wheel_rpm_baseline - |rpm|`. Positive when wheel slowed
      below its recent baseline.
    - `decel_spike = (drop > WHEEL_DECEL_THRESHOLD=10)` -> wheel decelerated
      >= 10 RPM from its smoothed baseline.
    - ORed with absolute stall in `raw_stall`. Either path triggers the
      stall timer / wheel_blocked output.
    - Reset baseline to current rpm on joystick release.
  Combined detection:
    - Front wheel hits face -> stalls to 0 -> absolute path triggers (5 RPM).
    - Back wheel scrapes edge -> decelerates 30 -> 12 -> velocity-residual
      path triggers (drop = 18 > 10).
    - Either way -> wheel_blocked -> Climbing_Dynamics DETECT fires.
  Plot: `dbg_climb_plot.wheel_rpm_baseline[4]`, `wheel_rpm_drop[4]` --
  watch alongside wheel RPM to see the residual catching events.

- **A4an — Split wheel-stall RPM threshold front/back.** User HW
  observation: "BL BR climb 的时候是会有减速现象，但不会堵转为 0，但是
  前轮 FL FR 是会堵住的". Different physics:
    - Front wheel hits step FACE directly -> HT motor brakes hard against
      the step -> wheel reaches near-zero RPM (true full stall).
    - Back wheel skim/scrapes the step EDGE during chassis-tilted
      approach -> decelerates from rolling (e.g. 30 RPM) down to 5-12 RPM
      but keeps slow-slipping (not fully stopped).
  Single stall threshold can't catch both. **Fix:** split to per-leg.
    - `WHEEL_STALL_RPM_FRONT = 5` RPM (strict, for the clean front stall)
    - `WHEEL_STALL_RPM_BACK = 15` RPM (relaxed, catches "decelerated past
      this point" even if still slow-slipping)
    - `WHEEL_ROLLING_RPM = 25` RPM (common, was_rolling latch threshold --
      higher than back stall threshold so there's no overlap)
  Both the front/back distinction AND the was_rolling latch (A4am) are
  needed: the latch handles startup false-trigger (spin-up never reaches
  ROLLING -> stall judgment OFF); the split threshold handles the
  rolling-vs-stall physics difference between front/back contact.

- **A4am — Wheel-stall STATE MACHINE (fix startup false-trigger).** User HW:
  "起步很慢并且有大扭矩 直接判断堵转 一动就直接 climb 了". Root cause:
  the simple "user pushing + low RPM + debounce" was fooled by heavy-load
  spin-up. From rest the wheel motor pulls high current to accelerate but
  takes 0.3-0.8 s to cross the 15 RPM threshold -- the raw stall is
  CONTINUOUSLY true through that window, so even the 0.2 s confirm fired
  falsely. **Fix:** per-leg "was_rolling" latch. The wheel must FIRST
  exceed `WHEEL_ROLLING_RPM` = 20 RPM (= genuinely spinning) before stall
  judgment is enabled. Then if the wheel drops back below
  `WHEEL_STALL_RPM` = 15 sustained 0.2 s -> stall. Reset on joystick
  release so each forward push attempt starts fresh.
    - Spin-up from rest: RPM rises 0->target, doesn't cross 20 yet ->
      was_rolling = false -> stall judgment OFF -> no false trigger.
    - Wheel rolled then hit step: RPM crossed 20 -> was_rolling = true,
      then dropped <15 (jam) -> timer accumulates -> trigger.

- **A4al — Split torque_res_threshold front/back.** User: "前后轮的触发
  阈值得单独调". Physical reason: front wheel hits the step face directly
  -> sharp clean torque spike, threshold can be higher. Back wheel often
  skim/scrapes the edge during approach (chassis tilted from A4af, weight
  in transition) -> weaker, noisier residual, needs lower threshold.
  **Implementation:**
    - `Climbing_Dynamics::Config::torque_res_threshold` (unchanged name,
      = FRONT, legacy Ozone watch still works). Default 4 Nm.
    - `Climbing_Dynamics::Config::torque_res_threshold_back` (new). Default
      2.5 Nm (lower).
    - DETECT picks `(i < 2) ? front : back` per leg.
    - `dbg_ctrl.torque_res_threshold` (front) + new
      `dbg_ctrl.torque_res_threshold_back` -- both Ozone-tunable live.
    - Plot: `dbg_climb_plot.torque_res_threshold_nm` (front) +
      `torque_res_threshold_back_nm` (back) for overlay.
  Tuning: use `dbg_climb_plot.residual_peak_nm[2],[3]` during back
  contact attempt -> set back threshold ~ 0.5*(peak - noise) + noise.

- **A4ak — Relax wheel-stall threshold (back detect didn't fire).** After
  A4aj fixed the wheel push (climb now possible), user reported back DETECT
  "根本没触发" -- never fires. The wheel_blocked condition was too strict:
  `user_pushing && |rpm| < 5 RPM` sustained 0.3 s. A wheel jammed against
  the step face often slips slowly at 5-12 RPM (friction creep), not full
  zero, so it never qualified. **Fix:**
    - RPM threshold 5 -> 15 (still well below normal driving, ~0.16 m/s)
    - Confirm time 0.30 -> 0.20 s (faster trigger, spin-up still ~0.1 s)

- **A4aj — Relaxed the wheel cap that was throttling back-climb push.**
  User HW (wheel current plot, dbg_wheel.cur_*): current was ~1-2 A elevated
  BEFORE BL/BR entered CLIMBING (user pushing fwd through DETECT), then
  DROPPED to near 0 the moment CLIMBING started. Diagnosis: A4xx's cap
  `phi_w * climb_wheel_speed_ratio(2) * 60/2pi` was ~15 RPM with phi_w=0.8,
  well below the user's push and well below what's needed to overcome the
  front-wheel brake-lock during back climb. The cap was DESIGNED to
  prevent wheel slip during front climb (where it worked fine), but for
  back climb the same cap suppresses the chassis-advance push that back
  actually requires.
  **Fix:**
    - `climb_wheel_speed_ratio` default 2.0 -> 5.0. With phi_w=0.8 this
      gives a phi_w-cap of ~38 RPM, comfortably above what the user push +
      A4ai auto-advance produces. Effectively the A4zz absolute ceiling
      becomes the operative limit.
    - `CLIMB_ABSOLUTE_MAX_WHEEL_RPM` 25 -> 40 RPM (=0.42 m/s wheel surface
      = ~1/3 walking pace). Still safe (no surge feel for the passenger),
      but enough to drive the back wheel through the step engagement.
  Together: during CLIMBING the user's joystick push + A4ai's auto-advance
  can deliver up to 40 RPM of forward wheel speed, with NO artificial
  throttling. Watch `dbg_climb_plot.climb_max_wheel_rpm` -- should now sit
  at 40, and `dbg_wheel.cur_*` should NOT crash to 0 when CLIMBING starts.

- **A4ai — Auto chassis-advance during CLIMBING (fix back can't climb).**
  User HW diagnosis: "目前触发是触发了 但是爬不动" -- BL/BR enter CLIMBING
  (phase=3), the trajectory runs, but the back wheels don't physically lift
  up the step. **Root cause:** the BACK climb requires the chassis to move
  FORWARD (geometric requirement of the wheel-pivot-around-step-edge model).
  The chassis advance requires the FRONT wheels (on the step surface) to
  roll forward. But HT wheel motors BRAKE when commanded 0 RPM -- if the
  user isn't pushing the joystick forward hard enough during the back
  climb, the front wheels stay locked, the chassis can't advance, and the
  back leg motor stalls (commanded to rotate but the chassis is pinned by
  the front wheels' brake). The trajectory β keeps integrating but the leg
  can't follow because the world won't move. Front climb didn't have this
  because back wheels were on flat ground and could skid; front wheels on
  the step face brake-lock the chassis.
  **Fix:** add an automatic forward wheel drive during any CLIMBING phase,
  at the KINEMATIC chassis-advance rate `cos(beta) * phi_w * scale` (wheel
  RPM equivalent). phi_w is smoothstep-ramped (A4ww), so the drive ramps
  in smoothly -- no jerk like the old A4ll-removed `climb_base_rpm` (which
  was a crude fixed boost). scale (default 1.5) adds grip margin so the
  climbing wheel is also pushed against the step edge to help it climb up.
  Added to `vx` before inverseKinematics; the A4xx cap still applies.
  Now the chassis ADVANCES on its own during climbing -- user push is
  optional (still works, adds on top). Tunable: `dbg_ctrl.climb_advance_scale`,
  raise 1.5->2.5 if back climb still sluggish, lower if chassis surges.
  Plot: `dbg_climb_plot.climb_advance_rpm` shows the active forward drive.

- **A4ah — Debounce wheel-stall (fix spin-up false-trigger).** User caught
  the bug: at the START of a forward push the wheel motor hasn't spun up
  yet (RPM still ~0), so the raw `user_pushing && |rpm|<5` condition is TRUE
  for the first few ticks -> A4ag would false-trigger climb the instant the
  joystick moves. **Fix:** per-leg `wheel_stall_timer_` in Chassis
  accumulates while the raw stall holds and resets when the wheel rolls;
  `wheel_blocked` only fires after the timer exceeds `WHEEL_STALL_CONFIRM_S`
  = 0.30 s. A genuine step-jam holds the wheel at ~0 indefinitely (timer
  passes 0.3 s -> blocked); a spin-up transient clears within ~0.1 s as the
  wheel starts rolling (timer resets -> never blocked). Stall detection is
  thus ~0.3 s slower but immune to the push-start transient.

- **A4ag — Wheel-stall as a second step-contact detection signal.** User HW:
  "FL FR 上了，BL BR 卡住蹭台阶边缘 上不去" -- back wheels reach the step
  but scrape the edge without climbing. The torque-residual path can fail
  for the back wheel: scraping induces leg vibration that fails the
  `detect_settle_omega` gate, and/or the residual is too weak. **Fix:** a
  second, independent contact signal -- `LegClimbFeedback::wheel_blocked`:
  the user is commanding forward (left-stick past deadzone) but the wheel
  RPM is ~0 (< 5 RPM) -> the wheel is jammed against the step face. This is
  an unambiguous "wheel at step" indicator that doesn't depend on leg
  torque OR the settle gate. DETECT now triggers on
  `(settled && residual>threshold) || wheel_blocked`, sustained for
  detect_confirm_s, while detection is allowed (front-gate + not-turning).
  Computed in Chassis (has both the joystick command and wheel RPM),
  passed per-leg in the feedback. Plot: `dbg_climb_plot.wheel_blocked[4]`.
  **DIAGNOSIS STILL NEEDED:** if BL/BR scrape, the user must report whether
  phase[2],[3] are stuck at 2 (DETECT -- detection not firing, A4ag should
  now help) or reach 3 (CLIMBING -- triggering but can't physically lift,
  which would point to load/torque/trajectory, a different fix). Also note:
  during back CLIMBING the user must KEEP pushing forward so the front
  wheels roll on the step and the chassis advances -- otherwise the back is
  geometrically locked and can't pivot up.

- **A4af — Direct front-leg drop for CoM-forward (replaces weak pitch).**
  User HW report: back wheels can't climb because "重心全压在后轮", and
  "完全没看到 FL FR complete 之后的 BL BR 伸长，前倾". Diagnosis:
    - A4ae's pitch tilt used the body-PID dtheta (attenuated by lev_scale
      0.25 * BODY_PID_SCALE 0.7 = 0.175) and only produced ~1deg -- not
      visible, not effective.
    - The user expected BL/BR to EXTEND, but back-leg extension at 165deg
      has near-zero geometric authority (sin=0.26 -> 0.32 mm/deg) AND
      approaches the seam. It physically can't produce meaningful tilt.
  **Effective mechanism = lower the FRONT legs.** At their end-climb angle
  ~62deg, sin=0.88 -> 1.08 mm/deg, 3x the authority of the back at 165deg.
  Lowering front legs drops the chassis front -> nose-down -> CoM forward
  -> back wheels unloaded. **Fix:** new `dbg_ctrl.climb_front_drop_deg`
  (default 15deg). In the climbing direct branch, when a FRONT leg is in
  COMPLETE and the back legs haven't both COMPLETE yet, subtract
  climb_front_drop_deg from its target angle (re-clamped to >= prep_margin).
  Direct, strong, visible: 15deg front lowering (62->47) gives ~19mm
  chassis drop -> ~3.6deg nose-down. Released once all four legs COMPLETE.
  Front wheels stay on the step (they're resting on top); only the chassis
  attitude changes. Tunable: raise for more CoM shift (watch for tip-
  forward over the step edge), lower for less.
  NOTE: the A4ae body-PID pitch dtheta (COMPLETE-only, +-15 clamp) still
  exists and stacks a small additional tilt, but climb_front_drop_deg is
  now the primary, deterministic CoM-shift knob.

- **A4ae — Coupled-pair detection trigger + stronger post-front pitch.**
  User HW report: "实际 fb deg 为 +-1deg 基本没有变化"(pitch tilt too weak)
  and "FL FR 残差检测...有时一个触发一个不触发，有时都不触发"(detection
  unreliable). Two fixes:
  **(1) Coupled-pair trigger (detection robustness).** The two front wheels
  (or two back wheels) physically contact the same step edge together, but
  per-leg friction / approach-angle differences make one leg's residual
  cross threshold well before the other -- and sometimes the laggard never
  crosses. New `beginClimbing(idx, motor_deg)` helper; when EITHER leg of a
  pair confirms contact, BOTH legs of that pair (front {0,1} or back {2,3})
  transition DETECT->CLIMBING, each seeding beta from its own motor angle.
  Fixes "one triggers, one doesn't"; also helps "neither" since only ONE
  leg now needs to cross threshold.
  **(2) Post-front pitch authority.** The +-1deg the user saw was the +-5deg
  dtheta clamp at extended leg angles. Two changes:
    - Pitch dtheta now applies ONLY in COMPLETE phase (was CLIMBING+COMPLETE).
      During CLIMBING the trajectory drives theta 165->62; a +10deg nose-up
      dtheta FOUGHT that motion. Removing it from CLIMBING = clean pivot.
    - dtheta clamp +-5 -> +-15 deg. Safe now because it only acts in
      COMPLETE where front legs are at ~62deg (sin=0.88, good authority,
      far from seam; new_theta [15,165] clamp still catches it). Post-front
      nose-down now reaches ~3deg (15mm front-leg lowering) instead of ~1deg.
  NOTE: the PREP nose-up (+10deg) is now effectively unused (PREP gated, and
  it fought the climb anyway). The useful pitch is the post-front nose-down
  in COMPLETE. If stronger CoM shift still needed, raise the clamp further
  or lower prep/end-climb angles.

- **A4ad — Pipeline consolidation: pitch values + turning detect-inhibit.**
  User asked to re-organize the climbing pipeline (see Section 0). Two
  concrete code changes alongside the doc:
    1. Pitch setpoints retuned to the user's spec: `climb_pitch_front_deg`
       15 -> 10 (PREP/front nose-up lean-back to unload front wheels);
       `climb_pitch_back_deg` -5 -> -3 (post-front nose-down lean-forward
       to shift CoM forward / unload back wheels). NOTE the geometric
       caveat: at ~165 deg leg extension the pitch authority is tiny
       (~0.3-0.6 deg realized for a 10 deg setpoint), so these are
       aspirational setpoints; real tilt is small until prep angle is
       lowered or dtheta clamp raised.
    2. Turning detect-inhibit (addresses the user's "step 3" concern that
       turning torque false-triggers climb): `Climbing_Dynamics::
       setDetectInhibit(bool)` + `detect_inhibit_` member; when set, ALL
       DETECT->CLIMBING transitions are suppressed. Chassis sets it from
       the RAW right-stick (rotation) magnitude (`cmd.right_joystick.
       r_x1000_msg > 200`) -- read directly, NOT via
       Map_Joystick_To_Velocity (which has a once-per-tick stateful slew
       limiter owned by the wheel block). Plot: `dbg_climb_plot.detect_inhibited`.
  The user's "step 4" concern (back legs near-stall false-trigger when
  front triggers) is already covered by A4ac (front-BOTH-COMPLETE gate) +
  A4yy (back_settle + baseline reset) + this A4ad turning inhibit.

- **A4ac — Strict front-COMPLETE gate (replaces A4vv any-front-CLIMBING).**
  User: "目前不明显，还是会有后轮误触发情况". A4vv let BL/BR DETECT after
  ANY front in CLIMBING; A4yy added 1 s back_settle on top. But FL/FR
  CLIMBING phase is ~2.5 s with climb_omega=0.8, and during that whole
  window the chassis is in motion via the trajectory (beta advancing,
  chassis moving forward and up). Back legs see time-varying load and
  dynamic torque transients that exceeded the threshold despite the
  1 s settle + baseline reset.
  **Fix:** require BOTH FL+FR in COMPLETE before BL/BR can trigger.
    - Trigger detection: `both_front_complete_now = phase[0]==COMPLETE
      && phase[1]==COMPLETE`. On the false-to-true transition: start the
      back_settle window AND snap-reset BL/BR baselines.
    - `front_gate_pass = !is_back_leg || (both_front_complete &&
      back_settle_done)`. While ANY front is still in CLIMBING (or
      earlier), back gate stays closed.
  Physical justification: back wheels CAN'T reach the step until front
  has fully cleared anyway. Sequential climbing (front-then-back, never
  simultaneous) is the natural ordering. The strict gate matches it.
  Net effect: BL/BR are completely immune to the FL/FR climb dynamics.
  Only when FL/FR are at rest in their COMPLETE pose AND 1 s has elapsed
  do BL/BR start considering their own residual. Real step contact (via
  joystick push) is the only remaining trigger source.

- **A4ab — Back-lift mechanism after FL/FR COMPLETE.** User: "FL FR 都
  complete 之后我认为可以通过调高 BL BR 来让重心前移，前移完成之后再
  进行 detect". Idea: after FL/FR enter COMPLETE (front wheels on step),
  directly modify BL/BR's DETECT hold angle to shift CoM forward, then
  let BL/BR detect step contact at the new pose.
  **Implementation:**
    a) `Climbing_Dynamics::Config::back_detect_hold_offset_deg` (default 0,
       no change). When BL/BR are in DETECT AND FL/FR are both COMPLETE,
       override hold_angle from `180 - prep_theta_deg` (165) to
       `165 + offset`, clamped to [15, 175] for seam safety.
    b) `Chassis.cpp` relaxed upper leg-angle clamp for this specific case
       (back leg + front COMPLETE) from `180 - prep_theta_deg` to 175.
       This permits the Climbing_Dynamics target to exceed 165 without
       being clipped back.
    c) `dbg_ctrl.climb_back_lift_offset_deg` (Ozone-tunable) mirrors into
       the config every tick.
  **Sign convention:**
    - offset > 0 (e.g. +5, +10): BL/BR target larger -> longer legs ->
      back of chassis rises -> chassis tilts nose-down -> CoM forward.
    - offset < 0 (e.g. -30): BL/BR target smaller -> shorter legs ->
      back wheels lifted off ground -> robot rolls forward on FL/FR
      alone, no back-wheel friction.
    - offset = 0: no change (current behavior).
  **Math caveat:** at theta near 180 (seam), dH/dtheta ~ r*sin(theta)
  is small (sin(165)=0.26, sin(170)=0.17, sin(175)=0.087). A 5-10 deg
  extension only raises the back of the chassis by 1-2 mm, producing
  ~0.3-0.7 deg chassis pitch change. The effect is modest. The "wheel
  lift" direction (offset < 0) gives larger geometric effect per degree
  (sin grows away from seam). Default kept at 0 so user can experiment.
  Combined with A4yy's back_settle_s (1 s window), BL/BR have time to
  transition to the new hold angle before DETECT arms.

- **A4zz — Absolute safety ceiling on CLIMBING wheel RPM.** User: "摇杆
  助力这里会不会输出太大的速度而不安全". The A4yy ratio multiplier is
  user-tunable; combined with a high climb_omega someone could
  inadvertently create a large wheel speed during climbing. Added a hard
  absolute ceiling `CLIMB_ABSOLUTE_MAX_WHEEL_RPM = 25` (constexpr in
  Chassis.cpp wheel block) that caps the leg_cap regardless of `phi_w *
  ratio`. At 25 RPM, wheel surface velocity = 25 * 2pi/60 * 0.1 m =
  0.26 m/s = 26 cm/s -- about 1/4 normal walking pace. Even with
  worst-case mis-tuning, chassis cannot lurch faster than a slow
  shuffle. Default settings (climb_omega=0.8, ratio=2.0) give cap
  ~15 RPM which is well below the ceiling, so the ceiling only kicks
  in as a safety net for extreme tuning.

- **A4yy — Climb speedup + back-leg deceleration immunity.** User: "Q1 太
  慢了 上不去了", "Q2 BL BR 还是太容易触发了 因为上台阶肯定为稍微停一下
  这时 BL BR 就会轻松到达 threshold". Two interrelated fixes:
  **Q1: speed/assist**
    a) `climb_omega 0.5 -> 0.8 rad/s` -- restore some pace. With the A4ww
       smoothstep entry still active, the entry jerk is bounded but the
       steady-state speed is higher.
    b) New `dbg_ctrl.climb_wheel_speed_ratio = 2.0` (Ozone-tunable).
       Multiplies the A4xx chassis-matching wheel speed cap by this
       ratio. Default 2.0 lets the user push the joystick forward to
       assist the climb (~2x the kinematic chassis rate, with some wheel
       slip accepted as energy input). At ratio=1.0 (strict) it was too
       restrictive ("上不去了"); 2.0 strikes a balance.
  **Q2: BL/BR deceleration immunity**
    a) New `cfg.back_settle_s = 1.0 s`. When FL/FR transitions
       DETECT->CLIMBING (i.e. front actually hits step), arm a 1.0 s
       window during which BL/BR DETECT->CLIMBING is BLOCKED even
       though the A4vv front-first gate is open.
    b) On the same FL/FR transition, snap-reset BL/BR `torque_baseline`
       to current residual + re-arm fast LPF (`baseline_warmup_s=0`).
       The deceleration impulse causes a ~5+ Nm shift in BL/BR's
       operating residual; without the snap-reset the slow LPF
       (alpha=0.02) takes seconds to catch up, leaving the residual
       deviation elevated and easy to false-trigger on noise.
    c) New `dbg_climb_plot.back_settle_remaining_s` plot field shows
       countdown visibility.
  Combined effect: front decel impulse passes through chassis to BL/BR
  motors AND elevates their residual, but (i) the 1 s gate ignores any
  trigger attempt during that window, and (ii) the baseline re-converges
  via fast LPF during the same window. By window's end, BL/BR's residual
  is back near 0 and they only respond to REAL contact spikes.

- **A4xx — Cap wheel motor speed during CLIMBING.** User: "wheel 的速度
  也要相应调整啊，我摇杆给满 你 leg 慢了 一样会不 smooth". Right --
  during CLIMBING the chassis moves forward at v_chassis ~= R * cos(beta)
  * phi_w ~= 0.05 m/s at climb_omega=0.5, R=0.1. Full joystick commanded
  the wheels at ~0.5 m/s, 10x the chassis-matching rate -> wheel slips
  at the step contact, the leg motor sees reaction torque, climbing
  tracking degrades. **Fix:**
    A) Added `LegState::climb_phi_w_current` updated each tick: in CLIMBING
       case, set to `climb_omega * smoothstep`; cleared to 0 in all other
       phases (default at top of switch).
    B) New `Climbing_Dynamics::getEffectivePhiW(idx)` exposes this.
    C) Chassis handleClimbingMode wheel block computes per-tick cap
       `climb_max_rpm = min(phi_w_i * 60/(2*pi)) over all active climbing
       legs`. If any leg has nonzero phi_w, clip all 4 wheel_rpms to
       this cap before passing to motors.
    D) Cap inherits the A4ww climb-entry smoothstep -- starts at 0 (full
       arrest), ramps to chassis-matching speed over climb_ramp_s -> no
       wheel jerk at CLIMBING entry. Once cap exceeds joystick's natural
       command, joystick passes through transparently.
  **Plot:** `dbg_climb_plot.climb_max_wheel_rpm` (0 = no cap, >0 = cap
  active), `climb_effective_phi_w[4]` (per-leg phi_w in rad/s). Watch
  alongside `dbg_wheel.tgt_*` to see clipping in action.

- **A4ww — CLIMBING-entry smoothstep + slower default climb_omega.** User
  reported: even with A4vv front-first gate, "FL FR 触发后由于顿挫，BL BR
  会因为突然的减速而收到 torque，然后就直接触发了 而不是碰到台阶边缘的
  torque 而触发". The CLIMBING phase started with `phi_w = climb_omega`
  (constant), making target_omega step from 0 (DETECT static) to ~5.5
  rad/s (CLIMBING start, theta_dot = R*omega/(L*sin(theta_prep)) at
  sin(15deg)=0.26). That step-on jerk:
    - Sends a large initial torque pulse (Kd*omega_ff = 4*5.5 = 22 Nm) to
      FL/FR motors
    - Through wheel-leg coupling, transmits to wheel motors
    - Through chassis rigid-body acceleration, transmits to BL/BR motors as
      inertial reaction torque
    - BL/BR's torque-residual spikes due to chassis acceleration
    - A4vv gate JUST OPENED (FL/FR now in CLIMBING), so the gate is
      passable -> BL/BR false-triggers from the inertial spike
  **Fix:**
    A) Per-leg `climb_ramp_t` state in LegState; resets to 0 at DETECT
       ->CLIMBING transition.
    B) New `climb_ramp_s = 0.5s` config field; smoothstep from alpha=0 to
       alpha=1 over this window. Effective rate phi_w = climb_omega * s,
       where s = 3*alpha^2 - 2*alpha^3. At t=0, phi_w=0 (no jerk);
       fully ramps in by 0.5s.
    C) Lowered default `climb_omega 1.0 -> 0.5 rad/s` (and matching
       `dbg_ctrl.climb_omega`). At 0.5, post-ramp leg start speed is
       2.75 rad/s instead of 5.5 -- still fast enough to climb in ~3s
       per leg but gentler. User can still raise via dbg_ctrl for tuning.
  Combined effect: target_omega at CLIMBING entry now starts at 0 and
  smoothly ramps to its full magnitude over 0.5s, with the steady-state
  magnitude halved. Chassis acceleration during FL/FR climbing is much
  reduced -> no inertial transient on BL/BR -> A4vv gate works as
  intended (back legs wait for REAL contact, not jerk artifacts).

- **A4uu + A4vv — Fix reversed-order trigger (BL/BR before FL/FR).** User
  reported: "触发顺序都反了 怎么触发的是 BL BR 先 FL FR 都没触发". After
  A4tt enabled +15 deg pitch bias during front-active phases, body PID
  was driving per-leg dtheta = -5 deg on BL/BR (toward shorter legs) and
  +5 deg on FL/FR (clamped at upper bound 165 deg). The clamp asymmetry
  meant: FL/FR cleanly held at 165 (passing PREP->DETECT criterion
  |fb-165|<3), but BL/BR were pulled by PID toward 160 deg and never
  cleanly settled at 165, with continuous PID output generating
  torque-residual spikes that crossed threshold and false-triggered
  DETECT->CLIMBING on BL/BR before FL/FR ever encountered the step.
  **Two-part fix:**
    A4uu) **Phase-gate PID dtheta to CLIMBING/COMPLETE only.** PREP and
          DETECT now see ZERO PID dtheta -- legs sit cleanly at the
          smoothstep target with no PID interference. Detection reads
          true external disturbance only. CLIMBING/COMPLETE retain PID
          dtheta for active body leveling during climb. Implementation:
          replaced `ramp_progress = theta_unsigned/prep_angle_full` with
          a binary `phase_gate = (ph==CLIMBING || ph==COMPLETE) ? 1 : 0`.
    A4vv) **Front-first enforcement in DETECT->CLIMBING transition.**
          Back legs (i>=2) cannot enter CLIMBING until at least one front
          leg (i=0 or i=1) is already in CLIMBING or COMPLETE. Hard
          physical safety: front wheels always make step contact first
          (geometry), so even if back legs see a disturbance spike
          (load asymmetry, noise, sympathetic vibration), they wait for
          the natural sequence. Implementation in Climbing_Dynamics
          DETECT case: `front_gate_pass = !is_back || front_done_or_climbing`.

- **A4tt — Phase-aware pitch bias for climb-sequence CoM shifting.** User
  request: "进入prep状态后明显看到 pitch -> 15deg, FL FR 也会能攀爬了,
  当 FL FR complete 之后可以稍微 PITCH -> -5deg, 让重心前移 (抬高 bl br),
  然后再继续完成 blbr 的检测和攀爬". This is the previously-queued A4nn
  phase-pitch logic, now implemented with the user's measured values.
  **Stages:**
    1. Front not COMPLETE (PREP / DETECT / CLIMBING on FL/FR):
       `target_pitch = climb_pitch_front_deg` = +15 deg (nose UP).
       Shifts CoM backward, UNLOADING the front wheels for easier pivot
       over step edge.
    2. Front COMPLETE, back still active:
       `target_pitch = climb_pitch_back_deg` = -5 deg (nose DOWN).
       Shifts CoM forward; back wheels lighter -> easier roll forward to
       contact step, easier pivot during CLIMBING.
    3. All COMPLETE -> 0 (level).
  **Smooth transitions:** the +15 -> -5 setpoint swing (20 deg) is fed
  through `climb_pitch_lpf_alpha` = 0.004 LPF (cutoff ~0.3 Hz). At 500 Hz
  that's tau ~ 0.5 s -> a step settles to 90% in ~1.2 s. Passenger feels
  a gentle ease from each attitude into the next, not a jolt. The body
  PID (with A4ii corrected 1/sin removed + A4kk softened gains + A4jj
  flat PD + A4ss stiff Kp=200 during DETECT) tracks the smoothly slewing
  setpoint without re-introducing the resonant shake.
  **Sign fix:** the previous code had the signs INVERTED -- front_active
  drove pitch DOWN (loaded the front wheels, making climb harder); back
  drove pitch UP (loaded the back). A4tt fixes both.
  **Tunable via dbg_ctrl:** `climb_pitch_front_deg`, `climb_pitch_back_deg`,
  `climb_pitch_lpf_alpha`. Old `climb_pitch_bias` deprecated (still in
  struct for backward Ozone watch compatibility, unused by code).
  **Plot:** `dbg_climb_plot.pitch_setpoint_raw_deg` shows the phase-logic
  output; `pitch_setpoint_filt_deg` shows what the PID actually targets;
  `pitch_fb_deg` shows what the chassis is doing. Trio plot to see the
  transition character.

- **A4ss — Phase-dependent Kp/Kd for climbing direct-control.** User
  observed during HW test: pressing wheel against step did NOT trigger
  DETECT -> instead the leg arm swung backward and the chassis pitched
  forward. Root cause: Kp=40 (A4kk soft-feel choice) is too soft for the
  DETECT phase, where the leg needs to RIGIDLY hold position so that any
  horizontal wheel-step contact force translates into motor torque (which
  spikes the residual signal) rather than into position yield (which gates
  off detection via `detect_settle_omega < 0.4 rad/s`). At Kp=40, 30 deg
  of yield is needed before the leg generates ~13 Nm of resistance; during
  that yield the detection is gated off (`vel > 0.4`), so the spike never
  triggers DETECT->CLIMBING.
  **Fix:** each phase gets its own Kp/Kd:
    - PREP     -> 40 / 2   (soft, smoothstep lift, gentle on passenger)
    - DETECT   -> 200 / 5  (RIGID hold for contact detection)
    - CLIMBING -> 80 / 4   (firm trajectory tracking)
    - COMPLETE -> 80 / 4   (firm end-pose hold)
  At Kp=200, 1 deg yield = 3.5 Nm, 2 deg = 7 Nm -- leg stays within ~2 deg
  of target while delivering full reaction torque, so residual spikes
  cleanly AND leg velocity stays below settle_omega gate. Also addresses
  A4nn-class observations indirectly: less yield under load = less PID
  fight = better stability.

- **A4rr — Residual peak-hold for threshold tuning.** User: "有没有变量
  是看残差的，我可以清晰看到多少是峰值 多少应该设为触发值". Added
  `residual_peak_nm[4]` to DbgClimbPlot: per-leg max of |residual - baseline|
  since the last per-leg phase transition. Resets every time the leg
  changes climbing phase, so each segment (PREP / DETECT / CLIMBING /
  COMPLETE) reads a fresh peak. Also mirrored `torque_res_threshold_nm`
  into the plot struct for overlay comparison. **Tuning procedure:**
    1. Enter CLIMBING, press X -> wait for DETECT.
    2. Read `residual_peak_nm[i]` during DETECT before any step contact ->
       this is the NOISE FLOOR (typical 0.5 - 1.5 Nm).
    3. Push joystick forward, wheel hits step -> read peak again -> this
       is the STEP CONTACT magnitude (typical 5 - 10 Nm under rider).
    4. Set `dbg_ctrl.torque_res_threshold` to ~ noise + 0.5*(peak - noise),
       e.g. floor=1, peak=8 -> threshold = 4.5 Nm.

- **A4qq — PREP-phase live mass measurement.** User insight: "可以在 prep
  阶段就知道 mass 知道空载还是载人". During the PREP smoothstep, all four
  legs sweep through theta_motor ~ 90 deg where sin(theta) ~ 1. The motor
  torque feedback at that angle satisfies the static balance equation
  `tau_motor = (M/4 + LEG_MASS) * g * r * sin(theta) + small_dynamics` --
  inverting gives a direct measurement of the load on top of the chassis,
  BEFORE accurate FFW is needed for DETECT / CLIMBING. **Implementation:**
    - State in Chassis: `prep_mass_load_sum_[4]`, `prep_mass_count_[4]`,
      `climb_mass_estimated_`, `climb_mass_estimate_kg_`. Reset on
      `Set_Mode(CLIMBING)`.
    - Per-tick sampling in handleClimbingMode after `climbing_.update()`:
      for each leg in PREP with `|sin(theta_motor)| > 0.7` (theta_motor in
      [44, 136] deg), accumulate `tau_fb / (g*r*sin(theta_motor))`. signed
      ratio gives consistent +m_per_leg regardless of climb_sign.
    - Finalize when min sample count >= 50 (~ 0.1 s in the sin>0.7 window
      during a 3 s PREP). Compute M_sprung = sum(per_leg_avg) - 4*LEG_MASS,
      seedMass impedance with this value (so future COMFORT entry continues
      from the measured mass).
  **getAdaptiveLoadPerLeg() priority updated:** (1) PREP estimate when
  finalized; (2) impedance.M_est warmed from COMFORT; (3) empty-chassis
  fallback 23 kg if cold. The fallback was `73 kg` (loaded default) before
  A4qq -- changed to 23 kg because under-shoot on a loaded cold start
  (~12 deg sag, briefly until PREP estimator finalizes) is safer than
  over-shoot on an empty cold start (which used to push theta_motor past
  the +/-180 seam -> potential multi-turn unwind). Loaded cold-start is
  also fully recovered by the PREP estimator within ~0.5 s.
  **DbgClimbPlot extension:** `prep_mass_estimate_kg`, `prep_mass_estimated`,
  `prep_mass_sample_count[4]` -- plot to watch live convergence.

- **A4oo — Self-adaptive gravity comp (rider-weight-aware residual & FFW).**
  User: "这个 grav 得自适应上面乘客的质量". The old `Get_LegGravityTorque()`
  used `-LEG_MASS_kg * g * r * cos(theta)` -- two bugs:
    1. Compensated only 4 kg (leg+wheel assembly); ignored the ~70 kg of
       chassis + rider that the leg also has to hold.
    2. Used `cos(theta)` formula (phase-shifted 90 deg from the correct
       eccentric-leg geometry `sin(theta)`).
  Residual computation `tau_fb - g_comp` therefore had a large mass- and
  angle-dependent offset that the LPF baseline absorbed. This made
  detection sensitive to anything that shifted the offset: rider movement,
  pitch_setpoint changes, mass shifts -> baseline-LPF lag -> false residual
  spikes -> false CLIMBING triggers.
  **Fix:** new `Chassis::getAdaptiveLoadPerLeg()` returns
  `(M_est/4 + LEG_MASS)` where `M_est = impedance_.getEstimatedMass()`
  (self-calibrates in COMFORT RUN via static balance from per-leg torque).
  Fallback: if `M_est < 10 kg` (cold start, never warmed in COMFORT) use
  `COMFORT_HOMING_CHASSIS_MASS_GUESS = 73 kg`. Used in:
    - ALL-modes torque residual (`Update()` dbg_torque block)
    - `handleClimbingMode` `tres[]` (passed to Climbing_Dynamics DETECT)
    - `handleClimbingMode` ACTIVE direct-control `ffw_climb`
  Gravity formula corrected to `+load * g * r * sin(theta_motor)` (physical
  truth for eccentric leg). Now in steady state: `tau_fb` (motor reports
  what we commanded ~= ffw_climb) and `grav_static` (computed from same
  load + sin formula) cancel -> `residual ~= 0` regardless of rider
  weight. Spikes only when something disturbs the leg (wheel hitting step
  edge etc).

- **A4pp — DbgClimbPlot: plot-friendly aggregated climbing signals.** User
  asked to organize observation/plot values into climb dbg. New struct
  `DbgClimbPlot dbg_climb_plot` in [Chassis.hpp](Applications/Chassis.hpp)
  groups everything Ozone needs for time-series plotting during a climb:
    - Per-leg: phase, target_theta_deg, angle_fb_deg, dtheta_pid_deg,
      target_omega_rad, actual_omega_rad, ffw_grav_nm, tau_fb_nm,
      residual_nm, residual_baseline_nm, residual_dev_nm, beta_rad
    - Chassis: pitch_setpoint_raw_deg, pitch_setpoint_filt_deg,
      pitch_fb_deg, pitch_h_adj_m
    - Trajectory: beta0_rad, prep_ramp_progress
    - Adaptive grav: M_est_climb_kg, ffw_load_per_leg_kg
  Populated only in `handleClimbingMode` ACTIVE+kinematic block (other
  modes leave values stale -- intentional, since the struct is climbing-
  specific). Recommended Ozone plot pairs:
    - target_theta_deg[i] vs angle_fb_deg[i] -> trajectory tracking
    - target_omega_rad[i] vs actual_omega_rad[i] -> velocity tracking
    - ffw_grav_nm[i] vs tau_fb_nm[i] -> FFW health (should overlap)
    - residual_nm[i] -> direct disturbance signal (should be ~0 except
      around wheel-step contact spikes)
    - residual_dev_nm[i] vs torque_res_threshold -> detection margin
    - M_est_climb_kg over time -> mass self-calibration

- **A4mm — Gate wheel compensation by climbing phase (no comp during
  CLIMBING / DETECT / COMPLETE).** User insight: "在这时就不能加 wheel
  compansation 了 不然就会和轨迹冲突". A4gg's FF wheel comp was applied
  for ALL isDirectControl phases (PREP/DETECT/CLIMBING/COMPLETE). Correct
  for PREP (wheel rolls on flat ground while leg lifts chassis), wrong for
  CLIMBING. Theoretical analysis:
    - `Wheel_Compensation` formula `n_wheel = leg_rpm * (1 + r/R*cos(theta))
      * wheel_coupling_sign` is derived from rolling-without-slip on a flat
      ground -- leg arm rotation moves wheel center along an arc, wheel
      must spin to roll without slip.
    - The climbing trajectory model (Climbing_Dynamics) has the wheel
      PIVOTING around the step edge E: constraint `cos(theta) = (R+L-h-
      R*sin(beta))/L`, with the leg angular velocity `theta_dot =
      R*cos(beta)*beta_dot/(L*sin(theta))` derived for the pivot geometry
      -- the wheel center traces an arc around E, NOT around the leg
      motor, and the wheel itself doesn't need to roll for the kinematic
      constraint to hold.
    - Applying wheel comp during CLIMBING forces the wheel motor to spin
      at the rolling rate, which conflicts with the pivot geometry. The
      wheel either slips at E or back-drives reaction torque into the leg
      motor via the mechanical coupling -> trajectory tracking degrades.
  **Fix:** explicit phase gating ([Chassis.cpp:1958-1990](Applications/Chassis.cpp#L1958-L1990)):
    - PREP -> FF wheel comp (rolling on flat ground; same as before).
    - DETECT / CLIMBING / COMPLETE -> NO wheel comp added.
    - IDLE / manual_climb -> FB wheel comp (normal joystick sweep).
  DETECT and COMPLETE had target_omega=0 so FF comp was mathematically 0
  anyway; CLIMBING was the real conflict. Now the trajectory math is the
  sole authority for leg motion during CLIMBING, wheel motor only follows
  joystick (the user assistive push to climb).

- **A4ll — Remove auto wheel-rpm boost during CLIMBING / COMPLETE.** User:
  "目前还保留了之前一旦有complete的wheel leg就会疯狂提速，目前换了新电机
  扭矩大了很多，所以不用再疯狂加速了，反而是保障我们KD V 可以让电机去到
  最大输出". Old code (band-aid for under-powered old motors):
  ```cpp
  if (any_leg.phase == CLIMBING || COMPLETE)
      climb_base_rpm = climb_omega * (60/2pi) * climb_wheel_scale;
  if (vx > 0.5) vx += climb_base_rpm;
  ```
  Caused the chassis to suddenly accelerate the moment ONE leg flipped to
  COMPLETE, even with no joystick. With new higher-torque motors, no need
  for this -- joystick + FF wheel comp (A4gg) keeps the wheels rolling in
  sync with leg motion, and the leg PD's Kd*V can pump arbitrarily large
  torque (up to tau_max=200) without the wheel boost helping it crawl.
  Removed the boost entirely; `dbg_climb.wheel_rpm` left as 0 for now
  (debug field used in Ozone watches).

- **A4kk — Soften climb ACTIVE Kp/Kd for compliant feel.** After A4jj
  killed the shake, user reported "按下X后电机响应很僵硬" -- response is
  too rigid. Initial gains Kp=80, Kd=4 made Kp*err produce hard torque
  reactions to small position residuals at PREP entry, giving a "snappy"
  feel inappropriate for a wheelchair lift. **Fix:** Kp 80 -> 40, Kd 4 -> 2.
  Damping ratio drops 0.68 -> 0.48 (slightly underdamped, feels lively but
  not bouncy). Natural freq 4.3 Hz -> 3.0 Hz under loaded sprung-mass --
  still ~2x above body-PID LPF (1.6 Hz), no resonance risk. FFW (~15 Nm
  static at theta=90) still carries the rider; Kp=40 contributes only
  3.5 Nm correction torque per 5 deg of position error, a true MICRO-
  spring on top of FFW rather than the primary support. If the leg sags
  visibly under heavier-than-expected rider (FFW underestimate > 5%),
  bump Kp back to 60 -- but expect rigid feel to return.

- **A4jj — Switch climbing direct-control from cascade to FLAT PD; tighten
  dtheta clamp.** A4ii reduced the body-PID gain by 5.5x but the shake
  persisted. User restated the desired control architecture: "ffw 任何时候
  平衡上面人的重量，KD V 限制速度，KP P 来稳定，pid 平衡微调这个 P 来做
  到平衡". This is exactly `Set_Leg_PD_Torque` -- a flat PD-as-torque law,
  same as COMFORT / HOMING_IN/OUT / FREE all use. Climbing was the lone
  holdout still using the cascade `Set_Leg_Torque_Track` (`kp_pos -> omega
  clamp -> kd_vel`). Why the cascade was unstable here even after A4ii:
    - Cascade outer-loop bandwidth = kp_pos = 8 rad/s ~= 1.3 Hz
    - Body-PID LPF bandwidth (pitch) = 1.6 Hz
    - The two loops are in the SAME frequency band -- the body-PID's
      dtheta output acts as a position disturbance into the cascade, and
      the cascade's slow position response phase-lags into the body-PID
      ~90 deg around 1.5 Hz -> positive-feedback resonance.
  **Fix:** replace `Set_Leg_Torque_Track` with `Set_Leg_PD_Torque(angle_cmd,
  omega_ff, Kp=80, Kd=4, ffw, tau_max=200)`. The flat PD has
  natural frequency ~4 Hz under loaded sprung-mass inertia -- well above
  the body-PID's 1.6 Hz, so no resonance. The control law also matches
  the user's mental model exactly:
    - `ffw`     = `m_per_leg * g * r * sin(theta_smoothstep)` -- gravity balance
    - `Kp * P`  = position spring against disturbances
    - `Kd * V`  = velocity damping / soft speed cap
    - `omega_ff`= matched smoothstep derivative so Kd doesn't fight intended motion
  Also tightened the body-PID dtheta clamp +/-10 -> +/-5 deg. The 5 deg
  clamp is well inside the 15 deg new_theta-boundary headroom on both
  sides -> impossible to hit the clamp -> asymmetric-saturation
  relaxation-oscillator pattern cannot form regardless of body-PID
  behavior. At Kp=80, a 5 deg position error -> 7 Nm correction torque,
  an order of magnitude less than the ~15 Nm gravity FFW -- PID leveling
  is a true MICRO-adjustment on top of FFW's static balance, never a
  competing controller. The cascade machinery in Climbing_Dynamics
  (climb_pos_kp / climb_omega_max / climb_kd_vel / climb_tau_max) stays
  in the config struct for now but is no longer driven by the ACTIVE
  branch (HOMING_IN/OUT already used flat PD per A4dd/A4ee).

- **A4ii — Kill the 1/sin geometric amplification in climbing direct-control
  body PID (root-cause fix for "CLIMBING -> PREP 抖个不停").** User
  observed continuous shake during PREP and asked for an analysis. Trace
  showed a positive-feedback body-PID loop with ~5.5x the gain of COMFORT,
  caused by:
    1. Missing `lev_scale` (0.25) and `BODY_PID_SCALE_PITCH` (0.7) in the
       direct-control branch (both applied in COMFORT and in the non-direct
       branch).
    2. Kinematic 1/sin(theta) blow-up at PREP angles: dtheta = dH/(r*sin)
       has sin(165 deg) = 0.26 -> 3.86x geometric amplification.
    3. Asymmetric saturation against the `new_theta` [15, 165] clamp:
       PID demanding +dtheta hits the upper bound (no effect); demanding
       -dtheta lets the leg drop 50 deg (the saturating clamp value). One
       polarity drops front legs, the other drops back legs ->
       relaxation-oscillator behavior.
    4. PREP -> DETECT requires leg within 3 deg of prep angle; oscillating
       leg never settles -> stuck in PREP indefinitely, shake continues.
    5. FFW = load*g*r*sin(angle_cmd) used the PID-leveled angle, so the
       gravity FFW step-changed (3.2 <-> 11.3 Nm) as PID swung -> step-change
       leg torque -> chassis moves -> IMU sees motion -> PID re-swings.
  **Elegant fix (single conceptual change):** replace the kinematically
  exact `dtheta = dH / (r * sin(theta))` transform with the constant-gain
  `dtheta = dH / r`. This acknowledges the geometric truth: near PREP the
  leg has reduced authority to correct chassis tilt, and asking it to
  rotate 250 deg to deliver an 8 cm height adjustment is physically
  nonsensical. The constant `1/r` happens to match what COMFORT delivers
  at theta=90 (where sin=1), so the gain feels natural across modes.
  Concrete changes ([Chassis.cpp:1762-1788](Applications/Chassis.cpp#L1762-L1788)):
    - `dh = lev_scale * BODY_PID_SCALE_PITCH * lev_signs[i][0] * pitch_h_adj
      * ramp_progress` (effective authority matches COMFORT/non-direct: x0.175).
    - `dtheta_deg = dh / r_m * (180/pi)` (no 1/sin).
    - dtheta clamp tightened: ±50 deg -> ±10 deg (well below the 15 deg
      new_theta-clamp headroom on either side; no saturation against
      boundary, so the relaxation-oscillator pattern cannot form).
    - FFW signed by `climb_sign[i] * theta_unsigned` not `angle_cmd` -- the
      gravity FFW tracks the smoothstep target only, decoupled from PID
      swings. No more 3.5x FFW step disturbance.

- **A4hh — FF wheel comp also for ES descent and COMFORT HOMING.** User
  followed up "我发现所有模式切换中都没有加wheel compansation，导致leg需要
  升起时候抵抗wheel". Audit confirmed: `handleEnergySaving` (legs sweep from
  any prior angle -> 0 deg) and `handleComfortMode` HOMING (legs sweep 0 ->
  +/-60 deg) both intentionally skipped `Add_Wheel_Compensation` (old
  comments rationalized this for the FB variant: "comp prop to leg_rpm
  would drive front/back wheels apart"). That reasoning is invalid for the
  FF variant added in A4gg -- the FF variant takes the commanded omega
  per leg, which is by construction self-consistent across the four legs
  (each leg's wheel rolls with its own commanded leg motion). **Fix:**
    - ES descent: per-leg `omega_ff_rad_wcmp = -start * ds_dalpha * pi/180
      / ES_DURATION_S` (same expression used for the leg PD's omega_ff).
    - COMFORT HOMING: added `ds_dalpha = 6*alpha*(1-alpha)` next to the
      existing `s` smoothstep, then `omega_ff_rad_wcmp = (target_deg_w -
      homing_start_angle[i]) * ds_dalpha * pi/180 / DURATION_S`. Each leg
      uses its own (target - start) delta -> per-leg comp magnitude scales
      with how far that leg has to move.
  Net: all five mode-transition phases (ES descent, COMFORT HOMING,
  CLIMB HOMING_IN, CLIMB HOMING_OUT, CLIMB ACTIVE PREP) now use FF wheel
  comp from the commanded leg omega. Steady-state holds (COMFORT HOLD,
  WAIT_START, CLIMBING after PREP completes) naturally get zero comp
  because ds_dalpha or commanded omega is zero -- no spurious comp.

- **A4gg — Feedforward wheel compensation for climbing (PREP wheel scrub).**
  User reported "climb 进入prep操作leg的时候没有add compansation，导致leg
  起来，轮子和地面摩擦" -- wheel scrubs the ground during PREP lift. Audit:
  `Add_Wheel_Compensation` *was* being called (ACTIVE branch + HOMING_IN/OUT),
  but `Wheel_Compensation()` reads `leg_motor->getRPMFeedback()` -- the
  MEASURED leg RPM. During PREP startup the leg accelerates faster than the
  encoder/CAN-poll feedback can track, so the wheel motor's velocity loop
  lags the leg -> wheel doesn't roll fast enough -> ground scrubs. **Fix:**
  added a feedforward overload `Wheel_Compensation(float leg_omega_radps)`
  that takes the *commanded* leg omega and applies the same `(1 + r/R *
  cos(theta_actual))` geometric factor and `wheel_coupling_sign`. Used in:
    - **CLIMBING ACTIVE direct-control legs:** seeded with `climb_sign[i] *
      climbing_.getTargetOmega(i)` -- the trajectory's own commanded omega
      from `Climbing_Dynamics::update()` (PREP smoothstep derivative, or
      CLIMBING `theta_dot`).
    - **HOMING_IN / HOMING_OUT:** seeded with the local smoothstep
      derivative `-start * ds_dalpha * pi/180 / DURATION_S` -- same value
      used for the leg PD's omega_ff.
  Non-climbing legs (manual_climb trigger sweep, or any path where the
  caller doesn't have a commanded omega handy) keep the FB variant -- the
  joystick-driven trigger ramp is slow enough that feedback lag is
  negligible. Position-factor `cos(theta)` still reads the actual motor
  angle in the FF variant (instantaneous geometry, not the commanded angle)
  so the geometric coupling is correct even if the leg is lagging its
  commanded position under load.

- **A4dd / A4ee — Mode<->CLIMBING transitions made ES-style smooth.** User
  asked for "其它模式到climb这个切换 参考comfort 到es的平滑过度" and "从
  climb任意phase切换出去会经历归零下降，那个下降也可以参考comfort 到es的
  平滑过度". The CLIMBING `HOMING_IN` (entry) and `HOMING_OUT` (exit) branches
  used a single rough recipe: `Set_Leg_PD_Torque(start*(1-s), 0, 80, 4, 0, 30)`
  -- no matched omega_ff, no gravity FFW, fixed Kp/Kd, tau_max=30. Under load
  the PD had to do all the gravity work alone -> ~9 deg lag at theta=90 ->
  the leg "fell" the last few degrees as PD caught up at the end, feeling
  sudden/jerky. **Fix:** rewrite the branch to mirror `handleEnergySaving`
  exactly:
    - `target_now   = start * (1 - smoothstep)` (unchanged)
    - `omega_ff_rad = -start * ds/dalpha / DURATION_S * pi/180` (matched FFV)
    - `ffw_gravity  = (M_est/4 + LEG_MASS) * g * r * sin(motor_fb)`
    - `Kp/Kd        = ramp from per-leg `climb_homing_kp_start_[i]` -> KP_END`
    - `tau_max      = 200` (DM hardware ceiling)
  Per-leg `climb_homing_kp_start_[4]` / `kd_start_[4]` added to `Chassis.hpp`,
  populated in `Set_Mode` exactly like `energy_homing_kp_start_/kd_start_`:
    - Entry from COMFORT: seed with per-leg `exit_kp_/exit_kd_`
    - Entry from ES: seed with `(ENERGY_HOMING_KP/KD_END)` (continuous, no jump)
    - Entry from IDLE: seed soft (0, 0.5)
    - Exit (HOMING_OUT): seed with `(CLIMB_HOMING_KP_END, KD_END)` so the
      Kp/Kd ramp is a no-op -- only the angle ramp matters during exit.
  Result: HOMING_IN feels identical to COMFORT->ES (smooth, no impact); the
  descent on climb exit feels identical to ES descent.

- **A4ff — PREP under load reaches target: bump velocity-loop authority.** User
  reported "climb prep在有人负载情况下还是不够扭矩到达target" *after* A4cc
  already raised `climb_tau_max` 20 -> 200 Nm. This means the bottleneck
  wasn't the tau_max clamp -- it was the velocity-loop's intrinsic torque
  ceiling `kd_vel * (omega_max - omega_actual)`. With `omega_max=3`,
  `kd_vel=12`, max velocity-loop torque was only `12 * 3 = 36 Nm`. Under a
  ~73 kg loaded chassis the PD had to deliver mid-PREP acceleration on top of
  the static FFW, and 36 Nm wasn't enough -> leg lagged the smoothstep, never
  closed the position error before the ramp ended. **Fix:**
    - `climb_omega_max 3 -> 5 rad/s` -- more headroom for catch-up transients.
      The smoothstep target's own derivative peaks at ~1.5 rad/s during the
      3 s synchronized lift, so 5 rad/s is only reached during catch-up.
    - `climb_kd_vel 12 -> 18 Nm.s/rad` -- more torque per rad/s of velocity
      error. Combined with omega_max=5, velocity-loop ceiling is now ~90 Nm.
  Passenger safety unchanged: passenger never experiences > 1.5 rad/s steady
  lift speed because the smoothstep target itself enforces it. The bump only
  matters when the leg is BEHIND target under load.

- **A4cc — PREP under load: raised climb_tau_max 20 -> 200 Nm.** User
  reported "prep起来如果在负载情况下扭矩不够" -- with rider on board, the
  PREP lift stalled mid-ramp. Root cause: `Climbing_Dynamics::Config::
  climb_tau_max = 20 Nm` clamped the velocity-loop torque term
  (excl. gravity FFW). Gravity FFW handled the static hold, but the PD
  had no headroom to push the lift through friction + rider inertia ->
  `omega_cmd - omega` saturated, leg couldn't accelerate. **Fix:**
  `climb_tau_max 20 -> 200 Nm`, matching the DM hardware ceiling
  already used in COMFORT_TAU_MAX and HOMING_TAU_MAX. Safety NOT
  reduced: `climb_omega_max = 3 rad/s` still bounds leg SPEED for the
  passenger; this only unclamps the torque required to *reach* that
  speed. With kd_vel=12 and omega_max=3, the natural ceiling of the PD
  term is ~36 Nm without rider; under load it can transiently demand
  more during acceleration, which is what 200 Nm now permits.

- **A4q remains in effect.** Body PID during COMFORT HOMING (gated by
  smoothstep s) is still active and working as intended (user confirmed
  ES->COMFORT lift is smooth and ends level for off-centre rider).

- **POSSIBLE SECONDARY BUG (not yet fixed):** `Get_LegGravityTorque()` returns
  `-mgr·cos(θ)` but the eccentric leg's gravity torque physically is
  `-mgr·sin(θ)` (mass at horizontal offset `r·sin θ`). Correct comp should be
  `+mgr·sin(θ)`. The current cos formula is **90° phase-shifted**: over-
  compensates near θ ≈ 0/180, under-compensates near θ ≈ ±90. Symmetric across
  legs (cos is even), so does NOT explain FL-only slowness; but may explain a
  small global "lag" near the horizontal mid-point of the PREP trajectory.
  Fix candidate (defer until A3c is closed): change formula to
  `+LEG_MASS_kg * GRAVITY_g * r_meter * sinf(theta_rad)`.

- **A4a — DONE in code (needs HW validation): ES migrated to torque-track.**
  `handleEnergySaving()` collapsed: HOMING/RUN sub-states removed; legs now
  call `Set_Leg_Torque_Track(0, 0, ...)` every tick (target = motor-frame 0,
  ES folded pose). `NearestEquivalentTarget(0)` resolves to the nearest
  360-equivalent of 0 from the current continuous position, so transition
  into ES from any prior mode -- including CLIMBING leftovers with possible
  DM multi-turn accumulation -- takes the SHORT way to the equivalent low
  pose. This eliminates the **IDLE->ES wild-flail** root cause: the old
  position-control HOMING (`Set_Leg_Target` with `Pos_KP`) let the DM's
  internal multi-turn resolution unwind full turns; the new torque-only path
  (`Pos_KP = Vel_KD = 0`) gives the DM zero authority to unwind. Speed
  clamp (`climb_omega_max=3 rad/s`) keeps the lift gentle without an
  explicit smoothstep. Wheels follow joystick with NO `Wheel_Compensation`
  (it was the old "rear creep" risk during motion). Validate IDLE->ES is
  smooth from any prior pose, no flail, gentle lift to 0.

- **A4b — DONE (needs HW validation): COMFORT migrated.**
  - New `Wheel_Leg::Set_Leg_PD_Torque(target_deg, omega_ff, kp, kd, ffw, tau_max)`
    primitive: direct PD-as-torque, same `Kp/Kd` units as MIT (1:1 transfer of
    impedance gains), `NearestEquivalentTarget` for multi-turn safety, sends
    via FFW only (`Pos_KP = Vel_KD = 0`). Lives next to `Set_Leg_Torque_Track`
    -- they cover the two control regimes: PD-Torque for STIFF position
    holding (COMFORT impedance, HOMING ramps, FREE), Torque-Track for SPEED-
    LIMITED trajectory tracking (CLIMBING).
  - 5-arg `Wheel_Leg::Set_Leg_Height(h, v, kp, kd, ffw)` (the impedance
    overload, only used by COMFORT) re-implemented to call `Set_Leg_PD_Torque`
    internally. Same external interface, so `executeBodyControlImpedance()`
    in Chassis didn't have to change. The impedance gains from
    `Impedance_Controller::getLegOutput(i)` now flow directly into the
    software PD-torque -> motor sees Pos_KP = Vel_KD = 0.
  - `handleComfortMode` HOMING sub-state: replaced the per-tick
    `Set_Leg_Target(cmd_deg, 0, ffw_homing, kp_use, kd_use)` with
    `Set_Leg_PD_Torque(cmd_deg, 0, kp_use, kd_use, ffw_homing, 30 Nm)`.
    Smoothstep angle ramp and Kp/Kd ramp from previous mode's stiffness
    preserved -- only the underlying control path changed.
  - **ES->COMFORT M_est overshoot -> roll oscillation** is still a separate
    bug; address it AFTER the unification is hardware-validated.

- **A4c — DONE (needs HW validation): remaining sub-states migrated.**
  - `handleFreeControl`: legs now use `Set_Leg_PD_Torque(target, 0, 50, 1,
    gravity_ffw, 30)`. `folded_angle` reduced from 180 -> 175 to keep the
    trigger-driven target 5 deg off the +/-180 seam (same overshoot/wrap
    class of bugs as CLIMBING; 360-deg unlimited rotation means no
    mechanical stop to catch overshoot). Switched from `Set_Wheel_Leg` to
    `Set_Wheel_Target + Add_Wheel_Compensation + executeMotorCommands`.
  - `handleClimbingMode` HOMING_IN / HOMING_OUT / WAIT_START sub-states:
    replaced `Set_Leg_Target(cmd, 0, 0, kp_h, kd_h)` with
    `Set_Leg_PD_Torque(cmd, 0, kp_h, kd_h, 0, 30)`. Same Kp/Kd (80/4) and
    smoothstep ramp, just routed through the safe software path.
  - `handleDebugMode`: `Set_Leg_Target(..., 80, 4)` -> `Set_Leg_PD_Torque(..., 80, 4, 0, 30)`.
  - `manual_climb` (trigger-debug branch inside `handleClimbingMode`):
    same migration.
  - **IDLE** was already safe (`Set_Wheel_Leg(stop_params)` with state=IDLE
    sets `Leg_Kp=0, Leg_Kd=3` -> pure damping, no position resolution by
    the DM).
  - **executeBodyControl(cmd, mode_dh)** (the position-based leveling helper,
    not the impedance one) has no callers -> dead code, the 2-arg
    `Set_Leg_Height(h, v)` calls inside it are unreachable. Both will be
    deleted in A5.
  - End state: NO mode in the entire codebase commands the DM's internal
    position loop. The DM only ever sees `Pos_KP = Vel_KD = 0` + FFW torque
    -> the multi-turn unwind class of bugs is structurally eliminated.

- **A5 — TODO.** After all modes use the unified law: delete dead control paths
  — old `Set_Leg_Target` velocity/position usages that are now unused, the
  `HOMING_OUT`/`HOMING_IN`/`WAIT_START` return-to-0 band-aid logic if no longer
  needed for safety, `executeBodyControl()` (already dead), and any stale vars.

### Track B — code clarity (after A is stable, or interleaved carefully)

- **B1.** Consolidate ALL tuning vars into one place (`Tuning.hpp` or a single
  config struct): currently scattered across `DbgControl` (Ozone-tunable),
  `Impedance_Controller::Config`, `Climbing_Dynamics::Config`, and many Chassis
  `constexpr`.
- **B2.** Unify debug into one hierarchical struct; make **all** big global dbg
  refresh every cycle in **every** mode (consistency); mode-specific dbg as
  sub-structs; delete stale fields (e.g. `DbgClimbing` beta/tres/raw_theta once
  the control law no longer produces them).
- **B3.** Delete dead/duplicate functions & residual vars. Confirmed dead:
  `Chassis::executeBodyControl(cmd, mode_dh)` (no callers). Audit
  `handleDebugMode`, `handleFreeControl` usage.
- **B4.** Add a clear purpose header to each cpp/hpp; **rewrite
  `!Code_Struct.txt`** — it is badly stale (says M3508/GM6020/HT8115 + multi-stage
  PID; reality: HT8115 wheels + DM J10010L legs, MIT mode).

---

## 4. Key files & responsibilities (current)

- `Chassis_Task.cpp` — FreeRTOS 500 Hz loop entry.
- `Chassis.cpp/.hpp` — orchestrator + state machine + per-mode handlers + IK +
  body leveling + dbg structs. (Largest; main refactor target.)
- `Wheel_Leg.cpp/.hpp` — one wheel-leg unit; leg/wheel pipelines; **now hosts the
  continuous-frame + torque-track primitives (A1/A2).**
- `Climbing_Dynamics.cpp/.hpp` — per-leg climbing FSM (IDLE→PREP→DETECT→CLIMBING
  →COMPLETE); outputs unsigned target θ/ω; config holds climb torque gains.
- `Impedance_Controller.cpp/.hpp` — COMFORT variable impedance (Kp/Kd/FFW + mass
  estimate). See §5 caveat.
- `Ground_Contact.cpp/.hpp` — warp PI compensator (diagonal ΔH).
- `Controller.cpp/.hpp` — joystick → (Vx, Wz).
- `PC_Comm.cpp/.hpp` — UART to PC/ROS.
- `Robot_Config.cpp/.hpp` — motor instances, Wheel_Leg pairing, per-leg
  `leg_offset` / `bending_direction` / `wheel_coupling_sign`.
- `Robot_Params.hpp` — mechanical constants. `Helper.hpp` — `normalizeAngle`
  (wraps to (−π,π]), deg/rad. `Cust_Types.hpp` — `Chassis_State`, `MIT_Params`,
  `Wheel_Leg_Params`. `Comm_Msg.hpp` — `PC_Msg`(13B) / `Reachable_Msg`(40B).

---

## 5. Known separate bug (do NOT couple into climbing yet)

**ES→COMFORT mass-estimate overshoot → roll oscillation, poorly damped.**
Likely positive feedback: `M_est↑ → FFW↑ → support↑ → body moves → current
changes → M_est changes`, plus mistimed `seedMass`/ramp on transition; left/right
FFW asymmetry shows as roll. For now climbing uses **fixed** `Get_LegGravityTorque`
(no `M_est`). Fix this as its own task (read `Impedance_Controller.cpp` M_est
update + Chassis ES→COMFORT transition) before feeding `M_est` into any FFW.

---

## 6. Validation checklist (per phase, on hardware)

1. Compiles clean; flash succeeds (verify the binary actually updated — a
   byte-identical debug value across runs means stale flash or a hard limit).
2. No regression in untouched modes.
3. Migrated mode: correct direction, firm hold, no oscillation, no multi-turn
   unwind during operation OR on mode entry/exit.
4. Tune torque gains via config; keep gains conservative first.
