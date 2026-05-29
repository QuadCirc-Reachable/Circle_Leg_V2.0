# Unified Leg-Control Refactor — Plan & Memory

> Wheelchair project (passenger on board). **Safety-critical:** the leg motors
> allow unlimited 360° rotation, so the controller must NEVER let a leg unwind
> multiple turns (passenger bobs up/down = unsafe). This whole refactor exists
> to make multi-turn unwind *structurally impossible*, not just unlikely.

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
