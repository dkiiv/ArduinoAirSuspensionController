# Height Control Supervisor (HCS)

Autonomous ride-height holding, leak compensation and load compensation for **height-sensor mode**, designed
for a daily-driven car whose only knowledge of the world is four height sensors, four bag pressures, tank
pressure, valve/compressor state and "is a BLE client connected".

Code: `src/heightControl/` (core: `hcs_core.*`, ESP32 adapter: `heightControlSupervisor.*`, tunables:
`hcs_config.h`), hooks in `components/wheel.cpp`, `components/compressor.cpp`, `airSuspensionUtil.cpp`,
`bluetooth/ble.cpp`, `bluetooth/bp32.cpp`, `tasks/tasks.cpp`. PC scenario simulator: `eval/hcs_sim.cpp`.

Status: **compiles, 44/44 simulator checks pass, NOT yet run on the car.** Every threshold marked
*estimate* below must be confirmed with the shadow build before the active build is flown.

---

## 1. Assessment of the existing control layer (tesla branch @ `6f7d6b78`)

### Sound -- kept as is
- **The actuator** (`Wheel::goalRoutine` / `achieveFineGoal`): closed loop with a valve-closed true reading,
  fine-pulse landing with a shrinking burst on every goal crossing, a stall detector, and the
  "never wait at the barrier with a valve open" invariant (`docs/goal-sync-barrier.md`). This is
  car-tuned and is the part a rewrite would most likely make worse.
- Solenoids are driven LOW in the constructor (a crash -> reboot closes every valve).
- The BLE-ignition gate on the compressor (`compressor.cpp`: `!isVehicleOn()` -> off).
- The AI offset model is only used to estimate a *reading* during flow; it never chooses targets.

### Tangled
- Height-mode "maintain" is spread over four `Wheel` methods (`trackPressureStability`,
  `pressureCaptureBaseline`, `maintainPressure`, `heightsensorlessLevelling`) sharing `sl*` state with the
  pressure-mode sensorless leveller.
- Each wheel task makes *global* decisions with racy shared state (`isAnyWheelActive()` from four tasks).
- A transient `initPressureGoal` failure permanently disables maintain **in NVS** (`setmaintainPressure(false)`).

### Unsafe or wrong (by reading the code; line numbers are `wheel.cpp` unless noted)
| # | Finding | Where |
|---|---------|-------|
| U1 | **No driving/cornering detection in height mode.** Maintain fires whenever a corner has stayed within +-2 % for 5 s and is >= 5 % from its baseline. A long sweeper (on-ramp loop) satisfies that; the correction is **bidirectional** (`fabs`, `onlyAirUp = false`), so it fills the outside and dumps the inside corners mid-corner. The BLE gate is no help: the phone is connected while driving. | 616-625, 727 |
| U2 | **Fights the hill spot.** The baseline is captured once per valve-close event and is not re-taken after a drive, so arriving on the crown (diagonal +-8-10 %) fills the compressed corners and dumps the hanging ones, then re-baselines and repeats on the next disturbance. | 748-769, 616-625 |
| U3 | **No upper bound while filling in height mode.** `overCeiling` is pressure-mode only (the TODO). `initPressureGoal` accepts goals up to 103 % while the normalised reading clamps at 100, so a goal > 100 is unreachable and the valve stays open to the 15 s timeout. A dead / unplugged height sensor that reads low makes maintain fill blind for 15 s+ with no bag-pressure check. | 157, 514-516, 108-120 |
| U4 | **No sensor fault handling.** `readLevelSensorNormalized` clamps 0..100, hiding wire breaks (floating input drifts high) and shorts. | 112-124 |
| U5 | **Lift / jack.** With the car on a lift, every corner reads "too high" -> height maintain dumps all four bags (bidirectional). The car later lands on empty bags. | 616-625 |
| U6 | **Global baseline nullify couples the corners** (the root of the suspected bug, section 2). | 752-755 |
| U7 | `millis() > start + TIMEOUT` timeouts are wrong across the 49.7-day `millis()` wrap. The board is powered 24/7, so a routine running at the wrap can lose its timeout. *Legacy code; not changed in this PR.* HCS uses `now - start` everywhere. | 375, 466 |
| U8 | `loadProfileAirUp`: `profileIndex > MAX_PROFILE_COUNT` should be `>=` (index 5 reads `profile[5]` out of bounds). *Not changed in this PR.* | `airSuspensionUtil.cpp` 286 |
| U9 | Compressor has a "frozen" (no-progress) pause but **no maximum continuous run time**. | `compressor.cpp` |

## 2. Verdict on the suspected leak-below-min-ride bug: **CONFIRMED (conditional)**

Step by step, from the code:

1. `pressureCaptureBaseline()` (748): if **any** wheel is active it calls `nullifySensorlessBaseline()` for
   *this* wheel (752-755). So any actuation on any corner, plus every boot, leaves `slBaselineCaptured = false`.
2. Re-capture (759-767) requires valves closed + 5 s stable, and then **returns early when
   `heightCalMinRide > current height`** (761-764). A corner below min ride can never get a baseline.
3. `maintainPressure()` (602): with no baseline it goes to the `else` branch, which is gated
   `!getheightSensorMode()` (637). **In height mode that branch does nothing.**
4. Therefore a corner that is below `heightCalMinRide` *and* has lost its baseline is never lifted by
   maintain, no matter how long the owner waits. Only a manual preset load moves it.

When it bites (realistic triggers):
- a reboot (OTA, brownout, `REBOOT` packet) while a corner is below min ride;
- **two corners leaking overnight**: when the owner returns, the first corner's refill makes
  `isAnyWheelActive()` true, which nullifies the second corner's baseline. If that corner is below min ride,
  it is never recaptured and never lifted;
- any manual valve jog while a corner is below min ride.

When it does *not* bite: one corner leaks with no reboot and no other actuation. The old baseline survives
and maintain refills it once BLE connects. The car also has to have `heightCalMinRide` calibrated: it
defaults to 0, so uncalibrated cars never hit the refusal.

Other parts of the hypothesis:
- `MAINTAIN_PRESSURE_MIN_ACTIVATION_LEVEL` (1 %) only blocks baselines at or below 1 % height. It is not
  involved.
- `initPressureGoal` does **not** clamp goals near min ride. Its only bound is the > 103 % rejection.
  **Refuted.**

Related flaw found on the way: the baseline is the *measured* height, never the preset. After a reboot, a
leaked corner that is still above min ride becomes the new baseline and is never corrected back.

Fix in HCS:
- `BOTTOM_GUARD`: any corner below its floor is lifted, to `max(target, minRide + 3)`, regardless of
  baseline state.
- Targets and pressure references are persisted in NVS, so a reboot does not forget the preset.
- A manual commit re-baselines **only the corners the user actually moved**.

Simulator S3 replays the full scenario: leak below min ride with no BLE, reboot at 5 h, owner returns at
8 h. RD goes 28.9 -> 47.6 (routine timeout) -> completion pass -> 49.9.

## 3. Recommendation: re-architect the decision layer, keep the actuator

The defects in section 1 are not missing `if`s. The legacy layer has **no notion of vehicle activity**, it
makes **per-corner decisions that need whole-car context** (terrain vs. load can only be told apart across
corners), and it **conflates "baseline" with "target"**. Extending it would mean bolting a vehicle-state
machine onto four racing per-wheel tasks. Rewriting everything would throw away the tuned actuator.

So:
- a single **supervisor task** owns every autonomous decision;
- it commands the **existing** goal routine through a narrow API (`initAutonomousGoal` /
  `requestAutonomousAbort`);
- all hard bounds sit in one arbiter (in the core) **and** are re-checked inside the routine on every loop.

The core is platform independent so the logic can be replayed on a PC.

Feature flag: `HEIGHT_CONTROL_SUPERVISOR` (compile time, default **false**).
- `manifold_v4_release` is the legacy build, unchanged in behaviour.
- New envs, all extending `manifold_v4_release` (same pins, release flags and OTA name):
  - `manifold_v4_hcs_shadow`: decides and logs, **never actuates**; legacy maintain stays active.
  - `manifold_v4_diag`: active, plus a 1 Hz state dump and raw sensor voltages. Replaces the reverted diag
    env.
  - `manifold_v4_hcs`: active, event log only.
- Runtime enable inside an HCS build: height-sensor mode **and** the existing "maintain pressure" toggle.
  **No BLE protocol change**: firmware, controller and app are untouched.

## 4. Architecture

```
 wheel tasks (x4, existing)          supervisor task (new, 10 Hz)                   wheel tasks
 readInputs(): h, raw h, p, seq -->  hcs::Core::tick(Inputs)                  --> initAutonomousGoal()
 valve / routine flags         -->    1 ingest + fault checks                      goalRoutine (existing)
 tank, compressor, BLE (isVehicleOn)  2 motion detector -> episodes                + abort flag, autonomous
 preset / manual hooks ---------->    3 state machine                                timeout, height ceiling,
                                      4 classifier (per corner + total load)          height floor, bag psi
                                      5 arbiter: bounds, presence, budgets,           ceiling re-checked every
                                        dwell, reversal lock, leak latch              loop
                                      6 batch executor + watchdogs        --> requestAutonomousAbort()
                                    persist targets (NVS "hcs") / serial log
```

The supervisor does **no** extra I2C reads; it consumes the wheel tasks' cached samples, using a sequence
counter to reject stale samples.

**Corner indices are logical.** `FP, RP, FD, RD` = `WHEEL_*` 0..3. The supervisor never touches pins or ADC
channels. On this `tesla` branch the physical valve-block order differs from upstream:

- v4 connector slots are **FD, RD, RP, FP**;
- ADS A pressure channels follow the valve ports (ch0 FD, ch3 RD, ch2 RP, ch1 FP);
- ADS B height channels keep the upstream map (ch0 FP, ch3 RP, ch2 FD, ch1 RD).

The classifier pairs height *i* with pressure *i*, so it **depends on those maps agreeing per corner**.
This is guarded at runtime (section 8, MAPPING) and is the first on-car check in section 11.

## 5. State machine

```
            boot
             |  (10 s buffer fill; persisted targets restored)
             v
   +----> SETTLING ----(quiet >= Q)----> evaluate -----> PARKED <----------------+
   |         ^   \                         |   |            |                     |
   |  motion |    \ motion                 |   +--> CORRECTING --(done+6s)--------+
   |  ended  |     v                       |           |  abort: motion / BLE lost / manual /
   |         +-- MOTION (DISTURBED) -------+           |  fault / watchdog / mapping
   |              | episode > 15 s                     v
   |              v                                  (closes valves) -> MOTION / PARKED / FAULT / MANUAL
   +---------- MOTION (DRIVING)  -- sets "arrival" flag
  any state: user valve / preset / gamepad --> MANUAL --(idle 10 s + steady)--> COMMIT touched corners --> PARKED
  any state: >=2 height faults or MAPPING ---> FAULT (observe only)
  any state: not height mode / maintain off / safety mode ---> INACTIVE (observe only)
```

- **Q (quiet required before evaluating)** = 120 s after a drive or a boot (`HCS_ARRIVAL_QUIET_MS`), 8 s after
  a short disturbance (`HCS_DISTURB_QUIET_MS`). A 60 s red light never reaches an evaluation.
- While PARKED: re-evaluate every 30 s, on a BLE connect edge, after every batch, and after every motion
  episode ends.
- No BLE client: everything runs (motion tracking, classification, logs) **except actuation**. The decision
  is logged as `VETO (no BLE client)`.

## 6. Motion (driving / cornering) detector

There is no speed signal, so motion is inferred from signatures:

- **Activity per corner** = EMA (alpha 0.15, about 0.6 s) of |second difference| of height and of bag
  pressure. A second difference is blind to smooth ramps (our own fills) but sees road input, body bounce,
  wheel hop, and the onset/offset of roll and pitch.
- A corner votes "moving" if height activity > 0.7 % or pressure activity > 1.2 psi (*estimates*). Only
  corners that are idle (valves closed >= 3 s), healthy and have history get to vote.
- **Motion** = >= 2 corners voting, for 3 consecutive ticks to start an episode. An episode ends after 3 s of
  quiet. An episode longer than 15 s is **DRIVING**; shorter is a **disturbance** (door, person, trunk).
- If fewer than 2 corners are observable, the detector treats it as "not quiet" (fail-safe: no evaluation).
- During a correction it is more sensitive: 1 strongly active corner (1.5x threshold) or 2 active corners,
  held 300 ms, abort the batch.

**Why cornering cannot cause actuation even if the detector misses it** (defence in depth):
1. Roll / pitch / braking **redistribute** load at constant total. The classifier calls that `SHIFT` and
   *accepts* it, which by construction never actuates (simulator S2 "blind detector": a 5 min sustained
   0.18 roll with sensor-noise-only road gives 0 starts).
2. Any evaluation needs 120 s of continuous quiet after a drive and 2 s windows with < 1.5 % / 2 psi spread.
3. Any detected motion during a batch aborts it (S5: abort 500 ms after the car moves).
4. Batches are bounded (10 s routine, 30 s watchdog, 12 s valve watchdog, 8 batches per hour).

## 7. Classifier

Inputs per corner: settled height mean `h` and pressure mean `p` over the last 20 valve-closed samples;
target `tgt`; pressure reference `pRef` (pressure when the target was last confirmed); `pScale` (pressure at
the last commit, used only as a load normaliser).

**Total-load test.** `L = mean_i((p_i - pRef_i) / pScale_i)`.
- Dividing by `pScale` turns each dp into a fraction of that corner's static load, so the different front
  and rear bag areas cancel.
- Redistribution (crown, slope, cornering, braking) gives L ~ 0. People and cargo do not.
- `L > +1.5 %` is UP, `L < -1.5 %` is DOWN, otherwise NEUTRAL (`HCS_LOAD_FRAC`, *estimate*: about 28 kg).
- Needs >= 3 corners with valid pressure; otherwise UNKNOWN.

**EXTERNAL support.** Any corner losing more than 35 % of its pressure means a jack, a lift or an
unsupported wheel. Freeze all actuation and accept nothing (S8).

**Physics behind the per-corner rules** (quasi-static air spring): bag pressure carries the load (p ~ F / A),
air mass sets the height at that load.
- More load: height down, pressure up.
- Less air (leak, cooling): height down, pressure unchanged.
- **A corner can only be compressed by a force, and a force always raises its bag pressure.** That one fact
  separates terrain/load from air loss.

| Condition (e = h - tgt, dp = p - pRef) | Class | Action |
|---|---|---|
| h < floor (minRide, or 5 % if uncalibrated) | BOTTOM_GUARD | fill to max(tgt, minRide + 3) |
| h > ceiling + 3 (ceiling = 95 %) | CEILING | dump to ceiling (if auto-lower) |
| \|e\| <= 3 (deadband) | WITHIN | none; re-reference pRef if \|e\| <= 1.5 and no event open |
| e < -3, dp <= +1.5 psi | AIR_LOSS (leak, cooling) | fill to tgt |
| e < -3, dp > +1.5, L UP | LOAD | fill to tgt |
| e < -3, dp > +1.5, L NEUTRAL | SHIFT (terrain, crown, someone moved) | **accept**: tgt = h, pRef = p |
| e > +3, dp >= -1.5 | AIR_GAIN (thermal expansion) | hold. Never dump: it reverses as the bag cools, and dumping would just make the compressor refill it later |
| e > +3, dp < -1.5, L DOWN | UNLOAD | dump to tgt (if `HCS_AUTO_LOWER`) |
| e > +3, dp < -1.5, L NEUTRAL | SHIFT | accept |
| contradictory (e.g. low, dp up, L DOWN) or no pressure | AMBIGUOUS | nothing |
| no pRef but pressure valid | SHIFT | accept. Fail-safe: unknown reference, never pump |

**After a drive (arrival) only: preset anchor.**
- A non-terrain corner whose target is within 3 % of the active preset snaps back to the preset.
- A SHIFT corner snaps only if it is moving *toward* the preset, i.e. the car came back to flat ground.
- This stops the reference random-walking away from the preset over many trips, and never pulls a corner
  toward flat when the car has just parked on terrain.

**Event latch.**
- While any corner still wants a correction, pressure references are frozen, so the evidence for the event
  survives the first corrected corner.
- Simulator finding: re-referencing the first corrected LOAD corner erased the total-load rise, and the
  second loaded corner was then mis-read as SHIFT.

**Completion pass.** A batch that lands > 1 % short (routine timeout, tank) gets **one** follow-up with its
original class after the dwell.

Simulator findings that changed the design (all fixed, all covered by tests):
1. Normalising by the *current* pRef made a shift reversal (118/82 psi -> 100) look like +3.4 % load and
   pumped two corners. Fixed with `pScale`.
2. Re-referencing on batch completion erased load evidence. Fixed with the event latch.
3. Single noisy samples started episodes and aborted fills. Fixed with 3-tick persistence and the threshold
   change.

## 8. Arbiter, bounds, pacing

Every autonomous start must pass **all** of these gates:

| Gate | Value |
|---|---|
| BLE presence (`isVehicleOn()`) | required (R6) |
| enabled (height mode + maintain), not safety mode, no FAULT / MAPPING / EXTERNAL | required |
| corner healthy (height; plus pressure for fills) | required |
| fill target <= ceiling = 100 - 5 %; routine cut at ceiling + 1 on the live reading | hard |
| dump target >= minRide + 3 (or the user-held low preset) ; routine cut at floor - 1 | hard |
| bag psi < min(bagMaxPressure, `MAX_PRESSURE_SAFETY`) - 10, settled before / raw (conservative) during | hard |
| tank >= bag + 20 psi; refuse outright if bag + 20 > compressor cut-off | hard |
| per-corner dwell 60 s; **reversal lock** 10 min (no up-then-down without a new motion episode) | anti-chatter |
| 8 batches / h, 90 s autonomous fill / h | budget |
| leak latch: 4 refills of one corner in 6 h -> stop refilling it; one refill re-granted per BLE connect | compressor / battery |
| >= 2 healthy non-actuated corners to watch for motion | required |

**Batches**
- One batch = the axle holding the largest error, same direction, max 2 corners (FP+FD or RP+RD). Ties go
  by index order.
- Axle pairs keep the car level side to side, cap tank draw (no 4-corner fill starving the others), and
  leave 2 corners to watch.
- After the batch settles (6 s), everything is re-evaluated and the next axle goes.
- Autonomous fills are fill-only (`onlyAirUp`), dumps are dump-only (`onlyAirDown`, new). A correction never
  overshoots and then corrects back the other way.

**Watchdogs:** 10 s autonomous routine timeout (inside `goalRoutine`), 12 s continuous valve-open, 30 s batch.

**MAPPING guard:**
- After a batch: if a non-actuated corner moved more than 1.5x the actuated one (and >= 2 %), freeze
  globally until reboot.
- Live during a batch: an idle corner moving >= 3 % while the actuated corner moved < 1 % aborts at once
  (S12: a cross-wired RP/RD is stopped at +3.3 %).

**Compressor (HCS builds):**
- Max 8 min continuous run, then 10 min cooldown, via the existing freeze pause (the app's
  COMPRESSOR_FROZEN bit). A user's manual compressor command still overrides it.
- Small corrections draw from the tank. The existing 140/180 psi hysteresis means the compressor does not
  restart for every small correction.

## 9. Scenarios walked end to end

### 9.1 Hill spot (R4), including "3 a.m., slow leak, phone connected"
1. Driving home: DRIVING confirmed after 15 s of motion. Nothing actuates; the arrival flag is set.
2. Parking manoeuvre onto the crown. Motion stops; SETTLING for 120 s.
3. Arrival evaluation vs. the held targets (preset 50):
   - FP 41.9 (dp +15), RD 45.7 (dp +11), RP 66.3 (dp -15), FD 59.4 (dp -9); L ~ 0, NEUTRAL.
   - All four corners are **SHIFT**: accepted, targets = arrival geometry, pRef = arrival pressures.
   - No valve moves.
4. The bags cool over hours, so all corners sag slightly. Corners that pass 3 % are AIR_LOSS (pressure
   unchanged) and are refilled **to their arrival heights**. The compressed corners stay compressed.
5. 3 a.m.: RP (a hanging, low-pressure corner) leaks slowly. Once it is > 3 % below its arrival target with
   pressure unchanged, it is AIR_LOSS.
   - The phone is connected, so the refill is allowed (R6). It goes back to its **arrival** height, alone
     (rear axle, same direction).
   - Budgets and dwell apply. If it leaks fast (4 refills in 6 h), the latch stops it and logs it.
6. Simulator S1 (8 h): no valve activity while driving; no dump on the hanging corners; FP/RD still
   compressed (41.8 / 45.5); worst corner within 2.4 % of arrival; RP refilled 3x; no false leak latch.

Trade-off, stated plainly: refilling a leaking corner at 3 a.m. runs the compressor at 3 a.m. if the tank
needs it. There is no clock and no better presence signal. The alternative is letting the corner sit on its
bump stop until morning.

### 9.2 Cornering at speed (R5)
1. Road input keeps >= 2 corners active, so a motion episode runs. After 15 s it is DRIVING; nothing
   evaluates.
2. A long sweeper: the roll onset is a burst of second-difference activity, and road input continues.
   Still DRIVING.
3. Worst case, a perfectly smooth sustained corner with the detector blind:
   - an evaluation would also need 120 s of quiet *after* the last motion, plus 2 s steady windows;
   - even then, outside corners (compressed, dp up) and inside corners (extended, dp down) with L ~ 0 are
     SHIFT and are accepted, not corrected;
   - when the roll unwinds, the reversal is SHIFT again (L ~ 0 thanks to `pScale`);
   - zero actuation (S2, both variants).
4. If a correction had started at a standstill and the car drives off: abort on the watching corners within
   300 ms of sustained activity, valves closed (S5: 500 ms).

Residual risk: a car creeping at walking pace on glass-smooth pavement with no pitch on take-off might not
look like motion. Anything that does actuate is still bounded: only air loss, load, floor or ceiling;
one axle; 10 s.

### 9.3 Leak below min ride overnight, owner returns (the suspected bug)
1. Evening: preset committed (target 50, pRef ~100). The owner leaves; PRESENCE off.
2. RD leaks overnight. Evaluations keep classifying it (AIR_LOSS, then BOTTOM_GUARD once below min ride 35)
   and log `VETO (no BLE client)`, throttled.
3. 05:00: the manifold reboots (OTA, brownout). Targets, pRef and preset are restored from NVS.
   `bootPending` means the long quiet is required.
4. 08:00: the phone connects (PRESENCE edge), the door opens, the driver sits.
   - That is a disturbance: 8 s quiet, then evaluate. L UP (driver), so FD is LOAD; RD is BOTTOM_GUARD
     (28.9 < 35).
   - The largest error goes first: RD is filled toward 50. The 10 s routine timeout stops it at 47.6 (2.4
     short), which schedules a completion pass.
   - FD LOAD next (43.6 -> 49.7), then the RD completion (47.5 -> 49.8).
5. If nothing was persisted (first boot), BOTTOM_GUARD still lifts RD to minRide + 3 (S3 variant: 37.9).

### 9.4 Load (R3)
- Two rear passengers: disturbance, then 8 s quiet. L +4 % (UP); RP/RD are LOAD. Rear axle batch back to
  50 +- 0.2.
- Groceries of 10 kg: inside the deadband, nothing happens.
- Passengers leave after more than 10 min: L DOWN, so UNLOAD, and the rear is lowered back (S4).

## 10. Faults and safe states

| Fault | Detection | Effect |
|---|---|---|
| Height wire break / short | raw < -4 % or > 104 % of the 0.5-4.5 V span, 10 samples | corner frozen; cleared after 5 min good |
| Height implausible | raw > 15 % outside the calibrated travel | corner frozen |
| Calibration degenerate | \|calMax - calMin\| < 5 | corner frozen until recalibrated |
| Height stuck | silent (< 0.05) for 60 s while >= 2 other corners show road motion | corner frozen; clears when it moves |
| Height does not follow the bag | batch moved >= 3 psi but < 0.5 % height | corner frozen until the next preset / manual commit |
| Valve / sensor mapping mismatch | section 8 | **global freeze until reboot** |
| Bag pressure out of range | < -5 or > 240 psi | no fills on that corner; excluded from L |
| Tank sensor invalid | out of range | no fills |
| >= 2 height faults | | FAULT: observe only |
| EXTERNAL support (jack / lift) | any bag loses > 35 % of its pressure | observe only until it returns |

| Event | Safe state |
|---|---|
| BLE lost mid-correction | abort, valves closed, car stays where it is; re-evaluated on reconnect |
| Car moves mid-correction | abort within ~300-500 ms |
| User touches anything | abort; MANUAL yields; re-baselines only the touched corners |
| Firmware crash / watchdog reset mid-fill | solenoids default LOW on boot (existing); persisted targets restored; long quiet before acting |
| Power loss mid-fill | valves de-energise closed (hardware); same as above on boot |
| Wheel task hangs with a valve open | supervisor valve watchdog (12 s) closes the valves *from its own task* (autonomous routines only) |
| OTA failure | OTA path untouched; the supervisor is not started in update mode; a bad NVS blob is rejected (magic, version, range) |
| OTA *success* from the official worker | replaces this build with upstream `manifold_v4` (legacy behaviour). The `hcs` NVS namespace is ignored |

Not covered: there is no hardware task watchdog on the wheel tasks. A hang during a *user* routine is
unchanged legacy behaviour.

## 11. Rollout and on-vehicle test plan (falsifiable)

Build envs. Reproduce release builds with
`version_num=test release_tag_name=build-local pio run -e <env>`.

**Step 0: mapping check (do first, shadow build).** With the car parked and still, jog each corner
alone with the app's valve buttons. In the `HCSD` dump, the jogged corner's `h` **and** `p` must change
the most.
- Fail if any corner's `h` and `p` respond to different valves. In that case fix the ADS B / ADS A maps
  before going further.
- On this branch the pressure channels were re-mapped and the height channels were not, so this is the
  single most important check.

**Step 1: shadow** (`manifold_v4_hcs_shadow`, 2+ days of normal use). Record the parked noise floor
(`aH`/`aP` in `HCSD`), driving activity, and every `SHADOW would START`.
- Tune `HCS_MOTION_*` so that parked activity sits < 1/2 the threshold and driving > 2x.

| Req | Test | Pass | Fail (falsifies) |
|---|---|---|---|
| R1 | Load preset, park on level ground 12 h with phone in range | all corners within +-3 % of preset at the end; every correction logged with class | a corner > 4 % off with no logged refusal, or any dump while unattended |
| R2 | Crack a corner's bleed (or use a known slow leaker) for a measured -6 % | one AIR_LOSS batch back to target +-1 %, logged; leak latch only if refills are >= 4 in 6 h | no correction within 60 s of crossing 3 %, or correction on another corner |
| R2b | Repeat with the corner forced below `heightCalMinRide`, then reboot the manifold, then connect BLE | BOTTOM_GUARD lifts it to the preset (completion pass if needed) | not lifted (the legacy bug) |
| R3 | Two adults into the rear seats; 10 kg in the trunk | rear back to preset +-1.5 % within 60 s; the 10 kg causes no action | front corners actuated, or > 1 batch per axle |
| R4 | Drive home, park on the hill spot, stay connected 8 h (overnight) | log shows SHIFT on arrival; **no** fill of FR/RL-compressed corners toward flat and **no** dump of the hanging ones; final heights within 3 % of arrival | any batch whose direction reduces the diagonal |
| R5 | 30 min drive: on-ramp loops, hard braking, highway, red lights | zero `START` while moving; `DRIVING confirmed` within 15-20 s of pulling away | any START with the car moving |
| R5b | Load the trunk, then drive off as soon as `START` logs | `ABORT ... motion` within 1 s, valves closed | valve open > 1 s after motion |
| R6 | Disconnect the phone (out of range), load the trunk | `VETO (no BLE client)` logged, nothing moves; reconnect -> corrected | any valve or compressor activity without a client |
| F1 | Unplug one height sensor connector (parked) | `FAULT .. out of 0.5-4.5V band` within 1 s; that corner never actuated | any actuation of that corner |
| F2 | Car on a jack at one corner (wheel off the ground) | `EXTERNAL` logged, zero actuation | any dump / fill while jacked |

## 12. Challenge and confidence

| Claim | Confidence | Why / what would falsify it |
|---|---|---|
| Legacy bug analysis (section 2) | high | pure code reading, reproduced in the sim |
| Terrain / cornering cannot trigger actuation via the classifier | medium-high | relies on "force raises pressure" and on L ~ 0 for redistribution. Front/rear corner loads differ (about 47/53) -> about 0.7 % error vs the 1.5 % threshold. Falsified if the car's bags show large hysteresis or effective-area change with height (pressure not following load) |
| Motion thresholds | **low until measured** | sim noise is assumed; the real ADS noise and road activity are unknown -> shadow mode |
| Load threshold 1.5 % / pressure noise 1.5 psi | medium | real bag hysteresis and temperature could exceed it -> shadow logs |
| Hill spot is held | medium-high | the sim models the crown as pure load redistribution. A real crown also moves the wheels geometrically; the classifier still sees compressed = pressure up |
| Slow creep at walking pace is detected | **low** | defended by the long quiet, abort on motion, SHIFT acceptance and budgets, not by the detector |
| Persisted targets survive OTA / reboot | high | NVS blob, versioned and range-checked; same mechanism as the calibration |
| Compressor cap does not hurt normal use | medium | 8 min continuous is far above any normal refill; the user can override |

The 3 a.m. question (hill spot, slow leak, phone connected) is answered in 9.1:
- the car holds its **arrival** geometry;
- the leaking corner alone is refilled to its arrival height;
- the compressor may run at night;
- a fast leak stops after 4 refills and is logged.

Without the car I could not verify: real noise floors; valve flow rates (and so whether 10 s routines
suffice); bag hysteresis; the sign/magnitude of dp for this car's bags on the crown; BLE presence behaviour
with the phone in the house; the corner mapping on this branch.

## 13. Observability

Every decision is one serial line prefixed `HCS t=<ms>`:
- `MOTION start/end`, `DRIVING confirmed`;
- `EVAL <BOOT|ARRIVAL|EVENT|PERIODIC> load=.. (UP|DOWN|NEUTRAL)` followed by one line per non-WITHIN corner
  (class, h, tgt old->new, e, dp);
- `REFUSE <corner> <class>: <gate>`, `VETO (<reason>)`, `SHADOW would START`;
- `START`, `RESULT`, `ABORT batch: <why>`;
- `FAULT ...`, `EXTERNAL`, `LEAK`, `COMMIT`, `PRESENCE on/off`, `RESTORED`.

Repeated identical lines (periodic re-evaluation, vetoes, refusals) are throttled to once per 10 min.

Diag builds add, every second:
- `HCSD ...`: state, quiet time, voting corners, tank, budget, and per corner h, p, tgt, pRef, activity,
  fault bits, last class;
- `HRAW ...`: raw voltages and calibration points, as in the reverted diag env.

## 14. Simulator

`eval/hcs_sim.cpp` compiles the production core against a quasi-static plant:
- `h = 50 + 60 (mT/F - 1)`, `p = 100 F`;
- extension limit at 100, bump stop at 0;
- road noise, roll / pitch / warp load transfer, leak, thermal, jack, cross-wired valves;
- an emulated goal routine and compressor.

Build and run:

```
cd OASMan_ESP32/eval && g++ -std=c++17 -O2 -Wall -o hcs_sim hcs_sim.cpp && ./hcs_sim   # -v for the full log
```

12 scenarios, 44 checks: hill spot, driving (plus blind detector), leak below min ride with reboot, load,
drive-off mid-fill, no BLE, sensor faults, jack, manual override, fast-leak latch, below-min user preset,
mapping mismatch.

The simulator tests the **decision logic**, not the physics or the thresholds.
