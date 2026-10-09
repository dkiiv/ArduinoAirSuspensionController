# Height Control Supervisor (HCS)

Automatic ride-height holding for **height-sensor mode**:
- holds the preset;
- refills leaks, including a corner that leaked below min ride;
- lifts the car back after weight is added;
- leaves the car alone on uneven parking spots;
- never acts while the car is driving;
- tops up a slow leak during long drives.

It is built for a car whose only knowledge of the world is four height sensors, four bag pressures, tank pressure, valve/compressor state and "is a BLE client connected". There is no speed, gear, door or IMU signal; everything is inferred.

Status: **compiles (all envs), 61/61 bench-simulator checks pass, NOT yet run on a car.** Thresholds marked ESTIMATE in `hcs_config.h` must be measured first; see [hcs-field-data.md](hcs-field-data.md).

Bench simulator: [hcs-simulator.md](hcs-simulator.md).

---

## 1. Where it sits

```
wheel tasks (existing)                     supervisor task (new, 10 Hz)
  read height + pressure  ───────────────▶  hcs::Core::tick()
  goal routine (existing, tuned)  ◀──────   START / ABORT / PULSE
```

- **Keeps the existing actuator.** `Wheel::goalRoutine` / `achieveFineGoal` (fine landing, sync barrier) is tuned on cars. The supervisor only *decides*, then calls `initAutonomousGoal(target, dir, bounds)`. The routine re-checks the hard bounds on every loop.
- **Replaces only the decision layer** in height mode + maintain. The legacy `Wheel::maintainPressure` path returns early when the supervisor owns maintenance. Pressure mode and sensorless levelling are untouched.
- **Behind a compile flag.** `HEIGHT_CONTROL_SUPERVISOR` defaults to false; `manifold_v4_release` is the legacy build, unchanged. Inside an HCS build it runs when height-sensor mode **and** the existing "maintain" toggle are on.
- **No BLE protocol change.** Firmware, controller and app are untouched.

| File | Job |
|---|---|
| `hcs_config.h` | every tunable, with units and a one-line reason |
| `hcs_core.h/.cpp` | types, state machine, user override, running a correction, persistence, logging |
| `hcs_sense.cpp` | sample intake, electrical sensor faults, motion detector, STATS |
| `hcs_classify.cpp` | evaluation: classify each corner, arbitrate, start one correction |
| `hcs_cruise.cpp` | cruise top-up (slow leak while driving) |
| `heightControlSupervisor.cpp` | ESP32 adapter: builds inputs, executes commands, NVS, diag dump |

The core has no Arduino includes; `eval/hcs_sim.cpp` compiles the same files on a PC. Cost when compiled in: about 2.9 KB RAM and 23 KB flash. The core does a few hundred float operations per 100 ms tick, which is negligible on an ESP32.

## 2. States

| State | Meaning | Actuates? |
|---|---|---|
| INIT | first 10 s after boot, buffers filling | no |
| INACTIVE | not height mode, maintain off, or safety mode | no |
| MOTION | a motion episode is in progress (driving or a disturbance) | no (cruise top-up only, see 6) |
| SETTLING | quiet, but not long enough yet: 120 s after a drive or boot, 8 s after a disturbance | no |
| PARKED | evaluated; re-evaluates every 30 s, on events, and when a confirmation is due | may start a correction |
| CORRECTING | one correction in flight, then 6 s settle and a result check | its own |
| MANUAL | the user is doing something | no; yields, then re-baselines what the user touched |
| FAULT | >= 2 height sensors faulted | no |

## 3. Motion detector (no speed signal)

- **Per-corner activity.** EMA (~0.6 s) of |second difference| of height and of bag pressure. A second difference ignores smooth ramps (our own fills) but sees road texture, body bounce, and the start and end of roll, pitch and heave.
- **Votes.** A corner votes "moving" above 0.7 % height or 1.2 psi (ESTIMATES). Only idle, healthy corners vote.
- **Motion.** >= 2 corners voting for 3 ticks starts an episode; 3 s of quiet ends it.
  - An episode longer than 15 s is **DRIVING**; shorter is a **disturbance** (door, person, trunk).
  - Fewer than 2 observable corners counts as "not quiet" (fail-safe).
- **Units.** Height % is a share of each corner's *manually calibrated* travel, so the thresholds already scale with your calibration. They are deliberately not raised automatically from measured noise: a higher threshold makes the "moving" veto weaker, which is the unsafe direction. Measure them instead ([hcs-field-data.md](hcs-field-data.md)).

## 4. Classifier

Physics: bag pressure carries the corner's load, and air mass sets the height at that load. For each corner, versus the moment it was last known-good:

- **Sign test (air).** A bag whose air is unchanged moves along its own p(h) curve: down means pressure up, up means pressure down. Down >= 1.5 % without a pressure rise means it **lost air** (leak, cooling). Up without a pressure drop means it **gained air** (heat). If >= 3 bags agree, it is temperature, not one bag's event.
- **Total load.** `L = mean((p - pRef) / pScale)`. Terrain, slopes, crowns and cornering move load *between* corners (L ~ 0); people and cargo add load (L > +1.5 %, ESTIMATE, ~28 kg).
- **Rigid body.** One bag losing air also tilts the other three. They are **COUPLED**: never fixed or accepted, because refilling the leaking bag restores them.

| Situation (e = h - target) | Class | Action |
|---|---|---|
| below min ride (or 5 % if uncalibrated) | BOTTOM_GUARD | lift to max(target, minRide + 3) |
| above 95 % + 3 | CEILING | lower to 95 % |
| \|e\| <= 3 | WITHIN | none |
| low, this bag lost air | AIR_LOSS | fill to target |
| low, another bag's air changed | COUPLED | hold |
| low, air same, L up | LOAD | fill to target |
| low, air same, L neutral | SHIFT (terrain / someone moved) | **accept**: target = where it is |
| high, this bag gained air | AIR_GAIN | hold, never dump (it reverses as it cools) |
| high, another bag changed / air same + L down / air same + L neutral | COUPLED / UNLOAD (lower to target) / SHIFT (accept) | |
| contradicting evidence, or no reference | AMBIGUOUS / SHIFT | never pump |

Supporting rules:
- **Confirmation.** A correction only starts if two evaluations at least 10 s apart want it, with no motion episode in between. A dip or the bottom of a hill at speed compresses all four corners for a second or two; a person or cargo stays. (Bench: with this rule disabled, a long highway sag on a glass-smooth road triggers fills. With it, none.)
- **Event latch.** While any correction is pending, the load references stay frozen, so correcting the first loaded corner cannot erase the evidence for the second.
- **Preset anchor.** After a drive, a target within 3 % of the preset snaps back to it, so the reference cannot wander over many trips.
- **Completion pass.** A correction that lands more than 1 % short (routine timeout, tank) gets one follow-up.
- **User-held low corners.** A corner the user put below min ride (show preset, stance) is held there, never lifted.

## 5. Arbiter: what must be true before anything moves

| Gate | Value |
|---|---|
| BLE client connected (`isVehicleOn()`) | required. No phone: nothing moves |
| enabled, not safety mode, no FAULT, no EXTERNAL (jack / lift) | required |
| corner sensor healthy; bag pressure valid for fills; tank sensor valid | required |
| fill target <= 95 %; routine cut at 96 % on the live reading | hard |
| dump target >= minRide + 3; routine cut 1 below | hard |
| bag psi < min(bagMaxPressure, MAX_PRESSURE_SAFETY) - 10 | hard |
| tank >= bag + 20 psi | hard |
| confirmation (section 4) | required |
| per-corner dwell 60 s; no up-then-down within 10 min without motion in between | no oscillation |
| 8 corrections per hour (x 10 s routine timeout = 80 s of valve time per hour) | budget |
| fast-leak latch: 3 refills each < 30 min apart; one more refill per BLE reconnect | burst bag / fitting |
| >= 2 healthy corners not being corrected (to watch for motion) | required |

- One correction at a time: the axle with the largest error, same direction, at most two corners. This keeps the car level side to side, limits tank draw and leaves two corners watching for motion.
- Fills are fill-only and dumps are dump-only, so a correction never overshoots and comes back.
- Any motion on the watching corners aborts the correction within ~300 ms.

## 6. Cruise top-up (slow leak on a long drive)

The parked path never acts while driving. For a slow leak on a road trip:

- **When it can act:** after 10 min of driving, using 2-minute averages of each corner's height and pressure.
- **Signature** (rigid body):
  - **Load warp.** The leaking bag and its diagonal partner unload; the other diagonal loads up.
  - **Sign test against the trip reference** (the averages at minute 10). The leaking bag went *down* without its pressure rising.
- **What does not trigger it.** Cornering, braking, passengers and camber produce no warp. Dips and hills compress all four corners and raise all four pressures, which is neither warp nor "down without pressure".
- **Action.** An open-loop fill-only pulse on that one corner, <= 1.5 s, sized from the parked fill rate (x0.7) for ~2 %; 400 ms until the rate is known.
- **Limits.**
  - Needs >= 20 s of steady driving.
  - Aborted the moment roll or pitch moves.
  - At most 4 pulses an hour, and one per corner per 5 min.
  - The valve deadline is enforced in the wheel's own loop, so the valve closes on time even if the supervisor task stalls.

## 7. Scenarios

**Hill spot, 3 a.m., slow leak, phone connected.**
1. On arrival every corner is SHIFT (terrain): the crown geometry becomes the target, and no valve moves.
2. Cooling sags all corners a little; a corner past 3 % is refilled to its *arrival* height, so the compressed corners stay compressed.
3. A slowly leaking corner is refilled to its arrival height every few hours, logging its leak rate each time.

Bench S1: 8 h on the crown. The hanging corners are never dumped, the compressed corners are never pushed toward flat, and the worst corner ends 2.3 % from arrival. S14 (RD compressed and leaking 2 %/h for 24 h): never latched, worst sag 3.0 %.

**Cornering / driving.**
- Road texture keeps >= 2 corners active, so the car is DRIVING and nothing evaluates.
- If the detector were blind: an evaluation needs 120 s of quiet, cornering is SHIFT (L ~ 0, accepted, not corrected), and a correction needs confirmation.

Bench S2: zero actuation.

**Dips and the bottom of a hill at speed** (all four corners compress, all pressures rise).
- On a normal road: DRIVING, nothing evaluates.
- On a glass-smooth road with the detector blind: an evaluation can see "all four low, load up" (LOAD), but the confirmation 10 s later sees it gone.

Bench S16: 15 min of sharp dips (normal and glassy) and long highway sags, with zero fills, dumps or pulses. Weight added after parking is still compensated (S16, S4).

**Leak below min ride** (the reported bug: the legacy code never lifts it after a reboot, another corner's correction or a manual jog).
- BOTTOM_GUARD lifts any corner below its floor regardless of history.
- Targets persist in NVS, so after a reboot it lifts to the preset; with nothing persisted, to min ride + 3.

Bench S3: overnight leak with no BLE and a reboot at 5 h. Nothing moves while the phone is away; when it connects, RD 34.4 -> 49.7 and the driver's load on FD is also compensated.

**Weight added.** Two adults into the rear seats: a disturbance, then 8 s quiet, then L up, LOAD, confirmed 10 s later, and the rear axle goes back to preset. 10 kg of groceries stays inside the deadband. When the passengers leave: UNLOAD, lowered back (S4).

**Show preset (0 % / 0 psi).** It is user-held below min ride, so it is never lifted and never cruise-pulsed (S11b).

## 8. Faults and safe states

| Fault | Detection | Effect |
|---|---|---|
| height wire break / short / implausible | raw outside the 0.5-4.5 V band, or > 15 % outside the calibrated travel, or calibration span < 5; 10 samples | corner frozen; cleared after 5 min good |
| bag pressure out of range | < -12 or > 240 psi | no fills on that corner |
| tank sensor out of range | same band | no fills |
| >= 2 height faults | | FAULT, observe only |
| jack / lift / wheel unsupported | any bag loses > 35 % of its pressure | EXTERNAL: everything frozen until it returns |

Wiring a sensor or valve to the wrong corner is the installer's responsibility. It shows up as wrong vehicle behaviour, and the supervisor does not try to detect it.

| Event | Safe state |
|---|---|
| BLE lost mid-correction | abort, valves closed, car stays where it is |
| car moves mid-correction | abort within ~300-500 ms |
| user touches anything | abort; MANUAL; re-baseline only the touched corners |
| crash / power loss mid-fill | solenoids de-energise closed; persisted targets restored; 120 s quiet before acting |
| wheel task hangs with a valve open | supervisor watchdogs (12 s valve, 30 s correction) close it from the supervisor task |
| bad / old NVS blob | rejected (magic, version, every field range-checked in `Core::begin`); starts unanchored |

## 9. Builds and rollout

| Env (all extend `manifold_v4_release`) | What it does |
|---|---|
| `manifold_v4_release` | legacy, unchanged |
| `manifold_v4_hcs_shadow` | decides and logs every decision (`SHADOW would ...`), **never actuates**; legacy maintain stays active; 1 Hz dump |
| `manifold_v4_diag` | active + 1 Hz `HCSD` state line + `HRAW` raw sensor volts |
| `manifold_v4_hcs` | active, event log only |

`version_num=test release_tag_name=build-local ~/.venvs/pio/bin/pio run -e manifold_v4_hcs_shadow`

Order: shadow (collect data, [hcs-field-data.md](hcs-field-data.md)) -> set thresholds -> diag on the car with you watching -> hcs.

## 10. On-car test plan (pass / fail)

| # | Test | Pass | Fail |
|---|---|---|---|
| R1 | preset on level ground, connected 12 h | all corners within 3 % at the end, every correction logged | > 4 % off with no logged reason |
| R2 | slow leak (your RD) | AIR_LOSS refills to target +-1 %; neighbours log COUPLED, not SHIFT | other corners actuated |
| R2b | let a corner leak below min ride, power-cycle the manifold, connect | BOTTOM_GUARD lifts it to the preset | not lifted |
| R2c | >= 2 h drive with the leak | `CRUISE RD ... watching`, then pulses on RD only, only in steady driving | pulse on another corner or during a corner / brake |
| R3 | two adults into the rear; 10 kg in the trunk | rear back to preset +-1.5 % within 60 s; the 10 kg does nothing | front actuated |
| R4 | park on the hill spot overnight, connected | SHIFT on arrival; no correction that reduces the diagonal | any such correction |
| R5 | 30 min mixed drive incl. dips, a hill bottom at speed, and the drive up into the hill spot | zero START / SHADOW START while moving | any |
| R5b | load the trunk, drive off as soon as START logs | ABORT within 1 s | valve open > 1 s after moving |
| R6 | phone out of range, load the trunk | `VETO (no BLE client)`, nothing moves; reconnect -> corrected | anything moves without a client |
| F1 | unplug a height sensor | FAULT within 1 s, corner never actuated | actuated |
| F2 | jack one corner | EXTERNAL, nothing moves | anything moves |
| S | preset 1 (0 %), people in and out, 1 h | no autonomous fill | any |

## 11. Confidence and limits

| Claim | Confidence | What would falsify it |
|---|---|---|
| leak-below-min-ride fix | high | R2b fails |
| terrain / cornering cannot trigger a correction | medium-high | bags with large hysteresis (pressure lagging height) would show AIR_LOSS / AIR_GAIN on door events in the shadow log |
| dips / hills cannot trigger a correction | medium-high | an evaluation wanting LOAD while driving, confirmed (look for `CONFIRM` + START while moving) |
| motion thresholds | **low until measured** | shadow STATS |
| load threshold 1.5 %, pressure noise 1.5 psi | medium | shadow EVAL lines on door events |
| cruise leak signature | medium | pulses on a healthy corner, or none while a corner sinks |
| a car creeping at walking pace on glass-smooth floor is detected | **low** | bounded instead: confirmation, one axle, 10 s |

## 12. Legacy findings (tesla @ `6f7d6b78`), for reference

- **Leak below min ride never lifted (confirmed).**
  - `pressureCaptureBaseline()` refuses a baseline below `heightCalMinRide`.
  - Any valve activity on any corner, and every boot, wipes every baseline.
  - `maintainPressure()`'s no-baseline branch is pressure-mode only.
  - So after a reboot, another corner's correction or a manual jog, a corner below min ride is never lifted.
  - The claim that `initPressureGoal` clamps near min ride is refuted.
- **No driving detection in height mode**, and corrections are bidirectional, so a long sweeper gets corrected mid-corner.
- **Fights the hill spot** (the baseline is not re-taken after a drive).
- **No height ceiling while filling**, and **no sensor fault handling** (the readings are clamped).
- **On a lift, all four bags are dumped.**
- **Not changed here:**
  - `millis()` wrap in routine timeouts;
  - `loadProfileAirUp` uses `>` where it needs `>=` (out-of-bounds preset read);
  - no compressor duty cycle (pro's todo item, to be a standalone feature).
