# Height Control Supervisor (HCS)

Automatic ride-height holding for **height-sensor mode**:
- holds the preset;
- refills leaks, including a corner that leaked below min ride;
- lifts the car back after weight is added, also when you drive off before it could;
- leaves the car alone on uneven parking spots;
- never runs a closed-loop correction while driving;
- tops up a slow leak during long drives.

It is built for a car whose only knowledge of the world is four height sensors, four bag pressures, tank pressure, valve/compressor state and "is a BLE client connected". There is no speed, gear, door or IMU signal; everything is inferred.

Status: **compiles (all envs), 72/72 bench-simulator checks pass (19 scenarios), NOT yet run on a car.** Thresholds marked ESTIMATE in `hcs_config.h` must be measured first; see [hcs-field-data.md](hcs-field-data.md).

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
| `hcs_cruise.cpp` | while driving: drive-away top-up (load added just before leaving) and cruise top-up (slow leak) |
| `heightControlSupervisor.cpp` | ESP32 adapter: builds inputs, executes commands, NVS, diag dump |

The core has no Arduino includes; `eval/hcs_sim.cpp` compiles the same files on a PC. Cost when compiled in: about 3.1 KB RAM and 26 KB flash. The core does a few hundred float operations per 100 ms tick, which is negligible on an ESP32.

## 2. States

| State | Meaning | Actuates? |
|---|---|---|
| INIT | first 10 s after boot, buffers filling | no |
| INACTIVE | not height mode, maintain off, or safety mode | no |
| MOTION | a motion episode is in progress (driving or a disturbance) | no (only the open-loop top-ups of section 6) |
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
| below min ride (or 5 % if uncalibrated) **and** this bag lost air, or its air history is unknown | BOTTOM_GUARD | lift to max(target, minRide + 3) |
| above 95 % + 3 | CEILING | hold; lower to 95 % only if this bag gained air (heat) |
| \|e\| <= 3 | WITHIN | none |
| low, this bag lost air | AIR_LOSS | fill to target |
| low, another bag's air changed | COUPLED | hold |
| low, air same, L up | LOAD | fill to target |
| low, air same, L neutral | SHIFT (terrain / someone moved) | **accept**: target = where it is |
| high, this bag gained air | AIR_GAIN | hold, never dump (it reverses as it cools) |
| high, another bag changed / air same + L down / air same + L neutral | COUPLED / UNLOAD (lower to target) / SHIFT (accept) | |
| contradicting evidence, or no reference | AMBIGUOUS / SHIFT | never pump |

Supporting rules:
- **Arrival targets.** After a drive the old targets belong to the old parking spot. The new ones are rebuilt as the **preset's level plane** (heave, pitch, roll: what you asked for) **plus this spot's twist** (FP + RD - FD - RP).
  - Uneven ground such as a crown is pure twist on a rigid car (one diagonal up, the other down), so it is kept and never corrected, and never carried to the next spot.
  - Load and leaks change the plane, so they still show up.
  - Corners within the deadband of these targets are then held exactly where they arrived, with a fresh air reference for this spot.
  - **No load verdict on arrival.** The pressure references belong to the previous spot. On uneven ground the bags share the load differently (a hanging wheel carries almost nothing), so the pressures are not comparable. Arrival judges air (sign test) and safety only; load added before leaving is judged on the road (6a).
- **Terrain is never "below min ride".** A corner pressed below min ride by the ground still has its air: pressure up, the sign test reads "same". It is left alone. Only a bag that lost air (or one with no history, e.g. a fresh board) is lifted. Likewise a hanging corner above the ceiling is held unless it gained air.
- **Confirmation.** A correction only starts once it has persisted 10 s with no motion: either every corner has stayed within 1.5 % since the last movement for 10 s, or two evaluations 10 s apart both want it. A dip or the bottom of a hill at speed compresses all four corners for a second or two (and moves them); a person or cargo stays. (Bench: with confirmation disabled, a long highway sag on a glass-smooth road triggers fills. With it, none.)
- **Event latch.** While any correction is pending, the load references stay frozen, so correcting the first loaded corner cannot erase the evidence for the second.
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

## 6. Top-ups while driving

The parked, closed-loop path never acts while driving. Two open-loop, fill-only paths do, both with pulses of at most 1.5 s on one corner at a time. Each is cut short the moment the car stops being calm or steady, and the valve deadline is enforced in the wheel's own loop.

### 6a. Drive-away top-up (load added right before leaving)

With the in-car controller as the BLE client, nothing can happen until the driver sits down. People and cargo usually go in at the same moment, and you often pull away before the parked correction (about 10 s after the last movement) has run. What is owed is delivered while driving:

- **Decided parked.** A fill that the last parked evaluation wanted on steady readings, or a fill cut short by the drive-off (from the last height read before the car moved), becomes **owed** when DRIVING is confirmed. Only its level-plane part is owed (twist removed). On uneven ground people's weight lands on the compressed corners, and owing that per-corner sag would over-fill them on flat ground.
- **Decided while driving** (load nobody evaluated). From 60 s into the drive, drive-long averages (restarted after every pulse) are checked every 10 s. It fires once per drive when both hold for 30 s:
  - total bag pressure is up versus **the previous trip's road average** (more than 1.5 % load). Road against road, so no parking spot is involved; before the first complete trip after boot it falls back to the parked reference;
  - the level plane, with the twist removed, is low by more than the deadband.

  The plane deficit per corner is owed. Slopes and leaks keep the total load, dips average out, and heat raises the car, so none of them qualify.
- **Delivery.** At most 2 % per pulse, sized from the corner's learned fill rate (or 2 %/s, ESTIMATE, until a parked fill teaches it). Pulses go only when every corner's 1-s average has stayed within 2 % for 8 s: turning in or out, braking and accelerating all break that. Never more than owed, and at most 10 % per corner. A cut-short pulse is credited only for the time it ran.
- **On arrival** the parked path checks the result against the new spot's targets.

### 6b. Cruise top-up (slow leak on a long drive)

For a slow leak on a road trip:

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
1. On arrival the targets become the preset plane plus the crown's twist (`ARRIVAL targets ... twist -9`); every corner is held where it arrived, and no valve moves.
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
- BOTTOM_GUARD lifts a corner below its floor whose bag lost air, or whose history is unknown (fresh board). A corner pressed down by the ground is left alone.
- Targets persist in NVS, so after a reboot it lifts to the preset; with nothing persisted, to min ride + 3.

Bench S3: overnight leak with no BLE and a reboot at 5 h. Nothing moves while the phone is away; when it connects, RD 34.4 -> 49.7 and the driver's load on FD is also compensated.

**Weight added, parked.** Two adults into the rear seats: a disturbance, then quiet. Once every corner has held still for 10 s it is LOAD, and the rear axle fills back to its target, about 10 s after the last movement plus 3-5 s of fill. 10 kg of groceries stays inside the deadband. When the passengers leave: UNLOAD, lowered back (S4).

**Get in and drive, from the hill spot** (bench S18). The phone is not connected overnight; the in-car controller connects when the driver sits.
1. Overnight on the crown, nothing moves; leaks and the arrival geometry are logged.
2. With the car off, 20 kg go in the trunk and a rear passenger gets in. This is evaluated and vetoed (no BLE), and remembered.
3. The driver sits, the controller connects, and you pull away 8 s later.

Result:
- 12 drive-away pulses in a 15-min town drive: the trunk load as decided parked, then the passenger and driver as DRIVE LOAD.
- On flat ground it arrives at FP 49.7, RP 50.4, FD 49.6, RD 50.3 (preset 50) with nothing left to correct.
- Back home on the crown, the twist (-9) is kept and the level plane is restored.
- If nobody drives off, the first parked correction starts 11 s after the driver sat down, and fills toward the crown-shaped targets, not toward flat.
- Before the arrival-target rule, the same trip ended at FP 61.9, RP 59.9, FD 54.3, RD 52.2: the stale crown targets were filled on flat ground.

**The owner's real hill spot** (bench S19). Screen readings on the stock tesla branch: FD and RP 80-100 %, FP 30-35 %, RD 15-20 %, a twist of about -33 %. The sim reproduces it as FP 34, RP 87, FD 100, RD 23. One hanging wheel is nearly unloaded (bag pressure -46 %), and both compressed corners sit below min ride.
- **At the spot:** a nearly unloaded wheel looks exactly like a corner on a jack, so EXTERNAL freezes everything. Nothing is corrected while parked there: no lift of the compressed corners, no dump of the hanging ones, no load correction.
- **Leaving:** DRIVING clears the freeze. Load added at the spot (trunk, passenger, driver) plus RD's overnight leak are recognised on the road (DRIVE LOAD, about 90 s in) and topped up: 12 pulses in 15 min.
- **Arriving on flat ground:** FP 50.7, RP 52.1, FD 48.3, RD 49.7 (preset 50).
- **Back at the spot:** the twist (-33) is kept, the corners sit where an empty car arrived, and it freezes again.
- **Load at the spot with nobody driving:** nothing happens until you drive.

On a milder uneven spot (no wheel unloaded) the normal rules apply: the twist is accepted, terrain-compressed corners are not lifted, and hanging ones are not dumped.

**Show preset (0 % / 0 psi).** It is user-held below min ride, so it is never lifted and never cruise-pulsed (S11b).

## 8. Faults and safe states

| Fault | Detection | Effect |
|---|---|---|
| height wire break / short / implausible | raw outside the 0.5-4.5 V band, or > 15 % outside the calibrated travel, or calibration span < 5; 10 samples | corner frozen; cleared after 5 min good |
| bag pressure out of range | < -12 or > 240 psi | no fills on that corner |
| tank sensor out of range | same band | no fills |
| >= 2 height faults | | FAULT, observe only |
| jack / lift / wheel unsupported (including a wheel hanging off severe uneven ground) | any bag loses > 35 % of its pressure | EXTERNAL: everything frozen until it returns, or until DRIVING is confirmed |

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
| R4 | park on the hill spot overnight (connect the phone for the test) | `ARRIVAL targets` with the twist, no correction at arrival; no correction that reduces the diagonal | any such correction |
| R5 | 30 min mixed drive incl. dips, a hill bottom at speed, and the drive up into the hill spot | zero START / SHADOW START while moving | any |
| R5b | load the trunk, drive off as soon as START logs | ABORT within 1 s | valve open > 1 s after moving |
| R7 | at the hill spot: 2 people + trunk load, driver in last, drive off within 10 s; drive 15 min in town, park on flat ground | `DRIVE-AWAY` / `DRIVE LOAD` pulses only while going straight at steady speed; arrival within 3 % of the preset on every corner | a pulse during a turn / brake, or > 4 % off after arrival |
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
| drive-load check (total pressure up + level plane low on drive averages) | medium | `DRIVE LOAD` on a drive with nobody added (aero, long climbs, temperature), or pulses that overshoot on arrival |
| crown handled as twist | medium-high | arrival on the hill spot logs LOAD / AIR_LOSS with nobody added, or corrections that change the twist |
| the severe spot freezes as EXTERNAL | medium | the real hanging wheel keeps > 65 % of its pressure: then no freeze, and the normal terrain rules apply (also covered) |
| road-average pressure as the load reference | medium | `DRIVE LOAD` without anyone added, e.g. after a drive in very different weather or on a long descent |
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
