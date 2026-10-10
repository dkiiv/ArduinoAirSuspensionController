# Height Control Supervisor (HCS)

Automatic ride-height holding for **height-sensor mode**. It holds the preset, refills leaks, treats min ride as a hard floor, lifts the car back after weight is added (also when you drive off before it could), leaves uneven parking spots alone, and never runs a closed-loop correction while driving.

The only inputs are four heights, four bag pressures, tank pressure, valve / compressor state and "is a BLE client connected". There is no speed, door or IMU signal; everything else is inferred.

Status: **compiles (all envs), 51/51 bench checks pass (17 scenarios, [hcs-simulator.md](hcs-simulator.md)), NOT yet run on a car.** Thresholds marked ESTIMATE in `hcs_config.h` must be measured first ([hcs-field-data.md](hcs-field-data.md)).

## 1. Where it sits

```
wheel tasks (existing)                     supervisor task (new, 10 Hz)
  read height + pressure  ───────────────▶  hcs::Core::tick()
  goal routine (existing, tuned)  ◀──────   START / ABORT / PULSE
```

- **Keeps the existing actuator.** `Wheel::goalRoutine` / `achieveFineGoal` is tuned on cars. The supervisor only decides, then calls `initAutonomousGoal(target, dir, bounds)`; the routine re-checks the hard bounds every loop.
- **Replaces only the decision layer** in height mode + maintain. `Wheel::maintainPressure` returns early when the supervisor owns maintenance. Pressure mode and sensorless levelling are untouched.
- **Behind a compile flag.** `HEIGHT_CONTROL_SUPERVISOR` defaults to false; `manifold_v4_release` is unchanged. In an HCS build it runs when height-sensor mode **and** "maintain" are on.
- **No BLE protocol change.**

| File | Job |
|---|---|
| `hcs_config.h` | every tunable, with units |
| `hcs_core.h/.cpp` | types, state machine, user override, running a correction, persistence, logging |
| `hcs_sense.cpp` | sample intake, electrical sensor faults, motion detector |
| `hcs_classify.cpp` | parked evaluation: classify each corner, arbitrate, start one correction |
| `hcs_drive.cpp` | while driving: road level and fill pulses |
| `heightControlSupervisor.cpp` | ESP32 adapter: inputs, commands, persistence file, diag dump |

The core has no Arduino includes; `eval/hcs_sim.cpp` compiles the same files on a PC. Cost when compiled in: 1.8 KB RAM, 21.5 KB flash, a few hundred float operations per 100 ms tick.

## 2. States

| State | Meaning | Actuates? |
|---|---|---|
| INIT | first 10 s after boot | no |
| INACTIVE | not height mode, maintain off, or safety mode | no |
| MOTION | a motion episode (driving or a disturbance) | only the fill pulses of section 5 |
| SETTLING | quiet, not long enough yet: 120 s after a drive or boot, 3 s after a disturbance | no |
| PARKED | evaluated; re-evaluates every 30 s, on events, and when a confirmation is due | may start a correction |
| CORRECTING | one correction in flight, then 6 s settle and a result check | its own |
| MANUAL | the user is doing something | no; yields, then re-baselines what the user touched |
| FAULT | >= 2 height sensors faulted | no |

## 3. Motion detector (no speed signal)

- Per corner: EMA (~0.6 s) of |second difference| of height and of bag pressure. It ignores smooth ramps (our own fills) but sees road texture, body bounce and the start / end of roll, pitch and heave.
- A corner votes "moving" above 0.7 % height or 1.2 psi (ESTIMATES); >= 2 voting corners for 3 ticks start an episode; 3 s of quiet ends it. Longer than 15 s is **DRIVING**, shorter a **disturbance** (door, person, trunk). Fewer than 2 observable corners counts as "not quiet".
- Thresholds are in % of each corner's calibrated travel, so they scale with your calibration. They are not raised automatically from measured noise (a higher threshold weakens the "moving" veto); measure them instead.

## 4. Parked: classifier

Bag pressure carries the corner's load; air mass sets the height at that load. Per corner, versus when it was last known-good:

- **Sign test (air).** Unchanged air moves along its own p(h) curve: down means pressure up. Down >= 1.5 % without a pressure rise = **lost air**; up without a pressure drop = **gained air** (heat). >= 3 bags agreeing = temperature, not one bag's event.
- **Total load** `L = mean((p - pRef) / pScale)`. Terrain and slopes move load between corners (L ~ 0); people and cargo add it (> +1.5 %, ESTIMATE, ~28 kg).
- **Rigid body.** One bag losing air tilts the other three: they are **COUPLED**, never fixed or accepted.

| Situation (e = h - target) | Class | Action |
|---|---|---|
| below min ride (5 % if uncalibrated), any cause: leak, load, the ground | BOTTOM_GUARD | lift to max(target, min ride + 3), even if the other corners rise |
| above 95 % + 3 | CEILING | hold; lower to 95 % only if it gained air |
| after a drive, sat off the preset level on the road | ROAD | correct by that much |
| \|e\| <= 3 | WITHIN | none |
| low: lost air / another bag changed / air same + L up / air same + L neutral | AIR_LOSS / COUPLED / LOAD / SHIFT | fill / hold / fill / **accept** (target = where it is) |
| high: gained air / another bag changed / air same + L down / air same + L neutral | AIR_GAIN / COUPLED / UNLOAD / SHIFT | hold / hold / lower / accept |
| contradicting evidence, or no reference | AMBIGUOUS / SHIFT | never pump |

Rules around it:
- **Min ride is a hard floor.** Lifted whatever put the corner there; a corner dropping below it is evaluated at once (not at the next 30 s check). A corner the user put below min ride on purpose (show preset) is held there instead.
- **Lift at connect.** When the BLE client connects, a corner the last parked evaluation found below min ride (overnight leak) is lifted at once, without waiting for people to settle; only confirmed driving stops it. Parked evaluations keep running without a client, so that finding is at most 30 s old.
- **Arrival targets.** After a drive the old targets belong to the old spot. New targets = the **preset's level plane** + **this spot's twist** (FP + RD - FD - RP). Uneven ground is pure twist on a rigid car, so it is held, never corrected and never carried to the next spot. Corners within the deadband are held where they arrived, with fresh air references. Load is not compared across spots (a hanging wheel carries almost nothing); load added before leaving is caught on the road (section 5).
- **Road level (ROAD).** The road is level on average, so a corner that sat off the preset level for the last drive window (twist removed) is off for an air reason: a leak the spot hides, air added at the spot, load. On arrival it is corrected by that much, and kept pending until within the deadband. Fills use the full deficit. Dumps use only the tilt part (heat lifts all four together and goes away), unless we added air since the last arrival (a min-ride lift, drive pulses).
- **Confirmation.** A correction starts only after every corner held within 1.5 % with no motion for 10 s, or for 3 s right after a detected disturbance or for a corner below min ride. "Held" counts from the last reading outside the band. People and cargo rock the car as they get in (a disturbance), so they are corrected within seconds. A smooth dip or sag at speed causes no disturbance and moves the car for several seconds, so the 10 s stands between it and a fill. (Bench: without it, a long highway sag on a glassy road triggers fills.)
- **Event latch.** While a correction is pending, the load references stay frozen, so fixing the first loaded corner cannot erase the evidence for the second.

## 5. Driving: road level and fill pulses

The parked path never acts while moving. This path is open-loop, fill-only, one corner at a time, pulses of at most 1.5 s:

- **Every 2 min** of driving, each corner's average height, twist removed, is compared with the preset plane. Curves and braking tilt the car but cannot lower it on average (total load is constant). So only when the car sits **low overall** (mean deficit > 1 %) are the corners carrying it owed their deficit: those with at least half the worst corner's deficit, and more than 1 %. (A leaking bag's neighbours show about a third of its deficit once the twist is removed; refilling the bag restores them.) That covers load added just before leaving, a lift cut short by driving off, and a leak on a long trip. (Bench: a 3-minute spiral ramp reads +7.8 % "low" on the outside corners and is correctly ignored.)
- **Rough roads.** On a rough road the average reading is not where the car sits: rebound damping packs it down, and spring / linkage nonlinearity shifts the mean. A window whose mean height activity is above 2.5x the motion threshold (ESTIMATE) is ignored. Each window's verdict replaces the last, so nothing owed earlier is delivered after a rough stretch. And a drive adds at most 10 % per corner, whatever the readings say. (Bench: the build before this change added air on a rough stretch that read 6 % low, and raised the car +17 % on a calm road reading 15 % low; now 0 and about +11.)
- **Pulses** go only when every corner's 1-s average has held within 2 % for 8 s (pulling out, braking and turning in all break that). A pulse is cut the moment roll or pitch starts to change (bench: within 0.1-0.2 s). At most 2 % per pulse, never more than owed.
- **Self-calibrating.** Pulse length comes from the corner's fill rate. The next window measures what the pulses really did and updates that rate (as parked fills do). Until a rate is known, a window owes only half its deficit, so a wrong default cannot overshoot.
- Each window's top-up counts as a refill for the fast-leak latch, so a burst bag latches while driving too.

## 6. Arbiter: what must be true before anything moves

| Gate | Value |
|---|---|
| BLE client connected (`isVehicleOn()`) | required: no client, nothing moves |
| enabled, not safety mode, no FAULT, no EXTERNAL (min-ride lifts excepted, never on the unloaded corner) | required |
| corner sensor healthy, bag pressure valid for fills, tank sensor valid | required |
| fill target <= 95 %, routine cut at 96 % live | hard |
| dump target >= min ride + 3, routine cut 1 below | hard |
| bag psi < min(bagMaxPressure, MAX_PRESSURE_SAFETY) - 10 | hard |
| tank >= bag + 20 psi; if the cut-out can never get there, refused and logged | hard |
| confirmation (section 4) | required |
| per-corner dwell 60 s; no up-then-down within 10 min without motion between | no oscillation |
| 8 corrections per hour (x 10 s timeout = 80 s of valve time) | budget |
| fast-leak latch: 3 refills each < 30 min apart; one more per BLE reconnect | burst bag / fitting |
| >= 2 healthy corners not being corrected (they watch for motion) | required |

One correction at a time: the axle with the largest error, one direction. Motion on the watching corners aborts it within ~300 ms.

## 7. Scenarios (bench numbers; preset 50)

- **Hill spot overnight, slow leak (S1, S14).** Targets become the preset plane + the crown's twist; nothing moves at arrival; hanging corners are never dumped, compressed ones never pushed flat. 8 h: worst corner 2.2 % from arrival. 24 h of RD leaking 2 %/h: 7 refills, never latched, worst sag 3.0 %.
- **Driving (S2).** Sweepers, braking, a spiral ramp, highway, red lights: zero corrections and zero pulses.
- **Dips / hill bottoms at speed (S16).** Normal and glassy roads, long highway sags: zero fills, dumps or pulses. Load after parking is still corrected.
- **Leak below min ride, no BLE, reboot at 5 h (S3).** Nothing moves while you are away; at connect RD 34.4 -> 48.4. A corner bled below min ride by hand while parked starts lifting 3.7 s after the air stops.
- **Two rear passengers (S4).** Rear 45.0; the correction starts 4.7 s after the last movement and the rear is back inside the deadband at 5.5 s (fill speed is the car's); groceries ignored; lowered back when they leave.
- **Get in and drive from the hill spot (S18).** No BLE overnight, trunk + passenger loaded with the car off, the driver sits, you leave 8 s later. The parked correction starts 4 s after the driver sits and is cut when you pull away; the rest comes as 12 pulses in a 15-min town drive. Flat-ground arrival 52.7 / 50.5 / 49.1 / 46.9; home again, the crown's twist (-9) is kept and the plane restored (worst 2.5).
- **Long trip, RD leaking 2.5 %/h (S15).** 2 pulses, RD only, none left open after a curve began; RD within 3.2 % of the others.
- **Your spot (S19)**, modelled with your numbers (65 psi at the preset, tank 145 / 180, min ride 20): FP 34, RP 87, FD 100, RD 23.5, bags 89 / 40 / 35 / 116 psi. The hanging wheel looks like a jack (EXTERNAL): only min-ride lifts may act. Nothing is below min ride, so nothing moves. Overnight RD leaks to 14 % (it would drag); when the controller connects the lift starts at once, and 12 s later RD is at 19.4 and rising (bag ~134 psi with the people in, tank down to 143 and refilling). Driving clears the freeze; flat-ground arrival 51.4 / 49.7 / 48.7 / 47.0. Separately: a spot that presses corners below min ride gets them lifted on the spot, and a tank that can never get 20 psi above the bag is refused with the reason logged.
- **Show preset 0 % / 0 psi (S11b).** User-held below min ride: never lifted, never pulsed.

## 8. Faults and safe states

| Fault / event | Detection | Effect |
|---|---|---|
| height wire break / short / implausible | raw outside 0.5-4.5 V, > 15 % outside calibrated travel, or span < 5; 10 samples | corner frozen; cleared after 5 min good |
| bag or tank pressure out of range | < -12 or > 240 psi | no fills on that corner / no fills |
| >= 2 height faults | | FAULT, observe only |
| jack / lift / wheel hanging off severe ground | a bag loses > 35 % of its pressure | EXTERNAL: frozen except min-ride lifts (never on the unloaded corner) until it returns or DRIVING |
| BLE lost / car moves / user touches anything mid-correction | | abort, valves closed; MANUAL re-baselines only the touched corners |
| crash / power loss mid-fill | | solenoids close; persisted targets restored; 120 s quiet first |
| wheel task hangs with a valve open | 12 s valve / 30 s correction watchdogs | closed from the supervisor task |
| bad persistence file | magic, version, every field range-checked | starts unanchored; learned fill rates kept if valid on their own |

Wiring a sensor or valve to the wrong corner is the installer's responsibility; the supervisor does not try to detect it.

**Persistence:** targets, references and the learned fill rate per corner are written to `/hcsState.bin` in SPIFFS (`writeBytes` / `readBytes`, the store the pressure AI uses), at most once a minute (immediately on a user commit). Nothing is ever wiped.

## 9. Builds and rollout

| Env (all extend `manifold_v4_release`) | What it does |
|---|---|
| `manifold_v4_release` | legacy, unchanged |
| `manifold_v4_hcs_shadow` | decides and logs (`SHADOW would ...`), **never actuates**; legacy maintain stays active; 1 Hz `HCSD` / `HRAW` |
| `manifold_v4_diag` | active + 1 Hz `HCSD` / `HRAW` |
| `manifold_v4_hcs` | active, event log only |

`version_num=test release_tag_name=build-local ~/.venvs/pio/bin/pio run -e manifold_v4_hcs_shadow`

Order: shadow (collect data) -> set thresholds -> diag with you watching -> hcs.

## 10. On-car test plan

| # | Test | Pass | Fail |
|---|---|---|---|
| R1 | preset on level ground, connected 12 h | all corners within 3 %, every correction logged | > 4 % off with no logged reason |
| R2 | RD's slow leak | AIR_LOSS refills to target; neighbours log COUPLED, not SHIFT | another corner actuated |
| R2b | RD below min ride, power-cycle the manifold, connect | lifted to the preset | not lifted |
| R2e | morning with RD below min ride, get in normally | `URGENT ... lifting now` within a second of connecting | lift waits for people, or RD still low with no logged reason |
| R3 | two adults into the rear; 10 kg in the trunk | rear starts rising within ~5 s of them sitting still, back within 3 %; the 10 kg does nothing | front actuated, or nothing within 10 s |
| R4 | hill spot overnight (phone connected for the test) | `ARRIVAL targets` with the twist; no correction that changes the twist | any |
| R5 | 30 min mixed drive incl. dips, a hill bottom at speed, a parking ramp | zero START / SHADOW START while moving; no `-> sits low` without load or a leak | any |
| R5b | load the trunk, drive off as START logs | ABORT within 1 s | valve open > 1 s after moving |
| R7 | at the hill spot: 2 people + trunk, drive off within 10 s, 15 min town, park on flat ground | `PULSE` only while calm; arrival within 3 % on every corner | a pulse left open into a turn, or > 4 % off |
| R6 | phone out of range, load the trunk | `VETO (no BLE client)`; corrected on reconnect | anything moves without a client |
| F1 | unplug a height sensor | FAULT within 1 s, corner never actuated | actuated |
| F2 | jack one corner | EXTERNAL, nothing moves | anything moves |
| S | preset 1 (0 %), people in and out, 1 h | no autonomous fill | any |

## 11. Confidence and limits

| Claim | Confidence | What would falsify it |
|---|---|---|
| min-ride floor / leak-below-min-ride fix | high | R2b / R2e fail |
| terrain cannot trigger a correction | medium-high | bag hysteresis: door events showing `air=lost/gained` in the shadow log |
| dips / hills cannot trigger a correction | medium-high | `CONFIRM` + START while moving |
| motion thresholds | **low until measured** | shadow `HCSD` aH / aP parked vs driving |
| load threshold 1.5 %, pressure noise 1.5 psi | medium | shadow EVAL lines on known loads |
| "sits low overall" means air or load | medium | `-> sits low` on drives where nothing changed (aero lift at speed, a commute that is one long climb) |
| rough-road windows are recognised (2.5x) | **low until measured** | `ROAD ... rough x` values on known rough / smooth roads; the car rising on rough roads |
| crown handled as twist | medium-high | arrival on the hill spot logs LOAD / AIR_LOSS with nobody added |
| the severe spot freezes as EXTERNAL | medium | the real hanging wheel keeps > 65 % of its pressure (then normal rules apply; min-ride lifts either way) |
| a car creeping at walking pace on a glassy floor is detected | **low** | bounded instead: confirmation, one axle, 10 s |

## 12. Legacy findings (tesla @ `6f7d6b78`)

- **Leak below min ride never lifted (confirmed):** `pressureCaptureBaseline()` refuses a baseline below `heightCalMinRide`; any valve activity and every boot wipe every baseline; the no-baseline branch is pressure-mode only.
- No driving detection in height mode; corrections are bidirectional, so a long sweeper is corrected mid-corner. Fights the hill spot. No height ceiling while filling; no sensor fault handling. On a lift, all four bags are dumped.
- Not changed here: `millis()` wrap in routine timeouts; `loadProfileAirUp` uses `>` where it needs `>=`; no compressor duty cycle (a separate feature).
