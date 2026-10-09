# HCS bench simulator

`eval/hcs_sim.cpp` runs the **production** supervisor sources (`src/heightControl/hcs_*.cpp`, unmodified) against a simulated car on a PC, replays scenarios, and checks what must and must not happen. It tests the **decision logic**; the car model, noise and road are assumptions, so thresholds still have to be confirmed on the car ([hcs-field-data.md](hcs-field-data.md)).

## Build and run

From `OASMan_ESP32/` (any C++17 compiler):

```
g++ -std=c++17 -O2 -Wall -Wextra -o hcs_sim eval/hcs_sim.cpp
./hcs_sim              # all scenarios; exit code 1 if anything failed (~7 s, ~75 simulated hours)
./hcs_sim S3 S16       # only those
./hcs_sim -v S1        # plus the full "HCS t=..." decision log
HCS_DUMP_FROM=790000 HCS_DUMP_TO=800000 ./hcs_sim S1   # HCSD state lines between two sim times (ms)
```

Every value in `hcs_config.h` can be overridden with `-D`, e.g. `-DHCS_MOTION_H_THRESH=1.0f`.

**What a check is for.** 47 checks over 17 scenarios. Each asserts one behaviour no other check covers, or a precondition that stops a scenario from passing vacuously. Each guarded rule was disabled with a `-D` override to confirm a check fails:

| Override | Fails |
|---|---|
| `-DHCS_CONFIRM_MS=0` (no confirmation) | S16 dips |
| `-DHCS_URGENT_MAX_AGE_MS=0` (no lift at connect) | S19 |
| `-DHCS_OWE_MAX=0.0f` (no drive pulses) | S15, S18, S19 |
| `-DHCS_ROAD_HEAVE_MIN=-100.0f` (no "sits low overall" gate) | S2 spiral ramp, S15 |
| `-DHCS_AUTO_LOWER=false` | S4 unload, S15, S19 |

## The simulated car

A **rigid body** (heave, pitch, roll) on four air springs, solved for static equilibrium every 100 ms.

- **Air springs**, isothermal: `p = pNom * m * T * 60 / (h + 10)`, `m` = air mass (1.0 = preset), `T` = temperature factor, preset at 50 %. `pNom` is the bag pressure at the preset (default 100; the owner's car ~65). Filling one corner re-loads the other three, a crown loads one diagonal, a leak tilts the whole body; no coupling is hand-tuned.
- **Limits.** Bump stops below 2 %; past 100 % a wheel leaves the ground and its bag carries only the wheel; a jack can be placed under a corner.
- **Loads.** `extraF[i]` people / cargo (fraction of a corner's static load); `roll` / `pitch` cornering / braking (fraction of the weight); `heave` vertical g - 1.
- **Ground.** `warp`: a crown (FP and RD up, FD and RP down). `ground[i]`: extra height under one wheel. `parkSevere()` reproduces the owner's spot.
- **Air.** `leakPerHour[i]`; `T` for heating / cooling.
- **Compressor.** On below `tankOn`, off at `tankOff` (default 140 / 180), **only with a BLE client**; +1 psi/s.
- **Valves.** Flow proportional to tank - bag. The goal routine is emulated (stops at target, ceiling or floor; times out). Manual valves and fill pulses too. A fill with the tank below the bag never pushes air back here, as it could on a car.
- **Measurement.** Noise 0.15 % height, 0.25 psi; `rough` adds road input (random + 1.4 Hz bounce); bag pressure reads high while its IN valve is open; `rawFault[i]` forces a raw reading (wire break).

Not modelled: bag hysteresis, tyre compliance, fast (adiabatic) compression, real valve flow curves, the AI flow-offset model, the BLE link. Calibration is raw 10..90, min ride 35 unless a scenario sets it.

## Scenarios

| Id | Scenario | Key checks |
|---|---|---|
| S1 | hill spot: crown + slope, connected 8 h, cooling, slow leak on a hanging corner | targets = preset plane + crown twist; hanging corners never dumped; compressed corners never pushed flat; leak refilled; no false latch |
| S2 | 30 min mixed driving (sweepers, braking, a 3-min spiral ramp, highway, red lights); a blind-detector variant | zero corrections and pulses while moving |
| S3 | overnight leak below min ride, no BLE, reboot at 5 h, owner returns at 8 h; a "nothing persisted" variant | nothing moves without BLE; RD lifted into the deadband; only RD actuated |
| S4 | two rear passengers, 10 kg groceries, passengers leave | rear lifted back; front untouched; groceries ignored; lowered back |
| S5 | drive off in the middle of a correction | aborted within 1.5 s, valves closed |
| S6 | load with no BLE client, then it connects | vetoed while absent; corrected after |
| S7 | RD wire break, then a second sensor | corner frozen; FAULT with two |
| S8 | jacked at RP, phone connected | EXTERNAL freeze; nothing moves; clears |
| S9 | user jogs FD during an automatic rear correction | supervisor yields; only FD re-baselined |
| S10 | fast leak (25 %/h) | latched after 3 fast refills; one refill per reconnect |
| S11b | show preset (0 % / 0 psi), people in and out | zero automatic actions; no false fault |
| S14 | RD compressed on the hill spot, leaking 2 %/h, 24 h connected | never latched; RD held at its arrival height |
| S15 | 4 h road trip, RD leaking 2.5 %/h | pulses on RD only; one caught by a curve / brake starting is cut within 0.3 s; RD kept near the others |
| S16 | dips and hill bottoms at speed (normal, glassy, long sags); then real load after parking | no fill / dump / pulse; the confirmation rule exercised; load after parking still corrected |
| S17 | shadow build, leak + load 2 h | `SHADOW would START` logged; zero actuation; no leak bookkeeping from decisions that never ran |
| S18 | get in and drive from the hill spot (no BLE overnight, loaded with the car off, leave 8 s after the driver sits); flat ground / back home / nobody drives | level while still driving; level on arrival; crown twist kept at home |
| S19 | the owner's spot (twist ~-33 %), with the owner's car (65 psi, tank 145 / 180, min ride 20): night with an RD leak, load, connect, leave / come back / stay; plus a spot that presses corners below min ride, and a tank that can never get above the bag | lift starts the moment the controller connects; hanging corners never dumped; flat arrival level; min-ride lift on the spot; refusal logged |

## Adding a scenario

```cpp
static void scenarioMyCase()
{
    printf("\nS20 short description\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);            // boot, preset 50/50/50/50 committed
    s.drive(300, 0.6f);           // 5 min driving
    s.run(180);                   // parked 3 min
    s.disturbance(6);             // someone gets in...
    s.extraF[C_RD] += 0.15f;      // ...behind the driver
    s.run(120);
    check(fabsf(s.h[C_RD] - 50) <= HCS_DEADBAND_H, "S20", fmt("RD back (%.1f)", s.h[C_RD]));
}
```

Add `{"S20", scenarioMyCase}` to the table in `main()`, and confirm the new check fails when its rule is disabled.

Handles: `s.run(seconds, perTick)`, `s.drive(seconds, roughness, events, periodS)`, `s.disturbance(seconds)`, `s.boot(restore)`, `s.presence`, `s.tank`, `s.pNom`, `s.minRide`, `s.T`, `s.warp`, `s.roll`, `s.pitch`, `s.heave`, `s.extraF[]`, `s.leakPerHour[]`, `s.jacked[]`, `s.rawFault[]`, `s.manualIn[]`; readings `s.h[]`, `s.p[]`; counters `s.starts`, `s.startsWhileMoving`, `s.fillsOn[]`, `s.dumpsOn[]`, `s.pulsesOn[]`, `s.pulsesInEvent`; `countLog("text")`.

**From a field log:** take the `HCS` / `HCSD` lines around the event and reproduce the situation (what the car was doing, load, leak, BLE), not the samples. If the car shows something the plant cannot produce (e.g. bag hysteresis), extend the plant first.
