# HCS bench simulator

`eval/hcs_sim.cpp` runs the **production** Height Control Supervisor sources (`src/heightControl/hcs_*.cpp`, unmodified) against a simulated car on a PC, replays scenarios, and checks what must and must not happen.

Use it to:
- check a change to the supervisor before it goes near the car;
- reproduce an edge case you saw in a field log;
- see what a threshold change does across all scenarios.

It tests the **decision logic**. The car model, noise and road are assumptions, so a threshold that passes here still has to be confirmed on the car ([hcs-field-data.md](hcs-field-data.md)).

## Build and run

From `OASMan_ESP32/` (any C++17 compiler; no Arduino, no PlatformIO):

```
g++ -std=c++17 -O2 -Wall -Wextra -o hcs_sim eval/hcs_sim.cpp
./hcs_sim              # all scenarios, PASS/FAIL per check, exit code 1 if anything failed
./hcs_sim S3 S16       # only those scenarios
./hcs_sim -v S1        # plus the full HCS decision log (same "HCS t=..." lines the firmware prints)
```

**Try a different tunable:** every value in `hcs_config.h` can be overridden with `-D`.

```
g++ -std=c++17 -O2 -DHCS_MOTION_H_THRESH=1.0f -DHCS_CONFIRM_MS=0 -o hcs_sim_test eval/hcs_sim.cpp && ./hcs_sim_test
```

(With `HCS_CONFIRM_MS=0`, S16 fails: that is how the dips rule was shown to be load-bearing.)

**Dump the internal state** between two sim times (ms) with the same `HCSD` line the diag build prints:

```
HCS_DUMP_FROM=790000 HCS_DUMP_TO=800000 ./hcs_sim S1
```

Runtime: all scenarios together take about a second of wall time and cover ~45 simulated hours.

## The simulated car

A **rigid body** (heave, pitch, roll) on four air springs, solved for static equilibrium every 100 ms.

- **Air springs.** Isothermal: `p = 100 * m * T * 60 / (h + 10)`, where `m` is the air mass in a bag (1.0 = preset) and `T` the temperature factor. Height is in %, with the preset at 50 and about 100 psi per corner.
- **Coupling.** Filling one corner moves and re-loads the other three, a ground crown loads one diagonal and unloads the other, and a leak tilts the whole body, as on a real car. No coupling is hand-tuned.
- **Limits.** Bump stops below 2 %; the wheel leaves the ground past 100 % (jack); a jack can be placed under a corner.
- **Loads.**
  - `extraF[i]`: people or cargo at a corner, as a fraction of a corner's static load.
  - `roll` / `pitch`: cornering or braking load transfer, as a fraction of total weight.
  - `heave`: vertical g minus 1 (+0.3 = the bottom of a dip at 1.3 g).
- **Ground.** `warp` is a crown, in height %: FP and RD up, FD and RP down. This is the hill spot.
- **Air.** `leakPerHour[i]` is the share of a bag's air lost per hour. `T` models heating and cooling.
- **Compressor.** On below 140 psi and off at 180, **only with a BLE client** (like `compressor.cpp`). It adds 1 psi/s.
- **Valves.** Flow is proportional to the tank-to-bag difference.
  - The goal routine is emulated: it stops at the target, the ceiling or the floor, and times out at `HCS_ROUTINE_TIMEOUT_MS`.
  - Manual valves (`manualIn[i]`) and cruise pulses are modelled too.
  - **Plant limitation:** a fill with the tank below the bag never pushes air back into the tank here, as it can on a real car (see the tank guard in [height-control-supervisor.md](height-control-supervisor.md)).
- **Measurement.**
  - Gaussian noise: 0.15 % on height, 0.25 psi on pressure.
  - `rough` adds road input: random plus a 1.4 Hz body bounce.
  - Bag pressure reads high while that corner's IN valve is open (flow offset), as the real sensor does.
  - `rawFault[i]` forces a raw height reading (wire break).

**Not modelled:**
- bag hysteresis, and pressure that lags height on a load change;
- tyre compliance;
- adiabatic (fast) compression;
- real valve flow curves;
- the AI flow-offset model;
- the BLE link itself.

Calibration is fixed at raw 10..90 with min ride 35. These are the gaps the field data has to close.

## Scenarios

| Id | Scenario | Key checks |
|---|---|---|
| S1 | hill spot: crown + slope, phone connected 8 h, cooling, slow leak on a hanging corner | arrival: targets rebuilt once (preset plane + crown twist), nothing corrected; hanging corners never dumped; compressed corners never pushed toward flat; leak refilled to the arrival height; no false leak latch |
| S2 | 30 min mixed driving (sweepers, braking, highway, red lights), plus a blind-detector variant | zero actuation while moving |
| S3 | overnight leak below min ride, no BLE, reboot at 5 h, owner returns at 8 h; plus a "nothing persisted" variant | nothing moves without BLE; RD lifted back to the preset (BOTTOM_GUARD + completion); driver load also compensated; only RD actuated |
| S4 | two rear passengers, 10 kg groceries, passengers leave | rear lifted back; front untouched; groceries ignored; lowered back on UNLOAD |
| S5 | drive off in the middle of a correction | abort within ~500 ms; the unfinished part is owed and topped up while driving |
| S6 | load with no BLE client, then the client connects | VETO while absent; corrected after |
| S7 | RD height wire break, then a second sensor | corner frozen; global FAULT with two |
| S8 | car jacked at RP with the phone connected | EXTERNAL freeze; nothing moves; clears afterwards |
| S9 | user jogs FD during an automatic rear correction | supervisor yields; only FD re-baselined |
| S10 | fast leak (25 %/h) | latched after 3 fast refills; one refill per reconnect |
| S11 | user parks on a stance preset below min ride | not lifted |
| S11b | show preset 1 (0 % / 0 psi), people in and out | zero automatic actions; no false sensor fault |
| S13 | rigid-body coupling with a leak | only the leaking corner refilled; all corners end within the deadband |
| S14 | the owner's leak: RD compressed on the hill spot, 2 %/h, 24 h connected | never latched; RD held at its arrival height |
| S15 | 4 h road trip with RD leaking 2.5 %/h | cruise pulses on RD only, never during a corner or brake; RD within 2 % of the others |
| S16 | dips and the bottom of hills at speed: normal road, glass-smooth road, long highway sags; then real load after parking | no fill, dump or pulse in any case; the confirmation rule exercised and holding; weight after parking still compensated |
| S17 | the shadow build: leak plus load for 2 h, phone connected | logs `SHADOW would START`; zero actuation; no leak bookkeeping from decisions that never ran |
| S18 | get in and drive from the hill spot: no BLE overnight, trunk + passenger loaded with the car off, controller connects when the driver sits, drive off 8 s later; arrive on flat ground, or back on the crown; plus a "nobody drives off" variant | level plane back at the preset (twist removed) within the deadband after arrival; the crown's twist kept at home; prints the drive-away pulses and how fast the parked correction starts |

(S12, the valve/sensor cross-wiring scenario, was removed together with that check.)

## Adding a scenario

1. Write a function in `eval/hcs_sim.cpp`:

   ```cpp
   static void scenarioMyCase()
   {
       printf("\nS17 short description\n");
       Sim s;
       g_log.clear();
       parkedAtPreset(s);            // boot, 15 s, preset 50/50/50/50 committed, 30 s
       s.drive(300, 0.6f);           // 5 min driving, roughness 0.6, sweepers + braking every 60 s
       s.run(180);                   // parked 3 min
       s.disturbance(6);             // someone gets in (6 s of shaking)
       s.extraF[C_RD] += 0.15f;      // ...and sits behind the driver
       s.run(120, [](Sim &x) { /* per-tick changes, e.g. x.heave = ... */ });
       check(s.startsWhileMoving == 0, "S17", "nothing while moving");
       check(fabsf(s.h[C_RD] - 50) <= HCS_DEADBAND_H, "S17", fmt("RD back (%.1f)", s.h[C_RD]));
       check(countLog("LOAD") > 0, "S17", "classified as LOAD");
   }
   ```

2. Add `{"S17", scenarioMyCase}` to the table in `main()`.

**Useful handles:**
- **Time:** `s.run(seconds, perTick)`, `s.drive(seconds, roughness, events, periodS)`, `s.disturbance(seconds)`.
- **Supervisor:** `s.boot(restore)` (reboot, optionally restoring the last persisted blob), `s.commitPreset()`, `s.core.notifyPresetLoad(h)`.
- **Car and world:** `s.presence`, `s.enabled`, `s.tank`, `s.T`, `s.warp`, `s.roll`, `s.pitch`, `s.heave`, `s.extraF[]`, `s.leakPerHour[]`, `s.jacked[]`, `s.rawFault[]`, `s.manualIn[]`.
- **Readings and counters:** `s.h[]`, `s.p[]`, `s.starts`, `s.startsWhileMoving`, `s.valveMsWhileMoving`, `s.fillsOn[]`, `s.dumpsOn[]`, `s.pulses`, `s.pulsesOn[]`, `s.pulsesInEvent`, `s.maxH[]`, `s.core.state()`.
- **Log:** `countLog("text")` counts HCS log lines containing the text.

**Turning a field log into a scenario:** take the `HCS ...` and `HCSD` lines around the event, then reproduce the situation, not the samples: what the car was doing, the load, the leak, BLE presence. The simulator does not replay recorded sensor data (yet). If the real car shows something the plant cannot produce (for example bag hysteresis), extend the plant first and note it in this document.
