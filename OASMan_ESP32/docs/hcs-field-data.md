# HCS field data: what to record on the car, and why

Most supervisor thresholds are estimates (ESTIMATE in `hcs_config.h`). These recordings turn them into measured values, in priority order. All of it comes from the **shadow build**: it runs the supervisor and logs every decision but **never actuates**; the legacy maintain keeps driving the car as today.

## 1. Setup (once)

1. Flash the shadow build (USB): `version_num=test release_tag_name=build-local ~/.venvs/pio/bin/pio run -e manifold_v4_hcs_shadow -t upload` (from `OASMan_ESP32/`).
2. In the app: height-sensor mode **on**, "maintain" **on**, min ride calibrated.
3. Record the manifold's USB serial port (115200 baud); nothing is stored on the board.

### Recording on the comma (always powered, Linux, lots of storage)

`eval/hcs_logger.py` (stdlib-only Python) timestamps every line, writes one file per day, reconnects after an unplug or a manifold reboot, keeps 60 days, and opens the port **without toggling DTR/RTS** (on most ESP32 boards those reset the chip). It keeps only `HCS t=` (decisions), `HCSD` (1 Hz state) and `HRAW` (raw sensor volts) lines.

1. **Cable** from the manifold's USB port to the comma's AUX USB-C port. Do the first test parked: some boards still reset once when the port opens (safe: valves close, targets reload).
2. **Device:** over SSH, `ls /dev/ttyUSB* /dev/ttyACM*`. If nothing appears, that kernel lacks the driver; a Raspberry Pi Zero 2 W on the same cable runs the same script.
3. **Logger:**
   ```
   scp eval/hcs_logger.py comma@<comma-ip>:/data/hcs_logger.py
   ssh comma@<comma-ip> 'mkdir -p /data/hcs_logs; nohup python3 /data/hcs_logger.py --dir /data/hcs_logs > /data/hcs_logs/logger.out 2>&1 &'
   ```
   `/data` survives reboots and updates; the process does not unless your openpilot fork's launch script starts it.
4. **Collect:** `scp 'comma@<comma-ip>:/data/hcs_logs/hcs-*.log' .` (about 20-25 MB per day).

Short sessions with a laptop: `pio device monitor -b 115200 -f time -f log2file`, or the same script.

### What the log contains (and what it does not)

Time and what the manifold measures and decides: heights, pressures, tank, valves, compressor, BLE presence, decisions. **No door, seat, gear or load input**: "disturbance", "LOAD" or "DRIVING" are the supervisor's own inferences. A one-line note per event ("08:12 two people + groceries in, left right away") is what makes them checkable. The `HCS t=` counter is uptime in ms and wraps every 49.7 days; trust the logger's wall clock.

## 2. Sessions, in priority order

All happen in normal use once the logger runs; only 3 needs doing on purpose.

| # | Session | What it settles |
|---|---|---|
| 1 | **Parked nights at home** | `HCSD` `aH` / `aP` while parked: the motion detector's noise floor. `EXTERNAL ... bag lost N%` on arrival: how close the spot is to the jack line. `ARRIVAL targets ... twist`: the crown's real twist. Overnight `EVAL` lines (vetoed, still logged): RD's leak and cooling, hour by hour |
| 2 | **Normal drives** with a smooth highway stretch, dips, a hill bottom at speed, a parking ramp, the drive up to the spot | `aH` / `aP` while driving vs the thresholds; `MOTION` / `DRIVING confirmed` timing. Any `SHADOW would START` while moving is a failure. `ROAD ... -> sits low` with nobody added is a false positive |
| 3 | **Known loads, parked, controller connected**: driver in / out, one and two rear passengers, 10 kg and 20 kg in the trunk, a minute apart, with notes | `EVAL ... load=+x%` per known weight sets `HCS_LOAD_FRAC`. `air=same` must hold for unchanged air; `lost` / `gained` on door events means bag hysteresis |
| 4 | **Get in and drive** (your usual) | the `ROAD` / `SHADOW would PULSE` decisions and when |
| 5 | **A week of normal use** | RD's height / pressure trend from `HCSD`: leak rate over days, compressed at home vs flat |
| 6 | **A long drive** (1 h+) | `ROAD` lines: does the mean drift on a healthy car (aero, temperature)? |
| 7 | **Real corrections** (later, after 1-3 look right): diag build, you at the car, ~20 kg in the trunk, then out | `RESULT ... h a->b, IN open N ms`: the real fill rate and landing accuracy (shadow cannot log this) |

## 3. Setting each threshold from the data

| Tunable | From | Rule |
|---|---|---|
| `HCS_MOTION_H_THRESH`, `HCS_MOTION_P_THRESH` | `HCSD` `aH` / `aP`, parked vs driving | parked values well below the threshold, driving values above; too low = refuses to correct parked, too high = misses driving |
| `HCS_PRESSURE_NOISE_PSI`, `HCS_AIR_SIGN_DH`, `HCS_STABLE_RANGE_H` | parked `HCSD` h / p spread; door events | about 3x the parked standard deviation |
| `HCS_LOAD_FRAC` | known-weight events | between 10 kg (ignored) and one passenger (corrected) |
| `HCS_DEADBAND_H` | parked spread, `RESULT` landing | a landed correction never re-triggers |
| `HCS_ROAD_HEAVE_MIN` | `ROAD ... mean` on drives where nothing changed | above what a healthy car shows |
| `HCS_CALM_H` | `HCSD` h while driving straight | the 1-s wobble on straight roads plus margin |
| `HCS_FILL_RATE_DEFAULT` | `RESULT ... IN open` | measured %/s (learned per corner anyway, and persisted) |
| `HCS_ARRIVAL_QUIET_MS`, `HCS_CONFIRM_MS` | drive logs: longest stop-and-go stop, longest compression | longer than both |

## 4. Facts about the car

- Travel about 1 ft (~300 mm) from 0 psi to max: 1 % ~ 3 mm, the 3 % deadband ~ 9 mm, the 0.7 % motion threshold ~ 2 mm.
- Bags ~60-70 psi at normal height. On the home spot the compressed corners read 100-120 psi a little above min ride; FD / RP 80-100 %, FP 30-35 %, RD 15-20 % (stock tesla branch screen). Min ride ~20 % (unconfirmed).
- Tank 4 gal, cut-in 145 / cut-out 180 psi; two 480C-type compressors. With these, every bag on the spot stays more than 20 psi below the cut-out, so min-ride lifts there need no special tank margin.
- Phone not connected overnight; the in-car controller connects at power-up, so "BLE connected" means "car in use".
- The manifold runs 24/7. Targets, references and learned fill rates persist in the SPIFFS file `/hcsState.bin` (the pressure AI's store) and are never wiped. The supervisor's timing is wrap-safe across the 49.7-day `millis()` rollover.

Still useful: min ride per corner (to confirm the 20 %), per-corner travel if front and rear differ, the compressors' rated duty cycle (for the separate duty-cycle feature), usual passenger / cargo loads.
