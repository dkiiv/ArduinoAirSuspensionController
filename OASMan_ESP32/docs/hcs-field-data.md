# HCS field data: what to record on the car, and why

Most supervisor thresholds are estimates (ESTIMATE in `hcs_config.h`). These recordings turn them into measured values, in priority order. Two builds log the same lines: the **shadow build** decides but **never actuates** (legacy maintain drives the car), the **diag build** is the live supervisor plus the 1 Hz dump. A car already driving on the supervisor records with diag.

## 1. Setup (once)

1. Flash (USB, from `OASMan_ESP32/`): `version_num=test release_tag_name=build-local ~/.venvs/pio/bin/pio run -e manifold_v4_diag -t upload` (or `manifold_v4_hcs_shadow`).
2. In the app: height-sensor mode **on**, "maintain" **on**, min ride calibrated.
3. Record the manifold's USB serial port (115200 baud); nothing is stored on the board.

### Recording on a Raspberry Pi Zero W under the hood

A Zero W (v1, single core) is enough: the logger is stdlib Python at a few hundred lines per second. It needs its own 5 V; the ESP32's USB port is a device port and does not source power.

1. **SD card:** Raspberry Pi Imager, *Raspberry Pi OS Lite (32-bit)* (the v1 Zero is ARMv6: 64-bit images do not boot). In the Imager settings: hostname `hcs-pi`, enable SSH, a user, the home Wi-Fi (2.4 GHz only on a Zero W), timezone.
2. **Power:** a 12 V -> 5 V buck converter (automotive type, 2-3 A) on the same always-on 12 V feed as the manifold, into the Pi's **PWR IN** micro-USB (the port at the board's edge). Draw is ~0.2 A at 5 V.
3. **Data:** the Pi is the USB host, so the OTG adapter goes on its side: Pi **USB** port (the inner micro-USB) -> micro-USB OTG adapter (micro-B male to USB-A female) -> USB-A to USB-C cable -> ESP32. Connect the ESP32 end while parked: enumeration can reset the board once (valves close, targets reload).
4. **Logger as a service** (survives power cycles). From `OASMan_ESP32/`:
   ```
   scp eval/hcs_logger.py eval/setup_pi_logger.sh <user>@hcs-pi.local:
   ssh -t <user>@hcs-pi.local sudo sh setup_pi_logger.sh
   ```
   The script installs the logger, adds the user to `dialout`, creates and starts a systemd service, and prints the port, the log directory and the collect command. Check: `tail -f ~/hcs_logs/hcs-*.log` shows `HCSD` lines once a second.
5. **Clock:** the Pi has no RTC. Its clock is right once it has seen the home Wi-Fi (NTP) and stays right while powered; after a power loss it resumes from the last saved time until the next sync. The `HCS t=` counter is uptime in ms, so intervals are always right.
6. **Collect** (in the driveway): `scp '<user>@hcs-pi.local:hcs_logs/hcs-*.log' .` -- about 20-25 MB per day; the logger keeps 60 days.

### The logger

`eval/hcs_logger.py` (stdlib-only Python, Linux) timestamps every line, writes one file per day, reconnects after an unplug or a manifold reboot, keeps 60 days, and opens the port **without toggling DTR/RTS** (on most ESP32 boards those reset the chip). It keeps only `HCS t=` (decisions), `HCSD` (1 Hz state) and `HRAW` (raw sensor volts) lines. The same script runs on a comma device (AUX USB-C port, `/data`) if its kernel has the USB-serial driver. Short sessions with a laptop: `pio device monitor -b 115200 -f time -f log2file`.

### What the log contains (and what it does not)

Time and what the manifold measures and decides: heights, pressures, tank, valves, compressor, BLE presence, decisions. **No door, seat, gear or load input**: "disturbance", "LOAD" or "DRIVING" are the supervisor's own inferences. A one-line note per event ("08:12 two people + groceries in, left right away") is what makes them checkable. The `HCS t=` counter is uptime in ms and wraps every 49.7 days; trust the logger's wall clock.

## 2. Sessions, in priority order

All happen in normal use once the logger runs; only 3 needs doing on purpose.

| # | Session | What it settles |
|---|---|---|
| 1 | **Parked nights at home** | `HCSD` `aH` / `aP` while parked: the motion detector's noise floor. `EXTERNAL ... bag lost N%` on arrival: how close the spot is to the jack line. `ARRIVAL targets ... twist`: the crown's real twist. Overnight `EVAL` lines (vetoed, still logged): RD's leak and cooling, hour by hour |
| 2 | **Normal drives** with a smooth highway stretch, dips, a hill bottom at speed, a parking ramp, the drive up to the spot, and the rough hilly I-5 stretch where the car rose | `aH` / `aP` while driving vs the thresholds; `MOTION` / `DRIVING confirmed` timing -- **first question: does the detector see highway driving at all?** (`EVAL` lines while moving = blind). Any `SHADOW would START` while moving is a failure. `ROAD ... rough x` on the rough stretch vs normal roads. `ROAD ... -> sits low` with nobody added is a false positive |
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
| `HCS_ROAD_ROUGH_MAX` | `ROAD ... rough x` / `ROAD window rough (x)` on your smooth and rough roads (note which) | between your normal roads and the rough ones; the car must not rise on rough stretches |
| `HCS_FILL_RATE_DEFAULT` | `RESULT ... IN open` | measured %/s (learned per corner anyway, and persisted) |
| `HCS_ARRIVAL_QUIET_MS`, `HCS_CONFIRM_MS` | drive logs: longest stop-and-go stop, longest compression | longer than both |
| `HCS_CONFIRM_LOAD_MS`, `HCS_DISTURB_QUIET_MS` | known-load session: how long the car keeps settling after people sit | just longer than that |

## 4. Facts about the car

- Travel about 1 ft (~300 mm) from 0 psi to max: 1 % ~ 3 mm, the 3 % deadband ~ 9 mm, the 0.7 % motion threshold ~ 2 mm.
- Bags ~60-70 psi at normal height. On the home spot the compressed corners read 100-120 psi a little above min ride; FD / RP 80-100 %, FP 30-35 %, RD 15-20 % (stock tesla branch screen). Min ride ~20 % (unconfirmed).
- Tank 4 gal, cut-in 145 / cut-out 180 psi; two 480C-type compressors. With these, every bag on the spot stays more than 20 psi below the cut-out, so min-ride lifts there need no special tank margin.
- Phone not connected overnight; the in-car controller connects at power-up, so "BLE connected" means "car in use".
- The manifold runs 24/7. Targets, references and learned fill rates persist in the SPIFFS file `/hcsState.bin` (the pressure AI's store) and are never wiped. The supervisor's timing is wrap-safe across the 49.7-day `millis()` rollover.

Still useful: min ride per corner (to confirm the 20 %), per-corner travel if front and rear differ, the compressors' rated duty cycle (for the separate duty-cycle feature), usual passenger / cargo loads.
