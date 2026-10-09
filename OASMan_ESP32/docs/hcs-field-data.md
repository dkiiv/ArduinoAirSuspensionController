# HCS field data: what to record on the car, and why

Most supervisor thresholds are engineering estimates (marked ESTIMATE in `hcs_config.h`). This page lists the recordings that turn them into measured values, in the order they matter.

All of it comes from the **shadow build**. It runs the supervisor and logs every decision, but **never actuates**: the legacy maintain keeps driving the car exactly as today. Nothing here needs the active build.

## 1. Setup (once)

1. Flash the shadow build (USB):

   ```
   cd OASMan_ESP32
   version_num=test release_tag_name=build-local ~/.venvs/pio/bin/pio run -e manifold_v4_hcs_shadow -t upload
   ```

2. In the app: height-sensor mode **on**, "maintain" **on** (the supervisor only runs when both are on). Min ride height calibrated.

3. Capture the serial port to a file with timestamps. The manifold talks at 115200 baud over its USB port.
   - **Windows** (simplest in the car with a laptop). With PlatformIO installed on Windows, or any serial terminal that logs to a file:

     ```
     pio device monitor -b 115200 -f time -f log2file
     ```

     `log2file` writes `platformio-device-monitor-<date>.log` in the current folder.
   - **WSL**: attach the USB device first (`usbipd list`, then `usbipd attach --wsl --busid <id>` in an admin PowerShell), then run the same command.

4. The laptop has to stay connected and awake for the session (car 12 V or a power bank). The board does not log to flash.

The lines that matter start with `HCS t=` (decisions), `HCSD` (1 Hz state) and `HRAW` (raw sensor volts, diag/shadow builds). Everything else can be dropped:

```
grep -E "HCS t=|HCSD|HRAW" platformio-device-monitor-*.log > hcs-session.log
```

Send the trimmed file and a one-line note per session saying what happened and when ("08:12 drove off, 08:40 parked at home on the hill spot", "18:05 two adults got in the back, ~80 kg each").

## 2. Sessions, in priority order

| # | Session | How | What it settles |
|---|---|---|---|
| 1 | **Parked night at home** (the hill spot) | park, stay in BLE range (phone in the house is fine if it stays connected), log until morning | `STATS QUIET`: the parked noise floor of the motion detector, and `sdH` / `sdP` (height and pressure noise). `EVAL ARRIVAL`: how the crown really classifies. `PRESENCE` lines: whether your phone holds the BLE link overnight, which decides whether anything can happen at 3 a.m. at all. Cooling over the night: AIR_LOSS on all corners vs your RD leak |
| 2 | **Mixed drive, 30+ min** | highway, on-ramp loops, stop-and-go, a few dips, the bottom of a hill at speed, and the drive up into the hill spot | `STATS DRIVE`: motion activity while driving vs the thresholds. `MOTION` / `DRIVING confirmed` timing (pulling away, red lights). Any `SHADOW would START` while moving is a failure, so tell me where and when |
| 3 | **Door and load events, parked** | with notes: driver in / out, one rear passenger, two rear passengers, a known weight in the trunk (water bottles: 10 kg, then 20 kg) | `EVAL EVENT load=+x%` per known weight sets `HCS_LOAD_FRAC` from data. The `air=same/lost/gained` and `dp` values show whether the sign test holds on your bags (unchanged air should always read `same`). If door events show `air=lost/gained`, the bags have hysteresis, which matters a lot |
| 4 | **A few real corrections** (later, after 1-3 look right) | needs the **diag** build (active), with you at the car: put ~20 kg in the trunk and let it lift the rear, then remove it. Shadow does not log this: `RESULT` lines exist only for corrections the supervisor ran itself | `RESULT ... h a->b, IN open N ms`: how fast a corner fills (height % per second of open valve), which sizes cruise pulses and checks the 10 s routine timeout; how close it lands |
| 5 | **A week of normal use** | just leave the shadow build on and grab the log now and then | the 1 Hz `HCSD` lines give RD's height and pressure trend between the legacy refills: your leak rate over days, day vs night, compressed at home vs flat. (The `LEAK ... refill` lines only appear for real supervisor refills, i.e. with the diag / hcs builds) |
| 6 | **A long drive** (1 h+) | road trip or a long highway stretch | `CRUISE` lines: whether the leak signature shows on RD and nowhere else, and how often steady 20 s windows occur on your roads |

With the diag build instead of shadow, the same sessions also give `HRAW` raw voltages. Use that for any wiring or calibration question, e.g. a sensor sitting near the 0.5 / 4.5 V edges.

## 3. How each threshold gets set from the data

| Tunable | From | Rule |
|---|---|---|
| `HCS_MOTION_H_THRESH`, `HCS_MOTION_P_THRESH` | `STATS QUIET` / `STATS DRIVE` | buckets are multiples of the threshold `[<.25, .25-.5, .5-1, 1-2, 2-4, >4]`. Parked mass should sit in the first two buckets with ~0 at >= 1; driving mass at >= 1. If parked shows counts >= 1 the threshold is too low (the car would refuse to correct while parked); if highway driving sits below 1, it is too high |
| `HCS_PRESSURE_NOISE_PSI`, `HCS_AIR_SIGN_DH` | `sdP`, `sdH`; door events | about 3x the parked standard deviation |
| `HCS_LOAD_FRAC` | known-weight events | between "10 kg of groceries" (should be ignored) and "one passenger" (should be compensated) |
| `HCS_DEADBAND_H` | `sdH`, `RESULT` landing accuracy | wide enough that a landed correction never re-triggers |
| `HCS_CRUISE_*` | long drive `CRUISE` lines, the measured leak rate | warp and sign thresholds just above what healthy corners show |
| `HCS_ROUTINE_TIMEOUT_MS`, pulse sizing | `RESULT ... IN open` | the routine should land well inside the timeout |
| `HCS_ARRIVAL_QUIET_MS`, `HCS_CONFIRM_MS` | drive logs: longest stop-and-go stop, longest compression on your roads | longer than both |

## 4. Facts about the car I can't log

- Per corner, how many mm of travel your calibration covers (min to max), so % can be read as mm.
- Bag type / brand, tank size, compressor model and its duty-cycle rating (for the separate compressor duty-cycle feature).
- The usual passenger and cargo loads.
- Whether the phone stays connected when you are inside the house, i.e. whether "BLE connected" means "owner nearby" at your place.
