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

3. Record the serial port (115200 baud) somewhere that stays in the car. The manifold prints everything on its USB port; nothing is stored on the board itself.

### Recording on a comma 3X / four (recommended: always powered, Linux, lots of storage)

`eval/hcs_logger.py` is a stdlib-only Python logger. It:
- timestamps every line;
- writes one file per day;
- reconnects after an unplug or a manifold reboot;
- keeps 60 days;
- opens the port **without toggling DTR/RTS** (on most ESP32 boards those lines reset the chip).

1. **Cable.** Run a USB cable from the manifold's USB port to the comma's **AUX USB-C** port (a USB-C to USB-A/micro adapter as needed). Do the first test parked: some boards still reset once when the port opens, which is safe (valves close, targets reload).
2. **Device.** SSH in (comma docs: "connect to a comma 3X or comma four") and check that the port exists:

   ```
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

   If nothing appears, that kernel has no driver for the manifold's USB-serial chip. Then use a Raspberry Pi Zero 2 W on the same cable instead; the same script works.
3. **Logger.** Copy the script and start it in the background:

   ```
   scp eval/hcs_logger.py comma@<comma-ip>:/data/hcs_logger.py
   ssh comma@<comma-ip>
   mkdir -p /data/hcs_logs
   nohup python3 /data/hcs_logger.py --dir /data/hcs_logs > /data/hcs_logs/logger.out 2>&1 &
   ```

   `/data` survives reboots and openpilot updates. The logger process does not: after the comma reboots, start it again with the `nohup` line. (Making it start on boot means editing openpilot's launch script, which updates overwrite.)
4. **Collect** whenever convenient. On the same Wi-Fi as the car:

   ```
   scp 'comma@<comma-ip>:/data/hcs_logs/hcs-*.log' .
   ```

   Size: about 20-25 MB per day (the shadow build prints a 1 Hz state line), so 60 days is ~1.5 GB.

### Recording with a laptop (short sessions)

```
pio device monitor -b 115200 -f time -f log2file
```

Or run `python3 eval/hcs_logger.py --dir hcs_logs` on any Linux machine. On WSL, attach the USB device first: `usbipd attach --wsl --busid <id>` in an admin PowerShell.

### What the log contains (and what it does not)

Only time and what the manifold measures and decides: heights, pressures, tank, valve and compressor state, BLE presence, and the supervisor's decisions. There is **no door, seat, gear or load input**. Words like "disturbance", "LOAD" or "DRIVING" in the log are the supervisor's own inferences from those sensors. Comparing them with what really happened is the point of the notes below.

The `HCS t=` counter is the manifold's uptime in ms. Your board never powers off, so it wraps to 0 every 49.7 days. The logger's wall-clock stamp is the one to trust, provided the comma's clock had synced. Right after a comma boot the clock can be off until it syncs; the uptime counter still orders events correctly.

### What to send

The logger keeps only lines starting `HCS t=` (decisions), `HCSD` (1 Hz state) and `HRAW` (raw sensor volts). Send the day files plus a one-line note per event: what happened and roughly when. For example:
- "08:12 two people + groceries in, drove off right away"
- "08:40 parked on the hill spot"

Clock times in the notes are enough; the files carry the comma's clock.

## 2. Sessions, in priority order

All of these happen in normal use once the logger runs. Only number 3 needs you to do something on purpose.

| # | Session | How | What it settles |
|---|---|---|---|
| 1 | **Parked nights at home** (the hill spot) | nothing to do | Expect `EXTERNAL ... bag lost N%` on arrival: the hanging wheel is nearly unloaded. N tells how close the spot is to that line. `STATS QUIET`: the parked noise floor of the motion detector, and `sdH` / `sdP` (height and pressure noise). The `ARRIVAL targets ... twist` line: how much twist the crown really puts on the car. Overnight `EVAL` lines (vetoed, no BLE, but still logged): the RD leak and cooling, hour by hour |
| 2 | **Normal drives**, ideally one with a long smooth highway stretch, a few dips, the bottom of a hill at speed, and the drive up into the hill spot | nothing to do | `STATS DRIVE`: the driving signal vs the thresholds. `MOTION` / `DRIVING confirmed` timing (pulling away, red lights). Any `SHADOW would START` while moving is a failure: tell me where and when. `DRIVE LOAD` lines on drives where nobody was added would be false positives |
| 3 | **Known loads, parked, controller connected** | car on, in P, controller connected: driver in / out; one rear passenger; two; then 10 kg and 20 kg in the trunk (water bottles), a minute apart, with notes | `EVAL ... load=+x%` per known weight sets `HCS_LOAD_FRAC` from data. The `air=same/lost/gained` values show whether the sign test holds on your bags (unchanged air must read `same`). If door events show `lost` / `gained`, the bags have hysteresis, which matters a lot |
| 4 | **Get in and drive** (your usual) | nothing special: note when people and cargo went in and when you left | the `DRIVE-AWAY` / `DRIVE LOAD` decisions the shadow build would have taken, and when |
| 5 | **A week of normal use** | nothing to do | RD's height and pressure trend from the 1 Hz `HCSD` lines: the leak rate over days, day vs night, compressed at home vs flat |
| 6 | **A long drive** (1 h+) | road trip | `CRUISE` lines: whether the leak signature shows on RD and nowhere else, and how often steady windows occur on your roads |
| 7 | **Real corrections** (later, after 1-3 look right) | needs the **diag** build (active), with you at the car: put ~20 kg in the trunk and let it lift the rear, then remove it. Shadow does not log this: `RESULT` lines only exist for corrections the supervisor ran itself | `RESULT ... h a->b, IN open N ms`: the real fill rate (height % per second of open valve), which sizes every drive-away and cruise pulse and replaces the 2 %/s estimate; landing accuracy |

With the diag build instead of shadow, the same sessions also give `HRAW` raw voltages. Use that for any wiring or calibration question, e.g. a sensor sitting near the 0.5 / 4.5 V edges.

## 3. How each threshold gets set from the data

| Tunable | From | Rule |
|---|---|---|
| `HCS_MOTION_H_THRESH`, `HCS_MOTION_P_THRESH` | `STATS QUIET` / `STATS DRIVE` | buckets are multiples of the threshold `[<.25, .25-.5, .5-1, 1-2, 2-4, >4]`. Parked mass should sit in the first two buckets with ~0 at >= 1; driving mass at >= 1. If parked shows counts >= 1 the threshold is too low (the car would refuse to correct while parked); if highway driving sits below 1, it is too high |
| `HCS_PRESSURE_NOISE_PSI`, `HCS_AIR_SIGN_DH`, `HCS_STABLE_RANGE_H` | `sdP`, `sdH`; door events | about 3x the parked standard deviation |
| `HCS_LOAD_FRAC` | known-weight events | between "10 kg of groceries" (should be ignored) and "one passenger" (should be compensated) |
| `HCS_DEADBAND_H` | `sdH`, `RESULT` landing accuracy | wide enough that a landed correction never re-triggers |
| `HCS_FILL_RATE_DEFAULT`, pulse sizing | `RESULT ... IN open` | measured %/s per corner |
| `HCS_OWE_CALM_H` | `HCSD` lines while driving | the 1-s height wobble on straight, steady roads, plus margin |
| `HCS_CRUISE_*` | long-drive `CRUISE` lines, the measured leak rate | warp and sign thresholds just above what healthy corners show |
| `HCS_ARRIVAL_QUIET_MS`, `HCS_CONFIRM_MS` | drive logs: longest stop-and-go stop, longest compression on your roads | longer than both |

## 4. Facts about the car

Known so far:
- Travel: about 1 ft (~300 mm) from 0 psi on the bump stops to max pressure. So 1 % of calibrated travel is ~3 mm: the 3 % deadband is ~9 mm, and the 0.7 % motion threshold ~2 mm.
- Tank 4 gal; two 480C-type compressors.
- Phone not connected overnight; the in-car controller connects when the car powers up. So "BLE connected" means "car in use", and nothing is corrected while the car sleeps (leaks are logged and corrected at the next power-up).
- The manifold runs 24/7 (constant 12 V), so its state stays in RAM between trips (learned fill rates, last-trip road pressures). The supervisor's timing is wrap-safe across the 49.7-day `millis()` rollover. The few legacy comparisons that are not only misbehave in a window of seconds at the wrap (an early timeout).
- Home spot: FD and RP read 80-100 %, FP 30-35 %, RD 15-20 % (stock tesla branch screen).

Still useful:
- the per-corner travel, if the front and rear differ;
- the compressors' rated duty cycle (for the separate compressor duty-cycle feature);
- the usual passenger and cargo loads.
