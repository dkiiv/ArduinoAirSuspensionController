// Height Control Supervisor (HCS) -- compile-time switches and tunables. Units: height = calibrated height % (0..100),
// pressure = psi, time = ms. ESTIMATE = not measured on a car yet (see docs/hcs-field-data.md); those and the switches
// are #ifndef-guarded so a platformio env or the simulator can override them with -D.
// Docs: OASMan_ESP32/docs/height-control-supervisor.md

#ifndef hcs_config_h
#define hcs_config_h

// ---- switches
#ifndef HEIGHT_CONTROL_SUPERVISOR
#define HEIGHT_CONTROL_SUPERVISOR false // false = legacy behaviour, supervisor not compiled in (manifold_v4_release)
#endif
#ifndef HCS_SHADOW_MODE
#define HCS_SHADOW_MODE false // decide + log ("SHADOW would ..."), never actuate; legacy maintain keeps running
#endif
#ifndef HCS_DIAG_DUMP
#define HCS_DIAG_DUMP false // "HCSD" state + "HRAW" raw sensor lines every second (threshold data: aH / aP)
#endif
#ifndef HCS_AUTO_LOWER
#define HCS_AUTO_LOWER true // allow autonomous dumps (load removed, road level, our own air, heat above the ceiling)
#endif
#define HCS_DUMP_PERIOD_MS 1000
#define HCS_TICK_MS 100
#define HCS_LOG_THROTTLE_MS 600000UL // identical repeated lines printed at most this often

// ---- motion (no speed signal): activity = EMA of |second difference| of height and pressure per corner
#define HCS_ACT_ALPHA 0.15f
#ifndef HCS_MOTION_H_THRESH
#define HCS_MOTION_H_THRESH 0.7f // height %, ESTIMATE
#endif
#ifndef HCS_MOTION_P_THRESH
#define HCS_MOTION_P_THRESH 1.2f // psi, ESTIMATE
#endif
#define HCS_MOTION_MIN_CORNERS 2
#define HCS_MOTION_START_TICKS 3
#define HCS_ABORT_TICKS 3          // motion ticks that abort a correction in flight
#define HCS_EPISODE_GAP_MS 3000
#define HCS_DRIVE_CONFIRM_MS 15000 // longer episode = DRIVING, shorter = disturbance (door, person)
#ifndef HCS_ARRIVAL_QUIET_MS
#define HCS_ARRIVAL_QUIET_MS 120000 // quiet after a drive / boot before evaluating (rides out red lights); after it
#endif                              // the car counts as parked (fast confirmation for people / cargo)
#define HCS_DISTURB_QUIET_MS 3000 // quiet after a door / person / trunk disturbance
#define HCS_EVAL_PERIOD_MS 30000

// ---- settled readings / confirmation
#define HCS_WINDOW 20 // samples per settled reading
#define HCS_STABLE_RANGE_H 1.5f
#define HCS_STABLE_RANGE_P 2.0f
#define HCS_P_SETTLE_MS 3000 // bag pressure trusted this long after the corner's valves closed
// A correction starts only after every corner held within HCS_STABLE_RANGE_H for this long with no motion: a dip
// or the bottom of a hill compresses (and moves) the car for seconds; a person or cargo stays.
#ifndef HCS_CONFIRM_MS
#define HCS_CONFIRM_MS 10000
#endif
// At a car known to be parked, right after a detected disturbance (people / cargo rock the car) and for a corner
// below min ride, the confirmation is this short, so added weight or a dropped corner is corrected within seconds.
#ifndef HCS_CONFIRM_LOAD_MS
#define HCS_CONFIRM_LOAD_MS 3000
#endif

// ---- classifier
#define HCS_DEADBAND_H 3.0f
#define HCS_AIR_SIGN_DH 1.5f // sign test: down by more than this without a pressure rise = lost air
#ifndef HCS_PRESSURE_NOISE_PSI
#define HCS_PRESSURE_NOISE_PSI 1.5f // ESTIMATE
#endif
#ifndef HCS_LOAD_FRAC
#define HCS_LOAD_FRAC 0.015f // total load change that counts as people / cargo (~28 kg), ESTIMATE
#endif
#define HCS_PREF_MIN_PSI 20.0f
#define HCS_EXTERNAL_UNLOAD_FRAC 0.35f // a bag losing this share of its pressure = jack / lift / wheel unloaded
#define HCS_EVENT_MAX_MS 900000UL      // load references stay frozen at most this long while a correction is pending

// ---- hard bounds (autonomous only)
#define HCS_FLOOR_LIFT_MARGIN 3.0f  // below min ride -> lifted to min ride + this
#define HCS_ABS_FLOOR 5.0f          // floor when min ride was never calibrated
#define HCS_CEIL_MARGIN 5.0f        // fills never target above 100 - this
#define HCS_BAG_P_MARGIN 10.0f      // fills stop at min(bagMaxPressure, MAX_PRESSURE_SAFETY) - this
#define HCS_TANK_HEADROOM_PSI 20.0f // a fill needs tank >= bag + this

// ---- pacing / watchdogs
#define HCS_ROUTINE_TIMEOUT_MS 10000 // autonomous goal routine (users keep the legacy 15 s)
#define HCS_BATCH_WATCHDOG_MS 30000
#define HCS_VALVE_MAX_OPEN_MS 12000
#define HCS_POST_CORRECTION_SETTLE_MS 6000
#define HCS_MIN_DWELL_MS 60000       // same corner not corrected again sooner
#define HCS_REVERSAL_LOCK_MS 600000  // no up-then-down on a corner without motion in between
#define HCS_MAX_BATCHES_PER_HOUR 8
#define HCS_LEAK_FAST_INTERVAL_MS (30UL * 60UL * 1000UL) // 3 refills each sooner than this = fast leak, latched
#define HCS_LEAK_FAST_COUNT 3
#define HCS_MANUAL_SETTLE_MS 10000
#define HCS_BOOT_HOLD_MS 10000
#define HCS_PERSIST_MIN_INTERVAL_MS 60000

// ---- sensor faults (electrical only)
#define HCS_RAW_H_MIN -4.0f // raw % of the 0.5-4.5 V span
#define HCS_RAW_H_MAX 104.0f
#define HCS_CAL_MIN_SPAN 5.0f
#define HCS_P_MIN_VALID -12.0f
#define HCS_P_MAX_VALID 240.0f
#define HCS_FAULT_LATCH_SAMPLES 10
#define HCS_FAULT_CLEAR_MS 300000

// ---- road level while driving (used by the arrival correction): each corner's average height per window versus the
// preset plane. Curves and braking tilt the car but cannot lower it on average; a window in which the car sat LOW
// overall (mean deficit > HCS_ROAD_HEAVE_MIN) means load or lost air. Rough roads bias the average: dropped.
#define HCS_ROAD_WINDOW_MS 120000UL
#ifndef HCS_ROAD_HEAVE_MIN
#define HCS_ROAD_HEAVE_MIN 1.0f // height %, ESTIMATE
#endif
#ifndef HCS_ROAD_ROUGH_MAX
#define HCS_ROAD_ROUGH_MAX 2.5f // window ignored above this mean height activity (x motion threshold). ESTIMATE (bench:
#endif                          // highway ~1, busy town ~2.2, rough ~3)
#define HCS_FILL_RATE_DEFAULT 2.0f // height %/s of open IN valve until learned from a fill, ESTIMATE
// At the BLE-connect edge a corner the last parked evaluation found below min ride is lifted at once (driving off on a
// bottomed corner drags the frame), if that evaluation is at most this old.
#ifndef HCS_URGENT_MAX_AGE_MS
#define HCS_URGENT_MAX_AGE_MS 900000UL
#endif

#endif
