// Height Control Supervisor (HCS) -- compile-time switches and tunables. Every value is #ifndef-guarded so a
// platformio env or the simulator can override it with -D. Units: height = calibrated height % (0..100), pressure =
// psi, time = ms. ESTIMATE = not measured on a car yet (see docs/hcs-field-data.md).
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
#define HCS_DIAG_DUMP false // "HCSD" state + "HRAW" raw sensor lines every HCS_DUMP_PERIOD_MS (threshold data: aH / aP)
#endif
#ifndef HCS_DUMP_PERIOD_MS
#define HCS_DUMP_PERIOD_MS 1000
#endif
#ifndef HCS_AUTO_LOWER
#define HCS_AUTO_LOWER true // allow autonomous dumps (load removed, road level, heat above the ceiling)
#endif
#ifndef HCS_TICK_MS
#define HCS_TICK_MS 100
#endif
#ifndef HCS_LOG_THROTTLE_MS
#define HCS_LOG_THROTTLE_MS 600000UL // identical repeated lines printed at most this often
#endif

// ---- motion (no speed signal): activity = EMA of |second difference| of height and pressure per corner
#ifndef HCS_ACT_ALPHA
#define HCS_ACT_ALPHA 0.15f
#endif
#ifndef HCS_MOTION_H_THRESH
#define HCS_MOTION_H_THRESH 0.7f // height %, ESTIMATE
#endif
#ifndef HCS_MOTION_P_THRESH
#define HCS_MOTION_P_THRESH 1.2f // psi, ESTIMATE
#endif
#ifndef HCS_MOTION_MIN_CORNERS
#define HCS_MOTION_MIN_CORNERS 2
#endif
#ifndef HCS_MOTION_START_TICKS
#define HCS_MOTION_START_TICKS 3
#endif
#ifndef HCS_ABORT_TICKS
#define HCS_ABORT_TICKS 3 // motion ticks that abort a correction in flight
#endif
#ifndef HCS_EPISODE_GAP_MS
#define HCS_EPISODE_GAP_MS 3000
#endif
#ifndef HCS_DRIVE_CONFIRM_MS
#define HCS_DRIVE_CONFIRM_MS 15000 // longer episode = DRIVING, shorter = disturbance (door, person)
#endif
#ifndef HCS_ARRIVAL_QUIET_MS
#define HCS_ARRIVAL_QUIET_MS 120000 // quiet after a drive / boot before evaluating (rides out red lights)
#endif
#ifndef HCS_DISTURB_QUIET_MS
#define HCS_DISTURB_QUIET_MS 3000 // quiet after a door / person / trunk disturbance
#endif
#ifndef HCS_EVAL_PERIOD_MS
#define HCS_EVAL_PERIOD_MS 30000
#endif

// ---- settled readings / confirmation
#ifndef HCS_WINDOW
#define HCS_WINDOW 20 // samples per settled reading
#endif
#ifndef HCS_STABLE_RANGE_H
#define HCS_STABLE_RANGE_H 1.5f
#endif
#ifndef HCS_STABLE_RANGE_P
#define HCS_STABLE_RANGE_P 2.0f
#endif
#ifndef HCS_P_SETTLE_MS
#define HCS_P_SETTLE_MS 3000 // bag pressure trusted this long after the corner's valves closed
#endif
// A correction starts only after every corner held within HCS_STABLE_RANGE_H for this long with no motion: a dip
// or the bottom of a hill compresses (and moves) the car for seconds; a person or cargo stays.
#ifndef HCS_CONFIRM_MS
#define HCS_CONFIRM_MS 10000
#endif
// Right after a detected disturbance (people / cargo rock the car; a smooth sag at speed does not), and for a corner
// below min ride, the confirmation is this short, so added weight or a dropped corner is corrected within seconds.
#ifndef HCS_CONFIRM_LOAD_MS
#define HCS_CONFIRM_LOAD_MS 3000
#endif

// ---- classifier
#ifndef HCS_DEADBAND_H
#define HCS_DEADBAND_H 3.0f
#endif
#ifndef HCS_AIR_SIGN_DH
#define HCS_AIR_SIGN_DH 1.5f // sign test: down by more than this without a pressure rise = lost air
#endif
#ifndef HCS_PRESSURE_NOISE_PSI
#define HCS_PRESSURE_NOISE_PSI 1.5f // ESTIMATE
#endif
#ifndef HCS_LOAD_FRAC
#define HCS_LOAD_FRAC 0.015f // total load change that counts as people / cargo (~28 kg), ESTIMATE
#endif
#ifndef HCS_PREF_MIN_PSI
#define HCS_PREF_MIN_PSI 20.0f
#endif
#ifndef HCS_EXTERNAL_UNLOAD_FRAC
#define HCS_EXTERNAL_UNLOAD_FRAC 0.35f // a bag losing this share of its pressure = jack / lift / wheel unloaded
#endif
#ifndef HCS_EVENT_MAX_MS
#define HCS_EVENT_MAX_MS 900000UL // load references stay frozen at most this long while a correction is pending
#endif

// ---- hard bounds (autonomous only)
#ifndef HCS_FLOOR_LIFT_MARGIN
#define HCS_FLOOR_LIFT_MARGIN 3.0f // below min ride -> lifted to min ride + this
#endif
#ifndef HCS_ABS_FLOOR
#define HCS_ABS_FLOOR 5.0f // floor when min ride was never calibrated
#endif
#ifndef HCS_CEIL_MARGIN
#define HCS_CEIL_MARGIN 5.0f // fills never target above 100 - this
#endif
#ifndef HCS_BAG_P_MARGIN
#define HCS_BAG_P_MARGIN 10.0f // fills stop at min(bagMaxPressure, MAX_PRESSURE_SAFETY) - this
#endif
#ifndef HCS_TANK_HEADROOM_PSI
#define HCS_TANK_HEADROOM_PSI 20.0f // a fill needs tank >= bag + this
#endif

// ---- pacing / watchdogs
#ifndef HCS_ROUTINE_TIMEOUT_MS
#define HCS_ROUTINE_TIMEOUT_MS 10000 // autonomous goal routine (users keep the legacy 15 s)
#endif
#ifndef HCS_BATCH_WATCHDOG_MS
#define HCS_BATCH_WATCHDOG_MS 30000
#endif
#ifndef HCS_VALVE_MAX_OPEN_MS
#define HCS_VALVE_MAX_OPEN_MS 12000
#endif
#ifndef HCS_POST_CORRECTION_SETTLE_MS
#define HCS_POST_CORRECTION_SETTLE_MS 6000
#endif
#ifndef HCS_MIN_DWELL_MS
#define HCS_MIN_DWELL_MS 60000 // same corner not corrected again sooner
#endif
#ifndef HCS_REVERSAL_LOCK_MS
#define HCS_REVERSAL_LOCK_MS 600000 // no up-then-down on a corner without motion in between
#endif
#ifndef HCS_MAX_BATCHES_PER_HOUR
#define HCS_MAX_BATCHES_PER_HOUR 8
#endif
#ifndef HCS_LEAK_FAST_INTERVAL_MS
#define HCS_LEAK_FAST_INTERVAL_MS (30UL * 60UL * 1000UL) // 3 refills each sooner than this = fast leak, latched
#endif
#ifndef HCS_LEAK_FAST_COUNT
#define HCS_LEAK_FAST_COUNT 3
#endif
#ifndef HCS_MANUAL_SETTLE_MS
#define HCS_MANUAL_SETTLE_MS 10000
#endif
#ifndef HCS_BOOT_HOLD_MS
#define HCS_BOOT_HOLD_MS 10000
#endif
#ifndef HCS_PERSIST_MIN_INTERVAL_MS
#define HCS_PERSIST_MIN_INTERVAL_MS 60000
#endif

// ---- sensor faults (electrical only)
#ifndef HCS_RAW_H_MIN
#define HCS_RAW_H_MIN -4.0f // raw % of the 0.5-4.5 V span
#endif
#ifndef HCS_RAW_H_MAX
#define HCS_RAW_H_MAX 104.0f
#endif
#ifndef HCS_CAL_MIN_SPAN
#define HCS_CAL_MIN_SPAN 5.0f
#endif
#ifndef HCS_P_MIN_VALID
#define HCS_P_MIN_VALID -12.0f
#endif
#ifndef HCS_P_MAX_VALID
#define HCS_P_MAX_VALID 240.0f
#endif
#ifndef HCS_FAULT_LATCH_SAMPLES
#define HCS_FAULT_LATCH_SAMPLES 10
#endif
#ifndef HCS_FAULT_CLEAR_MS
#define HCS_FAULT_CLEAR_MS 300000
#endif

// ---- driving: road level. The road is level on average; curves and braking only tilt the car, they cannot lower it
// on average (the total load is constant). A window in which the car sits LOW overall (mean deficit versus the preset
// > HCS_ROAD_HEAVE_MIN: load added before leaving, a leak) tops up the corners carrying that deficit.
#ifndef HCS_ROAD_WINDOW_MS
#define HCS_ROAD_WINDOW_MS 120000UL
#endif
#ifndef HCS_ROAD_ROUGH_MAX
#define HCS_ROAD_ROUGH_MAX 2.5f // window ignored above this mean height activity (x motion threshold): rough roads bias
#endif                          // the average reading. ESTIMATE (bench: highway ~1, busy town ~2.2, rough ~3)
#ifndef HCS_ROAD_HEAVE_MIN
#define HCS_ROAD_HEAVE_MIN 1.0f // height %, ESTIMATE
#endif
#ifndef HCS_OWE_MAX
#define HCS_OWE_MAX 10.0f // most ever added per corner per drive (hard cap, whatever the readings say)
#endif
#ifndef HCS_PULSE_STEP
#define HCS_PULSE_STEP 2.0f // height % per pulse
#endif
#ifndef HCS_PULSE_MAX_MS
#define HCS_PULSE_MAX_MS 1500
#endif
#ifndef HCS_PULSE_GAP_MS
#define HCS_PULSE_GAP_MS 5000UL
#endif
#ifndef HCS_CALM_MS
#define HCS_CALM_MS 8000UL // pulses only after every corner's 1-s average held within HCS_CALM_H this long
#endif
#ifndef HCS_CALM_H
#define HCS_CALM_H 2.0f // ESTIMATE
#endif
#ifndef HCS_FILL_RATE_DEFAULT
#define HCS_FILL_RATE_DEFAULT 2.0f // height %/s of open IN valve until learned from a parked fill, ESTIMATE
#endif
// At the BLE-connect edge a corner the last parked evaluation found below min ride is lifted at once (driving off on a
// bottomed corner drags the frame), if that evaluation is at most this old.
#ifndef HCS_URGENT_MAX_AGE_MS
#define HCS_URGENT_MAX_AGE_MS 900000UL
#endif

#endif
