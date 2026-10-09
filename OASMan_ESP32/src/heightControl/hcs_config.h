// Height Control Supervisor (HCS) -- compile-time switches and tunables.
// Docs: OASMan_ESP32/docs/height-control-supervisor.md (what / why), OASMan_ESP32/docs/hcs-simulator.md (bench).
//
// Manifold-only header (not in ESP32_SHARED_LIBS, the controller is untouched). Every value is #ifndef-guarded so a
// platformio env (or the simulator) can override it with -D.
// Units: height = calibrated height % (0..100, the app's scale in height-sensor mode), pressure = psi, time = ms.
// "ESTIMATE" = not measured on a car yet; the shadow build's STATS / EVAL lines are how to measure it.

#ifndef hcs_config_h
#define hcs_config_h

// ---- master switches -------------------------------------------------------------------------------------
// false (default, what manifold_v4_release builds) = legacy behaviour, the supervisor is not compiled in.
#ifndef HEIGHT_CONTROL_SUPERVISOR
#define HEIGHT_CONTROL_SUPERVISOR false
#endif
// Shadow: decide + log every decision ("SHADOW would ..."), never actuate; legacy maintain keeps running.
#ifndef HCS_SHADOW_MODE
#define HCS_SHADOW_MODE false
#endif
// 1 Hz "HCSD" state line + "HRAW" raw sensor line on serial.
#ifndef HCS_DIAG_DUMP
#define HCS_DIAG_DUMP false
#endif
#ifndef HCS_DUMP_PERIOD_MS
#define HCS_DUMP_PERIOD_MS 1000
#endif
// Autonomous lowering: only for a measured load removal (UNLOAD) or a corner above the ceiling. Thermal
// expansion is never dumped (it reverses as the bag cools).
#ifndef HCS_AUTO_LOWER
#define HCS_AUTO_LOWER true
#endif
#ifndef HCS_TICK_MS
#define HCS_TICK_MS 100
#endif
// Identical repeated lines (periodic evaluations, refusals) are printed at most this often.
#ifndef HCS_LOG_THROTTLE_MS
#define HCS_LOG_THROTTLE_MS 600000UL
#endif

// ---- motion detector (no speed signal: driving is inferred from the sensors) ------------------------------
// Activity = EMA of |second difference| of each corner's height and bag pressure. Blind to smooth ramps (our own
// fills), sees road texture, body bounce and the onset / end of roll, pitch and heave.
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
#define HCS_MOTION_MIN_CORNERS 2 // corners that must be active at once
#endif
#ifndef HCS_MOTION_START_TICKS
#define HCS_MOTION_START_TICKS 3 // consecutive ticks to start an episode (one noisy sample is not motion)
#endif
#ifndef HCS_ABORT_TICKS
#define HCS_ABORT_TICKS 3 // consecutive motion ticks that abort a correction in flight
#endif
#ifndef HCS_EPISODE_GAP_MS
#define HCS_EPISODE_GAP_MS 3000 // quiet that ends an episode
#endif
#ifndef HCS_DRIVE_CONFIRM_MS
#define HCS_DRIVE_CONFIRM_MS 15000 // an episode longer than this is DRIVING, shorter is a disturbance
#endif
#ifndef HCS_ARRIVAL_QUIET_MS
#define HCS_ARRIVAL_QUIET_MS 120000 // quiet needed after a drive or a boot (rides out red lights)
#endif
#ifndef HCS_DISTURB_QUIET_MS
#define HCS_DISTURB_QUIET_MS 8000 // quiet needed after a door / person / trunk disturbance
#endif
#ifndef HCS_EVAL_PERIOD_MS
#define HCS_EVAL_PERIOD_MS 30000 // re-evaluate this often while parked (slow leaks)
#endif
#ifndef HCS_STATS_PERIOD_MS
#define HCS_STATS_PERIOD_MS 600000UL // STATS lines (threshold tuning data), see docs
#endif

// ---- settled readings -------------------------------------------------------------------------------------
#ifndef HCS_WINDOW
#define HCS_WINDOW 20 // samples averaged for a settled reading
#endif
#ifndef HCS_STABLE_RANGE_H
#define HCS_STABLE_RANGE_H 1.5f // max spread inside the window
#endif
#ifndef HCS_STABLE_RANGE_P
#define HCS_STABLE_RANGE_P 2.0f
#endif
#ifndef HCS_P_SETTLE_MS
#define HCS_P_SETTLE_MS 3000 // bag pressure trusted this long after the corner's valves closed (flow offset)
#endif

// ---- classifier -------------------------------------------------------------------------------------------
#ifndef HCS_DEADBAND_H
#define HCS_DEADBAND_H 3.0f // corrections only outside +-this (no hunting)
#endif
// Sign test noise: a bag whose air is unchanged moves along its own p(h) curve, so dh and dp have opposite signs.
// Down by more than HCS_AIR_SIGN_DH without a pressure rise of more than HCS_PRESSURE_NOISE_PSI = it lost air.
#ifndef HCS_AIR_SIGN_DH
#define HCS_AIR_SIGN_DH 1.5f
#endif
#ifndef HCS_PRESSURE_NOISE_PSI
#define HCS_PRESSURE_NOISE_PSI 1.5f // ESTIMATE
#endif
// Total-load test: mean over corners of dp / (pressure at the last commit). Terrain / slopes / cornering move load
// between corners (mean ~0); people and cargo add it. 0.015 = 1.5 % of the car's weight (~28 kg). ESTIMATE.
#ifndef HCS_LOAD_FRAC
#define HCS_LOAD_FRAC 0.015f
#endif
#ifndef HCS_PREF_MIN_PSI
#define HCS_PREF_MIN_PSI 20.0f // floor for the normaliser (nearly empty bags)
#endif
// One corner losing more than this share of its pressure = jack / lift / wheel unsupported -> freeze everything.
#ifndef HCS_EXTERNAL_UNLOAD_FRAC
#define HCS_EXTERNAL_UNLOAD_FRAC 0.35f
#endif
// Every correction must be wanted by two evaluations at least this far apart with no motion in between. A dip /
// the bottom of a hill compresses all four corners for a second or two; a person or cargo stays.
#ifndef HCS_CONFIRM_MS
#define HCS_CONFIRM_MS 10000
#endif
// An event (load, leak) keeps the load references frozen until resolved, at most this long.
#ifndef HCS_EVENT_MAX_MS
#define HCS_EVENT_MAX_MS 900000UL
#endif
// After a drive, an accepted target within this of the preset snaps back to the preset (no drift over trips).
#ifndef HCS_ANCHOR_TOL
#define HCS_ANCHOR_TOL 3.0f
#endif
// A correction that lands further than this from its target gets ONE completion attempt.
#ifndef HCS_LAND_TOL
#define HCS_LAND_TOL 1.0f
#endif

// ---- hard bounds (autonomous only; user commands are never restricted) ------------------------------------
#ifndef HCS_FLOOR_LIFT_MARGIN
#define HCS_FLOOR_LIFT_MARGIN 3.0f // below min ride -> lifted to min ride + this
#endif
#ifndef HCS_ABS_FLOOR
#define HCS_ABS_FLOOR 5.0f // floor used when min ride was never calibrated
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

// ---- pacing / watchdogs -----------------------------------------------------------------------------------
#ifndef HCS_ROUTINE_TIMEOUT_MS
#define HCS_ROUTINE_TIMEOUT_MS 10000 // autonomous goal routine timeout (users keep the legacy 15 s)
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
#define HCS_MAX_BATCHES_PER_HOUR 8 // x 10 s routine timeout = at most 80 s of autonomous valve time per hour
#endif
// Leaks: every refill logs a rate. Only a FAST leak latches (3 refills each < 30 min apart: a burst bag / fitting);
// a latched corner gets one refill per BLE (re)connect. Slow leaks are refilled indefinitely.
#ifndef HCS_LEAK_FAST_INTERVAL_MS
#define HCS_LEAK_FAST_INTERVAL_MS (30UL * 60UL * 1000UL)
#endif
#ifndef HCS_LEAK_FAST_COUNT
#define HCS_LEAK_FAST_COUNT 3
#endif
#ifndef HCS_MANUAL_SETTLE_MS
#define HCS_MANUAL_SETTLE_MS 10000 // after a preset / valve command, wait this long idle before re-baselining
#endif
#ifndef HCS_BOOT_HOLD_MS
#define HCS_BOOT_HOLD_MS 10000
#endif
#ifndef HCS_PERSIST_MIN_INTERVAL_MS
#define HCS_PERSIST_MIN_INTERVAL_MS 60000 // NVS writes at most this often (except user commits)
#endif

// ---- sensor faults (electrical only) ----------------------------------------------------------------------
#ifndef HCS_RAW_H_MIN
#define HCS_RAW_H_MIN -4.0f // raw % of the 0.5-4.5 V span; outside = wire break / short
#endif
#ifndef HCS_RAW_H_MAX
#define HCS_RAW_H_MAX 104.0f
#endif
#ifndef HCS_CAL_MIN_SPAN
#define HCS_CAL_MIN_SPAN 5.0f // calibration narrower than this = uncalibrated
#endif
#ifndef HCS_P_MIN_VALID
#define HCS_P_MIN_VALID -12.0f // empty bag reads a little below 0; an open wire reads ~-29
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

// ---- cruise top-up: slow leak compensation while DRIVING (road trips) -------------------------------------
// Open-loop, fill-only, one-corner pulses, decided on 2-minute averages, only in steady driving.
// Leak signature (rigid body): the leaking bag and its diagonal partner unload (load warp), and versus the trip
// reference the leaking bag went DOWN without its pressure rising. A dip / hill compresses all four corners and
// raises all four pressures: no warp, no "down without pressure rise" -> never a pulse.
#ifndef HCS_CRUISE_TOPUP
#define HCS_CRUISE_TOPUP true
#endif
#ifndef HCS_CRUISE_TAU_MS
#define HCS_CRUISE_TAU_MS 120000UL
#endif
#ifndef HCS_CRUISE_MIN_DRIVE_MS
#define HCS_CRUISE_MIN_DRIVE_MS 600000UL // trip reference taken after 10 min of driving
#endif
#ifndef HCS_CRUISE_WARP
#define HCS_CRUISE_WARP 0.006f // load warp (fraction) that says which diagonal unloaded. ESTIMATE
#endif
#ifndef HCS_CRUISE_SIGN_DH
#define HCS_CRUISE_SIGN_DH 1.5f
#endif
#ifndef HCS_CRUISE_PERSIST_MS
#define HCS_CRUISE_PERSIST_MS 300000UL // net 5 min of evidence
#endif
#ifndef HCS_CRUISE_STEADY_DEV
#define HCS_CRUISE_STEADY_DEV 2.0f // 1-s roll / pitch within this of the 2-min mean = steady
#endif
#ifndef HCS_CRUISE_STEADY_MS
#define HCS_CRUISE_STEADY_MS 20000UL
#endif
#ifndef HCS_CRUISE_STEP
#define HCS_CRUISE_STEP 2.0f // height % per pulse
#endif
#ifndef HCS_CRUISE_PULSE_DEFAULT_MS
#define HCS_CRUISE_PULSE_DEFAULT_MS 400 // until the corner's fill rate is known from parked fills
#endif
#ifndef HCS_CRUISE_PULSE_MAX_MS
#define HCS_CRUISE_PULSE_MAX_MS 1500
#endif
#ifndef HCS_CRUISE_DWELL_MS
#define HCS_CRUISE_DWELL_MS 300000UL
#endif
#ifndef HCS_CRUISE_MAX_PER_HOUR
#define HCS_CRUISE_MAX_PER_HOUR 4
#endif

#endif
