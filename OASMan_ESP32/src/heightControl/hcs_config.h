// Height Control Supervisor (HCS) -- compile-time switches and tunables.
//
// Design + rationale for every number here: OASMan_ESP32/docs/height-control-supervisor.md
//
// Manifold-only header (deliberately NOT in ESP32_SHARED_LIBS/user_defines.h so the controller firmware
// is untouched). Every value is #ifndef-guarded so a platformio env can override it with -D.
//
// Units: heights are calibrated height % (0..100, same scale the app shows in height-sensor mode),
// pressures are psi, times are ms.
//
// IMPORTANT: almost every threshold below is an ENGINEERING ESTIMATE that has not yet been measured on the
// car. The intended rollout is: flash manifold_v4_hcs_shadow (decides + logs, never actuates), collect the
// "HCS" serial log through a drive, a hill-spot park and a load test, tune, then flash manifold_v4_hcs.

#ifndef hcs_config_h
#define hcs_config_h

// ---------------------------------------------------------------------------------------------------------
// Master switches
// ---------------------------------------------------------------------------------------------------------

// Compiles the supervisor in. false (the default, and what manifold_v4_release builds) = legacy behaviour,
// byte-for-byte the same control path as before this feature existed.
#ifndef HEIGHT_CONTROL_SUPERVISOR
#define HEIGHT_CONTROL_SUPERVISOR false
#endif

// Shadow mode: the supervisor runs, classifies and logs every decision ("HCS ... SHADOW would ...") but
// never actuates; the legacy maintain-pressure path keeps running exactly as in release.
#ifndef HCS_SHADOW_MODE
#define HCS_SHADOW_MODE false
#endif

// Periodic one-line state dump on serial (diag builds only).
#ifndef HCS_DIAG_DUMP
#define HCS_DIAG_DUMP false
#endif
#ifndef HCS_DUMP_PERIOD_MS
#define HCS_DUMP_PERIOD_MS 1000
#endif
// Repeated identical decision lines (periodic re-evaluation, refusals, vetoes) are printed at most this often.
#ifndef HCS_LOG_THROTTLE_MS
#define HCS_LOG_THROTTLE_MS 600000UL
#endif

// Autonomous LOWERING (dump) is only ever used for two classes: a measured load REMOVAL (UNLOAD) and a corner
// above the calibrated ceiling. Thermal expansion is never dumped (it goes away by itself as the bag cools).
#ifndef HCS_AUTO_LOWER
#define HCS_AUTO_LOWER true
#endif

// ---------------------------------------------------------------------------------------------------------
// Sampling / motion (driving + disturbance) detector
// ---------------------------------------------------------------------------------------------------------
#ifndef HCS_TICK_MS
#define HCS_TICK_MS 100
#endif
// Activity = EMA of |second difference| of each corner's height / bag pressure. A second difference is blind
// to the smooth ramp of our own fills but sees road input, body bounce and roll/pitch onsets.
#ifndef HCS_ACT_ALPHA
#define HCS_ACT_ALPHA 0.15f // ~0.6 s time constant at 10 Hz
#endif
#ifndef HCS_MOTION_H_THRESH
#define HCS_MOTION_H_THRESH 0.7f // height %, mean |d2|. ESTIMATE: tune from parked vs driving diag logs. ; was: 0.6 (sim: too close to noise)
#endif
#ifndef HCS_MOTION_P_THRESH
#define HCS_MOTION_P_THRESH 1.2f // psi, mean |d2|. ESTIMATE. ; was: 0.9 (sim: 2 corners crossed it on noise alone)
#endif
// At least this many (non-actuated, healthy) corners must show activity for the car to count as moving.
#ifndef HCS_MOTION_MIN_CORNERS
#define HCS_MOTION_MIN_CORNERS 2
#endif
// Field statistics for tuning the two thresholds above (one STATS line per corner and context every period):
// histogram of the detector's activity metric in multiples of its threshold, buckets
// [<.25 | .25-.5 | .5-1 | 1-2 | 2-4 | >4] x threshold, for QUIET (parked, no motion episode) and DRIVE, plus the
// parked standard deviation of height and pressure. Target: QUIET mass in the first two buckets, DRIVE above 1.
#ifndef HCS_STATS_PERIOD_MS
#define HCS_STATS_PERIOD_MS 600000UL
#endif

// Consecutive motion ticks needed to START an episode (single-sample noise spikes are not motion).
#ifndef HCS_MOTION_START_TICKS
#define HCS_MOTION_START_TICKS 3
#endif
// During a correction, the motion condition must hold this many consecutive ticks to abort (one noisy sample
// must not kill a fill; 3 ticks = 300 ms keeps drive-off aborts well under a second).
#ifndef HCS_ABORT_TICKS
#define HCS_ABORT_TICKS 3
#endif
// A motion episode ends after this much continuous quiet.
#ifndef HCS_EPISODE_GAP_MS
#define HCS_EPISODE_GAP_MS 3000
#endif
// A motion episode longer than this is treated as DRIVING (vs. a door / person / bag disturbance).
#ifndef HCS_DRIVE_CONFIRM_MS
#define HCS_DRIVE_CONFIRM_MS 15000
#endif
// Continuous quiet required before ANY evaluation after a drive (long enough to ride out red lights and
// most stop-and-go stops). Also used after boot, since we cannot know what the car did while we were down.
#ifndef HCS_ARRIVAL_QUIET_MS
#define HCS_ARRIVAL_QUIET_MS 120000
#endif
// Continuous quiet required after a short disturbance (someone got in/out, trunk loaded).
#ifndef HCS_DISTURB_QUIET_MS
#define HCS_DISTURB_QUIET_MS 8000
#endif
// While parked and quiet, re-evaluate (slow leak / thermal drift) this often.
#ifndef HCS_EVAL_PERIOD_MS
#define HCS_EVAL_PERIOD_MS 30000
#endif
// Settled-value window: classification uses the mean of the last N valve-closed samples, and requires the
// spread inside the window to be small.
#ifndef HCS_WINDOW
#define HCS_WINDOW 20
#endif
#ifndef HCS_STABLE_RANGE_H
#define HCS_STABLE_RANGE_H 1.5f
#endif
#ifndef HCS_STABLE_RANGE_P
#define HCS_STABLE_RANGE_P 2.0f
#endif
// Bag pressure is only trusted once that corner's valves have been closed this long (flow offset gone).
#ifndef HCS_P_SETTLE_MS
#define HCS_P_SETTLE_MS 3000
#endif

// ---------------------------------------------------------------------------------------------------------
// Classifier
// ---------------------------------------------------------------------------------------------------------
// A corner is corrected only when it is outside its target by more than this (hysteresis vs. the exact
// landing of the goal routine -> no hunting).
#ifndef HCS_DEADBAND_H
#define HCS_DEADBAND_H 3.0f
#endif
// Per-corner pressure noise (used for the pressure-reference bookkeeping and logs).
#ifndef HCS_PRESSURE_NOISE_PSI
#define HCS_PRESSURE_NOISE_PSI 1.5f
#endif
// AIR CONTENT index of a bag: air ~ p * V, V ~ (h + L0)  ->  a = p * (h + L0).
// Load, terrain, cornering, braking and a NEIGHBOUR's leak or fill change a bag's p and h but not its air mass;
// only a leak, temperature or our own valves do. A corner whose air index fell by more than HCS_AIR_FRAC lost air.
// L0 = bag volume at h = 0 expressed in height-% units. Unknown for a given bag -> learned per corner from load
// events (people in/out: air constant, p and h change) and logged; this is the starting value.
#ifndef HCS_BAG_L0_DEFAULT
#define HCS_BAG_L0_DEFAULT 30.0f
#endif
#ifndef HCS_BAG_L0_MIN
#define HCS_BAG_L0_MIN 2.0f
#endif
#ifndef HCS_BAG_L0_MAX
#define HCS_BAG_L0_MAX 150.0f
#endif
#ifndef HCS_AIR_FRAC
#define HCS_AIR_FRAC 0.03f
#endif
// The magnitude test above needs a trustworthy L0; until a corner has this many learned samples only the
// L0-free SIGN test is used: a bag with unchanged air moves along its own p(h) curve, so dp and dh always
// have opposite signs. Down without a pressure rise (or up without a pressure drop) = its air changed.
#ifndef HCS_L0_TRUST_SAMPLES
#define HCS_L0_TRUST_SAMPLES 3
#endif
// Height change (vs the air reference) needed before the sign test says anything.
#ifndef HCS_AIR_SIGN_DH
#define HCS_AIR_SIGN_DH 1.5f
#endif
// Total-load test: mean over corners of (dp / pRef). Load is redistributed by slopes, crowns, cornering and
// braking (mean ~0) but changed by people / cargo (mean != 0). Fraction, i.e. 0.015 = 1.5 % of vehicle load.
#ifndef HCS_LOAD_FRAC
#define HCS_LOAD_FRAC 0.015f
#endif
// A single corner losing more than this fraction of its bag pressure is not a person leaving the car; it is a
// jack, a lift, a wheel in a pothole/kerb drop: EXTERNAL support -> freeze all autonomous actuation.
#ifndef HCS_EXTERNAL_UNLOAD_FRAC
#define HCS_EXTERNAL_UNLOAD_FRAC 0.35f
#endif
// pRef floor used for the fractional maths (avoid divide-by-small on nearly empty bags).
#ifndef HCS_PREF_MIN_PSI
#define HCS_PREF_MIN_PSI 20.0f
#endif
// An "event" (load arrives, leak, ...) stays open while any corner still wants a correction; pressure references
// are frozen while it is open so the evidence for the event (e.g. the total-load rise) is not erased by
// re-referencing the corners that were already corrected. Force-closed after this long.
#ifndef HCS_EVENT_MAX_MS
#define HCS_EVENT_MAX_MS 900000UL
#endif
// After a drive, a corner whose (accepted) target lands within this of the active preset snaps back to the
// preset (anchor; stops the reference random-walking away from the preset over many trips).
#ifndef HCS_ANCHOR_TOL
#define HCS_ANCHOR_TOL 3.0f
#endif

// ---------------------------------------------------------------------------------------------------------
// Cruise top-up: slow-leak compensation WHILE DRIVING (long road trips). A separate, deliberately weaker path:
// open-loop, fill-only, single-corner pulses, decided on 2-minute averages (cornering / braking / bumps
// average out; a leak does not), only during steady driving, aborted on any roll / pitch excursion.
// ---------------------------------------------------------------------------------------------------------
#ifndef HCS_CRUISE_TOPUP
#define HCS_CRUISE_TOPUP true
#endif
#ifndef HCS_CRUISE_TAU_MS
#define HCS_CRUISE_TAU_MS 120000UL // long-average time constant
#endif
#ifndef HCS_CRUISE_MIN_DRIVE_MS
#define HCS_CRUISE_MIN_DRIVE_MS 600000UL // driving at least 10 min (averages settled, bags warm)
#endif
// Leak signal while driving, on the 2-minute averages. On a rigid body one bag losing air does NOT make one corner
// low (the four lengths stay coplanar on a road); it tilts the body toward that corner and puts a LOAD WARP on the
// car: that bag and its diagonal partner unload, the other diagonal loads up. Cornering / braking (moments), people
// (point loads) and road camber (planar) produce ~no warp, and road-surface warp averages out over 2 min.
//  - air-index mode (all four L0 learned): bag air content vs its reference, differential vs the median bag.
//  - warp mode (L0-free fallback): load warp says which diagonal unloaded (> HCS_CRUISE_WARP / 2) and the sign test
//    below (vs the trip reference) says which bag of that diagonal lost air.
#ifndef HCS_CRUISE_AIR_FRAC
#define HCS_CRUISE_AIR_FRAC 0.04f
#endif
#ifndef HCS_CRUISE_WARP
#define HCS_CRUISE_WARP 0.012f
#endif
// Leaker identification (L0-free): versus a trip reference taken from the 2-min averages at minute 10, the bag
// losing air goes DOWN while its pressure does NOT rise (its diagonal partner also unloads but goes UP).
#ifndef HCS_CRUISE_SIGN_DH
#define HCS_CRUISE_SIGN_DH 1.5f
#endif
#ifndef HCS_CRUISE_PERSIST_MS
#define HCS_CRUISE_PERSIST_MS 300000UL // ...continuously for 5 min
#endif
#ifndef HCS_CRUISE_STEADY_DEV
#define HCS_CRUISE_STEADY_DEV 2.0f // 1 s roll / pitch deviation from the 2 min mean that still counts as steady
#endif
#ifndef HCS_CRUISE_STEADY_MS
#define HCS_CRUISE_STEADY_MS 20000UL
#endif
#ifndef HCS_CRUISE_STEP
#define HCS_CRUISE_STEP 2.0f // max height % added per pulse
#endif
#ifndef HCS_CRUISE_PULSE_MAX_MS
#define HCS_CRUISE_PULSE_MAX_MS 1500
#endif
#ifndef HCS_CRUISE_PULSE_DEFAULT_MS
#define HCS_CRUISE_PULSE_DEFAULT_MS 400 // until this corner's fill rate has been learned from parked fills
#endif
#ifndef HCS_CRUISE_DWELL_MS
#define HCS_CRUISE_DWELL_MS 300000UL
#endif
#ifndef HCS_CRUISE_MAX_PER_HOUR
#define HCS_CRUISE_MAX_PER_HOUR 4
#endif

// ---------------------------------------------------------------------------------------------------------
// Hard bounds (autonomous actuation only; user commands are not restricted by the supervisor)
// ---------------------------------------------------------------------------------------------------------
// Floor: below heightCalMinRide the corner is lifted (BOTTOM_GUARD) to minRide + this margin.
#ifndef HCS_FLOOR_LIFT_MARGIN
#define HCS_FLOOR_LIFT_MARGIN 3.0f
#endif
// If heightCalMinRide was never calibrated (stored 0), this absolute floor is used instead.
#ifndef HCS_ABS_FLOOR
#define HCS_ABS_FLOOR 5.0f
#endif
// Ceiling: autonomous fills never target above (100 - margin) and the routine is cut at (100 - margin + 1).
#ifndef HCS_CEIL_MARGIN
#define HCS_CEIL_MARGIN 5.0f
#endif
// Bag pressure ceiling for autonomous fills = min(bagMaxPressure, MAX_PRESSURE_SAFETY) - this margin. Checked
// on the settled reading before a fill and on the raw (flow-inflated, i.e. conservative) reading during it.
#ifndef HCS_BAG_P_MARGIN
#define HCS_BAG_P_MARGIN 10.0f
#endif
// A fill only starts if tank >= bag + headroom (otherwise air would flow the wrong way / crawl).
#ifndef HCS_TANK_HEADROOM_PSI
#define HCS_TANK_HEADROOM_PSI 20.0f
#endif

// ---------------------------------------------------------------------------------------------------------
// Actuation pacing, budgets, watchdogs
// ---------------------------------------------------------------------------------------------------------
// Per-routine timeout for autonomous goals (the legacy ROUTINE_TIMEOUT_MS of 15 s still applies to users).
#ifndef HCS_ROUTINE_TIMEOUT_MS
#define HCS_ROUTINE_TIMEOUT_MS 10000
#endif
// Supervisor-side watchdog on a whole autonomous batch (routine + settle + barrier rechecks).
#ifndef HCS_BATCH_WATCHDOG_MS
#define HCS_BATCH_WATCHDOG_MS 30000
#endif
// Any valve of an autonomous batch open continuously longer than this -> force close + abort.
#ifndef HCS_VALVE_MAX_OPEN_MS
#define HCS_VALVE_MAX_OPEN_MS 12000
#endif
// After a batch, wait this long (valves closed, quiet) before measuring the result.
#ifndef HCS_POST_CORRECTION_SETTLE_MS
#define HCS_POST_CORRECTION_SETTLE_MS 6000
#endif
// A batch that ends further than this from its target (routine timeout, tank ran low) gets ONE follow-up
// "completion" attempt after the dwell, with the batch's original classification (not re-classified).
#ifndef HCS_LAND_TOL
#define HCS_LAND_TOL 1.0f
#endif
// Same corner may not be corrected again sooner than this.
#ifndef HCS_MIN_DWELL_MS
#define HCS_MIN_DWELL_MS 60000
#endif
// A corner corrected in one direction may not be corrected in the other direction within this window unless a
// fresh motion episode (someone got in/out, the car moved) happened in between. No-oscillation guarantee.
#ifndef HCS_REVERSAL_LOCK_MS
#define HCS_REVERSAL_LOCK_MS 600000
#endif
// Rolling one-hour budget for autonomous batches and autonomous fill time.
#ifndef HCS_MAX_BATCHES_PER_HOUR
#define HCS_MAX_BATCHES_PER_HOUR 8
#endif
#ifndef HCS_MAX_FILL_MS_PER_HOUR
#define HCS_MAX_FILL_MS_PER_HOUR 90000
#endif
// Leak policy. Slow leaks are refilled indefinitely (bounded by the hourly budgets + compressor duty cap);
// every refill logs the estimated leak rate. Only a FAST leak latches: HCS_LEAK_FAST_COUNT consecutive refills
// each less than HCS_LEAK_FAST_INTERVAL_MS after the previous one (i.e. losing a full deadband in < 30 min,
// a burst bag / failed fitting where pumping all night only drains the battery). A latched corner gets one
// refill each time a BLE client newly connects. ; was: 4 refills in 6 h (too strict for a 1-2 psi/h leak)
#ifndef HCS_LEAK_FAST_INTERVAL_MS
#define HCS_LEAK_FAST_INTERVAL_MS (30UL * 60UL * 1000UL)
#endif
#ifndef HCS_LEAK_FAST_COUNT
#define HCS_LEAK_FAST_COUNT 3
#endif
// After a user preset / manual valve move, wait this long with everything idle before re-baselining.
#ifndef HCS_MANUAL_SETTLE_MS
#define HCS_MANUAL_SETTLE_MS 10000
#endif
// Boot hold: fill the sample buffers before doing anything.
#ifndef HCS_BOOT_HOLD_MS
#define HCS_BOOT_HOLD_MS 10000
#endif
// Persist (NVS) at most this often, except on user preset / manual commits.
#ifndef HCS_PERSIST_MIN_INTERVAL_MS
#define HCS_PERSIST_MIN_INTERVAL_MS 60000
#endif

// ---------------------------------------------------------------------------------------------------------
// Sensor fault detection
// ---------------------------------------------------------------------------------------------------------
// Raw height % of the 0.5-4.5 V span (unclamped). Outside this band = wire break / short / lost ground.
#ifndef HCS_RAW_H_MIN
#define HCS_RAW_H_MIN -4.0f
#endif
#ifndef HCS_RAW_H_MAX
#define HCS_RAW_H_MAX 104.0f
#endif
// Raw reading further than this outside the calibrated min..max travel = implausible (linkage off / drift).
#ifndef HCS_CAL_TRAVEL_MARGIN
#define HCS_CAL_TRAVEL_MARGIN 15.0f
#endif
// Calibration with less span than this (raw %) is treated as uncalibrated.
#ifndef HCS_CAL_MIN_SPAN
#define HCS_CAL_MIN_SPAN 5.0f
#endif
// A healthy sensor on an empty bag (show / air-out preset) can read a few psi below 0; an open wire reads about
// -29 psi (0 V vs the 0.5 V zero). ; was: -5
#ifndef HCS_P_MIN_VALID
#define HCS_P_MIN_VALID -12.0f
#endif
#ifndef HCS_P_MAX_VALID
#define HCS_P_MAX_VALID 240.0f
#endif
// Consecutive bad samples to latch a fault / continuous good time to clear it.
#ifndef HCS_FAULT_LATCH_SAMPLES
#define HCS_FAULT_LATCH_SAMPLES 10
#endif
#ifndef HCS_FAULT_CLEAR_MS
#define HCS_FAULT_CLEAR_MS 300000
#endif
// Stuck height sensor: silent (activity below this) while >= 2 other corners show motion during a drive.
#ifndef HCS_STUCK_ACT
#define HCS_STUCK_ACT 0.05f
#endif
#ifndef HCS_STUCK_MS
#define HCS_STUCK_MS 60000
#endif
// After a batch: valve opened, bag pressure moved by >= this, but height moved < HCS_NO_RESPONSE_DH ->
// height sensor does not follow the corner (or the wheel is hanging): fault the corner.
#ifndef HCS_NO_RESPONSE_DP
#define HCS_NO_RESPONSE_DP 3.0f
#endif
#ifndef HCS_NO_RESPONSE_DH
#define HCS_NO_RESPONSE_DH 0.5f
#endif

// Valve <-> sensor mapping check after every batch: if a corner that was NOT actuated moved more than
// RATIO x the actuated corner and by at least MIN_DH, the per-corner valve / height / pressure wiring does not
// match (e.g. a pin-map reorder applied to valves + pressure but not height) -> global freeze until reboot.
#ifndef HCS_MAPPING_RATIO
#define HCS_MAPPING_RATIO 1.5f
#endif
#ifndef HCS_MAPPING_MIN_DH
#define HCS_MAPPING_MIN_DH 2.0f
#endif
// Same check live, during the batch: an idle corner has moved >= this while every actuated corner moved
// < HCS_MAPPING_LIVE_STILL -> abort immediately (limits a wrong-corner fill to a few %).
#ifndef HCS_MAPPING_LIVE_DH
#define HCS_MAPPING_LIVE_DH 3.0f
#endif
#ifndef HCS_MAPPING_LIVE_STILL
#define HCS_MAPPING_LIVE_STILL 1.0f
#endif

// ---------------------------------------------------------------------------------------------------------
// Compressor protection (applies to every compressor run in HCS builds, user-triggered or not)
// ---------------------------------------------------------------------------------------------------------
#ifndef HCS_COMPRESSOR_MAX_RUN_MS
#define HCS_COMPRESSOR_MAX_RUN_MS (8UL * 60UL * 1000UL)
#endif
#ifndef HCS_COMPRESSOR_COOLDOWN_MS
#define HCS_COMPRESSOR_COOLDOWN_MS (10UL * 60UL * 1000UL)
#endif

#endif
