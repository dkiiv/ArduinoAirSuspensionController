// Height Control Supervisor -- ESP32 adapter (hardware I/O + task + persistence around hcs::Core).
// Design: OASMan_ESP32/docs/height-control-supervisor.md

#ifndef heightControlSupervisor_h
#define heightControlSupervisor_h

#include <user_defines.h>
#include "hcs_config.h"

#if HEIGHT_CONTROL_SUPERVISOR

// Load the persisted targets and start the supervisor task. Called from setup_tasks().
void hcsSetup();
// A preset was loaded (BLE / RF / gamepad / rise-on-start all go through loadProfileAirUp). Thread-safe.
void hcsNotifyPresetLoad(int profileIndex);
// A user moved valves directly (BLE valve bitset / gamepad). Thread-safe.
void hcsNotifyManual();
// True when the supervisor (not the legacy Wheel::maintainPressure) owns height maintenance.
bool hcsOwnsHeightMaintain();

#else

// Legacy builds: hooks compile to nothing.
inline void hcsNotifyPresetLoad(int) {}
inline void hcsNotifyManual() {}

#endif

#endif
