// Height Control Supervisor (HCS) core -- platform independent (no Arduino / FreeRTOS).
//
// Owns every AUTONOMOUS height decision in height-sensor mode: is the car moving, what should each corner hold, why
// is a corner off (leak / load / terrain / ...), and may a correction run. It never touches hardware: the ESP32
// adapter (heightControlSupervisor.cpp) feeds it sensor snapshots every 100 ms and executes its commands through the
// existing Wheel goal routine. eval/hcs_sim.cpp compiles the same sources on a PC. Single-threaded.
//   hcs_core.cpp      state machine, user override, corrections, persistence, logging
//   hcs_sense.cpp     sample intake, sensor faults, motion detector
//   hcs_classify.cpp  parked evaluation: classify each corner, arbitrate, start a correction
//   hcs_drive.cpp     while driving: road level and fill pulses
// Docs: OASMan_ESP32/docs/height-control-supervisor.md

#ifndef hcs_core_h
#define hcs_core_h

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "hcs_config.h"

namespace hcs
{

// Logical corner order (WHEEL_* indices). Pins / ADC channels are the wheel's business.
static const int NC = 4;
static const int C_FP = 0, C_RP = 1, C_FD = 2, C_RD = 3;
inline bool isFront(int c) { return c == C_FP || c == C_FD; }

enum class State : uint8_t
{
    INIT,       // boot hold
    INACTIVE,   // not height mode / maintain off / safety mode
    MOTION,     // motion episode (driving or a disturbance)
    SETTLING,   // quiet, not long enough to evaluate yet
    PARKED,     // evaluated, holding targets
    CORRECTING, // a correction in flight / settling
    MANUAL,     // a user command in flight / settling: supervisor yields
    FAULT       // >= 2 height sensors faulted
};

enum class Cls : uint8_t
{
    NONE,
    WITHIN,       // inside the deadband
    AIR_LOSS,     // low, this bag lost air -> fill
    LOAD,         // low, air unchanged, total load up -> fill
    SHIFT,        // load moved between corners at constant total (terrain / someone moved) -> accept
    COUPLED,      // displaced because ANOTHER bag's air changed -> hold
    AMBIGUOUS,    // contradicting evidence -> hold
    AIR_GAIN,     // high, this bag gained air (heat) -> hold
    UNLOAD,       // high, air unchanged, total load down -> lower
    BOTTOM_GUARD, // below min ride (any cause) -> lift
    CEILING,      // above the ceiling -> lower only if it gained air
    ROAD,         // after a drive: sat off the preset level on the road -> correct by that much
    FROZEN        // height sensor fault
};

struct CornerIn
{
    float h;    // calibrated height %
    float hRaw; // raw % of the 0.5-4.5 V span, unclamped
    float p;    // bag psi
    bool inOpen, outOpen;
    bool routineActive, routineAutonomous;
    uint32_t seq; // bumps on every fresh reading
    float calMinRaw, calMaxRaw, minRide;
};

struct Inputs
{
    uint32_t now;
    CornerIn c[NC];
    float tank;
    bool presence; // isVehicleOn(): an authenticated BLE client is connected
    bool enabled;  // height-sensor mode && maintain
    bool safetyMode;
    float bagCeilPsi;       // min(bagMaxPressure, MAX_PRESSURE_SAFETY)
    float compressorOffPsi; // tank cut-off
};

enum class Cmd : uint8_t
{
    NONE,
    START, // autonomous goals on corners with g[i].active
    ABORT, // stop goals / pulses on corners with g[i].active
    PULSE  // fill pulse on the g[i].active corner for g[i].pulseMs
};

struct GoalRequest
{
    bool active;
    int8_t dir;     // +1 fill only, -1 dump only
    uint8_t target; // height %
    float ceilH, floorH, ceilP;
    uint16_t pulseMs;
};

struct Outputs
{
    Cmd cmd;
    GoalRequest g[NC];
    bool persistNow;
};

// Persisted (SPIFFS file, the store the pressure AI keeps its samples in): targets, references, learned fill rates.
// Every field is range-checked on restore.
static const uint32_t PERSIST_MAGIC = 0x48435331; // "HCS1"
static const uint8_t PERSIST_VERSION = 5;
struct PersistBlob
{
    uint32_t magic;
    uint8_t version, presetValid;
    uint8_t presetH[NC];
    uint8_t reserved[2];
    float tgt[NC]; // NAN = none
    float floorOverride[NC];
    float pRef[NC], pScale[NC], airH[NC], airP[NC];
    float fillRate[NC]; // learned: height %/s of open IN valve; NAN = not yet
};

typedef void (*LogFn)(const char *line);

// Bounded append (never writes past n, whatever vsnprintf returns).
void appendf(char *buf, size_t n, size_t &w, const char *fmt, ...);

class Core
{
public:
    void begin(uint32_t now, const PersistBlob *restored, bool shadow, LogFn log);
    void tick(const Inputs &in, Outputs &out);
    void notifyPresetLoad(const uint8_t heights[NC]); // implies notifyManual()
    void notifyManual() { pendingManual = true; }
    bool exportPersist(PersistBlob &b) const;
    int dump(char *buf, size_t n) const;
    State state() const { return st; }
    static const char *stateName(State s);
    static const char *clsName(Cls c);

private:
    struct Corner
    {
        // samples (sense)
        float wh[HCS_WINDOW], wp[HCS_WINDOW];
        int nh, ih, np, ip, ndh, ndp;
        float dh[3], dp[3], actH, actP;
        uint32_t lastSeq, idleSince, valveOpenSince;
        bool busy, valveWasOpen;
        bool hFault, pFault;
        int badH, badP;
        uint32_t goodSinceH, goodSinceP;
        bool qOk; // steadiness: height range [qLo, qHi] since qStart
        float qLo, qHi;
        uint32_t qStart, qSince;
        // what to hold and the evidence (classify)
        bool tgtValid;
        float tgt, floorOverride; // floorOverride >= 0: user holds it below min ride on purpose
        float pRef;               // load reference (frozen while an event is open)
        float pScale;             // load normaliser: pressure at the last user commit
        float airH, airP;         // air reference: (h, p) when this bag's air was last known-good
        Cls lastCls;
        uint32_t lastCorrAt, lastCorrEpisode;
        int8_t lastCorrDir;
        bool haveRefill, leakFault, leakAllowance;
        uint32_t lastRefillAt;
        int fastCount;
        // correction in flight (core)
        bool inBatch, touched;
        int8_t batchDir;
        Cls batchCls;
        float batchH0, batchP0, fillRate;
        uint32_t batchOpenMs;
        // driving (drive)
        bool hsOk, roadPending;
        float hS, pS, calmLo, calmHi;              // 1-s averages, calm band
        float sdH, roadDef, owe;                   // road window sum, last window's deficit, owed fill
        float driveAdded;                          // owed by road windows this drive (capped at HCS_OWE_MAX)
        uint32_t pulsedMs;                         // IN-valve time pulsed since the last window
    };

    Corner c[NC];
    State st;
    bool shadow;
    LogFn logFn;
    uint32_t now, bootAt, lastDt;
    float tank;
    bool tankValid;
    // motion
    bool inEpisode, driving, armedAfterDrive, bootPending, afterDisturb; // afterDisturb: the last episode was a disturbance
    uint32_t episodeStart, lastMotion, episodeId;
    int lastIncluded, lastVoting, lastStrong, motionRun, abortRun;
    // evaluation
    bool needEval, eventOpen, externalFreeze, prevPresence, prevEnabled, presetValid;
    int extCorner;
    uint32_t lastEvalAt, eventOpenSince, confirmAt, lastPostponeLogAt;
    float presetH[NC];
    // user override
    bool manualActive, presetPending, pendingManual, manualSawDrive, manualIdle;
    uint8_t pendingPreset[NC];
    uint32_t manualIdleSince;
    // correction in flight
    bool batchSettling, urgentPending, urgentBatch, ownAir;
    uint32_t batchStart, settleStart, batchT[HCS_MAX_BATCHES_PER_HOUR];
    int batchI;
    // driving
    bool roadDefValid;
    int pulseCorner;
    uint32_t sdN, calmSince, pulseStartAt, pulseEnd, pulseMs, pulseAt;
    float pulseStep, sdAct;
    // persistence + logging
    bool dirty, persistImmediate;
    uint32_t lastPersistAt, thrHash[8], thrAt[8];

    // hcs_core.cpp
    void vlog(bool throttle, const char *fmt, va_list ap);
    void logf(const char *fmt, ...);
    void logq(const char *fmt, ...); // throttled
    void tickInner(const Inputs &in, Outputs &out);
    void enterManual(const char *why, Outputs &out);
    void tickManual(const Inputs &in);
    void startBatch(const Inputs &in, Outputs &out, const bool *inB, const int8_t *dir, const float *goal, const Cls *cls,
                    const float *hm, const float *pm, const bool *pOk);
    bool urgentFloorLift(const Inputs &in, Outputs &out);
    void tickCorrecting(const Inputs &in, Outputs &out);
    void finalizeBatch();
    void abortBatch(Outputs &out, const char *why, State next);
    void recordRefill(int i, float deficit);
    void markPersist(bool immediate);
    // hcs_sense.cpp
    void ingest(const Inputs &in);
    void detectFaults(const Inputs &in, int i);
    void motionDetect();
    bool windowMean(int i, float &hm, float &pm, bool &pOk, bool &stable) const;
    uint32_t steadyRemaining(uint32_t confirmMs) const; // ms still needed; 0 = steady that long
    bool pUsable(int i) const { return !c[i].pFault && c[i].ndp >= 3; }
    // hcs_classify.cpp
    void evaluate(const Inputs &in, Outputs &out, const char *kind, bool arrival);
    Cls classify(const Inputs &in, int i, float hm, bool pOk, int loadDir, bool airElsewhere, int8_t airChg, int8_t &dir,
                 float &goal);
    const char *gate(const Inputs &in, int i, int8_t dir, bool pOk, float pm) const;
    void retargetArrival(const Inputs &in, const float *hm);
    bool planeDeficit(const float *h, float *d, const float *ref = nullptr) const;
    // hcs_drive.cpp
    void driveTick(const Inputs &in, Outputs &out);
    void roadWindow();

    // helpers
    float floorBase(const Inputs &in, int i) const;
    float floorTrigger(const Inputs &in, int i) const;
    float liftTarget(const Inputs &in, int i) const;
    float ceilH() const { return 100.0f - HCS_CEIL_MARGIN; }
    bool allIdleFor(uint32_t ms) const;
    bool anyRoutine(const Inputs &in) const;
    int heightFaultCount() const;
    int batchesLastHour() const;
};

static const char *const CN[NC] = {"FP", "RP", "FD", "RD"};
static const uint32_t HOUR_MS = 3600000UL;
static inline float fmaxf_(float a, float b) { return a > b ? a : b; }
static inline float fminf_(float a, float b) { return a < b ? a : b; }
static inline bool finite_(float v) { return !(v != v) && v < 1e30f && v > -1e30f; }

} // namespace hcs

#endif
