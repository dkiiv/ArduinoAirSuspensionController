// Height Control Supervisor (HCS) core -- platform independent (no Arduino / FreeRTOS).
//
// Owns every AUTONOMOUS height decision in height-sensor mode: is the car moving, what should each corner hold,
// why is a corner off (leak / load / terrain / ...), and is a correction allowed. It never touches hardware: the
// ESP32 adapter (heightControlSupervisor.cpp) feeds it sensor snapshots every 100 ms and executes its commands
// through the existing Wheel goal routine. eval/hcs_sim.cpp compiles the same sources on a PC.
//
// Source map (all one class, split by job):
//   hcs_core.cpp      state machine, user override, correction execution, persistence, logging
//   hcs_sense.cpp     sample intake, sensor faults, motion detector, STATS
//   hcs_classify.cpp  evaluation: classify each corner, arbitrate, start a correction
//   hcs_cruise.cpp    cruise top-up (slow leak while driving)
// Single-threaded: every method runs on the one supervisor task (or the simulator).
// Docs: OASMan_ESP32/docs/height-control-supervisor.md

#ifndef hcs_core_h
#define hcs_core_h

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "hcs_config.h"

namespace hcs
{

// Logical corner order (WHEEL_* indices). Pins / ADC channels are the wheel's business, never the supervisor's.
static const int NC = 4;
static const int C_FP = 0, C_RP = 1, C_FD = 2, C_RD = 3;
inline bool isFront(int c) { return c == C_FP || c == C_FD; }

enum class State : uint8_t
{
    INIT,       // boot hold, buffers filling
    INACTIVE,   // not height mode / maintain off / safety mode: observe only
    MOTION,     // motion episode in progress (driving or a disturbance)
    SETTLING,   // quiet, not long enough to evaluate yet
    PARKED,     // evaluated, holding targets
    CORRECTING, // an autonomous correction is in flight / settling
    MANUAL,     // a user command is in flight / settling: supervisor yields
    FAULT       // >= 2 height sensors faulted: no autonomous actuation
};

enum class Cls : uint8_t
{
    NONE,
    WITHIN,       // inside the deadband
    AIR_LOSS,     // low and this bag lost air (leak / cooling) -> fill
    LOAD,         // low, air unchanged, total load up -> fill
    SHIFT,        // load moved between corners at ~constant total (terrain / crown / someone moved) -> accept
    COUPLED,      // displaced because ANOTHER bag's air changed -> hold (comes back when that one is fixed)
    AMBIGUOUS,    // contradicting evidence -> hold
    AIR_GAIN,     // high and this bag gained air (thermal) -> hold, never dump
    UNLOAD,       // high, air unchanged, total load down -> lower (HCS_AUTO_LOWER)
    BOTTOM_GUARD, // below the calibrated minimum ride height -> lift
    CEILING,      // above the autonomous ceiling -> lower (HCS_AUTO_LOWER)
    FROZEN        // height sensor fault -> never actuated
};

struct CornerIn
{
    float h;    // calibrated height %
    float hRaw; // raw % of the 0.5-4.5 V span, unclamped (wire-break check)
    float p;    // bag psi
    bool inOpen, outOpen;
    bool routineActive;     // a goal routine is flagged on this wheel
    bool routineAutonomous; // ...started by the supervisor
    uint32_t seq;           // bumps on every fresh reading from the wheel task
    float calMinRaw, calMaxRaw, minRide;
};

struct Inputs
{
    uint32_t now;
    CornerIn c[NC];
    float tank;
    bool presence; // isVehicleOn(): with BT ignition = an authenticated BLE client is connected
    bool enabled;  // height-sensor mode && maintain
    bool safetyMode;
    float bagCeilPsi;       // min(bagMaxPressure, MAX_PRESSURE_SAFETY)
    float compressorOffPsi; // tank cut-off
};

enum class Cmd : uint8_t
{
    NONE,
    START, // start autonomous goals on corners with g[i].active
    ABORT, // stop autonomous goals / pulses on corners with g[i].active
    PULSE  // cruise top-up: open IN on the g[i].active corner for g[i].pulseMs
};

struct GoalRequest
{
    bool active;
    int8_t dir;     // +1 fill only, -1 dump only
    uint8_t target; // height %
    float ceilH;    // stop filling at/above this height
    float floorH;   // stop dumping at/below this height
    float ceilP;    // stop filling at/above this (flowing, i.e. conservative) bag psi
    uint16_t pulseMs;
};

struct Outputs
{
    Cmd cmd;
    GoalRequest g[NC];
    bool persistNow;
};

// Persisted (NVS) so a reboot does not forget what the car should hold. Range-checked on restore (begin()).
static const uint32_t PERSIST_MAGIC = 0x48435331; // "HCS1"
static const uint8_t PERSIST_VERSION = 3;
struct PersistBlob
{
    uint32_t magic;
    uint8_t version;
    uint8_t presetValid;
    uint8_t presetH[NC];
    uint8_t reserved[2];
    float tgt[NC];           // NAN = no target
    float floorOverride[NC]; // < 0 = none (user holds the corner below min ride, e.g. a show preset)
    float pRef[NC];
    float pScale[NC];
    float airH[NC];
    float airP[NC];
};

typedef void (*LogFn)(const char *line);

class Core
{
public:
    void begin(uint32_t now, const PersistBlob *restored, bool shadow, LogFn log);
    void tick(const Inputs &in, Outputs &out);
    void notifyPresetLoad(const uint8_t heights[NC]); // implies notifyManual()
    void notifyManual();
    bool exportPersist(PersistBlob &b) const;
    int dump(char *buf, size_t n) const;
    State state() const { return st; }
    bool isDriving() const { return driving; }
    static const char *stateName(State s);
    static const char *clsName(Cls c);

private:
    struct Corner
    {
        // samples (hcs_sense.cpp)
        float wh[HCS_WINDOW], wp[HCS_WINDOW];
        int nh, ih, np, ip;
        float dh[3], dp[3];
        int ndh, ndp;
        float actH, actP;
        uint32_t lastSeq, idleSince, valveOpenSince;
        bool busy, valveWasOpen;
        bool hFault, pFault;
        int badH, badP;
        uint32_t goodSinceH, goodSinceP;
        float lastH;           // last idle height sample
        float liveH;           // during a correction: last height read before any sign of motion
        bool qOk;              // steadiness since the last movement: height range [qLo, qHi] since qStart
        float qLo, qHi;
        uint32_t qStart, qSince;
        // what to hold and the evidence references (hcs_classify.cpp)
        bool tgtValid;
        float tgt, floorOverride;
        float pRef;       // load reference: pressure when the target was last confirmed (frozen during an event)
        float pScale;     // load normaliser: pressure at the last user commit
        float airH, airP; // air reference: (h, p) when this bag's air was last known-good
        Cls lastCls;
        int8_t wantDir;   // confirmation: a correction wanted since wantSince, in motion episode wantEpisode
        uint32_t wantSince, wantEpisode;
        uint32_t lastCorrAt, lastCorrEpisode;
        int8_t lastCorrDir;
        Cls completeCls;  // != NONE: one completion attempt pending
        int8_t completeDir;
        // leaks
        bool haveRefill, leakFault, leakAllowance;
        uint32_t lastRefillAt;
        int fastCount;
        float leakRate;
        // correction in flight (hcs_core.cpp)
        bool inBatch, touched;
        int8_t batchDir;
        Cls batchCls;
        float batchH0, batchP0;
        uint32_t batchOpenMs;
        float batchGoal;
        float fillRate; // height % per second of open IN valve, learned from parked fills (cruise pulse sizing)
        float sdH, sdP; // drive-long sums for the drive-away load check
        float pRoad;    // mean bag pressure over the last drive: a load reference that does not depend on a parking spot
        float calmLo, calmHi;
        float oweCand;  // fill the last parked evaluation wanted (height %), becomes owe when DRIVING is confirmed
        float owe;      // drive-away top-up still to deliver (height %)
        // cruise (hcs_cruise.cpp)
        float hL, pL, hS, hC, pC;
        bool cruiseInit, cruiseRef, cruiseLow;
        int cruiseCnt;
        uint32_t cruiseLowSince, lastPulseAt;
    };

    Corner c[NC];
    State st;
    bool shadow;
    LogFn logFn;
    uint32_t now, bootAt, lastDt;
    float tank;
    bool tankValid;

    // motion
    bool inEpisode, driving, armedAfterDrive, bootPending;
    uint32_t episodeStart, lastMotion, episodeId;
    int lastIncluded, lastVoting, lastStrong, motionRun, abortRun;
    // evaluation
    bool needEval, eventOpen, externalFreeze;
    uint32_t lastEvalAt, eventOpenSince, confirmAt, lastPostponeLogAt;
    bool prevPresence, prevEnabled;
    bool presetValid;
    float presetH[NC];
    // user override
    bool manualActive, presetPending, pendingManual, manualSawDrive, manualIdle;
    uint8_t pendingPreset[NC];
    uint32_t manualIdleSince;
    // correction in flight
    uint32_t batchStart, settleStart;
    bool batchSettling;
    uint32_t batchT[HCS_MAX_BATCHES_PER_HOUR];
    int batchI;
    // cruise
    uint32_t driveStartAt, steadySince, cruiseEvalAt, pulseEnd, oweAt, driveLoadAt;
    uint32_t sdN;
    bool driveLoadDone, pulseOwe, roadValid;
    uint32_t calmSince, pulseStartAt, pulseMs;
    float pulseStep;
    int driveLoadCnt;
    bool steady;
    int pulseCorner;
    uint32_t pulseT[HCS_CRUISE_MAX_PER_HOUR];
    int pulseI;
    // STATS
    uint16_t stH[2][NC][6], stP[2][NC][6];
    double sqN[NC], sqH[NC], sqH2[NC], sqP[NC], sqP2[NC];
    uint32_t statsAt;
    // persistence + logging
    bool dirty, persistImmediate;
    uint32_t lastPersistAt;
    uint32_t thrHash[8], thrAt[8];

    // hcs_core.cpp
    void vlog(bool throttle, const char *fmt, va_list ap);
    void logf(const char *fmt, ...);
    void logq(const char *fmt, ...); // throttled (HCS_LOG_THROTTLE_MS) for repeated identical lines
    void tickInner(const Inputs &in, Outputs &out);
    void enterManual(const char *why, Outputs &out);
    void tickManual(const Inputs &in);
    void startBatch(const Inputs &in, Outputs &out, const bool *inB, const int8_t *dir, const float *goal, const Cls *cls,
                    const float *hm, const float *pm, const bool *pOk);
    void tickCorrecting(const Inputs &in, Outputs &out);
    void finalizeBatch();
    void abortBatch(Outputs &out, const char *why, State next);
    void recordRefill(int i, float deficit);
    void markPersist(bool immediate);
    // hcs_sense.cpp
    void ingest(const Inputs &in);
    void detectFaults(const Inputs &in, int i);
    void motionDetect(const Inputs &in, uint32_t dt);
    void statsSample(const Inputs &in, const bool *inc);
    void statsFlush();
    bool windowMean(int i, float &hm, float &pm, bool &pOk, bool &stable) const;
    bool pUsable(int i) const { return !c[i].pFault && c[i].ndp >= 3; }
    // hcs_classify.cpp
    void evaluate(const Inputs &in, Outputs &out, const char *kind, bool arrival);
    Cls classify(const Inputs &in, int i, float hm, bool pOk, float pm, int loadDir, bool airElsewhere, int8_t airChg,
                 int8_t &dir, float &goal);
    const char *gate(const Inputs &in, int i, int8_t dir, bool pOk, float pm) const;
    void retargetArrival(const Inputs &in, const float *hm);
    bool planeDeficit(const float *h, float *d) const; // per corner: target plane - current plane, twist removed
    uint32_t steadyRemaining() const; // 0 = every healthy corner steady for HCS_CONFIRM_MS; 0xFFFFFFFF = not steady
    // hcs_cruise.cpp
    void cruiseTick(const Inputs &in, Outputs &out);
    bool oweTick(const Inputs &in, Outputs &out);
    void driveLoadCheck(const Inputs &in);

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

// shared helpers for the hcs_*.cpp files
static const char *const CN[NC] = {"FP", "RP", "FD", "RD"};
static const uint32_t HOUR_MS = 3600000UL;
static inline float fmaxf_(float a, float b) { return a > b ? a : b; }
static inline float fminf_(float a, float b) { return a < b ? a : b; }
static inline bool finite_(float v) { return !(v != v) && v < 1e30f && v > -1e30f; }

} // namespace hcs

#endif
