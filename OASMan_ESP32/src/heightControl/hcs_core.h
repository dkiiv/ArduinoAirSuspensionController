// Height Control Supervisor (HCS) core -- platform independent (no Arduino / FreeRTOS includes).
//
// The core owns every AUTONOMOUS height decision: when the car is moving, when it has settled, what the
// per-corner targets are, how a deviation is classified (air loss / load / terrain shift / ...), and whether a
// correction is allowed (bounds, budgets, presence, faults). It never touches hardware: the ESP32 adapter
// (heightControlSupervisor.cpp) feeds it sensor snapshots and executes its START / ABORT commands through the
// existing Wheel goal routine. The same source is compiled on a PC by eval/hcs_sim.cpp to replay scenarios.
//
// Single-threaded by contract: every method is called from the one supervisor task (or the PC simulator).
//
// Design doc: OASMan_ESP32/docs/height-control-supervisor.md

#ifndef hcs_core_h
#define hcs_core_h

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "hcs_config.h"

namespace hcs
{

// LOGICAL corner index order FP, RP, FD, RD (WHEEL_FRONT_PASSENGER=0 ... WHEEL_REAR_DRIVER=3). This is index order
// only; the supervisor never touches pins or ADC channels. The physical valve-block order is a pin-map matter in
// user_defines.h (on the dkiiv `tesla` branch the v4 connector slots are FD, RD, RP, FP and the ADS A pressure
// channels follow them, while the ADS B height channels keep the upstream map). Never reorder these indices.
static const int NC = 4;
static const int C_FP = 0, C_RP = 1, C_FD = 2, C_RD = 3;
inline bool isFront(int c) { return c == C_FP || c == C_FD; }

enum class State : uint8_t
{
    INIT,       // boot hold, buffers filling
    INACTIVE,   // not height mode / maintain off / safety mode: observe only
    MOTION,     // motion episode in progress (driving flag says DRIVING vs DISTURBED)
    SETTLING,   // quiet, but not long enough to evaluate yet
    PARKED,     // evaluated and holding targets
    CORRECTING, // an autonomous batch is in flight (or settling after it)
    MANUAL,     // user command in flight / settling: supervisor yields
    FAULT       // >= 2 height sensors faulted: no autonomous actuation at all
};

enum class Cls : uint8_t
{
    NONE,
    WITHIN,       // inside deadband
    AIR_LOSS,     // low, bag pressure did not rise: air mass lost (leak / cooling) -> fill
    LOAD,         // low, pressure up, total vehicle load up -> fill
    SHIFT,        // load moved between corners at ~constant total (terrain, crown, slope, cornering, someone moved) -> accept
    COUPLED,      // displaced because ANOTHER bag's air changed (rigid body) -> hold: it comes back when that one is fixed
    AMBIGUOUS,    // contradicting evidence -> hold, do nothing
    AIR_GAIN,     // high, pressure did not drop (thermal expansion) -> hold, never dump
    UNLOAD,       // high, pressure down, total load down -> lower (if HCS_AUTO_LOWER)
    BOTTOM_GUARD, // below the calibrated minimum ride height -> lift (fixes the leak-below-min-ride bug)
    CEILING,      // above the autonomous ceiling -> lower (if HCS_AUTO_LOWER)
    FROZEN        // sensor fault on this corner -> never actuated autonomously
};

enum HFault : uint8_t
{
    HF_NONE = 0,
    HF_RANGE = 1,       // raw voltage outside 0.5-4.5 V band (wire break / short)
    HF_CAL = 2,         // calibration degenerate (min == max)
    HF_TRAVEL = 4,      // raw outside calibrated travel by a wide margin
    HF_STUCK = 8,       // silent while the other corners move (driving)
    HF_NO_RESPONSE = 16 // valve moved air, height did not follow
};

struct CornerIn
{
    float h;    // calibrated height % (0..100)
    float hRaw; // raw % of the 0.5-4.5 V span, NOT clamped
    float p;    // bag psi (only trusted after the corner's valves have been closed HCS_P_SETTLE_MS)
    bool inOpen, outOpen;
    bool routineActive;     // a goal routine is flagged on this wheel
    bool routineAutonomous; // ...and it was started by the supervisor
    uint32_t seq;           // increments whenever the wheel task took a fresh reading
    float calMinRaw, calMaxRaw, minRide;
};

struct Inputs
{
    uint32_t now;
    CornerIn c[NC];
    float tank;
    bool tankValid;
    bool compressorOn;
    bool presence;  // isVehicleOn(): in BT-ignition builds == an authenticated BLE client is connected
    bool enabled;   // height-sensor mode && maintain enabled
    bool safetyMode;
    float bagCeilPsi;       // min(bagMaxPressure, MAX_PRESSURE_SAFETY)
    float compressorOffPsi; // tank cut-off (a fill needing more than this can never get headroom)
};

enum class Cmd : uint8_t
{
    NONE,
    START, // start autonomous goals for the corners with g[i].active
    ABORT, // abort autonomous goals / close valves on the corners with g[i].active
    PULSE  // cruise top-up: open the IN valve of the g[i].active corner for g[i].pulseMs, then close
};

struct GoalRequest
{
    bool active;
    int8_t dir;     // +1 fill only, -1 dump only
    uint8_t target; // height %
    float ceilH;    // routine must stop filling at/above this height
    float floorH;   // routine must stop dumping at/below this height
    float ceilP;    // routine must stop filling at/above this (raw, flow-inflated) bag psi
    uint16_t pulseMs;
};

struct Outputs
{
    Cmd cmd;
    GoalRequest g[NC];
    bool persistNow;
};

// Persisted (NVS) so a reboot / OTA / brownout does not forget what the car is supposed to be holding.
static const uint32_t PERSIST_MAGIC = 0x48435331; // "HCS1"
static const uint8_t PERSIST_VERSION = 2; // v2: + air reference, learned L0
struct PersistBlob
{
    uint32_t magic;
    uint8_t version;
    uint8_t presetValid;
    uint8_t presetH[NC];
    uint8_t reserved[2];
    float tgt[NC];
    float pRef[NC];
    float floorOverride[NC]; // < 0 = none
    float pScale[NC];
    float airP[NC];
    float airH[NC];
    float L0[NC];
};

typedef void (*LogFn)(const char *line);

class Core
{
public:
    void begin(uint32_t now, const PersistBlob *restored, bool shadow, LogFn log);
    void tick(const Inputs &in, Outputs &out);

    // Cross-task events, forwarded by the adapter on the supervisor task.
    void notifyPresetLoad(const uint8_t heights[NC]);
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
        // settled window
        float wh[HCS_WINDOW];
        float wp[HCS_WINDOW];
        int nh, ih, np, ip;
        // second-difference history
        float dh[3];
        float dp[3];
        int ndh, ndp;
        float actH, actP;
        uint32_t lastSeq;
        uint32_t idleSince; // valves closed + no routine since
        bool busy;
        // faults
        uint8_t hFault;
        bool pFault;
        int badH, badP;
        uint32_t goodSinceH, goodSinceP;
        uint32_t silentMs;
        // control
        bool tgtValid;
        float tgt, pRef, floorOverride;
        float pScale; // load-proportional normaliser for the total-load test; set on commits, NEVER on SHIFT accepts
        float airP, airH;  // air-content reference: the (p, h) pair when this bag's air was last known-good
        uint32_t airAt;    // when the air reference was taken
        float L0;          // learned bag volume offset (height-% units)
        int l0n;           // number of L0 learning samples
        float fillRate;    // learned parked fill rate, height % per second of valve-open time (NAN = unknown)
        uint32_t batchOpenMs;
        float hL, pL, hS;  // cruise: 2-min height / pressure averages, 1-s height average
        float hC, pC;      // cruise reference: the 2-min averages once the trip has settled (minute 10)
        bool cruiseInit, cruiseRef;
        uint32_t cruiseLowSince;
        bool cruiseLow;
        int cruiseCnt; // leaky evidence counter: +1 per low 10-s evaluation, -1 otherwise
        uint32_t lastPulseAt;
        uint32_t lastCorrAt;
        int8_t lastCorrDir;
        uint32_t lastCorrEpisode;
        uint32_t lastRefillAt;
        bool haveRefill;
        int fastCount;
        float leakRate; // smoothed estimate, height % per hour (logged; diagnostics only)
        bool leakFault, leakAllowance;
        bool touched;
        // batch
        bool inBatch;
        int8_t batchDir;
        Cls batchCls;
        float batchH0, batchP0;
        uint32_t valveOpenSince;
        bool valveWasOpen;
        Cls lastCls;
        Cls completeCls; // != NONE: one completion attempt pending with this class
        int8_t completeDir;
    };

    Corner c[NC];
    State st;
    bool shadow;
    LogFn logFn;
    uint32_t now, bootAt, lastDt;

    // motion
    bool inEpisode, driving, armedAfterDrive, bootPending;
    uint32_t episodeStart, lastMotion, episodeId;
    int lastIncluded, lastVoting, lastStrong, motionRun, abortRun;
    bool eventOpen;
    uint32_t eventOpenSince;
    // cruise top-up
    uint32_t driveStartAt, steadySince, cruiseEvalAt, pulseEnd;
    bool steady;
    int pulseCorner;
    uint32_t pulseT[HCS_CRUISE_MAX_PER_HOUR];
    int pulseI;
    float cruiseRoll, cruisePitch;
    void cruiseTick(const Inputs &in, Outputs &out);
    // field statistics (see HCS_STATS_PERIOD_MS)
    uint16_t stH[2][NC][6], stP[2][NC][6];
    double sqN[NC], sqH[NC], sqH2[NC], sqP[NC], sqP2[NC];
    uint32_t statsAt;
    void statsSample(const Inputs &in, const bool *inc);
    void statsFlush();

    // evaluation
    bool needEval;
    uint32_t lastEvalAt;
    bool evaluatedSinceEpisode;
    bool externalFreeze;
    bool mappingFault; // latched until reboot: actuating corner i moved a different corner's height
    float batchAllH0[NC];
    uint32_t lastPostponeLogAt;

    // presence / enable
    bool prevPresence, prevEnabled;
    bool tankValid;
    float tank;

    // preset anchor
    bool presetValid;
    float presetH[NC];

    // manual
    bool manualActive, presetPending, pendingManual, manualSawDrive;
    uint8_t pendingPreset[NC];
    uint32_t manualIdleSince;
    bool manualIdle;

    // batch
    uint32_t batchStart, settleStart, batchFillMs;
    bool batchSettling;
    uint32_t batchT[HCS_MAX_BATCHES_PER_HOUR];
    int batchI;
    uint32_t fillT[16], fillMs[16];
    int fillI;

    // persistence
    bool dirty, persistImmediate;
    uint32_t lastPersistAt;

    uint32_t thrHash[8], thrAt[8];
    void vlog(bool throttle, const char *fmt, va_list ap);
    void logf(const char *fmt, ...);
    void logq(const char *fmt, ...); // throttled: identical text suppressed for HCS_LOG_THROTTLE_MS
    void ingest(const Inputs &in);
    void detectFaults(const Inputs &in, int i);
    void tickInner(const Inputs &in, Outputs &out);
    void motionDetect(const Inputs &in, uint32_t dt);
    bool windowMean(int i, float &hm, float &pm, bool &pOk, bool &stable) const;
    float floorBase(const Inputs &in, int i) const;
    float floorTrigger(const Inputs &in, int i) const;
    float liftTarget(const Inputs &in, int i) const;
    float ceilH() const { return 100.0f - HCS_CEIL_MARGIN; }
    bool pUsable(int i) const;
    bool allIdleFor(uint32_t ms) const;
    bool anyRoutine(const Inputs &in) const;
    int heightFaultCount() const;
    int batchesLastHour() const;
    uint32_t fillMsLastHour() const;
    void evaluate(const Inputs &in, Outputs &out, const char *kind, bool arrival);
    void tickCorrecting(const Inputs &in, Outputs &out);
    void finalizeBatch(const Inputs &in);
    void abortBatch(Outputs &out, const char *why, State next);
    void recordRefill(int i, float deficit);
    void setAirRef(int i, float p, float h);
    float airDelta(int i, float p, float h) const; // fractional air-content change vs reference (NAN if unknown)
    void tickManual(const Inputs &in);
    void enterManual(const Inputs &in, const char *why, Outputs &out);
    void markPersist(bool immediate);
};

} // namespace hcs

#endif
