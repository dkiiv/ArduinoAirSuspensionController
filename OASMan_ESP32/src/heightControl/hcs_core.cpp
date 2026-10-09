// Height Control Supervisor core. See hcs_core.h for the contract and
// OASMan_ESP32/docs/height-control-supervisor.md for the design.

#include "hcs_core.h"

// Legacy builds (HEIGHT_CONTROL_SUPERVISOR false) compile none of this; the PC simulator defines HCS_HOST_BUILD.
#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace hcs
{

static const char *const CN[NC] = {"FP", "RP", "FD", "RD"};
static const uint32_t HOUR_MS = 3600000UL;

static inline float fmaxf_(float a, float b) { return a > b ? a : b; }
static inline float fminf_(float a, float b) { return a < b ? a : b; }
static inline bool finite_(float v) { return !(v != v) && v < 1e30f && v > -1e30f; }

const char *Core::stateName(State s)
{
    switch (s)
    {
    case State::INIT: return "INIT";
    case State::INACTIVE: return "INACTIVE";
    case State::MOTION: return "MOTION";
    case State::SETTLING: return "SETTLING";
    case State::PARKED: return "PARKED";
    case State::CORRECTING: return "CORRECTING";
    case State::MANUAL: return "MANUAL";
    case State::FAULT: return "FAULT";
    }
    return "?";
}

const char *Core::clsName(Cls c)
{
    switch (c)
    {
    case Cls::NONE: return "-";
    case Cls::WITHIN: return "WITHIN";
    case Cls::AIR_LOSS: return "AIR_LOSS";
    case Cls::LOAD: return "LOAD";
    case Cls::SHIFT: return "SHIFT";
    case Cls::AMBIGUOUS: return "AMBIGUOUS";
    case Cls::AIR_GAIN: return "AIR_GAIN";
    case Cls::UNLOAD: return "UNLOAD";
    case Cls::BOTTOM_GUARD: return "BOTTOM_GUARD";
    case Cls::CEILING: return "CEILING";
    case Cls::FROZEN: return "FROZEN";
    }
    return "?";
}

void Core::vlog(bool throttle, const char *fmt, va_list ap)
{
    if (!logFn)
        return;
    char buf[224];
    int n = snprintf(buf, sizeof(buf), "HCS t=%lu ", (unsigned long)now);
    if (n < 0)
        n = 0;
    vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, ap);
    if (throttle)
    {
        // Identical message (ignoring the timestamp) seen within the throttle window -> drop it. Keeps the
        // overnight log readable (e.g. "VETO no BLE client" for a pending refill every 30 s).
        uint32_t hsh = 2166136261u;
        for (const char *s = buf + n; *s; s++)
            hsh = (hsh ^ (uint8_t)*s) * 16777619u;
        int slot = -1, oldest = 0;
        for (int i = 0; i < 8; i++)
        {
            if (thrHash[i] == hsh)
            {
                slot = i;
                break;
            }
            if ((now - thrAt[i]) > (now - thrAt[oldest]))
                oldest = i;
        }
        if (slot >= 0 && (now - thrAt[slot]) < HCS_LOG_THROTTLE_MS)
            return;
        if (slot < 0)
            slot = oldest;
        thrHash[slot] = hsh;
        thrAt[slot] = now;
    }
    logFn(buf);
}

void Core::logf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vlog(false, fmt, ap);
    va_end(ap);
}

void Core::logq(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vlog(true, fmt, ap);
    va_end(ap);
}

// ---------------------------------------------------------------------------------------------------------
// Setup / persistence
// ---------------------------------------------------------------------------------------------------------

void Core::begin(uint32_t t, const PersistBlob *r, bool sh, LogFn lf)
{
    memset(c, 0, sizeof(c));
    now = bootAt = t;
    shadow = sh;
    logFn = lf;
    st = State::INIT;

    inEpisode = driving = armedAfterDrive = false;
    bootPending = true; // first evaluation after boot uses the long (arrival) quiet requirement
    episodeStart = 0;
    lastMotion = t;
    episodeId = 0;
    lastIncluded = lastVoting = lastStrong = 0;
    motionRun = abortRun = 0;
    eventOpen = false;
    eventOpenSince = t;

    needEval = true;
    lastEvalAt = t;
    externalFreeze = false;
    mappingFault = false;
    for (int i = 0; i < NC; i++)
        batchAllH0[i] = NAN;
    lastPostponeLogAt = t - 3600000UL;

    prevPresence = prevEnabled = false;
    tankValid = false;
    tank = 0;

    presetValid = false;
    manualActive = presetPending = manualIdle = pendingManual = manualSawDrive = false;
    manualIdleSince = t;

    batchStart = settleStart = 0;
    batchSettling = false;
    const uint32_t old = t - 2 * HOUR_MS; // "long ago" timestamps (unsigned maths: now - old >= 2h)
    for (int i = 0; i < HCS_MAX_BATCHES_PER_HOUR; i++)
        batchT[i] = old;
    for (int i = 0; i < 16; i++)
    {
        fillT[i] = old;
        fillMs[i] = 0;
    }
    fillI = batchI = 0;

    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        k.idleSince = t;
        k.tgt = 0;
        k.pRef = NAN;
        k.pScale = NAN;
        k.floorOverride = -1;
        k.lastCorrAt = t - 2 * HOUR_MS;
        k.lastCorrDir = 0;
        k.lastCorrEpisode = 0xFFFFFFFF;
        for (int j = 0; j < HCS_LEAK_FAULT_COUNT; j++)
            k.leakT[j] = t - 2 * (HCS_LEAK_WINDOW_MS);
        k.goodSinceH = k.goodSinceP = t;
        k.lastSeq = 0xFFFFFFFF;
    }

    dirty = persistImmediate = false;
    lastPersistAt = t;
    for (int i = 0; i < 8; i++)
    {
        thrHash[i] = 0;
        thrAt[i] = t - 2 * HOUR_MS;
    }

    if (r && r->magic == PERSIST_MAGIC && r->version == PERSIST_VERSION)
    {
        bool ok = true;
        int nValid = 0;
        for (int i = 0; i < NC; i++)
        {
            if (finite_(r->tgt[i]) && (r->tgt[i] < 0 || r->tgt[i] > 100))
                ok = false;
            if (finite_(r->tgt[i]))
                nValid++;
            if (finite_(r->pRef[i]) && (r->pRef[i] < 0 || r->pRef[i] > 250))
                ok = false;
        }
        if (ok && nValid > 0)
        {
            for (int i = 0; i < NC; i++)
            {
                c[i].tgtValid = finite_(r->tgt[i]); // a corner saved without a target gets a fresh baseline
                c[i].tgt = c[i].tgtValid ? r->tgt[i] : 0;
                c[i].pRef = r->pRef[i];
                c[i].floorOverride = finite_(r->floorOverride[i]) ? r->floorOverride[i] : -1;
                c[i].pScale = (finite_(r->pScale[i]) && r->pScale[i] > 0 && r->pScale[i] < 250) ? r->pScale[i] : NAN;
                presetH[i] = r->presetH[i];
            }
            presetValid = r->presetValid != 0;
            logf("RESTORED tgt=%.1f/%.1f/%.1f/%.1f pRef=%.1f/%.1f/%.1f/%.1f preset=%d", c[0].tgt, c[1].tgt, c[2].tgt,
                 c[3].tgt, c[0].pRef, c[1].pRef, c[2].pRef, c[3].pRef, presetValid ? 1 : 0);
        }
        else
        {
            logf("persisted state rejected (out of range) -> starting unanchored");
        }
    }
    else
    {
        logf("no persisted state -> starting unanchored (baseline = first settled reading)");
    }
}

bool Core::exportPersist(PersistBlob &b) const
{
    memset(&b, 0, sizeof(b));
    b.magic = PERSIST_MAGIC;
    b.version = PERSIST_VERSION;
    b.presetValid = presetValid ? 1 : 0;
    for (int i = 0; i < NC; i++)
    {
        b.presetH[i] = (uint8_t)(presetValid ? presetH[i] : 0);
        b.tgt[i] = c[i].tgtValid ? c[i].tgt : NAN; // NAN = this corner has no target (restored as such)
        b.pRef[i] = c[i].pRef;
        b.floorOverride[i] = c[i].floorOverride;
        b.pScale[i] = c[i].pScale;
    }
    for (int i = 0; i < NC; i++)
        if (c[i].tgtValid)
            return true;
    return false; // nothing worth saving yet
}

void Core::markPersist(bool immediate)
{
    dirty = true;
    if (immediate)
        persistImmediate = true;
}

// ---------------------------------------------------------------------------------------------------------
// Events from other tasks (forwarded by the adapter)
// ---------------------------------------------------------------------------------------------------------

void Core::notifyPresetLoad(const uint8_t heights[NC])
{
    for (int i = 0; i < NC; i++)
    {
        pendingPreset[i] = heights[i];
        c[i].touched = true;
    }
    presetPending = true;
    pendingManual = true;
}

void Core::notifyManual() { pendingManual = true; }

// ---------------------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------------------

float Core::floorBase(const Inputs &in, int i) const
{
    float mr = in.c[i].minRide;
    return (mr > 0.5f) ? mr : (float)HCS_ABS_FLOOR;
}
float Core::floorTrigger(const Inputs &in, int i) const
{
    if (c[i].floorOverride >= 0)
        return c[i].floorOverride - HCS_DEADBAND_H; // user deliberately holds this corner below min ride
    return floorBase(in, i);
}
float Core::liftTarget(const Inputs &in, int i) const
{
    if (c[i].floorOverride >= 0)
        return c[i].floorOverride;
    return fminf_(floorBase(in, i) + HCS_FLOOR_LIFT_MARGIN, ceilH());
}

bool Core::pUsable(int i) const { return !c[i].pFault && c[i].ndp >= 3; }

bool Core::allIdleFor(uint32_t ms) const
{
    for (int i = 0; i < NC; i++)
        if (c[i].busy || (now - c[i].idleSince) < ms)
            return false;
    return true;
}

bool Core::anyRoutine(const Inputs &in) const
{
    for (int i = 0; i < NC; i++)
        if (in.c[i].routineActive)
            return true;
    return false;
}

int Core::heightFaultCount() const
{
    int n = 0;
    for (int i = 0; i < NC; i++)
        if (c[i].hFault)
            n++;
    return n;
}

int Core::batchesLastHour() const
{
    int n = 0;
    for (int i = 0; i < HCS_MAX_BATCHES_PER_HOUR; i++)
        if (now - batchT[i] < HOUR_MS)
            n++;
    return n;
}

uint32_t Core::fillMsLastHour() const
{
    uint32_t s = 0;
    for (int i = 0; i < 16; i++)
        if (now - fillT[i] < HOUR_MS)
            s += fillMs[i];
    return s;
}

bool Core::windowMean(int i, float &hm, float &pm, bool &pOk, bool &stable) const
{
    const Corner &k = c[i];
    pOk = false;
    stable = false;
    hm = pm = NAN;
    if (k.nh < HCS_WINDOW)
        return false;
    float s = 0, lo = 1e9f, hi = -1e9f;
    for (int j = 0; j < HCS_WINDOW; j++)
    {
        s += k.wh[j];
        lo = fminf_(lo, k.wh[j]);
        hi = fmaxf_(hi, k.wh[j]);
    }
    hm = s / HCS_WINDOW;
    bool hStable = (hi - lo) <= HCS_STABLE_RANGE_H;
    bool pStable = true;
    if (!k.pFault && k.np >= HCS_WINDOW)
    {
        float sp = 0, plo = 1e9f, phi = -1e9f;
        for (int j = 0; j < HCS_WINDOW; j++)
        {
            sp += k.wp[j];
            plo = fminf_(plo, k.wp[j]);
            phi = fmaxf_(phi, k.wp[j]);
        }
        pm = sp / HCS_WINDOW;
        pOk = true;
        pStable = (phi - plo) <= HCS_STABLE_RANGE_P;
    }
    stable = hStable && pStable;
    return true;
}

// ---------------------------------------------------------------------------------------------------------
// Sensing: sample ingest, faults, motion
// ---------------------------------------------------------------------------------------------------------

void Core::detectFaults(const Inputs &in, int i)
{
    const CornerIn &x = in.c[i];
    Corner &k = c[i];

    if (in.enabled)
    {
        float span = fabsf(x.calMaxRaw - x.calMinRaw);
        bool calBad = !(span >= HCS_CAL_MIN_SPAN);
        if (calBad && !(k.hFault & HF_CAL))
        {
            k.hFault |= HF_CAL;
            logf("FAULT %s height calibration degenerate (min=%.1f max=%.1f) -> corner frozen", CN[i], x.calMinRaw, x.calMaxRaw);
        }
        else if (!calBad && (k.hFault & HF_CAL))
        {
            k.hFault &= ~HF_CAL;
            logf("FAULT %s calibration now valid -> cleared", CN[i]);
        }

        float lo = fminf_(x.calMinRaw, x.calMaxRaw) - HCS_CAL_TRAVEL_MARGIN;
        float hi = fmaxf_(x.calMinRaw, x.calMaxRaw) + HCS_CAL_TRAVEL_MARGIN;
        bool range = !(x.hRaw >= HCS_RAW_H_MIN && x.hRaw <= HCS_RAW_H_MAX);
        bool travel = !calBad && !(x.hRaw >= lo && x.hRaw <= hi);
        if (range || travel)
        {
            k.badH++;
            k.goodSinceH = now;
            if (k.badH >= HCS_FAULT_LATCH_SAMPLES)
            {
                uint8_t f = range ? HF_RANGE : HF_TRAVEL;
                if (!(k.hFault & f))
                {
                    k.hFault |= f;
                    logf("FAULT %s height %s raw=%.1f%% (%.2fV) -> corner frozen", CN[i], range ? "out of 0.5-4.5V band" : "outside calibrated travel",
                         x.hRaw, 0.5f + x.hRaw * 0.04f);
                }
            }
        }
        else
        {
            k.badH = 0;
            if ((k.hFault & (HF_RANGE | HF_TRAVEL)) && (now - k.goodSinceH) >= HCS_FAULT_CLEAR_MS)
            {
                k.hFault &= ~(HF_RANGE | HF_TRAVEL);
                logf("FAULT %s height back in range for %lus -> cleared", CN[i], (unsigned long)(HCS_FAULT_CLEAR_MS / 1000));
            }
        }
    }

    bool pBad = !(x.p >= HCS_P_MIN_VALID && x.p <= HCS_P_MAX_VALID);
    if (pBad)
    {
        k.badP++;
        k.goodSinceP = now;
        if (k.badP >= HCS_FAULT_LATCH_SAMPLES && !k.pFault)
        {
            k.pFault = true;
            k.np = k.ip = k.ndp = 0;
            logf("FAULT %s bag pressure out of range (%.1f psi) -> no fills on this corner", CN[i], x.p);
        }
    }
    else
    {
        k.badP = 0;
        if (k.pFault && (now - k.goodSinceP) >= HCS_FAULT_CLEAR_MS)
        {
            k.pFault = false;
            logf("FAULT %s bag pressure back in range -> cleared", CN[i]);
        }
    }
}

void Core::ingest(const Inputs &in)
{
    tank = in.tank;
    tankValid = in.tankValid && in.tank >= HCS_P_MIN_VALID && in.tank <= HCS_P_MAX_VALID;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        const CornerIn &x = in.c[i];
        bool vo = x.inOpen || x.outOpen;
        bool busy = vo || x.routineActive;
        if (busy)
        {
            // Our own (or the user's) actuation: the corner's readings are flow-affected and its height ramps.
            // Restart its windows; it is excluded from motion voting until it has been idle HCS_P_SETTLE_MS.
            k.busy = true;
            k.idleSince = now;
            k.nh = k.ih = k.np = k.ip = 0;
            k.ndh = k.ndp = 0;
        }
        else if (k.busy)
        {
            k.busy = false;
            k.idleSince = now;
        }
        if (vo && !k.valveWasOpen)
            k.valveOpenSince = now;
        k.valveWasOpen = vo;

        if (x.seq == k.lastSeq)
            continue; // no fresh reading from the wheel task since last tick
        k.lastSeq = x.seq;

        detectFaults(in, i);
        if (busy)
            continue;

        k.wh[k.ih] = x.h;
        k.ih = (k.ih + 1) % HCS_WINDOW;
        if (k.nh < HCS_WINDOW)
            k.nh++;
        k.dh[0] = k.dh[1];
        k.dh[1] = k.dh[2];
        k.dh[2] = x.h;
        if (k.ndh < 3)
            k.ndh++;
        if (k.ndh >= 3)
        {
            float d2 = fabsf(k.dh[2] - 2.0f * k.dh[1] + k.dh[0]);
            k.actH += HCS_ACT_ALPHA * (d2 - k.actH);
        }

        if (!k.pFault && (now - k.idleSince) >= HCS_P_SETTLE_MS)
        {
            k.wp[k.ip] = x.p;
            k.ip = (k.ip + 1) % HCS_WINDOW;
            if (k.np < HCS_WINDOW)
                k.np++;
            k.dp[0] = k.dp[1];
            k.dp[1] = k.dp[2];
            k.dp[2] = x.p;
            if (k.ndp < 3)
                k.ndp++;
            if (k.ndp >= 3)
            {
                float d2 = fabsf(k.dp[2] - 2.0f * k.dp[1] + k.dp[0]);
                k.actP += HCS_ACT_ALPHA * (d2 - k.actP);
            }
        }
    }
}

void Core::motionDetect(const Inputs &in, uint32_t dt)
{
    (void)in;
    int included = 0, voting = 0, strong = 0;
    bool vote[NC] = {false, false, false, false};
    bool inc[NC] = {false, false, false, false};
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        if (k.hFault || k.busy || (now - k.idleSince) < HCS_P_SETTLE_MS || k.ndh < 3)
            continue;
        inc[i] = true;
        included++;
        bool pv = pUsable(i);
        bool v = k.actH > HCS_MOTION_H_THRESH || (pv && k.actP > HCS_MOTION_P_THRESH);
        bool s = k.actH > 1.5f * HCS_MOTION_H_THRESH || (pv && k.actP > 1.5f * HCS_MOTION_P_THRESH);
        vote[i] = v;
        if (v)
            voting++;
        if (s)
            strong++;
    }
    lastIncluded = included;
    lastVoting = voting;
    lastStrong = strong;

    if (included < HCS_MOTION_MIN_CORNERS)
    {
        // Cannot observe (corners busy / faulted). Not evidence of driving, but definitely not "quiet".
        lastMotion = now;
        return;
    }

    bool motion = voting >= HCS_MOTION_MIN_CORNERS;
    // A new episode needs a few consecutive motion ticks (rejects single noise spikes); once running, any
    // motion tick keeps it alive.
    motionRun = motion ? motionRun + 1 : 0;
    if (motion && !inEpisode && motionRun < HCS_MOTION_START_TICKS)
        motion = false;
    if (motion)
    {
        if (!inEpisode)
        {
            inEpisode = true;
            episodeStart = now;
            episodeId++;
            logf("MOTION start ep=%lu (corners voting %d/%d)", (unsigned long)episodeId, voting, included);
        }
        lastMotion = now;
        if (!driving && (now - episodeStart) >= HCS_DRIVE_CONFIRM_MS)
        {
            driving = true;
            armedAfterDrive = true;
            logf("DRIVING confirmed (motion for %lus) -> all autonomous actuation vetoed; re-baseline on arrival",
                 (unsigned long)((now - episodeStart) / 1000));
        }
    }
    else if (inEpisode && (now - lastMotion) >= HCS_EPISODE_GAP_MS)
    {
        inEpisode = false;
        logf("MOTION end ep=%lu after %lus (%s)", (unsigned long)episodeId, (unsigned long)((lastMotion - episodeStart) / 1000),
             driving ? "drive" : "disturbance");
        driving = false;
        needEval = true;
    }

    // Stuck height sensor: silent for a long time while at least two other corners show road motion.
    if (driving)
    {
        for (int i = 0; i < NC; i++)
        {
            if (!inc[i])
                continue;
            Corner &k = c[i];
            int others = voting - (vote[i] ? 1 : 0);
            if (others >= 2 && k.actH < HCS_STUCK_ACT)
            {
                k.silentMs += dt;
                if (k.silentMs >= HCS_STUCK_MS && !(k.hFault & HF_STUCK))
                {
                    k.hFault |= HF_STUCK;
                    logf("FAULT %s height sensor silent for %lus while driving (stuck?) -> corner frozen", CN[i],
                         (unsigned long)(k.silentMs / 1000));
                }
            }
            else
                k.silentMs = 0;
        }
    }
    for (int i = 0; i < NC; i++)
    {
        // a stuck-flagged sensor that shows real movement again is alive
        if ((c[i].hFault & HF_STUCK) && c[i].actH > HCS_MOTION_H_THRESH)
        {
            c[i].hFault &= ~HF_STUCK;
            c[i].silentMs = 0;
            logf("FAULT %s height sensor moving again -> stuck cleared", CN[i]);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// Main tick
// ---------------------------------------------------------------------------------------------------------

void Core::tick(const Inputs &in, Outputs &out)
{
    uint32_t dt = in.now - now;
    now = in.now;
    memset(&out, 0, sizeof(out));

    ingest(in);

    if ((now - bootAt) < HCS_BOOT_HOLD_MS)
    {
        st = State::INIT;
        lastMotion = now;
        prevPresence = in.presence;
        prevEnabled = in.enabled;
        return;
    }

    motionDetect(in, dt);
    tickInner(in, out);

    if (dirty && (persistImmediate || (now - lastPersistAt) >= HCS_PERSIST_MIN_INTERVAL_MS))
    {
        PersistBlob b;
        if (exportPersist(b))
            out.persistNow = true;
        dirty = persistImmediate = false;
        lastPersistAt = now;
    }
}

void Core::tickInner(const Inputs &in, Outputs &out)
{
    // ---- presence / enable edges
    if (in.presence && !prevPresence)
    {
        logf("PRESENCE on (BLE client connected) -> evaluate");
        needEval = true;
        for (int i = 0; i < NC; i++)
            if (c[i].leakFault && !c[i].leakAllowance)
            {
                c[i].leakAllowance = true;
                logf("LEAK %s latched: owner returned -> one refill granted", CN[i]);
            }
    }
    else if (!in.presence && prevPresence)
        logf("PRESENCE off -> autonomous actuation vetoed");
    prevPresence = in.presence;
    if (in.enabled != prevEnabled)
    {
        logf("%s (height-sensor mode + maintain)", in.enabled ? "ENABLED" : "DISABLED");
        needEval = true;
        prevEnabled = in.enabled;
    }

    // ---- user / legacy actuation always wins
    bool userActivity = pendingManual;
    const char *why = pendingManual ? (presetPending ? "preset load" : "manual command") : "valve activity";
    pendingManual = false;
    for (int i = 0; i < NC; i++)
    {
        const CornerIn &x = in.c[i];
        bool mine = (st == State::CORRECTING) && c[i].inBatch;
        if (x.routineActive && !x.routineAutonomous)
        {
            userActivity = true;
            c[i].touched = true;
        }
        else if ((x.inOpen || x.outOpen) && !x.routineActive && !mine)
        {
            userActivity = true;
            c[i].touched = true;
        }
    }
    if (userActivity)
        enterManual(in, why, out);

    if (st == State::CORRECTING)
    {
        tickCorrecting(in, out);
        return;
    }
    if (manualActive)
    {
        if (driving)
            manualSawDrive = true;
        tickManual(in);
        if (manualActive)
            return;
    }

    // ---- global gates (observe-only states)
    if (mappingFault)
    {
        st = State::FAULT;
        return;
    }
    if (heightFaultCount() >= 2)
    {
        if (st != State::FAULT)
            logf("FAULT %d height sensors faulted -> ALL autonomous actuation frozen", heightFaultCount());
        st = State::FAULT;
        return;
    }
    if (!in.enabled || in.safetyMode)
    {
        st = State::INACTIVE;
        return;
    }
    if (inEpisode)
    {
        st = State::MOTION;
        return;
    }

    const bool arrival = armedAfterDrive || bootPending;
    const uint32_t needQuiet = arrival ? HCS_ARRIVAL_QUIET_MS : HCS_DISTURB_QUIET_MS;
    if ((now - lastMotion) < needQuiet)
    {
        st = State::SETTLING;
        return;
    }
    bool periodic = (now - lastEvalAt) >= HCS_EVAL_PERIOD_MS;
    if (!needEval && !periodic)
    {
        st = State::PARKED;
        return;
    }
    if (anyRoutine(in) || !allIdleFor(HCS_P_SETTLE_MS))
    {
        st = State::SETTLING;
        return;
    }
    evaluate(in, out, bootPending ? "BOOT" : (armedAfterDrive ? "ARRIVAL" : (needEval ? "EVENT" : "PERIODIC")), arrival);
}

// ---------------------------------------------------------------------------------------------------------
// Classifier + arbiter
// ---------------------------------------------------------------------------------------------------------

void Core::evaluate(const Inputs &in, Outputs &out, const char *kind, bool arrival)
{
    float hm[NC], pm[NC];
    bool pOk[NC], stable[NC], hOk[NC];
    for (int i = 0; i < NC; i++)
    {
        hOk[i] = c[i].hFault == 0;
        if (!windowMean(i, hm[i], pm[i], pOk[i], stable[i]))
        {
            if (hOk[i])
            {
                st = State::SETTLING;
                return; // window not full yet
            }
        }
        else if (hOk[i] && !c[i].pFault && !pOk[i])
        {
            st = State::SETTLING;
            return; // bag-pressure window still refilling after valve activity
        }
        else if (hOk[i] && !stable[i])
        {
            if ((now - lastPostponeLogAt) > 30000)
            {
                logq("EVAL %s postponed: %s readings not steady", kind, CN[i]);
                lastPostponeLogAt = now;
            }
            st = State::SETTLING;
            return;
        }
    }

    lastEvalAt = now;
    needEval = false;
    armedAfterDrive = false;
    bootPending = false;
    const float ch = ceilH();

    // ---- first baseline (nothing persisted, nothing commanded yet): hold what the car is doing now
    for (int i = 0; i < NC; i++)
    {
        if (!hOk[i] || c[i].tgtValid)
            continue;
        c[i].tgt = fminf_(hm[i], ch);
        c[i].pRef = pOk[i] ? pm[i] : NAN;
        c[i].pScale = c[i].pRef;
        c[i].tgtValid = true;
        logf("BASELINE %s init tgt=%.1f pRef=%.1f", CN[i], c[i].tgt, c[i].pRef);
        markPersist(false);
    }

    // ---- total-load signature: mean of dp / pScale, where pScale ~ the corner's static load / bag area at the
    // last commit. Dividing by it turns each dp into a load fraction, so different front / rear bag areas cancel.
    // Redistribution (crown, slope, cornering, braking -- and the reversal of an accepted shift) sums to ~0;
    // people / cargo do not. pScale must NOT follow SHIFT accepts: normalising a shift reversal by the shifted
    // pressures (118 vs 82 psi) would turn a symmetric +-18 psi return into a fake +3.4 % "load" (found in sim).
    int nl = 0;
    float sumFrac = 0;
    int extCorner = -1;
    float extFrac = 0;
    for (int i = 0; i < NC; i++)
    {
        if (!pOk[i] || !finite_(c[i].pRef))
            continue;
        float scale = finite_(c[i].pScale) ? c[i].pScale : c[i].pRef;
        float f = (pm[i] - c[i].pRef) / fmaxf_(scale, HCS_PREF_MIN_PSI);
        sumFrac += f;
        nl++;
        if (f < -HCS_EXTERNAL_UNLOAD_FRAC && f < extFrac)
        {
            extCorner = i;
            extFrac = f;
        }
    }
    enum
    {
        G_UNKNOWN,
        G_NEUTRAL,
        G_UP,
        G_DOWN
    } g = G_UNKNOWN;
    float loadFrac = nl ? sumFrac / nl : 0;
    if (nl >= 3)
        g = loadFrac > HCS_LOAD_FRAC ? G_UP : (loadFrac < -HCS_LOAD_FRAC ? G_DOWN : G_NEUTRAL);
    static const char *const GN[] = {"UNKNOWN", "NEUTRAL", "UP", "DOWN"};

    if (extCorner >= 0)
    {
        if (!externalFreeze)
            logf("EXTERNAL %s bag lost %.0f%% of its pressure (jack / lift / wheel unsupported?) -> autonomous actuation frozen",
                 CN[extCorner], -extFrac * 100.0f);
        externalFreeze = true;
        st = State::PARKED;
        return; // accept nothing, change nothing while something else is holding the car
    }
    if (externalFreeze)
    {
        logf("EXTERNAL cleared");
        externalFreeze = false;
    }

    const bool periodicEval = strcmp(kind, "PERIODIC") == 0;
    if (periodicEval)
        logq("EVAL %s load=%+.1f%% (%s, n=%d) pres=%d", kind, loadFrac * 100.0f, GN[g], nl, in.presence ? 1 : 0);
    else
        logf("EVAL %s load=%+.1f%% (%s, n=%d) tank=%.0f pres=%d", kind, loadFrac * 100.0f, GN[g], nl, tank, in.presence ? 1 : 0);

    // ---- per-corner classification
    Cls cls[NC];
    int8_t dir[NC];
    float goal[NC];
    bool rerefWithin[NC] = {false, false, false, false};
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        cls[i] = Cls::NONE;
        dir[i] = 0;
        goal[i] = k.tgt;
        if (!hOk[i])
        {
            cls[i] = Cls::FROZEN;
            k.lastCls = cls[i];
            continue;
        }
        const float e = hm[i] - k.tgt;
        const bool pv = pOk[i] && finite_(k.pRef);
        const float dp = pv ? pm[i] - k.pRef : 0;
        const float ft = floorTrigger(in, i);
        const float lt = liftTarget(in, i);
        const float oldTgt = k.tgt;

        const float shortBy = (k.completeDir > 0) ? -e : e;
        if (k.completeCls != Cls::NONE && shortBy > HCS_LAND_TOL && hm[i] >= ft && hm[i] <= ch + HCS_DEADBAND_H)
        {
            cls[i] = k.completeCls; // completion of a previous batch: same evidence, same direction
            dir[i] = k.completeDir;
            goal[i] = k.tgt;
        }
        else if (k.completeCls != Cls::NONE && shortBy <= HCS_LAND_TOL)
        {
            k.completeCls = Cls::NONE; // already there
            cls[i] = Cls::WITHIN;
        }
        else if (hm[i] < ft)
        {
            cls[i] = Cls::BOTTOM_GUARD;
            dir[i] = +1;
            goal[i] = fminf_(fmaxf_(k.tgt, lt), ch);
        }
        else if (hm[i] > ch + HCS_DEADBAND_H)
        {
            cls[i] = Cls::CEILING;
            if (HCS_AUTO_LOWER)
            {
                dir[i] = -1;
                goal[i] = ch;
            }
        }
        else if (fabsf(e) <= HCS_DEADBAND_H)
        {
            cls[i] = Cls::WITHIN;
            // track slow pressure drift only while the corner really sits on target (applied below, and only
            // when no event is open)
            rerefWithin[i] = pOk[i] && fabsf(e) <= 0.5f * HCS_DEADBAND_H;
        }
        else if (!pv)
        {
            // No pressure reference (fresh preset commit across a drive, pressure sensor fault...). Cannot tell
            // terrain from air loss -> fail-safe: accept the geometry, never pump.
            cls[i] = pOk[i] ? Cls::SHIFT : Cls::AMBIGUOUS;
        }
        else if (e < 0)
        {
            if (dp <= HCS_PRESSURE_NOISE_PSI)
                cls[i] = Cls::AIR_LOSS; // compression always raises bag pressure; this did not -> less air
            else if (g == G_UP)
                cls[i] = Cls::LOAD;
            else if (g == G_NEUTRAL)
                cls[i] = Cls::SHIFT;
            else
                cls[i] = Cls::AMBIGUOUS;
            if (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::LOAD)
                dir[i] = +1;
        }
        else
        {
            if (dp >= -HCS_PRESSURE_NOISE_PSI)
                cls[i] = Cls::AIR_GAIN; // thermal expansion: hold, it reverses itself when the bag cools
            else if (g == G_DOWN)
                cls[i] = Cls::UNLOAD;
            else if (g == G_NEUTRAL)
                cls[i] = Cls::SHIFT;
            else
                cls[i] = Cls::AMBIGUOUS;
            if (cls[i] == Cls::UNLOAD && HCS_AUTO_LOWER)
                dir[i] = -1;
        }

        if (cls[i] == Cls::SHIFT)
        {
            // Load moved between corners at constant total (terrain / crown / slope / someone moved):
            // this IS the surface the car is standing on. Accept it as the new target -- never fight it.
            float nt = fminf_(hm[i], ch);
            if (arrival && presetValid && fabsf(nt - presetH[i]) <= HCS_ANCHOR_TOL &&
                fabsf(nt - presetH[i]) < fabsf(oldTgt - presetH[i]))
                nt = presetH[i]; // came back to (near) flat ground: re-anchor on the preset
            k.tgt = nt;
            if (pOk[i])
                k.pRef = pm[i];
            markPersist(false);
        }
        else if (arrival && presetValid && cls[i] != Cls::FROZEN && cls[i] != Cls::AMBIGUOUS && cls[i] != Cls::BOTTOM_GUARD &&
                 cls[i] != Cls::CEILING && fabsf(k.tgt - presetH[i]) <= HCS_ANCHOR_TOL && k.tgt != presetH[i])
        {
            k.tgt = presetH[i]; // non-terrain corner: pull the reference back onto the preset after a drive
            markPersist(false);
        }

        if (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::LOAD || cls[i] == Cls::UNLOAD)
            goal[i] = k.tgt;
        if (cls[i] == Cls::WITHIN && k.completeCls != Cls::NONE && dir[i] == 0 && fabsf(e) <= HCS_LAND_TOL)
            k.completeCls = Cls::NONE;
        if (dir[i] > 0)
            goal[i] = fminf_(goal[i], ch);
        if (dir[i] < 0)
            goal[i] = fmaxf_(goal[i], lt);
        if ((dir[i] > 0 && goal[i] <= hm[i] + 0.5f) || (dir[i] < 0 && goal[i] >= hm[i] - 0.5f))
            dir[i] = 0;

        if (cls[i] != Cls::WITHIN)
        {
            if (periodicEval && cls[i] == k.lastCls)
                logq("  %s %s (unchanged)", CN[i], clsName(cls[i]));
            else
                logf("  %s %s h=%.1f tgt=%.1f->%.1f e=%+.1f dp=%+.1f%s", CN[i], clsName(cls[i]), hm[i], oldTgt, k.tgt, e, dp,
                     pv ? "" : " (no pRef)");
        }
        k.lastCls = cls[i];
    }

    // ---- event latch: freeze pressure references while anything is still pending
    bool candidates = false;
    for (int i = 0; i < NC; i++)
        if (dir[i] != 0 || c[i].completeCls != Cls::NONE)
            candidates = true;
    if (candidates && !eventOpen)
    {
        eventOpen = true;
        eventOpenSince = now;
    }
    else if (!candidates && eventOpen)
        eventOpen = false;
    if (eventOpen && (now - eventOpenSince) > HCS_EVENT_MAX_MS)
    {
        logq("EVENT open for >%lus without resolution -> re-referencing settled corners", (unsigned long)(HCS_EVENT_MAX_MS / 1000));
        eventOpenSince = now;
        for (int i = 0; i < NC; i++)
            if (rerefWithin[i])
                c[i].pRef = pm[i];
    }
    else if (!eventOpen)
    {
        for (int i = 0; i < NC; i++)
            if (rerefWithin[i])
                c[i].pRef = pm[i];
    }

    // ---- arbiter: global vetoes
    const char *veto = nullptr;
    if (!in.presence)
        veto = "no BLE client (vehicle-off gate)";
    else if (batchesLastHour() >= HCS_MAX_BATCHES_PER_HOUR)
        veto = "hourly batch budget exhausted";

    // ---- arbiter: per-corner gates
    for (int i = 0; i < NC; i++)
    {
        if (dir[i] == 0)
            continue;
        Corner &k = c[i];
        const char *no = nullptr;
        if ((now - k.lastCorrAt) < HCS_MIN_DWELL_MS)
            no = "min dwell";
        else if (k.lastCorrDir == -dir[i] && (now - k.lastCorrAt) < HCS_REVERSAL_LOCK_MS && k.lastCorrEpisode == episodeId)
            no = "reversal lock (no oscillation)";
        else if (dir[i] > 0 && k.leakFault && !k.leakAllowance)
            no = "leak fault latched";
        else if (dir[i] > 0 && !pOk[i])
            no = "bag pressure unknown (cannot bound fill)";
        else if (dir[i] > 0 && !tankValid)
            no = "tank sensor invalid";
        else if (dir[i] > 0 && pm[i] >= in.bagCeilPsi - HCS_BAG_P_MARGIN)
            no = "bag pressure ceiling";
        else if (dir[i] > 0 && pm[i] + HCS_TANK_HEADROOM_PSI > in.compressorOffPsi)
            no = "tank can never reach headroom";
        else if (dir[i] > 0 && tank < pm[i] + HCS_TANK_HEADROOM_PSI)
            no = "waiting for tank headroom";
        else if (dir[i] > 0 && fillMsLastHour() >= HCS_MAX_FILL_MS_PER_HOUR)
            no = "hourly fill budget exhausted";
        if (no)
        {
            logq("  REFUSE %s %s %s: %s", CN[i], clsName(cls[i]), dir[i] > 0 ? "fill" : "dump", no);
            dir[i] = 0;
        }
    }

    // ---- pick ONE batch: the axle holding the largest error, same direction, max 2 corners
    int best = -1;
    float bestE = 0;
    for (int i = 0; i < NC; i++)
    {
        if (dir[i] == 0)
            continue;
        float m = fabsf(goal[i] - hm[i]);
        if (m > bestE + 1e-3f)
        {
            best = i;
            bestE = m;
        }
    }
    st = State::PARKED;
    if (best < 0)
        return;

    bool inB[NC] = {false, false, false, false};
    int watchers = 0;
    for (int i = 0; i < NC; i++)
        inB[i] = dir[i] == dir[best] && isFront(i) == isFront(best);
    for (int i = 0; i < NC; i++)
        if (!inB[i] && hOk[i])
            watchers++;
    if (!veto && watchers < 2)
        veto = "fewer than 2 healthy corners left to watch for motion";

    char what[96];
    int w = 0;
    for (int i = 0; i < NC && w < (int)sizeof(what) - 24; i++)
        if (inB[i])
            w += snprintf(what + w, sizeof(what) - w, "%s %s %.1f->%.0f ", CN[i], clsName(cls[i]), hm[i], goal[i]);

    if (veto)
    {
        char who[40];
        int ww = 0;
        for (int i = 0; i < NC; i++)
            if (inB[i])
                ww += snprintf(who + ww, sizeof(who) - ww, "%s/%s ", CN[i], clsName(cls[i]));
        logq("VETO (%s): would %s %s", veto, dir[best] > 0 ? "fill" : "dump", who);
        return;
    }

    // ---- commit the batch
    batchT[batchI] = now;
    batchI = (batchI + 1) % HCS_MAX_BATCHES_PER_HOUR;
    for (int i = 0; i < NC; i++)
    {
        if (!inB[i])
            continue;
        Corner &k = c[i];
        k.lastCorrAt = now;
        k.lastCorrDir = dir[i];
        k.lastCorrEpisode = episodeId;
        bool wasCompletion = k.completeCls != Cls::NONE;
        k.completeCls = Cls::NONE;
        if (wasCompletion)
            k.completeDir = -2; // marker: this batch IS the completion; finalize must not schedule another
        if (dir[i] > 0 && !wasCompletion && (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::BOTTOM_GUARD))
        {
            for (int j = HCS_LEAK_FAULT_COUNT - 1; j > 0; j--)
                k.leakT[j] = k.leakT[j - 1];
            k.leakT[0] = now;
            int n = 0;
            for (int j = 0; j < HCS_LEAK_FAULT_COUNT; j++)
                if ((now - k.leakT[j]) < HCS_LEAK_WINDOW_MS)
                    n++;
            if (k.leakAllowance)
                k.leakAllowance = false;
            else if (n >= HCS_LEAK_FAULT_COUNT && !k.leakFault)
            {
                k.leakFault = true;
                logf("LEAK %s: %d refills inside %luh -> latched; no further autonomous refills of this corner (check the bag/lines)",
                     CN[i], n, (unsigned long)(HCS_LEAK_WINDOW_MS / HOUR_MS));
            }
        }
    }

    if (shadow)
    {
        logf("SHADOW would START %s %s", dir[best] > 0 ? "fill" : "dump", what);
        return;
    }

    for (int i = 0; i < NC; i++)
    {
        if (!inB[i])
            continue;
        Corner &k = c[i];
        out.g[i].active = true;
        out.g[i].dir = dir[i];
        float gt = goal[i];
        if (gt < 0)
            gt = 0;
        if (gt > 100)
            gt = 100;
        out.g[i].target = (uint8_t)lroundf(gt);
        out.g[i].ceilH = ch + 1.0f;
        out.g[i].floorH = liftTarget(in, i) - 1.0f;
        out.g[i].ceilP = in.bagCeilPsi - HCS_BAG_P_MARGIN;
        k.inBatch = true;
        k.batchDir = dir[i];
        k.batchCls = cls[i];
        k.batchH0 = hm[i];
        k.batchP0 = pOk[i] ? pm[i] : NAN;
        k.valveOpenSince = now;
    }
    for (int i = 0; i < NC; i++)
        batchAllH0[i] = hm[i];
    out.cmd = Cmd::START;
    abortRun = 0;
    batchStart = now;
    batchSettling = false;
    st = State::CORRECTING;
    logf("START %s %s(batch %d/h, fill %lus/h used)", dir[best] > 0 ? "fill" : "dump", what, batchesLastHour(),
         (unsigned long)(fillMsLastHour() / 1000));
}

// ---------------------------------------------------------------------------------------------------------
// Correction in flight
// ---------------------------------------------------------------------------------------------------------

void Core::abortBatch(Outputs &out, const char *why, State next)
{
    out.cmd = Cmd::ABORT;
    for (int i = 0; i < NC; i++)
    {
        if (!c[i].inBatch)
            continue;
        out.g[i].active = true;
        c[i].inBatch = false;
    }
    if (!batchSettling)
    {
        fillT[fillI] = now;
        fillMs[fillI] = now - batchStart;
        fillI = (fillI + 1) % 16;
    }
    logf("ABORT batch: %s", why);
    batchSettling = false;
    st = next;
    needEval = true;
}

void Core::tickCorrecting(const Inputs &in, Outputs &out)
{
    if (!in.presence)
        return abortBatch(out, "BLE presence lost", State::PARKED);
    if (!in.enabled || in.safetyMode)
        return abortBatch(out, "supervisor disabled", State::INACTIVE);
    if (heightFaultCount() >= 2)
        return abortBatch(out, "multiple height sensor faults", State::FAULT);
    for (int i = 0; i < NC; i++)
        if (c[i].inBatch && (c[i].hFault || (c[i].batchDir > 0 && c[i].pFault)))
            return abortBatch(out, "sensor fault on a corrected corner", State::PARKED);
    abortRun = (lastStrong >= 1 || lastVoting >= 2) ? abortRun + 1 : 0;
    if (inEpisode || abortRun >= HCS_ABORT_TICKS)
        return abortBatch(out, "motion detected on the watching corners", State::MOTION);
    if (!batchSettling)
    {
        float maxIdle = 0, maxBatch = 0;
        int idleC = -1;
        for (int i = 0; i < NC; i++)
        {
            if (c[i].hFault || !finite_(batchAllH0[i]))
                continue;
            float d = fabsf(in.c[i].h - batchAllH0[i]);
            if (c[i].inBatch)
                maxBatch = fmaxf_(maxBatch, d);
            else if (d > maxIdle)
            {
                maxIdle = d;
                idleC = i;
            }
        }
        if (idleC >= 0 && maxIdle >= HCS_MAPPING_LIVE_DH && maxBatch < HCS_MAPPING_LIVE_STILL && abortRun == 0)
        {
            mappingFault = true;
            logf("FAULT MAPPING (live): idle %s moved %.1f%% while the actuated corner(s) moved %.1f%% -> wiring mismatch "
                 "suspected; ALL autonomous actuation frozen until reboot",
                 CN[idleC], maxIdle, maxBatch);
            return abortBatch(out, "valve/sensor corner mapping mismatch", State::FAULT);
        }
    }
    if (!batchSettling && (now - batchStart) > HCS_BATCH_WATCHDOG_MS)
        return abortBatch(out, "batch watchdog", State::PARKED);
    for (int i = 0; i < NC; i++)
        if (c[i].inBatch && c[i].valveWasOpen && (now - c[i].valveOpenSince) > HCS_VALVE_MAX_OPEN_MS)
            return abortBatch(out, "valve open too long (watchdog)", State::PARKED);

    bool running = false;
    for (int i = 0; i < NC; i++)
        if (c[i].inBatch && (in.c[i].routineActive || in.c[i].inOpen || in.c[i].outOpen))
            running = true;
    if (running)
        return;
    if (!batchSettling)
    {
        batchSettling = true;
        settleStart = now;
        bool up = false;
        for (int i = 0; i < NC; i++)
            if (c[i].inBatch && c[i].batchDir > 0)
                up = true;
        fillT[fillI] = now;
        fillMs[fillI] = up ? (now - batchStart) : 0;
        fillI = (fillI + 1) % 16;
        return;
    }
    if ((now - settleStart) < HCS_POST_CORRECTION_SETTLE_MS)
        return;
    finalizeBatch(in);
}

void Core::finalizeBatch(const Inputs &in)
{
    (void)in;
    // Mapping check: the corner we actuated must be the one whose height moved the most.
    {
        float maxOther = 0, minBatch = 1e9f;
        int other = -1, bc = -1;
        for (int i = 0; i < NC; i++)
        {
            float hm, pm;
            bool pOk, stable;
            if (c[i].hFault || !finite_(batchAllH0[i]) || !windowMean(i, hm, pm, pOk, stable))
                continue;
            float d = fabsf(hm - batchAllH0[i]);
            if (c[i].inBatch)
            {
                if (d < minBatch)
                {
                    minBatch = d;
                    bc = i;
                }
            }
            else if (d > maxOther)
            {
                maxOther = d;
                other = i;
            }
        }
        if (bc >= 0 && other >= 0 && maxOther >= HCS_MAPPING_MIN_DH && maxOther > HCS_MAPPING_RATIO * minBatch)
        {
            mappingFault = true;
            logf("FAULT MAPPING: actuated %s moved %.1f%% but %s moved %.1f%% -> valve/height/pressure corner wiring "
                 "mismatch suspected; ALL autonomous actuation frozen until reboot",
                 CN[bc], minBatch, CN[other], maxOther);
        }
    }
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!k.inBatch)
            continue;
        k.inBatch = false;
        float hm, pm;
        bool pOk, stable;
        if (!windowMean(i, hm, pm, pOk, stable))
        {
            logf("RESULT %s: no settled reading", CN[i]);
            continue;
        }
        float dh = hm - k.batchH0;
        float dpp = (pOk && finite_(k.batchP0)) ? pm - k.batchP0 : 0;
        logf("RESULT %s %s %+d: h %.1f->%.1f (tgt %.1f, e=%+.1f) p %+.1f", CN[i], clsName(k.batchCls), k.batchDir, k.batchH0, hm, k.tgt,
             hm - k.tgt, dpp);
        float short_ = (k.batchDir > 0) ? (k.tgt - hm) : (hm - k.tgt);
        bool wasCompletion = k.completeDir == -2;
        k.completeDir = 0;
        if (!wasCompletion && k.batchCls != Cls::NONE && short_ > HCS_LAND_TOL && fabsf(dh) >= HCS_NO_RESPONSE_DH)
        {
            k.completeCls = k.batchCls; // one follow-up, keeps the original evidence
            k.completeDir = k.batchDir;
            logf("  %s landed %.1f short -> one completion attempt after dwell", CN[i], short_);
        }
        else
            k.completeCls = Cls::NONE;
        if (fabsf(dh) < HCS_NO_RESPONSE_DH && fabsf(dpp) >= HCS_NO_RESPONSE_DP && !(k.hFault & HF_NO_RESPONSE))
        {
            k.hFault |= HF_NO_RESPONSE;
            logf("FAULT %s height did not follow the bag (dp=%+.1f psi, dh=%+.1f) -> corner frozen until next preset/manual", CN[i], dpp, dh);
        }
        // pRef deliberately NOT updated here: a fill at constant load leaves bag pressure where it was, and the
        // event's evidence (e.g. the total-load rise) must survive until every affected corner is resolved.
        // The WITHIN rule re-references once the event closes.
    }
    batchSettling = false;
    st = mappingFault ? State::FAULT : State::PARKED;
    needEval = true; // the next corner/axle may still need work (dwell keeps the same corner from repeating)
    markPersist(false);
}

// ---------------------------------------------------------------------------------------------------------
// Manual override
// ---------------------------------------------------------------------------------------------------------

void Core::enterManual(const Inputs &in, const char *why, Outputs &out)
{
    (void)in;
    if (st == State::CORRECTING)
        abortBatch(out, "manual override", State::MANUAL);
    if (!manualActive)
    {
        logf("MANUAL override (%s): supervisor yields", why);
        manualSawDrive = false;
    }
    manualActive = true;
    manualIdle = false;
    st = State::MANUAL;
}

void Core::tickManual(const Inputs &in)
{
    st = State::MANUAL;
    bool idle = !anyRoutine(in);
    for (int i = 0; i < NC; i++)
        if (in.c[i].inOpen || in.c[i].outOpen)
            idle = false;
    if (!idle)
    {
        manualIdle = false;
        return;
    }
    if (!manualIdle)
    {
        manualIdle = true;
        manualIdleSince = now;
    }
    if ((now - manualIdleSince) < HCS_MANUAL_SETTLE_MS || inEpisode)
        return;

    float hm[NC], pm[NC];
    bool pOk[NC], stable[NC], have[NC];
    for (int i = 0; i < NC; i++)
    {
        have[i] = windowMean(i, hm[i], pm[i], pOk[i], stable[i]);
        if (c[i].touched && c[i].hFault == 0 && (!have[i] || !stable[i]))
            return; // wait for a steady reading of every corner the user moved
    }

    const float ch = ceilH();
    if (presetPending)
    {
        presetValid = true;
        for (int i = 0; i < NC; i++)
            presetH[i] = pendingPreset[i];
    }
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!k.touched)
            continue; // corners the user did NOT move keep their targets (legacy bug: one global re-baseline)
        k.touched = false;
        if (!have[i])
            continue;
        float t = presetPending ? presetH[i] : hm[i];
        float fb = floorBase(in, i);
        k.floorOverride = (t < fb) ? t : -1; // user explicitly put it below min ride: hold it there, do not lift
        k.tgt = fminf_(t, ch);
        // The pressure reference is only meaningful if the car has not driven since the user's command.
        k.pRef = (pOk[i] && !manualSawDrive) ? pm[i] : NAN;
        if (pOk[i])
            k.pScale = pm[i]; // magnitude only: fine even across a drive
        k.tgtValid = true;
        k.leakFault = k.leakAllowance = false;
        k.hFault &= ~HF_NO_RESPONSE;
        logf("COMMIT %s %s tgt=%.1f pRef=%.1f%s", CN[i], presetPending ? "preset" : "manual", k.tgt, k.pRef,
             k.floorOverride >= 0 ? " (user-held below min ride)" : "");
    }
    presetPending = false;
    manualActive = false;
    bootPending = false; // a user command defines a known state; no need for the long post-boot quiet
    if (manualSawDrive)
        armedAfterDrive = true;
    manualSawDrive = false;
    st = State::PARKED;
    needEval = true;
    lastEvalAt = now;
    markPersist(true);
}

// ---------------------------------------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------------------------------------

int Core::dump(char *buf, size_t n) const
{
    int w = snprintf(buf, n, "HCSD t=%lu st=%s%s drv=%d ep=%lu q=%lus inc=%d vote=%d tank=%.1f ext=%d bat=%d/%d fill=%lus |",
                     (unsigned long)now, stateName(st), mappingFault ? "(MAPPING)" : "", driving ? 1 : 0, (unsigned long)episodeId, (unsigned long)((now - lastMotion) / 1000),
                     lastIncluded, lastVoting, tank, externalFreeze ? 1 : 0, batchesLastHour(), HCS_MAX_BATCHES_PER_HOUR,
                     (unsigned long)(fillMsLastHour() / 1000));
    for (int i = 0; i < NC && w > 0 && (size_t)w < n; i++)
    {
        const Corner &k = c[i];
        float hl = k.nh ? k.wh[(k.ih + HCS_WINDOW - 1) % HCS_WINDOW] : NAN;
        float pl = k.np ? k.wp[(k.ip + HCS_WINDOW - 1) % HCS_WINDOW] : NAN;
        w += snprintf(buf + w, n - w, " %s h=%.1f p=%.1f t=%.1f pr=%.1f aH=%.2f aP=%.2f f=%u%s%s c=%s |", CN[i], hl, pl, k.tgt, k.pRef,
                      k.actH, k.actP, (unsigned)k.hFault, k.pFault ? "P" : "", k.leakFault ? "L" : "", clsName(k.lastCls));
    }
    return w;
}

} // namespace hcs

#endif // HEIGHT_CONTROL_SUPERVISOR || HCS_HOST_BUILD
