// HCS state machine, user override, correction execution, persistence, logging. See hcs_core.h.

#include "hcs_core.h"

#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace hcs
{

const char *Core::stateName(State s)
{
    static const char *const N[] = {"INIT", "INACTIVE", "MOTION", "SETTLING", "PARKED", "CORRECTING", "MANUAL", "FAULT"};
    return N[(int)s];
}

const char *Core::clsName(Cls c)
{
    static const char *const N[] = {"NONE",  "WITHIN",   "AIR_LOSS", "LOAD",         "SHIFT",   "COUPLED",
                                    "AMBIG", "AIR_GAIN", "UNLOAD",   "BOTTOM_GUARD", "CEILING", "ROAD", "FROZEN"};
    return N[(int)c];
}

void appendf(char *buf, size_t n, size_t &w, const char *fmt, ...)
{
    if (n == 0 || w + 1 >= n)
        return;
    va_list ap;
    va_start(ap, fmt);
    const int r = vsnprintf(buf + w, n - w, fmt, ap);
    va_end(ap);
    w = r < 0 ? n - 1 : (w + (size_t)r >= n ? n - 1 : w + (size_t)r);
}

// ---- logging: "HCS t=<ms> ...". logq() drops a line identical to one printed < HCS_LOG_THROTTLE_MS ago.
void Core::vlog(bool throttle, const char *fmt, va_list ap)
{
    if (!logFn)
        return;
    char body[200];
    vsnprintf(body, sizeof(body), fmt, ap);
    if (throttle)
    {
        uint32_t h = 2166136261u; // FNV-1a
        for (const char *p = body; *p; p++)
            h = (h ^ (uint8_t)*p) * 16777619u;
        int slot = -1, oldest = 0;
        for (int i = 0; i < 8; i++)
        {
            if (thrHash[i] == h)
                slot = i;
            if ((now - thrAt[i]) > (now - thrAt[oldest]))
                oldest = i;
        }
        if (slot >= 0 && (now - thrAt[slot]) < HCS_LOG_THROTTLE_MS)
            return;
        slot = slot < 0 ? oldest : slot;
        thrHash[slot] = h;
        thrAt[slot] = now;
    }
    char line[224];
    snprintf(line, sizeof(line), "HCS t=%lu %s", (unsigned long)now, body);
    logFn(line);
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

// ---- boot + persistence
void Core::begin(uint32_t t, const PersistBlob *r, bool sh, LogFn lf)
{
    memset((void *)this, 0, sizeof(*this)); // POD members only; non-zero defaults below
    now = bootAt = lastMotion = lastEvalAt = eventOpenSince = manualIdleSince = lastPersistAt = t;
    calmSince = pulseEnd = pulseAt = t;
    shadow = sh;
    logFn = lf;
    st = State::INIT;
    bootPending = needEval = true; // first evaluation after boot uses the long (arrival) quiet
    pulseCorner = -1;
    const uint32_t old = t - 2 * HOUR_MS; // "long ago" (unsigned maths)
    lastPostponeLogAt = old;
    for (int i = 0; i < HCS_MAX_BATCHES_PER_HOUR; i++)
        batchT[i] = old;
    for (int i = 0; i < 8; i++)
        thrAt[i] = old;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        k.idleSince = k.goodSinceH = k.goodSinceP = t;
        k.lastSeq = 0xFFFFFFFF;
        k.pRef = k.pScale = k.airH = k.airP = k.fillRate = NAN;
        k.floorOverride = -1;
        k.lastCorrAt = old;
        k.lastCorrEpisode = 0xFFFFFFFF;
    }
    if (!r || r->magic != PERSIST_MAGIC || r->version != PERSIST_VERSION)
    {
        logf("no persisted state -> starting unanchored (baseline = first settled reading)");
        return;
    }
    // learned fill rates: checked on their own, kept even if the anchor state is rejected
    for (int i = 0; i < NC; i++)
        if (finite_(r->fillRate[i]) && r->fillRate[i] > 0.1f && r->fillRate[i] < 50.0f)
            c[i].fillRate = r->fillRate[i];
    logf("RESTORED fill rate %.2f/%.2f/%.2f/%.2f %%/s", c[0].fillRate, c[1].fillRate, c[2].fillRate, c[3].fillRate);
    bool ok = true;
    int nValid = 0;
    for (int i = 0; i < NC; i++)
    {
        const float ps[3] = {r->pRef[i], r->pScale[i], r->airP[i]}, hs[2] = {r->tgt[i], r->airH[i]};
        for (float f : ps)
            ok = ok && (!finite_(f) || (f >= HCS_P_MIN_VALID && f <= HCS_P_MAX_VALID));
        for (float f : hs)
            ok = ok && (!finite_(f) || (f >= 0 && f <= 100));
        ok = ok && !(finite_(r->floorOverride[i]) && r->floorOverride[i] > 100) && r->presetH[i] <= 100;
        nValid += finite_(r->tgt[i]) ? 1 : 0;
    }
    if (!ok || nValid == 0)
    {
        logf("persisted state rejected (out of range) -> starting unanchored");
        return;
    }
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        k.tgtValid = finite_(r->tgt[i]);
        k.tgt = k.tgtValid ? r->tgt[i] : 0;
        k.floorOverride = (finite_(r->floorOverride[i]) && r->floorOverride[i] >= 0) ? r->floorOverride[i] : -1;
        k.pRef = r->pRef[i];
        k.pScale = r->pScale[i];
        const bool air = finite_(r->airH[i]) && finite_(r->airP[i]);
        k.airH = air ? r->airH[i] : NAN;
        k.airP = air ? r->airP[i] : NAN;
        presetH[i] = r->presetH[i];
    }
    presetValid = r->presetValid != 0;
    logf("RESTORED tgt=%.1f/%.1f/%.1f/%.1f pRef=%.1f/%.1f/%.1f/%.1f preset=%d", c[0].tgt, c[1].tgt, c[2].tgt, c[3].tgt, c[0].pRef,
         c[1].pRef, c[2].pRef, c[3].pRef, presetValid ? 1 : 0);
}

bool Core::exportPersist(PersistBlob &b) const
{
    memset(&b, 0, sizeof(b));
    b.magic = PERSIST_MAGIC;
    b.version = PERSIST_VERSION;
    b.presetValid = presetValid ? 1 : 0;
    bool any = false;
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        b.presetH[i] = (uint8_t)(presetValid ? presetH[i] : 0);
        b.tgt[i] = k.tgtValid ? k.tgt : NAN;
        b.floorOverride[i] = k.floorOverride;
        b.pRef[i] = k.pRef;
        b.pScale[i] = k.pScale;
        b.airH[i] = k.airH;
        b.airP[i] = k.airP;
        b.fillRate[i] = k.fillRate;
        any = any || k.tgtValid || finite_(k.fillRate);
    }
    return any;
}

void Core::markPersist(bool immediate)
{
    dirty = true;
    persistImmediate = persistImmediate || immediate;
}

void Core::notifyPresetLoad(const uint8_t heights[NC])
{
    for (int i = 0; i < NC; i++)
    {
        pendingPreset[i] = heights[i];
        c[i].touched = true;
    }
    presetPending = pendingManual = true;
}

// ---- helpers
float Core::floorBase(const Inputs &in, int i) const { return in.c[i].minRide > 0.5f ? in.c[i].minRide : (float)HCS_ABS_FLOOR; }
// floorOverride: the user put this corner below min ride on purpose (show / stance preset): hold it there.
float Core::floorTrigger(const Inputs &in, int i) const
{
    return c[i].floorOverride >= 0 ? c[i].floorOverride - HCS_DEADBAND_H : floorBase(in, i);
}
float Core::liftTarget(const Inputs &in, int i) const
{
    return c[i].floorOverride >= 0 ? c[i].floorOverride : fminf_(floorBase(in, i) + HCS_FLOOR_LIFT_MARGIN, ceilH());
}
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
        n += c[i].hFault ? 1 : 0;
    return n;
}
int Core::batchesLastHour() const
{
    int n = 0;
    for (int i = 0; i < HCS_MAX_BATCHES_PER_HOUR; i++)
        n += (now - batchT[i]) < HOUR_MS ? 1 : 0;
    return n;
}

// ---- tick
void Core::tick(const Inputs &in, Outputs &out)
{
    lastDt = in.now - now;
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
    motionDetect();
    driveTick(in, out);
    if (out.cmd == Cmd::NONE)
        tickInner(in, out);
    if (dirty && (persistImmediate || (now - lastPersistAt) >= HCS_PERSIST_MIN_INTERVAL_MS))
    {
        PersistBlob b;
        out.persistNow = exportPersist(b);
        dirty = persistImmediate = false;
        lastPersistAt = now;
    }
}

void Core::tickInner(const Inputs &in, Outputs &out)
{
    if (in.presence && !prevPresence)
    {
        logf("PRESENCE on (BLE client connected) -> evaluate");
        needEval = urgentPending = true;
        for (int i = 0; i < NC; i++)
            if (c[i].leakFault && !c[i].leakAllowance)
            {
                c[i].leakAllowance = true;
                logf("LEAK %s latched: owner returned -> one refill granted", CN[i]);
            }
    }
    else if (!in.presence && prevPresence)
    {
        logf("PRESENCE off -> autonomous actuation vetoed");
        urgentPending = false;
    }
    prevPresence = in.presence;
    if (in.enabled != prevEnabled)
    {
        logf("%s (height-sensor mode + maintain)", in.enabled ? "ENABLED" : "DISABLED");
        needEval = true;
        prevEnabled = in.enabled;
    }

    // The user always wins. Detection is the valve state itself (a valve open / routine running that the supervisor
    // did not start), so every manual path is covered; notifyManual() / notifyPresetLoad() are the fast path.
    bool user = pendingManual;
    const char *why = pendingManual ? (presetPending ? "preset load" : "manual command") : "valve activity";
    pendingManual = false;
    if (user && pulseCorner >= 0)
    {
        logf("PULSE %s released: user command", CN[pulseCorner]);
        pulseCorner = -1; // the adapter already handed the solenoids to the user
    }
    for (int i = 0; i < NC; i++)
    {
        const CornerIn &x = in.c[i];
        const bool mine = (st == State::CORRECTING && c[i].inBatch) || i == pulseCorner;
        if ((x.routineActive && !x.routineAutonomous) || ((x.inOpen || x.outOpen) && !x.routineActive && !mine))
            user = c[i].touched = true;
    }
    if (user)
        enterManual(why, out);
    if (st == State::CORRECTING)
        return tickCorrecting(in, out);
    if (manualActive)
    {
        manualSawDrive = manualSawDrive || driving;
        tickManual(in);
        if (manualActive)
            return;
    }
    if (urgentPending)
    {
        urgentPending = false;
        if (urgentFloorLift(in, out))
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
    if ((now - lastMotion) < (arrival ? (uint32_t)HCS_ARRIVAL_QUIET_MS : (uint32_t)HCS_DISTURB_QUIET_MS))
    {
        st = State::SETTLING;
        return;
    }
    const bool confirmDue = confirmAt != 0 && (int32_t)(now - confirmAt) >= 0;
    if (!needEval && !confirmDue && (now - lastEvalAt) < HCS_EVAL_PERIOD_MS)
    {
        st = State::PARKED;
        return;
    }
    if (anyRoutine(in) || !allIdleFor(HCS_P_SETTLE_MS))
    {
        st = State::SETTLING;
        return;
    }
    const char *kind = bootPending ? "BOOT" : armedAfterDrive ? "ARRIVAL" : confirmDue ? "CONFIRM" : needEval ? "EVENT" : "PERIODIC";
    evaluate(in, out, kind, arrival);
}

// ---- user override: yield, wait until the command has settled, then re-baseline the corners the user touched
void Core::enterManual(const char *why, Outputs &out)
{
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
        idle = idle && !in.c[i].inOpen && !in.c[i].outOpen;
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
        if (c[i].touched && !c[i].hFault && (!have[i] || !stable[i]))
            return; // wait for a steady reading of every corner the user moved
    }
    if (presetPending)
    {
        presetValid = true;
        for (int i = 0; i < NC; i++)
            presetH[i] = pendingPreset[i];
    }
    roadDefValid = false; // the user set a new level
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!k.touched)
            continue; // corners the user did not move keep their targets
        k.touched = false;
        if (!have[i])
            continue;
        const float t = presetPending ? presetH[i] : hm[i];
        k.floorOverride = t < floorBase(in, i) ? t : -1;
        k.tgt = fminf_(t, ceilH());
        k.tgtValid = true;
        k.pRef = (pOk[i] && !manualSawDrive) ? pm[i] : NAN; // load evidence is void if the car drove since
        if (pOk[i])
        {
            k.pScale = k.airP = pm[i]; // air content is valid even across a drive
            k.airH = hm[i];
        }
        k.leakFault = k.leakAllowance = k.roadPending = false;
        k.fastCount = 0;
        k.owe = 0;
        logf("COMMIT %s %s tgt=%.1f pRef=%.1f%s", CN[i], presetPending ? "preset" : "manual", k.tgt, k.pRef,
             k.floorOverride >= 0 ? " (user-held below min ride)" : "");
    }
    presetPending = manualActive = bootPending = false; // a user command defines a known state
    armedAfterDrive = armedAfterDrive || manualSawDrive;
    manualSawDrive = false;
    st = State::PARKED;
    needEval = true;
    lastEvalAt = now;
    markPersist(true);
}

// ---- correction in flight
void Core::startBatch(const Inputs &in, Outputs &out, const bool *inB, const int8_t *dir, const float *goal, const Cls *cls,
                      const float *hm, const float *pm, const bool *pOk)
{
    char what[96] = "";
    size_t w = 0;
    int8_t d = 0;
    for (int i = 0; i < NC; i++)
        if (inB[i])
        {
            d = dir[i];
            appendf(what, sizeof(what), w, "%s %s %.1f->%.0f ", CN[i], clsName(cls[i]), hm[i], goal[i]);
        }
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
        // leak bookkeeping counts real refills only (shadow re-wants the same refill every dwell period)
        if (!shadow && dir[i] > 0 && (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::BOTTOM_GUARD))
            recordRefill(i, goal[i] - hm[i]);
    }
    if (shadow)
    {
        logq("SHADOW would START %s %s", d > 0 ? "fill" : "dump", what);
        return;
    }
    for (int i = 0; i < NC; i++)
    {
        if (!inB[i])
            continue;
        Corner &k = c[i];
        GoalRequest &g = out.g[i];
        g.active = true;
        g.dir = dir[i];
        g.target = (uint8_t)lroundf(fminf_(fmaxf_(goal[i], 0), 100));
        g.ceilH = ceilH() + 1.0f;
        g.floorH = liftTarget(in, i) - 1.0f;
        g.ceilP = in.bagCeilPsi - HCS_BAG_P_MARGIN;
        k.inBatch = true;
        k.batchDir = dir[i];
        k.batchCls = cls[i];
        k.batchH0 = hm[i];
        k.batchP0 = pOk[i] ? pm[i] : NAN;
        k.batchOpenMs = 0;
        k.valveOpenSince = now;
        ownAir = ownAir || (cls[i] == Cls::BOTTOM_GUARD && dir[i] > 0);
    }
    out.cmd = Cmd::START;
    abortRun = 0;
    batchStart = now;
    batchSettling = false;
    st = State::CORRECTING;
    logf("START %s %s(batch %d/h)", d > 0 ? "fill" : "dump", what, batchesLastHour());
}

void Core::abortBatch(Outputs &out, const char *why, State next)
{
    out.cmd = Cmd::ABORT;
    for (int i = 0; i < NC; i++)
        if (c[i].inBatch)
        {
            out.g[i].active = true;
            c[i].inBatch = false;
        }
    logf("ABORT batch: %s", why);
    batchSettling = urgentBatch = false;
    st = next;
    needEval = true;
}

// BLE-connect edge: a corner the last parked evaluation found below min ride (overnight leak) is lifted right away --
// waiting for the car to be quiet would let the driver leave on a bottomed corner. The live reading confirms it is
// still low; every per-corner gate (tank, pressure ceiling, leak latch) still applies.
bool Core::urgentFloorLift(const Inputs &in, Outputs &out)
{
    if (!in.presence || !in.enabled || in.safetyMode || driving || heightFaultCount() >= 2 || (now - lastEvalAt) > HCS_URGENT_MAX_AGE_MS ||
        batchesLastHour() >= HCS_MAX_BATCHES_PER_HOUR)
        return false;
    bool inB[NC], pOk[NC];
    int8_t dir[NC];
    float goal[NC], hm[NC], pm[NC];
    Cls cls[NC];
    int n = 0;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        inB[i] = false;
        dir[i] = 0;
        cls[i] = Cls::NONE;
        hm[i] = in.c[i].h;
        pm[i] = in.c[i].p;
        pOk[i] = !k.pFault;
        goal[i] = fminf_(fmaxf_(k.tgt, liftTarget(in, i)), ceilH());
        if (k.hFault || k.lastCls != Cls::BOTTOM_GUARD || hm[i] >= floorTrigger(in, i) || (externalFreeze && i == extCorner))
            continue;
        if (const char *no = gate(in, i, +1, pOk[i], pm[i]))
        {
            logf("URGENT lift %s refused: %s", CN[i], no);
            continue;
        }
        inB[i] = true;
        dir[i] = +1;
        cls[i] = Cls::BOTTOM_GUARD;
        n++;
    }
    if (!n)
        return false;
    logf("URGENT controller connected with corners below min ride -> lifting now (does not wait for people to settle)");
    startBatch(in, out, inB, dir, goal, cls, hm, pm, pOk);
    urgentBatch = st == State::CORRECTING;
    return out.cmd == Cmd::START || shadow;
}

void Core::tickCorrecting(const Inputs &in, Outputs &out)
{
    if (!in.presence)
        return abortBatch(out, "BLE presence lost", State::PARKED);
    if (!in.enabled || in.safetyMode)
        return abortBatch(out, "supervisor disabled", State::INACTIVE);
    for (int i = 0; i < NC; i++)
        if (c[i].inBatch && (c[i].hFault || (c[i].batchDir > 0 && c[i].pFault)))
            return abortBatch(out, "sensor fault on a corrected corner", State::PARKED);
    // the corners NOT being corrected watch for motion (drive-off, someone getting in); an urgent min-ride lift is
    // only stopped by driving (the rest is topped up on the road)
    abortRun = (lastStrong >= 1 || lastVoting >= 2) ? abortRun + 1 : 0;
    if (urgentBatch ? driving : (inEpisode || abortRun >= HCS_ABORT_TICKS))
        return abortBatch(out, "motion detected on the watching corners", State::MOTION);
    if (!batchSettling && (now - batchStart) > HCS_BATCH_WATCHDOG_MS)
        return abortBatch(out, "batch watchdog", State::PARKED);
    bool running = false;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!k.inBatch)
            continue;
        if (k.valveWasOpen && (now - k.valveOpenSince) > HCS_VALVE_MAX_OPEN_MS)
            return abortBatch(out, "valve open too long (watchdog)", State::PARKED);
        running = running || in.c[i].routineActive || in.c[i].inOpen || in.c[i].outOpen;
        if (in.c[i].inOpen)
            k.batchOpenMs += lastDt;
    }
    if (running)
        return;
    if (!batchSettling)
    {
        batchSettling = true;
        settleStart = now;
    }
    else if ((now - settleStart) >= HCS_POST_CORRECTION_SETTLE_MS)
        finalizeBatch();
}

void Core::finalizeBatch()
{
    urgentBatch = batchSettling = false;
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
        const float dh = hm - k.batchH0;
        if (k.batchDir > 0 && dh > 1.0f && k.batchOpenMs >= 300)
        {
            const float r = dh / (k.batchOpenMs / 1000.0f);
            k.fillRate = finite_(k.fillRate) ? 0.7f * k.fillRate + 0.3f * r : r; // learned, persisted
        }
        logf("RESULT %s %s %+d: h %.1f->%.1f (tgt %.1f) p %+.1f, IN open %lu ms", CN[i], clsName(k.batchCls), k.batchDir, k.batchH0, hm,
             k.tgt, (pOk && finite_(k.batchP0)) ? pm - k.batchP0 : 0.0f, (unsigned long)k.batchOpenMs);
        // pRef stays (event latch); the AIR reference follows: we changed this bag's air on purpose
        if (pOk)
        {
            k.airH = hm;
            k.airP = pm;
        }
    }
    st = State::PARKED;
    needEval = true; // the other axle, or a correction that landed short, is picked up by the next evaluation
    markPersist(false);
}

// Leak bookkeeping: every refill logs a rate; only a FAST leak latches (see HCS_LEAK_FAST_*).
void Core::recordRefill(int i, float deficit)
{
    Corner &k = c[i];
    if (k.haveRefill)
    {
        const uint32_t iv = now - k.lastRefillAt;
        const float hrs = iv / 3600000.0f;
        k.fastCount = iv < HCS_LEAK_FAST_INTERVAL_MS ? k.fastCount + 1 : 0;
        logf("LEAK %s refill: %.1f%% after %.1fh (~%.2f %%/h)%s", CN[i], deficit, hrs, hrs > 0.01f ? deficit / hrs : 0.0f,
             k.fastCount ? " FAST" : "");
    }
    k.haveRefill = true;
    k.lastRefillAt = now;
    if (k.leakAllowance)
        k.leakAllowance = false;
    else if (k.fastCount >= HCS_LEAK_FAST_COUNT && !k.leakFault)
    {
        k.leakFault = true;
        logf("LEAK %s: %d refills < %lumin apart -> FAST leak latched; no more autonomous refills of this corner until a BLE "
             "client reconnects / a preset is loaded (check the bag and fittings)",
             CN[i], k.fastCount + 1, (unsigned long)(HCS_LEAK_FAST_INTERVAL_MS / 60000UL));
    }
}

// ---- diagnostics: one bounded line
int Core::dump(char *buf, size_t n) const
{
    if (n == 0)
        return 0;
    size_t w = 0;
    buf[0] = 0;
    appendf(buf, n, w, "HCSD t=%lu st=%s drv=%d ep=%lu q=%lus vote=%d/%d tank=%.1f ext=%d bat=%d/%d |", (unsigned long)now,
            stateName(st), driving ? 1 : 0, (unsigned long)episodeId, (unsigned long)((now - lastMotion) / 1000), lastVoting,
            lastIncluded, tank, externalFreeze ? 1 : 0, batchesLastHour(), HCS_MAX_BATCHES_PER_HOUR);
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        const float hl = k.nh ? k.wh[(k.ih + HCS_WINDOW - 1) % HCS_WINDOW] : NAN;
        const float pl = k.np ? k.wp[(k.ip + HCS_WINDOW - 1) % HCS_WINDOW] : NAN;
        appendf(buf, n, w, " %s h=%.1f p=%.1f t=%.1f pr=%.1f aH=%.2f aP=%.2f %s%s%s c=%s |", CN[i], hl, pl, k.tgt, k.pRef, k.actH, k.actP,
                k.hFault ? "H" : "", k.pFault ? "P" : "", k.leakFault ? "L" : "", clsName(k.lastCls));
    }
    return (int)w;
}

} // namespace hcs

#endif
