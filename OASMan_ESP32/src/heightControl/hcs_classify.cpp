// HCS parked evaluation: classify each corner, arbitrate, start one correction. See hcs_core.h and the design doc.
//
// Physics (quasi-static air spring): bag pressure carries the corner's load, air mass sets the height at that load.
// Versus the moment a corner was last known-good:
//   - a bag whose AIR is unchanged moves along its own p(h) curve: down => pressure up. Down without a pressure rise
//     = it lost air (leak, cooling); up without a pressure drop = it gained air (heat).
//   - people / cargo raise the TOTAL load (mean pressure change > 0); terrain and slopes only move load between
//     corners (mean ~0).
//   - on a rigid body one bag losing air also tilts the others: they are COUPLED and must not be "fixed" or accepted.

#include "hcs_core.h"

#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace hcs
{

enum
{
    L_UNKNOWN,
    L_NEUTRAL,
    L_UP,
    L_DOWN
};
static const char *const LN[] = {"UNKNOWN", "NEUTRAL", "UP", "DOWN"};
static const float TW[NC] = {+1, -1, -1, +1}; // twist sign per corner, FP RP FD RD (FP + RD - FD - RP)

// One corner. dir/goal: the correction it wants (0 = none). airChg: this bag's sign test (-1 lost, +1 gained).
Cls Core::classify(const Inputs &in, int i, float hm, bool pOk, int loadDir, bool airElsewhere, int8_t airChg, int8_t &dir, float &goal)
{
    Corner &k = c[i];
    const float e = hm - k.tgt, ch = ceilH();
    dir = 0;
    goal = k.tgt;
    // Min ride is a hard floor: below it the corner is lifted whatever put it there (leak, load, the ground), even if
    // that raises the other corners. Above the ceiling only a bag that gained air (heat) is lowered.
    if (hm < floorTrigger(in, i))
    {
        dir = +1;
        goal = fminf_(fmaxf_(k.tgt, liftTarget(in, i)), ch);
        return Cls::BOTTOM_GUARD;
    }
    if (hm > ch + HCS_DEADBAND_H)
    {
        if (HCS_AUTO_LOWER && airChg > 0)
            dir = -1, goal = ch;
        return Cls::CEILING;
    }
    if (fabsf(e) <= HCS_DEADBAND_H)
        return Cls::WITHIN;
    if (!pOk || !finite_(k.pRef) || !finite_(k.airP))
        return pOk ? Cls::SHIFT : Cls::AMBIGUOUS; // no reference: cannot tell terrain from a leak -> never pump
    if (e < 0)
    {
        const Cls r = airChg < 0 ? Cls::AIR_LOSS : airElsewhere ? Cls::COUPLED : loadDir == L_UP ? Cls::LOAD
                                                                              : loadDir == L_NEUTRAL ? Cls::SHIFT : Cls::AMBIGUOUS;
        dir = (r == Cls::AIR_LOSS || r == Cls::LOAD) ? +1 : 0;
        return r;
    }
    const Cls r = airChg > 0 ? Cls::AIR_GAIN : airElsewhere ? Cls::COUPLED : loadDir == L_DOWN ? Cls::UNLOAD
                                                                          : loadDir == L_NEUTRAL ? Cls::SHIFT : Cls::AMBIGUOUS;
    dir = (r == Cls::UNLOAD && HCS_AUTO_LOWER) ? -1 : 0;
    return r;
}

// Per-corner gates. Returns why a correction may not start, or nullptr.
const char *Core::gate(const Inputs &in, int i, int8_t dir, bool pOk, float pm) const
{
    const Corner &k = c[i];
    if ((now - k.lastCorrAt) < HCS_MIN_DWELL_MS)
        return "min dwell";
    if (k.lastCorrDir == -dir && (now - k.lastCorrAt) < HCS_REVERSAL_LOCK_MS && k.lastCorrEpisode == episodeId)
        return "reversal lock (no oscillation)";
    if (dir < 0)
        return nullptr;
    if (k.leakFault && !k.leakAllowance)
        return "fast leak latched";
    if (!pOk)
        return "bag pressure unknown (cannot bound the fill)";
    if (!tankValid)
        return "tank sensor invalid";
    if (pm >= in.bagCeilPsi - HCS_BAG_P_MARGIN)
        return "bag pressure ceiling";
    if (pm + HCS_TANK_HEADROOM_PSI > in.compressorOffPsi)
        return "tank can never reach headroom";
    if (tank < pm + HCS_TANK_HEADROOM_PSI)
        return "waiting for tank headroom";
    return nullptr;
}

// Plane deficit: (reference's level plane) - (heights' level plane) per corner, twist removed from both. > 0 = sits
// low there for a reason other than the ground's twist (load, air). Reference = targets unless given.
bool Core::planeDeficit(const float *h, float *d, const float *ref) const
{
    float r[NC], wT = 0, wH = 0;
    for (int i = 0; i < NC; i++)
    {
        if (c[i].hFault || (!ref && !c[i].tgtValid))
            return false;
        r[i] = ref ? ref[i] : c[i].tgt;
        wT += 0.25f * TW[i] * r[i];
        wH += 0.25f * TW[i] * h[i];
    }
    for (int i = 0; i < NC; i++)
        d[i] = (r[i] - TW[i] * wT) - (h[i] - TW[i] * wH);
    return true;
}

// After a drive the old targets belong to the old parking spot. New targets = the preset's plane (what the user asked
// for) + THIS spot's twist (what uneven ground does to a rigid car; never corrected). Load and leaks change the plane,
// so they still show against these targets.
void Core::retargetArrival(const Inputs &in, const float *hm)
{
    float ref[NC], wRef = 0, wCur = 0;
    for (int i = 0; i < NC; i++)
    {
        if (c[i].hFault || !c[i].tgtValid)
            return; // twist needs all four corners
        ref[i] = presetValid ? (float)presetH[i] : c[i].tgt;
        wRef += 0.25f * TW[i] * ref[i];
        wCur += 0.25f * TW[i] * hm[i];
    }
    for (int i = 0; i < NC; i++)
        if (c[i].floorOverride < 0) // user holds it low on purpose
            c[i].tgt = fminf_(fmaxf_(ref[i] + TW[i] * (wCur - wRef), liftTarget(in, i)), ceilH());
    logf("ARRIVAL targets = preset plane + this spot's twist %+.1f: FP %.1f RP %.1f FD %.1f RD %.1f", wCur, c[C_FP].tgt, c[C_RP].tgt,
         c[C_FD].tgt, c[C_RD].tgt);
    markPersist(false);
}

void Core::evaluate(const Inputs &in, Outputs &out, const char *kind, bool arrival)
{
    // ---- 1. settled readings for every healthy corner, or postpone
    float hm[NC], pm[NC];
    bool pOk[NC], stable[NC], hOk[NC];
    for (int i = 0; i < NC; i++)
    {
        hOk[i] = !c[i].hFault;
        const bool have = windowMean(i, hm[i], pm[i], pOk[i], stable[i]);
        if (hOk[i] && (!have || (!c[i].pFault && !pOk[i]) || !stable[i]))
        {
            if (have && !stable[i] && (now - lastPostponeLogAt) > 30000)
            {
                logq("EVAL %s postponed: %s readings not steady", kind, CN[i]);
                lastPostponeLogAt = now;
            }
            st = State::SETTLING;
            return;
        }
    }
    lastEvalAt = now;
    needEval = armedAfterDrive = bootPending = false;
    confirmAt = 0;

    // ---- 2. first baseline (nothing persisted or commanded yet): hold what the car does now
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!hOk[i] || k.tgtValid)
            continue;
        k.tgt = fminf_(hm[i], ceilH());
        k.pRef = k.pScale = k.airP = pOk[i] ? pm[i] : NAN;
        k.airH = hm[i];
        k.tgtValid = true;
        logf("BASELINE %s init tgt=%.1f pRef=%.1f", CN[i], k.tgt, k.pRef);
        markPersist(false);
    }

    // ---- 3. total load (mean dp / pScale) and external support (one bag unloaded: jack / lift / hanging wheel)
    int nl = 0, ext = -1;
    float sum = 0, extF = 0;
    for (int i = 0; i < NC; i++)
    {
        if (!pOk[i] || !finite_(c[i].pRef))
            continue;
        const float f = (pm[i] - c[i].pRef) / fmaxf_(finite_(c[i].pScale) ? c[i].pScale : c[i].pRef, HCS_PREF_MIN_PSI);
        sum += f;
        nl++;
        if (f < -HCS_EXTERNAL_UNLOAD_FRAC && f < extF)
            ext = i, extF = f;
    }
    const float load = nl ? sum / nl : 0;
    // after a drive the load references belong to another spot (bags share the load differently): only air and
    // safety count; load added before leaving is caught on the road (hcs_drive.cpp)
    const int loadDir = arrival && !bootPending ? L_NEUTRAL : nl < 3 ? L_UNKNOWN : load > HCS_LOAD_FRAC ? L_UP : load < -HCS_LOAD_FRAC ? L_DOWN : L_NEUTRAL;
    if (ext >= 0)
    {
        if (!externalFreeze)
            logf("EXTERNAL %s bag lost %.0f%% of its pressure (jack / lift / wheel hanging?) -> frozen, except lifting corners "
                 "below min ride",
                 CN[ext], -extF * 100.0f);
        externalFreeze = true;
        extCorner = ext;
    }
    else if (externalFreeze)
    {
        logf("EXTERNAL cleared");
        externalFreeze = false;
    }
    if (arrival)
        retargetArrival(in, hm);
    const bool periodic = strcmp(kind, "PERIODIC") == 0;
    if (periodic)
        logq("EVAL %s load=%+.1f%% (%s, n=%d) pres=%d", kind, load * 100.0f, LN[loadDir], nl, in.presence ? 1 : 0);
    else
        logf("EVAL %s load=%+.1f%% (%s, n=%d) tank=%.0f pres=%d", kind, load * 100.0f, LN[loadDir], nl, tank, in.presence ? 1 : 0);

    // ---- 4. sign test per bag; >= 3 bags agreeing = temperature, not "another bag's event"
    int8_t airChg[NC];
    int nLoss = 0, nGain = 0;
    for (int i = 0; i < NC; i++)
    {
        airChg[i] = 0;
        if (!hOk[i] || !pOk[i] || !finite_(c[i].airP))
            continue;
        const float dH = hm[i] - c[i].airH, dP = pm[i] - c[i].airP;
        if (dH < -HCS_AIR_SIGN_DH && dP <= HCS_PRESSURE_NOISE_PSI)
            airChg[i] = -1, nLoss++;
        else if (dH > HCS_AIR_SIGN_DH && dP >= -HCS_PRESSURE_NOISE_PSI)
            airChg[i] = +1, nGain++;
    }

    // ---- 5. classify each corner
    // The road is level on average: a corner that sat off the preset level for the whole last drive is off for an AIR
    // reason (a leak the parking spot hides, air added at a severe spot, load) -> ROAD corrects it by that much. Fills
    // use the full deficit; dumps only the tilt part (heat lifts all four together and goes away), unless we added air
    // since the last arrival (a min-ride lift, drive pulses).
    float heave = 0;
    for (int i = 0; i < NC; i++)
        heave += 0.25f * c[i].roadDef;
    Cls cls[NC];
    int8_t dir[NC];
    float goal[NC];
    bool reref[NC];
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        reref[i] = false;
        if (!hOk[i])
        {
            k.lastCls = cls[i] = Cls::FROZEN;
            dir[i] = 0;
            goal[i] = k.tgt;
            continue;
        }
        bool elsewhere = false;
        for (int j = 0; j < NC; j++)
            elsewhere = elsewhere || (j != i && ((airChg[j] < 0 && nLoss < 3) || (airChg[j] > 0 && nGain < 3)));
        const float oldTgt = k.tgt;
        cls[i] = classify(in, i, hm[i], pOk[i], loadDir, elsewhere, airChg[i], dir[i], goal[i]);
        const bool holdCls = cls[i] == Cls::WITHIN || cls[i] == Cls::SHIFT || cls[i] == Cls::COUPLED || cls[i] == Cls::AIR_GAIN ||
                             cls[i] == Cls::AMBIGUOUS;
        if (externalFreeze)
        {
            if (cls[i] != Cls::BOTTOM_GUARD || i == extCorner)
                dir[i] = 0; // frozen: only the min-ride floor acts, never on the unloaded corner itself
        }
        else if (!k.roadPending && (cls[i] == Cls::SHIFT || (arrival && cls[i] == Cls::WITHIN && k.floorOverride < 0)))
        {
            // terrain, or close to the preset plane on arrival: hold exactly this (references taken on THIS spot)
            k.tgt = fminf_(hm[i], ceilH());
            if (pOk[i])
            {
                k.airH = hm[i];
                k.airP = pm[i];
                if (cls[i] == Cls::SHIFT)
                    k.pRef = pm[i];
            }
            markPersist(false);
        }
        const float dumpBy = ownAir ? k.roadDef : fmaxf_(k.roadDef, k.roadDef - heave);
        const bool roadFill = k.roadDef > HCS_DEADBAND_H, roadDump = HCS_AUTO_LOWER && dumpBy < -HCS_DEADBAND_H;
        if (!externalFreeze && holdCls && k.floorOverride < 0)
        {
            if (arrival && roadDefValid && (roadFill || roadDump))
            {
                k.tgt = fminf_(fmaxf_(hm[i] + (roadFill ? k.roadDef : dumpBy), 0), ceilH());
                k.roadPending = true; // finish it whatever this spot makes it look like, until inside the deadband
                markPersist(false);
            }
            if (k.roadPending && fabsf(hm[i] - k.tgt) > HCS_DEADBAND_H && (hm[i] < k.tgt || HCS_AUTO_LOWER))
            {
                cls[i] = Cls::ROAD;
                dir[i] = hm[i] < k.tgt ? +1 : -1;
                goal[i] = k.tgt;
            }
            else
                k.roadPending = false;
        }
        if (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::LOAD || cls[i] == Cls::UNLOAD)
            goal[i] = k.tgt;
        reref[i] = cls[i] == Cls::WITHIN && pOk[i] && fabsf(hm[i] - k.tgt) <= 0.5f * HCS_DEADBAND_H;
        goal[i] = dir[i] > 0 ? fminf_(goal[i], ceilH()) : dir[i] < 0 ? fmaxf_(goal[i], liftTarget(in, i)) : goal[i];
        if ((dir[i] > 0 && goal[i] <= hm[i] + 0.5f) || (dir[i] < 0 && goal[i] >= hm[i] - 0.5f))
            dir[i] = 0;
        if (cls[i] != Cls::WITHIN)
        {
            if (!arrival && cls[i] == k.lastCls)
                logq("  %s %s (unchanged)", CN[i], clsName(cls[i]));
            else
                logf("  %s %s h=%.1f tgt=%.1f->%.1f e=%+.1f dp=%+.1f air=%s", CN[i], clsName(cls[i]), hm[i], oldTgt, k.tgt, hm[i] - oldTgt,
                     (pOk[i] && finite_(k.pRef)) ? pm[i] - k.pRef : 0.0f, airChg[i] < 0 ? "lost" : airChg[i] > 0 ? "gained" : "same");
        }
        k.lastCls = cls[i];
    }
    if (arrival)
        roadDefValid = ownAir = false; // used once, on the parking right after that drive

    // ---- 6. event latch: while anything is pending, keep the load references (the evidence) frozen
    bool pending = false;
    for (int i = 0; i < NC; i++)
        pending = pending || dir[i] != 0;
    if (pending && !eventOpen)
        eventOpen = true, eventOpenSince = now;
    else if (!pending)
        eventOpen = false;
    const bool forceReref = eventOpen && (now - eventOpenSince) > HCS_EVENT_MAX_MS;
    if (forceReref)
    {
        logq("EVENT open for >%lus -> re-referencing settled corners", (unsigned long)(HCS_EVENT_MAX_MS / 1000));
        eventOpenSince = now;
    }
    for (int i = 0; i < NC; i++)
        if (reref[i] && (!eventOpen || forceReref) && !externalFreeze)
            c[i].pRef = pm[i];

    // ---- 7. confirmation: every corner steady for HCS_CONFIRM_MS with no motion (a dip / hill bottom moves the car)
    const uint32_t left = pending ? steadyRemaining() : 0;
    if (left != 0)
    {
        const uint32_t wait = left == 0xFFFFFFFFUL ? (uint32_t)HCS_CONFIRM_MS : left;
        for (int i = 0; i < NC; i++)
            if (dir[i] != 0)
            {
                logq("  %s %s: wanted, confirming in %lus", CN[i], clsName(cls[i]), (unsigned long)((wait + 999) / 1000));
                dir[i] = 0;
            }
        confirmAt = now + wait + 100;
    }

    // ---- 8. arbiter: per-corner gates, then ONE batch = the axle with the largest error, same direction
    for (int i = 0; i < NC; i++)
        if (dir[i] != 0)
            if (const char *no = gate(in, i, dir[i], pOk[i], pm[i]))
            {
                logq("  REFUSE %s %s %s: %s", CN[i], clsName(cls[i]), dir[i] > 0 ? "fill" : "dump", no);
                dir[i] = 0;
            }
    int best = -1;
    float bestE = 0;
    for (int i = 0; i < NC; i++)
        if (dir[i] != 0 && fabsf(goal[i] - hm[i]) > bestE + 1e-3f)
            best = i, bestE = fabsf(goal[i] - hm[i]);
    st = State::PARKED;
    if (best < 0)
        return;
    bool inB[NC];
    int watchers = 0;
    for (int i = 0; i < NC; i++)
    {
        inB[i] = dir[i] == dir[best] && isFront(i) == isFront(best);
        watchers += (!inB[i] && hOk[i]) ? 1 : 0;
    }
    const char *veto = !in.presence                                    ? "no BLE client (vehicle-off gate)"
                       : batchesLastHour() >= HCS_MAX_BATCHES_PER_HOUR ? "hourly batch budget exhausted"
                       : watchers < HCS_MOTION_MIN_CORNERS             ? "fewer than 2 healthy corners left to watch for motion"
                                                                       : nullptr;
    if (veto)
    {
        char who[64] = "";
        size_t w = 0;
        for (int i = 0; i < NC; i++)
            if (inB[i])
                appendf(who, sizeof(who), w, "%s/%s ", CN[i], clsName(cls[i]));
        logq("VETO (%s): would %s %s", veto, dir[best] > 0 ? "fill" : "dump", who);
        return;
    }
    startBatch(in, out, inB, dir, goal, cls, hm, pm, pOk);
}

} // namespace hcs

#endif
