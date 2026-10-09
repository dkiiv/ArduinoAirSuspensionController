// HCS evaluation: classify each corner, arbitrate, start one correction. See hcs_core.h and the design doc.
//
// The physics this rests on (quasi-static air spring): bag pressure carries the corner's load, air mass sets the
// height at that load. So, versus the moment a corner was last known-good:
//   - a bag whose AIR is unchanged moves along its own p(h) curve: down => pressure up, up => pressure down.
//     Down without a pressure rise = it lost air (leak, cooling). Up without a pressure drop = gained air (heat).
//   - people / cargo raise the TOTAL load (mean pressure change > 0); terrain, slopes and cornering only move load
//     between corners (mean ~0).
//   - on a rigid body one bag losing air also tilts the others: they are COUPLED and must not be "fixed" or
//     accepted -- restoring the leaking bag restores them.

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

// One corner. dir/goal: the correction it wants (0 = none). airChg: this bag's sign test (-1 lost, +1 gained).
Cls Core::classify(const Inputs &in, int i, float hm, bool pOk, float pm, int loadDir, bool airElsewhere, int8_t airChg,
                   int8_t &dir, float &goal)
{
    Corner &k = c[i];
    const float e = hm - k.tgt, ch = ceilH();
    dir = 0;
    goal = k.tgt;

    // completion of a correction that landed short: same evidence, same direction, no re-classification
    if (k.completeCls != Cls::NONE)
    {
        const float shortBy = k.completeDir > 0 ? -e : e;
        if (shortBy > HCS_LAND_TOL && hm >= floorTrigger(in, i) && hm <= ch + HCS_DEADBAND_H)
        {
            dir = k.completeDir;
            return k.completeCls;
        }
        k.completeCls = Cls::NONE;
    }
    if (hm < floorTrigger(in, i))
    {
        dir = +1; // below min ride (e.g. leaked down overnight): lift to the target, at least min ride + margin
        goal = fminf_(fmaxf_(k.tgt, liftTarget(in, i)), ch);
        return Cls::BOTTOM_GUARD;
    }
    if (hm > ch + HCS_DEADBAND_H)
    {
        if (HCS_AUTO_LOWER)
        {
            dir = -1;
            goal = ch;
        }
        return Cls::CEILING;
    }
    if (fabsf(e) <= HCS_DEADBAND_H)
        return Cls::WITHIN;
    if (!pOk || !finite_(k.pRef) || !finite_(k.airP))
        return pOk ? Cls::SHIFT : Cls::AMBIGUOUS; // no reference: cannot tell terrain from a leak -> never pump

    Cls r;
    if (e < 0)
    {
        r = airChg < 0 ? Cls::AIR_LOSS
            : airElsewhere ? Cls::COUPLED
            : loadDir == L_UP ? Cls::LOAD
            : loadDir == L_NEUTRAL ? Cls::SHIFT
            : Cls::AMBIGUOUS;
        if (r == Cls::AIR_LOSS || r == Cls::LOAD)
            dir = +1;
    }
    else
    {
        r = airChg > 0 ? Cls::AIR_GAIN
            : airElsewhere ? Cls::COUPLED
            : loadDir == L_DOWN ? Cls::UNLOAD
            : loadDir == L_NEUTRAL ? Cls::SHIFT
            : Cls::AMBIGUOUS;
        if (r == Cls::UNLOAD && HCS_AUTO_LOWER)
            dir = -1;
    }
    (void)pm;
    return r;
}

// Per-corner arbiter gates. Returns why a correction may not start, or nullptr.
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

    // ---- 2. first baseline (nothing persisted or commanded yet): hold what the car is doing now
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (!hOk[i] || k.tgtValid)
            continue;
        k.tgt = fminf_(hm[i], ceilH());
        k.pRef = k.pScale = pOk[i] ? pm[i] : NAN;
        k.airH = hm[i];
        k.airP = k.pRef;
        k.tgtValid = true;
        logf("BASELINE %s init tgt=%.1f pRef=%.1f", CN[i], k.tgt, k.pRef);
        markPersist(false);
    }

    // ---- 3. total load (mean dp / pScale) and external support (jack / lift)
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
        {
            ext = i;
            extF = f;
        }
    }
    const float load = nl ? sum / nl : 0;
    const int loadDir = nl < 3 ? L_UNKNOWN : load > HCS_LOAD_FRAC ? L_UP : load < -HCS_LOAD_FRAC ? L_DOWN : L_NEUTRAL;
    if (ext >= 0)
    {
        if (!externalFreeze)
            logf("EXTERNAL %s bag lost %.0f%% of its pressure (jack / lift / wheel unsupported?) -> autonomous actuation frozen",
                 CN[ext], -extF * 100.0f);
        externalFreeze = true;
        st = State::PARKED;
        return; // something else is holding the car: accept nothing, change nothing
    }
    if (externalFreeze)
    {
        logf("EXTERNAL cleared");
        externalFreeze = false;
    }
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

    // ---- 5. classify; accept terrain; re-anchor on the preset after a drive
    Cls cls[NC];
    int8_t dir[NC];
    float goal[NC];
    bool reref[NC];
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        dir[i] = 0;
        goal[i] = k.tgt;
        reref[i] = false;
        if (!hOk[i])
        {
            k.lastCls = cls[i] = Cls::FROZEN;
            continue;
        }
        bool elsewhere = false;
        for (int j = 0; j < NC; j++)
            if (j != i && ((airChg[j] < 0 && nLoss < 3) || (airChg[j] > 0 && nGain < 3)))
                elsewhere = true;
        const float oldTgt = k.tgt;
        cls[i] = classify(in, i, hm[i], pOk[i], pm[i], loadDir, elsewhere, airChg[i], dir[i], goal[i]);

        if (cls[i] == Cls::SHIFT)
        {
            // the surface the car is standing on: accept it as the new target, never fight it
            float nt = fminf_(hm[i], ceilH());
            if (arrival && presetValid && fabsf(nt - presetH[i]) <= HCS_ANCHOR_TOL && fabsf(nt - presetH[i]) < fabsf(oldTgt - presetH[i]))
                nt = presetH[i]; // back on (near) flat ground
            k.tgt = nt;
            if (pOk[i])
            {
                k.pRef = k.airP = pm[i];
                k.airH = hm[i];
            }
            markPersist(false);
        }
        else if (arrival && presetValid && cls[i] != Cls::AMBIGUOUS && cls[i] != Cls::BOTTOM_GUARD && cls[i] != Cls::CEILING &&
                 fabsf(k.tgt - presetH[i]) <= HCS_ANCHOR_TOL && k.tgt != presetH[i])
        {
            k.tgt = presetH[i]; // pull a non-terrain reference back onto the preset after a drive
            markPersist(false);
        }
        if (cls[i] == Cls::AIR_LOSS || cls[i] == Cls::LOAD || cls[i] == Cls::UNLOAD)
            goal[i] = k.tgt;
        if (cls[i] == Cls::WITHIN)
            reref[i] = pOk[i] && fabsf(hm[i] - k.tgt) <= 0.5f * HCS_DEADBAND_H;
        if (dir[i] > 0)
            goal[i] = fminf_(goal[i], ceilH());
        if (dir[i] < 0)
            goal[i] = fmaxf_(goal[i], liftTarget(in, i));
        if ((dir[i] > 0 && goal[i] <= hm[i] + 0.5f) || (dir[i] < 0 && goal[i] >= hm[i] - 0.5f))
            dir[i] = 0;
        if (cls[i] != Cls::WITHIN)
        {
            if (periodic && cls[i] == k.lastCls)
                logq("  %s %s (unchanged)", CN[i], clsName(cls[i]));
            else
                logf("  %s %s h=%.1f tgt=%.1f->%.1f e=%+.1f dp=%+.1f air=%s", CN[i], clsName(cls[i]), hm[i], oldTgt, k.tgt, hm[i] - oldTgt,
                     (pOk[i] && finite_(k.pRef)) ? pm[i] - k.pRef : 0.0f, airChg[i] < 0 ? "lost" : airChg[i] > 0 ? "gained" : "same");
        }
        k.lastCls = cls[i];
    }

    // ---- 6. event latch: while anything is pending, keep the load references (the evidence) frozen
    bool pending = false;
    for (int i = 0; i < NC; i++)
        if (dir[i] != 0 || c[i].completeCls != Cls::NONE)
            pending = true;
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
        if (reref[i] && (!eventOpen || forceReref))
            c[i].pRef = pm[i];

    // ---- 7. confirmation: the same correction wanted twice, >= HCS_CONFIRM_MS apart, no motion in between
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (dir[i] == 0)
        {
            k.wantDir = 0;
            continue;
        }
        if (k.completeCls != Cls::NONE && cls[i] == k.completeCls)
            continue; // a completion was confirmed with the original correction
        if (k.wantDir != dir[i] || k.wantEpisode != episodeId)
        {
            k.wantDir = dir[i];
            k.wantSince = now;
            k.wantEpisode = episodeId;
        }
        const uint32_t age = now - k.wantSince;
        if (age < HCS_CONFIRM_MS)
        {
            logq("  %s %s: wanted, confirming in %lus", CN[i], clsName(cls[i]), (unsigned long)((HCS_CONFIRM_MS - age) / 1000));
            const uint32_t due = now + (HCS_CONFIRM_MS - age) + 100;
            if (confirmAt == 0 || (int32_t)(due - confirmAt) < 0)
                confirmAt = due;
            dir[i] = 0;
        }
    }

    // ---- 8. arbiter: per-corner gates, then ONE batch = the axle with the largest error, same direction
    for (int i = 0; i < NC; i++)
    {
        if (dir[i] == 0)
            continue;
        const char *no = gate(in, i, dir[i], pOk[i], pm[i]);
        if (no)
        {
            logq("  REFUSE %s %s %s: %s", CN[i], clsName(cls[i]), dir[i] > 0 ? "fill" : "dump", no);
            dir[i] = 0;
        }
    }
    int best = -1;
    float bestE = 0;
    for (int i = 0; i < NC; i++)
        if (dir[i] != 0 && fabsf(goal[i] - hm[i]) > bestE + 1e-3f)
        {
            best = i;
            bestE = fabsf(goal[i] - hm[i]);
        }
    st = State::PARKED;
    if (best < 0)
        return;
    bool inB[NC];
    int watchers = 0;
    for (int i = 0; i < NC; i++)
    {
        inB[i] = dir[i] == dir[best] && isFront(i) == isFront(best);
        if (!inB[i] && hOk[i])
            watchers++;
    }
    const char *veto = !in.presence                                       ? "no BLE client (vehicle-off gate)"
                       : batchesLastHour() >= HCS_MAX_BATCHES_PER_HOUR    ? "hourly batch budget exhausted"
                       : watchers < HCS_MOTION_MIN_CORNERS                ? "fewer than 2 healthy corners left to watch for motion"
                                                                          : nullptr;
    if (veto)
    {
        char who[48];
        int w = 0;
        for (int i = 0; i < NC; i++)
            if (inB[i] && w >= 0 && w < (int)sizeof(who))
                w += snprintf(who + w, sizeof(who) - w, "%s/%s ", CN[i], clsName(cls[i]));
        logq("VETO (%s): would %s %s", veto, dir[best] > 0 ? "fill" : "dump", who);
        return;
    }
    startBatch(in, out, inB, dir, goal, cls, hm, pm, pOk);
}

} // namespace hcs

#endif
