// HCS sensing: sample intake, electrical sensor faults, motion detector. See hcs_core.h.

#include "hcs_core.h"

#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace hcs
{

// Electrical faults only: a corner whose height sensor is out of its 0.5-4.5 V band (wire break / short), outside
// its calibrated travel by a wide margin, or uncalibrated is FROZEN (never actuated). A bag pressure sensor out of
// range blocks fills on that corner. Latched after HCS_FAULT_LATCH_SAMPLES bad samples, cleared after
// HCS_FAULT_CLEAR_MS of good ones. Whether the sensors are wired to the right corners is the installer's job.
void Core::detectFaults(const Inputs &in, int i)
{
    const CornerIn &x = in.c[i];
    Corner &k = c[i];
    if (in.enabled)
    {
        const float span = fabsf(x.calMaxRaw - x.calMinRaw);
        const bool calBad = !(span >= HCS_CAL_MIN_SPAN);
        const float lo = fminf_(x.calMinRaw, x.calMaxRaw) - 15.0f, hi = fmaxf_(x.calMinRaw, x.calMaxRaw) + 15.0f;
        const bool bad = calBad || !(x.hRaw >= HCS_RAW_H_MIN && x.hRaw <= HCS_RAW_H_MAX) || !(x.hRaw >= lo && x.hRaw <= hi);
        if (bad)
        {
            k.goodSinceH = now;
            if (++k.badH >= HCS_FAULT_LATCH_SAMPLES && !k.hFault)
            {
                k.hFault = true;
                logf("FAULT %s height sensor raw=%.1f%% (%.2fV) cal=[%.1f..%.1f] -> corner frozen", CN[i], x.hRaw,
                     0.5f + x.hRaw * 0.04f, x.calMinRaw, x.calMaxRaw);
            }
        }
        else
        {
            k.badH = 0;
            if (k.hFault && (now - k.goodSinceH) >= HCS_FAULT_CLEAR_MS)
            {
                k.hFault = false;
                logf("FAULT %s height sensor good for %lus -> cleared", CN[i], (unsigned long)(HCS_FAULT_CLEAR_MS / 1000));
            }
        }
    }
    if (!(x.p >= HCS_P_MIN_VALID && x.p <= HCS_P_MAX_VALID))
    {
        k.goodSinceP = now;
        if (++k.badP >= HCS_FAULT_LATCH_SAMPLES && !k.pFault)
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
    tankValid = in.tank >= HCS_P_MIN_VALID && in.tank <= HCS_P_MAX_VALID; // invalid tank -> no fills (gate())
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        const CornerIn &x = in.c[i];
        const bool vo = x.inOpen || x.outOpen;
        const bool busy = vo || x.routineActive;
        if (busy)
        {
            // valves moving air on this corner: readings are flow-affected, restart its windows
            k.busy = true;
            k.idleSince = now;
            k.qOk = false;
            k.nh = k.ih = k.np = k.ip = k.ndh = k.ndp = 0;
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
            continue; // no fresh reading since the last tick
        k.lastSeq = x.seq;
        detectFaults(in, i);
        if (busy)
            continue;
        // steadiness: held inside HCS_STABLE_RANGE_H since qStart. Restarts after a movement / our own valve activity
        // and whenever a reading leaves the band, so "steady for N s" counts from the moment the corner stopped moving
        // (a slow leak's drift just restarts it)
        if (!k.qOk || k.qSince != lastMotion || fmaxf_(k.qHi, x.h) - fminf_(k.qLo, x.h) > HCS_STABLE_RANGE_H)
        {
            k.qLo = k.qHi = x.h;
            k.qSince = lastMotion;
            k.qStart = now;
            k.qOk = true;
        }
        else
        {
            k.qLo = fminf_(k.qLo, x.h);
            k.qHi = fmaxf_(k.qHi, x.h);
        }

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
            k.actH += HCS_ACT_ALPHA * (fabsf(k.dh[2] - 2.0f * k.dh[1] + k.dh[0]) - k.actH);

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
                k.actP += HCS_ACT_ALPHA * (fabsf(k.dp[2] - 2.0f * k.dp[1] + k.dp[0]) - k.actP);
        }
    }
}

// Settled reading = mean of the last HCS_WINDOW valve-closed samples; "stable" = small spread inside the window.
bool Core::windowMean(int i, float &hm, float &pm, bool &pOk, bool &stable) const
{
    const Corner &k = c[i];
    pOk = stable = false;
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
    bool ok = (hi - lo) <= HCS_STABLE_RANGE_H;
    if (!k.pFault && k.np >= HCS_WINDOW)
    {
        s = 0, lo = 1e9f, hi = -1e9f;
        for (int j = 0; j < HCS_WINDOW; j++)
        {
            s += k.wp[j];
            lo = fminf_(lo, k.wp[j]);
            hi = fmaxf_(hi, k.wp[j]);
        }
        pm = s / HCS_WINDOW;
        pOk = true;
        ok = ok && (hi - lo) <= HCS_STABLE_RANGE_P;
    }
    stable = ok;
    return true;
}

// No speed signal exists, so motion is inferred: a corner "votes" when its height or pressure activity is above
// threshold; >= HCS_MOTION_MIN_CORNERS votes for HCS_MOTION_START_TICKS start an episode; HCS_EPISODE_GAP_MS of
// quiet ends it; an episode longer than HCS_DRIVE_CONFIRM_MS is DRIVING. Corners whose valves are moving air are
// excluded; if fewer than 2 corners can be watched the car is treated as "not quiet" (fail-safe).
void Core::motionDetect()
{
    int included = 0, voting = 0, strong = 0;
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        if (k.hFault || k.busy || (now - k.idleSince) < HCS_P_SETTLE_MS || k.ndh < 3)
            continue;
        included++;
        const bool pv = pUsable(i);
        if (k.actH > HCS_MOTION_H_THRESH || (pv && k.actP > HCS_MOTION_P_THRESH))
            voting++;
        if (k.actH > 1.5f * HCS_MOTION_H_THRESH || (pv && k.actP > 1.5f * HCS_MOTION_P_THRESH))
            strong++;
    }
    lastIncluded = included;
    lastVoting = voting;
    lastStrong = strong;

    if (included < HCS_MOTION_MIN_CORNERS)
    {
        lastMotion = now; // cannot observe: not "quiet"
        return;
    }
    bool motion = voting >= HCS_MOTION_MIN_CORNERS;
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
            driving = armedAfterDrive = true;
            afterDisturb = false;
            logf("DRIVING confirmed (motion for %lus) -> parked corrections vetoed; re-baseline on arrival",
                 (unsigned long)((now - episodeStart) / 1000));
            roadDefValid = false; // a new drive decides afresh
            sdN = 0;
            sdAct = 0;
            for (int i = 0; i < NC; i++)
                c[i].roadPending = false, c[i].sdH = c[i].owe = c[i].driveAdded = 0, c[i].pulsedMs = 0;
            if (externalFreeze)
            {
                externalFreeze = false; // a car on a jack is not driving (the unloaded wheel was uneven ground)
                logf("EXTERNAL cleared: driving");
            }
        }
    }
    else if (inEpisode && (now - lastMotion) >= HCS_EPISODE_GAP_MS)
    {
        inEpisode = false;
        logf("MOTION end ep=%lu after %lus (%s)", (unsigned long)episodeId, (unsigned long)((lastMotion - episodeStart) / 1000),
             driving ? "drive" : "disturbance");
        afterDisturb = !driving;
        driving = false;
        needEval = true;
    }
}

// Has every healthy corner held inside HCS_STABLE_RANGE_H, with no movement / own valve activity, for confirmMs?
// Returns the ms still needed (0 = yes), 0xFFFFFFFF when a corner has no reading since it last moved.
uint32_t Core::steadyRemaining(uint32_t confirmMs) const
{
    uint32_t need = 0;
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        if (k.hFault)
            continue;
        if (!k.qOk)
            return 0xFFFFFFFFUL;
        const uint32_t age = now - k.qStart;
        if (age < confirmMs && confirmMs - age > need)
            need = confirmMs - age;
    }
    return need;
}

} // namespace hcs

#endif
