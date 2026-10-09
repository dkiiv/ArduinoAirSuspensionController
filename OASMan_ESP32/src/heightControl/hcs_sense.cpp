// HCS sensing: sample intake, electrical sensor faults, motion detector, STATS. See hcs_core.h.

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
void Core::motionDetect(const Inputs &in, uint32_t dt)
{
    (void)dt;
    int included = 0, voting = 0, strong = 0;
    bool inc[NC] = {false, false, false, false};
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        if (k.hFault || k.busy || (now - k.idleSince) < HCS_P_SETTLE_MS || k.ndh < 3)
            continue;
        inc[i] = true;
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
    statsSample(in, inc);
    if ((now - statsAt) >= HCS_STATS_PERIOD_MS)
    {
        statsAt = now;
        statsFlush();
    }

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
            driving = true;
            armedAfterDrive = true;
            driveStartAt = episodeStart;
            for (int i = 0; i < NC; i++)
            {
                c[i].cruiseInit = c[i].cruiseLow = c[i].cruiseRef = false;
                c[i].cruiseCnt = 0;
            }
            logf("DRIVING confirmed (motion for %lus) -> parked corrections vetoed; re-baseline on arrival",
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
}

// STATS: per corner, a histogram of the motion detector's activity in multiples of its threshold
// [<.25 | .25-.5 | .5-1 | 1-2 | 2-4 | >4] for QUIET (parked, no episode) and DRIVE, plus the parked standard deviation
// of height and pressure. This is the data needed to set HCS_MOTION_*_THRESH for a real car.
static int statBucket(float r) { return r < 0.25f ? 0 : r < 0.5f ? 1 : r < 1.0f ? 2 : r < 2.0f ? 3 : r < 4.0f ? 4 : 5; }

void Core::statsSample(const Inputs &in, const bool *inc)
{
    const int ctx = driving ? 1 : (!inEpisode ? 0 : -1); // disturbances are neither
    if (ctx < 0)
        return;
    for (int i = 0; i < NC; i++)
    {
        if (!inc[i])
            continue;
        uint16_t &bh = stH[ctx][i][statBucket(c[i].actH / HCS_MOTION_H_THRESH)];
        if (bh < 65535)
            bh++;
        if (pUsable(i))
        {
            uint16_t &bp = stP[ctx][i][statBucket(c[i].actP / HCS_MOTION_P_THRESH)];
            if (bp < 65535)
                bp++;
        }
        if (ctx == 0)
        {
            sqN[i] += 1;
            sqH[i] += in.c[i].h;
            sqH2[i] += (double)in.c[i].h * in.c[i].h;
            sqP[i] += in.c[i].p;
            sqP2[i] += (double)in.c[i].p * in.c[i].p;
        }
    }
}

void Core::statsFlush()
{
    static const char *const CTX[2] = {"QUIET", "DRIVE"};
    for (int ctx = 0; ctx < 2; ctx++)
        for (int i = 0; i < NC; i++)
        {
            const uint16_t *a = stH[ctx][i], *b = stP[ctx][i];
            if (a[0] + a[1] + a[2] + a[3] + a[4] + a[5] == 0)
                continue;
            char sd[48] = "";
            if (ctx == 0 && sqN[i] > 10)
            {
                double mh = sqH[i] / sqN[i], mp = sqP[i] / sqN[i];
                double vh = sqH2[i] / sqN[i] - mh * mh, vp = sqP2[i] / sqN[i] - mp * mp;
                snprintf(sd, sizeof(sd), " sdH=%.3f sdP=%.2f", sqrt(vh > 0 ? vh : 0), sqrt(vp > 0 ? vp : 0));
            }
            logf("STATS %s %s H/th[%u %u %u %u %u %u] P/th[%u %u %u %u %u %u]%s", CTX[ctx], CN[i], a[0], a[1], a[2], a[3], a[4], a[5],
                 b[0], b[1], b[2], b[3], b[4], b[5], sd);
        }
    memset(stH, 0, sizeof(stH));
    memset(stP, 0, sizeof(stP));
    for (int i = 0; i < NC; i++)
        sqN[i] = sqH[i] = sqH2[i] = sqP[i] = sqP2[i] = 0;
}

} // namespace hcs

#endif
