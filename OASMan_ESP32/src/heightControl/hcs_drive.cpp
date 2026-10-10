// HCS while driving: road level and fill pulses. See hcs_config.h HCS_ROAD_* / HCS_PULSE_* / HCS_CALM_*.
//
// The parked path never acts while moving. This one is deliberately weaker: open-loop, fill-only, short pulses, only
// in calm driving. It covers load added just before driving off (nobody waited for the parked correction), a leak
// on a long trip, and an urgent lift cut short by driving away.
//
// The road is level on average, so a 2-minute average of each corner, twist removed, says where the car really sits.
// Cornering, braking, hills and dips tilt or bounce it, but cannot LOWER it on average: the total load is constant.
// So the window must show the car low overall (mean deficit > HCS_ROAD_HEAVE_MIN) before the corners carrying that
// deficit are topped up -- by at most what that window says each is missing.

#include "hcs_core.h"

#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>

namespace hcs
{

// Body roll / pitch of four per-corner deviations (sums of two corners: more sensitive than one corner alone).
static bool tilted(const float *d, float lim)
{
    const float roll = 0.5f * ((d[C_FP] + d[C_RP]) - (d[C_FD] + d[C_RD])), pitch = 0.5f * ((d[C_FP] + d[C_FD]) - (d[C_RP] + d[C_RD]));
    return fabsf(roll) > lim || fabsf(pitch) > lim;
}

void Core::driveTick(const Inputs &in, Outputs &out)
{
    // ---- 1-s averages (calm, pulse gates); a corner moving air makes the road window skip this sample
    bool quiet = true;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (k.hFault)
            continue;
        if (k.busy || (now - k.idleSince) < HCS_P_SETTLE_MS)
        {
            quiet = false;
            continue;
        }
        const float h = in.c[i].h, p = in.c[i].p;
        if (!k.hsOk)
            k.hS = h, k.pS = p, k.calmLo = k.calmHi = h, k.hsOk = true;
        k.hS += 0.1f * (h - k.hS);
        k.pS += 0.1f * (p - k.pS);
    }
    // calm: every corner's 1-s average inside HCS_CALM_H since calmSince (turning in / out, braking, bumps move it; a
    // long steady curve does not -- a pulse there adds nearly the same air)
    bool calm = true;
    float jolt[NC];
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        jolt[i] = 0;
        if (k.hFault || !k.hsOk)
            continue;
        jolt[i] = i == pulseCorner ? 0 : in.c[i].h - k.hS; // the raw reading leaving the 1-s average: a curve / brake starting
        if (k.busy)
            continue; // the pulsed corner moves on purpose
        k.calmLo = fminf_(k.calmLo, k.hS);
        k.calmHi = fmaxf_(k.calmHi, k.hS);
        calm = calm && (k.calmHi - k.calmLo) <= HCS_CALM_H;
    }
    if (!calm)
    {
        calmSince = now;
        for (int i = 0; i < NC; i++)
            c[i].calmLo = c[i].calmHi = c[i].hS;
    }
    if (driving && quiet)
    {
        for (int i = 0; i < NC; i++)
            c[i].sdH += in.c[i].h;
        sdN++;
        float act = 0; // how hard the suspension works (bumps), in multiples of the motion threshold
        for (int i = 0; i < NC; i++)
            act += 0.25f * c[i].actH / HCS_MOTION_H_THRESH;
        sdAct += act;
    }

    // ---- a pulse in flight: end it on time, or the moment anything is not right
    if (pulseCorner >= 0)
    {
        Corner &k = c[pulseCorner];
        const char *why = !in.presence                          ? "BLE presence lost"
                          : (!in.enabled || in.safetyMode)       ? "supervisor disabled"
                          : (!calm || tilted(jolt, HCS_CALM_H)) ? "car moving (not calm)"
                          : (k.hFault || k.pFault)               ? "sensor fault"
                                                                 : nullptr;
        if (why || (int32_t)(now - pulseEnd) >= 0)
        {
            k.pulsedMs += now - pulseStartAt;
            out.cmd = Cmd::ABORT;
            out.g[pulseCorner].active = true;
            if (why)
            {
                logf("PULSE %s cut short: %s", CN[pulseCorner], why);
                if ((now - pulseStartAt) < pulseMs)
                    k.owe += pulseStep * (1.0f - (float)(now - pulseStartAt) / (float)pulseMs); // not delivered
            }
            pulseCorner = -1;
            sdN = 0; // the car changed on purpose: restart the road window
            sdAct = 0;
            for (int i = 0; i < NC; i++)
                c[i].sdH = 0;
            roadDefValid = false;
        }
        return;
    }
    if (!driving)
        return;
    if (sdN * (uint32_t)HCS_TICK_MS >= HCS_ROAD_WINDOW_MS)
        roadWindow();

    // ---- deliver what is owed: one corner, at most HCS_PULSE_STEP per pulse
    int best = -1;
    for (int i = 0; i < NC; i++)
        if (c[i].owe > 0.5f && (best < 0 || c[i].owe > c[best].owe))
            best = i;
    if (best < 0 || (now - pulseAt) < HCS_PULSE_GAP_MS || (now - calmSince) < HCS_CALM_MS)
        return;
    pulseAt = now;
    Corner &k = c[best];
    const char *no = !in.presence                                          ? "no BLE client"
                     : (!in.enabled || in.safetyMode || externalFreeze)     ? "supervisor disabled / frozen"
                     : (k.hFault || k.pFault || !k.hsOk)                    ? "sensor fault"
                     : (k.leakFault && !k.leakAllowance)                   ? "fast leak latched"
                     : (!tankValid || tank < k.pS + HCS_TANK_HEADROOM_PSI) ? "waiting for tank headroom"
                     : k.pS >= in.bagCeilPsi - HCS_BAG_P_MARGIN             ? "bag pressure ceiling"
                     : k.hS + 1.0f > ceilH()                               ? "ceiling"
                                                                           : nullptr;
    if (no)
    {
        logq("PULSE %s refused: %s", CN[best], no);
        return;
    }
    const float step = fminf_(k.owe, HCS_PULSE_STEP);
    const float rate = (finite_(k.fillRate) && k.fillRate > 0.1f) ? k.fillRate : HCS_FILL_RATE_DEFAULT;
    uint32_t ms = (uint32_t)(0.85f * step / rate * 1000.0f);
    ms = ms < 100 ? 100 : (ms > HCS_PULSE_MAX_MS ? HCS_PULSE_MAX_MS : ms);
    k.owe -= step;
    if (shadow)
    {
        logf("SHADOW would PULSE %s: %lu ms", CN[best], (unsigned long)ms);
        return;
    }
    out.cmd = Cmd::PULSE;
    out.g[best].active = true;
    out.g[best].dir = +1;
    out.g[best].pulseMs = (uint16_t)ms;
    out.g[best].ceilP = in.bagCeilPsi - HCS_BAG_P_MARGIN;
    pulseCorner = best;
    ownAir = true; // a common rise on arrival is (partly) our air, not only heat
    pulseStartAt = now;
    pulseMs = ms;
    pulseStep = step;
    pulseEnd = now + ms;
    logf("PULSE %s: fill %lu ms (~%.1f%%, rate %s), %.1f%% still owed", CN[best], (unsigned long)ms, step,
         finite_(k.fillRate) ? "learned" : "default", k.owe > 0 ? k.owe : 0.0f);
}

// One road window: where the car sat on average versus the preset plane (kept for the arrival evaluation, ROAD). If it
// sat low overall, the corners carrying it are owed their deficit. Rough windows are ignored; each window's verdict
// replaces the last; a drive adds at most HCS_OWE_MAX per corner.
void Core::roadWindow()
{
    float hmean[NC], ref[NC], d[NC];
    for (int i = 0; i < NC; i++)
    {
        hmean[i] = c[i].sdH / sdN;
        ref[i] = presetValid ? (float)presetH[i] : c[i].tgt;
        c[i].sdH = 0;
    }
    const float rough = sdAct / sdN;
    sdN = 0;
    sdAct = 0;
    if (rough > HCS_ROAD_ROUGH_MAX)
    {
        // on a rough road the average reading is not where the car sits (rebound damping packs it down, spring /
        // linkage nonlinearity shifts the mean): this window says nothing, and nothing owed from before is delivered
        for (int i = 0; i < NC; i++)
            c[i].owe = 0;
        logq("ROAD window rough (%.1f) -> ignored", rough);
        return;
    }
    roadDefValid = planeDeficit(hmean, d, ref);
    if (!roadDefValid)
        return;
    float heave = 0;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        // what the last window's pulses really did = the fill rate (open-loop pulses calibrate themselves)
        const float rise = k.roadDef - d[i];
        if (k.pulsedMs >= 500 && rise > 0.5f)
        {
            const float r = rise / (k.pulsedMs / 1000.0f);
            if (r > 0.1f && r < 50.0f)
            {
                k.fillRate = finite_(k.fillRate) ? 0.7f * k.fillRate + 0.3f * r : r;
                markPersist(false);
            }
        }
        k.pulsedMs = 0;
        k.roadDef = d[i];
        heave += 0.25f * d[i];
    }
    const bool low = heave > HCS_ROAD_HEAVE_MIN && !externalFreeze;
    logq("ROAD level vs preset (+ = low): FP %+.1f RP %+.1f FD %+.1f RD %+.1f, mean %+.1f, rough %.1f%s", d[C_FP], d[C_RP], d[C_FD],
         d[C_RD], heave, rough, low ? " -> sits low: top up" : "");
    // the corners carrying it: at least half the worst deficit (a leaking bag's neighbours show ~1/3 of its deficit
    // after the twist is removed -- refilling the leaking bag restores them), and more than HCS_ROAD_HEAVE_MIN
    float worst = 0;
    for (int i = 0; i < NC; i++)
        worst = fmaxf_(worst, d[i]);
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        k.owe = 0; // each window's verdict replaces the last (nothing stale is delivered later)
        if (!low || d[i] <= fmaxf_(HCS_ROAD_HEAVE_MIN, 0.5f * worst) || k.floorOverride >= 0 || k.hFault)
            continue;
        k.owe = fminf_(finite_(k.fillRate) ? d[i] : 0.5f * d[i], HCS_OWE_MAX - k.driveAdded); // rate unknown: half
        if (k.owe < 0.5f)
        {
            k.owe = 0;
            logq("ROAD %s: %.0f %% already added this drive (cap) -> no more", CN[i], (double)k.driveAdded);
            continue;
        }
        k.driveAdded += k.owe; // hard cap per drive, whatever the readings say
        if (!shadow)
            recordRefill(i, k.owe); // a burst bag latches like a parked one
    }
}

} // namespace hcs

#endif
