// HCS cruise top-up: slow-leak compensation while DRIVING (long road trips). See hcs_config.h HCS_CRUISE_*.
//
// The parked path never acts while moving, so on a long trip a slowly leaking bag would sink. This path is
// deliberately weaker than the parked one: open-loop, fill-only, one corner, at most HCS_CRUISE_STEP per pulse,
// decided on 2-minute averages and only during steady driving.
//
// Leak signature on a rigid body: the four suspension lengths stay roughly coplanar on a road, so one bag losing
// air tilts the whole body toward it and puts a LOAD WARP on the car (that bag and its diagonal partner unload, the
// other diagonal loads up). Cornering / braking (moments), people (point loads), camber (planar) and dips / hills
// (all four together) produce no warp, and road-surface warp averages out over 2 minutes. Which bag of the unloaded
// diagonal is leaking: versus the trip reference (the averages at minute 10) it went DOWN without its pressure
// rising -- impossible for a bag whose air is unchanged.

#include "hcs_core.h"

#if HEIGHT_CONTROL_SUPERVISOR || defined(HCS_HOST_BUILD)

#include <math.h>

namespace hcs
{

void Core::cruiseTick(const Inputs &in, Outputs &out)
{
    // ---- 2-minute (leak) and ~1-second (roll / pitch excursion) averages
    const float aL = (float)HCS_TICK_MS / (float)HCS_CRUISE_TAU_MS, aS = 0.1f;
    bool allInit = true;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        if (k.hFault || k.pFault)
        {
            allInit = false;
            continue;
        }
        if (k.busy || (now - k.idleSince) < HCS_P_SETTLE_MS)
        {
            allInit = allInit && k.cruiseInit;
            continue; // our own pulse: flow-affected readings
        }
        if (!k.cruiseInit)
        {
            k.hL = k.hS = in.c[i].h;
            k.pL = in.c[i].p;
            k.cruiseInit = true;
        }
        k.hL += aL * (in.c[i].h - k.hL);
        k.pL += aL * (in.c[i].p - k.pL);
        k.hS += aS * (in.c[i].h - k.hS);
        if (driving)
        {
            k.sdH += in.c[i].h;
            k.sdP += in.c[i].p;
        }
    }
    if (driving && allInit)
        sdN++;
    driveLoadCheck(in);
    bool st_ = false;
    if (allInit)
    {
        float d[NC];
        for (int i = 0; i < NC; i++)
            d[i] = c[i].hS - c[i].hL;
        const float roll = 0.5f * ((d[C_FP] + d[C_RP]) - (d[C_FD] + d[C_RD]));
        const float pitch = 0.5f * ((d[C_FP] + d[C_FD]) - (d[C_RP] + d[C_RD]));
        st_ = fabsf(roll) < HCS_CRUISE_STEADY_DEV && fabsf(pitch) < HCS_CRUISE_STEADY_DEV;
    }
    if (st_ && !steady)
        steadySince = now;
    steady = st_;
    bool calmOk = allInit;
    for (int i = 0; i < NC && calmOk; i++)
    {
        Corner &k = c[i];
        if (k.hFault || k.busy)
            continue; // the pulsed corner moves on purpose
        k.calmLo = fminf_(k.calmLo, k.hS);
        k.calmHi = fmaxf_(k.calmHi, k.hS);
        calmOk = (k.calmHi - k.calmLo) <= HCS_OWE_CALM_H;
    }
    if (!calmOk)
    {
        calmSince = now;
        for (int i = 0; i < NC; i++)
            c[i].calmLo = c[i].calmHi = c[i].hS;
    }

    // ---- a pulse in flight: end it on time, or the moment anything is not right
    if (pulseCorner >= 0)
    {
        const char *why = !in.presence                                  ? "BLE presence lost"
                          : (!in.enabled || in.safetyMode)             ? "supervisor disabled"
                          : (pulseOwe ? !calmOk : !steady)              ? "roll / pitch excursion"
                          : (c[pulseCorner].hFault || c[pulseCorner].pFault) ? "sensor fault"
                                                                        : nullptr;
        if (why || (int32_t)(now - pulseEnd) >= 0)
        {
            out.cmd = Cmd::ABORT;
            out.g[pulseCorner].active = true;
            if (why)
            {
                logf("%s pulse %s cut short: %s", pulseOwe ? "DRIVE-AWAY" : "CRUISE", CN[pulseCorner], why);
                if (pulseOwe && pulseMs > 0 && (now - pulseStartAt) < pulseMs)
                    c[pulseCorner].owe += pulseStep * (1.0f - (float)(now - pulseStartAt) / (float)pulseMs); // not delivered
            }
            pulseCorner = -1;
            sdN = 0; // the car changed on purpose: restart the drive-long averages
            for (int i = 0; i < NC; i++)
                c[i].sdH = c[i].sdP = 0;
        }
        return;
    }

    if (oweTick(in, out))
        return;
    if (!HCS_CRUISE_TOPUP || !driving || (now - driveStartAt) < HCS_CRUISE_MIN_DRIVE_MS || (now - cruiseEvalAt) < 10000UL)
        return;
    cruiseEvalAt = now;
    for (int i = 0; i < NC; i++)
        if (!c[i].cruiseRef && c[i].cruiseInit)
        {
            c[i].hC = c[i].hL; // trip reference
            c[i].pC = c[i].pL;
            c[i].cruiseRef = true;
        }
    if (!presetValid || !allInit || externalFreeze || heightFaultCount() > 0 || !in.enabled || in.safetyMode)
        return;

    // ---- leak evidence (10-s evaluations, leaky counter, hysteresis)
    static const int DIAG[NC] = {C_RD, C_FD, C_RP, C_FP};
    float fr[NC];
    for (int i = 0; i < NC; i++)
        fr[i] = c[i].pL / fmaxf_(finite_(c[i].pScale) ? c[i].pScale : 100.0f, HCS_PREF_MIN_PSI) - 1.0f;
    const float warp = 0.25f * (fr[C_FP] + fr[C_RD] - fr[C_FD] - fr[C_RP]);
    const int need = (int)(HCS_CRUISE_PERSIST_MS / 10000UL);
    int best = -1;
    float bestScore = 0;
    for (int i = 0; i < NC; i++)
    {
        Corner &k = c[i];
        const float hy = k.cruiseLow ? 0.5f : 1.0f; // enter at the full threshold, stay above half
        const bool pairUnloaded = (i == C_FP || i == C_RD) ? warp < -hy * HCS_CRUISE_WARP : warp > hy * HCS_CRUISE_WARP;
        const float dH = k.hL - k.hC, dP = k.pL - k.pC, dHd = c[DIAG[i]].hL - c[DIAG[i]].hC;
        const bool low = k.cruiseRef && pairUnloaded && dH < -hy * HCS_CRUISE_SIGN_DH && dP <= 0.5f + (1 - hy) &&
                         dHd > dH + hy * HCS_CRUISE_SIGN_DH && k.floorOverride < 0 && presetH[i] >= floorBase(in, i);
        k.cruiseCnt = low ? (k.cruiseCnt < 2 * need ? k.cruiseCnt + 1 : k.cruiseCnt) : (k.cruiseCnt > 0 ? k.cruiseCnt - 1 : 0);
        if (low && !k.cruiseLow)
        {
            k.cruiseLow = true;
            k.cruiseLowSince = now;
            if (k.cruiseCnt == 1)
                logf("CRUISE %s looks like it is losing air (warp %+.1f%%, trip dh %+.1f dp %+.1f) -> watching", CN[i], warp * 100.0f, dH, dP);
        }
        else if (k.cruiseCnt == 0 && k.cruiseLow)
            k.cruiseLow = false;
        if (k.cruiseLow && k.cruiseCnt >= need && -dH > bestScore)
        {
            best = i;
            bestScore = -dH;
        }
    }
    if (best < 0)
        return;

    // ---- gates
    Corner &k = c[best];
    int nh = 0;
    for (int j = 0; j < HCS_CRUISE_MAX_PER_HOUR; j++)
        if ((now - pulseT[j]) < HOUR_MS)
            nh++;
    const char *no = !in.presence                                             ? "no BLE client"
                     : (!steady || (now - steadySince) < HCS_CRUISE_STEADY_MS) ? "not steady long enough"
                     : (now - k.lastPulseAt) < HCS_CRUISE_DWELL_MS             ? "dwell"
                     : nh >= HCS_CRUISE_MAX_PER_HOUR                          ? "hourly cruise budget"
                     : (k.leakFault && !k.leakAllowance)                      ? "fast leak latched"
                     : (!tankValid || tank < k.pL + HCS_TANK_HEADROOM_PSI)    ? "waiting for tank headroom"
                     : k.pL >= in.bagCeilPsi - HCS_BAG_P_MARGIN               ? "bag pressure ceiling"
                     : k.hL + HCS_CRUISE_STEP > ceilH()                       ? "ceiling"
                                                                               : nullptr;
    if (no)
    {
        logq("CRUISE %s top-up refused: %s", CN[best], no);
        return;
    }
    uint32_t ms = (finite_(k.fillRate) && k.fillRate > 0.1f) ? (uint32_t)(0.7f * HCS_CRUISE_STEP / k.fillRate * 1000.0f)
                                                            : (uint32_t)HCS_CRUISE_PULSE_DEFAULT_MS;
    ms = ms < 100 ? 100 : (ms > HCS_CRUISE_PULSE_MAX_MS ? HCS_CRUISE_PULSE_MAX_MS : ms);
    k.lastPulseAt = now;
    k.cruiseLow = false;
    k.cruiseCnt = 0;
    pulseT[pulseI] = now;
    pulseI = (pulseI + 1) % HCS_CRUISE_MAX_PER_HOUR;
    if (shadow)
    {
        logf("SHADOW would CRUISE top-up %s: %lu ms", CN[best], (unsigned long)ms);
        return;
    }
    recordRefill(best, HCS_CRUISE_STEP);
    out.cmd = Cmd::PULSE;
    out.g[best].active = true;
    out.g[best].dir = +1;
    out.g[best].pulseMs = (uint16_t)ms;
    out.g[best].ceilP = in.bagCeilPsi - HCS_BAG_P_MARGIN;
    pulseCorner = best;
    pulseOwe = false;
    pulseEnd = now + ms;
    logf("CRUISE top-up %s: losing air for %lus, steady %lus -> fill pulse %lu ms (~%.1f%%, rate %s)", CN[best],
         (unsigned long)((now - k.cruiseLowSince) / 1000), (unsigned long)((now - steadySince) / 1000), (unsigned long)ms,
         HCS_CRUISE_STEP, finite_(k.fillRate) ? "learned" : "default");
}

// Load added right before driving off: decide it from the drive-long means (see HCS_DRIVE_LOAD_*), owe the plane
// deficit to the low corners, deliver it with oweTick. Once per drive.
void Core::driveLoadCheck(const Inputs &in)
{
    if (!driving || driveLoadDone || (now - driveStartAt) < HCS_DRIVE_LOAD_MIN_MS || (now - driveLoadAt) < 10000UL)
        return;
    driveLoadAt = now;
    if ((now - driveStartAt) > HCS_DRIVE_LOAD_MAX_MS)
    {
        driveLoadDone = true;
        return;
    }
    if (sdN < HCS_DRIVE_LOAD_MIN_MS / HCS_TICK_MS || heightFaultCount() > 0 || externalFreeze || !in.enabled)
        return; // (sdN restarts after every pulse)
    static const float TW[NC] = {+1, -1, -1, +1};
    float hmean[NC], ref[NC], load = 0, wRef = 0, wCur = 0;
    for (int i = 0; i < NC; i++)
    {
        const Corner &k = c[i];
        if (k.pFault || k.owe > 0.5f || !k.tgtValid || !finite_(k.pRef))
            return; // owed already (parked decision) or no load reference
        hmean[i] = k.sdH / sdN;
        load += 0.25f * ((k.sdP / sdN) - k.pRef) / fmaxf_(finite_(k.pScale) ? k.pScale : k.pRef, HCS_PREF_MIN_PSI);
        ref[i] = presetValid ? (float)presetH[i] : k.tgt;
        wRef += 0.25f * TW[i] * ref[i];
        wCur += 0.25f * TW[i] * hmean[i];
    }
    float d[NC], worst = 0;
    for (int i = 0; i < NC; i++)
    {
        d[i] = (ref[i] - TW[i] * wRef) - (hmean[i] - TW[i] * wCur); // > 0: the level plane is low at this corner
        worst = fmaxf_(worst, d[i]);
    }
    const bool low = load > HCS_LOAD_FRAC && worst > HCS_DEADBAND_H;
    driveLoadCnt = low ? driveLoadCnt + 1 : 0;
    if (driveLoadCnt < 3)
        return; // must hold for 3 checks (30 s)
    driveLoadDone = true;
    for (int i = 0; i < NC; i++)
        if (d[i] > HCS_LAND_TOL && c[i].floorOverride < 0)
            c[i].owe = fminf_(d[i], HCS_OWE_MAX);
    logf("DRIVE LOAD: load %+.1f%% since parking, level plane low by FP %.1f RP %.1f FD %.1f RD %.1f -> top up in steady driving",
         load * 100.0f, d[C_FP], d[C_RP], d[C_FD], d[C_RD]);
}

// Drive-away top-up: deliver what a parked evaluation decided (load / leak, on steady readings) but could not finish
// before the car drove off. Fill-only, open-loop, never more than owed; the parked path checks the result on arrival.
// Returns true when it handled this tick (pulse started or refused), so the leak logic waits.
bool Core::oweTick(const Inputs &in, Outputs &out)
{
    if (!driving || (now - oweAt) < HCS_OWE_PULSE_GAP_MS)
        return false;
    int best = -1;
    for (int i = 0; i < NC; i++)
        if (c[i].owe > 0.5f && (best < 0 || c[i].owe > c[best].owe))
            best = i;
    if (best < 0)
        return false;
    oweAt = now;
    Corner &k = c[best];
    const float p = k.cruiseInit ? k.pL : in.c[best].p;
    const char *no = !in.presence                                             ? "no BLE client"
                     : (!in.enabled || in.safetyMode || externalFreeze)        ? "supervisor disabled / frozen"
                     : (k.hFault || k.pFault)                                  ? "sensor fault"
                     : (now - calmSince) < HCS_OWE_CALM_MS                     ? nullptr
                     : (k.leakFault && !k.leakAllowance)                      ? "fast leak latched"
                     : (!tankValid || tank < p + HCS_TANK_HEADROOM_PSI)       ? "waiting for tank headroom"
                     : p >= in.bagCeilPsi - HCS_BAG_P_MARGIN                   ? "bag pressure ceiling"
                     : k.hS + 1.0f > ceilH()                                  ? "ceiling"
                                                                               : "";
    if (!no)
        return true; // not steady yet: wait quietly
    if (*no)
    {
        logq("DRIVE-AWAY %s top-up refused: %s", CN[best], no);
        return true;
    }
    const float step = fminf_(k.owe, HCS_CRUISE_STEP);
    const float rate = (finite_(k.fillRate) && k.fillRate > 0.1f) ? k.fillRate : HCS_FILL_RATE_DEFAULT;
    uint32_t ms = (uint32_t)(0.85f * step / rate * 1000.0f);
    ms = ms < 100 ? 100 : (ms > HCS_CRUISE_PULSE_MAX_MS ? HCS_CRUISE_PULSE_MAX_MS : ms);
    k.owe -= step;
    if (shadow)
    {
        logf("SHADOW would DRIVE-AWAY top-up %s: %lu ms", CN[best], (unsigned long)ms);
        return true;
    }
    out.cmd = Cmd::PULSE;
    out.g[best].active = true;
    out.g[best].dir = +1;
    out.g[best].pulseMs = (uint16_t)ms;
    out.g[best].ceilP = in.bagCeilPsi - HCS_BAG_P_MARGIN;
    pulseCorner = best;
    pulseOwe = true;
    pulseStartAt = now;
    pulseMs = ms;
    pulseStep = step;
    pulseEnd = now + ms;
    logf("DRIVE-AWAY top-up %s: fill pulse %lu ms (~%.1f%%, rate %s), %.1f%% still owed", CN[best], (unsigned long)ms, step,
         finite_(k.fillRate) ? "learned" : "default", k.owe > 0 ? k.owe : 0.0f);
    return true;
}

} // namespace hcs

#endif
