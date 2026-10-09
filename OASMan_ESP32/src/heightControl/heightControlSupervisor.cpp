// Height Control Supervisor -- ESP32 adapter. The decision logic lives in hcs_core.cpp (platform independent,
// replayed on a PC by eval/hcs_sim.cpp); this file only moves data between it and the hardware:
//   sensors (cached by the wheel tasks) -> hcs::Inputs -> Core::tick -> START / ABORT -> Wheel goal routines
// Design: OASMan_ESP32/docs/height-control-supervisor.md

#include "heightControlSupervisor.h"

#if HEIGHT_CONTROL_SUPERVISOR

#include <Arduino.h>
#include <Preferences.h>
#include "hcs_core.h"
#include "../airSuspensionUtil.h"
#include "../manifoldSaveData.h"

static hcs::Core core;
static portMUX_TYPE hcsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool evPreset = false;
static volatile bool evManual = false;
static uint8_t evPresetH[hcs::NC];

static const char *const HCS_NVS_NS = "hcs";
static const char *const HCS_NVS_KEY = "st";

bool hcsOwnsHeightMaintain()
{
    return !HCS_SHADOW_MODE && getheightSensorMode() && getmaintainPressure();
}

void hcsNotifyPresetLoad(int profileIndex)
{
    if (profileIndex < 0 || profileIndex >= MAX_PROFILE_COUNT)
    {
        hcsNotifyManual();
        return;
    }
    ProfileRaw p = readProfile(profileIndex); // RAM-cached preferences, safe from any task
    portENTER_CRITICAL(&hcsMux);
    for (int i = 0; i < hcs::NC; i++)
    {
        evPresetH[i] = p.pressure[i]; // in height-sensor mode the profile bytes are height %
    }
    evPreset = true;
    portEXIT_CRITICAL(&hcsMux);
}

void hcsNotifyManual()
{
    evManual = true;
}

static void hcsLog(const char *line)
{
    Serial.println(line);
}

static bool loadPersist(hcs::PersistBlob &b)
{
    Preferences pr;
    if (!pr.begin(HCS_NVS_NS, true))
    {
        return false; // namespace not created yet (first boot with HCS)
    }
    bool ok = false;
    if (pr.getBytesLength(HCS_NVS_KEY) == sizeof(b))
    {
        ok = pr.getBytes(HCS_NVS_KEY, &b, sizeof(b)) == sizeof(b);
    }
    pr.end();
    return ok;
}

static void savePersist()
{
    hcs::PersistBlob b;
    if (!core.exportPersist(b))
    {
        return;
    }
    Preferences pr;
    if (!pr.begin(HCS_NVS_NS, false))
    {
        Serial.println("HCS persist: nvs open failed");
        return;
    }
    pr.putBytes(HCS_NVS_KEY, &b, sizeof(b));
    pr.end();
}

static void buildInputs(hcs::Inputs &in)
{
    memset(&in, 0, sizeof(in));
    in.now = millis();
    for (int i = 0; i < hcs::NC; i++)
    {
        Wheel *w = getWheel(i);
        hcs::CornerIn &c = in.c[i];
        c.h = w->getLevelCached();
        c.hRaw = w->getLevelRaw();
        c.p = w->getPressureCached();
        c.inOpen = w->getInSolenoid()->isOpen();
        c.outOpen = w->getOutSolenoid()->isOpen();
        c.routineActive = w->isRoutineFlagged();
        c.routineAutonomous = w->isAutonomousRoutine();
        c.seq = w->getSampleSeq();
        c.calMinRaw = getheightCalMin(i);
        c.calMaxRaw = getheightCalMax(i);
        c.minRide = getheightCalMinRide(i);
    }
    in.tank = getCompressor()->getTankPressure();
    in.tankValid = true; // range-checked inside the core
    in.compressorOn = getCompressor()->isOn();
    in.presence = isVehicleOn(); // BT-ignition builds: an authenticated BLE client is connected
    in.enabled = getheightSensorMode() && getmaintainPressure();
    in.safetyMode = getsafetyMode();
    float bagMax = (float)getbagMaxPressure();
    in.bagCeilPsi = bagMax < (float)MAX_PRESSURE_SAFETY ? bagMax : (float)MAX_PRESSURE_SAFETY;
    in.compressorOffPsi = (float)getcompressorOffPSI();
}

static void applyOutputs(const hcs::Outputs &out)
{
    if (out.cmd == hcs::Cmd::START)
    {
        for (int i = 0; i < hcs::NC; i++)
        {
            const hcs::GoalRequest &g = out.g[i];
            if (!g.active)
            {
                continue;
            }
            bool ok = getWheel(i)->initAutonomousGoal(g.target, g.dir, g.ceilH, g.floorH, g.ceilP);
            if (!ok)
            {
                Serial.printf("HCS wheel %d refused goal %d\n", i, g.target);
            }
        }
    }
    else if (out.cmd == hcs::Cmd::ABORT)
    {
        for (int i = 0; i < hcs::NC; i++)
        {
            if (out.g[i].active)
            {
                getWheel(i)->requestAutonomousAbort();
            }
        }
    }
}

#if HCS_DIAG_DUMP
static void diagDump()
{
    static char buf[720];
    core.dump(buf, sizeof(buf));
    Serial.println(buf);
    // Raw electrical view (what the reverted manifold_v4_diag used to print): pre-calibration, UNclamped.
    // >100 or <0 = voltage outside 0.5-4.5 V at the ADC pin (floating input / lost ground) -- electrical.
    static const char *const cn[4] = {"FP", "RP", "FD", "RD"};
    int w = snprintf(buf, sizeof(buf), "HRAW");
    for (int i = 0; i < 4; i++)
    {
        float raw = getWheel(i)->getLevelRaw();
        w += snprintf(buf + w, sizeof(buf) - w, " %s=%.1f(%.2fV)", cn[i], raw, 0.5f + raw * 0.04f);
    }
    w += snprintf(buf + w, sizeof(buf) - w, " CAL");
    for (int i = 0; i < 4; i++)
    {
        w += snprintf(buf + w, sizeof(buf) - w, " %s[%.1f..%.1f mr%.1f]", cn[i], getheightCalMin(i), getheightCalMax(i), getheightCalMinRide(i));
    }
    snprintf(buf + w, sizeof(buf) - w, " HM=%d MP=%d BLE=%d comp=%d", getheightSensorMode() ? 1 : 0, getmaintainPressure() ? 1 : 0,
             isVehicleOn() ? 1 : 0, getCompressor()->isOn() ? 1 : 0);
    Serial.println(buf);
}
#endif

static void task_hcs(void *)
{
    hcs::PersistBlob blob;
    bool have = loadPersist(blob);
    core.begin(millis(), have ? &blob : nullptr, HCS_SHADOW_MODE, hcsLog);
    Serial.printf("HCS supervisor started (%s%s)\n", HCS_SHADOW_MODE ? "SHADOW: log only, legacy maintain active" : "ACTIVE",
                  HCS_DIAG_DUMP ? ", diag dump" : "");
#if HCS_DIAG_DUMP
    uint32_t lastDump = 0;
#endif
    for (;;)
    {
        bool p, m;
        uint8_t ph[hcs::NC];
        portENTER_CRITICAL(&hcsMux);
        p = evPreset;
        m = evManual;
        memcpy(ph, evPresetH, sizeof(ph));
        evPreset = false;
        evManual = false;
        portEXIT_CRITICAL(&hcsMux);
        if (p)
        {
            core.notifyPresetLoad(ph);
        }
        else if (m)
        {
            core.notifyManual();
        }

        hcs::Inputs in;
        buildInputs(in);
        hcs::Outputs out;
        core.tick(in, out);
        applyOutputs(out);
        if (out.persistNow)
        {
            savePersist();
        }
#if HCS_DIAG_DUMP
        if (millis() - lastDump >= HCS_DUMP_PERIOD_MS)
        {
            lastDump = millis();
            diagDump();
        }
#endif
        delay(HCS_TICK_MS);
    }
}

void hcsSetup()
{
    xTaskCreate(task_hcs, "HCS", 512 * 10, NULL, 1000, NULL);
}

#endif
