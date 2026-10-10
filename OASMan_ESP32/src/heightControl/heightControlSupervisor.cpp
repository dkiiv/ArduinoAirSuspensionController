// Height Control Supervisor -- ESP32 adapter. The decisions live in hcs_*.cpp (platform independent, replayed on a
// PC by eval/hcs_sim.cpp); this file only moves data between them and the hardware:
//   wheel tasks' cached readings -> hcs::Inputs -> Core::tick (every 100 ms) -> START / ABORT / PULSE -> Wheel
// Docs: OASMan_ESP32/docs/height-control-supervisor.md

#include "heightControlSupervisor.h"

#if HEIGHT_CONTROL_SUPERVISOR

#include <Arduino.h>
#include <preferencable.h> // writeBytes / readBytes: the same SPIFFS store the pressure AI keeps its samples in
#include "hcs_core.h"
#include "../airSuspensionUtil.h"
#include "../manifoldSaveData.h"

static hcs::Core core;
static portMUX_TYPE hcsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool evPreset = false;
static volatile bool evManual = false;
static uint8_t evPresetH[hcs::NC];

// Targets, references and learned fill rates. Kept forever, like the AI samples: survives reboots and power loss;
// rejected only if its magic / version / ranges are wrong (Core::begin).
static const char *const HCS_STATE_FILE = "/hcsState.bin";

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
    for (int i = 0; i < hcs::NC; i++)
    {
        getWheel(i)->releaseAutonomousPulse(); // the user's valve command owns the solenoids from here
    }
    evManual = true;
}

static void hcsLog(const char *line)
{
    Serial.println(line);
}

static bool loadPersist(hcs::PersistBlob &b)
{
    if (!SPIFFS.exists(HCS_STATE_FILE))
    {
        return false; // first boot with HCS
    }
    return readBytes(HCS_STATE_FILE, &b, sizeof(b)) == sizeof(b); // magic / version / ranges: Core::begin
}

static void savePersist()
{
    hcs::PersistBlob b;
    if (!core.exportPersist(b))
    {
        return;
    }
    writeBytes(HCS_STATE_FILE, &b, sizeof(b), "w");
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
    in.tank = getCompressor()->getTankPressure(); // range-checked in Core::ingest (invalid -> no fills)
    in.presence = isVehicleOn();                  // BT-ignition builds: an authenticated BLE client is connected
    in.enabled = getheightSensorMode() && getmaintainPressure();
    in.safetyMode = getsafetyMode();
    const float bagMax = (float)getbagMaxPressure();
    in.bagCeilPsi = bagMax < (float)MAX_PRESSURE_SAFETY ? bagMax : (float)MAX_PRESSURE_SAFETY;
    in.compressorOffPsi = (float)getcompressorOffPSI();
}

static void applyOutputs(const hcs::Outputs &out)
{
    for (int i = 0; i < hcs::NC; i++)
    {
        const hcs::GoalRequest &g = out.g[i];
        if (!g.active)
        {
            continue;
        }
        Wheel *w = getWheel(i);
        if (out.cmd == hcs::Cmd::START && !w->initAutonomousGoal(g.target, g.dir, g.ceilH, g.floorH, g.ceilP))
        {
            Serial.printf("HCS wheel %d refused goal %d\n", i, g.target);
        }
        else if (out.cmd == hcs::Cmd::ABORT)
        {
            w->endAutonomousPulse();
            w->requestAutonomousAbort();
        }
        else if (out.cmd == hcs::Cmd::PULSE && g.dir > 0 && !w->startAutonomousPulse(g.pulseMs, g.ceilP))
        {
            Serial.printf("HCS wheel %d refused pulse\n", i);
        }
    }
}

#if HCS_DIAG_DUMP
static void diagDump()
{
    static char buf[720];
    core.dump(buf, sizeof(buf));
    Serial.println(buf);
    // Raw electrical view: pre-calibration, UNclamped. <0 or >100 = voltage outside 0.5-4.5 V at the ADC pin.
    static const char *const cn[4] = {"FP", "RP", "FD", "RD"};
    size_t w = 0;
    buf[0] = 0;
    hcs::appendf(buf, sizeof(buf), w, "HRAW");
    for (int i = 0; i < 4; i++)
    {
        const float raw = getWheel(i)->getLevelRaw();
        hcs::appendf(buf, sizeof(buf), w, " %s=%.1f(%.2fV)", cn[i], raw, 0.5f + raw * 0.04f);
    }
    hcs::appendf(buf, sizeof(buf), w, " CAL");
    for (int i = 0; i < 4; i++)
    {
        hcs::appendf(buf, sizeof(buf), w, " %s[%.1f..%.1f mr%.1f]", cn[i], getheightCalMin(i), getheightCalMax(i), getheightCalMinRide(i));
    }
    hcs::appendf(buf, sizeof(buf), w, " HM=%d MP=%d BLE=%d comp=%d", getheightSensorMode() ? 1 : 0, getmaintainPressure() ? 1 : 0,
            isVehicleOn() ? 1 : 0, getCompressor()->isOn() ? 1 : 0);
    Serial.println(buf);
}
#endif

static void task_hcs(void *)
{
    hcs::PersistBlob blob;
    const bool have = loadPersist(blob);
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
        evPreset = evManual = false;
        portEXIT_CRITICAL(&hcsMux);
        if (p)
        {
            core.notifyPresetLoad(ph); // also counts as the manual event, so a same-tick evManual is not lost
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
