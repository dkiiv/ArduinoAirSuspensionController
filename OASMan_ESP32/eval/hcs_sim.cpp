// PC scenario simulator for the Height Control Supervisor. Never compiled into firmware.
//
// Compiles the PRODUCTION supervisor core (src/heightControl/hcs_core.cpp) against a crude quasi-static
// air-spring plant and replays the scenarios from docs/height-control-supervisor.md, asserting the safety
// properties (R1-R6). It tests the DECISION LOGIC, not the physics: the plant, the noise levels and the road
// model are assumptions, so thresholds that pass here still have to be confirmed with the diag build on the car.
//
// Build & run (from this directory):
//   g++ -std=c++17 -O2 -Wall -o hcs_sim hcs_sim.cpp && ./hcs_sim          (summary)
//   ./hcs_sim -v                                                          (with the full HCS decision log)
//
// Plant model (per corner, normalised): m = air mass, F = corner load, T = gas temperature factor.
//   equilibrium:  m*T / F = 1 + (h - 50) / 60      -> h = 50 + 60 (mT/F - 1)   (height %, 50 = preset)
//                 p = 100 F                         (psi; pressure carries the load, independent of air mass)
//   so: more load  -> h down, p up      (LOAD, and the compressed corners on a crown / in a corner)
//       less air   -> h down, p same    (leak, cooling)
//   Extension is limited at h = 100 (wheel hanging: p then follows the air mass), compression at h = 0.

#define HCS_HOST_BUILD
#include "../src/heightControl/hcs_core.cpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace hcs;

static bool g_verbose = false;
static std::vector<std::string> g_log;
static void logSink(const char *line)
{
    g_log.push_back(line);
    if (g_verbose)
        printf("    %s\n", line);
}

static const char *const NAMES[4] = {"FP", "RP", "FD", "RD"};
// load-transfer sign vectors in FP, RP, FD, RD order (passenger = right side, US)
static const float WARP[4] = {+1, -1, -1, +1};  // FP+RD loaded, FD+RP unloaded (the owner's hill spot)
static const float ROLL[4] = {+1, +1, -1, -1};  // load onto the passenger side (left-hand turn)
static const float PITCH[4] = {+1, -1, +1, -1}; // load onto the front (braking / nose-down slope)

struct Routine
{
    bool active = false, autonomous = false;
    int8_t dir = 0;
    float target = 0, ceilH = 100, floorH = 0, ceilP = 999;
    uint32_t start = 0;
};

struct Sim
{
    std::mt19937 rng{12345};
    std::normal_distribution<float> N{0.0f, 1.0f};

    uint32_t t = 1000;
    Core core;
    PersistBlob blob{};
    bool haveBlob = false;

    // plant
    float m[4] = {1, 1, 1, 1};
    float baseF[4] = {1, 1, 1, 1};
    float extraF[4] = {0, 0, 0, 0}; // people / cargo
    float warp = 0, roll = 0, pitch = 0;
    float T = 1.0f;
    float leakPerHour[4] = {0, 0, 0, 0}; // fraction of air mass per hour
    bool jacked[4] = {false, false, false, false};
    float tank = 170;
    bool compOn = false;
    bool presence = true;
    bool enabled = true;
    float rough = 0;  // road roughness (0 = parked)
    bool moving = false;
    float bounceT = 0;
    float rawFault[4] = {NAN, NAN, NAN, NAN}; // forced raw height (wire break)
    bool manualIn[4] = {false, false, false, false};
    int valveTo[4] = {0, 1, 2, 3}; // physical corner whose bag the logical corner's valves actually feed
    Routine r[4];
    uint32_t seq = 0;

    // calibration
    float calMin = 10, calMax = 90, minRide = 35;

    // bookkeeping
    float h[4], p[4];
    int starts = 0, aborts = 0;
    int startsWhileMoving = 0;
    uint32_t valveMsWhileMoving = 0;
    uint32_t valveMsAuto[4] = {0, 0, 0, 0};
    int dumpsOn[4] = {0, 0, 0, 0}, fillsOn[4] = {0, 0, 0, 0};
    float maxH[4] = {0, 0, 0, 0};
    uint32_t lastAbortAt = 0, lastStartAt = 0;
    bool anyValveOpenNow = false;

    void boot(bool restore)
    {
        core.begin(t, (restore && haveBlob) ? &blob : nullptr, false, logSink);
        for (auto &x : r)
            x = Routine();
    }

    float F(int i) const
    {
        float f = baseF[i] * (1 + warp * WARP[i] + roll * ROLL[i] + pitch * PITCH[i]) + extraF[i];
        if (jacked[i])
            f = 0.02f;
        return f < 0.02f ? 0.02f : f;
    }

    void solve()
    {
        for (int i = 0; i < 4; i++)
        {
            float f = F(i);
            float hh = 50 + 60 * (m[i] * T / f - 1);
            float pp = 100 * f;
            if (hh > 100)
            {
                hh = 100;
                pp = 100 * m[i] * T / (1 + 50.0f / 60.0f);
            }
            if (hh < 0)
            {
                hh = 0;
                float pAir = 100 * m[i] * T / (1 - 50.0f / 60.0f);
                pp = pAir < pp ? pAir : pp;
            }
            h[i] = hh;
            p[i] = pp;
        }
    }

    void step(uint32_t dtMs = 100)
    {
        float dt = dtMs / 1000.0f;
        t += dtMs;
        seq++;
        for (int i = 0; i < 4; i++)
            m[i] *= (1 - leakPerHour[i] * dt / 3600.0f);
        solve();

        // compressor (gated by presence exactly like compressor.cpp)
        if (presence && tank < 140)
            compOn = true;
        if (tank >= 180 || !presence)
            compOn = false;
        if (compOn)
            tank += 1.0f * dt;

        // valves: emulated goal routines + manual
        anyValveOpenNow = false;
        for (int i = 0; i < 4; i++)
        {
            Routine &R = r[i];
            bool inOpen = manualIn[i], outOpen = false;
            if (R.active)
            {
                bool stop = (t - R.start) > (R.autonomous ? (uint32_t)HCS_ROUTINE_TIMEOUT_MS : 15000u);
                float praw = p[i] + 0.4f * (tank - p[i]);
                if (R.dir > 0 && (h[i] >= R.target - 0.3f || h[i] >= R.ceilH || praw >= R.ceilP))
                    stop = true;
                if (R.dir < 0 && (h[i] <= R.target + 0.3f || h[i] <= R.floorH))
                    stop = true;
                if (stop)
                    R.active = false;
                else if (R.dir > 0)
                    inOpen = true;
                else
                    outOpen = true;
            }
            const int v = valveTo[i];
            if (inOpen && tank > p[v])
            {
                float dm = 0.083f * (tank - p[v]) / 100.0f * dt;
                m[v] += dm;
                tank -= dm * 150.0f;
            }
            if (outOpen)
                m[v] -= 0.06f * (p[v] / 100.0f) * dt;
            if (inOpen || outOpen)
            {
                anyValveOpenNow = true;
                if (moving)
                    valveMsWhileMoving += dtMs;
                if (R.active && R.autonomous)
                    valveMsAuto[i] += dtMs;
            }
            r[i].dir = R.dir;
        }
        solve();

        // sensors
        Inputs in;
        memset(&in, 0, sizeof(in));
        in.now = t;
        bounceT += dt;
        for (int i = 0; i < 4; i++)
        {
            float road = 0, proad = 0;
            if (rough > 0)
            {
                road = rough * (0.8f * N(rng) + 0.9f * sinf(2 * 3.14159f * 1.4f * bounceT + i));
                proad = rough * 2.5f * N(rng);
            }
            float hm = h[i] + 0.15f * N(rng) + road;
            float hmc = hm < 0 ? 0 : (hm > 100 ? 100 : hm);
            float pm = p[i] + 0.25f * N(rng) + proad;
            bool vo = manualIn[i] || r[i].active;
            if (vo && (manualIn[i] || r[i].dir > 0))
                pm += 0.4f * (tank - p[i]); // flow offset
            if (vo && r[i].dir < 0 && !manualIn[i])
                pm *= 0.7f;
            in.c[i].h = hmc;
            in.c[i].hRaw = std::isnan(rawFault[i]) ? calMin + hm * (calMax - calMin) / 100.0f : rawFault[i];
            in.c[i].p = pm;
            in.c[i].inOpen = manualIn[i] || (r[i].active && r[i].dir > 0);
            in.c[i].outOpen = r[i].active && r[i].dir < 0;
            in.c[i].routineActive = r[i].active;
            in.c[i].routineAutonomous = r[i].active && r[i].autonomous;
            in.c[i].seq = seq;
            in.c[i].calMinRaw = calMin;
            in.c[i].calMaxRaw = calMax;
            in.c[i].minRide = minRide;
            if (h[i] > maxH[i])
                maxH[i] = h[i];
        }
        in.tank = tank + 0.3f * N(rng);
        in.tankValid = true;
        in.compressorOn = compOn;
        in.presence = presence;
        in.enabled = enabled;
        in.safetyMode = false;
        in.bagCeilPsi = 200;
        in.compressorOffPsi = 180;

        Outputs out;
        core.tick(in, out);
        static long dfrom = getenv("HCS_DUMP_FROM") ? atol(getenv("HCS_DUMP_FROM")) : -1;
        static long dto = getenv("HCS_DUMP_TO") ? atol(getenv("HCS_DUMP_TO")) : -1;
        if (dfrom >= 0 && (long)t >= dfrom && (long)t <= dto)
        {
            char b[600];
            core.dump(b, sizeof(b));
            printf("    %s\n", b);
        }
        if (out.cmd == Cmd::START)
        {
            starts++;
            lastStartAt = t;
            if (moving)
                startsWhileMoving++;
            for (int i = 0; i < 4; i++)
                if (out.g[i].active)
                {
                    Routine &R = r[i];
                    R.active = true;
                    R.autonomous = true;
                    R.dir = out.g[i].dir;
                    R.target = out.g[i].target;
                    R.ceilH = out.g[i].ceilH;
                    R.floorH = out.g[i].floorH;
                    R.ceilP = out.g[i].ceilP;
                    R.start = t;
                    if (R.dir > 0)
                        fillsOn[i]++;
                    else
                        dumpsOn[i]++;
                }
        }
        else if (out.cmd == Cmd::ABORT)
        {
            aborts++;
            lastAbortAt = t;
            for (int i = 0; i < 4; i++)
                if (out.g[i].active && r[i].autonomous)
                    r[i].active = false;
        }
        if (out.persistNow && core.exportPersist(blob))
            haveBlob = true;
    }

    void run(float seconds, std::function<void(Sim &)> each = nullptr)
    {
        uint32_t n = (uint32_t)(seconds * 10);
        for (uint32_t k = 0; k < n; k++)
        {
            if (each)
                each(*this);
            step();
        }
    }

    // user loads a preset that the car is already sitting at (commits the anchor)
    void commitPreset()
    {
        uint8_t ph[4] = {50, 50, 50, 50};
        core.notifyPresetLoad(ph);
        run(20);
    }

    // driving segment: roughness plus cornering / braking events
    void drive(float seconds, float roughness, bool events = true)
    {
        moving = true;
        rough = roughness;
        float t0 = 0;
        run(seconds, [&](Sim &s) {
            t0 += 0.1f;
            if (!events)
                return;
            float ph = fmodf(t0, 60.0f);
            s.roll = (ph > 10 && ph < 30) ? 0.15f : ((ph > 35 && ph < 45) ? -0.12f : 0.0f); // long sweepers
            s.pitch = (ph > 50 && ph < 53) ? 0.20f : ((ph > 55 && ph < 58) ? -0.10f : 0.0f); // braking / accel
        });
        roll = pitch = 0;
        rough = 0;
        moving = false;
    }

    void disturbance(float seconds, float amp = 0.8f)
    {
        rough = amp;
        run(seconds);
        rough = 0;
    }

    void reset()
    {
        *this = Sim();
    }
};

// ---------------------------------------------------------------------------------------------------------

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char *scenario, const std::string &what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (ok)
        g_pass++;
    else
        g_fail++;
    (void)scenario;
}
static std::string fmt(const char *f, ...)
{
    char b[256];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof(b), f, ap);
    va_end(ap);
    return b;
}
static int countLog(const char *needle, size_t from = 0)
{
    int n = 0;
    for (size_t i = from; i < g_log.size(); i++)
        if (g_log[i].find(needle) != std::string::npos)
            n++;
    return n;
}

static void parkedAtPreset(Sim &s)
{
    s.boot(false);
    s.run(15);
    s.commitPreset();
    s.run(30);
}

// R4 + 3 a.m. challenge: hill spot, phone connected all night, cooling, then a slow leak on an extended corner
static void scenarioHill()
{
    printf("\nS1 hill spot (crown + slope), phone connected overnight, cooling, slow leak on an extended corner\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.T = 1.03f; // bags hot from the drive
    s.drive(600, 0.8f);
    // parking manoeuvre onto the crown: warp builds up while still rolling
    s.moving = true;
    s.rough = 0.6f;
    s.run(8, [](Sim &x) { x.warp += 0.15f / 80; x.pitch += 0.04f / 80; });
    s.rough = 0;
    s.moving = false;
    size_t mark = g_log.size();
    s.run(240);
    float arr[4];
    for (int i = 0; i < 4; i++)
        arr[i] = s.h[i];
    printf("  arrival heights FP=%.1f RP=%.1f FD=%.1f RD=%.1f (FP/RD compressed, RP/FD hanging)\n", arr[0], arr[1], arr[2], arr[3]);
    check(s.startsWhileMoving == 0 && s.valveMsWhileMoving == 0, "S1", "no valve activity while driving");
    check(countLog("SHIFT", mark) >= 4, "S1", fmt("arrival classified as terrain SHIFT on all 4 corners (%d SHIFT lines)", countLog("SHIFT", mark)));
    int startsBefore = s.starts;
    // cool down over 3 h (6 % of gas temperature)
    s.run(3 * 3600, [](Sim &x) { if (x.T > 0.97f) x.T -= 0.06f / (3600 * 10); });
    // slow leak on RP (an extended, low-pressure corner) for 5 h
    s.leakPerHour[C_RP] = 0.015f; // ~1.5 %/h of the air: a slow leak (about 12 % overnight)
    s.run(5 * 3600);
    float worst = 0;
    for (int i = 0; i < 4; i++)
        worst = fmaxf(worst, fabsf(s.h[i] - arr[i]));
    printf("  after 8 h: FP=%.1f RP=%.1f FD=%.1f RD=%.1f, %d batches, RP fills=%d\n", s.h[0], s.h[1], s.h[2], s.h[3], s.starts - startsBefore,
           s.fillsOn[C_RP]);
    check(s.dumpsOn[C_FD] == 0 && s.dumpsOn[C_RP] == 0, "S1", "never dumped the hanging corners (FD, RP)");
    check(s.h[C_FP] < 46 && s.h[C_RD] < 46, "S1", fmt("compressed corners left compressed (FP=%.1f RD=%.1f, flat preset is 50)", s.h[C_FP], s.h[C_RD]));
    check(worst <= HCS_DEADBAND_H + 1.0f, "S1", fmt("car held at its ARRIVAL geometry (worst corner %.1f%% from arrival)", worst));
    check(s.fillsOn[C_RP] >= 1, "S1", "slow leak on RP refilled toward its arrival height");
    check(countLog("LEAK RP") == 0, "S1", "slow leak did not trip the leak latch");
}

// R5: driving / cornering / braking / smooth highway / red lights, with a leak and a passenger tempting the classifier
static void scenarioDriving()
{
    printf("\nS2 driving: sweepers, braking, smooth highway, red lights, leak present\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.leakPerHour[C_RD] = 0.10f; // fairly fast leak while driving
    for (int lap = 0; lap < 6; lap++)
    {
        s.drive(240, 0.8f);         // normal road with sweepers / braking
        s.drive(300, 0.35f);        // smooth highway, still cornering
        s.moving = false;           // red light: stopped but "in traffic"
        s.run(60);
    }
    printf("  starts while moving=%d, valve-ms while moving=%u, DRIVING confirmations=%d, total starts=%d\n", s.startsWhileMoving,
           s.valveMsWhileMoving, countLog("DRIVING confirmed"), s.starts);
    check(s.startsWhileMoving == 0 && s.valveMsWhileMoving == 0, "S2", "zero autonomous actuation while moving (cornering, braking, highway)");
    check(s.starts == 0, "S2", "no actuation at 60 s red lights either (arrival quiet = 120 s)");

    printf("  worst case: motion detector BLIND (sensor-noise-only road) during a 5 min sustained sweeper\n");
    Sim b;
    g_log.clear();
    parkedAtPreset(b);
    b.moving = true;
    b.rough = 0;
    b.run(300, [](Sim &x) { x.roll = 0.18f; });
    b.roll = 0;
    b.run(300);
    b.moving = false;
    check(b.starts == 0, "S2", fmt("blind-detector cornering classified as SHIFT, never actuated (starts=%d, SHIFT lines=%d)", b.starts,
                                    countLog("SHIFT")));
}

// Suspected bug: leak below min ride overnight with no BLE, reboot in between, owner returns
static void scenarioLeakBelowMin()
{
    printf("\nS3 overnight leak below min ride (RD), BLE absent, manifold reboot at 5 h, owner returns at 8 h\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.presence = false;
    s.leakPerHour[C_RD] = 0.05f; // 40 % of the air over 8 h -> ~ -24 % height
    s.run(5 * 3600);
    int startsAbsent = s.starts;
    s.boot(true); // reboot (OTA / brownout): restore persisted targets
    s.run(3 * 3600);
    startsAbsent += s.starts - startsAbsent;
    printf("  RD before owner returns: %.1f (min ride %.0f), starts while absent=%d\n", s.h[C_RD], s.minRide, s.starts);
    check(s.h[C_RD] < s.minRide, "S3", "precondition: RD is below the calibrated min ride");
    check(s.starts == 0, "S3", "nothing actuated while no BLE client was connected (R6)");
    s.leakPerHour[C_RD] = 0;
    s.presence = true;
    s.disturbance(6);         // door, driver gets in
    s.extraF[C_FD] += 0.12f;  // driver
    s.extraF[C_RD] += 0.04f;
    s.run(300);
    printf("  5 min after owner returns: FP=%.1f RP=%.1f FD=%.1f RD=%.1f, starts=%d\n", s.h[0], s.h[1], s.h[2], s.h[3], s.starts);
    check(s.h[C_RD] >= 50 - HCS_LAND_TOL - 0.5f, "S3", fmt("RD lifted back to the preset, incl. completion pass (%.1f)", s.h[C_RD]));
    check(fabsf(s.h[C_FD] - 50) <= 1.5f, "S3", fmt("driver's load on FD compensated too (%.1f)", s.h[C_FD]));
    check(s.maxH[C_RD] <= 100 - HCS_CEIL_MARGIN + 1.5f, "S3", fmt("never past the ceiling (max %.1f)", s.maxH[C_RD]));

    printf("  variant: fresh boot with NOTHING persisted while RD already sits at %.0f\n", 26.0f);
    Sim f;
    g_log.clear();
    f.m[C_RD] = 1 - 24.0f / 60.0f;
    f.boot(false);
    f.run(200);
    printf("  RD after boot: %.1f, starts=%d\n", f.h[C_RD], f.starts);
    check(f.h[C_RD] >= f.minRide + HCS_FLOOR_LIFT_MARGIN - 1.0f, "S3", "BOTTOM_GUARD lifts an unanchored corner to min ride + margin");
    check(f.h[C_FP] < 52 && f.h[C_FD] < 52, "S3", "other corners untouched");
}

// R3: load / unload compensation, small load ignored
static void scenarioLoad()
{
    printf("\nS4 load: two rear passengers, groceries, passengers leave\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.disturbance(10);
    s.extraF[C_RP] += 0.09f;
    s.extraF[C_RD] += 0.09f;
    s.extraF[C_FP] -= 0.01f;
    s.extraF[C_FD] -= 0.01f;
    s.run(5);
    printf("  loaded: RP=%.1f RD=%.1f\n", s.h[C_RP], s.h[C_RD]);
    s.run(90);
    printf("  90 s later: RP=%.1f RD=%.1f starts=%d\n", s.h[C_RP], s.h[C_RD], s.starts);
    check(fabsf(s.h[C_RP] - 50) <= 1.5f && fabsf(s.h[C_RD] - 50) <= 1.5f, "S4", "rear lifted back to preset");
    check(s.fillsOn[C_FP] == 0 && s.fillsOn[C_FD] == 0, "S4", "front not touched");
    int st0 = s.starts;
    s.disturbance(5);
    s.extraF[C_RP] += 0.01f;
    s.extraF[C_RD] += 0.01f;
    s.run(120);
    check(s.starts == st0, "S4", "10 kg of groceries ignored (inside deadband)");
    s.run(600); // let the reversal lock window age (passengers sit for 10 min)
    s.disturbance(8);
    s.extraF[C_RP] -= 0.10f;
    s.extraF[C_RD] -= 0.10f;
    s.extraF[C_FP] += 0.01f;
    s.extraF[C_FD] += 0.01f;
    s.run(120);
    printf("  unloaded: RP=%.1f RD=%.1f dumps RP=%d RD=%d\n", s.h[C_RP], s.h[C_RD], s.dumpsOn[C_RP], s.dumpsOn[C_RD]);
    check(fabsf(s.h[C_RP] - 50) <= 2.0f && fabsf(s.h[C_RD] - 50) <= 2.0f, "S4", "rear lowered back after passengers left (UNLOAD)");
}

// R5: car driven off in the middle of a load correction
static void scenarioDriveOffMidFill()
{
    printf("\nS5 drive-off in the middle of a correction\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.disturbance(6);
    s.extraF[C_RP] += 0.12f;
    s.extraF[C_RD] += 0.12f;
    s.tank = 125; // slow fill so the routine is still running when the car leaves
    uint32_t until = s.t + 60000;
    while (s.starts == 0 && s.t < until)
        s.step();
    check(s.starts == 1, "S5", "load correction started");
    s.run(1.0f);
    uint32_t t0 = s.t;
    s.moving = true;
    s.rough = 0.8f;
    while (s.aborts == 0 && s.t < t0 + 10000)
        s.step();
    printf("  abort %u ms after the car started moving\n", s.t - t0);
    check(s.aborts == 1 && (s.t - t0) <= 1500, "S5", "autonomous fill aborted within 1.5 s of motion");
    s.step();
    check(!s.anyValveOpenNow, "S5", "all valves closed after abort");
    int st = s.starts;
    s.drive(300, 0.8f);
    check(s.starts == st, "S5", "no new correction while driving");
}

// R6
static void scenarioNoPresence()
{
    printf("\nS6 load with no BLE client, then the client connects\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.presence = false;
    s.disturbance(6);
    s.extraF[C_RP] += 0.12f;
    s.extraF[C_RD] += 0.12f;
    s.run(300);
    check(s.starts == 0 && countLog("VETO (no BLE") >= 1, "S6", "correction vetoed and logged while no client connected");
    s.presence = true;
    s.run(120);
    check(s.starts >= 1 && fabsf(s.h[C_RP] - 50) <= 1.5f, "S6", "corrected once the client connected");
}

// Faults
static void scenarioFaults()
{
    printf("\nS7 sensor faults: RD wire break, then a second sensor\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.rawFault[C_RD] = 112; // floating input drifts high
    s.leakPerHour[C_RD] = 0.2f;
    s.run(1800);
    check(countLog("FAULT RD height out of") == 1, "S7", "RD wire break latched");
    check(s.fillsOn[C_RD] == 0 && s.dumpsOn[C_RD] == 0, "S7", "faulted corner never actuated (no blind actuation)");
    s.rawFault[C_FP] = -8;
    s.leakPerHour[C_RP] = 0.2f;
    int st = s.starts;
    s.run(1800);
    check(s.core.state() == State::FAULT && s.starts == st, "S7", "two faulted sensors -> global FAULT, nothing actuated");
}

// External support: jack
static void scenarioJack()
{
    printf("\nS8 car jacked at RP (tyre change) with phone connected\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.disturbance(20, 1.2f);
    s.jacked[C_RP] = true;
    s.extraF[C_RD] += 0.2f;
    s.extraF[C_FP] += 0.1f;
    s.run(1800);
    printf("  RP h=%.1f p=%.1f, starts=%d\n", s.h[C_RP], s.p[C_RP], s.starts);
    check(countLog("EXTERNAL") >= 1 && s.starts == 0, "S8", "EXTERNAL support detected, zero autonomous actuation while jacked");
    s.disturbance(10, 1.2f);
    s.jacked[C_RP] = false;
    s.extraF[C_RD] -= 0.2f;
    s.extraF[C_FP] -= 0.1f;
    s.run(300);
    check(countLog("EXTERNAL cleared") == 1, "S8", "freeze released once the car is back on its wheels");
}

// Manual override
static void scenarioManual()
{
    printf("\nS9 user jogs FD during an autonomous rear correction\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.disturbance(6);
    s.extraF[C_RP] += 0.12f;
    s.extraF[C_RD] += 0.12f;
    s.tank = 125;
    uint32_t until = s.t + 60000;
    while (s.starts == 0 && s.t < until)
        s.step();
    s.run(0.5f);
    s.manualIn[C_FD] = true;
    s.run(2);
    s.manualIn[C_FD] = false;
    check(s.aborts == 1 && countLog("MANUAL override") >= 1, "S9", "manual valve wins: autonomous batch aborted");
    float fdAfter = s.h[C_FD];
    s.run(240);
    printf("  FD after jog %.1f -> %.1f, RP=%.1f RD=%.1f\n", fdAfter, s.h[C_FD], s.h[C_RP], s.h[C_RD]);
    check(countLog("COMMIT FD manual") == 1 && countLog("COMMIT RP manual") == 0 && countLog("COMMIT RD manual") == 0 && countLog("COMMIT FP manual") == 0,
          "S9", "only the corner the user touched was re-baselined");
    check(fabsf(s.h[C_FD] - fdAfter) <= 1.0f, "S9", "user's FD height respected (not undone)");
    check(fabsf(s.h[C_RP] - 50) <= 1.5f && fabsf(s.h[C_RD] - 50) <= 1.5f, "S9", "rear correction completed after the override settled");
}

// Fast leak latch + owner-return allowance
static void scenarioFastLeak()
{
    printf("\nS10 fast leak on RD: latch after %d refills, one refill per owner return\n", HCS_LEAK_FAULT_COUNT);
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.leakPerHour[C_RD] = 0.25f;
    s.run(6 * 3600);
    printf("  RD fills=%d, latched=%d\n", s.fillsOn[C_RD], countLog("LEAK RD:"));
    check(countLog("LEAK RD:") == 1, "S10", "leak latch tripped");
    check(s.fillsOn[C_RD] <= HCS_LEAK_FAULT_COUNT + 1, "S10", fmt("compressor not run endlessly (RD fills=%d)", s.fillsOn[C_RD]));
    int f0 = s.fillsOn[C_RD];
    s.presence = false;
    s.run(600);
    s.presence = true;
    s.run(300);
    check(s.fillsOn[C_RD] == f0 + 1, "S10", "owner return grants exactly one refill");
}

// User preset below min ride must be held, not "rescued"
static void scenarioLowPreset()
{
    printf("\nS11 user parks on a stance preset below min ride\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    for (int i = 0; i < 4; i++)
        s.m[i] = 1 - 25.0f / 60.0f; // user routine took it to 25
    uint8_t ph[4] = {25, 25, 25, 25};
    s.core.notifyPresetLoad(ph);
    s.run(600);
    check(s.starts == 0 && s.h[C_FP] < 30, "S11", "explicit user preset below min ride is held, not lifted");
}

// Valve / sensor corner mapping mismatch (e.g. a pin reorder applied to valves but not to height channels)
static void scenarioMapping()
{
    printf("\nS12 RP/RD valves cross-wired relative to the height sensors\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.valveTo[C_RP] = C_RD;
    s.valveTo[C_RD] = C_RP;
    s.leakPerHour[C_RP] = 0.10f;
    s.run(3 * 3600);
    printf("  starts=%d, RD max=%.1f\n", s.starts, s.maxH[C_RD]);
    check(countLog("FAULT MAPPING") == 1, "S12", "mismatch detected after the first batch");
    check(s.starts == 1, "S12", "exactly one batch ran before the global freeze");
    check(s.maxH[C_RD] < 50 + HCS_MAPPING_LIVE_DH + 2.0f, "S12", fmt("live check stopped the wrong-corner fill early (RD max %.1f)", s.maxH[C_RD]));
    check(s.core.state() == State::FAULT, "S12", "supervisor stays frozen");
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "-v") == 0)
        g_verbose = true;
    scenarioHill();
    scenarioDriving();
    scenarioLeakBelowMin();
    scenarioLoad();
    scenarioDriveOffMidFill();
    scenarioNoPresence();
    scenarioFaults();
    scenarioJack();
    scenarioManual();
    scenarioFastLeak();
    scenarioLowPreset();
    scenarioMapping();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
