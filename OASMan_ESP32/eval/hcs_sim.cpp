// PC bench simulator for the Height Control Supervisor. Never compiled into firmware.
// Docs: OASMan_ESP32/docs/hcs-simulator.md
//
// Compiles the PRODUCTION supervisor sources (src/heightControl/hcs_*.cpp) against a simulated car and replays
// scenarios, asserting what must (not) happen. It tests the DECISION LOGIC; the plant, noise and road models are
// assumptions, so thresholds that pass here still have to be confirmed with the shadow build on the car.
//
// Build & run (from OASMan_ESP32/):
//   g++ -std=c++17 -O2 -Wall -Wextra -o hcs_sim eval/hcs_sim.cpp && ./hcs_sim     (summary)
//   ./hcs_sim -v                                                                    (+ the full HCS decision log)
//   ./hcs_sim -v S16                                                                (one scenario)
//
// Plant model: a RIGID BODY (heave z, pitch, roll) on four isothermal air springs, solved for static
// equilibrium every 100 ms (Newton, 3 unknowns).
//   corner body height  zc_i = z + pitch * x_i + roll * y_i      (x: +1 front / -1 rear, y: +1 passenger / -1 driver)
//   suspension length   h_i  = zc_i - g_i                        (g_i = ground height under that wheel = terrain)
//   bag pressure        p_i  = 100 * m_i * T * (50 + 10) / (h_i + 10)   (p V = m R T, V ~ h + 10)
//   force balance       sum p_i = W,  sum p_i x_i = Mx,  sum p_i y_i = My   (loads, braking, cornering)
// So with no hand-tuned coupling: a crown (g warp) loads one diagonal and unloads the other; filling one corner
// moves its neighbours AND shifts their load (pressures) the way a real body does; a leak lowers its corner and
// re-distributes load; a corner whose length passes 100 has its wheel off the ground (jack) and carries nothing.
// Bump stops below h = 2. Road input / sensor noise are added on top of the measurements only.

#define HCS_HOST_BUILD
#include "../src/heightControl/hcs_core.cpp"
#include "../src/heightControl/hcs_sense.cpp"
#include "../src/heightControl/hcs_classify.cpp"
#include "../src/heightControl/hcs_cruise.cpp"

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
    float extraF[4] = {0, 0, 0, 0}; // people / cargo at that corner, fraction of a corner's static load
    float warp = 0;                 // ground warp (height %): FP+RD ground raised, FD+RP lowered (the hill spot crown)
    float roll = 0, pitch = 0;      // lateral / longitudinal load transfer, fraction of total weight
    float heave = 0;                // vertical load factor - 1 (bottom of a dip / hill at speed: +0.3 = 1.3 g)
    float bz = 50, bp = 0, br = 0;  // body heave / pitch / roll (solved)
    float T = 1.0f;
    float leakPerHour[4] = {0, 0, 0, 0}; // fraction of air mass per hour
    bool jacked[4] = {false, false, false, false};
    float tank = 170;
    bool compOn = false;
    bool presence = true;
    bool shadow = false; // boot the supervisor in shadow mode (decide + log, never actuate)
    bool enabled = true;
    float rough = 0;  // road roughness (0 = parked)
    bool moving = false;
    float bounceT = 0;
    float rawFault[4] = {NAN, NAN, NAN, NAN}; // forced raw height (wire break)
    bool manualIn[4] = {false, false, false, false};
    Routine r[4];
    uint32_t pulseUntil[4] = {0, 0, 0, 0};
    uint32_t seq = 0;
    int pulses = 0, pulsesInEvent = 0, pulsesOn[4] = {0, 0, 0, 0};
    uint32_t pulseMsTotal = 0;

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
        core.begin(t, (restore && haveBlob) ? &blob : nullptr, shadow, logSink);
        for (auto &x : r)
            x = Routine();
    }

    float F(int i) const { return p[i] / 100.0f; } // corner load fraction (diagnostics)

    float pAir(int i, float L) const { return 100.0f * m[i] * T * 60.0f / ((L < 0 ? 0 : L) + 10.0f); }

    // corner force on the body and its derivative w.r.t. the corner's body height
    void cornerForce(int i, float L, float zc, float &f, float &d) const
    {
        float La = L < 100 ? L : 100;
        float pa = pAir(i, La);
        float on = L <= 100 ? 1.0f : (L >= 101 ? 0.0f : 101 - L); // wheel leaves the ground past full extension
        f = pa * on;
        d = (L < 100 ? -pa / (La + 10) : 0) * on - (L > 100 && L < 101 ? pa : 0);
        if (L < 2)
        {
            f += 60 * (2 - L);
            d -= 60;
        }
        if (jacked[i] && zc < 120)
        {
            f += 40 * (120 - zc); // jack under the body at this corner
            d -= 40;
        }
    }

    void solve()
    {
        static const float PX[4] = {+1, -1, +1, -1}, PY[4] = {+1, +1, -1, -1};
        float W = 400 * (1 + heave), Mx = 400 * pitch, My = 400 * roll;
        for (int i = 0; i < 4; i++)
        {
            W += 100 * extraF[i];
            Mx += 100 * extraF[i] * PX[i];
            My += 100 * extraF[i] * PY[i];
        }
        for (int it = 0; it < 30; it++)
        {
            float R[3] = {-W, -Mx, -My}, J[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            for (int i = 0; i < 4; i++)
            {
                float zc = bz + bp * PX[i] + br * PY[i];
                float L = zc - warp * WARP[i];
                float f, d;
                cornerForce(i, L, zc, f, d);
                float sv[3] = {1, PX[i], PY[i]};
                for (int r2 = 0; r2 < 3; r2++)
                {
                    R[r2] += f * sv[r2];
                    for (int c2 = 0; c2 < 3; c2++)
                        J[r2][c2] += d * sv[r2] * sv[c2];
                }
            }
            if (fabsf(R[0]) + fabsf(R[1]) + fabsf(R[2]) < 1e-3f)
                break;
            for (int k = 0; k < 3; k++)
                J[k][k] -= 1e-3f; // regularise (all wheels hanging)
            // solve J * dx = -R (Cramer)
            float det = J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
                        J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
            if (fabsf(det) < 1e-9f)
                break;
            float dx[3];
            for (int c2 = 0; c2 < 3; c2++)
            {
                float M[3][3];
                for (int r2 = 0; r2 < 3; r2++)
                    for (int k = 0; k < 3; k++)
                        M[r2][k] = (k == c2) ? -R[r2] : J[r2][k];
                dx[c2] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                          M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
                if (dx[c2] > 5)
                    dx[c2] = 5;
                if (dx[c2] < -5)
                    dx[c2] = -5;
            }
            bz += dx[0];
            bp += dx[1];
            br += dx[2];
        }
        for (int i = 0; i < 4; i++)
        {
            float L = bz + bp * PX[i] + br * PY[i] - warp * WARP[i];
            float La = L < 0 ? 0 : (L > 100 ? 100 : L);
            h[i] = La;
            p[i] = pAir(i, La); // a hanging wheel's bag sits at full extension
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
            bool pulsing = pulseUntil[i] != 0 && (int32_t)(t - pulseUntil[i]) < 0;
            if (pulseUntil[i] != 0 && !pulsing)
                pulseUntil[i] = 0;
            if (pulsing && (fabsf(roll) > 0.01f || fabsf(pitch) > 0.01f))
                pulsesInEvent++; // a valve open at speed while the body is loaded by a corner / brake event
            if (pulsing)
                pulseMsTotal += dtMs;
            bool inOpen = manualIn[i] || pulsing, outOpen = false;
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
            const int v = i;
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
            bool vo = manualIn[i] || r[i].active || pulseUntil[i] != 0;
            if (vo && (manualIn[i] || pulseUntil[i] != 0 || r[i].dir > 0))
                pm += 0.4f * (tank - p[i]); // flow offset
            if (vo && r[i].dir < 0 && !manualIn[i])
                pm *= 0.7f;
            in.c[i].h = hmc;
            in.c[i].hRaw = std::isnan(rawFault[i]) ? calMin + hm * (calMax - calMin) / 100.0f : rawFault[i];
            in.c[i].p = pm;
            in.c[i].inOpen = manualIn[i] || pulseUntil[i] != 0 || (r[i].active && r[i].dir > 0);
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
            bool wasPulse = false;
            for (int i = 0; i < 4; i++)
            {
                if (out.g[i].active && pulseUntil[i] != 0)
                {
                    pulseUntil[i] = 0;
                    wasPulse = true;
                }
                if (out.g[i].active && r[i].autonomous)
                    r[i].active = false;
            }
            if (!wasPulse)
            {
                aborts++;
                lastAbortAt = t;
            }
        }
        else if (out.cmd == Cmd::PULSE)
        {
            for (int i = 0; i < 4; i++)
                if (out.g[i].active)
                {
                    pulseUntil[i] = t + out.g[i].pulseMs + 100; // adapter closes on the core's ABORT, +1 tick
                    pulses++;
                    pulsesOn[i]++;
                }
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
    void drive(float seconds, float roughness, bool events = true, float period = 60.0f)
    {
        moving = true;
        rough = roughness;
        float t0 = 0;
        run(seconds, [&](Sim &s) {
            t0 += 0.1f;
            if (!events)
                return;
            float ph = fmodf(t0, period);
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
    s.run(8, [](Sim &x) { x.warp += 9.0f / 80; x.pitch += 0.04f / 80; }); // ~9 % ground crown, nose-down slope
    s.rough = 0;
    s.moving = false;
    size_t mark = g_log.size();
    const int startsAtArrival = s.starts;
    s.run(240);
    float arr[4];
    for (int i = 0; i < 4; i++)
        arr[i] = s.h[i];
    printf("  arrival heights FP=%.1f RP=%.1f FD=%.1f RD=%.1f (FP/RD compressed, RP/FD hanging)\n", arr[0], arr[1], arr[2], arr[3]);
    check(s.startsWhileMoving == 0 && s.valveMsWhileMoving == 0, "S1", "no valve activity while driving");
    check(s.starts == startsAtArrival && countLog("ARRIVAL targets", mark) == 1, "S1",
          fmt("arrival on the crown: targets rebuilt once, nothing corrected (%d starts)", s.starts - startsAtArrival));
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
    check(s.h[C_FP] - arr[C_FP] <= 1.5f && s.h[C_RD] - arr[C_RD] <= 1.5f && s.h[C_FP] < 46 && s.h[C_RD] < 48, "S1",
          fmt("compressed corners never pushed toward flat (FP %.1f->%.1f, RD %.1f->%.1f; flat preset is 50)", arr[C_FP], s.h[C_FP],
              arr[C_RD], s.h[C_RD]));
    check(worst <= HCS_DEADBAND_H + 1.0f, "S1", fmt("car held at its ARRIVAL geometry (worst corner %.1f%% from arrival)", worst));
    check(s.fillsOn[C_RP] >= 1, "S1", "slow leak on RP refilled toward its arrival height");
    check(countLog("FAST leak latched") == 0, "S1", "slow leak did not trip the leak latch");
}

// R5: driving / cornering / braking / smooth highway / red lights, with a leak and a passenger tempting the classifier
static void scenarioDriving()
{
    printf("\nS2 driving: sweepers, braking, smooth highway, red lights (no leak: nothing may actuate)\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    for (int lap = 0; lap < 6; lap++)
    {
        s.drive(240, 0.8f);         // normal road with sweepers / braking
        s.drive(300, 0.35f);        // smooth highway, still cornering
        s.moving = false;           // red light: stopped but "in traffic"
        s.run(60);
    }
    printf("  starts while moving=%d, valve-ms while moving=%u, DRIVING confirmations=%d, total starts=%d\n", s.startsWhileMoving,
           s.valveMsWhileMoving, countLog("DRIVING confirmed"), s.starts);
    check(s.startsWhileMoving == 0 && s.valveMsWhileMoving == 0 && s.pulses == 0, "S2",
          "zero autonomous actuation while moving incl. cruise top-up (cornering, braking, highway)");
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
    check(f.fillsOn[C_FP] + f.fillsOn[C_RP] + f.fillsOn[C_FD] + f.dumpsOn[C_FP] + f.dumpsOn[C_RP] + f.dumpsOn[C_FD] == 0, "S3",
          fmt("only RD actuated (the others moved by body coupling only: FP=%.1f RP=%.1f FD=%.1f)", f.h[C_FP], f.h[C_RP], f.h[C_FD]));
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
    check(countLog("FAULT RD height sensor") == 1, "S7", "RD wire break latched");
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
    s.run(1800);
    printf("  RP h=%.1f p=%.1f, starts=%d\n", s.h[C_RP], s.p[C_RP], s.starts);
    check(countLog("EXTERNAL") >= 1 && s.starts == 0, "S8", "EXTERNAL support detected, zero autonomous actuation while jacked");
    s.disturbance(10, 1.2f);
    s.jacked[C_RP] = false;
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
    printf("\nS10 fast leak on RD (25 %%/h of its air): latch after %d fast refills, one refill per owner return\n", HCS_LEAK_FAST_COUNT);
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.leakPerHour[C_RD] = 0.25f;
    s.run(6 * 3600);
    printf("  RD fills=%d, latched=%d\n", s.fillsOn[C_RD], countLog("LEAK RD:"));
    check(countLog("LEAK RD:") == 1, "S10", "leak latch tripped");
    check(s.fillsOn[C_RD] <= HCS_LEAK_FAST_COUNT + 2, "S10", fmt("compressor not run endlessly (RD fills=%d)", s.fillsOn[C_RD]));
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

// Show mode: user preset with every corner at 0 (bags dumped, car on its bump stops)
static void scenarioShowMode()
{
    printf("\nS11b show preset: all corners 0 %% height / ~0 psi, phone connected, people get in and out\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    for (int i = 0; i < 4; i++)
        s.m[i] = 0.02f; // the user's preset routine dumped everything
    uint8_t ph[4] = {0, 0, 0, 0};
    s.core.notifyPresetLoad(ph);
    s.run(120);
    printf("  aired out: FP=%.1f RP=%.1f FD=%.1f RD=%.1f psi FP=%.1f\n", s.h[0], s.h[1], s.h[2], s.h[3], s.p[0]);
    s.disturbance(8);
    s.extraF[C_FD] += 0.15f;
    s.run(600);
    s.disturbance(8);
    s.extraF[C_FD] -= 0.15f;
    s.run(2 * 3600);
    check(s.starts == 0, "S11b", fmt("show preset held: zero autonomous actuation in 2 h (starts=%d)", s.starts));
    check(countLog("FAULT") == 0, "S11b", "no false sensor fault on empty bags");
}

// Rigid-body coupling: a fill on one corner moves the other three; only the leaking corner may be actuated
static void scenarioCoupling()
{
    printf("\nS13 rigid-body coupling: every fill moves the neighbours too (correct wiring)\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.leakPerHour[C_RD] = 0.04f; // single-corner refills: the body tilts, neighbours follow
    s.run(3 * 3600);
    s.leakPerHour[C_RD] = 0;
    s.disturbance(8);
    s.extraF[C_RP] += 0.14f; // uneven rear load -> axle batch with one corner needing much less
    s.extraF[C_RD] += 0.05f;
    s.run(900);
    printf("  starts=%d, final FP=%.1f RP=%.1f FD=%.1f RD=%.1f\n", s.starts, s.h[0], s.h[1], s.h[2], s.h[3]);
    check(s.starts >= 1 && s.core.state() != State::FAULT, "S13", fmt("corrections kept working (%d batches)", s.starts));
    float worst = 0;
    for (int i = 0; i < 4; i++)
        worst = fmaxf(worst, fabsf(s.h[i] - 50));
    check(worst <= HCS_DEADBAND_H + 0.5f, "S13", fmt("all corners within deadband despite coupling (worst %.1f)", worst));
}

// The owner's real leak: rear driver (RD) on the hill spot, compressed overnight, ~1-2 psi/h
static void scenarioOwnersLeak()
{
    printf("\nS14 owner's leak: RD compressed on the hill spot, 2 %%/h of its air, 24 h connected\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    s.drive(300, 0.8f);
    s.moving = true;
    s.rough = 0.6f;
    s.run(8, [](Sim &x) { x.warp += 9.0f / 80; });
    s.rough = 0;
    s.moving = false;
    s.run(300);
    float arrRD = s.h[C_RD];
    s.leakPerHour[C_RD] = 0.02f;
    float worstRD = 0;
    s.run(24 * 3600, [&](Sim &x) { worstRD = fmaxf(worstRD, arrRD - x.h[C_RD]); });
    printf("  RD arrival %.1f, now %.1f, worst sag %.1f, RD fills=%d, latched=%d\n", arrRD, s.h[C_RD], worstRD, s.fillsOn[C_RD],
           countLog("FAST leak latched"));
    check(countLog("FAST leak latched") == 0, "S14", "slow leak never latched");
    check(worstRD <= HCS_DEADBAND_H + 1.0f, "S14", fmt("RD held at its arrival height all day (worst sag %.1f)", worstRD));
    check(countLog("LEAK RD refill") >= 1, "S14", "leak rate logged on every refill");
}

// Road trip: 4 h of continuous driving with a slow leak -> cruise top-up keeps the corner up, only when steady
static void scenarioRoadTrip()
{
    printf("\nS15 road trip: 4 h non-stop, RD leaking 2.5 %%/h of its air; 5 min twisty / 5 min highway alternating\n");
    Sim s;
    g_log.clear();
    parkedAtPreset(s);
    // one parked correction first so the RD fill rate is learned (as it would be in daily use)
    s.disturbance(6);
    s.extraF[C_RD] += 0.10f;
    s.run(120);
    s.disturbance(6);
    s.extraF[C_RD] -= 0.10f;
    s.run(900);
    s.leakPerHour[C_RD] = 0.025f;
    float worstLate = 0;
    float mStart = s.m[C_RD];
    for (int seg = 0; seg < 24; seg++) // 24 x 10 min
    {
        s.drive(300, 0.8f, true);         // twisty: a sweeper / brake event every minute
        s.drive(300, 0.35f, true, 150.0f); // highway: an event every 2.5 min
        if (seg >= 6)
        {
            float others = (s.h[C_FP] + s.h[C_RP] + s.h[C_FD]) / 3.0f;
            worstLate = fmaxf(worstLate, others - s.h[C_RD]);
        }
    }
    printf("  pulses: FP=%d RP=%d FD=%d RD=%d (total %u ms), pulses overlapping a corner/brake event: %d\n", s.pulsesOn[0], s.pulsesOn[1],
           s.pulsesOn[2], s.pulsesOn[3], s.pulseMsTotal, s.pulsesInEvent);
    printf("  RD air lost %.1f%% to the leak; worst RD deficit vs others after the first hour: %.1f%%\n", (mStart - s.m[C_RD]) * 100.0f,
           worstLate);
    s.roll = s.pitch = 0;
    s.solve();
    printf("  static now: h FP=%.1f RP=%.1f FD=%.1f RD=%.1f | p FP=%.1f RP=%.1f FD=%.1f RD=%.1f\n", s.h[0], s.h[1], s.h[2], s.h[3], s.p[0],
           s.p[1], s.p[2], s.p[3]);
    check(s.pulsesOn[C_RD] >= 1 && s.pulsesOn[C_FP] + s.pulsesOn[C_RP] + s.pulsesOn[C_FD] == 0, "S15", "only the leaking corner topped up");
    check(s.startsWhileMoving == 0, "S15", "no closed-loop goal routine while moving (pulses only)");
    check(s.pulsesInEvent == 0, "S15", "no valve open during any cornering / braking event");
    check(worstLate <= 6.0f, "S15", fmt("RD kept within %.1f%% of the others (no top-up: grows ~1.5 %%/h)", worstLate));
    check(s.presence, "S15", "(precondition) phone connected for the whole trip");
    check(countLog("FAST leak latched") == 0, "S15", "slow leak not latched");
}

// Dips / the bottom of a hill at speed: all four corners compress and every bag pressure rises together.
// Must never be filled. Then real weight is added after parking: must still be compensated.
static void scenarioDips()
{
    printf("\nS16 dips / bottom of hills at speed (all 4 compress, pressures up), then real load after parking\n");
    // case 0: normal road texture, sharp dips (0.35 g for 3 s, every 4th one 8 s)
    // case 1: glassy road (no texture at all), same dips: the detector only sees the dips themselves
    // case 2: glassy road, long gentle highway sag curves (0.12 g over 9 s): the detector sees NOTHING, the car looks
    //         parked, evaluations run -- only the confirmation rule (same correction wanted twice >= 10 s apart,
    //         no motion in between) stands between a sag and a fill.
    static const char *const NAME[3] = {"normal road, sharp dips", "glassy road, sharp dips", "glassy road, long sags"};
    for (int cs = 0; cs < 3; cs++)
    {
        Sim s;
        g_log.clear();
        parkedAtPreset(s);
        s.moving = true;
        s.rough = cs == 0 ? 0.35f : 0.0f;
        float t0 = 0, minH = 100;
        s.run(900, [&](Sim &x) {
            t0 += 0.1f;
            const float ph = fmodf(t0, 45.0f);
            const float dur = cs == 2 ? 9.0f : ((fmodf(t0, 180.0f) < 45.0f) ? 8.0f : 3.0f);
            const float g = cs == 2 ? 0.12f : 0.35f;
            x.heave = (ph < dur) ? g * 0.5f * (1 - cosf(2 * 3.14159f * ph / dur)) : 0.0f;
            for (int i = 0; i < 4; i++)
                minH = fminf(minH, x.h[i]);
        });
        s.heave = 0;
        s.rough = 0;
        s.moving = false;
        const int wanted = countLog("wanted, confirming");
        printf("  %s: deepest compression %.1f%% (preset 50), DRIVING confirmed %d, corrections held for confirmation %d\n", NAME[cs],
               minH, countLog("DRIVING confirmed"), wanted);
        check(s.startsWhileMoving == 0 && s.valveMsWhileMoving == 0 && s.pulses == 0, "S16",
              fmt("%s: no fill / dump / pulse in 15 min", NAME[cs]));
        if (cs == 2)
            check(wanted > 0, "S16", "long sags: the confirmation rule was exercised (an evaluation saw the sag) and held");
        if (cs != 0)
            continue;
        // parked now: two adults into the rear seats
        s.run(180);
        const int st0 = s.starts;
        s.disturbance(8);
        s.extraF[C_RP] += 0.18f;
        s.extraF[C_RD] += 0.18f;
        s.run(150);
        check(s.starts > st0 && fabsf(s.h[C_RP] - 50) <= HCS_DEADBAND_H && fabsf(s.h[C_RD] - 50) <= HCS_DEADBAND_H, "S16",
              fmt("weight added after parking is still compensated (RP %.1f RD %.1f)", s.h[C_RP], s.h[C_RD]));
    }
}

// Shadow build (the first thing flashed on a car): decides and logs, must never actuate or latch anything.
static void scenarioShadow()
{
    printf("\nS17 shadow build: leak + load for 2 h, phone connected -> logs SHADOW decisions, never actuates\n");
    Sim s;
    g_log.clear();
    s.shadow = true;
    parkedAtPreset(s);
    s.leakPerHour[C_RD] = 0.10f;
    s.run(1800);
    s.disturbance(6);
    s.extraF[C_RP] += 0.18f;
    s.run(5400);
    printf("  SHADOW would START lines: %d, starts=%d, RD now %.1f\n", countLog("SHADOW would START"), s.starts, s.h[C_RD]);
    check(s.starts == 0 && s.pulses == 0, "S17", "no actuation at all");
    check(countLog("SHADOW would START") > 0, "S17", "the decisions it would have taken are logged");
    check(countLog("FAST leak latched") == 0 && countLog("LEAK RD refill") == 0, "S17", "no leak bookkeeping from decisions that never ran");
}

// Get in and drive from the hill spot: no BLE overnight; trunk loaded and passengers get in while the car is off;
// the controller connects when the driver sits; drive off quickly; arrive on flat ground (or back home).
static void scenarioGetInAndDrive()
{
    printf("\nS18 get in and drive from the hill spot (BLE only once the driver sits), arrive on flat ground / back home\n");
    for (int dest = 0; dest < 3; dest++)
    {
        Sim s;
        g_log.clear();
        parkedAtPreset(s);
        s.drive(300, 0.6f);
        s.drive(8, 0.3f, false);
        s.run(8, [](Sim &x) { x.warp += 9.0f / 80; x.pitch += 0.04f / 80; }); // park on the crown
        s.run(600);
        s.presence = false; // owner leaves; overnight
        s.run(3600);
        // morning, car off: 20 kg in the trunk, one passenger in the rear (driver side)
        s.disturbance(5);
        s.extraF[C_RP] += 0.06f;
        s.extraF[C_RD] += 0.06f;
        s.run(20);
        s.disturbance(5);
        s.extraF[C_RD] += 0.14f;
        s.run(5);
        // driver gets in (FD), car powers up, the in-car controller connects
        s.disturbance(5);
        s.extraF[C_FD] += 0.16f;
        s.presence = true;
        const uint32_t seated = s.t;
        const int st0 = s.starts;
        if (dest == 2)
        {
            // nobody drives off: how fast is the parked correction?
            uint32_t firstStart = 0;
            s.run(120, [&](Sim &x) {
                if (!firstStart && x.starts > st0)
                    firstStart = x.t;
            });
            printf("  parked, nobody drives off: first correction started %.0f s after the driver sat down; RP %.1f RD %.1f FD %.1f\n",
                   firstStart ? (firstStart - seated) / 1000.0f : -1.0f, s.h[C_RP], s.h[C_RD], s.h[C_FD]);
            continue;
        }
        s.run(8); // pull away 8 s after sitting down
        const int pulses0 = s.pulses;
        // 15 min drive, ending either on flat ground or back on the crown
        if (dest == 0)
            s.run(4, [](Sim &x) { x.warp -= 9.0f / 40; x.pitch -= 0.04f / 40; });
        s.drive(900, 0.6f);
        s.drive(8, 0.3f, false);
        s.run(300);
        const float wp = 0.25f * (s.h[C_FP] + s.h[C_RD] - s.h[C_FD] - s.h[C_RP]);
        printf("  %s: drive-away pulses %d (RP %d RD %d FD %d FP %d); after arrival FP %.1f RP %.1f FD %.1f RD %.1f (twist %+.1f)\n",
               dest == 0 ? "arrive on flat ground" : "arrive back home on the crown", s.pulses - pulses0, s.pulsesOn[C_RP], s.pulsesOn[C_RD],
               s.pulsesOn[C_FD], s.pulsesOn[C_FP], s.h[C_FP], s.h[C_RP], s.h[C_FD], s.h[C_RD], wp);
        float plane[4];
        {
            // plane part of the heights (twist removed) vs the preset plane (50 everywhere)
            const float sg[4] = {+1, -1, -1, +1};
            for (int i = 0; i < 4; i++)
                plane[i] = s.h[i] - sg[i] * wp;
        }
        float worst = 0;
        for (int i = 0; i < 4; i++)
            worst = fmaxf(worst, fabsf(plane[i] - 50));
        check(worst <= HCS_DEADBAND_H + 0.5f, "S18", fmt("%s: car level to the preset apart from the ground's twist (worst %.1f)", dest == 0 ? "flat" : "crown", worst));
        if (dest == 1)
            check(fabsf(wp) > 4.0f, "S18", fmt("back home: the crown's twist is left alone (%.1f)", wp));
    }
}

int main(int argc, char **argv)
{
    // usage: hcs_sim [-v] [S<n> ...]   (-v = print the HCS decision log; S<n> = run only those scenarios)
    std::vector<std::string> only;
    for (int a = 1; a < argc; a++)
    {
        if (strcmp(argv[a], "-v") == 0)
            g_verbose = true;
        else
            only.push_back(argv[a]);
    }
    struct
    {
        const char *id;
        void (*fn)();
    } const all[] = {{"S1", scenarioHill},       {"S2", scenarioDriving},     {"S3", scenarioLeakBelowMin}, {"S4", scenarioLoad},
                     {"S5", scenarioDriveOffMidFill}, {"S6", scenarioNoPresence}, {"S7", scenarioFaults},  {"S8", scenarioJack},
                     {"S9", scenarioManual},     {"S10", scenarioFastLeak},   {"S11", scenarioLowPreset},   {"S11b", scenarioShowMode},
                     {"S13", scenarioCoupling},  {"S14", scenarioOwnersLeak}, {"S15", scenarioRoadTrip},    {"S16", scenarioDips},
                     {"S17", scenarioShadow},
                     {"S18", scenarioGetInAndDrive}};
    for (const auto &sc : all)
    {
        bool run = only.empty();
        for (const auto &o : only)
            run = run || o == sc.id;
        if (run)
            sc.fn();
    }
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
