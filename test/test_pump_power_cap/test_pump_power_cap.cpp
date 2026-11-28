// Unit tests: the pump driver (DimmedPump) with a flow cap on power phases (fork, maxa4turbo.18).
// The real DimmedPump.cpp and PressureController.cpp run against the model of Max's machine used in
// test_power_flow_cap (pump map and pucks from his shots 567-570), commanded the way the display does it:
// setFlowTarget / setPower from another task, a new value at most every 100 ms. The hardware side of DimmedPump
// (dimmer, pressure sensor, meter link, FreeRTOS) is replaced by the stand-ins below.
// Host-side: pio test -e native -f test_pump_power_cap.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <vector>

// ---- stand-ins for what DimmedPump takes from Arduino / FreeRTOS / ESP-IDF ----
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
#define FALLING 2
#define configMINIMAL_STACK_SIZE 768
#define pdMS_TO_TICKS(ms) (ms)
typedef void *xTaskHandle;
typedef uint32_t TickType_t;
static int64_t g_nowUs = 1000000;
inline int64_t esp_timer_get_time() { return g_nowUs; }
inline int xTaskCreate(void (*)(void *), const char *, uint32_t, void *, int, xTaskHandle *) { return 1; }
inline TickType_t xTaskGetTickCount() { return 0; }
inline void xTaskDelayUntil(TickType_t *, TickType_t) {}

// Headers that pull in the rest of the controller: skipped through their include guards, replaced right here.
#define GAGGIMATECONTROLLER_H
#define PRESSURESENSOR_H
#define METERLINK_H
#include "peripherals/MeterDiagnostics.h"
#include "peripherals/PressureTiming.h"

class PressureSensor {
  public:
    float bar = 0.0f;
    bool consumeAverage() { return true; }
    float getAveragedPressure() const { return bar; }
    float getSampleTime() const { return 1.0f / 60.0f; }
};

class MeterLink {
  public:
    using diagnostics_callback_t = std::function<bool(MeterDiagnostics &)>;
    float reading = 0.0f;
    bool valid = true;
    float flow() const { return reading; }
    bool flowValid() const { return valid; }
    void onDiagnostics(diagnostics_callback_t cb) { diagnostics = std::move(cb); }
    diagnostics_callback_t diagnostics;
};

#include "PressureController/PressureController.cpp"
#include "SimpleKalmanFilter/SimpleKalmanFilter.cpp"
#include "peripherals/DimmedPump.cpp"

// The firmware never defines these (the optimiser drops the base class's vtable); a host build asks for them.
void Pump::setup() {}
void Pump::loop() {}
void Pump::setPower(float) {}
float *Pump::getPumpPowerPtr() { return nullptr; }

namespace {

constexpr float kPumpPerPct = 0.2646f, kPumpPerBar = 0.7879f, kPumpOffset = 9.080f;
float pumpFlow(float powerPct, float bar) { return std::max(0.0f, kPumpPerPct * powerPct - kPumpPerBar * bar - kPumpOffset); }

FlowLoopGains maxGains() { // Max's meter page, Nov 27 2026
    FlowLoopGains g;
    g.kp = 0.025f;
    g.ki = 0.075f;
    g.startDuty = 0.20f;
    g.ff = 1.0f;
    g.ramp = 4.0f;
    g.limKp = 3.0f;
    g.limKi = 1.5f;
    g.deadZone = 0.10f;
    return g;
}

// What the display sends for the phase that is running.
struct Command {
    enum Kind { POWER, FLOW, PRESSURE } kind = POWER;
    float power = 0.0f;    // POWER
    float maxFlow = 0.0f;  // POWER: the phase's cap, 0 = none
    float flow = 0.0f;     // FLOW target / PRESSURE limit
    float pressure = 0.0f; // FLOW ceiling / PRESSURE target
    bool operator==(const Command &o) const {
        return kind == o.kind && power == o.power && maxFlow == o.maxFlow && flow == o.flow && pressure == o.pressure;
    }
};

struct Rig {
    static constexpr float kDt = 1.0f / 60.0f;

    PressureSensor sensor;
    MeterLink meter;
    DimmedPump pump{0, 0, &sensor, PressureControlRate::Hz60};

    std::function<Command(float)> commandAt = [](float) { return Command{}; };
    std::function<float(float)> resistanceAt = [](float) { return 1.0f; };
    bool useCapCommand = true; // false: a display that does not know the cap (sends the plain power command)
    float pumpLag = 0.15f;
    float powerEff = 0.0f, pressure = 0.0f, trueFlow = 0.0f, volume = 0.0f;
    std::deque<std::pair<float, float>> volumeLog;
    float nextMeter = 0.0f, nextDisplay = 0.0f, t = 0.0f;
    Command sent;
    bool sentAny = false;
    std::vector<float> powerTrace, pressureTrace;
    float maxStep = 0.0f; // largest change of the pump power in one control cycle

    Rig() {
        pump.setMeterLink(&meter);
        pump.setFlowLoopGains(maxGains());
        pump.setPumpFlowPolyCoeffs(0.0f, 0.122f, -2.7966f, 26.1908f); // his settings; flow phases run with ff = 1
        pump.setPumpSlipPolyCoeffs(0.0f, -0.122f, 2.7966f, 0.3062f);
        pump.setGains(0.05f, 3.5f, 0.05f);
        pump.setValveState(true);
    }

    float power() { return *pump.getPumpPowerPtr(); }

    void settleAt(float powerPct, float resistance) {
        powerEff = powerPct;
        float p = 6.0f;
        for (int i = 0; i < 200; i++)
            p = std::max(0.05f, p + 0.2f * (pumpFlow(powerPct, p) - std::sqrt(p) / resistance));
        pressure = p;
        trueFlow = meter.reading = pumpFlow(powerPct, p);
    }

    void send(const Command &c) {
        if (c.kind == Command::FLOW)
            pump.setFlowTarget(c.flow, c.pressure);
        else if (c.kind == Command::PRESSURE)
            pump.setPressureTarget(c.pressure, c.flow);
        else if (useCapCommand && c.maxFlow > 0.0f)
            pump.setPower(c.power, c.maxFlow); // what GaggiMateController does with a power command that has a flow
        else
            pump.setPower(c.power);
        sent = c;
        sentAny = true;
    }

    void step() {
        powerEff += (power() - powerEff) * std::min(1.0f, kDt / pumpLag);
        trueFlow = pumpFlow(powerEff, pressure);
        const float puckFlow = std::sqrt(std::max(0.0f, pressure)) / resistanceAt(t);
        pressure = std::max(0.0f, pressure + (trueFlow - puckFlow) / (3.5f / std::max(pressure, 0.5f)) * kDt);
        sensor.bar = pressure;
        volume += trueFlow * kDt;
        volumeLog.emplace_back(t, volume);
        while (!volumeLog.empty() && volumeLog.front().first < t - 0.15f - 1e-6f)
            volumeLog.pop_front();
        if (t >= nextMeter - 1e-6f) {
            if (volumeLog.size() > 1)
                meter.reading =
                    (volumeLog.back().second - volumeLog.front().second) / (volumeLog.back().first - volumeLog.front().first);
            nextMeter += 0.05f;
        }
        if (t >= nextDisplay - 1e-6f) { // the display sends what changed, every 100 ms
            const Command c = commandAt(t);
            if (!sentAny || !(c == sent))
                send(c);
            nextDisplay += 0.1f;
        }
        const float before = power();
        g_nowUs += 16667;
        pump.loop();
        maxStep = std::max(maxStep, std::fabs(power() - before));
        powerTrace.push_back(power());
        pressureTrace.push_back(pressure);
        t += kDt;
    }

    void run(float seconds) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; i++)
            step();
    }
};

// His profile from the rise on: "Rise 1.7 max 9" (flow phase; kRise seconds here, so the flow loop has settled
// when the power phases begin), "Hold power 66" (1 s), "Decline power 66 to 60" (linear over 25 s).
// cap: the maxFlow of the two power phases.
constexpr float kRise = 12.0f;
Command dflowPowerDecline(float t, float cap) {
    Command c;
    if (t < kRise) {
        c.kind = Command::FLOW;
        c.flow = 1.7f;
        c.pressure = 9.0f;
    } else {
        c.kind = Command::POWER;
        c.power = t < kRise + 1.0f ? 66.0f : std::max(60.0f, 66.0f - 6.0f * (t - kRise - 1.0f) / 25.0f);
        c.maxFlow = cap;
    }
    return c;
}
// Shot 570: resistance 1.4 at the end of the rise, 1.0 three seconds later.
float easyPuck(float t) { return t < kRise ? 1.4f : std::max(1.0f, 1.4f - 0.4f * (t - kRise) / 3.0f); }
// Shots 568 / 569: 1.565 at the end of the rise, climbing to 2.4.
float stiffPuck(float t) { return t < kRise ? 1.565f : std::min(2.4f, 1.565f + 0.835f * (t - kRise) / 25.0f); }

char msg[160];

} // namespace

// maxFlow 0 or missing: the plain power command, and the pump gets exactly the power that was sent.
void test_power_phase_without_cap_is_unchanged() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 0.0f); };
    rig.run(kRise);
    int checked = 0;
    for (int i = 0; i < 60 * 20; i++) {
        rig.step();
        if (rig.sent.kind != Command::POWER)
            continue; // the display's first power command is up to 100 ms away
        TEST_ASSERT_EQUAL_FLOAT(rig.sent.power, rig.power());
        checked++;
    }
    TEST_ASSERT_TRUE(checked > 60 * 19);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCap());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCapReport());
    TEST_ASSERT_FALSE(rig.pump.isFlowCapLimiting());
    TEST_ASSERT_TRUE(rig.trueFlow > 2.4f);
}

// A cap the flow never reaches gives the same shot, cycle for cycle, as the same profile without the cap: same
// pump power, same pressure.
void test_cap_out_of_reach_changes_nothing() {
    Rig with, without;
    for (Rig *r : {&with, &without}) {
        r->settleAt(61.8f, 1.565f);
        r->resistanceAt = stiffPuck;
    }
    with.commandAt = [](float t) { return dflowPowerDecline(t, 3.5f); };
    without.commandAt = [](float t) { return dflowPowerDecline(t, 0.0f); };
    with.run(kRise + 26.0f);
    without.run(kRise + 26.0f);
    TEST_ASSERT_EQUAL_UINT32(without.powerTrace.size(), with.powerTrace.size());
    for (size_t i = 0; i < with.powerTrace.size(); i++) {
        if (with.powerTrace[i] != without.powerTrace[i] || with.pressureTrace[i] != without.pressureTrace[i]) {
            snprintf(msg, sizeof(msg), "cycle %u: power %.4f / %.4f, pressure %.4f / %.4f", static_cast<unsigned>(i),
                     with.powerTrace[i], without.powerTrace[i], with.pressureTrace[i], without.pressureTrace[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, with.pump.getFlowCapReport());
    TEST_ASSERT_EQUAL_FLOAT(3.5f, with.pump.getFlowCap());
}

// Shots 568 / 569 (old beans) with maxFlow 2.0. The flow sits at 1.8 ml/s and under through both power phases,
// but the profile's own step from the flow loop's power (62 %) to 66 % makes a short spike in the pump flow.
// If that touches the cap, the cap may trim the power for that moment only: a few percent at most, and the pump
// is back on the ramp within two seconds and stays there.
void test_old_beans_only_touch_the_cap_at_the_step() {
    Rig rig;
    rig.settleAt(61.8f, 1.565f);
    rig.resistanceAt = stiffPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 2.0f); };
    rig.run(kRise);
    float lowestGap = 0.0f; // most the pump power fell under the power that was asked for
    for (int i = 0; i < 120; i++) {
        rig.step();
        if (rig.sent.kind == Command::POWER)
            lowestGap = std::max(lowestGap, rig.sent.power - rig.power());
    }
    snprintf(msg, sizeof(msg), "power fell %.2f %% under the ramp at the step", lowestGap);
    TEST_ASSERT_TRUE_MESSAGE(lowestGap < 3.0f, msg);
    snprintf(msg, sizeof(msg), "still holding 2 s after the step (power %.2f %%, flow %.2f ml/s)", rig.power(), rig.trueFlow);
    TEST_ASSERT_FALSE_MESSAGE(rig.pump.isFlowCapLimiting(), msg);
    for (int i = 0; i < 60 * 24; i++) {
        rig.step();
        TEST_ASSERT_EQUAL_FLOAT(rig.sent.power, rig.power());
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCapReport());
    TEST_ASSERT_TRUE(rig.trueFlow < 1.9f);
}

// The whole hand-over on shot 570's puck: flow phase -> power phase with a cap -> cap holds 2.0 ml/s.
void test_easy_puck_is_held_at_the_cap() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 2.0f); };
    rig.run(kRise - 0.05f);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DimmedPump::ControlMode::FLOW), static_cast<int>(rig.pump.getMode()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCapReport());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 1.7f, rig.trueFlow); // the flow loop is at the rise's 1.7 ml/s, under the cap
    rig.run(1.0f);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DimmedPump::ControlMode::POWER), static_cast<int>(rig.pump.getMode()));
    TEST_ASSERT_TRUE(rig.pump.isFlowCapLimiting());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rig.pump.getFlowCap());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rig.pump.getFlowCapReport());
    TEST_ASSERT_EQUAL_FLOAT(66.0f, rig.pump.getPowerSetpoint());
    rig.run(15.0f);
    snprintf(msg, sizeof(msg), "flow %.2f ml/s, power %.1f %% (asked %.1f %%)", rig.trueFlow, rig.power(), rig.sent.power);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.06f, 2.0f, rig.trueFlow, msg);
    TEST_ASSERT_TRUE_MESSAGE(rig.power() < rig.sent.power - 3.0f, msg);
    for (float p : rig.powerTrace)
        TEST_ASSERT_TRUE(p <= 66.0f + 1e-4f);
}

// While the cap holds the pump lower, a new ramp value from the display must not put the uncapped power on the
// pump, not even until the next control cycle (the CAN drive reads the power every 20 ms on its own clock).
void test_ramp_updates_never_reach_the_pump_uncapped() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 2.0f); };
    rig.run(kRise + 11.0f);
    TEST_ASSERT_TRUE(rig.pump.isFlowCapLimiting());
    const float held = rig.power();
    TEST_ASSERT_TRUE(held < 60.0f);
    rig.pump.setPower(63.5f, 2.0f); // the display's next step of the ramp
    TEST_ASSERT_EQUAL_FLOAT(held, rig.power());
    rig.pump.loop();
    TEST_ASSERT_FLOAT_WITHIN(0.2f, held, rig.power());
}

// Stopping is as immediate as it always was: power 0 is on the pump when setPower() returns, cap or not.
void test_stop_is_immediate() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 2.0f); };
    rig.run(kRise + 11.0f);
    TEST_ASSERT_TRUE(rig.pump.isFlowCapLimiting());
    rig.pump.setPower(0.0f); // shot ends
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.power());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCap());
    rig.pump.loop();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.power());
    TEST_ASSERT_FALSE(rig.pump.isFlowCapLimiting());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCapReport());
    rig.pump.setPower(0.0f, 2.0f); // a pause phase that still carries a cap in the profile
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.power());
    rig.pump.loop();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.power());
}

// What the display is told: the cap while it holds the pump and for half a second after, 0 otherwise.
void test_report_to_the_display() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    // the puck tightens 12 s into the power phase, so the cap lets go again
    rig.resistanceAt = [](float t) {
        return t < kRise + 12.0f ? easyPuck(t) : std::min(2.2f, 1.0f + (t - kRise - 12.0f) / 2.0f);
    };
    rig.commandAt = [](float t) {
        Command c = dflowPowerDecline(t, 2.0f);
        if (c.kind == Command::POWER)
            c.power = 66.0f;
        return c;
    };
    rig.run(kRise + 12.0f);
    TEST_ASSERT_TRUE(rig.pump.isFlowCapLimiting());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rig.pump.getFlowCapReport());
    float releasedAt = -1.0f, reportEndedAt = -1.0f;
    bool wasLimiting = true;
    for (int i = 0; i < 60 * 20 && reportEndedAt < 0.0f; i++) {
        rig.step();
        const bool limiting = rig.pump.isFlowCapLimiting();
        if (wasLimiting && !limiting)
            releasedAt = rig.t;
        if (limiting)
            releasedAt = -1.0f; // took the pump again: the hold starts over
        wasLimiting = limiting;
        if (releasedAt >= 0.0f && rig.pump.getFlowCapReport() == 0.0f)
            reportEndedAt = rig.t;
    }
    TEST_ASSERT_TRUE_MESSAGE(releasedAt > kRise + 12.0f, "the cap never let go");
    snprintf(msg, sizeof(msg), "report ended %.2f s after the cap let go", reportEndedAt - releasedAt);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.04f, 0.5f, reportEndedAt - releasedAt, msg);
    TEST_ASSERT_EQUAL_FLOAT(66.0f, rig.power());
}

// The $D line for the meter board's log: mode 0 (power), target_ml_s = the cap, and while the cap holds the pump
// ctrl_ml_s = the cap, i_pct = its integral and flag 32.
void test_diagnostics_line() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) { return dflowPowerDecline(t, 2.0f); };
    rig.run(kRise - 1.0f);
    MeterDiagnostics d;
    TEST_ASSERT_TRUE(rig.meter.diagnostics(d));
    TEST_ASSERT_EQUAL_UINT8(2, d.mode);
    TEST_ASSERT_EQUAL_FLOAT(1.7f, d.targetFlow);
    TEST_ASSERT_EQUAL_UINT8(0, d.flags & MeterDiagnostics::FLOW_CAP);
    rig.run(10.0f);
    TEST_ASSERT_TRUE(rig.meter.diagnostics(d));
    TEST_ASSERT_EQUAL_UINT8(0, d.mode);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, d.targetFlow);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, d.ctrlFlow);
    TEST_ASSERT_EQUAL_UINT8(MeterDiagnostics::FLOW_CAP, d.flags & MeterDiagnostics::FLOW_CAP);
    TEST_ASSERT_EQUAL_UINT8(0, d.flags &
                                   (MeterDiagnostics::MEASURED_LOOP | MeterDiagnostics::CEILING | MeterDiagnostics::FLOW_FAULT));
    TEST_ASSERT_TRUE(d.iPct > 30.0f && d.iPct < 70.0f);
    TEST_ASSERT_EQUAL_FLOAT(rig.power(), d.pumpPct);
    // a power phase without a cap: as before
    Rig plain;
    plain.settleAt(60.2f, 1.4f);
    plain.resistanceAt = easyPuck;
    plain.commandAt = [](float t) { return dflowPowerDecline(t, 0.0f); };
    plain.run(kRise + 9.0f);
    TEST_ASSERT_TRUE(plain.meter.diagnostics(d));
    TEST_ASSERT_EQUAL_UINT8(0, d.mode);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.targetFlow);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.ctrlFlow);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.iPct);
    TEST_ASSERT_EQUAL_UINT8(MeterDiagnostics::VALVE_OPEN, d.flags);
}

// A flow phase after a capped power phase starts from the power the cap was holding, not from the ramp power,
// and the cap's state is gone.
void test_flow_phase_after_a_capped_power_phase() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.resistanceAt = easyPuck;
    rig.commandAt = [](float t) {
        if (t < kRise + 11.0f)
            return dflowPowerDecline(t, 2.0f);
        Command c;
        c.kind = Command::FLOW;
        c.flow = 2.0f;
        c.pressure = 9.0f;
        return c;
    };
    rig.run(kRise + 10.95f);
    TEST_ASSERT_TRUE(rig.pump.isFlowCapLimiting());
    const float held = rig.power();
    rig.maxStep = 0.0f;
    rig.run(1.0f);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(DimmedPump::ControlMode::FLOW), static_cast<int>(rig.pump.getMode()));
    TEST_ASSERT_FALSE(rig.pump.isFlowCapLimiting());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCapReport());
    snprintf(msg, sizeof(msg), "power %.1f -> %.1f %%, largest change in one cycle %.2f %%", held, rig.power(), rig.maxStep);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxStep < 1.5f, msg);
    rig.run(6.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 2.0f, rig.trueFlow);
}

// No meter link (stock wiring) or a pump in on/off mode: the cap is ignored and the command is the plain one.
void test_no_meter_link_ignores_the_cap() {
    PressureSensor sensor;
    DimmedPump pump(0, 0, &sensor, PressureControlRate::Hz60);
    pump.setPower(66.0f, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(66.0f, *pump.getPumpPowerPtr());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pump.getFlowCap());
    pump.loop();
    TEST_ASSERT_EQUAL_FLOAT(66.0f, *pump.getPumpPowerPtr());

    Rig rig;
    rig.pump.setBinaryMode(true);
    rig.pump.setPower(66.0f, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(66.0f, rig.power());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCap());
}

// A cap that is not a positive number is no cap.
void test_bad_cap_values() {
    Rig rig;
    for (float cap : {0.0f, -1.0f, NAN, INFINITY}) {
        rig.pump.setPower(50.0f, cap);
        TEST_ASSERT_EQUAL_FLOAT(50.0f, rig.power());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.pump.getFlowCap());
    }
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_power_phase_without_cap_is_unchanged);
    RUN_TEST(test_cap_out_of_reach_changes_nothing);
    RUN_TEST(test_old_beans_only_touch_the_cap_at_the_step);
    RUN_TEST(test_easy_puck_is_held_at_the_cap);
    RUN_TEST(test_ramp_updates_never_reach_the_pump_uncapped);
    RUN_TEST(test_stop_is_immediate);
    RUN_TEST(test_report_to_the_display);
    RUN_TEST(test_diagnostics_line);
    RUN_TEST(test_flow_phase_after_a_capped_power_phase);
    RUN_TEST(test_no_meter_link_ignores_the_cap);
    RUN_TEST(test_bad_cap_values);
    return UNITY_END();
}
