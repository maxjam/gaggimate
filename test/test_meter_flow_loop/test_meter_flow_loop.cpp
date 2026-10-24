// Unit tests: measured-flow control in PressureController (fork).
// Drives the controller in FLOW mode against a simulated Velofuso vane pump + puck, with the
// flow meter modelled as a 20 Hz sampler one sample late. Host-side — pio test -e native -f
// test_meter_flow_loop.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#define ESP_LOGI(tag, fmt, ...) ((void)0)

#include "PressureController/PressureController.cpp"
#include "SimpleKalmanFilter/SimpleKalmanFilter.cpp"

namespace {

// Pump model: Max's Oct-2026 Velofuso coefficients (full-drive curve and slip, bar -> ml/s).
constexpr float kFlowA = -0.00220f, kFlowB = 0.03818f, kFlowC = -1.3725f, kFlowD = 23.655f;
constexpr float kSlipA = 0.00220f, kSlipB = -0.03818f, kSlipC = 1.3725f, kSlipD = 6.325f;

float poly(float a, float b, float c, float d, float p) { return ((a * p + b) * p + c) * p + d; }
float fullDriveFlow(float p) { return std::max(0.0f, poly(kFlowA, kFlowB, kFlowC, kFlowD, p)); }
float slip(float p) { return std::max(0.0f, poly(kSlipA, kSlipB, kSlipC, kSlipD, p)); }
float pumpFlow(float duty, float p) { return std::max(0.0f, duty * (fullDriveFlow(p) + slip(p)) - slip(p)); }

struct Rig {
    static constexpr float kDt = 1.0f / 60.0f; // DimmedPump control period on the Pro (60 Hz)
    static constexpr float kMeterPeriod = 0.05f; // meter board sends $F at 20 Hz

    float rawPressureSetpoint = 0.0f; // the phase's ceiling in FLOW mode
    float rawFlowSetpoint = 0.0f;
    float rawPressure = 0.0f;
    float controllerOutput = 0.0f; // 0..100 %
    int valveStatus = 1;
    PressureController controller;

    // plant
    float compliance = 1.4f;        // ml/bar
    float puckConductance = 1e6f;   // ml/s per bar; huge = open group
    float pumpLag = 0.10f;          // s, BLDC speed response
    float dutyEff = 0.0f;           // actual pump duty after the lag
    float pressure = 0.0f;          // bar
    float trueFlow = 0.0f;          // ml/s out of the pump
    float meterReading = 0.0f;      // what the controller sees
    float meterAccum = 0.0f;        // ml counted in the current 50 ms window
    float meterClock = 0.0f;
    bool meterValid = true;
    float t = 0.0f;
    float maxPressure = 0.0f;
    float maxFlow = 0.0f;

    Rig() : controller(kDt, &rawPressureSetpoint, &rawFlowSetpoint, &rawPressure, &controllerOutput, &valveStatus) {
        controller.setPumpFlowPolyCoeffs(kFlowA, kFlowB, kFlowC, kFlowD);
        controller.setPumpSlipPolyCoeffs(kSlipA, kSlipB, kSlipC, kSlipD);
        controller.enableMeasuredFlowControl(true);
    }

    void step(PressureController::ControlMode mode = PressureController::ControlMode::FLOW) {
        // plant from the last duty
        const float dutyCmd = controllerOutput / 100.0f;
        dutyEff += (dutyCmd - dutyEff) * (kDt / pumpLag);
        trueFlow = pumpFlow(dutyEff, pressure);
        const float puckFlow = puckConductance * pressure;
        pressure += (trueFlow - puckFlow) / compliance * kDt;
        pressure = std::max(0.0f, pressure);
        rawPressure = pressure;
        // meter: counts volume over 50 ms, publishes once per window
        meterAccum += trueFlow * kDt;
        meterClock += kDt;
        if (meterClock >= kMeterPeriod - 1e-6f) {
            meterReading = meterAccum / meterClock;
            meterAccum = 0.0f;
            meterClock = 0.0f;
        }
        controller.setMeasuredFlow(meterReading, meterValid);
        controller.update(mode, true, kDt, kDt);
        t += kDt;
        maxPressure = std::max(maxPressure, pressure);
        maxFlow = std::max(maxFlow, trueFlow);
    }

    void run(float seconds, PressureController::ControlMode mode = PressureController::ControlMode::FLOW) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; i++)
            step(mode);
    }
};

} // namespace

// Open group: flow must settle on the target with no large overshoot.
void test_open_group_tracks_target() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f; // ceiling far away
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxFlow < 2.0f * 1.6f, "flow overshoot above 60 %");
    // settles: average of the last second within 5 %
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < 60; i++) {
        rig.step();
        sum += rig.trueFlow;
        n++;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, sum / n);
}

// Target step mid-phase (profile ramp): follows within a couple of seconds.
void test_follows_setpoint_change() {
    Rig rig;
    rig.rawFlowSetpoint = 1.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 1.0f, rig.trueFlow);
    rig.rawFlowSetpoint = 3.0f;
    rig.run(3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 3.0f, rig.trueFlow);
    rig.rawFlowSetpoint = 0.5f;
    rig.run(3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 0.5f, rig.trueFlow);
}

// Tight puck: pressure must be held at the ceiling, not stall below it and not run away above it.
void test_pressure_ceiling_is_held() {
    Rig rig;
    rig.puckConductance = 0.12f; // passes 1.08 ml/s at 9 bar, less than the 2 ml/s target
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(16.0f);
    char msg[96];
    snprintf(msg, sizeof(msg), "peak pressure %.2f bar", rig.maxPressure);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxPressure < 9.8f, msg);
    snprintf(msg, sizeof(msg), "settled pressure %.2f bar (stock stalled ~2 bar low)", rig.pressure);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.25f, 9.0f, rig.pressure, msg);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
    TEST_ASSERT_TRUE(rig.controller.getEffectiveFlowTarget() < 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 0.12f * 9.0f, rig.trueFlow);
}

// Same, with the softer system (air in the headspace) stock assumes early in a shot.
void test_pressure_ceiling_soft_system() {
    Rig rig;
    rig.compliance = 6.0f;
    rig.puckConductance = 0.10f;
    rig.rawFlowSetpoint = 2.5f;
    rig.rawPressureSetpoint = 8.0f;
    rig.run(40.0f);
    TEST_ASSERT_TRUE(rig.maxPressure < 8.8f);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 8.0f, rig.pressure);
}

// Puck opens up after the ceiling held: flow returns to the target and pressure drops off the ceiling.
void test_releases_ceiling_when_puck_opens() {
    Rig rig;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(10.0f);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
    rig.puckConductance = 0.6f; // channel / softening: 2 ml/s now passes at ~3.3 bar
    rig.run(8.0f);
    TEST_ASSERT_FALSE(rig.controller.isPressureLimiting());
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 2.0f, rig.trueFlow);
    TEST_ASSERT_TRUE(rig.pressure < 5.0f);
}

// No ceiling (pressure setpoint 0) means no limiter at all.
void test_no_ceiling_without_pressure_setpoint() {
    Rig rig;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 0.0f;
    rig.run(16.0f);
    TEST_ASSERT_FALSE(rig.controller.isPressureLimiting());
    TEST_ASSERT_TRUE_MESSAGE(rig.pressure > 9.5f, "without a ceiling the loop keeps pushing past 9 bar (real OPV territory)");
}

// Fail-safe: stale meter data stops the pump at once; fresh data resumes the phase.
void test_fail_safe_without_measurement() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(3.0f);
    TEST_ASSERT_TRUE(rig.controllerOutput > 10.0f);
    rig.meterValid = false;
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
    rig.run(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
    rig.meterValid = true;
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 2.0f, rig.trueFlow);
}

// A phase asking for no flow (bloom) gives zero duty and leaves the loop reset.
void test_zero_target_is_zero_duty() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(2.0f);
    rig.rawFlowSetpoint = 0.0f;
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controller.getFlowLoopIntegral());
}

// The start duty pre-load shows up on the very first cycle (model-free head start).
void test_start_duty_preload() {
    Rig rig;
    FlowLoopGains g;
    g.startDuty = 0.25f;
    g.ff = 0.0f;
    rig.controller.setFlowLoopGains(g);
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.step();
    // first output = start duty + kp * (slewed setpoint ~ ramp*dt), i.e. essentially the start duty
    TEST_ASSERT_FLOAT_WITHIN(0.6f, 25.0f, rig.controllerOutput);
    rig.run(1.0f);
    TEST_ASSERT_TRUE(rig.controllerOutput > 25.0f); // and it climbs from there
}

// With feedforward enabled the first cycle uses the pump model; the loop still converges.
void test_feedforward_option() {
    Rig rig;
    FlowLoopGains g;
    g.ff = 1.0f;
    rig.controller.setFlowLoopGains(g);
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    g.ramp = 0.0f; // no slew, so the first cycle sees the full target
    rig.controller.setFlowLoopGains(g);
    rig.step();
    // model duty at 0 bar for 2 ml/s: (2 + slip(0)) / (Q100(0) + slip(0)) = 8.325 / 29.98 = 27.8 %
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 27.8f + g.kp * 2.0f * 100.0f, rig.controllerOutput);
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
}

// The flow target is slewed, so a step does not kick the pump: no overshoot beyond 10 %.
void test_setpoint_slew_limits_overshoot() {
    Rig rig;
    rig.rawFlowSetpoint = 1.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(3.0f);
    rig.maxFlow = 0.0f;
    rig.rawFlowSetpoint = 3.0f;
    rig.run(3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 3.0f, rig.trueFlow);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxFlow < 3.3f, "overshoot on a 1 -> 3 ml/s step");
    TEST_ASSERT_TRUE(rig.controller.getRampedFlowSetpoint() > 2.99f);
}

// Pure PRESSURE mode is untouched by the fork: identical output with the flow loop enabled or not.
void test_pressure_mode_unchanged() {
    Rig withLink;
    Rig stock;
    stock.controller.enableMeasuredFlowControl(false);
    for (Rig *r : {&withLink, &stock}) {
        r->puckConductance = 0.15f;
        r->rawFlowSetpoint = 0.0f;
        r->rawPressureSetpoint = 9.0f;
    }
    for (int i = 0; i < 60 * 8; i++) {
        withLink.step(PressureController::ControlMode::PRESSURE);
        stock.step(PressureController::ControlMode::PRESSURE);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, stock.controllerOutput, withLink.controllerOutput);
    }
}

// Telemetry: with a live meter the exported pump flow follows the measurement, not the model.
void test_exported_flow_is_measured() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, rig.meterReading, rig.controller.getPumpFlowRate());
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_open_group_tracks_target);
    RUN_TEST(test_follows_setpoint_change);
    RUN_TEST(test_pressure_ceiling_is_held);
    RUN_TEST(test_pressure_ceiling_soft_system);
    RUN_TEST(test_releases_ceiling_when_puck_opens);
    RUN_TEST(test_no_ceiling_without_pressure_setpoint);
    RUN_TEST(test_fail_safe_without_measurement);
    RUN_TEST(test_zero_target_is_zero_duty);
    RUN_TEST(test_start_duty_preload);
    RUN_TEST(test_feedforward_option);
    RUN_TEST(test_setpoint_slew_limits_overshoot);
    RUN_TEST(test_pressure_mode_unchanged);
    RUN_TEST(test_exported_flow_is_measured);
    return UNITY_END();
}
