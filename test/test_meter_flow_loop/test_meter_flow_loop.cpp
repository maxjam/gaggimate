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
    bool meterStuckAtZero = false; // meter keeps sending fresh lines but reads 0 (pulse wire off)
    bool sqrtPuck = false;         // puck flow ~ sqrt(P) instead of linear
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
        const float puckFlow = sqrtPuck ? puckConductance * 3.0f * std::sqrt(std::max(0.0f, pressure)) : puckConductance * pressure;
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
        controller.setMeasuredFlow(meterStuckAtZero ? 0.0f : meterReading, meterValid);
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
    TEST_ASSERT_TRUE(rig.controller.getMeasuredFlow() < 1.5f); // the ceiling, not the 2 ml/s target, sets the flow
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
    rig.run(16.0f);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
    rig.puckConductance = 0.6f; // channel / softening: 2 ml/s now passes at ~3.3 bar
    rig.run(8.0f);
    TEST_ASSERT_FALSE(rig.controller.isPressureLimiting());
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 2.0f, rig.trueFlow);
    TEST_ASSERT_TRUE(rig.pressure < 5.0f);
}

// No ceiling (pressure setpoint 0) means no limiter at all.
// A flow phase whose profile gives no pressure gets the default ceiling (10 bar, meterlink.7): a target the puck
// cannot pass ends in a held 10 bar, not in the pump at 100 %.
void test_default_ceiling_without_pressure_setpoint() {
    Rig rig;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 0.0f;
    rig.run(20.0f);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 10.0f, rig.pressure);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxPressure < 11.0f, "default ceiling overshoot");
}

// The default ceiling must not touch a flush on an open group (15 ml/s at a few bar).
void test_default_ceiling_idle_on_flush() {
    Rig rig;
    rig.puckConductance = 4.0f; // ~3.75 bar at 15 ml/s
    rig.rawFlowSetpoint = 15.0f;
    rig.rawPressureSetpoint = 0.0f;
    rig.run(10.0f);
    TEST_ASSERT_FALSE(rig.controller.isPressureLimiting());
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 15.0f, rig.trueFlow);
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


// Far below the ceiling the limiter must not touch the target: flow stays at target while pressure climbs.
void test_limiter_idle_far_from_ceiling() {
    Rig rig;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(3.0f); // pressure is a few bar by now, far from 9
    TEST_ASSERT_TRUE(rig.pressure > 1.0f && rig.pressure < 7.5f);
    TEST_ASSERT_FALSE(rig.controller.isPressureLimiting());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, rig.controller.getEffectiveFlowTarget());
    // the inner loop trails the target a little while pressure (and so slip) keeps rising; no limiter involved
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 2.0f, rig.trueFlow);
}

// Ceiling lowered mid-shot (9 -> 3 bar): the pump must stop delivering at once, the puck drains the
// pressure down to the new ceiling, and it lands there without undershooting. On a vane pump "stop
// delivering" is any duty below the slip hold duty, so this checks the pump's flow rather than its duty;
// the undershoot check catches an integral parked deep in the slip band (what dropped shot 292).
void test_ceiling_lowered_mid_shot() {
    Rig rig;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(16.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 9.0f, rig.pressure);
    rig.rawPressureSetpoint = 3.0f;
    rig.run(0.5f);
    TEST_ASSERT_TRUE_MESSAGE(rig.trueFlow < 0.05f, "pump should deliver nothing right after the ceiling drops");
    // the puck drains the system (tau = C/g = 11.7 s here); the pump adds next to nothing
    float pPrev = rig.pressure;
    for (int i = 0; i < 8; i++) {
        rig.run(1.0f);
        TEST_ASSERT_TRUE(rig.trueFlow < 0.25f);
        TEST_ASSERT_TRUE(rig.pressure < pPrev);
        pPrev = rig.pressure;
    }
    TEST_ASSERT_TRUE(rig.pressure < 5.0f);
    float lo = 99.0f;
    for (int i = 0; i < 60 * 30; i++) {
        rig.step();
        lo = std::min(lo, rig.pressure);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "undershoot to %.2f bar under a 3 bar ceiling", lo);
    TEST_ASSERT_TRUE_MESSAGE(lo > 2.8f, msg);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 3.0f, rig.pressure);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
}

// Meter stuck at 0 while lines keep coming: the flow PI drives the pump up, the ceiling PI still holds
// pressure. On the soft system (C = 1.4 ml/bar) with the default gains this costs one overshoot of about
// 1.5-2 bar, still well under the 12.5 bar hard cutoff, bled off by the ceiling PI's integral (lim ki) over
// ~20 s; on a stiff system there is none.
void test_stuck_meter_backstop() {
    for (float c : {1.4f, 0.25f}) {
        Rig rig;
        rig.compliance = c;
        rig.puckConductance = 0.05f;
        rig.meterStuckAtZero = true;
        rig.rawFlowSetpoint = 2.0f;
        rig.rawPressureSetpoint = 9.0f;
        rig.run(25.0f);
        char msg[64];
        snprintf(msg, sizeof(msg), "C %.2f: peak %.2f bar with a stuck meter", c, rig.maxPressure);
        TEST_ASSERT_TRUE_MESSAGE(rig.maxPressure < (c > 1.0f ? 11.0f : 9.3f), msg);
        TEST_ASSERT_FLOAT_WITHIN(0.3f, 9.0f, rig.pressure);
    }
}

// Choked puck (almost nothing passes): pressure must park at the ceiling, no runaway, no stall.
void test_choked_puck() {
    Rig rig;
    rig.puckConductance = 0.01f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(15.0f);
    TEST_ASSERT_TRUE(rig.maxPressure < 9.6f);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 9.0f, rig.pressure);
    TEST_ASSERT_TRUE(rig.trueFlow < 0.3f);
}

// Flow phase starting with the system already above its ceiling: no pumping until pressure is back under it.
void test_start_above_ceiling() {
    Rig rig;
    rig.puckConductance = 0.05f;
    rig.pressure = 6.0f;
    rig.rawPressure = 6.0f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 3.0f;
    for (int i = 0; i < 13; i++) {
        rig.run(0.5f);
        TEST_ASSERT_TRUE(rig.trueFlow < 0.05f); // duty below the slip hold duty: the pump delivers nothing
    }
    TEST_ASSERT_TRUE(rig.pressure < 6.0f * std::exp(-6.5f / 28.0f) + 0.2f); // draining through the puck alone
}

// Shot 292: a pressure phase holds 9 bar, then a 2 ml/s flow phase with a 9 bar ceiling follows on a
// puck that passes less than 2 ml/s at 9 bar. The old limiter let pressure fall to 5-6 bar here, because
// its backstop drove the flow integrator far below the duty that only covers the pump's slip. With the
// override PI the hand-over is bumpless and the pressure stays on the ceiling.
void test_flow_phase_after_pressure_phase_holds_ceiling() {
    Rig rig;
    rig.compliance = 0.25f; // purged, stiff system
    rig.sqrtPuck = true;
    rig.puckConductance = 0.15f; // 1.35 ml/s at 9 bar
    // the pressure phase has been holding 9 bar: its duty covers slip plus the puck (61.8 %)
    rig.pressure = rig.rawPressure = 9.0f;
    rig.controllerOutput = 100.0f * (slip(9.0f) + 0.15f * 3.0f * 3.0f) / (fullDriveFlow(9.0f) + slip(9.0f));
    rig.dutyEff = rig.controllerOutput / 100.0f;
    rig.meterReading = 0.15f * 3.0f * 3.0f;
    rig.controller.initSetpointFilter(9.0f);
    rig.rawPressureSetpoint = 9.0f;
    rig.run(1.0f, PressureController::ControlMode::POWER); // settle the pressure filter, pump held as is
    rig.controllerOutput = 100.0f * rig.dutyEff;
    rig.rawFlowSetpoint = 2.0f;
    float lo = 99.0f, hi = 0.0f;
    for (int i = 0; i < 60 * 20; i++) {
        rig.step();
        lo = std::min(lo, rig.pressure);
        if (i > 60 * 10)
            hi = std::max(hi, rig.pressure);
    }
    char msg[80];
    snprintf(msg, sizeof(msg), "pressure fell to %.2f bar after the hand-over", lo);
    TEST_ASSERT_TRUE_MESSAGE(lo > 8.4f, msg);
    TEST_ASSERT_TRUE(hi < 9.2f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 9.0f, rig.pressure);
    TEST_ASSERT_TRUE(rig.controller.isPressureLimiting());
}

// FLOW -> POWER -> FLOW: the second phase re-arms (start duty pre-load, slew from the running flow).
void test_mode_round_trip_rearms() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(3.0f);
    TEST_ASSERT_TRUE(rig.controller.getFlowLoopIntegral() > 0.0f);
    rig.controller.resetFlowLoop(); // what DimmedPump::loop does on the FLOW -> POWER edge
    rig.controllerOutput = 0.0f;
    rig.run(2.0f, PressureController::ControlMode::POWER);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controller.getFlowLoopIntegral());
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 20.0f, rig.controllerOutput); // pre-load again
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 2.0f, rig.trueFlow);
}

// Puck in sqrt(P) (closer to a real puck) with the stiff system: same ceiling behaviour.
void test_sqrt_puck_ceiling() {
    Rig rig;
    rig.sqrtPuck = true;
    rig.puckConductance = 0.12f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(20.0f);
    TEST_ASSERT_TRUE(rig.maxPressure < 9.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 9.0f, rig.pressure);
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

// Pump dead zone: with gains.deadZone set, the loop never sends a pump power between 0 and the dead zone
// (the band where the Velofuso does not turn), and still tracks the target and holds a pressure ceiling.
void test_dead_zone_never_parks_the_pump_in_it() {
    for (float ceiling : {0.0f, 9.0f}) {
        Rig rig;
        FlowLoopGains g;
        g.deadZone = 0.25f;
        rig.controller.setFlowLoopGains(g);
        rig.puckConductance = ceiling > 0.0f ? 0.12f : 1e6f; // tight puck under a ceiling, else open group
        rig.rawFlowSetpoint = 2.0f;
        rig.rawPressureSetpoint = ceiling;
        for (int i = 0; i < 60 * 20; i++) {
            rig.step();
            char msg[80];
            snprintf(msg, sizeof(msg), "pump power %.2f %% inside the 25 %% dead zone", rig.controllerOutput);
            TEST_ASSERT_TRUE_MESSAGE(rig.controllerOutput == 0.0f || rig.controllerOutput >= 25.0f - 1e-3f, msg);
        }
        if (ceiling > 0.0f) {
            TEST_ASSERT_FLOAT_WITHIN(0.25f, 9.0f, rig.pressure);
        } else {
            TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
        }
    }
}

// A start duty below the dead zone starts at its edge, and the flow loop off (no target) is still 0 %.
// (The rig pump delivers nothing below 21 % at 0 bar, so a 20 % dead zone is a correct setting for it.)
void test_dead_zone_start_and_off() {
    Rig rig;
    FlowLoopGains g;
    g.startDuty = 0.10f;
    g.deadZone = 0.20f;
    rig.controller.setFlowLoopGains(g);
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.step();
    TEST_ASSERT_TRUE(rig.controllerOutput >= 20.0f && rig.controllerOutput < 22.0f);
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
    rig.rawFlowSetpoint = 0.0f;
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
}

// No stall (meterlink.4): with a dead zone set, a flow target below what the pump gives at its slowest speed runs the
// pump at that speed. Before, the loop switched the pump off and on around the target (the pulses of flush 301).
// The rig pump starts delivering at 21 % on an open group, so a 25 % dead zone gives ~1.2 ml/s at the slowest speed.
void test_dead_zone_no_stall_below_min_flow() {
    Rig rig;
    FlowLoopGains g;
    g.deadZone = 0.25f;
    rig.controller.setFlowLoopGains(g);
    rig.rawFlowSetpoint = 0.5f; // below the slowest-speed flow
    rig.rawPressureSetpoint = 0.0f;
    rig.run(1.0f);
    float lo = 1e9f, hi = -1e9f, fmin = 1e9f, fmax = -1e9f;
    for (int i = 0; i < 60 * 8; i++) {
        rig.step();
        lo = std::min(lo, rig.controllerOutput);
        hi = std::max(hi, rig.controllerOutput);
        fmin = std::min(fmin, rig.trueFlow);
        fmax = std::max(fmax, rig.trueFlow);
    }
    char msg[96];
    snprintf(msg, sizeof(msg), "pump power %.2f..%.2f %%, flow %.2f..%.2f ml/s", lo, hi, fmin, fmax);
    TEST_ASSERT_TRUE_MESSAGE(lo >= 25.0f - 1e-3f, msg);          // never off
    TEST_ASSERT_TRUE_MESSAGE(hi <= 25.0f + 0.5f, msg);           // parked at the slowest speed
    TEST_ASSERT_TRUE_MESSAGE(fmax - fmin < 0.05f, msg);          // steady flow, no pulses
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, 1.17f, rig.trueFlow, msg);
    // A target the pump can reach is tracked as before.
    rig.rawFlowSetpoint = 2.0f;
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
    // Zero target still means off.
    rig.rawFlowSetpoint = 0.0f;
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
}

// The pressure ceiling may still stop the pump when even its slowest speed pushes the pressure past the ceiling
// (here 25 % holds ~0.8 bar against the puck, the ceiling is 0.5 bar).
void test_dead_zone_ceiling_still_stops_the_pump() {
    Rig rig;
    FlowLoopGains g;
    g.deadZone = 0.25f;
    rig.controller.setFlowLoopGains(g);
    rig.puckConductance = 0.1f;
    rig.rawFlowSetpoint = 0.5f;
    rig.rawPressureSetpoint = 0.5f;
    rig.run(3.0f);
    bool sawOff = false;
    float pmax = 0.0f;
    for (int i = 0; i < 60 * 10; i++) {
        rig.step();
        if (rig.controllerOutput == 0.0f)
            sawOff = true;
        TEST_ASSERT_TRUE(rig.controllerOutput == 0.0f || rig.controllerOutput >= 25.0f - 1e-3f);
        pmax = std::max(pmax, rig.pressure);
    }
    TEST_ASSERT_TRUE_MESSAGE(sawOff, "the ceiling never stopped the pump");
    char msg[64];
    snprintf(msg, sizeof(msg), "max pressure %.2f bar over a 0.5 bar ceiling", pmax);
    TEST_ASSERT_TRUE_MESSAGE(pmax < 0.75f, msg);
}

// The hard cutoff stops the pump with a dead zone too, and stays off for the rest of the phase (meterlink.7).
// A ceiling above the cutoff (14 bar) and a closed outlet get the pressure there.
void test_dead_zone_hard_cutoff() {
    Rig rig;
    FlowLoopGains g;
    g.deadZone = 0.25f;
    rig.controller.setFlowLoopGains(g);
    rig.puckConductance = 0.0f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 14.0f;
    bool sawOff = false;
    int transitions = 0;
    bool lastOff = false;
    for (int i = 0; i < 60 * 30; i++) {
        rig.step();
        const bool off = rig.controllerOutput == 0.0f;
        if (rig.controller.getFilteredPressure() > 12.5f && off)
            sawOff = true;
        if (off != lastOff)
            transitions++;
        lastOff = off;
    }
    TEST_ASSERT_TRUE_MESSAGE(sawOff, "pump not stopped past the hard cutoff");
    char msg[64];
    snprintf(msg, sizeof(msg), "max pressure %.2f bar", rig.maxPressure);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxPressure < 13.5f, msg);
    TEST_ASSERT_TRUE(rig.controller.isCutoffLatched());
    TEST_ASSERT_EQUAL_MESSAGE(1, transitions, "the cutoff must latch, not cycle");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
    // the next phase (zero flow resets the loop) runs again
    rig.rawFlowSetpoint = 0.0f;
    rig.step();
    TEST_ASSERT_FALSE(rig.controller.isCutoffLatched());
    rig.pressure = rig.rawPressure = 5.0f;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(1.0f);
    TEST_ASSERT_TRUE(rig.controllerOutput > 0.0f);
}

// Re-arming under pressure (meter data back after a dropout, or a flow phase after a zero-flow phase at 9 bar)
// must not hand the pump to the ceiling PI while the pressure is well under the ceiling: meterlink.6 sagged to
// 5.5 bar and never recovered within 12 s in this scenario.
void test_rearm_under_pressure_recovers() {
    for (int scenario = 0; scenario < 2; scenario++) {
        Rig rig;
        rig.puckConductance = 0.12f;
        rig.rawFlowSetpoint = 2.0f;
        rig.rawPressureSetpoint = 9.0f;
        rig.run(16.0f);
        if (scenario == 0) { // 1 s without meter data
            rig.meterValid = false;
            rig.run(1.0f);
            rig.meterValid = true;
        } else { // 2 s zero-flow phase
            rig.rawFlowSetpoint = 0.0f;
            rig.run(2.0f);
            rig.rawFlowSetpoint = 2.0f;
        }
        const float pStart = rig.pressure;
        float pMin = 99.0f, backAt = -1.0f;
        for (int i = 0; i < 60 * 8; i++) {
            rig.step();
            pMin = std::min(pMin, rig.pressure);
            if (backAt < 0.0f && rig.pressure > 8.7f)
                backAt = i / 60.0f;
        }
        char msg[96];
        snprintf(msg, sizeof(msg), "scenario %d: restart at %.2f bar, min %.2f bar, back over 8.7 bar after %.1f s", scenario, pStart,
                 pMin, backAt);
        TEST_ASSERT_TRUE_MESSAGE(pMin > pStart - 1.0f, msg);
        TEST_ASSERT_TRUE_MESSAGE(backAt >= 0.0f && backAt < 5.0f, msg);
        TEST_ASSERT_TRUE_MESSAGE(rig.maxPressure < 9.5f, msg);
    }
}

// Gains that arrive during a flow phase are applied at the next reset, not mid-phase (meterlink.7): a dead-zone
// change re-maps the loop output and would otherwise jump the pump.
void test_gains_deferred_during_phase() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(4.0f);
    const float before = rig.controllerOutput;
    FlowLoopGains g;
    g.deadZone = 0.25f;
    rig.controller.setFlowLoopGains(g);
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, before, rig.controllerOutput);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, rig.controller.getFlowLoopGains().deadZone); // reported as requested
    rig.rawFlowSetpoint = 0.0f; // next phase resets the loop and takes the gains
    rig.step();
    rig.rawFlowSetpoint = 2.0f;
    rig.step();
    TEST_ASSERT_TRUE(rig.controllerOutput >= 25.0f); // dead zone 25 % in force
    // outside a phase they apply at once
    FlowLoopGains g2;
    g2.deadZone = 0.10f;
    rig.rawFlowSetpoint = 0.0f;
    rig.step();
    rig.controller.setFlowLoopGains(g2);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.10f, rig.controller.getFlowLoopGains().deadZone);
}

// Max's machine idles at 12.5 bar (trapped water expanding against the relief valve). A flush or a shot started
// from that state reads the cutoff pressure for the first few hundred ms, until the open valve has vented it. The
// cutoff must hold the pump off meanwhile but not latch: once the pressure is down the flush has to run.
void test_trapped_idle_pressure_does_not_latch_cutoff() {
    Rig rig;
    rig.puckConductance = 4.0f; // open group: ~3.75 bar at 15 ml/s
    rig.pressure = rig.rawPressure = 12.6f;
    rig.rawFlowSetpoint = 15.0f;
    rig.rawPressureSetpoint = 0.0f; // a flush profile: no ceiling of its own
    // the simulated rig vents the trapped pressure through the "puck" within a few cycles
    rig.step();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput); // held off while over the cutoff
    rig.run(6.0f);
    TEST_ASSERT_FALSE(rig.controller.isCutoffLatched());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, rig.trueFlow); // the flush runs
    // and once the phase has been below 12 bar, a pump-driven rise past 12.5 does latch
    rig.puckConductance = 0.0f; // outlet sealed
    rig.rawPressureSetpoint = 14.0f;
    rig.run(20.0f);
    TEST_ASSERT_TRUE(rig.controller.isCutoffLatched());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controllerOutput);
}

// A cold start with the pressure already past the ceiling does not pump until it is back under it.
void test_cold_start_above_ceiling_waits() {
    Rig rig;
    rig.puckConductance = 0.02f;
    rig.pressure = rig.rawPressure = 3.0f;
    rig.rawFlowSetpoint = 0.4f;
    rig.rawPressureSetpoint = 2.0f;
    float maxDuty = 0.0f;
    for (int i = 0; i < 60 * 10; i++) {
        rig.step();
        if (rig.pressure > 2.3f)
            maxDuty = std::max(maxDuty, rig.controllerOutput);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, maxDuty);
}

// Diagnostics getters used by the $D line.
void test_diagnostics_getters() {
    Rig rig;
    rig.rawFlowSetpoint = 2.0f;
    rig.rawPressureSetpoint = 9.0f;
    rig.run(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, rig.controller.getEffectiveFlowTarget());
    // model flow at the running power and pressure matches the rig's pump (same coefficients)
    TEST_ASSERT_FLOAT_WITHIN(0.15f, pumpFlow(rig.controllerOutput / 100.0f, rig.controller.getFilteredPressure()),
                             rig.controller.getModelPumpFlow(rig.controllerOutput));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rig.controller.getModelPumpFlow(0.0f));
    // stock pressure loop: the integral term is reported while it holds a pressure
    Rig p;
    p.puckConductance = 0.3f;
    p.rawPressureSetpoint = 6.0f;
    p.rawFlowSetpoint = 0.0f;
    p.run(10.0f, PressureController::ControlMode::PRESSURE);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 6.0f, p.controller.getFilteredSetpoint());
    TEST_ASSERT_TRUE(p.controller.getPressureITerm() > 0.0f);
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
    RUN_TEST(test_default_ceiling_without_pressure_setpoint);
    RUN_TEST(test_default_ceiling_idle_on_flush);
    RUN_TEST(test_fail_safe_without_measurement);
    RUN_TEST(test_zero_target_is_zero_duty);
    RUN_TEST(test_start_duty_preload);
    RUN_TEST(test_feedforward_option);
    RUN_TEST(test_setpoint_slew_limits_overshoot);
    RUN_TEST(test_limiter_idle_far_from_ceiling);
    RUN_TEST(test_ceiling_lowered_mid_shot);
    RUN_TEST(test_stuck_meter_backstop);
    RUN_TEST(test_choked_puck);
    RUN_TEST(test_start_above_ceiling);
    RUN_TEST(test_flow_phase_after_pressure_phase_holds_ceiling);
    RUN_TEST(test_mode_round_trip_rearms);
    RUN_TEST(test_sqrt_puck_ceiling);
    RUN_TEST(test_pressure_mode_unchanged);
    RUN_TEST(test_exported_flow_is_measured);
    RUN_TEST(test_dead_zone_never_parks_the_pump_in_it);
    RUN_TEST(test_dead_zone_start_and_off);
    RUN_TEST(test_dead_zone_no_stall_below_min_flow);
    RUN_TEST(test_dead_zone_ceiling_still_stops_the_pump);
    RUN_TEST(test_dead_zone_hard_cutoff);
    RUN_TEST(test_rearm_under_pressure_recovers);
    RUN_TEST(test_gains_deferred_during_phase);
    RUN_TEST(test_cold_start_above_ceiling_waits);
    RUN_TEST(test_trapped_idle_pressure_does_not_latch_cutoff);
    RUN_TEST(test_diagnostics_getters);
    return UNITY_END();
}
