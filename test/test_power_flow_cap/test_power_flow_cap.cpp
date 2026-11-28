// Unit tests: flow cap on power phases in PressureController (fork, maxa4turbo.18).
// A power phase with "maxFlow" runs the pump at the profile's power until the measured flow passes the cap; then
// the flow PI of the measured-flow loop holds the cap: power = min(ramp power, PI).
//
// The plant is Max's machine as his shots 567-570 show it (Nov 28 2026, CAN speed control, 302 quasi-steady
// samples, rms 0.057 ml/s): pump flow = 0.2646 * power% - 0.7879 * bar - 9.080 ml/s for 55-67 % and 5.9-9 bar,
// puck flow = sqrt(bar) / resistance (GaggiMate's puck resistance: 1.5-2.4 on the old beans, 1.0 in shot 570).
// The meter is the meter board's link value: mean flow over the last 150 ms, sent 20 times a second. The display
// sends a new ramp power 10 times a second. Host-side: pio test -e native -f test_power_flow_cap.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <vector>

#define ESP_LOGI(tag, fmt, ...) ((void)0)

#include "PressureController/PressureController.cpp"
#include "SimpleKalmanFilter/SimpleKalmanFilter.cpp"

namespace {

constexpr float kPumpPerPct = 0.2646f, kPumpPerBar = 0.7879f, kPumpOffset = 9.080f;
float pumpFlow(float powerPct, float bar) { return std::max(0.0f, kPumpPerPct * powerPct - kPumpPerBar * bar - kPumpOffset); }

// Max's flow-loop gains (meter page, Nov 27 2026). ff = 1 is what his flow phases run with; the cap must not use it.
FlowLoopGains maxGains() {
    FlowLoopGains g;
    g.kp = 0.025f;
    g.ki = 0.075f;
    g.startDuty = 0.20f;
    g.ff = 1.0f;
    g.ramp = 4.0f;
    g.limKp = 3.0f;
    g.limKi = 1.5f;
    g.deadZone = 0.10f;
    // setupMeterLink() on the Pro: half gain below 25 % pump power (the meter board's $G leaves these two alone)
    g.knee = 0.25f;
    g.kneeGain = 0.5f;
    return g;
}

struct Rig {
    static constexpr float kDt = 1.0f / 60.0f; // DimmedPump control period on the Pro

    float rawPressureSetpoint = 0.0f;
    float rawFlowSetpoint = 0.0f;
    float rawPressure = 0.0f;
    float controllerOutput = 0.0f; // what DimmedPump keeps in _controllerPower
    int valveStatus = 1;
    PressureController controller;

    // profile side
    std::function<float(float)> rampAt = [](float) { return 66.0f; }; // power the display asks for at time t
    std::function<float(float)> capAt = [](float) { return 2.0f; };   // the phase's maxFlow at time t
    // plant
    std::function<float(float)> resistanceAt = [](float) { return 1.0f; };
    float pumpLag = 0.15f; // s, the VESC bringing the pump to a new speed
    float powerEff = 0.0f; // % the pump actually turns at
    float pressure = 0.0f; // bar
    float trueFlow = 0.0f; // ml/s out of the pump
    float meterReading = 0.0f;
    bool meterValid = true;
    float meterStuckAt = -1.0f;                    // >= 0: the meter reads this whatever the pump does
    std::deque<std::pair<float, float>> volumeLog; // (t, ml) for the meter's 150 ms window
    float volume = 0.0f;
    float nextMeter = 0.0f, nextDisplay = 0.0f;

    // pump driver side (what DimmedPump::updatePower does in POWER mode)
    float ramp = 0.0f;
    float power = 0.0f; // power the pump gets
    float t = 0.0f;

    // bookkeeping
    bool wasLimiting = false;
    float engagedAt = -1.0f, releasedAt = -1.0f;
    float crossedAt = -1.0f;      // first time the meter read more than the cap
    float stepAtEngage = 0.0f;    // |power change| in the cycle the cap took the pump
    float stepAtRelease = 0.0f;   // ... and in the cycle it let go
    float maxStepLimiting = 0.0f; // largest |power change| in one cycle while the cap holds the pump
    float maxFlowSeen = 0.0f;
    float waterOverCap = 0.0f; // ml pumped above the cap
    bool everAboveRamp = false;
    bool everDiffersFromRamp = false;
    float minPower = 1000.0f;
    std::vector<float> powerTrace;

    Rig() : controller(kDt, &rawPressureSetpoint, &rawFlowSetpoint, &rawPressure, &controllerOutput, &valveStatus) {
        controller.enableMeasuredFlowControl(true);
        controller.setFlowLoopGains(maxGains());
    }

    // Steady state at this power and puck: where a shot stands when the power phase takes over.
    void settleAt(float powerPct, float resistance) {
        powerEff = power = controllerOutput = powerPct;
        float p = 6.0f;
        for (int i = 0; i < 200; i++) { // bisection-free fixed point: pump flow falls and puck flow rises with pressure
            const float f = pumpFlow(powerPct, p) - std::sqrt(p) / resistance;
            p = std::max(0.05f, p + 0.2f * f);
        }
        pressure = rawPressure = p;
        trueFlow = meterReading = pumpFlow(powerPct, p);
    }

    void step() {
        // plant
        powerEff += (power - powerEff) * std::min(1.0f, kDt / pumpLag);
        trueFlow = pumpFlow(powerEff, pressure);
        const float puckFlow = std::sqrt(std::max(0.0f, pressure)) / resistanceAt(t);
        pressure = std::max(0.0f, pressure + (trueFlow - puckFlow) / (3.5f / std::max(pressure, 0.5f)) * kDt);
        rawPressure = pressure;
        // meter board: flow over the last 150 ms, a new value every 50 ms
        volume += trueFlow * kDt;
        volumeLog.emplace_back(t, volume);
        while (!volumeLog.empty() && volumeLog.front().first < t - 0.15f - 1e-6f)
            volumeLog.pop_front();
        if (t >= nextMeter - 1e-6f) {
            if (volumeLog.size() > 1)
                meterReading =
                    (volumeLog.back().second - volumeLog.front().second) / (volumeLog.back().first - volumeLog.front().first);
            nextMeter += 0.05f;
        }
        const float seen = meterStuckAt >= 0.0f ? meterStuckAt : meterReading;
        // display: a new ramp value every 100 ms
        if (t >= nextDisplay - 1e-6f) {
            ramp = rampAt(t);
            nextDisplay += 0.1f;
        }
        const float cap = capAt(t);

        // controller, as DimmedPump::loop() / updatePower() run it in POWER mode
        controller.setMeasuredFlow(seen, meterValid);
        controller.update(PressureController::ControlMode::POWER, true, kDt, kDt);
        const float before = power;
        if (cap > 0.0f) {
            power = controller.limitPowerToFlow(ramp, power, cap);
        } else {
            controller.resetPowerCap();
            power = ramp;
        }
        controllerOutput = power;

        // bookkeeping
        const bool limiting = controller.isPowerCapLimiting();
        const float change = std::fabs(power - before);
        if (cap > 0.0f && meterValid && seen > cap && crossedAt < 0.0f)
            crossedAt = t;
        if (limiting && !wasLimiting) {
            if (engagedAt < 0.0f)
                engagedAt = t;
            stepAtEngage = std::max(stepAtEngage, change);
        } else if (!limiting && wasLimiting) {
            releasedAt = t;
            stepAtRelease = std::max(stepAtRelease, change);
        } else if (limiting) {
            maxStepLimiting = std::max(maxStepLimiting, change);
        }
        wasLimiting = limiting;
        if (power > ramp + 1e-4f)
            everAboveRamp = true;
        if (power != ramp)
            everDiffersFromRamp = true;
        minPower = std::min(minPower, power);
        maxFlowSeen = std::max(maxFlowSeen, trueFlow);
        if (cap > 0.0f)
            waterOverCap += std::max(0.0f, trueFlow - cap) * kDt;
        powerTrace.push_back(power);
        t += kDt;
    }

    void run(float seconds) {
        const int n = static_cast<int>(seconds / kDt + 0.5f);
        for (int i = 0; i < n; i++)
            step();
    }
};

// His profile's two power phases: "Hold power 66" for 1 s, then "Decline power 66 to 60" over 25 s.
float holdAndDecline(float t) { return t < 1.0f ? 66.0f : std::max(60.0f, 66.0f - 6.0f * (t - 1.0f) / 25.0f); }
// Shot 570: the puck's resistance falls from about 1.4 (end of the rise, 60 % and 6.4 bar) to 1.0 within seconds.
float puck570(float t) { return std::max(1.0f, 1.4f - 0.4f * t / 3.0f); }
// Shots 568 / 569: resistance 1.56 at the start of the decline, rising to 2.4.
float puck569(float t) { return std::min(2.4f, 1.565f + 0.835f * t / 25.0f); }

char msg[160];

} // namespace

// No cap in the profile: the pump gets the ramp power, bit for bit, and the cap code keeps no state.
void test_no_cap_is_the_ramp() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = holdAndDecline;
    rig.resistanceAt = puck570;
    rig.capAt = [](float) { return 0.0f; };
    rig.run(26.0f);
    TEST_ASSERT_FALSE(rig.everDiffersFromRamp);
    TEST_ASSERT_TRUE(rig.engagedAt < 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.controller.getPowerCapIntegral());
    snprintf(msg, sizeof(msg), "shot 570 without a cap should run well over 2 ml/s, peaked at %.2f", rig.maxFlowSeen);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxFlowSeen > 2.4f, msg);
}

// Shots 568 / 569 (old beans): the flow never reaches 2.0 ml/s, so a 2.0 cap changes nothing at all.
void test_flow_under_the_cap_follows_the_ramp() {
    Rig rig;
    rig.settleAt(66.0f, 1.565f);
    rig.rampAt = holdAndDecline;
    rig.resistanceAt = puck569;
    rig.run(26.0f);
    snprintf(msg, sizeof(msg), "flow peaked at %.2f ml/s", rig.maxFlowSeen);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxFlowSeen < 2.0f, msg);
    TEST_ASSERT_FALSE_MESSAGE(rig.everDiffersFromRamp, "power left the ramp although the flow stayed under the cap");
    TEST_ASSERT_TRUE(rig.engagedAt < 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.controller.getPowerCapIntegral());
}

// Shot 570 (new beans, easy puck) with a 2.0 cap: the cap takes the pump and brings the flow to 2.0.
void test_cap_holds_the_flow_on_an_easy_puck() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = holdAndDecline;
    rig.resistanceAt = puck570;
    rig.run(12.0f);
    snprintf(msg, sizeof(msg), "cap engaged at %.2f s", rig.engagedAt);
    TEST_ASSERT_TRUE_MESSAGE(rig.engagedAt >= 0.0f && rig.engagedAt < 1.5f, msg);
    snprintf(msg, sizeof(msg), "flow %.2f ml/s 12 s into the power phases", rig.trueFlow);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.10f, 2.0f, rig.trueFlow, msg);
    rig.run(10.0f);
    snprintf(msg, sizeof(msg), "flow %.2f ml/s after 22 s", rig.trueFlow);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 2.0f, rig.trueFlow, msg);
    TEST_ASSERT_TRUE(rig.controller.isPowerCapLimiting());
    snprintf(msg, sizeof(msg), "power %.1f %% against a ramp at %.1f %%", rig.power, rig.ramp);
    TEST_ASSERT_TRUE_MESSAGE(rig.power < rig.ramp - 3.0f, msg);
    TEST_ASSERT_FALSE_MESSAGE(rig.everAboveRamp, "the cap may only lower the power");
    snprintf(msg, sizeof(msg), "%.1f ml went through above the cap", rig.waterOverCap);
    TEST_ASSERT_TRUE_MESSAGE(rig.waterOverCap < 3.0f, msg);
    // No hunting once settled: the power moves less than 0.5 % in the last 3 s
    const size_t n = rig.powerTrace.size();
    const auto mm = std::minmax_element(rig.powerTrace.begin() + (n - 180), rig.powerTrace.end());
    snprintf(msg, sizeof(msg), "power still moving %.2f %% in the last 3 s", *mm.second - *mm.first);
    TEST_ASSERT_TRUE_MESSAGE(*mm.second - *mm.first < 0.5f, msg);
}

// Taking the pump and holding it: no step in the pump power.
void test_cap_takes_the_pump_without_a_step() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = holdAndDecline;
    rig.resistanceAt = puck570;
    rig.run(22.0f);
    TEST_ASSERT_TRUE(rig.engagedAt >= 0.0f);
    snprintf(msg, sizeof(msg), "power stepped %.3f %% when the cap took over", rig.stepAtEngage);
    TEST_ASSERT_TRUE_MESSAGE(rig.stepAtEngage < 0.1f, msg);
    // While it holds, the power moves with the flow error and nothing else. The largest move in one 17 ms cycle is
    // right after the profile's own step from 60 to 66 %, when the meter reading climbs 0.2 ml/s between two of
    // its values: kp * 0.2 ml/s = 0.5 % of power.
    snprintf(msg, sizeof(msg), "largest power change in one cycle while holding: %.3f %%", rig.maxStepLimiting);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxStepLimiting < 1.0f, msg);
    // the cap engages in the control cycle in which the meter first reads more than the cap
    TEST_ASSERT_FLOAT_WITHIN(Rig::kDt * 1.5f, rig.crossedAt, rig.engagedAt);
}

// The puck tightens while the cap holds: the flow drops under the cap, the loop brings the power back up to the
// ramp without a step, and from there the ramp runs the pump again.
void test_returns_to_the_ramp_without_a_step() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.resistanceAt = [](float t) { return t < 12.0f ? puck570(t) : std::min(2.0f, 1.0f + (t - 12.0f) / 3.0f); };
    rig.run(12.0f);
    TEST_ASSERT_TRUE(rig.controller.isPowerCapLimiting());
    const float capped = rig.power;
    TEST_ASSERT_TRUE(capped < 62.0f);
    rig.releasedAt = -1.0f;
    rig.maxStepLimiting = 0.0f; // from here on: the way back up
    rig.run(14.0f);
    snprintf(msg, sizeof(msg), "still holding at %.1f %% with the flow at %.2f ml/s", rig.power, rig.trueFlow);
    TEST_ASSERT_FALSE_MESSAGE(rig.controller.isPowerCapLimiting(), msg);
    TEST_ASSERT_TRUE(rig.releasedAt > 12.0f);
    TEST_ASSERT_EQUAL_FLOAT(66.0f, rig.power);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.controller.getPowerCapIntegral());
    snprintf(msg, sizeof(msg), "power stepped %.3f %% when the cap let go", rig.stepAtRelease);
    TEST_ASSERT_TRUE_MESSAGE(rig.stepAtRelease < 0.5f, msg);
    snprintf(msg, sizeof(msg), "largest power change in one cycle on the way back: %.3f %%", rig.maxStepLimiting);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxStepLimiting < 0.2f, msg);
    TEST_ASSERT_TRUE_MESSAGE(rig.trueFlow < 2.0f, "the flow is under the cap at the full ramp power");
    TEST_ASSERT_FALSE(rig.everAboveRamp);
}

// A long stretch under the cap leaves nothing behind: when the puck gives way later, the cap takes the pump in
// the cycle the flow passes it, from the ramp power.
void test_nothing_winds_up_under_the_cap() {
    Rig rig;
    rig.settleAt(66.0f, 2.2f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.resistanceAt = [](float t) { return t < 20.0f ? 2.2f : std::max(0.9f, 2.2f - 1.3f * (t - 20.0f)); };
    rig.run(20.0f);
    TEST_ASSERT_TRUE(rig.engagedAt < 0.0f);
    TEST_ASSERT_FALSE(rig.everDiffersFromRamp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.controller.getPowerCapIntegral());
    rig.run(10.0f);
    TEST_ASSERT_TRUE(rig.crossedAt > 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(Rig::kDt * 1.5f, rig.crossedAt, rig.engagedAt);
    TEST_ASSERT_TRUE(rig.stepAtEngage < 0.1f);
    snprintf(msg, sizeof(msg), "flow %.2f ml/s 10 s after the puck gave way", rig.trueFlow);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.15f, 2.0f, rig.trueFlow, msg);
}

// Under the cap a step of the ramp (next phase, instant transition) reaches the pump in the same control cycle.
void test_ramp_step_passes_at_once_under_the_cap() {
    Rig rig;
    rig.settleAt(55.0f, 2.2f);
    rig.rampAt = [](float t) { return t < 2.0f ? 55.0f : 62.0f; };
    rig.resistanceAt = [](float) { return 2.2f; };
    rig.run(1.95f);
    TEST_ASSERT_EQUAL_FLOAT(55.0f, rig.power);
    rig.run(0.15f); // the display's next update carries 62
    TEST_ASSERT_EQUAL_FLOAT(62.0f, rig.power);
    TEST_ASSERT_FALSE(rig.everDiffersFromRamp);
}

// A power phase that begins with the flow already over its cap (here 62 % running, 2.46 ml/s, ramp power 66 %):
// the pump does not step up to the ramp power first.
void test_phase_starting_over_the_cap_does_not_step_up() {
    Rig rig;
    rig.settleAt(62.0f, 1.0f);
    TEST_ASSERT_TRUE(rig.trueFlow > 2.3f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.step();
    TEST_ASSERT_TRUE(rig.controller.isPowerCapLimiting());
    snprintf(msg, sizeof(msg), "first power %.2f %%", rig.power);
    TEST_ASSERT_TRUE_MESSAGE(rig.power <= 62.0f && rig.power > 61.9f, msg);
    rig.run(15.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
    float peak = 0.0f;
    for (float p : rig.powerTrace)
        peak = std::max(peak, p);
    snprintf(msg, sizeof(msg), "power went up to %.2f %% on the way", peak);
    TEST_ASSERT_TRUE_MESSAGE(peak <= 62.0f, msg);
}

// The cap never stops the pump: with a meter that keeps reading over the cap the power comes down to the pump's
// slowest speed (the dead zone, 10 %) and stays there, and the integral does not run on below it.
void test_cap_never_stops_the_pump() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.meterStuckAt = 3.0f;
    rig.run(40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, rig.power);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, rig.minPower);
    // integral parked where the output is 0, not far below: kp * (2 - 3) + integral ~ 0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.025f, rig.controller.getPowerCapIntegral());
    rig.meterStuckAt = -1.0f; // the real reading is back (no flow at 10 %): the loop climbs out at once
    rig.run(0.5f);
    snprintf(msg, sizeof(msg), "power %.1f %% half a second after the reading came back", rig.power);
    TEST_ASSERT_TRUE_MESSAGE(rig.power > 12.0f, msg);
    rig.run(20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 2.0f, rig.trueFlow);
}

// Measured flow only: the pump model's coefficients and the feedforward weight make no difference to the cap.
void test_cap_uses_no_pump_model() {
    Rig a, b;
    FlowLoopGains g = maxGains();
    g.ff = 0.0f;
    b.controller.setFlowLoopGains(g);
    b.controller.setPumpFlowPolyCoeffs(0.0f, 0.122f, -2.7966f, 26.1908f);
    b.controller.setPumpSlipPolyCoeffs(0.0f, -0.122f, 2.7966f, 0.3062f);
    a.controller.setPumpFlowPolyCoeffs(0.0f, 0.0f, -0.5f, 5.0f);
    for (Rig *r : {&a, &b}) {
        r->settleAt(60.2f, 1.4f);
        r->rampAt = holdAndDecline;
        r->resistanceAt = puck570;
        r->run(20.0f);
    }
    TEST_ASSERT_TRUE(a.engagedAt >= 0.0f);
    TEST_ASSERT_EQUAL_UINT32(a.powerTrace.size(), b.powerTrace.size());
    for (size_t i = 0; i < a.powerTrace.size(); i++) {
        if (a.powerTrace[i] != b.powerTrace[i]) {
            snprintf(msg, sizeof(msg), "power differs in cycle %u: %.4f vs %.4f", static_cast<unsigned>(i), a.powerTrace[i],
                     b.powerTrace[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
}

// Without a fresh meter reading there is nothing to cap with: the ramp runs the pump. When the reading is back
// and over the cap, the cap takes over from the ramp power without a step.
void test_no_meter_reading_runs_the_ramp() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.resistanceAt = puck570;
    rig.meterValid = false;
    rig.run(6.0f);
    TEST_ASSERT_FALSE(rig.everDiffersFromRamp);
    TEST_ASSERT_FALSE(rig.controller.isPowerCapLimiting());
    TEST_ASSERT_TRUE(rig.trueFlow > 2.4f);
    rig.meterValid = true;
    rig.run(0.1f);
    TEST_ASSERT_TRUE(rig.controller.isPowerCapLimiting());
    TEST_ASSERT_TRUE(rig.stepAtEngage < 0.1f);
    rig.run(15.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 2.0f, rig.trueFlow);
}

// The next phase carries another cap while this one holds the pump: no step in the power, then the new cap is held.
void test_new_cap_value_while_holding() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = [](float) { return 66.0f; };
    rig.resistanceAt = puck570;
    rig.capAt = [](float t) { return t < 10.0f ? 2.0f : 1.6f; };
    rig.run(9.98f);
    TEST_ASSERT_TRUE(rig.controller.isPowerCapLimiting());
    rig.maxStepLimiting = 0.0f;
    rig.run(0.1f);
    snprintf(msg, sizeof(msg), "power stepped %.3f %% at the new cap", rig.maxStepLimiting);
    TEST_ASSERT_TRUE_MESSAGE(rig.maxStepLimiting < 0.1f, msg);
    rig.run(20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.10f, 1.6f, rig.trueFlow);
}

// New gains (the meter board sends its settings every few seconds) wait until the cap is out of use, as they
// wait for a flow phase to end: a new dead zone would move the pump in one step.
void test_gains_wait_while_the_cap_is_in_use() {
    Rig a, b;
    for (Rig *r : {&a, &b}) {
        r->settleAt(60.2f, 1.4f);
        r->rampAt = [](float) { return 66.0f; };
        r->resistanceAt = puck570;
        r->run(8.0f);
    }
    TEST_ASSERT_TRUE(a.controller.isPowerCapLimiting());
    FlowLoopGains g = maxGains();
    g.deadZone = 0.30f;
    g.kp = 0.05f;
    a.controller.setFlowLoopGains(g);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, a.controller.getFlowLoopGains().deadZone); // reported as accepted
    a.run(4.0f);
    b.run(4.0f);
    for (size_t i = 0; i < a.powerTrace.size(); i++)
        TEST_ASSERT_EQUAL_FLOAT(b.powerTrace[i], a.powerTrace[i]);
    // the power phase ends (DimmedPump calls this when it leaves POWER mode or the cap goes away)
    a.controller.resetPowerCap();
    TEST_ASSERT_FALSE(a.controller.isPowerCapLimiting());
    // in use from the next capped phase on: its slowest speed is now 30 %
    a.meterStuckAt = 3.0f;
    a.run(60.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, a.power);
}

// The pump is off (power 0, e.g. a pause phase with a cap left in the profile): nothing to do, no state kept.
void test_zero_power_stays_zero() {
    Rig rig;
    rig.settleAt(60.2f, 1.4f);
    rig.rampAt = [](float) { return 0.0f; };
    rig.meterStuckAt = 3.0f;
    rig.run(2.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.power);
    TEST_ASSERT_FALSE(rig.controller.isPowerCapLimiting());
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_cap_is_the_ramp);
    RUN_TEST(test_flow_under_the_cap_follows_the_ramp);
    RUN_TEST(test_cap_holds_the_flow_on_an_easy_puck);
    RUN_TEST(test_cap_takes_the_pump_without_a_step);
    RUN_TEST(test_returns_to_the_ramp_without_a_step);
    RUN_TEST(test_nothing_winds_up_under_the_cap);
    RUN_TEST(test_ramp_step_passes_at_once_under_the_cap);
    RUN_TEST(test_phase_starting_over_the_cap_does_not_step_up);
    RUN_TEST(test_cap_never_stops_the_pump);
    RUN_TEST(test_cap_uses_no_pump_model);
    RUN_TEST(test_no_meter_reading_runs_the_ramp);
    RUN_TEST(test_new_cap_value_while_holding);
    RUN_TEST(test_gains_wait_while_the_cap_is_in_use);
    RUN_TEST(test_zero_power_stays_zero);
    return UNITY_END();
}
