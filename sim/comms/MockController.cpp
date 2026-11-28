#include "MockController.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

void MockController::begin() {
    active = true;
    lastUpdateMs = millis();
}

void MockController::setBoiler(const BoilerCommand &c) {
    if (c.mode == BoilerControlMode::Temperature && c.index == 0)
        targetTemp = c.setpoint;
}

void MockController::setPump(const PumpCommand &c) {
    pumpMode = c.mode;
    pumpPower = c.power;
    targetPressure = c.pressure;
    // Fork (maxa4turbo.18): the flow of a power command is the phase's flow cap, not a target
    targetFlow = c.mode == PumpControlMode::Power ? 0.0f : c.flow;
    flowCap = c.mode == PumpControlMode::Power ? c.flow : 0.0f;
}

void MockController::setRelay(const RelayCommand &c) {
    if (c.index == 0)
        brewValveOpen = c.open;
}

void MockController::update() {
    if (!active)
        return;
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    if (dt <= 0.0f)
        return;
    if (dt > 0.5f)
        dt = 0.5f; // clamp scheduling gaps so the model can't jump
    lastUpdateMs = now;

    // First-order approach to the boiler setpoint (or back to ambient when off).
    const float tau = targetTemp > 0.0f ? 12.0f : 90.0f;
    const float dest = targetTemp > 0.0f ? targetTemp : ambient;
    const float gain = (dest - temperature) * (1.0f - expf(-dt / tau));
    temperature += gain;

    // Fork (maxa4turbo.18): flow cap of a power phase. A crude stand-in for the Pro's measured-flow loop: once the
    // flow passes the cap an integrator takes the pump power down until the flow sits on the cap, and it hands
    // the pump back when it is at the commanded power again. Enough for the display side: the power it reports
    // drops under the commanded one and the cap is reported while it holds.
    float drive = pumpPower;
    if (pumpMode == PumpControlMode::Power && flowCap > 0.0f && pumpPower > 1.0f) {
        if (!capHolding && flow > flowCap) {
            capHolding = true;
            capPower = pumpPower;
        }
        if (capHolding) {
            capPower += (flowCap - flow) * 20.0f * dt; // % per ml/s of error per second
            if (capPower >= pumpPower)
                capHolding = false;
            else
                drive = std::max(0.0f, capPower);
        }
    } else {
        capHolding = false;
    }

    // The pump is "active" when the display asks for any drive.
    const bool pumpActive = pumpPower > 1.0f || targetPressure > 0.1f || targetFlow > 0.1f;
    float pDest = 0.0f, fDest = 0.0f;
    if (pumpActive) {
        if (pumpMode == PumpControlMode::Pressure) {
            pDest = targetPressure;
            fDest = std::max(0.5f, 4.0f - targetPressure * 0.3f);
        } else if (pumpMode == PumpControlMode::Flow) {
            fDest = targetFlow;
            pDest = std::min(11.0f, targetFlow * 4.0f);
        } else {                                                      // power
            pDest = drive / 100.0f * (brewValveOpen ? 10.0f : 12.0f); // closed valve: deadhead up to the OPV
            fDest = std::max(0.3f, drive / 40.0f);
        }
    }
    pressure += (pDest - pressure) * (1.0f - expf(-dt / 0.4f));
    flow += (fDest - flow) * (1.0f - expf(-dt / 0.4f));

    // Water reaches the cup (scale) only with the brew valve open.
    if (brewValveOpen && flow > 0.05f)
        weight += flow * dt; // ~1 ml ≈ 1 g

    if (now - lastSensorMs >= 100) {
        lastSensorMs = now;
        const float puckResistance = flow > 0.05f ? pressure / flow : 0.0f;
        if (onSensor)
            onSensor(temperature, temperature + 40.0f, pressure, flow, flow, puckResistance, drive,
                     constrain(gain * 100.0, 0.0f, 100.0f), 0.0f);
        if (onFlowCap)
            onFlowCap(capHolding ? flowCap : 0.0f);
        if (onVolumetric)
            onVolumetric(weight);
    }
    if (now - lastTofMs >= 1000) {
        lastTofMs = now;
        if (onTof)
            onTof(40); // mm to the water surface — a comfortably full tank
    }
}
