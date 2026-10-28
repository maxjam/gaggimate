#include "DimmedPump.h"

#include <ExtensionIOXL9555.hpp>
#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor, PressureControlRate controlRate)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), _psm(_sense_pin, _ssr_pin, 100, FALLING, 1, 4), _pressureSensor(pressure_sensor),
      _controlRate(controlRate), _pressureController(pressureControlPeriodS(controlRate), &_ctrlPressure, &_ctrlFlow,
                                                     &_currentPressure, &_controllerPower, &_valveStatus) {
    _psm.set(0);
}

void DimmedPump::setup() {
    _cps = _psm.cps();
    ESP_LOGI("DimmedPump", "Detected ZC frequency: %dHz", _cps);
    if (_cps > 70) {
        _cps = _cps / 2;
        _psm.setDivider(2);
        ESP_LOGI("DimmedPump", "Detected 2 signals per wave, dividing down to %dHz", _cps);
    }
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void DimmedPump::loop() {
    const int64_t now = esp_timer_get_time();
    const float elapsed = _lastUpdateUs ? (now - _lastUpdateUs) / 1000000.0f : pressureControlPeriodS(_controlRate);
    _lastUpdateUs = now;
    const bool fresh = _pressureSensor->consumeAverage();
    if (fresh)
        _currentPressure = _pressureSensor->getAveragedPressure();
    if (_meter != nullptr) {
        // Measured pump flow from the meter board; stale data (> FLOW_TIMEOUT_MS) is flagged invalid
        // and the flow loop stops the pump rather than guessing.
        _pressureController.setMeasuredFlow(_meter->flow(), _meter->flowValid());
    }
    // Leaving FLOW mode resets the measured-flow loop here, in the control task, so the reset can never
    // land halfway through a control cycle (the mode itself is set from the BLE callbacks).
    const ControlMode mode = _mode;
    if (mode != _lastLoopMode) {
        if (_lastLoopMode == ControlMode::FLOW)
            _pressureController.resetFlowLoop();
        _lastLoopMode = mode;
    }
    updatePower(fresh, _pressureSensor->getSampleTime(), elapsed);
    _currentFlow = _pressureController.getPumpFlowRate();
}

void DimmedPump::setMeterLink(MeterLink *link) {
    _meter = link;
    _pressureController.enableMeasuredFlowControl(link != nullptr);
    if (link != nullptr) {
        link->onDiagnostics([this](MeterDiagnostics &d) {
            fillDiagnostics(d);
            return true;
        });
    }
}

// Runs in the meter link's task. Every value is a single aligned float or int written by the pump task, so a
// read never tears; the fields may straddle one control cycle, which is fine for a 10 Hz log.
void DimmedPump::fillDiagnostics(MeterDiagnostics &d) const {
    const ControlMode mode = _mode;
    const float power = _power;
    const bool measuredLoop = mode == ControlMode::FLOW && _meter != nullptr;
    d.pumpPct = power;
    d.pressure = _currentPressure;
    d.mode = static_cast<uint8_t>(mode);
    d.targetBar = _ctrlPressure;
    d.targetFlow = _ctrlFlow;
    d.ctrlBar = _pressureController.getFilteredSetpoint();
    d.ctrlFlow = measuredLoop ? _pressureController.getEffectiveFlowTarget() : 0.0f;
    d.modelFlow = _pressureController.getModelPumpFlow(power);
    if (measuredLoop)
        d.iPct = _pressureController.getFlowLoopIntegral() * 100.0f;
    else if (mode != ControlMode::POWER)
        d.iPct = _pressureController.getPressureITerm() * 100.0f;
    else
        d.iPct = 0.0f;
    uint8_t flags = 0;
    if (_valveStatus)
        flags |= MeterDiagnostics::VALVE_OPEN;
    if (measuredLoop) {
        flags |= MeterDiagnostics::MEASURED_LOOP;
        if (_pressureController.isPressureLimiting())
            flags |= MeterDiagnostics::CEILING;
        if (isFlowLoopFaulted())
            flags |= MeterDiagnostics::FLOW_FAULT;
        if (_pressureController.isCutoffLatched())
            flags |= MeterDiagnostics::CUTOFF;
    }
    d.flags = flags;
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _ctrlPressure = setpoint > 0 ? 20.0f : 0.0f;
    _mode = ControlMode::POWER;
    _power = std::clamp(setpoint, 0.0f, 100.0f);
    _controllerPower = _power; // Feed manual control back into pressure controller
    _ctrlFlow = 0.0f;
    _ctrlPressure = 0.0f;
    if (_power == 0.0f) {
        _currentFlow = 0.0f;
    }
    _psm.set(static_cast<int>(_binaryMode ? (_power > 0 ? 100.0f : 0.0f) : _power));
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getCoffeeOutputEstimate(); }

float DimmedPump::getPumpFlow() { return _currentFlow; }

float DimmedPump::getPuckFlow() { return _pressureController.getCoffeeFlowRate(); }

float DimmedPump::getPuckResistance() { return _pressureController.getPuckResistance(); }

void DimmedPump::tare() {
    _pumpedWater = 0.0f;
    _pressureController.tare();
    _pressureController.reset();
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    uint8_t intervalIndex = 0;
    while (true) {
        pump->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(pressureControlIntervalMs(pump->_controlRate, intervalIndex)));
        intervalIndex = (intervalIndex + 1) % 3;
    }
}

void DimmedPump::updatePower(bool freshPressure, float sampleTime, float elapsed) {
    _pumpedWater += _currentFlow * elapsed;
    _pressureController.update(static_cast<PressureController::ControlMode>(_mode), freshPressure, sampleTime, elapsed);
    if (_mode != ControlMode::POWER) {
        _power = _controllerPower;
    } else {
        // Re-assert the manual power here in the control task: setPower() runs in the BLE task and a control
        // cycle that was mid-flight when the shot ended could leave the last flow duty in _controllerPower,
        // which the next flow phase would then start from (meterlink.7).
        _controllerPower = _power;
    }
    if (_binaryMode) {
        int binaryPower = (_controllerPower > 0 || _ctrlPressure > 0 || _ctrlFlow > 0) ? 100 : 0;
        _psm.set(binaryPower);
    } else {
        _psm.set(static_cast<int>(_power));
    }
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlFlow = targetFlow;
    _ctrlPressure = pressureLimit;
    _pressureController.setPressureLimit(pressureLimit);
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlFlow = flowLimit;
    _ctrlPressure = targetPressure;
    _pressureController.setFlowLimit(flowLimit);
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

void DimmedPump::setBinaryMode(bool binaryMode) { _binaryMode = binaryMode; }

void DimmedPump::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    _pressureController.setPumpFlowCoeff(oneBarFlow, nineBarFlow);
}

void DimmedPump::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpFlowPolyCoeffs(a, b, c, d);
}

void DimmedPump::setPumpSlipPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpSlipPolyCoeffs(a, b, c, d);
}

void DimmedPump::setGains(float commutationGain, float convergenceGain, float integralGain) {
    _pressureController.setGains(commutationGain, convergenceGain, integralGain);
}
