#ifndef DIMMEDPUMP_H
#define DIMMEDPUMP_H
#include "MeterLink.h"
#include "PSM.h"
#include "PressureController/PressureController.h"
#include "PressureSensor.h"
#include "Pump.h"
#include <Arduino.h>

class DimmedPump : public Pump {
  public:
    enum class ControlMode { POWER, PRESSURE, FLOW };

    DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressureSensor, PressureControlRate controlRate);
    ~DimmedPump() = default;

    void setup() override;
    void loop() override;
    void setPower(float setpoint) override;
    // Fork (maxa4turbo.18): power phase with a flow cap. The pump runs at `setpoint` until the meter reads more than
    // `maxFlow` (ml/s); then the measured-flow loop holds maxFlow: power = min(setpoint, flow loop). maxFlow <= 0,
    // no meter link or a pump in on/off mode = setPower(setpoint).
    void setPower(float setpoint, float maxFlow);

    float getCoffeeVolume();
    float getPumpFlow();
    float getPuckFlow();
    float getPuckResistance();
    float getPressureTarget() { return _ctrlPressure; }
    float getFlowTarget() { return _ctrlFlow; }
    float *getPumpPowerPtr() override { return &_power; } // For external pump control
    float *getPumpFlowPtr() { return &_currentFlow; }     // For thermal feedforward
    float getPumpedWater() { return _pumpedWater; }
    int *getValveStatusPtr() { return &_valveStatus; } // For thermal feedforward valve state
    void tare();

    void setFlowTarget(float targetFlow, float pressureLimit);
    void setPressureTarget(float targetPressure, float flowLimit);
    void setPumpFlowCoeff(float oneBarFlow, float nineBarFlow);
    void setPumpFlowPolyCoeffs(float a, float b, float c, float d);
    void setPumpSlipPolyCoeffs(float a, float b, float c, float d);
    void setGains(float commutationGain, float convergenceGain, float integralGain);
    void stop();
    void fullPower();
    void setValveState(bool open);
    void setBinaryMode(bool binaryMode);
    // Floor (0..100) for a pump with a pressure or flow setpoint above zero, so an FOC drive keeps spinning.
    void setMinPower(float minPower);

    // Fork: measured-flow control. With a meter link attached, FLOW mode runs closed loop on
    // the meter's ml/s and telemetry carries measured flow; without one everything is stock.
    void setMeterLink(MeterLink *link);
    MeterLink *getMeterLink() const { return _meter; }
    void setFlowLoopGains(const FlowLoopGains &gains) { _pressureController.setFlowLoopGains(gains); }
    const FlowLoopGains &getFlowLoopGains() const { return _pressureController.getFlowLoopGains(); }
    float getMeasuredFlow() const { return _pressureController.getMeasuredFlow(); }
    bool isMeasuredFlowValid() const { return _pressureController.isMeasuredFlowValid(); }
    float getEffectiveFlowTarget() const { return _pressureController.getEffectiveFlowTarget(); }
    bool isPressureLimiting() const { return _pressureController.isPressureLimiting(); }
    // True while a flow phase is being held at zero because the meter data is missing.
    bool isFlowLoopFaulted() const { return _mode == ControlMode::FLOW && _ctrlFlow > 0.0f && _meter != nullptr && !isMeasuredFlowValid(); }
    ControlMode getMode() const { return _mode; }
    // Fork (maxa4turbo.18): flow cap of the power phase that is running.
    float getPowerSetpoint() const { return _rampPower; } // power the display asked for, before the cap
    float getFlowCap() const { return _capMaxFlow; }      // ml/s, 0 = none
    bool isFlowCapLimiting() const { return _mode == ControlMode::POWER && _pressureController.isPowerCapLimiting(); }
    // For the display's chart: the cap while it holds the pump (and for CAP_REPORT_HOLD_US after), else 0.
    float getFlowCapReport() const { return _capReport; }
    // Pump diagnostics for the meter board's shot log ($D, sent by the meter link 10 times a second).
    void fillDiagnostics(MeterDiagnostics &d) const;

  private:
    uint8_t _ssr_pin;
    uint8_t _sense_pin;
    PSM _psm;
    PressureSensor *_pressureSensor;
    MeterLink *_meter = nullptr;
    const PressureControlRate _controlRate;
    PressureController _pressureController;

    xTaskHandle taskHandle;

    ControlMode _mode = ControlMode::POWER;
    ControlMode _lastLoopMode = ControlMode::POWER; // mode seen by the last control cycle (flow-loop reset edge)
    float _power = 0.0f;
    float _rampPower = 0.0f;  // POWER mode: the power asked for; _power is what the pump gets (lower under a flow cap)
    float _capMaxFlow = 0.0f; // POWER mode: flow cap in ml/s, 0 = none (maxa4turbo.18)
    float _capReport = 0.0f;  // what the display is told: the cap while it limits, else 0
    int64_t _capSeenUs = 0;   // last control cycle in which the cap limited
    float _controllerPower = 0.0f;
    float _ctrlPressure = 0.0f;
    float _ctrlFlow = 0.0f;
    float _currentPressure = 0.0f;
    float _currentFlow = 0.0f;
    float _lastPressure = 0.0f;
    float _pumpedWater = 0.0f;
    int _valveStatus = 0;
    int _cps = MAX_FREQ;
    bool _binaryMode = false;
    float _minPower = 0.0f;

    float _opvPressure = 0.0f;

    static constexpr float BASE_FLOW_RATE = 0.25f;
    static constexpr float MAX_PRESSURE = 15.0f;
    static constexpr float MAX_FREQ = 60.0f;
    // A cap that touches the flow for a moment still shows up in the display's 4-per-second shot samples.
    static constexpr int64_t CAP_REPORT_HOLD_US = 500000;

    void updatePower(bool freshPressure, float sampleTime, float elapsed);
    int64_t _lastUpdateUs = 0;
    void onPressureUpdate(float pressure);

    const char *LOG_TAG = "DimmedPump";
    static void loopTask(void *arg);
};

#endif // DIMMEDPUMP_H
