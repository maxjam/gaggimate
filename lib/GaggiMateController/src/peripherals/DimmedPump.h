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

    float _opvPressure = 0.0f;

    static constexpr float BASE_FLOW_RATE = 0.25f;
    static constexpr float MAX_PRESSURE = 15.0f;
    static constexpr float MAX_FREQ = 60.0f;

    void updatePower(bool freshPressure, float sampleTime, float elapsed);
    int64_t _lastUpdateUs = 0;
    void onPressureUpdate(float pressure);

    const char *LOG_TAG = "DimmedPump";
    static void loopTask(void *arg);
};

#endif // DIMMEDPUMP_H
