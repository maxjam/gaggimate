// PressureController.h
#ifndef PRESSURE_CONTROLLER_H
#define PRESSURE_CONTROLLER_H
#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

#include "FlowLoopGains.h"
#include "SimpleKalmanFilter/SimpleKalmanFilter.h"
#include <algorithm>
#include <cmath> // std::fmax (getCoffeeOutputEstimate())

class PressureController {
  private:
    // Utility function for first-order low-pass filtering
    static void applyLowPassFilter(float *filteredValue, float rawValue, float cutoffFreq, float dt);

  public:
    enum class ControlMode { POWER, PRESSURE, FLOW };
    PressureController(float dt, float *_rawPressureSetpoint, float *_rawFlowSetpoint, float *sensorOutput,
                       float *controllerOutput, int *valveStatus);
    void initSetpointFilter(float val = 0.0f);

    void setFlowLimit(float lim) { /* Flow limit not currently implemented */ };
    void setPressureLimit(float lim) { /* Pressure limit not currently implemented */ };

    void update(ControlMode mode, bool freshPressure = true, float sampleTime = 0.0f, float elapsed = 0.0f);
    void tare();
    void reset();

    float getCoffeeOutputEstimate() { return std::fmax(0.0f, _coffeeOutput); };
    void setPumpFlowCoeff(float oneBarFlow, float nineBarFlow);
    void setPumpFlowPolyCoeffs(float a, float b, float c, float d);
    // Rotary-vane slip polynomial (gear pump only); default zero -> no effect.
    void setPumpSlipPolyCoeffs(float a, float b, float c, float d);
    void setGains(float commutationGain, float convergenceGain, float integralGain);
    float getPumpFlowRate() { return exportPumpFlowRate; };
    float getCoffeeFlowRate() { return *_valveStatus == 1 ? _coffeeFlowRate : 0.0f; };
    float getPuckResistance() { return _puckResistance; }

    void setDeadVolume(float deadVol) { _puckSaturatedVolume = deadVol; };

    // === Measured-flow control (fork) ===
    // When enabled, FLOW mode runs closed loop on the flow reported by setMeasuredFlow() and the
    // pressure setpoint of the phase acts as a ceiling through an outer limiter. With no fresh
    // measurement the pump is stopped instead of falling back to the pump model.
    void enableMeasuredFlowControl(bool enabled) { _flowLoopEnabled = enabled; }
    bool isMeasuredFlowControlEnabled() const { return _flowLoopEnabled; }
    void setMeasuredFlow(float flow, bool valid) {
        _measuredFlow = flow;
        _measuredFlowValid = valid;
    }
    float getMeasuredFlow() const { return _measuredFlow; }
    bool isMeasuredFlowValid() const { return _measuredFlowValid; }
    void setFlowLoopGains(const FlowLoopGains &gains) { _flowGains = gains; }
    const FlowLoopGains &getFlowLoopGains() const { return _flowGains; }
    void resetFlowLoop();                                          // call when leaving FLOW mode
    float getEffectiveFlowTarget() const { return _flowTargetEff; } // target after the ceiling limiter (ml/s)
    bool isPressureLimiting() const { return _limBinding; }        // ceiling limiter is trimming the target
    float getFlowLoopDuty() const { return _flowDuty; }            // last inner-loop output (0..1)
    float getFlowLoopIntegral() const { return _flowIntegral; }
    float getRampedFlowSetpoint() const { return _flowSetpointRamped; }

  private:
    float getPumpDutyCycleForPressure();
    float getPumpDutyCycleMeasuredFlow(float target);
    float applyPressureLimiter(float target);
    float rampFlowSetpoint(float target);
    float modelDutyFraction(float flow) const; // pump-model feedforward duty (0..1) for a flow
    void virtualScale();
    void filterSensor(float sampleTime);
    bool _pressureInitialized = false;
    void filterSetpoint(float rawSetpoint);
    float pumpFlowModel(float alpha = 100.0f) const;
    float getAvailableFlow() const;
    float getSlip() const;          // Vane-pump internal leakage at current pressure (ml/s)
    float getGeometricFlow() const; // Full geometric flow Q_geo = full-drive curve + slip
    float getPumpDutyCycleForFlowRate() const;

    float _dt = 1.0f; // Controller sampling period (seconds)

    // Input/output pointers
    float *_rawPressureSetpoint = nullptr; // Pressure profile current setpoint/limit (bar)
    float *_rawFlowSetpoint = nullptr;     // Flow profile current setpoint/limit (ml/s)
    float *_rawPressure = nullptr;         // Raw pressure measurement from sensor (bar)
    float *_ctrlOutput = nullptr;          // Controller output power ratio (0-100%)
    int *_valveStatus = nullptr;           // 3-way valve status (group head open/closed)

    // Filtered values
    float _filteredPressureSensor = 0.0f;     // Filtered pressure sensor reading (bar)
    float _filteredSetpoint = 0.0f;           // Filtered pressure setpoint (bar)
    float _filteredSetpointDerivative = 0.0f; // Derivative of filtered setpoint (bar/s)
    float _filteredPressureDerivative = 0.0f; // Derivative of filtered pressure (bar/s)

    // Setpoint filter parameters
    float _setpointFilterFreq = 1.0f;    // Setpoint filter cutoff frequency (Hz)
    float _setpointFilterDamping = 1.2f; // Setpoint filter damping ratio
    bool _setpointFilterInitialized = false;

    // === System parameters ===
    const float _systemCompliance = 1.4f;                            // System compliance (ml/bar)
    float _puckResistance = 1e7f;                                    // Initial estimate of puck resistance
    const float _maxPressure = 15.0f;                                // Maximum pressure (bar)
    const float _maxPressureRate = 9.0f;                             // Maximum pressure rate (bar/s)
    float _pumpFlowCoefficients[4] = {0.0f, 0.0f, -0.5854f, 10.79f}; // Full-drive flow polynomial (Q_geo - slip)
    float _pumpSlipCoefficients[4] = {0.0f, 0.0f, 0.0f, 0.0f};       // Vane-pump slip polynomial (gear pump only)

    // === Controller Gains ===
    float _commutationGain = 0.7f;     // Commutation gain
    float _convergenceGain = 1.0f;     // Convergence gain
    float _epsilonCoefficient = 0.3f;  // Limit band coefficient
    float _deadbandCoefficient = 0.1f; // Dead band coefficient
    float _integralGain = 0.25f;       // Integral gain (dt/tau)

    // === Controller states ===
    float _previousPressure = 0.0f; // Previous pressure reading (bar)
    float _errorIntegral = 0.0f;    // Integral of pressure error
    float _pumpDutyCycle = 0.0f;    // Calculated pump duty cycle (0-100%)

    // === Flow estimation ===
    float _waterThroughPuckFlowRate = 0.0f; // Water through puck flow rate (ml/s)
    float _pumpFlowRate = 0.0f;             // Pump flow rate (ml/s)
    float _pumpVolume = 0.0f;               // Total pump volume (ml)
    float _coffeeOutput = 0.0f;             // Total coffee output (ml)
    float _coffeeFlowRate = 0.0f;           // Coffee output flow rate (mL/s)
    float _lastFilteredPressure = 0.0f;     // Previous filtered pressure for derivative calculation
    float _filterEstimatorFrequency = 1.0f; // Filter frequency for estimator
    float _pressureFilterEstimator = 0.0f;
    float _puckSaturationVolume = 0.0f; // Total volume to saturate the puck(ml)
    float _puckSaturatedVolume = 45.0f; // Volume at puck saturation (ml)
    float _lastPuckConductance = 0.0f;  // Previous puck resistance for derivative calculation
    float _puckConductance = 0.0f;
    float _puckConductanceDerivative = 0.0f; // Derivative of puck resistance
    bool _puckState[3] = {};
    int _puckCounter = 0;
    float exportPumpFlowRate = 0.0f; // To disociate the exported value from the internal because of filtering (cosmetic) purpose
    SimpleKalmanFilter *_pressureKalmanFilter;

    // === Measured-flow loop state (fork) ===
    bool _flowLoopEnabled = false;
    float _measuredFlow = 0.0f;      // ml/s from the flow meter
    bool _measuredFlowValid = false; // false = stale or no meter -> pump off in FLOW mode
    FlowLoopGains _flowGains{};
    bool _flowLoopArmed = false; // integrator pre-loaded for the current flow phase
    float _flowSetpointRamped = 0.0f; // slewed flow target (ml/s)
    float _flowIntegral = 0.0f;  // inner-loop integral term (duty fraction)
    float _flowDuty = 0.0f;      // last inner-loop output (duty fraction)
    float _limIntegral = 0.0f;   // ceiling limiter integral (ml/s)
    bool _limBinding = false;    // ceiling limiter currently trimming the target
    float _flowTargetEff = 0.0f; // flow target after the limiter (ml/s)
};

#endif // PRESSURE_CONTROLLER_H
