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
    // When enabled, FLOW mode with the brew valve open runs closed loop on the flow reported by
    // setMeasuredFlow() and the pressure setpoint of the phase acts as a ceiling through an override PI
    // (DEFAULT_CEILING_BAR when the phase gives none). With no fresh measurement the pump is stopped
    // instead of falling back to the pump model. FLOW mode with the valve closed (steam: the display sends
    // the steam pump as a flow target with the steam cutoff as the limit) is the stock model path, so
    // steaming works with or without the meter (meterlink.7).
    void enableMeasuredFlowControl(bool enabled) { _flowLoopEnabled = enabled; }
    bool isMeasuredFlowControlEnabled() const { return _flowLoopEnabled; }
    bool isMeasuredFlowControlActive() const { return _flowLoopEnabled && _valveStatus != nullptr && *_valveStatus == 1; }
    void setMeasuredFlow(float flow, bool valid) {
        _measuredFlow = flow;
        _measuredFlowValid = valid;
    }
    float getMeasuredFlow() const { return _measuredFlow; }
    bool isMeasuredFlowValid() const { return _measuredFlowValid; }
    // New gains take effect when the loop is not running a flow phase (meterlink.7): a dead-zone change
    // re-maps the loop output to a different pump power, so applying it mid-phase would jump the pump.
    void setFlowLoopGains(const FlowLoopGains &gains) {
        if (_flowLoopArmed) {
            _pendingGains = gains;
            _gainsPending = true;
        } else {
            _flowGains = gains;
            _gainsPending = false;
        }
    }
    const FlowLoopGains &getFlowLoopGains() const { return _gainsPending ? _pendingGains : _flowGains; }
    void resetFlowLoop();                                          // call when leaving FLOW mode
    float getEffectiveFlowTarget() const { return _flowTargetEff; } // slewed flow target (ml/s)
    bool isPressureLimiting() const { return _limBinding; }        // the pressure ceiling holds the pump
    bool isCutoffLatched() const { return _cutoffLatched; }        // hard cutoff tripped; pump off until the loop resets
    float getFlowLoopDuty() const { return _flowDuty; }            // last pump power sent (0..1)
    float getFlowLoopIntegral() const { return _flowReset; }        // flow PI reset state (duty)
    float getPressureLoopIntegral() const { return _pressReset; }   // ceiling PI reset state (duty)
    float getRampedFlowSetpoint() const { return _flowSetpointRamped; }

    // === Diagnostics (fork, for the meter board's shot log) ===
    float getModelPumpFlow(float powerPct) const;                   // pump model: ml/s at this power and the current pressure
    float getFilteredSetpoint() const { return _filteredSetpoint; } // what the stock pressure loop tracks (bar)
    float getFilteredPressure() const { return _filteredPressureSensor; }
    float getPressureITerm() const { return _pressureITerm; }       // stock pressure loop: duty added by its integral (0..1)

  private:
    float getPumpDutyCycleForPressure();
    float getPumpDutyCycleMeasuredFlow(float target);
    float rampFlowSetpoint(float target);
    float loopFromPump(float duty) const; // pump power -> loop output u (dead zone removed)
    float pumpFromLoop(float u) const;    // loop output u -> pump power
    static constexpr float HARD_PRESSURE_CUTOFF = 12.5f; // bar: flow loop refuses to push past this (latched, meterlink.7)
    static constexpr float DEFAULT_CEILING_BAR = 10.0f;  // ceiling for a flow phase whose profile gives none (meterlink.7)
    static constexpr float PRESSURE_LOOKAHEAD_S = 0.5f;  // ceiling PI acts on P + dP/dt * this (0.25 s up to meterlink.6, see PRESSURE_TRACK_S)
    static constexpr float PRESSURE_TRACK_S = 0.05f;     // ceiling PI reset tracking time while flow is in charge
    static constexpr float PUMP_IDLE_FLOW = 0.05f;       // ml/s: below this the pump is taken as delivering nothing
    static constexpr float PRESSURE_DRAIN_RATE = 0.1f;   // bar/s: pressure falling faster than this = puck draining
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
    float _pressureITerm = 0.0f;    // diagnostics (fork): duty fraction the integral adds

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
    FlowLoopGains _pendingGains{};  // gains received during a flow phase, applied at the next reset
    bool _gainsPending = false;
    bool _cutoffLatched = false;    // hard cutoff tripped in this flow phase: pump off until resetFlowLoop()
    bool _flowLoopArmed = false; // integrator pre-loaded for the current flow phase
    float _flowSetpointRamped = 0.0f; // slewed flow target (ml/s)
    float _flowReset = 0.0f;     // flow PI reset-feedback state (duty fraction, excludes feedforward)
    float _pressReset = 0.0f;    // ceiling PI reset-feedback state (duty fraction)
    float _flowDuty = 0.0f;      // last applied duty (fraction)
    bool _limBinding = false;    // the ceiling PI is the one in charge
    float _flowTargetEff = 0.0f; // slewed flow target (ml/s)
};

#endif // PRESSURE_CONTROLLER_H
