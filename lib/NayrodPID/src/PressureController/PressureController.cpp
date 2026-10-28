#include "PressureController.h"
#include "SimpleKalmanFilter/SimpleKalmanFilter.h"
#include <Arduino.h> // ESP_LOGI (reset())
#include <algorithm>
#include <math.h>

// Static utility function for first-order low-pass filtering
void PressureController::applyLowPassFilter(float *filteredValue, float rawValue, float cutoffFreq, float dt) {
    if (filteredValue == nullptr)
        return;

    float alpha = dt / (1.0f / (2.0f * M_PI * cutoffFreq) + dt);
    *filteredValue = alpha * rawValue + (1.0f - alpha) * (*filteredValue);
}

PressureController::PressureController(float dt, float *rawPressureSetpoint, float *rawFlowSetpoint, float *sensorOutput,
                                       float *controllerOutput, int *valveStatus) {
    this->_rawPressureSetpoint = rawPressureSetpoint;
    this->_rawFlowSetpoint = rawFlowSetpoint;
    this->_rawPressure = sensorOutput;
    this->_ctrlOutput = controllerOutput;
    this->_valveStatus = valveStatus;
    this->_dt = dt;
    this->_pressureKalmanFilter = new SimpleKalmanFilter(0.1f, 10.0f, powf(2 * _dt, 2));
    this->_previousPressure = *sensorOutput;
}

void PressureController::filterSetpoint(float rawSetpoint) {
    if (!_setpointFilterInitialized)
        initSetpointFilter();
    float omega = 2.0 * M_PI * _setpointFilterFreq;
    float d2r =
        (omega * omega) * (rawSetpoint - _filteredSetpoint) - 2.0f * _setpointFilterDamping * omega * _filteredSetpointDerivative;
    _filteredSetpointDerivative += std::clamp(d2r * _dt, -_maxPressureRate, _maxPressureRate);
    _filteredSetpoint += _filteredSetpointDerivative * _dt;
}

void PressureController::initSetpointFilter(float val) {
    _filteredSetpoint = *_rawPressureSetpoint;
    if (val != 0.0f)
        _filteredSetpoint = val;
    _filteredSetpointDerivative = 0.0f;
    _setpointFilterInitialized = true;
}

void PressureController::filterSensor(float sampleTime) {
    if (sampleTime <= 0.0f)
        sampleTime = _dt;
    _pressureKalmanFilter->setProcessNoise(powf(2 * sampleTime, 2));
    // Use Kalman filter for pressure (as originally intended)
    float newFiltered = this->_pressureKalmanFilter->updateEstimate(*_rawPressure);

    // Calculate pressure derivative using the filtered pressure
    float pressureDerivative = _pressureInitialized ? (newFiltered - _lastFilteredPressure) / sampleTime : 0.0f;
    _pressureInitialized = true;
    applyLowPassFilter(&_filteredPressureDerivative, pressureDerivative, _filterEstimatorFrequency, sampleTime);

    _lastFilteredPressure = newFiltered;
    _filteredPressureSensor = newFiltered;
}

void PressureController::update(ControlMode mode, bool freshPressure, float sampleTime, float elapsed) {
    if (elapsed > 0.0f)
        _dt = elapsed;
    filterSetpoint(*_rawPressureSetpoint);
    if (freshPressure)
        filterSensor(sampleTime);

    if (mode == ControlMode::FLOW && _flowLoopEnabled) {
        // Fork: flow phases run closed loop on the flow meter. The phase's pressure setpoint is a
        // ceiling, held by a pressure PI that overrides the flow PI (lowest duty wins); both use
        // reset feedback, so neither winds up. Stock zeroed the pressure integral whenever flow
        // won the min(), which is what stalled its flow phases below their limit.
        _errorIntegral = 0.0f; // leave the pressure loop clean for a following pressure phase
        if (!_measuredFlowValid) {
            // Fail-safe: no fresh measurement -> no pump. No silent fallback to the model.
            resetFlowLoop();
            *_ctrlOutput = 0.0f;
        } else {
            *_ctrlOutput = getPumpDutyCycleMeasuredFlow(rampFlowSetpoint(*_rawFlowSetpoint));
        }
    } else if ((mode == ControlMode::FLOW || mode == ControlMode::PRESSURE) && *_rawPressureSetpoint > 0.0f &&
               *_rawFlowSetpoint > 0.0f) {
        float flowOutput = getPumpDutyCycleForFlowRate();
        float pressureOutput = getPumpDutyCycleForPressure();
        *_ctrlOutput = std::min(flowOutput, pressureOutput);
        if (flowOutput < pressureOutput) {
            _errorIntegral = 0.0f; // Reset error buildup in flow target
        }
    } else if (mode == ControlMode::FLOW) {
        *_ctrlOutput = getPumpDutyCycleForFlowRate();
    } else if (mode == ControlMode::PRESSURE) {
        *_ctrlOutput = getPumpDutyCycleForPressure();
    }
    virtualScale();
}

float PressureController::pumpFlowModel(float alpha) const {
    // Positive-displacement model: net = duty * Q_geo - slip (affine in duty).
    // With slip = 0 this reduces to the previous proportional model.
    const float duty = alpha / 100.0f;
    const float slip = getSlip();
    return std::max(0.0f, duty * getGeometricFlow() - slip);
}

float PressureController::getAvailableFlow() const {
    const float P = _filteredPressureSensor;
    const float P2 = P * P;
    const float P3 = P2 * P;
    const float Q =
        _pumpFlowCoefficients[0] * P3 + _pumpFlowCoefficients[1] * P2 + _pumpFlowCoefficients[2] * P + _pumpFlowCoefficients[3];

    return Q;
}

float PressureController::getSlip() const {
    const float P = _filteredPressureSensor;
    const float P2 = P * P;
    const float P3 = P2 * P;
    const float slip =
        _pumpSlipCoefficients[0] * P3 + _pumpSlipCoefficients[1] * P2 + _pumpSlipCoefficients[2] * P + _pumpSlipCoefficients[3];
    return std::max(0.0f, slip); // leakage is never negative
}

// Full-drive curve is the duty=1 slice (Q_geo - slip), so Q_geo = full-drive + slip.
float PressureController::getGeometricFlow() const { return getAvailableFlow() + getSlip(); }

float PressureController::getPumpDutyCycleForFlowRate() const {
    if (*_rawFlowSetpoint <= 0.0f) {
        return 0.0f;
    }
    const float geometricFlow = getGeometricFlow();
    if (geometricFlow <= 0.0f) {
        return 0.0f;
    }
    // Feedforward duty to hit the target flow, accounting for slip (incl. Q_t = 0 hold duty).
    float duty = ((*_rawFlowSetpoint + getSlip()) / geometricFlow) * 100.0f;
    return std::clamp(duty, 0.0f, 100.0f);
}

// ---------------------------------------------------------------------------------------------
// Measured-flow control (fork)
// ---------------------------------------------------------------------------------------------

float PressureController::modelDutyFraction(float flow) const {
    const float geometricFlow = getGeometricFlow();
    if (flow <= 0.0f || geometricFlow <= 0.0f)
        return 0.0f;
    return std::clamp((flow + getSlip()) / geometricFlow, 0.0f, 1.0f);
}

void PressureController::resetFlowLoop() {
    _flowLoopArmed = false;
    _flowSetpointRamped = 0.0f;
    _flowReset = 0.0f;
    _pressReset = 0.0f;
    _flowDuty = 0.0f;
    _limBinding = false;
    _cutoffLatched = false;
    _cutoffArmed = false;
    _flowTargetEff = 0.0f;
    if (_gainsPending) { // gains that arrived during the phase (meterlink.7)
        _flowGains = _pendingGains;
        _gainsPending = false;
    }
}

// Slews the phase's flow target so a step (phase change, profile ramp, loop start) does not kick
// the PI into overshoot through the pump's lag. A new phase starts the slew from the flow that
// is actually running. ramp <= 0 disables the slew.
float PressureController::rampFlowSetpoint(float target) {
    if (target <= 0.0f) {
        _flowSetpointRamped = 0.0f;
        return 0.0f;
    }
    if (!_flowLoopArmed) {
        _flowSetpointRamped = std::clamp(_measuredFlow, 0.0f, target);
    }
    if (_flowGains.ramp <= 0.0f) {
        _flowSetpointRamped = target;
        return target;
    }
    const float maxStep = _flowGains.ramp * _dt;
    _flowSetpointRamped += std::clamp(target - _flowSetpointRamped, -maxStep, maxStep);
    return _flowSetpointRamped;
}

// Override control. Two PI controllers propose a pump duty and the lower one is applied:
//   flow PI      on (target - measured flow), the normal case;
//   pressure PI  on (ceiling - pressure); the phase's pressure value, or DEFAULT_CEILING_BAR without one.
// Each integral is kept as "reset feedback": it relaxes toward the duty actually applied (minus the
// feedforward for the flow PI) with time constant kp/ki. For the controller in charge this is exactly a
// PI; the other one sits at the applied duty plus its own proportional term, so it takes over without a
// bump and can never wind up, whether the other controller or the 0..100 % limit holds the pump.
// This matters on a vane pump: at 9 bar about half the duty only covers slip, so a controller whose
// integral has been pushed far below that has to climb through a dead band before water moves again.
// Pump dead zone (gains.deadZone): the two PIs below work on u (0..1), the share of the pump's power range
// above the dead zone. Pump power = deadZone + u * (1 - deadZone), so the loop never parks the pump in the
// band where it does not turn and has to climb back through it. u = 0 is the pump's slowest speed, not off:
// a flow target below what the pump gives at its slowest speed runs at that speed instead of switching the
// pump on and off (the pulses at low flow and low pressure). Only the pressure ceiling (its PI asking for
// nothing) or the hard cutoff stops the pump in a flow phase. With deadZone = 0, u is the pump power itself
// and u = 0 is off, as before.
float PressureController::loopFromPump(float duty) const {
    const float dz = _flowGains.deadZone;
    if (dz <= 0.0f)
        return duty;
    return std::clamp((duty - dz) / (1.0f - dz), 0.0f, 1.0f);
}

float PressureController::pumpFromLoop(float u) const {
    const float dz = _flowGains.deadZone;
    if (dz <= 0.0f)
        return u;
    return dz + u * (1.0f - dz);
}

float PressureController::getPumpDutyCycleMeasuredFlow(float target) {
    if (*_rawFlowSetpoint <= 0.0f) { // the phase asks for no flow (blooming etc.)
        resetFlowLoop();
        return 0.0f;
    }
    const float ffDuty = _flowGains.ff > 0.0f ? _flowGains.ff * loopFromPump(modelDutyFraction(target)) : 0.0f;
    const float e = target - _measuredFlow;
    // A flow phase without a pressure ceiling in its profile gets DEFAULT_CEILING_BAR (meterlink.7), so a meter
    // reading nothing (signal wire off, stalled turbine) ends in a held pressure, not in the pump at 100 %
    // bouncing off the hard cutoff. The phase's own ceiling, when it has one, is used as before.
    const float ceiling = *_rawPressureSetpoint > 0.2f ? *_rawPressureSetpoint : DEFAULT_CEILING_BAR;
    const float kpP = _flowGains.limKp * 0.01f; // gains are entered in % duty per bar (and per bar-second)
    const float kiP = _flowGains.limKi * 0.01f;
    // The pressure PI looks a little ahead (P + dP/dt * t) so it takes the pump before the pressure,
    // rising at several bar/s through a small dead volume, runs past the ceiling.
    const float pAhead = _filteredPressureSensor + _filteredPressureDerivative * PRESSURE_LOOKAHEAD_S;
    if (!_flowLoopArmed) {
        // Start from the duty already running (a pressure phase holding 9 bar sits well above startDuty;
        // dropping to startDuty would let the pressure fall through the slip band), or from startDuty.
        const float running = loopFromPump(std::clamp(*_ctrlOutput * 0.01f, 0.0f, 1.0f));
        const float start = std::max(loopFromPump(_flowGains.startDuty), running);
        _flowReset = std::max(0.0f, start - ffDuty);
        // The ceiling PI starts at the flow PI's first proposal (start + kp * e), not at `start`: with a flow
        // error of 2 ml/s the difference is 10 % duty, enough for the ceiling PI to "win" the min() with the
        // pressure 3 bar under the ceiling and then climb at its slow pace while the puck drains the pressure
        // (meterlink.6: a 1 s meter dropout at 9 bar re-armed into a 3 bar sag). It now takes over only when
        // the looked-ahead pressure is at the ceiling. A cold start (pump off) with the pressure already past
        // the ceiling begins at 0: no pumping until the pressure is back under it.
        const bool coldAboveCeiling = running <= 0.0f && pAhead > ceiling + 0.3f;
        _pressReset = coldAboveCeiling ? 0.0f : std::max(0.0f, start + _flowGains.kp * e);
        _flowLoopArmed = true;
    }
    _flowTargetEff = target;

    const float flowOut = ffDuty + _flowGains.kp * e + _flowReset;
    const float pressOut = kpP * (ceiling - pAhead) + _pressReset;

    float applied = std::min(flowOut, pressOut);
    _limBinding = pressOut < flowOut;
    // The pump stops only when the ceiling PI asks for nothing; the flow PI at u = 0 leaves it at its slowest speed.
    bool stop = pressOut <= 0.0f;
    // Hard cutoff. On Max's machine the pressure builds to 12.5 bar at switch-on while the boiler heats: the water
    // trapped between the check valve and the closed brew valve expands against the relief valve (and it can build
    // again later, depending on the temperature). That is the pressure the sensor reads when a flush or a shot starts
    // from that state, before the open valve has vented it (a few hundred ms). So the cutoff only
    // *latches* (stays off for the rest of the phase, meterlink.7) once the phase has seen the pressure below
    // CUTOFF_ARM_BAR, i.e. once a rise past 12.5 bar can only be the pump's doing. Above the cutoff before that it
    // just holds the pump off, as meterlink.6 did, and the ceiling PI's cold-start rule keeps it off until the
    // pressure is back under the ceiling.
    if (_filteredPressureSensor < CUTOFF_ARM_BAR)
        _cutoffArmed = true;
    const bool overCutoff = _filteredPressureSensor > HARD_PRESSURE_CUTOFF;
    if (overCutoff && _cutoffArmed)
        _cutoffLatched = true;
    if (overCutoff || _cutoffLatched) {
        applied = 0.0f;
        stop = true;
    }
    applied = std::clamp(applied, 0.0f, 1.0f);

    // Reset feedback (see above). ki = 0 turns a controller into a pure P controller.
    if (_flowGains.ki > 0.0f && _flowGains.kp > 0.0f) {
        const float a = std::min(1.0f, _dt * _flowGains.ki / _flowGains.kp);
        _flowReset += ((applied - ffDuty) - _flowReset) * a;
    } else if (_flowGains.ki > 0.0f) { // pure I controller
        _flowReset = std::clamp(_flowReset + _flowGains.ki * e * _dt, -1.0f, 1.0f);
    }
    // While the flow PI is in charge the pressure PI's reset follows the applied duty closely (PRESSURE_TRACK_S,
    // a few cycles), so it sits at "applied + kpP * error" and only takes the pump once the (looked-ahead)
    // pressure reaches the ceiling; in charge, it relaxes with kpP/kiP, which is plain PI action. kiP = 0 holds
    // it (pure P). meterlink.6 tracked with 0.2 s, which during a fast climb of the flow PI lagged by the
    // equivalent of ~2 bar of ceiling margin and handed the pump to the ceiling PI early.
    {
        const float before = _pressReset;
        float tau = PRESSURE_TRACK_S;
        if (_limBinding)
            tau = (kiP > 0.0f && kpP > 0.0f) ? kpP / kiP : 0.0f;
        if (tau > 0.0f)
            _pressReset += (applied - _pressReset) * std::min(1.0f, _dt / tau);
        // Slip band: once the pump delivers nothing (meter ~0) and the puck is visibly draining the pressure,
        // a lower duty cannot bring the pressure down any faster. Integrating on would park the duty deep
        // below the slip hold duty, and the pump would need seconds to climb back once the pressure reaches
        // the ceiling (an undershoot of bars). So the integral stops going down there. Pressure that is
        // steady or rising above the ceiling (a choked puck, a meter stuck at 0) keeps the integral moving.
        if (_limBinding && _measuredFlow < PUMP_IDLE_FLOW && _filteredPressureDerivative < -PRESSURE_DRAIN_RATE)
            _pressReset = std::max(_pressReset, before);
    }

    _flowDuty = stop ? 0.0f : pumpFromLoop(applied);
    return _flowDuty * 100.0f;
}

float PressureController::getModelPumpFlow(float powerPct) const { return pumpFlowModel(std::clamp(powerPct, 0.0f, 100.0f)); }

void PressureController::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    // Set the affine pump flow model coefficients based on flow measurement at 1 bar and 9 bar
    _pumpFlowCoefficients[0] = 0.0f;
    _pumpFlowCoefficients[1] = 0.0f;
    _pumpFlowCoefficients[2] = (nineBarFlow - oneBarFlow) / 8;
    _pumpFlowCoefficients[3] = oneBarFlow - _pumpFlowCoefficients[2] * 1.0f;
}

void PressureController::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    _pumpFlowCoefficients[0] = a;
    _pumpFlowCoefficients[1] = b;
    _pumpFlowCoefficients[2] = c;
    _pumpFlowCoefficients[3] = d;
}

void PressureController::setPumpSlipPolyCoeffs(float a, float b, float c, float d) {
    _pumpSlipCoefficients[0] = a;
    _pumpSlipCoefficients[1] = b;
    _pumpSlipCoefficients[2] = c;
    _pumpSlipCoefficients[3] = d;
}

void PressureController::setGains(float commutationGain, float convergenceGain, float integralGain) {
    _commutationGain = commutationGain;
    _convergenceGain = convergenceGain;
    _integralGain = integralGain;
}

void PressureController::tare() {
    _coffeeOutput = 0.0f;
    _pumpVolume = 0.0f;
    _puckSaturationVolume = 0.0f;
    _puckState[0] = false;
    _puckState[1] = false;
    _puckState[2] = false;
    _puckCounter = 0;
    _pumpFlowRate = 0.0f;
    // IIR-filtered accumulators: unreset, these carry a shot's end-of-brew
    // depressurization transient into the next shot's puck-conductance-
    // derivative arming signature, suppressing arming — GH #839.
    _filteredPressureDerivative = 0.0f;
    _waterThroughPuckFlowRate = 0.0f;
    _lastPuckConductance = 0.0f;
    _puckConductance = 0.0f;
    _puckConductanceDerivative = 0.0f;
    _coffeeFlowRate = 0.0f;
    _puckResistance = INFINITY;
}

void PressureController::virtualScale() {
    // Fork: with a live flow meter the estimator runs on measured pump flow instead of the model.
    const bool useMeasured = _flowLoopEnabled && _measuredFlowValid;
    float newPumpFlowRate = useMeasured ? std::max(0.0f, _measuredFlow) : pumpFlowModel(*_ctrlOutput);
    applyLowPassFilter(&_pumpFlowRate, newPumpFlowRate, _filterEstimatorFrequency, _dt);
    _pumpVolume += _pumpFlowRate * _dt;
    applyLowPassFilter(&exportPumpFlowRate, newPumpFlowRate, _filterEstimatorFrequency / 2, _dt);

    // Raw entering water puck flow
    float effectiveCompliance = 3.0f / fmax(0.2f, _filteredPressureSensor); // ml*s/bar
    float flowRaw = _pumpFlowRate - effectiveCompliance * _filteredPressureDerivative;

    applyLowPassFilter(&_waterThroughPuckFlowRate, flowRaw, 0.3f, _dt);
    if (_waterThroughPuckFlowRate > 0.0f && *_valveStatus == 1 && _filteredPressureSensor > 0.8f) {
        _puckCounter++;
        _puckSaturationVolume += _waterThroughPuckFlowRate * _dt;

        // PUCK CONDUCTANCE
        // applyLowPassFilter(&_pressureFilterEstimator, _filteredPressureSensor, 1.0f, _dt);
        _puckConductance = _waterThroughPuckFlowRate / sqrtf(_filteredPressureSensor);
        // PUCK CONDUCTANCE DERIVATIVE
        if (_puckCounter <= 1) // To avoid spike we set the derivative to 0 just for init
            _lastPuckConductance = _puckConductance;
        float newPuckConductanceDerivative = (_puckConductance - _lastPuckConductance) / _dt;
        applyLowPassFilter(&_puckConductanceDerivative, newPuckConductanceDerivative, 0.2f, _dt);
        _lastPuckConductance = _puckConductance;

        // Monitoring the puck resistance behavior
        /* Because there is a bit of headspace to be filled up the pressure is not rising fast compare to
        the amount of water pumped in. As per the equation used for estimation C dP/dt = Pumpflow - PuckFlow,
        everything that is not building up pressure is exciting as PuckFlow. Therefore the conductivity of
        the puck is estimated as crazy high at first, then plummits to negative values before going back to zero.
        We use this strange behavior to determine that an equilibrium is established and the equations is
        becoming valid to trigger estimation output.

        We expect the puck to follow three states :
            State 0 : conductivity decreases
            State 1 : conductivity settle down to a certain value ()
            State 2 : puck first drop

        Additionnaly if the first drop would be a user input then to trigger the coffee estimation start one
        would just nee to pass all the state to true.
        */

        if (_puckConductanceDerivative < -0.5f && float(_puckCounter) * _dt > 1.0f) // Puck conductivity is decreasing fast
            _puckState[0] = true;

        int timeStamp = 0;
        if (_puckState[0] && _puckConductanceDerivative > -0.1f && !_puckState[1]) { // Puck conductivity is settling down
            _puckState[1] = true;
            timeStamp = _puckCounter;
        }

        _puckResistance = 1.0 / _puckConductance;
        if (_puckState[1]) {
            if (_puckCounter == timeStamp) { // Intialise values
                _coffeeFlowRate =
                    _waterThroughPuckFlowRate; // Initiate the flow immediatly to the instantaneous flow to not waist time
                _puckResistance = 1.0f / _puckConductance; // Same for the puck resistance
            }
            applyLowPassFilter(&_puckResistance, 1.0f / _puckConductance, 0.1f, _dt); // Filter for cosmetic purpose
            // Reset the puck flow rate to avoid slow decay filter response by using the raw flow value for coffee flow
            applyLowPassFilter(&_coffeeFlowRate, _waterThroughPuckFlowRate, 0.2f, _dt);
            // Account for missed drops (WIP)
            if (!_puckState[2]) {
                float timeMissedDrops = 2.0f; // First drop occured X second ago
                float missedDops =
                    _coffeeFlowRate * timeMissedDrops /
                    2; // Assumption that flow was linearly increasing (triangle integral) that's BS but ... something.
                _puckState[2] = true;
                _coffeeOutput += _coffeeFlowRate * _dt + missedDops;
            } else {
                _coffeeOutput += _coffeeFlowRate * _dt;
            }
        }
        // ESP_LOGI("","%d;\t %1.3e;\t %1.3e;\t %1.3e;\t %1.3e;\t %1.3e", millis(),_puckConductance, _puckConductanceDerivative,
        // _waterThroughPuckFlowRate, _coffeeFlowRate, _puckResistance);
    }
}

float PressureController::getPumpDutyCycleForPressure() {
    // COMMAND IS ACTUALLY ZERO: The profile is asking for no pressure (ex: blooming phase)
    // Until otherwise, make the controller ready to start as if it is a new shot coming
    if (*_rawPressureSetpoint < 0.2f) {
        initSetpointFilter();
        _errorIntegral = 0.0f;
        _pressureITerm = 0.0f;
        *_ctrlOutput = 0.0f;
        _previousPressure = 0.0f;
        return 0.0f;
    }

    // CONTROL: The boiler is pressurised, the profile is something specific, let's try to
    // control that pressure now that all conditions are reunited
    float P = _filteredPressureSensor;
    float P_ref = _filteredSetpoint;
    float error = P - P_ref;
    _previousPressure = P;

    // Switching surface
    float epsilon = _epsilonCoefficient * _filteredSetpoint;
    float deadband = _deadbandCoefficient * _filteredSetpoint;

    float s = _convergenceGain * error;
    float sat_s = 0.0f;
    if (error > 0) {
        float tan = tanhf(s / epsilon - deadband * _convergenceGain / epsilon);
        sat_s = std::max(0.0f, tan);
    } else if (error < 0) {
        float tan = tanhf(s / epsilon + deadband * _convergenceGain / epsilon);
        sat_s = std::min(0.0f, tan);
    }

    // Integrator
    float pressureRatio = 0;
    if (P < _maxPressure) {
        pressureRatio = P / _maxPressure;
    }
    float denominator = fmaxf(1.0f - pressureRatio, 0.0001f); // Clamp to minimum 0.0001
    float Ki = _integralGain / denominator;
    // Integrate only inside the boundary layer so headspace fill can't wind up an overshoot the pump can't undo
    const bool integrating = fabsf(error) < epsilon;
    if (integrating)
        _errorIntegral += error * _dt;
    float iterm = Ki * _errorIntegral;

    // Plant-gain inversion: Qa is the duty->flow slope (d Q_in / d duty), which is the
    // geometric flow Q_geo under the affine model. Reduces to the full-drive curve when slip = 0.
    float Qa = getGeometricFlow();
    Qa = fmaxf(Qa, 1e-3f);
    float Ceq = _systemCompliance;
    float K = _commutationGain / denominator * Qa / Ceq;
    _pumpDutyCycle = Ceq / Qa * (-_convergenceGain * error - K * sat_s) - iterm;

    // Anti-windup against the actuator limits [0, 1]
    if (integrating && ((_pumpDutyCycle > 1.0f && error < 0.0f) || (_pumpDutyCycle < 0.0f && error > 0.0f))) {
        _errorIntegral -= error * _dt;
        iterm = Ki * _errorIntegral;
    }

    _pumpDutyCycle = Ceq / Qa * (-_convergenceGain * error - K * sat_s) - iterm;
    _pressureITerm = -iterm; // diagnostics (fork): what the integral adds to the duty
    return std::clamp(_pumpDutyCycle * 100.0f, 0.0f, 100.0f);
}

void PressureController::reset() {
    initSetpointFilter(_filteredPressureSensor);
    _errorIntegral = 0.0f;
    _pumpFlowRate = 0.0f;
    _puckSaturationVolume = 0.0f;
    _puckState[0] = false;
    _puckState[1] = false;
    _puckState[2] = false;
    _puckCounter = 0;
    ESP_LOGI("", "RESET");
}
