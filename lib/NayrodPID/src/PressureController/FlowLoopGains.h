// Gains of the measured-flow loop (fork: flow phases closed loop on a flow meter).
// Defaults are a conservative starting point for a Velofuso BLDC vane pump reporting
// through the J4/J5 meter link; the meter board can override them with a $G line and the
// controller's USB console with "G kp ki start ff ramp limkp limki [deadzone]".
#ifndef FLOWLOOPGAINS_H
#define FLOWLOOPGAINS_H

struct FlowLoopGains {
    // Inner loop: pump duty (0..1) from the flow error (ml/s).
    float kp = 0.050f;        // duty per ml/s of error           (plant gain ~0.3 ml/s per 1 % duty)
    float ki = 0.150f;        // duty per ml/s per second
    float startDuty = 0.20f;  // integrator pre-load when a flow phase starts (fraction of full duty)
    float ff = 0.0f;          // 0..1 weight of the pump-model feedforward; 0 = measured flow only
    float ramp = 4.0f;        // ml/s per second: the flow target is slewed at this rate (no step kicks)
    // Ceiling: a pressure PI that overrides the flow PI when the phase's pressure ceiling is reached.
    float limKp = 3.0f;       // % duty per bar of pressure error
    float limKi = 1.5f;       // % duty per bar per second
    // Pump dead zone: the pump power (0..1) below which the pump does not turn / delivers nothing. The loop
    // then never commands a power between 0 and this: its output u (0..1) goes to the pump as
    // deadZone + u * (1 - deadZone), and u = 0 is the pump's slowest speed. In a flow phase the pump only stops
    // for the pressure ceiling, the hard cutoff, a zero target or missing meter data (since meterlink.4; in
    // meterlink.3 u = 0 was off). 0 = no dead zone (plain duty, u = 0 is off).
    float deadZone = 0.0f;

    // Range check shared by every input path ($G line, USB console). NaN fails every comparison.
    bool valid() const {
        return kp >= 0.0f && kp <= 1.0f && ki >= 0.0f && ki <= 5.0f && startDuty >= 0.0f && startDuty <= 1.0f && ff >= 0.0f &&
               ff <= 1.0f && ramp >= 0.0f && ramp <= 100.0f && limKp >= 0.0f && limKp <= 20.0f && limKi >= 0.0f && limKi <= 20.0f &&
               deadZone >= 0.0f && deadZone <= 0.6f;
    }
};

#endif // FLOWLOOPGAINS_H
