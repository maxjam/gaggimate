// Gains of the measured-flow loop (fork: flow phases closed loop on a flow meter).
// Defaults are a conservative starting point for a Velofuso BLDC vane pump reporting
// through the J4/J5 meter link; the meter board can override them with a $G line and the
// controller's USB console with "G kp ki start ff limkp limki".
#ifndef FLOWLOOPGAINS_H
#define FLOWLOOPGAINS_H

struct FlowLoopGains {
    // Inner loop: pump duty (0..1) from the flow error (ml/s).
    float kp = 0.050f;        // duty per ml/s of error           (plant gain ~0.3 ml/s per 1 % duty)
    float ki = 0.150f;        // duty per ml/s per second
    float startDuty = 0.20f;  // integrator pre-load when a flow phase starts (fraction of full duty)
    float ff = 0.0f;          // 0..1 weight of the pump-model feedforward; 0 = measured flow only
    float ramp = 4.0f;        // ml/s per second: the flow target is slewed at this rate (no step kicks)
    // Outer loop: trims the flow target so the pressure ceiling of a flow phase is held.
    float limKp = 3.0f;       // ml/s of trim per bar below the ceiling (the limiter engages target/limKp bar below it)
    float limKi = 1.5f;       // ml/s per bar per second

    // Range check shared by every input path ($G line, USB console). NaN fails every comparison.
    bool valid() const {
        return kp >= 0.0f && kp <= 1.0f && ki >= 0.0f && ki <= 5.0f && startDuty >= 0.0f && startDuty <= 1.0f && ff >= 0.0f &&
               ff <= 1.0f && ramp >= 0.0f && ramp <= 100.0f && limKp >= 0.0f && limKp <= 20.0f && limKi >= 0.0f && limKi <= 20.0f;
    }
};

#endif // FLOWLOOPGAINS_H
