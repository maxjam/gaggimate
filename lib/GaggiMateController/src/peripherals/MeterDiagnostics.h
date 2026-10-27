// Pump diagnostics the controller sends to the flow-meter board (fork, since meterlink.4).
//
//   $D,<pump_pct>,<bar>,<mode>,<target_bar>,<target_ml_s>,<ctrl_bar>,<ctrl_ml_s>,<model_ml_s>,<i_pct>,<flags>,
//      <heater_pct>,<boiler_c>*XX          (the last two since meterlink.6; older meter boards read the first ten)
//
// 10 lines per second. The meter board writes the fields as extra columns of its shot log, next to the flow it
// measures itself, so one file holds pump power, pressure and flow on one time base (pump calibration, loop
// tuning). STL only, so the native tests can include it.

#ifndef METERDIAGNOSTICS_H
#define METERDIAGNOSTICS_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

struct MeterDiagnostics {
    enum Flags : uint8_t {
        VALVE_OPEN = 1 << 0,    // brew valve open
        CEILING = 1 << 1,       // flow phase: the pressure ceiling holds the pump
        FLOW_FAULT = 1 << 2,    // flow phase held at 0 because the meter data is stale
        MEASURED_LOOP = 1 << 3, // flow phase running closed loop on the meter
    };

    float pumpPct = 0.0f;    // pump power sent to the pump, 0..100 % (the DAC level on the Velofuso)
    float pressure = 0.0f;   // bar, pressure sensor (averaged)
    uint8_t mode = 0;        // 0 power, 1 pressure, 2 flow
    float targetBar = 0.0f;  // profile pressure: the target in a pressure phase, the ceiling in a flow phase
    float targetFlow = 0.0f; // profile flow: the target in a flow phase, the limit in a pressure phase (ml/s)
    float ctrlBar = 0.0f;    // filtered pressure setpoint the stock pressure loop tracks (bar)
    float ctrlFlow = 0.0f;   // ramped flow target the measured-flow loop tracks, 0 outside it (ml/s)
    float modelFlow = 0.0f;  // the pump model's flow at this power and pressure (ml/s)
    float iPct = 0.0f;       // integral part of the active loop, %: pressure phase = stock I term (duty),
                             // flow phase = flow PI reset (share of the range above the dead zone)
    uint8_t flags = 0;
    float heaterPct = 0.0f;  // brew heater output, 0..100 % (since meterlink.6)
    float boilerC = 0.0f;    // brew boiler sensor, deg C as measured (GaggiMate shows it minus its temperature offset)

    // Payload without '$' and checksum; returns its length, 0 if it does not fit.
    size_t format(char *out, size_t size) const {
        const int n = snprintf(out, size, "D,%.1f,%.2f,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%u,%.1f,%.1f", clean(pumpPct),
                               clean(pressure), static_cast<unsigned>(mode), clean(targetBar), clean(targetFlow), clean(ctrlBar),
                               clean(ctrlFlow), clean(modelFlow), clean(iPct), static_cast<unsigned>(flags), clean(heaterPct),
                               clean(boilerC));
        return (n > 0 && static_cast<size_t>(n) < size) ? static_cast<size_t>(n) : 0;
    }

  private:
    // Keeps "nan"/"inf" off the wire and every field short.
    static double clean(float v) {
        if (!std::isfinite(v))
            return 0.0;
        if (v > 9999.0f)
            return 9999.0;
        if (v < -9999.0f)
            return -9999.0;
        return v;
    }
};

#endif // METERDIAGNOSTICS_H
