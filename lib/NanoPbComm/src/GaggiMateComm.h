#ifndef GAGGIMATE_COMM_H
#define GAGGIMATE_COMM_H

#include "LedEffect.h"
#include <cstdint>

// Public protocol vocabulary shared by GaggiMateClient and GaggiMateServer; firmware never sees the nanopb structs.

// Pump control mode. Integer values match gaggimate_PumpMode in the schema.
enum class PumpControlMode : uint8_t {
    Power = 0,    // drive at a fixed power percentage
    Pressure = 1, // target a pressure, flow is the limit
    Flow = 2,     // target a flow, pressure is the limit
};

// Boiler control mode. Integer values match gaggimate_BoilerMode in the schema.
enum class BoilerControlMode : uint8_t {
    Temperature = 0, // setpoint is a target temperature in degC
    Pressure = 1,    // setpoint is a target pressure in bar
};

// Physical link carrying the controller session.
enum class LinkType : uint8_t {
    None = 0,
    Ble = 1,
    Uart = 2,
};

// Per-component commands: drive several components atomically in one frame, comparable so callers send only deltas.
struct BoilerCommand {
    uint8_t index = 0;
    BoilerControlMode mode = BoilerControlMode::Temperature;
    float setpoint = 0.0f;
    bool operator==(const BoilerCommand &o) const { return index == o.index && mode == o.mode && setpoint == o.setpoint; }
    bool operator!=(const BoilerCommand &o) const { return !(*this == o); }
};
struct PumpCommand {
    uint8_t index = 0;
    PumpControlMode mode = PumpControlMode::Power;
    float power = 0.0f;
    float pressure = 0.0f;
    float flow = 0.0f;
    bool operator==(const PumpCommand &o) const {
        return index == o.index && mode == o.mode && power == o.power && pressure == o.pressure && flow == o.flow;
    }
    bool operator!=(const PumpCommand &o) const { return !(*this == o); }
};
struct RelayCommand {
    uint8_t index = 0;
    bool open = false;
    bool operator==(const RelayCommand &o) const { return index == o.index && open == o.open; }
    bool operator!=(const RelayCommand &o) const { return !(*this == o); }
};
// One LED output's target brightness; packed into a single LedControl message so multi-channel updates can't be split.
struct LedChannelCommand {
    uint8_t channel = 0;
    uint8_t brightness = 0;
    LedEffect effect = LedEffect::Off;
    uint8_t minBrightness = 0;
    uint16_t cycleTimeMs = 0; // Alba ignores animation fields.
    bool operator==(const LedChannelCommand &o) const {
        return channel == o.channel && brightness == o.brightness && effect == o.effect && minBrightness == o.minBrightness &&
               cycleTimeMs == o.cycleTimeMs;
    }
    bool operator!=(const LedChannelCommand &o) const { return !(*this == o); }
};

// Error codes; values match gaggimate.ErrorCode and the old string protocol so existing comparisons keep working.
constexpr int ERROR_CODE_NONE = 0;
constexpr int ERROR_CODE_COMM_SEND = 1;
constexpr int ERROR_CODE_COMM_RCV = 2;
constexpr int ERROR_CODE_PROTO_ERR = 3;
constexpr int ERROR_CODE_RUNAWAY = 4;
constexpr int ERROR_CODE_TIMEOUT = 5;
// Autotune saw no reaction within its test window (distinct from TIMEOUT): no PID persist, no watchdog-disconnect UX.
constexpr int ERROR_CODE_AUTOTUNE_TIMEOUT = 6;

// Fork (maxjam.15): the CAN pump drive stopped the pumps (stall, motor too hot, VESC fault, VESC silent).
// The display ends the shot and shows the reason; unlike the codes above it is not a latched controller fault.
constexpr int ERROR_CODE_PUMP_STALL = 50;
constexpr int ERROR_CODE_PUMP_HOT = 51;
constexpr int ERROR_CODE_PUMP_FAULT = 52;
constexpr int ERROR_CODE_PUMP_NO_DATA = 53;
constexpr bool isPumpStopCode(int code) { return code >= ERROR_CODE_PUMP_STALL && code <= ERROR_CODE_PUMP_NO_DATA; }

// Fork (maxjam.15): one pump on the Flipsky dual VESC, as the controller reads it from the CAN bus.
// state: 0 off, 1 running, 2 stall, 3 hot, 4 fault, 5 no data, 6 not used.
constexpr uint8_t VESC_PUMP_OFF = 0;
constexpr uint8_t VESC_PUMP_RUNNING = 1;
constexpr uint8_t VESC_PUMP_STALL = 2;
constexpr uint8_t VESC_PUMP_HOT = 3;
constexpr uint8_t VESC_PUMP_FAULT = 4;
constexpr uint8_t VESC_PUMP_NO_DATA = 5;
constexpr uint8_t VESC_PUMP_NOT_USED = 6;
struct VescPumpReport {
    uint8_t state = VESC_PUMP_NO_DATA;
    int32_t erpm = 0;
    float motorCurrent = 0.0f; // A
    float duty = 0.0f;         // 0..1, measured
    float fetTemp = 0.0f;      // degC
    float motorTemp = 0.0f;    // degC
    float inputCurrent = 0.0f; // A
    float inputVoltage = 0.0f; // V
    uint8_t fault = 0;         // VESC fault code
    bool hasSpeed = false;     // STATUS 1 fresh (ERPM, motor current, duty)
    bool hasTemps = false;     // STATUS 4 fresh (temperatures, input current)
    bool hasVoltage = false;   // STATUS 5 fresh
    bool hasFault = false;     // fault poll answered
};
struct VescStatusReport {
    VescPumpReport pumps[2];
    uint8_t pumpCount = 0;
    float commandDuty = 0.0f; // 0..1, being sent now
    uint8_t trip = 0;         // latched stop: 0 none, 1 stall, 2 hot, 3 fault, 4 no data
    uint8_t tripPump = 0;     // 1 or 2
    uint8_t busState = 0;     // 0 running, 1 bus-off or recovering, 2 driver not started
    uint32_t txFailed = 0;
    bool dualPump = false;
    float motorTempLimit = 0.0f;
};

#endif // GAGGIMATE_COMM_H
