#ifndef VESCCANLOGIC_H
#define VESCCANLOGIC_H

#include <cmath>
#include <cstddef>
#include <cstdint>

// Fork (maxa4turbo.15): the Velofuso pumps on a Flipsky dual VESC, driven over CAN instead of the analog adapter.
// This header is the part without hardware: frame layouts and the stop rules. STL only, covered by
// test/test_vesc_can. The TWAI driver around it is VescCan.{h,cpp}.
//
// maxa4turbo.16: the pumps are driven with SET_RPM (the VESC's speed controller) instead of SET_DUTY. Firmware 7.00
// runs field weakening in current and speed control only (motor/foc_math.c, foc_run_fw), so with duty commands the
// pump stopped at the speed 95 % duty gives on 12 V (58,400 ERPM, 3.4 bar on the open group). Duty is kept as a
// setting.
//
// Frame layouts follow the VESC firmware 7.00 source (comm/comm_can.c, comm/commands.c, datatypes.h):
// 29-bit identifier = packet number << 8 | VESC ID, every value big-endian.
namespace vesc {

constexpr uint8_t PACKET_SET_DUTY = 0;             // int32 duty x 100000
constexpr uint8_t PACKET_SET_CURRENT = 1;          // int32 amps x 1000 (6+ bytes would put an off-delay in front)
constexpr uint8_t PACKET_SET_RPM = 3;              // int32 ERPM
constexpr uint8_t PACKET_PROCESS_SHORT_BUFFER = 8; // [sender ID][send flag][command ...]
constexpr uint8_t PACKET_STATUS_1 = 9;             // int32 ERPM, int16 motor A x 10, int16 duty x 1000
constexpr uint8_t PACKET_STATUS_4 = 16;            // int16 FET C x 10, int16 motor C x 10, int16 input A x 10, int16 pid pos x 50
constexpr uint8_t PACKET_STATUS_5 = 27;            // int32 tachometer, int16 input V x 10, int16 reserved
constexpr uint8_t COMM_GET_VALUES_SELECTIVE = 50;
constexpr uint32_t VALUES_MASK_FAULT = 1UL << 15; // one byte: mc_fault_code

constexpr size_t MAX_PUMPS = 2;

struct Frame {
    uint32_t id = 0; // 29-bit extended identifier
    uint8_t len = 0;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

inline uint32_t frameId(uint8_t packet, uint8_t vescId) { return (static_cast<uint32_t>(packet) << 8) | vescId; }

inline void putInt32(uint8_t *b, int32_t v) {
    const auto u = static_cast<uint32_t>(v);
    b[0] = static_cast<uint8_t>(u >> 24);
    b[1] = static_cast<uint8_t>(u >> 16);
    b[2] = static_cast<uint8_t>(u >> 8);
    b[3] = static_cast<uint8_t>(u);
}

inline int32_t getInt32(const uint8_t *b) {
    return static_cast<int32_t>((static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                                (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]));
}

inline int16_t getInt16(const uint8_t *b) {
    return static_cast<int16_t>(static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]));
}

// Duty 0..1. Each one also restarts the VESC's own 1000 ms command timeout.
inline Frame setDuty(uint8_t vescId, float duty) {
    if (!std::isfinite(duty) || duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;
    Frame f;
    f.id = frameId(PACKET_SET_DUTY, vescId);
    f.len = 4;
    putInt32(f.data, static_cast<int32_t>(std::lround(duty * 100000.0f)));
    return f;
}

// Speed control: the VESC's speed controller holds this ERPM (electrical RPM = RPM x 7 on the 14-pole Velofuso
// motor). Restarts the 1000 ms command timeout like every other command. Under its own minimum (Speed Controller ->
// Minimum ERPM, 900) the VESC does not release the motor but holds duty 0, which shorts the phases: the driver
// never sends a speed under SPEED_OFF_ERPM, it sends 0 A instead.
inline Frame setRpm(uint8_t vescId, int32_t erpm) {
    if (erpm < 0)
        erpm = 0;
    Frame f;
    f.id = frameId(PACKET_SET_RPM, vescId);
    f.len = 4;
    putInt32(f.data, erpm);
    return f;
}

// 0 A is below the VESC's minimum current, which switches the PWM off: the motor is released, no hum.
inline Frame setCurrent(uint8_t vescId, float amps) {
    if (!std::isfinite(amps))
        amps = 0.0f;
    Frame f;
    f.id = frameId(PACKET_SET_CURRENT, vescId);
    f.len = 4;
    putInt32(f.data, static_cast<int32_t>(std::lround(amps * 1000.0f)));
    return f;
}

// Asks one VESC for its fault code; it answers to `ownId` with the same command, the mask and one byte.
inline Frame faultRequest(uint8_t vescId, uint8_t ownId) {
    Frame f;
    f.id = frameId(PACKET_PROCESS_SHORT_BUFFER, vescId);
    f.len = 7;
    f.data[0] = ownId;
    f.data[1] = 0; // 0 = process the command and send the answer back to data[0]
    f.data[2] = COMM_GET_VALUES_SELECTIVE;
    f.data[3] = static_cast<uint8_t>(VALUES_MASK_FAULT >> 24);
    f.data[4] = static_cast<uint8_t>(VALUES_MASK_FAULT >> 16);
    f.data[5] = static_cast<uint8_t>(VALUES_MASK_FAULT >> 8);
    f.data[6] = static_cast<uint8_t>(VALUES_MASK_FAULT);
    return f;
}

// Speed control: what 100 % pump power asks for. 67,000 ERPM = 9,571 RPM is the 58,400 ERPM the pump reaches at
// full duty, times sqrt(4.5 / 3.4) for the 4.5 bar the kit controller gave on the open group. The upper bound keeps
// the motor's back-EMF under the 12 V supply (0.884 mWb flux linkage: line-to-line peak = supply at ~74,800 ERPM),
// so a drive that drops out at full speed cannot push current back into the supply; it is also the VESC's own
// Max ERPM on this machine.
constexpr int32_t MAX_ERPM_DEFAULT = 67000;
constexpr int32_t MAX_ERPM_LOWEST = 5000;
constexpr int32_t MAX_ERPM_HIGHEST = 75000;
// The pump is driven from this commanded speed up and released again under the second one (the gap keeps a
// command sitting on the edge from starting and stopping the motor every cycle). Both above the VESC's 900.
constexpr int32_t SPEED_ON_ERPM = 1000;
constexpr int32_t SPEED_OFF_ERPM = 950;

inline int32_t clampMaxErpm(int32_t maxErpm) {
    if (maxErpm < MAX_ERPM_LOWEST)
        return MAX_ERPM_LOWEST;
    if (maxErpm > MAX_ERPM_HIGHEST)
        return MAX_ERPM_HIGHEST;
    return maxErpm;
}

// power: pump power x maximum pump power, 0..1.
inline int32_t powerToErpm(float power, int32_t maxErpm) {
    if (!std::isfinite(power) || power <= 0.0f)
        return 0;
    if (power > 1.0f)
        power = 1.0f;
    return static_cast<int32_t>(std::lround(power * static_cast<float>(clampMaxErpm(maxErpm))));
}

inline bool speedDriveOn(int32_t erpm, bool wasOn) { return erpm >= (wasOn ? SPEED_OFF_ERPM : SPEED_ON_ERPM); }

// Latest values from one VESC, with the time each kind of frame last arrived.
struct PumpReadings {
    bool hasStatus1 = false;
    bool hasStatus4 = false;
    bool hasStatus5 = false;
    bool hasFault = false;
    uint32_t status1At = 0;
    uint32_t status4At = 0;
    uint32_t status5At = 0;
    uint32_t faultAt = 0;
    int32_t erpm = 0;
    float motorCurrent = 0.0f; // A
    float duty = 0.0f;         // 0..1, measured
    float fetTemp = 0.0f;      // C
    float motorTemp = 0.0f;    // C
    float inputCurrent = 0.0f; // A
    int32_t tachometer = 0;
    float inputVoltage = 0.0f; // V
    uint8_t fault = 0;         // mc_fault_code
    uint32_t frames = 0;
};

// Takes one received frame. Returns the index of the pump it updated, -1 when the frame is not from one of
// `vescIds` or is no frame this driver reads.
inline int applyFrame(const Frame &f, uint32_t now, uint8_t ownId, const uint8_t *vescIds, PumpReadings *pumps, size_t count) {
    const auto packet = static_cast<uint8_t>(f.id >> 8);
    const auto low = static_cast<uint8_t>(f.id & 0xFF);
    if ((f.id >> 16) != 0)
        return -1;
    auto indexOf = [&](uint8_t vescId) -> int {
        for (size_t i = 0; i < count; i++)
            if (vescIds[i] == vescId)
                return static_cast<int>(i);
        return -1;
    };
    if (packet == PACKET_PROCESS_SHORT_BUFFER) {
        // Answer to faultRequest(): [VESC ID][1][50][mask][fault], addressed to us.
        if (low != ownId || f.len != 8 || f.data[1] != 1 || f.data[2] != COMM_GET_VALUES_SELECTIVE)
            return -1;
        if (static_cast<uint32_t>(getInt32(f.data + 3)) != VALUES_MASK_FAULT)
            return -1;
        const int i = indexOf(f.data[0]);
        if (i < 0)
            return -1;
        pumps[i].fault = f.data[7];
        pumps[i].hasFault = true;
        pumps[i].faultAt = now;
        pumps[i].frames++;
        return i;
    }
    if (packet != PACKET_STATUS_1 && packet != PACKET_STATUS_4 && packet != PACKET_STATUS_5)
        return -1;
    if (f.len != 8)
        return -1;
    const int i = indexOf(low);
    if (i < 0)
        return -1;
    PumpReadings &p = pumps[i];
    if (packet == PACKET_STATUS_1) {
        p.erpm = getInt32(f.data);
        p.motorCurrent = static_cast<float>(getInt16(f.data + 4)) / 10.0f;
        p.duty = static_cast<float>(getInt16(f.data + 6)) / 1000.0f;
        p.hasStatus1 = true;
        p.status1At = now;
    } else if (packet == PACKET_STATUS_4) {
        p.fetTemp = static_cast<float>(getInt16(f.data)) / 10.0f;
        p.motorTemp = static_cast<float>(getInt16(f.data + 2)) / 10.0f;
        p.inputCurrent = static_cast<float>(getInt16(f.data + 4)) / 10.0f;
        p.hasStatus4 = true;
        p.status4At = now;
    } else {
        p.tachometer = getInt32(f.data);
        p.inputVoltage = static_cast<float>(getInt16(f.data + 4)) / 10.0f;
        p.hasStatus5 = true;
        p.status5At = now;
    }
    p.frames++;
    return i;
}

// Why the pumps were stopped. Values travel to the display as they are.
enum class Trip : uint8_t {
    NONE = 0,
    STALL = 1,   // power asked for, motor not turning
    HOT = 2,     // motor temperature at or over the limit
    FAULT = 3,   // the VESC reports a fault code
    NO_DATA = 4, // power asked for, no STATUS 1 from the VESC
};

struct GuardConfig {
    float stallMinDuty = 0.15f;      // duty control: stall is only judged from this commanded duty up
    int32_t stallMinErpm = 10000;    // speed control: ... from this commanded ERPM up (15 % of 67,000)
    int32_t stallMaxErpm = 200;      // |ERPM| under this counts as not turning
    uint32_t stallTimeMs = 500;      // for this long without a break
    uint32_t startGraceMs = 2000;    // no stall check for this long after the drive switches on
    float motorTempLimit = 80.0f;    // C, 0 = no check (the VESC's own cutback starts at 85 C)
    uint32_t dataTimeoutMs = 500;    // STATUS 1 (50 a second) older than this = no data
    uint32_t readingMaxAgeMs = 1000; // STATUS 4 (10 a second) and fault answers (5 a second) older than this are not used
    uint32_t releaseMs = 1000;       // a stop is released after the pump power has been 0 for this long
};

// What the profile asks of the drive in one control cycle.
struct Demand {
    bool requested = false;  // pump power above 0
    bool driving = false;    // a drive command goes out (speed control: not under SPEED_ON_ERPM)
    bool stallArmed = false; // enough is asked for that a pump standing still is a stall
};

inline bool fresh(bool has, uint32_t at, uint32_t now, uint32_t maxAgeMs) { return has && (now - at) <= maxAgeMs; }

// The stop rules. Fed once per control cycle with what the profile asks for and the latest readings; while
// tripped() the driver sends 0 A to every pump whatever the profile asks for.
class Guard {
  public:
    GuardConfig config;

    Demand dutyDemand(float duty) const {
        Demand d;
        d.requested = d.driving = std::isfinite(duty) && duty > 0.0f;
        d.stallArmed = d.driving && duty >= config.stallMinDuty;
        return d;
    }

    Demand speedDemand(bool requested, bool driving, int32_t erpm) const {
        Demand d;
        d.requested = requested;
        d.driving = requested && driving;
        d.stallArmed = d.driving && erpm >= config.stallMinErpm;
        return d;
    }

    // Duty control. Returns true in the cycle a stop latches.
    bool update(uint32_t now, float duty, const PumpReadings *pumps, const bool *enabled, size_t count) {
        return update(now, dutyDemand(duty), pumps, enabled, count);
    }

    // Returns true in the cycle a stop latches.
    bool update(uint32_t now, const Demand &demand, const PumpReadings *pumps, const bool *enabled, size_t count) {
        if (demand.driving && !_driving) {
            _runStart = now;
            for (size_t i = 0; i < MAX_PUMPS; i++)
                _stalling[i] = false;
        }
        if (!demand.requested && (_requested || !_idleKnown)) {
            _idleSince = now;
            _idleKnown = true;
        }
        _driving = demand.driving;
        _requested = demand.requested;

        if (_trip != Trip::NONE) {
            if (!demand.requested && (now - _idleSince) >= config.releaseMs)
                _trip = Trip::NONE;
            return false;
        }
        if (!demand.driving)
            return false;

        if (count > MAX_PUMPS)
            count = MAX_PUMPS;
        for (size_t i = 0; i < count; i++) {
            if (!enabled[i])
                continue;
            const PumpReadings &p = pumps[i];
            if (fresh(p.hasFault, p.faultAt, now, config.readingMaxAgeMs) && p.fault != 0)
                return latch(Trip::FAULT, i, p.fault, now);
            if (config.motorTempLimit > 0.0f && fresh(p.hasStatus4, p.status4At, now, config.readingMaxAgeMs) &&
                p.motorTemp >= config.motorTempLimit)
                return latch(Trip::HOT, i, static_cast<int32_t>(std::lround(p.motorTemp)), now);
            if (!fresh(p.hasStatus1, p.status1At, now, config.dataTimeoutMs)) {
                _stalling[i] = false;
                if ((now - _runStart) >= config.dataTimeoutMs)
                    return latch(Trip::NO_DATA, i, 0, now);
                continue;
            }
            const int32_t speed = p.erpm < 0 ? -p.erpm : p.erpm;
            if ((now - _runStart) >= config.startGraceMs && demand.stallArmed && speed < config.stallMaxErpm) {
                if (!_stalling[i]) {
                    _stalling[i] = true;
                    _stallSince[i] = now;
                } else if ((now - _stallSince[i]) >= config.stallTimeMs) {
                    return latch(Trip::STALL, i, p.erpm, now);
                }
            } else {
                _stalling[i] = false;
            }
        }
        return false;
    }

    bool tripped() const { return _trip != Trip::NONE; }
    Trip trip() const { return _trip; }
    uint8_t tripPump() const { return _tripPump; } // 1 or 2 while tripped()
    // The stop that latched last, kept after its release (for the console and the display).
    Trip lastTrip() const { return _lastTrip; }
    uint8_t lastTripPump() const { return _lastTripPump; }
    int32_t lastTripDetail() const { return _lastTripDetail; } // fault code, motor C or ERPM
    uint32_t lastTripAt() const { return _lastTripAt; }
    uint32_t tripCount() const { return _tripCount; }

  private:
    bool latch(Trip trip, size_t pump, int32_t detail, uint32_t now) {
        _trip = _lastTrip = trip;
        _tripPump = _lastTripPump = static_cast<uint8_t>(pump + 1);
        _lastTripDetail = detail;
        _lastTripAt = now;
        _tripCount++;
        for (size_t i = 0; i < MAX_PUMPS; i++)
            _stalling[i] = false;
        return true;
    }

    bool _driving = false;
    bool _requested = false;
    bool _idleKnown = false;
    uint32_t _runStart = 0;
    uint32_t _idleSince = 0;
    bool _stalling[MAX_PUMPS] = {false, false};
    uint32_t _stallSince[MAX_PUMPS] = {0, 0};
    Trip _trip = Trip::NONE;
    uint8_t _tripPump = 0;
    Trip _lastTrip = Trip::NONE;
    uint8_t _lastTripPump = 0;
    int32_t _lastTripDetail = 0;
    uint32_t _lastTripAt = 0;
    uint32_t _tripCount = 0;
};

// What the web UI shows per pump. Values travel to the display as they are.
enum class PumpState : uint8_t {
    OFF = 0,
    RUNNING = 1,
    STALL = 2,
    HOT = 3,
    FAULT = 4,
    NO_DATA = 5,
    NOT_USED = 6, // second pump while "second pump" is off
};

inline PumpState pumpState(size_t index, bool enabled, const PumpReadings &p, const Guard &guard, bool outputOn, uint32_t now) {
    if (!enabled)
        return PumpState::NOT_USED;
    if (guard.tripped() && guard.tripPump() == index + 1) {
        switch (guard.trip()) {
        case Trip::STALL:
            return PumpState::STALL;
        case Trip::HOT:
            return PumpState::HOT;
        case Trip::FAULT:
            return PumpState::FAULT;
        default:
            return PumpState::NO_DATA;
        }
    }
    if (!fresh(p.hasStatus1, p.status1At, now, guard.config.dataTimeoutMs))
        return PumpState::NO_DATA;
    if (fresh(p.hasFault, p.faultAt, now, guard.config.readingMaxAgeMs) && p.fault != 0)
        return PumpState::FAULT;
    if (guard.config.motorTempLimit > 0.0f && fresh(p.hasStatus4, p.status4At, now, guard.config.readingMaxAgeMs) &&
        p.motorTemp >= guard.config.motorTempLimit)
        return PumpState::HOT;
    return outputOn ? PumpState::RUNNING : PumpState::OFF;
}

inline const char *tripName(Trip trip) {
    switch (trip) {
    case Trip::STALL:
        return "stall";
    case Trip::HOT:
        return "hot";
    case Trip::FAULT:
        return "fault";
    case Trip::NO_DATA:
        return "no data";
    default:
        return "none";
    }
}

// mc_fault_code names (datatypes.h, firmware 7.00) for the console; the web UI has its own copy.
inline const char *faultName(uint8_t fault) {
    static const char *const NAMES[] = {"NONE",
                                        "OVER_VOLTAGE",
                                        "UNDER_VOLTAGE",
                                        "DRV",
                                        "ABS_OVER_CURRENT",
                                        "OVER_TEMP_FET",
                                        "OVER_TEMP_MOTOR",
                                        "GATE_DRIVER_OVER_VOLTAGE",
                                        "GATE_DRIVER_UNDER_VOLTAGE",
                                        "MCU_UNDER_VOLTAGE",
                                        "BOOTING_FROM_WATCHDOG_RESET",
                                        "ENCODER_SPI",
                                        "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
                                        "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
                                        "FLASH_CORRUPTION",
                                        "HIGH_OFFSET_CURRENT_SENSOR_1",
                                        "HIGH_OFFSET_CURRENT_SENSOR_2",
                                        "HIGH_OFFSET_CURRENT_SENSOR_3",
                                        "UNBALANCED_CURRENTS",
                                        "BRK",
                                        "RESOLVER_LOT",
                                        "RESOLVER_DOS",
                                        "RESOLVER_LOS",
                                        "FLASH_CORRUPTION_APP_CFG",
                                        "FLASH_CORRUPTION_MC_CFG",
                                        "ENCODER_NO_MAGNET",
                                        "ENCODER_MAGNET_TOO_STRONG",
                                        "PHASE_FILTER",
                                        "ENCODER_FAULT",
                                        "LV_OUTPUT_FAULT",
                                        "ENCODER_SLIP",
                                        "OVERSPEED",
                                        "UNDERSPEED",
                                        "ABS_OVERSPEED"};
    return fault < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[fault] : "UNKNOWN";
}

} // namespace vesc

#endif // VESCCANLOGIC_H
