#ifndef VESCCAN_H
#define VESCCAN_H

#include "VescCanLogic.h"
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <functional>

// Fork (maxa4turbo.15): drives the Velofuso pumps through a Flipsky dual VESC over CAN (ESP32-S3 TWAI + SN65HVD230
// on J6: ext5 = TX, ext4 = RX), in place of the analog Velofuso adapter.
//
//  - Every 20 ms while pumping, to VESC ID 18 (and 19 with a second pump): SET_RPM = pump power x max pump power x
//    max ERPM (maxa4turbo.16, speed control: the VESC's field weakening only runs there), or SET_DUTY = pump power x
//    max pump power with speed control switched off. With the pump off, or a speed under 1,000 ERPM asked for:
//    SET_CURRENT 0 A, which releases the motor.
//  - Reads STATUS 1 / 4 / 5 from both VESCs and asks each for its fault code every 200 ms.
//  - Stops every pump (0 A) and reports it when a pump stalls, runs hot, reports a fault or goes silent
//    (rules in VescCanLogic.h). The stop holds until the pump power has been 0 for a second.
//  - If this ESP32 hangs, each VESC releases its motor by itself after its 1000 ms command timeout.
//
// VESC Tool, both sides: App = No App, CAN 500K, Can Status Rate 1 = 50 Hz with Status 1, Can Status Rate 2 = 10 Hz
// with Status 4 + 5, Timeout 1000 ms with brake current 0 A.
class VescCan {
  public:
    static constexpr uint8_t OWN_ID = 42;
    static constexpr uint8_t PUMP_IDS[vesc::MAX_PUMPS] = {18, 19};

    enum BusState : uint8_t { BUS_RUNNING = 0, BUS_OFF = 1, BUS_NOT_STARTED = 2 };

    struct Snapshot {
        vesc::PumpReadings pumps[vesc::MAX_PUMPS];
        vesc::PumpState states[vesc::MAX_PUMPS] = {vesc::PumpState::NO_DATA, vesc::PumpState::NOT_USED};
        bool enabled[vesc::MAX_PUMPS] = {true, false};
        // Which readings are recent enough to show (same ages the stop rules use)
        bool freshSpeed[vesc::MAX_PUMPS] = {false, false};   // STATUS 1: ERPM, motor current, duty
        bool freshTemps[vesc::MAX_PUMPS] = {false, false};   // STATUS 4: temperatures, input current
        bool freshVoltage[vesc::MAX_PUMPS] = {false, false}; // STATUS 5: input voltage
        bool freshFault[vesc::MAX_PUMPS] = {false, false};   // fault poll answered
        float requestedPower = 0.0f;                         // pump power x max pump power the profile asks for, 0..1
        bool speedMode = true;                               // SET_RPM (true) or SET_DUTY in use
        int32_t maxErpm = vesc::MAX_ERPM_DEFAULT;            // speed control: ERPM at 100 %
        int32_t requestedErpm = 0;                           // speed control: what the profile asks for
        int32_t commandErpm = 0;                             // speed control: what is being sent (0 = released)
        float commandDuty = 0.0f;                            // duty control: what is being sent (0 = released)
        bool outputOn = false;                               // a drive command is going out
        vesc::Trip trip = vesc::Trip::NONE;
        uint8_t tripPump = 0;
        vesc::Trip lastTrip = vesc::Trip::NONE;
        uint8_t lastTripPump = 0;
        int32_t lastTripDetail = 0;
        uint32_t lastTripAgoMs = 0;
        uint32_t tripCount = 0;
        uint8_t busState = BUS_NOT_STARTED;
        uint32_t txFailed = 0;
        uint32_t busErrors = 0;
        uint32_t busOffs = 0;
        uint32_t rxFrames = 0;
        uint32_t txFrames = 0;
        bool dualPump = false;
        float motorTempLimit = 0.0f;
        float maxPower = 1.0f;
        uint32_t now = 0;
    };

    // Called from the CAN task in the cycle a stop latches, and again once a second while the display keeps
    // asking for pump power with the stop still latched.
    using TripCallback = std::function<void(vesc::Trip trip, uint8_t pump, int32_t detail)>;

    VescCan(uint8_t txPin, uint8_t rxPin);

    // power: the pump power 0..100 the pump controller writes (same pointer the analog adapter reads).
    bool setup(float *power);
    void setMaxPower(float maxPower);
    // Speed control on: pump power sets the speed (SET_RPM). Off: pump power is the duty (SET_DUTY), as in
    // maxa4turbo.15. A change waits until the pump is off.
    void setSpeedMode(bool speedMode) { _speedModeSetting = speedMode; }
    void setMaxErpm(int32_t maxErpm) { _maxErpm = vesc::clampMaxErpm(maxErpm); }
    void setDualPump(bool dualPump) { _dualPump = dualPump; }
    void setMotorTempLimit(float limitC);
    void onTrip(TripCallback cb) { _tripCb = std::move(cb); }
    Snapshot snapshot();
    uint8_t txPin() const { return _txPin; }
    uint8_t rxPin() const { return _rxPin; }

  private:
    static constexpr uint32_t LOOP_INTERVAL_MS = 10;
    static constexpr uint32_t COMMAND_INTERVAL_MS = 20;       // while pumping
    static constexpr uint32_t IDLE_COMMAND_INTERVAL_MS = 100; // 0 A keep-alive with the pump off
    static constexpr uint32_t FAULT_POLL_INTERVAL_MS = 200;
    static constexpr uint32_t BUS_CHECK_INTERVAL_MS = 100;
    static constexpr uint32_t TRIP_REPEAT_MS = 1000;
    static constexpr uint32_t START_RETRY_MS = 5000;

    bool startDriver();
    void loop();
    void serviceBus(uint32_t now);
    void drainRx(uint32_t now);
    bool transmit(const vesc::Frame &frame);
    static void loopTask(void *arg);

    uint8_t _txPin;
    uint8_t _rxPin;
    float *_power = nullptr;
    std::atomic<float> _maxPower{1.0f};
    std::atomic<bool> _speedModeSetting{true};
    std::atomic<int32_t> _maxErpm{vesc::MAX_ERPM_DEFAULT};
    std::atomic<bool> _dualPump{false};
    std::atomic<float> _motorTempLimit{80.0f};
    TripCallback _tripCb;

    SemaphoreHandle_t _mutex = nullptr; // guards _pumps, _guard and the figures below against snapshot()
    vesc::PumpReadings _pumps[vesc::MAX_PUMPS];
    vesc::Guard _guard;
    bool _enabled[vesc::MAX_PUMPS] = {true, false};
    float _requestedPower = 0.0f;
    bool _speedMode = true;
    int32_t _requestedErpm = 0;
    int32_t _commandErpm = 0;
    float _commandDuty = 0.0f;
    bool _outputOn = false;
    uint8_t _busState = BUS_NOT_STARTED;
    uint32_t _txFailed = 0;
    uint32_t _busErrors = 0;
    uint32_t _busOffs = 0;
    uint32_t _rxFrames = 0;
    uint32_t _txFrames = 0;

    bool _started = false;
    bool _wasOn = false;
    bool _speedOn = false; // speed control: the asked speed is in the range the pump is driven in
    uint32_t _lastCommand = 0;
    uint32_t _lastFaultPoll = 0;
    uint32_t _lastBusCheck = 0;
    uint32_t _lastTripReport = 0;
    uint32_t _lastBusLog = 0;
    uint32_t _lastStartAttempt = 0;
    xTaskHandle _taskHandle = nullptr;

    const char *LOG_TAG = "VescCan";
};

#endif // VESCCAN_H
