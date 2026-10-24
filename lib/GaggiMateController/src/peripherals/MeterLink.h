// Serial link to the Waveshare flow-meter board (fork).
//
// Three wires on J4 (or J5 while the Alba still sits on J4): 115200 8N1, 3.3 V, ASCII lines
// with an 8-bit XOR checksum, see LineProtocol.h.
//
//   meter board -> controller
//     $F,<ml_per_s>,<ml_total>,<seq>*XX      20 per second, measured pump flow
//     $T,<distance_mm>*XX                    4 per second, Alba tank sensor (if the Alba hangs on the meter board)
//     $H,<uptime_s>*XX                       1 per second, heartbeat
//     $G,<kp>,<ki>,<start_duty_pct>,<ff>,<ramp>,<lim_kp>,<lim_ki>*XX   flow-loop gains (meter board web page)
//   controller -> meter board
//     $Z*XX                                  zero the volume counter (display tare / shot start)
//     $L,<channel>,<brightness>*XX           LED command from the display, for the Alba's PCA9634
//     $B,<0|1>*XX                            brew valve / pump active flag (shot bookkeeping on the board)
//
// A fresh $F is one younger than FLOW_TIMEOUT_MS; the flow loop stops the pump without one.

#ifndef METERLINK_H
#define METERLINK_H

#include "LineProtocol.h"
#include <Arduino.h>
#include <PressureController/FlowLoopGains.h>
#include <atomic>
#include <functional>

class MeterLink {
  public:
    using tank_callback_t = std::function<void(int distanceMm)>;
    using gains_callback_t = std::function<void(const FlowLoopGains &gains)>;

    static constexpr uint32_t FLOW_TIMEOUT_MS = 500;  // per the plan: half a second without $F -> pump off
    static constexpr uint32_t LINK_TIMEOUT_MS = 2500; // no valid line at all -> link down
    static constexpr uint32_t BAUD = 115200;

    MeterLink(uint8_t rxPin, uint8_t txPin, uint8_t uartNum = 1);
    ~MeterLink() = default;

    void setup();

    // --- measurement side (read from the pump task) ---
    bool flowValid() const { return flowAgeMs() <= FLOW_TIMEOUT_MS; }
    uint32_t flowAgeMs() const;
    float flow() const { return _flow.load(); }     // ml/s
    float volume() const { return _volume.load(); } // ml since the meter board last zeroed
    uint32_t flowSeq() const { return _seq.load(); }
    bool linkUp() const;
    int tankDistance() const { return _tankMm.load(); } // -1 until the first $T
    uint32_t goodLines() const { return _good.load(); }
    uint32_t badLines() const { return _bad.load(); }
    uint32_t flowLines() const { return _flowLines.load(); }

    // --- command side (any task) ---
    void sendZero();
    void sendLed(uint8_t channel, uint8_t brightness);
    void sendBrewing(bool active);

    void onTank(tank_callback_t cb) { _tankCb = std::move(cb); }
    void onGains(gains_callback_t cb) { _gainsCb = std::move(cb); }

    uint8_t rxPin() const { return _rxPin; }
    uint8_t txPin() const { return _txPin; }

  private:
    void loop();
    void handle(const lineproto::Message &msg);
    void sendLine(const char *payload);
    static void loopTask(void *arg);

    uint8_t _rxPin;
    uint8_t _txPin;
    HardwareSerial _serial;
    lineproto::LineReader _reader;
    SemaphoreHandle_t _txMutex = nullptr;
    xTaskHandle taskHandle = nullptr;

    std::atomic<float> _flow{0.0f};
    std::atomic<float> _volume{0.0f};
    std::atomic<uint32_t> _seq{0};
    std::atomic<uint32_t> _lastFlowMs{0};
    std::atomic<uint32_t> _lastLineMs{0};
    std::atomic<int> _tankMm{-1};
    std::atomic<uint32_t> _good{0};
    std::atomic<uint32_t> _bad{0};
    std::atomic<uint32_t> _flowLines{0};
    bool _everValid = false;
    bool _reportedLoss = false;

    tank_callback_t _tankCb;
    gains_callback_t _gainsCb;

    const char *LOG_TAG = "MeterLink";
};

#endif // METERLINK_H
