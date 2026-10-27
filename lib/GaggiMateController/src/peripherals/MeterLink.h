// Serial link to the Waveshare flow-meter board (fork).
//
// Three wires on J4 (or J5 while the Alba still sits on J4): 115200 8N1, 3.3 V, ASCII lines
// with an 8-bit XOR checksum, see LineProtocol.h.
//
//   meter board -> controller
//     $F,<ml_per_s>,<ml_total>,<seq>*XX      20 per second, measured pump flow
//     $T,<distance_mm>*XX                    4 per second, Alba tank sensor (if the Alba hangs on the meter board)
//     $H,<uptime_s>*XX                       1 per second, heartbeat
//     $G,<kp>,<ki>,<start_duty_pct>,<ff>,<ramp>,<lim_kp>,<lim_ki>[,<dead_zone_pct>]*XX
//                                            flow-loop gains (meter board web page; dead zone since meter v1.7.2)
//   controller -> meter board
//     $Z*XX                                  zero the volume counter (display tare / shot start)
//     $L,<channel>,<brightness>*XX           LED command from the display, for the Alba's PCA9634
//     $B,<0|1>*XX                            brew valve / pump active flag (shot bookkeeping on the board)
//     $H,<uptime_s>*XX                       1 per second, so the meter board knows the Pro is there
//     $D,<pump_pct>,<bar>,<mode>,<target_bar>,<target_ml_s>,<ctrl_bar>,<ctrl_ml_s>,<model_ml_s>,<i_pct>,<flags>*XX
//                                            10 per second, pump diagnostics for the meter board's shot log
//                                            (since meterlink.4), see MeterDiagnostics.h
//
// A fresh $F is one younger than FLOW_TIMEOUT_MS; the flow loop stops the pump without one.

#ifndef METERLINK_H
#define METERLINK_H

#include "LineProtocol.h"
#include "MeterDiagnostics.h"
#include <Arduino.h>
#include <PressureController/FlowLoopGains.h>
#include <atomic>
#include <functional>

class MeterLink {
  public:
    using Diagnostics = MeterDiagnostics; // one $D line, see MeterDiagnostics.h
    using tank_callback_t = std::function<void(int distanceMm)>;
    using gains_callback_t = std::function<void(const FlowLoopGains &gains)>;
    using diagnostics_callback_t = std::function<bool(Diagnostics &out)>; // false = nothing to send

    static constexpr uint32_t FLOW_TIMEOUT_MS = 500;  // per the plan: half a second without $F -> pump off
    static constexpr uint32_t LINK_TIMEOUT_MS = 2500; // no valid line at all -> link down
    static constexpr uint32_t HEARTBEAT_MS = 1000;    // $H to the meter board
    static constexpr uint32_t DIAGNOSTICS_MS = 100;   // $D to the meter board
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
    uint32_t duplicateLines() const { return _duplicates.load(); }

    // --- command side (any task) ---
    void sendZero();
    void sendLed(uint8_t channel, uint8_t brightness);
    void sendBrewing(bool active);

    void onTank(tank_callback_t cb) { _tankCb = std::move(cb); }
    void onGains(gains_callback_t cb) { _gainsCb = std::move(cb); }
    // Called from the link task every DIAGNOSTICS_MS; set it before setup().
    void onDiagnostics(diagnostics_callback_t cb) { _diagCb = std::move(cb); }
    uint32_t diagnosticsSent() const { return _diagSent.load(); }

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
    std::atomic<uint32_t> _duplicates{0};
    std::atomic<uint32_t> _diagSent{0};
    bool _everValid = false;
    bool _reportedLoss = false;
    uint32_t _lastHeartbeatMs = 0;
    uint32_t _lastDiagMs = 0;

    tank_callback_t _tankCb;
    gains_callback_t _gainsCb;
    diagnostics_callback_t _diagCb;

    const char *LOG_TAG = "MeterLink";
};

#endif // METERLINK_H
