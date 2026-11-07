// Serial link to the Waveshare scale board (fork, meterlink.12).
//
// Three wires on J4 (gpio44 RX, gpio43 TX, GND): the scale board reads the two drip-tray load cells
// with the stock HardwareScale code and sends the result here; this class hands every reading to the
// display exactly as the stock HX711 code does, and sends the display's tare, the brewing flag and the
// hardware-scale settings back. Wire format: ScaleWire.h.

#ifndef SCALELINK_H
#define SCALELINK_H

#include "LineProtocol.h"
#include "ScaleWire.h"
#include <Arduino.h>
#include <atomic>
#include <functional>

class ScaleLink {
  public:
    using reading_callback_t = std::function<void(const scalewire::Reading &reading)>;

    static constexpr uint32_t LINK_TIMEOUT_MS = 2000;   // no valid line for this long: link down
    static constexpr uint32_t HEARTBEAT_MS = 1000;      // $H to the board
    static constexpr uint32_t BREW_REFRESH_MS = 500;    // $B repeated while brewing (and on every change)
    static constexpr uint32_t IDLE_REFRESH_MS = 2000;   // $B,0 repeated while idle
    static constexpr uint32_t CONFIG_REFRESH_MS = 5000; // $C repeated (the board applies it only when it changed)
    static constexpr uint32_t BAUD = 115200;

    ScaleLink(uint8_t rxPin, uint8_t txPin, uint8_t uartNum = 2);
    ~ScaleLink() = default;

    void setup();

    void onReading(reading_callback_t cb) { _readingCb = std::move(cb); } // set before setup()

    // --- command side (any task) ---
    void sendTare();
    void setConfiguration(float factor1, float factor2, uint16_t sps, float idleAlpha, float activeAlpha);
    void setActive(bool active) { _active.store(active); } // brew valve open or pump on, from the controller loop

    // --- status ---
    bool linkUp() const;
    uint32_t goodLines() const { return _good.load(); }
    uint32_t badLines() const { return _bad.load(); }
    uint32_t readings() const { return _readings.load(); }
    float lastGrams() const { return _lastGrams.load(); }
    bool scaleAvailable() const { return _available.load(); }
    uint8_t rxPin() const { return _rxPin; }
    uint8_t txPin() const { return _txPin; }

  private:
    void loop();
    void handle(const lineproto::Message &msg);
    void sendLine(const char *payload);
    void sendConfig();
    static void loopTask(void *arg);

    uint8_t _rxPin;
    uint8_t _txPin;
    HardwareSerial _serial;
    lineproto::LineReader _reader;
    SemaphoreHandle_t _txMutex = nullptr;
    portMUX_TYPE _cfgMux = portMUX_INITIALIZER_UNLOCKED;
    TaskHandle_t _taskHandle = nullptr;

    scalewire::Config _config;
    bool _haveConfig = false;
    std::atomic<bool> _configDirty{false};
    std::atomic<bool> _active{false};
    bool _sentActive = false;
    bool _wasUp = false;

    std::atomic<uint32_t> _lastLineMs{0};
    std::atomic<uint32_t> _good{0};
    std::atomic<uint32_t> _bad{0};
    std::atomic<uint32_t> _readings{0};
    std::atomic<float> _lastGrams{scalewire::UNAVAILABLE_GRAMS};
    std::atomic<bool> _available{false};
    uint32_t _lastHeartbeatMs = 0;
    uint32_t _lastBrewMs = 0;
    uint32_t _lastConfigMs = 0;

    reading_callback_t _readingCb;

    const char *LOG_TAG = "ScaleLink";
};

#endif // SCALELINK_H
