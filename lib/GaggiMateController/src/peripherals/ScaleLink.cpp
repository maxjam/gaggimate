#include "ScaleLink.h"
#include <cstdio>

ScaleLink::ScaleLink(uint8_t rxPin, uint8_t txPin, uint8_t uartNum) : _rxPin(rxPin), _txPin(txPin), _serial(uartNum) {}

void ScaleLink::setup() {
    _txMutex = xSemaphoreCreateMutex();
    // Tare ids start at a random point so a restarted Pro never repeats an id the board already handled.
    const uint32_t base = 100000 + esp_random() % 900000;
    _tareReq.store(base);
    _tareActive = base;
    _serial.setRxBufferSize(512);
    _serial.begin(BAUD, SERIAL_8N1, _rxPin, _txPin);
    ESP_LOGI(LOG_TAG, "Scale link on RX=%u TX=%u, %lu baud", _rxPin, _txPin, static_cast<unsigned long>(BAUD));
    // The reading callback encodes protobuf and notifies over BLE, as the stock HardwareScale task does.
    xTaskCreate(loopTask, "ScaleLink::loop", configMINIMAL_STACK_SIZE * 10, this, 1, &_taskHandle);
}

bool ScaleLink::linkUp() const {
    const uint32_t last = _lastLineMs.load();
    return last != 0 && millis() - last <= LINK_TIMEOUT_MS;
}

void ScaleLink::setConfiguration(float factor1, float factor2, uint16_t sps, float idleAlpha, float activeAlpha) {
    portENTER_CRITICAL(&_cfgMux);
    _config.factor1 = factor1;
    _config.factor2 = factor2;
    _config.sps = sps;
    _config.idleAlpha = idleAlpha;
    _config.activeAlpha = activeAlpha;
    _haveConfig = true;
    portEXIT_CRITICAL(&_cfgMux);
    _configDirty.store(true); // sent from the link task on its next pass
    ESP_LOGI(LOG_TAG, "Hardware scale settings from the display: factors %.3f / %.3f, %u SPS, alpha %.2f / %.2f", factor1,
             factor2, sps, idleAlpha, activeAlpha);
}

void ScaleLink::sendConfig() {
    scalewire::Config c;
    bool have;
    portENTER_CRITICAL(&_cfgMux);
    c = _config;
    have = _haveConfig;
    portEXIT_CRITICAL(&_cfgMux);
    if (!have)
        return; // the display has not sent valid factors yet: the board keeps the ones it saved
    char payload[lineproto::MAX_LINE];
    if (scalewire::formatConfig(c, payload, sizeof(payload)) > 0)
        sendLine(payload);
}

void ScaleLink::sendTare() { _tareReq.fetch_add(1); }

void ScaleLink::loop() {
    lineproto::Message msg;
    int budget = 512; // bytes per pass
    while (budget-- > 0 && _serial.available() > 0) {
        const int c = _serial.read();
        if (c < 0)
            break;
        if (_reader.feed(static_cast<char>(c), msg)) {
            _lastLineMs.store(millis());
            handle(msg);
        }
    }
    _good.store(_reader.goodLines());
    _bad.store(_reader.badLines());

    const uint32_t now = millis();
    const bool up = linkUp();
    if (up && !_wasUp) {
        // Board just appeared (or came back after a restart): give it the settings and the brew state right away.
        ESP_LOGI(LOG_TAG, "Scale board connected");
        _configDirty.store(true);
        _lastBrewMs = 0;
    } else if (!up && _wasUp) {
        ESP_LOGW(LOG_TAG, "Scale board silent for %lu ms", static_cast<unsigned long>(LINK_TIMEOUT_MS));
        _available.store(false);
    }
    _wasUp = up;

    if (now - _lastHeartbeatMs >= HEARTBEAT_MS) {
        _lastHeartbeatMs = now;
        char payload[20];
        snprintf(payload, sizeof(payload), "H,%lu", static_cast<unsigned long>(now / 1000));
        sendLine(payload);
    }

    const bool active = _active.load();
    if (active != _sentActive || _lastBrewMs == 0 || now - _lastBrewMs >= (active ? BREW_REFRESH_MS : IDLE_REFRESH_MS)) {
        _sentActive = active;
        _lastBrewMs = now == 0 ? 1 : now;
        sendLine(active ? "B,1" : "B,0");
    }

    // Tare: the newest id goes out every TARE_RESEND_MS until a $W carries it back, so one corrupted line or a
    // busy moment never leaves the cup weight in a shot.
    const uint32_t req = _tareReq.load();
    if (req != _tareActive) {
        _tareActive = req;
        _tareFirstMs = now;
        _tareLastMs = 0;
    }
    if (_tareFirstMs != 0 && _tareAck.load() != _tareActive) {
        if (now - _tareFirstMs > TARE_GIVE_UP_MS) {
            ESP_LOGW(LOG_TAG, "Tare %lu not acknowledged by the scale board", static_cast<unsigned long>(_tareActive));
            _taresLost.fetch_add(1);
            _tareFirstMs = 0;
        } else if (_tareLastMs == 0 || now - _tareLastMs >= TARE_RESEND_MS) {
            _tareLastMs = now;
            char payload[24];
            snprintf(payload, sizeof(payload), "Z,%lu", static_cast<unsigned long>(_tareActive));
            sendLine(payload);
        }
    } else if (_tareFirstMs != 0) {
        _tareFirstMs = 0; // acknowledged
    }

    if (_configDirty.exchange(false) || now - _lastConfigMs >= CONFIG_REFRESH_MS) {
        _lastConfigMs = now;
        sendConfig();
    }
}

void ScaleLink::handle(const lineproto::Message &msg) {
    if (msg.type[0] == 'W') {
        scalewire::Reading r;
        if (!scalewire::parseWeight(msg, r))
            return;
        _readings.fetch_add(1);
        _tareAck.store(r.tareAck);
        _available.store(r.available());
        _lastGrams.store(r.grams);
        if (_readingCb)
            _readingCb(r);
    }
    // $H and anything else only refresh _lastLineMs
}

void ScaleLink::sendLine(const char *payload) {
    char out[lineproto::MAX_LINE + 8];
    const size_t n = lineproto::frame(payload, out, sizeof(out));
    if (n == 0)
        return;
    if (_txMutex != nullptr && xSemaphoreTake(_txMutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return;
    _serial.write(reinterpret_cast<const uint8_t *>(out), n);
    if (_txMutex != nullptr)
        xSemaphoreGive(_txMutex);
}

void ScaleLink::loopTask(void *arg) {
    auto *link = static_cast<ScaleLink *>(arg);
    while (true) {
        link->loop();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
