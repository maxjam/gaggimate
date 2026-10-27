#include "MeterLink.h"
#include <cstdio>

MeterLink::MeterLink(uint8_t rxPin, uint8_t txPin, uint8_t uartNum) : _rxPin(rxPin), _txPin(txPin), _serial(uartNum) {}

void MeterLink::setup() {
    _txMutex = xSemaphoreCreateMutex();
    _serial.setRxBufferSize(1024);
    _serial.begin(BAUD, SERIAL_8N1, _rxPin, _txPin);
    ESP_LOGI(LOG_TAG, "Meter link on RX=%u TX=%u, %lu baud", _rxPin, _txPin, static_cast<unsigned long>(BAUD));
    xTaskCreate(loopTask, "MeterLink::loop", configMINIMAL_STACK_SIZE * 10, this, 2, &taskHandle); // $T callback encodes protobuf + BLE notify
}

uint32_t MeterLink::flowAgeMs() const {
    const uint32_t last = _lastFlowMs.load();
    if (last == 0)
        return UINT32_MAX;
    return millis() - last;
}

bool MeterLink::linkUp() const {
    const uint32_t last = _lastLineMs.load();
    return last != 0 && millis() - last <= LINK_TIMEOUT_MS;
}

void MeterLink::loop() {
    lineproto::Message msg;
    int budget = 512; // bytes per pass; keeps one burst from starving the loop
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

    // Heartbeat to the meter board once a second, so its page can show "Pro connected".
    const uint32_t now = millis();
    if (now - _lastHeartbeatMs >= HEARTBEAT_MS) {
        _lastHeartbeatMs = now;
        char payload[20];
        snprintf(payload, sizeof(payload), "H,%lu", static_cast<unsigned long>(now / 1000));
        sendLine(payload);
    }

    // Pump diagnostics for the meter board's shot log, 10 per second.
    if (_diagCb && now - _lastDiagMs >= DIAGNOSTICS_MS) {
        _lastDiagMs = now;
        Diagnostics d;
        char payload[lineproto::MAX_LINE];
        if (_diagCb(d) && d.format(payload, sizeof(payload)) > 0) {
            sendLine(payload);
            _diagSent.fetch_add(1);
        }
    }

    // One log line on each flow-data loss / recovery, not a flood.
    const bool valid = flowValid();
    if (_everValid && !valid && !_reportedLoss) {
        ESP_LOGW(LOG_TAG, "No flow data for %lu ms (link %s)", static_cast<unsigned long>(flowAgeMs()),
                 linkUp() ? "up" : "down");
        _reportedLoss = true;
    } else if (valid && _reportedLoss) {
        ESP_LOGI(LOG_TAG, "Flow data back");
        _reportedLoss = false;
    }
}

void MeterLink::handle(const lineproto::Message &msg) {
    switch (msg.type[0]) {
    case 'F': {
        if (msg.count < 2)
            return;
        float flow = msg.f(0, -1.0f);
        if (std::isnan(flow) || flow < -0.5f || flow > 50.0f) // 50 ml/s: far above any espresso pump
            return;
        if (flow < 0.0f)
            flow = 0.0f; // a direction-aware meter's backflow reads as "no forward flow", not as missing data
        const uint32_t seq = static_cast<uint32_t>(msg.i(2, 0));
        if (_flowLines.load() > 0 && seq == _seq.load()) {
            // Same sequence number again: a board stuck re-sending one line is not fresh data.
            _duplicates.fetch_add(1);
            return;
        }
        _flow.store(flow);
        _volume.store(std::max(0.0f, msg.f(1, 0.0f)));
        _seq.store(seq);
        _lastFlowMs.store(millis());
        _flowLines.fetch_add(1);
        if (!_everValid) {
            _everValid = true;
            ESP_LOGI(LOG_TAG, "First flow line received");
        }
        break;
    }
    case 'T': {
        const long mm = msg.i(0, -1);
        if (mm < 0 || mm > 4000)
            return;
        _tankMm.store(static_cast<int>(mm));
        if (_tankCb)
            _tankCb(static_cast<int>(mm));
        break;
    }
    case 'G': {
        if (msg.count < 7)
            return;
        FlowLoopGains g;
        g.kp = msg.f(0, g.kp);
        g.ki = msg.f(1, g.ki);
        g.startDuty = msg.f(2, g.startDuty * 100.0f) / 100.0f; // on the wire in percent
        g.ff = msg.f(3, g.ff);
        g.ramp = msg.f(4, g.ramp);
        g.limKp = msg.f(5, g.limKp);
        g.limKi = msg.f(6, g.limKi);
        g.deadZone = msg.f(7, g.deadZone * 100.0f) / 100.0f; // optional 8th field, percent (meter v1.7.2+)
        if (!g.valid()) {
            ESP_LOGW(LOG_TAG, "Rejected $G with out-of-range gains");
            return;
        }
        ESP_LOGI(LOG_TAG,
                 "Flow loop gains from meter: kp=%.4f ki=%.4f start=%.0f%% ff=%.2f ramp=%.1f limKp=%.2f limKi=%.2f deadzone=%.0f%%",
                 g.kp, g.ki, g.startDuty * 100.0f, g.ff, g.ramp, g.limKp, g.limKi, g.deadZone * 100.0f);
        if (_gainsCb)
            _gainsCb(g);
        break;
    }
    case 'H':
    default:
        break; // heartbeat only refreshes _lastLineMs
    }
}

void MeterLink::sendLine(const char *payload) {
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

void MeterLink::sendZero() { sendLine("Z"); }

void MeterLink::sendLed(uint8_t channel, uint8_t brightness) {
    char payload[24];
    snprintf(payload, sizeof(payload), "L,%u,%u", channel, brightness);
    sendLine(payload);
}

void MeterLink::sendBrewing(bool active) { sendLine(active ? "B,1" : "B,0"); }

void MeterLink::loopTask(void *arg) {
    auto *link = static_cast<MeterLink *>(arg);
    while (true) {
        link->loop();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
