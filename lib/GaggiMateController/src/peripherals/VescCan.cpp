#include "VescCan.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <driver/gpio.h>
#include <driver/twai.h>

VescCan::VescCan(uint8_t txPin, uint8_t rxPin) : _txPin(txPin), _rxPin(rxPin) {}

bool VescCan::setup(float *power) {
    _power = power;
    _mutex = xSemaphoreCreateMutex();
    configASSERT(_mutex);
    _started = startDriver();
    _busState = _started ? BUS_RUNNING : BUS_NOT_STARTED;
    // The task runs even without a driver: the stop rules then end a shot with "no data" instead of letting it
    // run with a pump that cannot be reached, and the driver start is tried again every few seconds.
    if (xTaskCreate(loopTask, "VescCan::loop", configMINIMAL_STACK_SIZE * 8, this, 2, &_taskHandle) != pdPASS) {
        ESP_LOGE(LOG_TAG, "Could not start the CAN task");
        return false;
    }
    return _started;
}

bool VescCan::startDriver() {
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(_txPin), static_cast<gpio_num_t>(_rxPin), TWAI_MODE_NORMAL);
    // Two cycles of commands and fault requests at most: nothing old is left to go out after a bus fault.
    general.tx_queue_len = 8;
    // Both VESCs together send about 150 frames a second; the task empties this every 10 ms.
    general.rx_queue_len = 48;
    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t err = twai_driver_install(&general, &timing, &filter);
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "TWAI driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    // Without a transceiver the RX pin floats; pulled up it reads as an idle bus.
    gpio_set_pull_mode(static_cast<gpio_num_t>(_rxPin), GPIO_PULLUP_ONLY);
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "TWAI start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return false;
    }
    ESP_LOGI(LOG_TAG, "CAN pump drive: TX gpio%u, RX gpio%u, 500 kbit/s, own ID %u, pump 1 = VESC ID %u, pump 2 = VESC ID %u",
             _txPin, _rxPin, OWN_ID, PUMP_IDS[0], PUMP_IDS[1]);
    return true;
}

void VescCan::setMaxPower(float maxPower) {
    _maxPower = std::isfinite(maxPower) ? std::max(0.0f, std::min(1.0f, maxPower)) : 0.0f;
}

void VescCan::setMotorTempLimit(float limitC) {
    // 0 switches the check off; anything else is kept inside a range that makes sense for a motor winding.
    if (!std::isfinite(limitC) || limitC <= 0.0f)
        _motorTempLimit = 0.0f;
    else
        _motorTempLimit = std::max(40.0f, std::min(120.0f, limitC));
}

bool VescCan::transmit(const vesc::Frame &frame) {
    if (!_started)
        return false;
    twai_message_t msg = {};
    msg.extd = 1;
    // Single shot: a frame nobody acknowledges is dropped, not repeated. The next cycle sends a new one.
    msg.ss = 1;
    msg.identifier = frame.id;
    msg.data_length_code = frame.len;
    memcpy(msg.data, frame.data, frame.len);
    esp_err_t err = twai_transmit(&msg, 0);
    if (err == ESP_ERR_TIMEOUT) {
        // Queue full: the bus is not taking frames. Drop what waits so only current values go out.
        twai_clear_transmit_queue();
        err = twai_transmit(&msg, 0);
    }
    if (err == ESP_OK)
        _txFrames++;
    return err == ESP_OK;
}

void VescCan::serviceBus(uint32_t now) {
    if (now - _lastBusCheck < BUS_CHECK_INTERVAL_MS)
        return;
    _lastBusCheck = now;
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK)
        return;
    uint8_t busState = BUS_RUNNING;
    bool busOffEvent = false;
    if (status.state == TWAI_STATE_BUS_OFF) {
        // Too many transmit errors (short on the bus, wrong bit rate). Recover, then start again.
        busState = BUS_OFF;
        busOffEvent = true;
        twai_initiate_recovery();
        if (now - _lastBusLog >= 5000) {
            _lastBusLog = now;
            ESP_LOGW(LOG_TAG, "CAN bus-off, recovering (tx errors %lu, bus errors %lu)",
                     static_cast<unsigned long>(status.tx_error_counter), static_cast<unsigned long>(status.bus_error_count));
        }
    } else if (status.state == TWAI_STATE_RECOVERING) {
        busState = BUS_OFF;
    } else if (status.state == TWAI_STATE_STOPPED) {
        busState = BUS_OFF;
        twai_start();
    }
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _busState = busState;
    if (busOffEvent)
        _busOffs++;
    _txFailed = status.tx_failed_count;
    _busErrors = status.bus_error_count;
    xSemaphoreGive(_mutex);
}

void VescCan::drainRx(uint32_t now) {
    twai_message_t msg;
    // Bounded, so a flood on the bus cannot hold the command cycle up.
    for (int n = 0; n < 64 && twai_receive(&msg, 0) == ESP_OK; n++) {
        if (!msg.extd || msg.rtr || msg.data_length_code > 8)
            continue;
        vesc::Frame frame;
        frame.id = msg.identifier;
        frame.len = msg.data_length_code;
        memcpy(frame.data, msg.data, frame.len);
        xSemaphoreTake(_mutex, portMAX_DELAY);
        vesc::applyFrame(frame, now, OWN_ID, PUMP_IDS, _pumps, vesc::MAX_PUMPS);
        _rxFrames++;
        xSemaphoreGive(_mutex);
    }
}

void VescCan::loop() {
    const uint32_t now = millis();
    if (_started) {
        serviceBus(now);
        drainRx(now);
    } else if (now - _lastStartAttempt >= START_RETRY_MS) {
        _lastStartAttempt = now;
        _started = startDriver();
        if (_started) {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _busState = BUS_RUNNING;
            xSemaphoreGive(_mutex);
        }
    }

    float duty = _power != nullptr ? *_power / 100.0f * _maxPower.load() : 0.0f;
    if (!std::isfinite(duty) || duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    _enabled[0] = true;
    _enabled[1] = _dualPump.load();
    _guard.config.motorTempLimit = _motorTempLimit.load();
    const bool latched = _guard.update(now, duty, _pumps, _enabled, vesc::MAX_PUMPS);
    const bool tripped = _guard.tripped();
    const vesc::Trip trip = _guard.lastTrip();
    const uint8_t tripPump = _guard.lastTripPump();
    const int32_t tripDetail = _guard.lastTripDetail();
    const bool enabled1 = _enabled[1];
    const float out = tripped ? 0.0f : duty;
    _requestedDuty = duty;
    _commandDuty = out;
    xSemaphoreGive(_mutex);

    if (latched) {
        ESP_LOGE(LOG_TAG, "PUMP STOP: pump %u %s (detail %ld) at duty %.3f -> 0 A to every pump", tripPump, vesc::tripName(trip),
                 static_cast<long>(tripDetail), duty);
        _lastTripReport = now;
        if (_tripCb)
            _tripCb(trip, tripPump, tripDetail);
    } else if (tripped && duty > 0.0f && now - _lastTripReport >= TRIP_REPEAT_MS) {
        // The display still asks for pump power: tell it again that the pumps are stopped.
        _lastTripReport = now;
        if (_tripCb)
            _tripCb(trip, tripPump, tripDetail);
    }

    // A change between on and off goes out at once; otherwise every 20 ms running, every 100 ms idle.
    const bool on = out > 0.0f;
    const uint32_t interval = on ? COMMAND_INTERVAL_MS : IDLE_COMMAND_INTERVAL_MS;
    if (on != _wasOn || now - _lastCommand >= interval) {
        _wasOn = on;
        _lastCommand = now;
        // Pump 2 gets 0 A whenever it is not in use, so a VESC with ID 19 on the bus never keeps an old command.
        transmit(on ? vesc::setDuty(PUMP_IDS[0], out) : vesc::setCurrent(PUMP_IDS[0], 0.0f));
        transmit(on && enabled1 ? vesc::setDuty(PUMP_IDS[1], out) : vesc::setCurrent(PUMP_IDS[1], 0.0f));
    }
    if (now - _lastFaultPoll >= FAULT_POLL_INTERVAL_MS) {
        _lastFaultPoll = now;
        transmit(vesc::faultRequest(PUMP_IDS[0], OWN_ID));
        transmit(vesc::faultRequest(PUMP_IDS[1], OWN_ID));
    }
}

VescCan::Snapshot VescCan::snapshot() {
    Snapshot s;
    s.now = millis();
    if (_mutex == nullptr)
        return s;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    for (size_t i = 0; i < vesc::MAX_PUMPS; i++) {
        const vesc::PumpReadings &p = _pumps[i];
        s.pumps[i] = p;
        s.enabled[i] = _enabled[i];
        s.states[i] = vesc::pumpState(i, _enabled[i], p, _guard, _commandDuty > 0.0f, s.now);
        s.freshSpeed[i] = vesc::fresh(p.hasStatus1, p.status1At, s.now, _guard.config.dataTimeoutMs);
        s.freshTemps[i] = vesc::fresh(p.hasStatus4, p.status4At, s.now, _guard.config.readingMaxAgeMs);
        s.freshVoltage[i] = vesc::fresh(p.hasStatus5, p.status5At, s.now, _guard.config.readingMaxAgeMs);
        s.freshFault[i] = vesc::fresh(p.hasFault, p.faultAt, s.now, _guard.config.readingMaxAgeMs);
    }
    s.requestedDuty = _requestedDuty;
    s.commandDuty = _commandDuty;
    s.trip = _guard.trip();
    s.tripPump = _guard.tripped() ? _guard.tripPump() : 0;
    s.lastTrip = _guard.lastTrip();
    s.lastTripPump = _guard.lastTripPump();
    s.lastTripDetail = _guard.lastTripDetail();
    s.lastTripAgoMs = s.now - _guard.lastTripAt();
    s.tripCount = _guard.tripCount();
    s.busState = _busState;
    s.txFailed = _txFailed;
    s.busErrors = _busErrors;
    s.busOffs = _busOffs;
    s.rxFrames = _rxFrames;
    s.txFrames = _txFrames;
    xSemaphoreGive(_mutex);
    s.dualPump = _dualPump.load();
    s.motorTempLimit = _motorTempLimit.load();
    s.maxPower = _maxPower.load();
    return s;
}

void VescCan::loopTask(void *arg) {
    auto *self = static_cast<VescCan *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        self->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_INTERVAL_MS));
    }
}
