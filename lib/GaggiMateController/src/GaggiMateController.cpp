#include "GaggiMateController.h"
#include "MachineHardwareSettings.h"
#include "utilities.h"
#include <Arduino.h>
#include <ExtensionIOXL9555.hpp>
#include <Preferences.h>
#include <peripherals/DimmedPump.h>
#include <peripherals/SimplePump.h>

#include <algorithm>
#include <ble/BleHeadroom.h>
#include <memory>
#include <peripherals/NtcThermistor.h>
#include <utility>

constexpr uint32_t ADDON_GEARPUMP = 7;
constexpr uint32_t ADDON_HW_SCALE = 8;
// Fork (maxa4turbo.15): the pump drive is a Flipsky dual VESC over CAN. Reported next to ADDON_GEARPUMP, so the
// display keeps every gear pump setting and adds the pump panel and its two settings.
constexpr uint32_t ADDON_VESC_CAN = 50;

GaggiMateController::GaggiMateController(String version) : _version(std::move(version)) {
    configs.push_back(GM_STANDARD_REV_1X);
    configs.push_back(GM_STANDARD_REV_2X);
    configs.push_back(GM_STANDARD_REV_3X);
    configs.push_back(GM_PRO_REV_1x);
    configs.push_back(GM_PRO_LEGO);
    configs.push_back(GM_PRO_REV_11);
    configs.push_back(GM_MAX_REV10);
}

bool GaggiMateController::isBondingSwitchOn() const {
    uint8_t pin = _config.waterButtonPin > 0 ? _config.waterButtonPin : _config.steamButtonPin;
    pinMode(pin, INPUT_PULLUP);
    for (int i = 0; i < 5; i++) { // active low; require a steady reading so a bouncing contact never opens the window
        if (digitalRead(pin) != LOW)
            return false;
        delay(10);
    }
    return true;
}

void GaggiMateController::setup() {
    delay(5000);
    hardwareRequests = xQueueCreate(1, sizeof(gm::MachineHardwareSettings));
    configASSERT(hardwareRequests);
    detectBoard();
    // Display UART TX lands here; keep the pin an input until the UART claims it.
    pinMode(_config.ext4Pin, INPUT);
    loadMachineHardware();
    detectAddon();

    if (!_config.capabilites.dualBoiler) {
        brewTemperature = std::make_unique<Max31855Thermocouple>(
            _config.maxCsPin, _config.maxMisoPin, _config.maxSckPin, [this](float temperature) { /* noop */ },
            [this]() { thermalRunawayShutdown(); });
    }
    if (_config.capabilites.pressure || _config.capabilites.dualBoiler) {
        adc = std::make_unique<ADSAdc>(_config.pressureSda, _config.pressureScl, _config.adcRates,
                                       gearpumpAddon ? gearpumpAddon->sharedBusMutex() : nullptr);
        pressureSensor = std::make_unique<PressureSensor>(this->adc.get(), _config.pressureControlRate);
        if (_config.capabilites.dualBoiler) {
            this->brewTemperature = std::make_unique<NtcThermistor>(
                this->adc.get(), 2, [this]() { thermalRunawayShutdown(); }, machineHardware.ntc_brew_nominal_resistance,
                machineHardware.ntc_brew_series_resistance, machineHardware.ntc_brew_supply_voltage,
                machineHardware.ntc_brew_beta, _config.ntcTiming[2]);
            this->steamTemperature = std::make_unique<NtcThermistor>(
                this->adc.get(), 3, [this]() { thermalRunawayShutdown(); }, machineHardware.ntc_steam_nominal_resistance,
                machineHardware.ntc_steam_series_resistance, machineHardware.ntc_steam_supply_voltage,
                machineHardware.ntc_steam_beta, _config.ntcTiming[3]);
        }
    }
    heater = std::make_unique<Heater>(
        this->brewTemperature.get(), _config.heaterPin, [this]() { thermalRunawayShutdown(); },
        [this](float Kp, float Ki, float Kd, float Kf) { _comms.sendAutotuneResult(Kp, Ki, Kd, Kf); },
        [this]() { _comms.sendError(ERROR_CODE_AUTOTUNE_TIMEOUT); });
    if (_config.capabilites.dualBoiler) {
        heater2 = std::make_unique<Heater>(
            this->steamTemperature.get(), _config.altPin, [this]() { thermalRunawayShutdown(); },
            [this](float Kp, float Ki, float Kd, float Kf) { _comms.sendAutotuneResult(Kp, Ki, Kd, Kf); },
            [this]() { _comms.sendError(ERROR_CODE_AUTOTUNE_TIMEOUT); });
        heaterCoordinator = std::make_unique<HeaterCoordinator>(heater.get(), heater2.get());
        refill = std::make_unique<SimpleRelay>(_config.refillPin, _config.valveOn);
        aux = std::make_unique<SimpleRelay>(_config.auxPin, _config.valveOn);
        if (machineHardware.steam_level_enabled && _config.waterSensePin != 0)
            waterSense = std::make_unique<DigitalInput>(
                _config.waterSensePin,
                [this](const bool state) {
                    steamLevelLow = state;
                    steamLevelKnown = true;
                    if (_config.capabilites.dualBoiler && machineHardware.steam_level_enabled && state && heater2)
                        heater2->setSetpoint(0);
                    _comms.sendButtonState(3, state);
                },
                machineHardware.steam_level_debounce_checks, 1);
        if (_config.capabilites.dualBoiler) {
            if (machineHardware.tank_level_enabled && _config.tankLevelPin != 0)
                tankLevel = std::make_unique<DigitalInput>(_config.tankLevelPin, [this](bool full) { tankLevelOK = full; });
        }
        if (_config.ledPin != 0)
            lights = std::make_unique<PwmLight>(_config.ledPin, 0);
        if (_config.baristaLedPin != 0)
            baristaLight = std::make_unique<PwmLight>(_config.baristaLedPin, 1);
    } else {
        alt = std::make_unique<SimpleRelay>(_config.altPin, _config.altOn);
    }
    valve = std::make_unique<SimpleRelay>(_config.valvePin, _config.valveOn);
    if (_config.capabilites.dimming) {
        pump = std::make_unique<DimmedPump>(_config.pumpPin, _config.pumpSensePin, pressureSensor.get(),
                                            _config.pressureControlRate);
    } else {
        pump = std::make_unique<SimplePump>(_config.pumpPin, _config.pumpOn, _config.capabilites.ssrPump ? 1000.0f : 5000.0f);
    }
    brewBtn =
        std::make_unique<DigitalInput>(_config.brewButtonPin, [this](const bool state) { _comms.sendButtonState(0, state); });
    steamBtn =
        std::make_unique<DigitalInput>(_config.steamButtonPin, [this](const bool state) { _comms.sendButtonState(1, state); });
    if (_config.waterButtonPin != 0) {
        waterBtn = std::make_unique<DigitalInput>(_config.waterButtonPin,
                                                  [this](const bool state) { _comms.sendButtonState(2, state); });
    }
#if defined(METER_LINK_PORT) && METER_LINK_PORT == 5
    // J5 carries the meter link in this build; its HX711 pins are the UART, so no hardware scale.
    ESP_LOGI(LOG_TAG, "Meter link on J5: hardware scale disabled");
#else
    this->hardwareScale = std::make_unique<HardwareScale>(
        _config.scaleDat0Pin, _config.scaleDat1Pin, _config.scaleClkPin,
        [this](float weight, float cell1Weight, float cell2Weight, bool cell1Valid, bool cell2Valid) {
            if (_comms.isConnected()) {
                _comms.sendScaleMeasurement(weight, cell1Weight, cell2Weight, cell1Valid, cell2Valid);
            }
        },
        [](float, float) { /* scale config changes are not forwarded */ });
#endif

    setupMeterLink();
    setupScaleLink();

    // 4-Pin peripheral port
    // J4 build: the pins are the meter link. With METER_LINK_ALBA the Alba hangs on the meter board: tank
    // distance arrives as $T and the display's LED commands leave as $L. With the scale board on J4 the Alba bus is
    // not started either.
    const bool j4IsMeterLink = (kMeterLinkPort == 4) && meterLink != nullptr;
    if (j4IsMeterLink && METER_LINK_ALBA) {
        _config.capabilites.ledControls = true;
        _config.capabilites.tof = true;
        meterLink->onTank([this](int distanceMm) {
            if (_comms.isConnected())
                _comms.sendTofMeasurement(static_cast<uint32_t>(distanceMm));
        });
    } else if (!j4IsMeterLink && scaleLink == nullptr && _config.sunriseSdaPin != 0 && _config.sunriseSclPin != 0) {
        albaComms = std::make_unique<SoftWire>(_config.sunriseSdaPin, _config.sunriseSclPin);
        albaComms->setTxBuffer(albaSwTxBuffer.data(), albaSwTxBuffer.size());
        albaComms->setRxBuffer(albaSwRxBuffer.data(), albaSwRxBuffer.size());
        albaComms->setTimeout_ms(200);
        albaComms->setDelay_us(20);
        albaComms->begin();
        albaBus = std::make_unique<SoftWireBus>(albaComms.get());
        this->ledController = std::make_unique<LedController>(albaBus.get());
        if (this->ledController->isAvailable()) {
            _config.capabilites.ledControls = true;
            // A configured Max tank switch owns tank reporting. Alba lighting
            // still works, but never starts a competing distance stream.
            if (tankLevel == nullptr) {
                distanceSensor = std::make_unique<DistanceSensor>(albaBus.get(),
                                                                  [this](int distance) { _comms.sendTofMeasurement(distance); });
                _config.capabilites.tof = true;
            }
        }
    }
    if (!_config.capabilites.dualBoiler && !_config.capabilites.tof && _config.tankLevelPin > 0) {
        tankLevel = std::make_unique<DigitalInput>(
            _config.tankLevelPin,
            [this](const bool full) {
                tankLevelOK = full;
                sendTankLevel();
            },
            5);
    }
    gm::DeviceCapabilities capabilities = gaggimate_Capabilities_init_zero;
    capabilities.dimming = _config.capabilites.dimming;
    capabilities.pressure = _config.capabilites.pressure;
    capabilities.tof = _config.capabilites.tof || tankLevel != nullptr;
    capabilities.led_control = _config.capabilites.ledControls;
    capabilities.dual_boiler = _config.capabilites.dualBoiler;

    if (this->hardwareScale != nullptr) {
        this->hardwareScale->setup();
    }

    auto addAddon = [&capabilities](uint32_t type) {
        const auto maxAddons = sizeof(capabilities.addons) / sizeof(capabilities.addons[0]);
        if (capabilities.addons_count >= maxAddons) {
            return;
        }
        capabilities.addons[capabilities.addons_count] = gaggimate_Addon_init_zero;
        capabilities.addons[capabilities.addons_count].type = type;
        capabilities.addons_count++;
    };
    if (hasGearPump()) {
        addAddon(ADDON_GEARPUMP);
    }
    if (this->vescCan != nullptr) {
        addAddon(ADDON_VESC_CAN);
    }
    if (this->hardwareScale != nullptr && this->hardwareScale->isAvailable()) {
        addAddon(ADDON_HW_SCALE);
    } else if (this->scaleLink != nullptr) {
        // The scale board may start after the Pro, so the display always gets the hardware-scale pages in this
        // build; without readings the display treats the scale as absent (its 1.5 s health timeout).
        addAddon(ADDON_HW_SCALE);
    }

    // A display plugged onto the board talks over the ext header: ext4 = our RX, ext5 = our TX.
    // Fork: not with a meter link, which owns Serial1 (see kDisplayUartLink).
    if (kDisplayUartLink) {
        _comms.enableUart(_config.ext4Pin, _config.ext5Pin);
    } else {
        ESP_LOGI(LOG_TAG, "Wired display link off in this build (Serial1 is the meter link); display stays on BLE");
    }
    // Flash writes stall the control loops, so a UART update starts from all outputs off.
    _comms.onUpdateStart([this]() { stopOutputs(); });
    // Steam switch held at power-on opens the BLE pairing window; read it here since steamBtn->setup() runs later.
    _comms.init("GPBLS", _config.name.c_str(), _version, capabilities, isBondingSwitchOn());

    if (_config.capabilites.ledControls && ledController != nullptr) {
        ledController->setup();
    }
    if (_config.capabilites.tof && distanceSensor != nullptr) {
        distanceSensor->setup();
    }
    if (meterLink != nullptr) {
        meterLink->setup();
    }
    if (scaleLink != nullptr) {
        scaleLink->setup();
    }

    if (this->gearpumpAddon != nullptr) {
        this->gearpumpAddon->setup(this->pump->getPumpPowerPtr());
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        dimmedPump->setBinaryMode(true);
    }
    setupVescCan();
    if (_config.capabilites.pressure || _config.capabilites.dualBoiler) {
        pressureSensor->setup();
        this->adc->setup();
        _comms.onPressureScale([this](float scale) { this->pressureSensor->setScale(scale); });
    }
    _comms.onLedControl(
        [this](uint8_t channel, uint8_t brightness, LedEffect effect, uint8_t minBrightness, uint16_t cycleTimeMs) {
            if (channel == SYSTEM_LED_CHANNEL && lights)
                lights->set(effect, brightness, minBrightness, cycleTimeMs);
            else if (channel == BARISTA_LED_CHANNEL && baristaLight)
                baristaLight->set(effect, brightness, minBrightness, cycleTimeMs);
            else if (channel < SYSTEM_LED_CHANNEL && ledController)
                ledController->setChannel(channel, brightness);
            else if (channel < SYSTEM_LED_CHANNEL && meterLink && kMeterLinkPort == 4 && METER_LINK_ALBA)
                meterLink->sendLed(channel, brightness); // fork: Alba on the meter board ($L)
        });
    brewTemperature->setup();
    heater->setup();
    valve->setup();
    pump->setup();
    brewBtn->setup();
    steamBtn->setup();
    if (waterBtn != nullptr) {
        waterBtn->setup();
    }
    if (_config.capabilites.dualBoiler) {
        steamTemperature->setup();
        heater2->setup();
        refill->setup();
        aux->setup();
        if (waterSense)
            waterSense->setup();
        if (lights)
            lights->setup();
        if (baristaLight)
            baristaLight->setup();
    } else {
        alt->setup();
    }
    if (tankLevel != nullptr) {
        tankLevel->setup();
    }
    // Set up thermal feedforward for main heater if pressure/dimming capability exists
    if (heater && _config.capabilites.dimming && _config.capabilites.pressure) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        float *pumpFlowPtr = dimmedPump->getPumpFlowPtr();
        int *valveStatusPtr = dimmedPump->getValveStatusPtr();

        heater->setThermalFeedforward(pumpFlowPtr, 23.0f, valveStatusPtr);
        heater->setFeedforwardScale(0.0f);
    }
    // Initialize last ping time
    lastPingTime = millis();

    // Output control is split into per-component, device-numbered messages. Each
    // arrives independently (or batched together in one frame for an atomic
    // update). Control messages feed the connection watchdog via handlePing().
    _comms.onBoilerControl([this](uint8_t index, BoilerControlMode mode, float setpoint) {
        if (!_config.capabilites.dualBoiler && index > 0) {
            return;
        }
        if (_config.capabilites.dualBoiler && !hardwareSessionReady)
            return;
        Heater *target = index == 0 ? this->heater.get() : this->heater2.get();
        handlePing();
        if (errorState != ERROR_CODE_NONE || _comms.isUartUpdating()) {
            return;
        }
        if (mode == BoilerControlMode::Temperature) {
            if (_config.capabilites.dualBoiler && index == 1) {
                steamRequestedSetpoint = setpoint;
                target->setSetpoint(machineHardware.steam_level_enabled && (!steamLevelKnown || steamLevelLow) ? 0.0f : setpoint);
            } else {
                target->setSetpoint(setpoint);
            }
        } else {
            // Pressure-regulated boiler not supported by this hardware yet.
            ESP_LOGW(LOG_TAG, "Boiler pressure mode requested but unsupported");
        }
    });
    _comms.onPumpControl([this](uint8_t index, PumpControlMode mode, float power, float pressure, float flow) {
        if (index != 0) { // single pump today; reject unknown devices
            ESP_LOGW(LOG_TAG, "Ignoring pump control for unsupported index %u", index);
            return;
        }
        handlePing();
        if (errorState != ERROR_CODE_NONE || _comms.isUartUpdating()) {
            return;
        }
        if (mode == PumpControlMode::Power) {
            this->pump->setPower(power); // the gear pump task applies a new power, including 0, within 30 ms
            updateScaleActivity();
            return;
        }
        if (!_config.capabilites.dimming) {
            return;
        }
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        if (mode == PumpControlMode::Pressure) {
            dimmedPump->setPressureTarget(pressure, flow);
        } else { // PumpControlMode::Flow
            dimmedPump->setFlowTarget(flow, pressure);
        }
    });
    // Binary outputs: index 0 = brew valve, index 1 = alt relay,
    // index 2 = steam refill, index 3 = steam-pressure hot-water valve.
    _comms.onRelayControl([this](uint8_t index, bool open) {
        handlePing();
        if (errorState != ERROR_CODE_NONE || _comms.isUartUpdating()) {
            return;
        }
        if (_config.capabilites.dualBoiler) {
            switch (index) {
            case 2:
                this->refill->set(open && hardwareSessionReady);
                return;
            case 3:
                this->aux->set(open);
                return;
            default:
                // noop
                break;
            }
        } else if (index == 1) {
            // Alt relay: independent function, no watchdog/error gating (matches
            // the previous dedicated alt-control path).
            this->alt->set(open);
            return;
        }
        if (index != 0) {
            ESP_LOGW(LOG_TAG, "Ignoring relay control for unsupported index %u", index);
            return;
        }
        this->valve->set(open);
        if (_config.capabilites.dimming) {
            static_cast<DimmedPump *>(pump.get())->setValveState(open);
        }
        if (meterLink != nullptr) {
            meterLink->sendBrewing(open);
        }
        updateScaleActivity(); // $B before the tare / shot end GaggiMate sends when a process ends
    });
    _comms.onMachineHardware(
        [this](const gm::MachineHardwareSettings &settings) { xQueueOverwrite(hardwareRequests, &settings); });
    // Heaters stay off until the display delivers these; coordination is applied before the gains enable heating.
    _comms.onHeaterSettings([this](const gm::HeaterSettings &settings) {
        if (heaterCoordinator != nullptr) {
            heaterCoordinator->configure(settings.heater_coordination_enabled, settings.handover_ms);
        }
        heater->setFeedforwardScale(settings.kf);
        heater->setTunings(settings.kp, settings.ki, settings.kd);
        if (heater2 != nullptr) {
            heater2->setTunings(settings.steam_kp, settings.steam_ki, settings.steam_kd);
        }
    });
    _comms.onPumpSettings([this](gm::PumpSettings settings) {
        if (_config.capabilites.dimming) {
            auto dimmedPump = static_cast<DimmedPump *>(pump.get());
            // Check if this is a flow measurement call (a and b are flow measurements, c and d are nan)
            if (isnan(settings.c) && isnan(settings.d)) {
                dimmedPump->setPumpFlowCoeff(settings.a, settings.b); // a = oneBarFlow, b = nineBarFlow
            } else {
                dimmedPump->setPumpFlowPolyCoeffs(settings.a, settings.b, settings.c,
                                                  settings.d); // a, b, c, d are polynomial coefficients
            }
            if (hasGearPump()) {
                dimmedPump->setGains(settings.commutationGain, settings.convergenceGain, settings.integralGain);
                // Slip model is only meaningful for the positive-displacement gear/rotary-vane pump.
                dimmedPump->setPumpSlipPolyCoeffs(settings.slipA, settings.slipB, settings.slipC, settings.slipD);
                dimmedPump->setMinPower(settings.minBLDCPower * 100.0f);
            }
        }
        if (this->gearpumpAddon != nullptr) {
            gearpumpAddon->setMaxPower(settings.maxBLDCPower);
        }
        if (this->vescCan != nullptr) {
            vescCan->setMaxPower(settings.maxBLDCPower); // scales the pump power before it becomes a speed or duty
        }
    });
    // Fork (maxa4turbo.15): second pump on/off and the motor temperature limit of the CAN pump drive.
    // maxa4turbo.16: speed or duty control and the ERPM at 100 % (0 = a display that does not send it: keep ours).
    _comms.onVescSettings([this](bool dualPump, float motorTempLimit, uint32_t maxErpm, bool dutyMode) {
        if (this->vescCan == nullptr)
            return;
        vescCan->setDualPump(dualPump);
        vescCan->setMotorTempLimit(motorTempLimit);
        if (maxErpm > 0)
            vescCan->setMaxErpm(static_cast<int32_t>(std::min<uint32_t>(maxErpm, vesc::MAX_ERPM_HIGHEST)));
        vescCan->setSpeedMode(!dutyMode);
    });
    _comms.onPing([this]() { handlePing(); });
    // A new link gets a full ping window; a stale lastPingTime would trip the watchdog while the display is still
    // discovering services. errorState is left alone so a timed-out session only recovers on a real ping.
    _comms.onConnectionChange([this](bool connected) {
        if (connected) {
            lastPingTime = millis();
            controllerErrorPending = true;
        }
    });
    _comms.onAutotune([this](uint32_t testTimeSec, uint32_t windowSize, uint32_t heaterWattage) {
        if (_config.capabilites.dualBoiler && !hardwareSessionReady)
            return;
        handlePing();
        if (errorState != ERROR_CODE_NONE || _comms.isUartUpdating()) { // don't re-engage the heater while faulted
            return;
        }
        this->heater->autotune(static_cast<int>(testTimeSec), static_cast<int>(windowSize), static_cast<int>(heaterWattage));
    });
    // Hardware-scale tare (#985): brew about to start, grind start, Smart Dose, the tare buttons.
    _comms.onHardwareScaleTare([this]() {
        if (hardwareScale != nullptr && hardwareScale->isAvailable()) {
            hardwareScale->tare();
        }
        if (scaleLink != nullptr) {
            scaleLink->sendTare();
        }
    });
    // Pump-volume reset: a process starts, a non-shot process ends, or a finished shot was saved.
    _comms.onTare([this]() {
        if (meterLink != nullptr) {
            meterLink->sendZero(); // zero the meter board's volume counter, as the pumped-water figure is
        }
        if (scaleLink != nullptr) {
            scaleLink->sendVolumeReset(); // $E: the scale board closes a finished shot without zeroing the cup
        }
        if (!_config.capabilites.dimming) {
            return;
        }
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        dimmedPump->tare();
    });
    _comms.onScaleFactors(
        [this](float scaleFactor1, float scaleFactor2, uint16_t sampleRateSps, float idleFilterAlpha, float activeFilterAlpha) {
            if (scaleLink != nullptr) {
                scaleLink->setConfiguration(scaleFactor1, scaleFactor2, sampleRateSps, idleFilterAlpha, activeFilterAlpha);
            }
            if (hardwareScale == nullptr || !hardwareScale->isAvailable()) {
                return;
            }
            hardwareScale->setConfiguration(scaleFactor1, scaleFactor2, sampleRateSps, idleFilterAlpha, activeFilterAlpha);
        });
    ESP_LOGI(LOG_TAG, "Initialization done");
}

void GaggiMateController::loop() {
    gm::MachineHardwareSettings requested;
    bool hardwareStatusPending = xQueueReceive(hardwareRequests, &requested, 0) == pdTRUE;
    if (hardwareStatusPending)
        storeMachineHardware(requested);
    if (hardwareConfigurationAccepted && !hardwareConfigurationError && !hardwareSessionReady &&
        !brewTemperature->isErrorState() && !steamTemperature->isErrorState()) {
        hardwareSessionReady = true;
        hardwareStatusPending = true;
    }
    if (hardwareStatusPending)
        _comms.sendMachineHardwareStatus(machineHardwareHash(machineHardware), hardwareSessionReady, hardwareConfigurationError);
    // Report a pre-existing fault after the encrypted comms session is established.
    // NONE also clears the display's old fault after a successful controller restart.
    if (_comms.isConnected() && controllerErrorPending.exchange(false))
        _comms.sendError(errorState);
    unsigned long now = millis();

    // Keep zero tracking and idle display quantization out of the measurement
    // path whenever the brew valve or pump can be adding liquid to the cup.
    updateScaleActivity();

    if (lastPingTime < now && (now - lastPingTime) / 1000 > PING_TIMEOUT_SECONDS) {
        handlePingTimeout();
    }
    if (_config.capabilites.dualBoiler)
        updateDualBoilerOutputs(now);
    if (!_config.capabilites.dualBoiler || lastSensorSend == 0 || now - lastSensorSend >= 250) {
        sendSensorData();
        sendVescStatus();
        lastSensorSend = now;
    }
    if (_config.capabilites.ledControls && ledController != nullptr && now - lastLedHealthCheck >= LED_HEALTH_CHECK_INTERVAL_MS) {
        lastLedHealthCheck = now;
        ledController->healthCheck();
    }
    if (meterLink != nullptr && _config.capabilites.dimming) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        if (dimmedPump->isFlowLoopFaulted() && now - lastFlowFaultLog >= 1000) {
            lastFlowFaultLog = now;
            ESP_LOGE(LOG_TAG, "FLOW FAULT: flow phase wants %.2f ml/s but no meter data for %lu ms -> pump held at 0",
                     dimmedPump->getFlowTarget(), static_cast<unsigned long>(meterLink->flowAgeMs()));
        }
    }
    if (tankLevel != nullptr && now - lastTankLevelSend >= TANK_LEVEL_SEND_INTERVAL_MS) {
        sendTankLevel();
    }
    logBleHeadroom(now);
    if (errorState != ERROR_CODE_NONE && errorState != ERROR_CODE_TIMEOUT) {
        ESP_LOGW("GaggiMateController", "Error state: %d", errorState);
    }
    delay(_config.capabilites.dualBoiler ? 50 : 250);
    if (Serial.available()) {
        while (Serial.available()) {
            char c = Serial.read();
            handleSerialCommand(c);
        }
    }
}

// Fork (maxa4turbo.17): one log line whenever one of the two Bluetooth tasks has used at least 64 bytes more of
// its stack than at the last line (BleHeadroom.h). The peaks only grow, so the lines stop once the link has seen
// its deepest moment. They are the check on the maxa4turbo.16 reboot loop: there the NimBLE host task ran out of
// its 4096 bytes while it acknowledged the display's first frame.
void GaggiMateController::logBleHeadroom(unsigned long now) {
    static constexpr unsigned long CHECK_INTERVAL_MS = 1000;
    static constexpr uint16_t STEP_BYTES = 64;
    if (now - lastBleHeadroomCheck < CHECK_INTERVAL_MS)
        return;
    lastBleHeadroomCheck = now;
    const BleHeadroom headroom = bleHeadroom();
    if (headroom.hostStackSize == 0)
        return;
    if (headroom.hostStackPeak < bleHostPeakLogged + STEP_BYTES &&
        headroom.controllerStackPeak < bleControllerPeakLogged + STEP_BYTES)
        return;
    bleHostPeakLogged = headroom.hostStackPeak;
    bleControllerPeakLogged = headroom.controllerStackPeak;
    ESP_LOGI(LOG_TAG, "Bluetooth stacks, most used so far: NimBLE host task %u of %u bytes, controller task %u of %u bytes",
             headroom.hostStackPeak, headroom.hostStackSize, headroom.controllerStackPeak, headroom.controllerStackSize);
}

// Reed switch closed (state true) means water present, so report a full-tank distance.
void GaggiMateController::sendTankLevel() {
    lastTankLevelSend = millis();
    _comms.sendTofMeasurement(tankLevel->getState() ? 10 : 1000);
}

void GaggiMateController::registerBoardConfig(ControllerConfig config) { configs.push_back(config); }

// Fork: serial link to the flow-meter board. Only on boards with a dimmed pump (the Pro); the
// link feeds the DimmedPump's measured-flow loop and can carry Alba traffic (J4 build).
void GaggiMateController::setupMeterLink() {
#ifdef METER_LINK_PORT
    if (!_config.capabilites.dimming) {
        ESP_LOGW(LOG_TAG, "Meter link needs a dimmed/BLDC pump board; not enabled on %s", _config.name.c_str());
        return;
    }
#if METER_LINK_PORT == 4
    const uint8_t rxPin = _config.sunriseSclPin; // J4 "RX" = gpio44 on the Pro
    const uint8_t txPin = _config.sunriseSdaPin; // J4 "TX" = gpio43
    const char *port = "J4";
#else
    const uint8_t rxPin = _config.scaleDat0Pin; // J5 "SDA"/DT1 = gpio18 on the Pro
    const uint8_t txPin = _config.scaleClkPin;  // J5 "SCL"/SCK = gpio17
    const char *port = "J5";
#endif
    meterLink = std::make_unique<MeterLink>(rxPin, txPin, 1);
    auto dimmedPump = static_cast<DimmedPump *>(pump.get());
    dimmedPump->setMeterLink(meterLink.get());
    {
        // Max's Velofuso: the loop acts at half gain below 25 % pump power (meterlink.10, see FlowLoopGains::knee).
        // $G from the meter board keeps these (it only sets its 8 fields).
        FlowLoopGains g = dimmedPump->getFlowLoopGains();
        g.knee = 0.25f;
        g.kneeGain = 0.5f;
        dimmedPump->setFlowLoopGains(g);
    }
    meterLink->onGains([this](const FlowLoopGains &gains) { static_cast<DimmedPump *>(pump.get())->setFlowLoopGains(gains); });
    meterLink->currentGains([this]() { return static_cast<DimmedPump *>(pump.get())->getFlowLoopGains(); });
    // $D: the pump's diagnostics plus the brew heater and boiler temperature, so the meter board's shot log shows
    // heat input next to pressure and flow (cached values, safe to read from the link task).
    meterLink->onDiagnostics([this](MeterDiagnostics &d) {
        static_cast<DimmedPump *>(pump.get())->fillDiagnostics(d);
        d.heaterPct = heater != nullptr ? heater->getDutyCycle() : 0.0f;
        d.boilerC = brewTemperature != nullptr ? brewTemperature->read() : 0.0f;
        return true;
    });
    ESP_LOGI(LOG_TAG, "Meter link configured on %s (RX gpio%u, TX gpio%u), Alba over link: %d", port, rxPin, txPin,
             METER_LINK_ALBA);
#endif
}

// Brew valve open or pump running: the stock HardwareScale's brewing flag, and $B to the scale board. Called from
// the loop and (fork) right after valve/pump commands, so the board sees the change before a following tare.
void GaggiMateController::updateScaleActivity() {
    if (hardwareScale == nullptr && scaleLink == nullptr)
        return;
    const float pumpPower = pump != nullptr && pump->getPumpPowerPtr() != nullptr ? *pump->getPumpPowerPtr() : 0.0f;
    const bool scaleActivity = (valve != nullptr && valve->getState()) || pumpPower > 0.01f ||
                               (_config.capabilites.dualBoiler && aux != nullptr && aux->getState());
    if (hardwareScale != nullptr)
        hardwareScale->setBrewingActive(scaleActivity);
    if (scaleLink != nullptr)
        scaleLink->setActive(scaleActivity); // sent as $B; the board runs the same stock filter logic
}

// Fork (meterlink.12): serial link to the scale board on J4. Its readings go to the display through the same
// sendScaleMeasurement() the stock HX711 code uses, so the display's weight, stop-at-weight and Hardware Scale
// settings work unchanged.
void GaggiMateController::setupScaleLink() {
#ifdef SCALE_LINK_PORT
    if (_config.sunriseSdaPin == 0 || _config.sunriseSclPin == 0 || _config.capabilites.dualBoiler) {
        ESP_LOGW(LOG_TAG, "Scale link needs J4; not enabled on %s", _config.name.c_str());
        return;
    }
    const uint8_t rxPin = _config.sunriseSclPin; // J4 "RX" = gpio44 on the Pro
    const uint8_t txPin = _config.sunriseSdaPin; // J4 "TX" = gpio43
    scaleLink = std::make_unique<ScaleLink>(rxPin, txPin, 2);
    scaleLink->onReading([this](const scalewire::Reading &r) {
        if (_comms.isConnected()) {
            _comms.sendScaleMeasurement(r.grams, r.cell1, r.cell2, r.cell1Valid, r.cell2Valid);
        }
    });
    ESP_LOGI(LOG_TAG, "Scale link configured on J4 (RX gpio%u, TX gpio%u); Alba bus not started", rxPin, txPin);
#endif
}

void GaggiMateController::detectBoard() {
    constexpr int MAX_DETECT_RETRIES = 3;
    pinMode(DETECT_EN_PIN, OUTPUT);
    pinMode(DETECT_VALUE_PIN, INPUT_PULLDOWN);

    for (int attempt = 0; attempt < MAX_DETECT_RETRIES; attempt++) {
        digitalWrite(DETECT_EN_PIN, HIGH);
        delay(10); // Allow voltage to stabilize before ADC read
        uint16_t millivolts = analogReadMilliVolts(DETECT_VALUE_PIN);
        digitalWrite(DETECT_EN_PIN, LOW);
        int boardId = round(((float)millivolts) / 100.0f - 0.5f);
        ESP_LOGI(LOG_TAG, "Board detect attempt %d/%d: ID=%d (raw: %d mV)", attempt + 1, MAX_DETECT_RETRIES, boardId, millivolts);
        for (ControllerConfig config : configs) {
            if (config.autodetectValue == boardId) {
                _config = config;
                ESP_LOGI(LOG_TAG, "Using Board: %s", _config.name.c_str());
                return;
            }
        }
        ESP_LOGW(LOG_TAG, "No match on attempt %d, retrying...", attempt + 1);
        delay(500);
    }
    ESP_LOGE(LOG_TAG, "No compatible board detected after %d attempts. Restarting...", MAX_DETECT_RETRIES);
    delay(5000);
    ESP.restart();
}

void GaggiMateController::updateDualBoilerOutputs(unsigned long now) {
    const bool safe = errorState == ERROR_CODE_NONE && hardwareSessionReady;
    const float target = safe && (!machineHardware.steam_level_enabled || (steamLevelKnown && !steamLevelLow))
                             ? steamRequestedSetpoint.load()
                             : 0.0f;
    if (heater2->getSetpoint() != target)
        heater2->setSetpoint(target);
    const bool fault = !safe || (machineHardware.tank_level_enabled && !tankLevelOK);
    if (lights)
        lights->update(now, fault, safe);
    if (baristaLight)
        baristaLight->update(now, false, safe);
}

// Fork (maxa4turbo.15): start the CAN pump drive (see VescCan.h). The pump controller keeps writing one pump
// power; the CAN task turns it into a speed (maxa4turbo.16) or a duty for one or two VESCs, as the adapter's task
// turns it into a voltage.
void GaggiMateController::setupVescCan() {
    if (this->vescCan == nullptr)
        return;
    this->vescCan->onTrip([this](vesc::Trip trip, uint8_t pumpNumber, int32_t detail) {
        // Runs in the CAN task; the pumps are already at 0 A. The display ends the shot on this message.
        const int code = ERROR_CODE_PUMP_STALL + (static_cast<int>(trip) - static_cast<int>(vesc::Trip::STALL));
        if (isPumpStopCode(code) && _comms.isConnected())
            _comms.sendPumpStop(code, pumpNumber, detail);
    });
    if (!this->vescCan->setup(this->pump->getPumpPowerPtr())) {
        ESP_LOGE(LOG_TAG, "CAN pump drive did not start; the pump will not run until it does");
    }
    // Same as with the adapter: the pump controller's power goes to the drive, the mains output is on or off.
    static_cast<DimmedPump *>(pump.get())->setBinaryMode(true);
}

// Fork (maxa4turbo.15): pump readings for the display's web UI, with the sensor data four times a second.
void GaggiMateController::sendVescStatus() {
    if (this->vescCan == nullptr || !_comms.isConnected())
        return;
    const VescCan::Snapshot s = vescCan->snapshot();
    VescStatusReport report;
    report.pumpCount = vesc::MAX_PUMPS;
    for (size_t i = 0; i < vesc::MAX_PUMPS; i++) {
        const vesc::PumpReadings &p = s.pumps[i];
        VescPumpReport &r = report.pumps[i];
        r.state = static_cast<uint8_t>(s.states[i]);
        r.erpm = p.erpm;
        r.motorCurrent = p.motorCurrent;
        r.duty = p.duty;
        r.fetTemp = p.fetTemp;
        r.motorTemp = p.motorTemp;
        r.inputCurrent = p.inputCurrent;
        r.inputVoltage = p.inputVoltage;
        r.fault = p.fault;
        r.hasSpeed = s.freshSpeed[i];
        r.hasTemps = s.freshTemps[i];
        r.hasVoltage = s.freshVoltage[i];
        r.hasFault = s.freshFault[i];
    }
    report.commandDuty = s.commandDuty;
    report.commandErpm = s.commandErpm;
    report.speedMode = s.speedMode;
    report.maxErpm = static_cast<uint32_t>(s.maxErpm);
    report.trip = static_cast<uint8_t>(s.trip);
    report.tripPump = s.tripPump;
    report.busState = s.busState;
    report.txFailed = s.txFailed;
    report.dualPump = s.dualPump;
    report.motorTempLimit = s.motorTempLimit;
    _comms.sendVescStatus(report);
}

void GaggiMateController::detectAddon() {
    const bool sharedAdcBus = (_config.capabilites.pressure || _config.capabilites.dualBoiler) &&
                              _config.ext3Pin == _config.pressureSda && _config.ext2Pin == _config.pressureScl;
    TwoWire &bus = sharedAdcBus ? Wire1 : Wire;
    const bool busReady = bus.begin(_config.ext3Pin, _config.ext2Pin, ADC_I2C_CLOCK_HZ);
    if (!busReady) {
        ESP_LOGE(LOG_TAG, "Could not initialize addon bus");
        if (!kVescCan)
            return;
    }
    for (uint8_t addr = XL9555_SLAVE_ADDRESS0; busReady && addr <= XL9555_SLAVE_ADDRESS7; ++addr) {
        bus.beginTransmission(addr);
        if (bus.endTransmission() == 0) {
            ESP_LOGI(LOG_TAG, "Found an extension at address 0x%X", addr);
            if (addr == 0x26) {
                ESP_LOGI(LOG_TAG, "Identified addon as Gearpump Addon");
                gearpumpAddon =
                    std::make_unique<GearpumpAddon>(addr, _config.ext3Pin, _config.ext2Pin, _config.ext1Pin, bus, sharedAdcBus);
            }
        }
    }
    // Fork (maxa4turbo.15): one missed answer must not flip a machine with the adapter over to CAN; ask twice more.
    for (int attempt = 0; kVescCan && busReady && gearpumpAddon == nullptr && attempt < 2; attempt++) {
        delay(30);
        bus.beginTransmission(0x26);
        if (bus.endTransmission() == 0) {
            ESP_LOGI(LOG_TAG, "Identified addon as Gearpump Addon (retry %d)", attempt + 1);
            gearpumpAddon =
                std::make_unique<GearpumpAddon>(0x26, _config.ext3Pin, _config.ext2Pin, _config.ext1Pin, bus, sharedAdcBus);
        }
    }
    // No Velofuso adapter on J6 means the pump hangs on the Flipsky; talk to it over CAN.
    if (kVescCan && gearpumpAddon == nullptr && _config.capabilites.dimming) {
        ESP_LOGI(LOG_TAG, "No Velofuso adapter on J6: pump drive is the Flipsky over CAN (TX gpio%u, RX gpio%u)", _config.ext5Pin,
                 _config.ext4Pin);
        vescCan = std::make_unique<VescCan>(_config.ext5Pin, _config.ext4Pin);
    } else if (kVescCan && gearpumpAddon != nullptr) {
        ESP_LOGI(LOG_TAG, "Velofuso adapter found on J6: analog pump output, CAN pump drive not started");
    }
}

void GaggiMateController::handlePing() {
    if (errorState == ERROR_CODE_TIMEOUT) {
        errorState = ERROR_CODE_NONE;
    }
    lastPingTime = millis();
    ESP_LOGV(LOG_TAG, "Ping received, system is alive");
}

void GaggiMateController::clearRetainedOutputDemands() { steamRequestedSetpoint = 0.0f; }

void GaggiMateController::stopOutputs() {
    clearRetainedOutputDemands();
    this->heater->stop();
    this->pump->setPower(0);
    this->valve->set(false);
    if (_config.capabilites.dualBoiler) {
        this->heater2->stop();
        this->refill->set(false);
        this->aux->set(false);
        if (baristaLight)
            baristaLight->stop();
    } else {
        this->alt->set(false);
    }
}

void GaggiMateController::handlePingTimeout() {
    // Turn off the heater and pump as a safety measure
    stopOutputs();
    // A communication timeout must not turn a latched thermal fault into a recoverable error.
    if (errorState != ERROR_CODE_NONE && errorState != ERROR_CODE_TIMEOUT)
        return;
    // On the healthy->timeout transition, drop the BLE link. An in-band error
    // notify can be silently swallowed on a wedged GATT (the failure mode this
    // is here to recover from), but LL_TERMINATE_IND propagates reliably at
    // the link layer. The display's existing disconnect path then rebuilds
    // the link and re-sends control state. loop() re-enters every 250 ms while
    // timed out -- guard so we don't repeatedly bounce the connection or spam
    // the log.
    if (errorState != ERROR_CODE_TIMEOUT) {
        ESP_LOGE(LOG_TAG, "Ping timeout detected. Turning off heater and pump for safety.");
        if (!_comms.isUpdating())
            _comms.disconnect();
    }
    errorState = ERROR_CODE_TIMEOUT;
}

void GaggiMateController::thermalRunawayShutdown() {
    if (errorState == ERROR_CODE_RUNAWAY)
        return;
    clearRetainedOutputDemands();
    ESP_LOGE(LOG_TAG, "Thermal runaway detected! Turning off heater and pump!\n");
    // Turn off the heater and pump immediately
    this->heater->stop();
    this->pump->setPower(0);
    this->valve->set(false);
    if (_config.capabilites.dualBoiler) {
        this->heater2->stop();
        this->refill->set(false);
        this->aux->set(false);
        if (baristaLight)
            baristaLight->stop();
    } else {
        this->alt->set(false);
    }
    errorState = ERROR_CODE_RUNAWAY;
    _comms.sendError(ERROR_CODE_RUNAWAY);
}

void GaggiMateController::sendSensorData() {
    const float pumpPower = *pump->getPumpPowerPtr();
    const float heaterPower = heater ? heater->getDutyCycle() : 0.0f;
    float puckFlow = 0.0f;
    float pumpFlow = 0.0f;
    float puckResistance = 0.0f;
    float pressure = 0.0f;
    float waterPumped = 0.0f;
    gm::Payload batch[2];
    size_t n = 0;
    if (_config.capabilites.pressure) {
        pressure = pressureSensor->getPressure();
        if (_config.capabilites.dimming) {
            auto dimmedPump = static_cast<DimmedPump *>(pump.get());
            puckFlow = dimmedPump->getPuckFlow();
            pumpFlow = dimmedPump->getPumpFlow();
            puckResistance = dimmedPump->getPuckResistance();
            waterPumped = dimmedPump->getPumpedWater();
            if (this->valve->getState()) {
                batch[n++] = _comms.buildVolumetricMeasurement(dimmedPump->getCoffeeVolume());
            }
        }
    }
    gm::Payload p = gaggimate_Payload_init_zero;
    p.which_content = gaggimate_Payload_sensor_tag;
    p.content.sensor.has_tank_level = tankLevel != nullptr;
    p.content.sensor.has_steam_boiler_low = waterSense != nullptr && steamLevelKnown.load();
    p.content.sensor.steam_boiler_low = steamLevelLow.load();
    p.content.sensor.tank_level_ok = tankLevelOK;
    p.content.sensor.boilers_count = _config.capabilites.dualBoiler ? 2 : 1; // boiler 0; schema allows more
    p.content.sensor.boilers[0].index = 0;
    p.content.sensor.boilers[0].temperature = this->brewTemperature->read();
    p.content.sensor.boilers[0].pressure = pressure;
    p.content.sensor.boilers[0].power = heaterPower;
    if (_config.capabilites.dualBoiler) {
        p.content.sensor.boilers[1].index = 1;
        p.content.sensor.boilers[1].temperature = this->steamTemperature->read();
        p.content.sensor.boilers[1].pressure = 0.0f;
    }
    p.content.sensor.puck_flow = puckFlow;
    p.content.sensor.pump_flow = pumpFlow;
    p.content.sensor.puck_resistance = puckResistance;
    p.content.sensor.pump_power = pumpPower;
    p.content.sensor.water_pumped = waterPumped;
    batch[n++] = p;
    _comms.sendUnreliableBatch(batch, n);
}

void GaggiMateController::loadMachineHardware() {
    machineHardware = defaultMachineHardware();
    Preferences prefs;
    if (prefs.begin("machine-hw", true)) {
        uint8_t bytes[sizeof(gm::MachineHardwareSettings) + 2 * sizeof(uint32_t)]{};
        const size_t size = prefs.getBytesLength("config");
        gm::MachineHardwareSettings stored = gaggimate_MachineHardwareSettings_init_zero;
        if (size <= sizeof(bytes) && prefs.getBytes("config", bytes, size) == size &&
            decodeStoredMachineHardware(bytes, size, stored)) {
            machineHardware = stored;
            hardwareConfigurationStored = size == sizeof(stored);
        }
        prefs.end();
    }
}

void GaggiMateController::storeMachineHardware(const gm::MachineHardwareSettings &settings) {
    if (!_config.capabilites.dualBoiler)
        return;
    if (!validMachineHardware(settings)) {
        hardwareConfigurationError = true;
        return;
    }
    const bool changed = machineHardwareHash(settings) != machineHardwareHash(machineHardware);
    if (changed) {
        hardwareConfigurationAccepted = false;
        hardwareSessionReady = false;
        heater->setSetpoint(0);
        heater->stop();
        heater2->setSetpoint(0);
        heater2->stop();
        pump->setPower(0);
        refill->set(false);
    }
    if (changed || !hardwareConfigurationStored) {
        auto stored = settings;
        stored.restart_if_required = false;
        Preferences prefs;
        if (!prefs.begin("machine-hw", false)) {
            hardwareConfigurationError = true;
            return;
        }
        const bool saved = prefs.putBytes("config", &stored, sizeof(stored)) == sizeof(stored);
        prefs.end();
        if (!saved) {
            hardwareConfigurationError = true;
            return;
        }
        hardwareConfigurationStored = true;
        ESP_LOGI("MachineHardware", "Saved configuration hash=%lu, structural=%d", (unsigned long)machineHardwareHash(settings),
                 changed);
    }
    hardwareConfigurationError = false;
    if (changed) {
        if (settings.restart_if_required) {
            delay(300);
            ESP.restart();
        }
        return;
    }
    machineHardware = settings;
    hardwareConfigurationAccepted = true;
    // Configuration remains writable while faulted; applying corrected NTC parameters
    // restarts the controller. Sensor readiness does not suppress fault reporting.
    hardwareSessionReady = hardwareSessionReady || (!brewTemperature->isErrorState() && !steamTemperature->isErrorState());
}

void GaggiMateController::handleSerialCommand(char c) {
    // Console on the USB port. 'S' alone still prints the status (as before); other commands
    // are whole lines: "G" shows the flow-loop gains, "G kp ki start% ff ramp limKp limKi [deadzone%]" sets them.
    if (c == '\r' || c == '\n') {
        if (consoleLen > 0) {
            consoleLine[consoleLen] = '\0';
            handleConsoleLine(consoleLine);
            consoleLen = 0;
        }
        return;
    }
    if (consoleLen == 0 && c == 'S') {
        printStatus();
        return;
    }
    if (consoleLen < sizeof(consoleLine) - 1) {
        consoleLine[consoleLen++] = c;
    } else {
        consoleLen = 0;
    }
}

void GaggiMateController::handleConsoleLine(const char *line) {
    if (line[0] == 'S' && line[1] == '\0') {
        printStatus();
        return;
    }
    if (line[0] == 'G' && _config.capabilites.dimming) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        FlowLoopGains g = dimmedPump->getFlowLoopGains();
        if (line[1] == '\0') {
            ESP_LOGI("Controller",
                     "Flow loop gains: kp=%.4f ki=%.4f start=%.0f%% ff=%.2f ramp=%.1f limKp=%.2f limKi=%.2f deadzone=%.0f%% "
                     "knee=%.0f%% x%.2f",
                     g.kp, g.ki, g.startDuty * 100.0f, g.ff, g.ramp, g.limKp, g.limKi, g.deadZone * 100.0f, g.knee * 100.0f,
                     g.kneeGain);
            return;
        }
        float kp, ki, start, ff, ramp, limKp, limKi, dz = g.deadZone * 100.0f;
        const int got = sscanf(line + 1, "%f %f %f %f %f %f %f %f", &kp, &ki, &start, &ff, &ramp, &limKp, &limKi, &dz);
        if (got == 7 || got == 8) {
            g.kp = kp;
            g.ki = ki;
            g.startDuty = start / 100.0f;
            g.ff = ff;
            g.ramp = ramp;
            g.limKp = limKp;
            g.limKi = limKi;
            g.deadZone = dz / 100.0f;
            if (!g.valid()) {
                ESP_LOGI("Controller",
                         "Rejected: gains out of range (kp 0-1, ki 0-5, start 0-100, ff 0-1, ramp 0-100, lim 0-20, deadzone 0-60)");
                return;
            }
            dimmedPump->setFlowLoopGains(g);
            ESP_LOGI("Controller", "Flow loop gains set, in effect from the next flow phase (until restart or the next $G from the meter board)");
        } else {
            ESP_LOGI("Controller",
                     "Usage: G <kp> <ki> <start_duty_percent> <ff 0..1> <ramp ml/s/s> <limKp %%/bar> <limKi %%/bar/s> [deadzone %%]");
        }
        return;
    }
    ESP_LOGI("Controller", "Unrecognized Input! Available commands: S (Status), G (flow loop gains)");
}

void GaggiMateController::printStatus() {
    ESP_LOGI("Controller", "");
    ESP_LOGI("Controller", "╔════════════════╗");
    ESP_LOGI("Controller", "║ Status Summary ║");
    ESP_LOGI("Controller", "╠════════════════╝");
    ESP_LOGI("Controller", "║");
    ESP_LOGI("Controller", "╠═ Error codes");
    ESP_LOGI("Controller", "║  ├─ Controller Error: %d", errorState);
    ESP_LOGI("Controller", "║  └─ Thermocouple Error: %d", brewTemperature->isErrorState());
    ESP_LOGI("Controller", "║");
    ESP_LOGI("Controller", "╠═ Readings");
    if (_config.capabilites.pressure) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        ESP_LOGI("Controller", "║  ├─ Pressure: %.2f", pressureSensor->getPressure());
        ESP_LOGI("Controller", "║  ├─ Flow: %.2f", dimmedPump->getPumpFlow());
        ESP_LOGI("Controller", "║  ├─ Pump Power: %.2f", dimmedPump->getPowerTarget());
    }
    if (_config.capabilites.dualBoiler) {
        ESP_LOGI("Controller", "║  ├─ Steam Boiler Low: %.2f", waterSense ? waterSense->getState() : false);
        ESP_LOGI("Controller", "║  ├─ Temperature2: %.2f", steamTemperature->read());
    }
    ESP_LOGI("Controller", "║  └─ Temperature: %.2f", brewTemperature->read());
    ESP_LOGI("Controller", "║");
    ESP_LOGI("Controller", "╠═ Control");
    if (_config.capabilites.dualBoiler) {
        ESP_LOGI("Controller", "║  ├─ Refill: %d", refill->getState());
        ESP_LOGI("Controller", "║  ├─ Aux: %d", aux->getState());
        ESP_LOGI("Controller", "║  ├─ Steam Temperature: %d", heater2->getSetpoint());
    }
    if (_config.capabilites.pressure) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        ESP_LOGI("Controller", "║  ├─ Pressure: %.2f", dimmedPump->getPressureTarget());
        ESP_LOGI("Controller", "║  ├─ Flow: %.2f", dimmedPump->getFlowTarget());
        ESP_LOGI("Controller", "║  ├─ Pump Power: %.2f", dimmedPump->getPowerTarget());
    }
    ESP_LOGI("Controller", "║  └─ Temperature: %.2f", heater->getSetpoint());
    ESP_LOGI("Controller", "║");
    if (meterLink != nullptr && _config.capabilites.dimming) {
        auto dimmedPump = static_cast<DimmedPump *>(pump.get());
        const FlowLoopGains &g = dimmedPump->getFlowLoopGains();
        ESP_LOGI("Controller", "╠═ Meter link (RX gpio%u, TX gpio%u)", meterLink->rxPin(), meterLink->txPin());
        ESP_LOGI("Controller", "║  ├─ Link: %s, flow data: %s (age %lu ms)", meterLink->linkUp() ? "up" : "down",
                 meterLink->flowValid() ? "fresh" : "stale",
                 meterLink->flowAgeMs() == UINT32_MAX ? 0UL : static_cast<unsigned long>(meterLink->flowAgeMs()));
        ESP_LOGI("Controller", "║  ├─ Measured flow: %.2f ml/s, volume: %.1f ml, seq %lu", meterLink->flow(), meterLink->volume(),
                 static_cast<unsigned long>(meterLink->flowSeq()));
        ESP_LOGI("Controller", "║  ├─ Lines ok/bad: %lu/%lu, flow lines: %lu, duplicate seq: %lu, diagnostics sent: %lu",
                 static_cast<unsigned long>(meterLink->goodLines()), static_cast<unsigned long>(meterLink->badLines()),
                 static_cast<unsigned long>(meterLink->flowLines()), static_cast<unsigned long>(meterLink->duplicateLines()),
                 static_cast<unsigned long>(meterLink->diagnosticsSent()));
        ESP_LOGI("Controller", "║  ├─ Tank distance: %d mm", meterLink->tankDistance());
        ESP_LOGI("Controller", "║  ├─ Flow loop: mode %d, target %.2f -> %.2f ml/s%s, duty %.1f%%, fault %d",
                 static_cast<int>(dimmedPump->getMode()), dimmedPump->getFlowTarget(), dimmedPump->getEffectiveFlowTarget(),
                 dimmedPump->isPressureLimiting() ? " (ceiling)" : "", *dimmedPump->getPumpPowerPtr(),
                 dimmedPump->isFlowLoopFaulted());
        ESP_LOGI("Controller", "║  └─ Gains: kp=%.4f ki=%.4f start=%.0f%% ff=%.2f ramp=%.1f limKp=%.2f limKi=%.2f deadzone=%.0f%%", g.kp,
                 g.ki, g.startDuty * 100.0f, g.ff, g.ramp, g.limKp, g.limKi, g.deadZone * 100.0f);
        ESP_LOGI("Controller", "║");
    }

    if (scaleLink != nullptr) {
        ESP_LOGI("Controller", "╠═ Scale link (RX gpio%u, TX gpio%u)", scaleLink->rxPin(), scaleLink->txPin());
        ESP_LOGI("Controller", "║  ├─ Link: %s, scale: %s, weight %.2f g", scaleLink->linkUp() ? "up" : "down",
                 scaleLink->scaleAvailable() ? "ok" : "not available", scaleLink->lastGrams());
        ESP_LOGI("Controller", "║  └─ Lines ok/bad: %lu/%lu, readings: %lu, tares not acknowledged: %lu",
                 static_cast<unsigned long>(scaleLink->goodLines()), static_cast<unsigned long>(scaleLink->badLines()),
                 static_cast<unsigned long>(scaleLink->readings()), static_cast<unsigned long>(scaleLink->taresLost()));
        ESP_LOGI("Controller", "║");
    }

    if (vescCan != nullptr) {
        static const char *const BUS[] = {"running", "bus-off / recovering", "not started"};
        static const char *const STATE[] = {"OFF", "RUNNING", "STALL", "HOT", "FAULT", "NO DATA", "NOT USED"};
        const VescCan::Snapshot v = vescCan->snapshot();
        ESP_LOGI("Controller", "╠═ CAN pump drive (TX gpio%u, RX gpio%u, 500 kbit/s, own ID %u)", vescCan->txPin(),
                 vescCan->rxPin(), VescCan::OWN_ID);
        ESP_LOGI("Controller", "║  ├─ Bus: %s, frames in/out: %lu/%lu, not acknowledged: %lu, bus errors: %lu, bus-off: %lu",
                 BUS[v.busState < 3 ? v.busState : 2], static_cast<unsigned long>(v.rxFrames),
                 static_cast<unsigned long>(v.txFrames), static_cast<unsigned long>(v.txFailed),
                 static_cast<unsigned long>(v.busErrors), static_cast<unsigned long>(v.busOffs));
        if (v.speedMode)
            ESP_LOGI("Controller",
                     "║  ├─ Speed control: ERPM asked/sent %ld/%ld (100 %% = %ld ERPM, max pump power %.2f; under %ld ERPM "
                     "the pump is released)",
                     static_cast<long>(v.requestedErpm), static_cast<long>(v.commandErpm), static_cast<long>(v.maxErpm),
                     v.maxPower, static_cast<long>(vesc::SPEED_ON_ERPM));
        else
            ESP_LOGI("Controller", "║  ├─ Duty control: duty asked/sent %.3f/%.3f (max pump power %.2f)", v.requestedPower,
                     v.commandDuty, v.maxPower);
        ESP_LOGI("Controller", "║  ├─ Second pump: %s, motor limit: %.0f C", v.dualPump ? "on" : "off", v.motorTempLimit);
        for (size_t i = 0; i < vesc::MAX_PUMPS; i++) {
            const vesc::PumpReadings &p = v.pumps[i];
            const auto state = static_cast<size_t>(v.states[i]);
            ESP_LOGI(
                "Controller",
                "║  ├─ Pump %u (VESC ID %u): %s, %ld ERPM, motor %.1f A, duty %.3f, motor %.1f C, FET %.1f C, in %.1f V %.1f A, "
                "fault %u %s, frames %lu",
                static_cast<unsigned>(i + 1), VescCan::PUMP_IDS[i], STATE[state < 7 ? state : 5], static_cast<long>(p.erpm),
                p.motorCurrent, p.duty, p.motorTemp, p.fetTemp, p.inputVoltage, p.inputCurrent, p.fault,
                v.freshFault[i] ? vesc::faultName(p.fault) : "(no answer)", static_cast<unsigned long>(p.frames));
        }
        if (v.lastTrip == vesc::Trip::NONE)
            ESP_LOGI("Controller", "║  └─ Pump stops: none");
        else
            ESP_LOGI("Controller", "║  └─ Pump stops: %lu, last: pump %u %s (detail %ld) %lu s ago%s",
                     static_cast<unsigned long>(v.tripCount), v.lastTripPump, vesc::tripName(v.lastTrip),
                     static_cast<long>(v.lastTripDetail), static_cast<unsigned long>(v.lastTripAgoMs / 1000),
                     v.trip != vesc::Trip::NONE ? ", still holding the pumps off" : "");
        ESP_LOGI("Controller", "║");
    }

    const BleHeadroom headroom = bleHeadroom();
    ESP_LOGI("Controller", "╠═ Bluetooth stacks, most used so far");
    ESP_LOGI("Controller", "║  ├─ NimBLE host task: %u of %u bytes", headroom.hostStackPeak, headroom.hostStackSize);
    ESP_LOGI("Controller", "║  └─ Controller task: %u of %u bytes", headroom.controllerStackPeak, headroom.controllerStackSize);
    ESP_LOGI("Controller", "║");

    size_t free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
    size_t total = heap_caps_get_total_size(MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
    float fragmentation = 100 - (largest * 100) / free;
    ESP_LOGI("Controller", "╠═ Memory");
    ESP_LOGI("Controller", "║  ├─ Heap: %d / %d (%.2f%%)", (total - free), total, (100.0f * (total - free)) / total);
    ESP_LOGI("Controller", "║  └─ Fragmentation: %.2f%%", fragmentation);
    ESP_LOGI("Controller", "");
}
