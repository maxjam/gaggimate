#ifndef GAGGIMATECONTROLLER_H
#define GAGGIMATECONTROLLER_H
#include "ControllerConfig.h"
#include "GaggiMateServer.h"
#include <peripherals/DigitalInput.h>
#include <peripherals/DistanceSensor.h>
#include <peripherals/FlowSensor.h>
#include <peripherals/HardwareScale.h>
#include <peripherals/Heater.h>
#include <peripherals/HeaterCoordinator.h>
#include <peripherals/LedController.h>
#include <peripherals/Max31855Thermocouple.h>
#include <peripherals/MeterLink.h>
#include <peripherals/PressureSensor.h>
#include <peripherals/Pump.h>
#include <peripherals/SimpleRelay.h>
#include <peripherals/addons/GearpumpAddon.h>
#include <vector>

constexpr double PING_TIMEOUT_SECONDS = 20.0;
constexpr unsigned long LED_HEALTH_CHECK_INTERVAL_MS = 5000;
constexpr unsigned long TANK_LEVEL_SEND_INTERVAL_MS = 5000;

constexpr int DETECT_EN_PIN = 40;
constexpr int DETECT_VALUE_PIN = 11;

// Fork: which Pro header carries the serial link to the flow-meter board.
//   METER_LINK_PORT=4  J4 (gpio44 RX / gpio43 TX). The Alba's SoftWire bus on those pins is dropped;
//                      with METER_LINK_ALBA=1 the Alba is assumed to hang on the meter board and its
//                      tank distance / LED commands travel over the link.
//   METER_LINK_PORT=5  J5 (gpio18 RX / gpio17 TX). Alba stays stock on J4; the HX711 hardware scale
//                      on J5 is not probed (pins are the UART now).
//   undefined          stock behaviour, no meter link.
#ifdef METER_LINK_PORT
#if METER_LINK_PORT != 4 && METER_LINK_PORT != 5
#error "METER_LINK_PORT must be 4 (J4) or 5 (J5)"
#endif
#ifndef METER_LINK_ALBA
#define METER_LINK_ALBA (METER_LINK_PORT == 4 ? 1 : 0)
#endif
#else
#define METER_LINK_ALBA 0
#endif

class GaggiMateController {
  public:
    explicit GaggiMateController(String version);
    void setup(void);
    void loop(void);

    void registerBoardConfig(ControllerConfig config);

  private:
    void detectBoard();
    void detectAddon();
    void handlePing();
    void handlePingTimeout(void);
    void thermalRunawayShutdown(void);
    void startPidAutotune(void);
    void stopPidAutotune(void);
    void sendSensorData(void);
    void sendTankLevel();
    void handleSerialCommand(char c);
    void handleConsoleLine(const char *line);
    void printStatus();
    void setupMeterLink();
    bool isSteamSwitchOn() const;

    ControllerConfig _config = ControllerConfig{};
    GaggiMateServer _comms;

    TemperatureSensor *brewTemperature = nullptr;
    TemperatureSensor *steamTemperature = nullptr;
    Heater *heater = nullptr;
    Heater *heater2 = nullptr;
    HeaterCoordinator *heaterCoordinator = nullptr;
    Pump *pump = nullptr;
    SimpleRelay *valve = nullptr;
    SimpleRelay *alt = nullptr;
    SimpleRelay *refill = nullptr;
    SimpleRelay *aux = nullptr;
    SimpleRelay *lights = nullptr;
    DigitalInput *brewBtn = nullptr;
    DigitalInput *steamBtn = nullptr;
    DigitalInput *waterBtn = nullptr;
    DigitalInput *waterSense = nullptr;
    DigitalInput *tankLevel = nullptr;
    PressureSensor *pressureSensor = nullptr;
    LedController *ledController = nullptr;
    DistanceSensor *distanceSensor = nullptr;
    ADSAdc *adc = nullptr;
    FlowSensor *flowSensor = nullptr;
    HardwareScale *hardwareScale = nullptr;
    MeterLink *meterLink = nullptr;

    GearpumpAddon *gearpumpAddon = nullptr;

    SoftWire *albaComms = nullptr;
    SoftWireBus *albaBus = nullptr;

    std::vector<ControllerConfig> configs;

    String _version;
    unsigned long lastPingTime = 0;
    unsigned long lastLedHealthCheck = 0;
    unsigned long lastTankLevelSend = 0;
    unsigned long lastFlowFaultLog = 0;
    size_t errorState = ERROR_CODE_NONE;
    char consoleLine[96] = {0};
    size_t consoleLen = 0;

    const char *LOG_TAG = "GaggiMateController";
};

#endif // GAGGIMATECONTROLLER_H
