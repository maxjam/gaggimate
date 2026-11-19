#ifndef GAGGIMATECONTROLLER_H
#define GAGGIMATECONTROLLER_H
#include "ControllerConfig.h"
#include "GaggiMateServer.h"
#include <array>
#include <atomic>
#include <freertos/queue.h>
#include <memory>
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
#include <peripherals/ScaleLink.h>
#include <peripherals/PwmLight.h>
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
constexpr int kMeterLinkPort = METER_LINK_PORT;
#else
#define METER_LINK_ALBA 0
constexpr int kMeterLinkPort = 0;
#endif

// Fork (meterlink.12): SCALE_LINK_PORT=4 puts the Waveshare scale board on J4 (gpio44 RX / gpio43 TX).
// The board reads the drip-tray load cells and its readings reach the display as the built-in hardware
// scale; the Alba's SoftWire bus on J4 is not started. Needs the meter link off J4 (METER_LINK_PORT=5).
#ifdef SCALE_LINK_PORT
#if SCALE_LINK_PORT != 4
#error "SCALE_LINK_PORT must be 4 (J4)"
#endif
#if defined(METER_LINK_PORT) && METER_LINK_PORT == 4
#error "The scale board and the meter board cannot both use J4"
#endif
constexpr int kScaleLinkPort = SCALE_LINK_PORT;
#else
constexpr int kScaleLinkPort = 0;
#endif

// Fork (meterlink.13): the nightly's wired display link (display <-> Pro over UART on J6 ext4/ext5) runs on
// Serial1, the UART the meter link already uses. Builds with a meter link leave it off; the display stays on BLE.
#ifdef METER_LINK_PORT
constexpr bool kDisplayUartLink = false;
#else
constexpr bool kDisplayUartLink = true;
#endif

class GaggiMateController {
  public:
    explicit GaggiMateController(String version);
    void setup(void);
    void loop(void);

    void registerBoardConfig(ControllerConfig config);

  private:
    void clearRetainedOutputDemands();
    void detectBoard();
    void detectAddon();
    void loadMachineHardware();
    void storeMachineHardware(const gm::MachineHardwareSettings &settings);
    void updateDualBoilerOutputs(unsigned long now);
    void handlePing();
    void handlePingTimeout(void);
    void stopOutputs();
    void thermalRunawayShutdown(void);
    void startPidAutotune(void);
    void stopPidAutotune(void);
    void sendSensorData(void);
    void sendTankLevel();
    void handleSerialCommand(char c);
    void handleConsoleLine(const char *line);
    void printStatus();
    void setupMeterLink();
    void setupScaleLink();
    void updateScaleActivity();
    bool isBondingSwitchOn() const;

    ControllerConfig _config = ControllerConfig{};
    GaggiMateServer _comms;

    std::unique_ptr<TemperatureSensor> brewTemperature;
    std::unique_ptr<TemperatureSensor> steamTemperature;
    std::unique_ptr<Heater> heater;
    std::unique_ptr<Heater> heater2;
    std::unique_ptr<HeaterCoordinator> heaterCoordinator;
    std::unique_ptr<Pump> pump;
    std::unique_ptr<SimpleRelay> valve;
    std::unique_ptr<SimpleRelay> alt;
    std::unique_ptr<SimpleRelay> refill;
    std::unique_ptr<SimpleRelay> aux;
    std::unique_ptr<PwmLight> lights;
    std::unique_ptr<PwmLight> baristaLight;
    std::unique_ptr<DigitalInput> brewBtn;
    std::unique_ptr<DigitalInput> steamBtn;
    std::unique_ptr<DigitalInput> waterBtn;
    std::unique_ptr<DigitalInput> waterSense;
    std::unique_ptr<DigitalInput> tankLevel;
    std::atomic<bool> steamLevelLow{true};
    std::atomic<bool> steamLevelKnown{false};
    std::atomic<bool> tankLevelOK{false};
    std::atomic<float> steamRequestedSetpoint{0.0f};
    unsigned long lastSensorSend = 0;
    std::unique_ptr<PressureSensor> pressureSensor;
    std::unique_ptr<LedController> ledController;
    std::unique_ptr<DistanceSensor> distanceSensor;
    QueueHandle_t hardwareRequests = nullptr;
    std::atomic<bool> controllerErrorPending{false};
    std::atomic<bool> hardwareSessionReady{false};
    bool hardwareConfigurationError = false;
    bool hardwareConfigurationStored = false;
    bool hardwareConfigurationAccepted = false;
    gm::MachineHardwareSettings machineHardware = gaggimate_MachineHardwareSettings_init_zero;
    std::unique_ptr<ADSAdc> adc;
    std::unique_ptr<FlowSensor> flowSensor;
    std::unique_ptr<HardwareScale> hardwareScale;
    std::unique_ptr<MeterLink> meterLink;
    std::unique_ptr<ScaleLink> scaleLink;

    std::unique_ptr<GearpumpAddon> gearpumpAddon;

    std::array<char, 128> albaSwTxBuffer{};
    std::array<char, 128> albaSwRxBuffer{};
    std::unique_ptr<SoftWire> albaComms;
    std::unique_ptr<SoftWireBus> albaBus;

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
