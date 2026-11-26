#include "BleHeadroom.h"
#include <NimBLEDevice.h>
#include <esp_task.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
// Both tasks are created once by NimBLEDevice::init() and never deleted, so the handles can be kept.
TaskHandle_t hostTask = nullptr;
TaskHandle_t controllerTask = nullptr;

uint16_t stackPeak(TaskHandle_t task, uint32_t size) {
    const UBaseType_t untouched = uxTaskGetStackHighWaterMark(task);
    return untouched < size ? static_cast<uint16_t>(size - untouched) : 0;
}
} // namespace

BleHeadroom bleHeadroom() {
    BleHeadroom headroom;
    if (hostTask == nullptr)
        hostTask = xTaskGetHandle("nimble_host");
    if (controllerTask == nullptr)
        controllerTask = xTaskGetHandle("btController");
    if (hostTask != nullptr) {
        headroom.hostStackSize = CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE;
        headroom.hostStackPeak = stackPeak(hostTask, CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE);
    }
    if (controllerTask != nullptr) {
        headroom.controllerStackSize = ESP_TASK_BT_CONTROLLER_STACK;
        headroom.controllerStackPeak = stackPeak(controllerTask, ESP_TASK_BT_CONTROLLER_STACK);
    }
    return headroom;
}
