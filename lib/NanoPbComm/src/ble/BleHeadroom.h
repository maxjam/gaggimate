#ifndef NANOPBCOMM_BLE_HEADROOM_H
#define NANOPBCOMM_BLE_HEADROOM_H

#include <cstdint>

// Fork (maxa4turbo.17): how much of their stacks the two Bluetooth tasks have used since boot.
//
//  - "nimble_host", NimBLE's host task. Every NimBLE-Arduino callback runs on it, so a frame from the display is
//    decoded and acknowledged there: Endpoint::handleData() and Endpoint::sendAck(), under NimBLE-Arduino's write
//    callback (a 512-byte buffer) and above the notify path down to the radio (a 256-byte buffer). NimBLE-Arduino
//    gives the task 4096 bytes unless the build sets CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE (platformio.ini).
//  - "btController", the task of the radio itself; 4096 bytes from the SDK.
//
// A peak is the size minus what FreeRTOS reports as never touched. The controller logs both while they grow and
// prints them with the console command S.
struct BleHeadroom {
    uint16_t hostStackSize = 0;       // bytes; 0 = the task does not exist (yet)
    uint16_t hostStackPeak = 0;       // most bytes of it in use at once so far
    uint16_t controllerStackSize = 0; // bytes; 0 = the task does not exist (yet)
    uint16_t controllerStackPeak = 0;
};

BleHeadroom bleHeadroom();

#endif // NANOPBCOMM_BLE_HEADROOM_H
