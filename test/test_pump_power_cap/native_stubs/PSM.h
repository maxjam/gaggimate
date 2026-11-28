// Native-host stand-in for the PSM library (phase-skip dimmer), so DimmedPump.cpp builds for test_pump_power_cap.
// The pump under test is driven over CAN from DimmedPump's power value; the dimmer output only records it.
#ifndef NATIVE_STUB_PSM_H
#define NATIVE_STUB_PSM_H

#include <cstdint>

// Last level written to the mains output (0..100), for the tests: with the CAN drive it is on (100) or off (0).
inline int &psmLastLevel() {
    static int level = 0;
    return level;
}

class PSM {
  public:
    PSM(uint8_t, uint8_t, int, int, int, int) {}
    void set(int value) {
        last = value;
        psmLastLevel() = value;
    }
    int cps() { return 60; }
    void setDivider(int) {}
    int last = 0;
};

#endif // NATIVE_STUB_PSM_H
