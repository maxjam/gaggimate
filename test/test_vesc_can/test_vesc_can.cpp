// Unit tests: VESC CAN frames and the pump stop rules of the fork's CAN pump drive (maxa4turbo.15).
// Host-side, STL only — pio test -e native -f test_vesc_can.
// The expected bytes are written out by hand from the VESC firmware 7.00 source (comm/comm_can.c, comm/commands.c):
// identifier = packet number << 8 | VESC ID, values big-endian.

#include <unity.h>

#include <cmath>
#include <cstring>
#include <initializer_list>

#include "peripherals/VescCanLogic.h"

using namespace vesc;

static const uint8_t IDS[MAX_PUMPS] = {18, 19};
static const uint8_t OWN = 42;

static Frame make(uint32_t id, std::initializer_list<uint8_t> bytes) {
    Frame f;
    f.id = id;
    f.len = static_cast<uint8_t>(bytes.size());
    size_t i = 0;
    for (uint8_t b : bytes)
        f.data[i++] = b;
    return f;
}

static void assertBytes(const Frame &f, uint32_t id, std::initializer_list<uint8_t> bytes) {
    TEST_ASSERT_EQUAL_HEX32(id, f.id);
    TEST_ASSERT_EQUAL_UINT8(bytes.size(), f.len);
    size_t i = 0;
    for (uint8_t b : bytes)
        TEST_ASSERT_EQUAL_HEX8(b, f.data[i++]);
}

// ---------------------------------------------------------------- frames out

void test_set_duty_frames() {
    assertBytes(setDuty(18, 0.50f), 0x012, {0x00, 0x00, 0xC3, 0x50}); // 50000
    assertBytes(setDuty(19, 0.50f), 0x013, {0x00, 0x00, 0xC3, 0x50});
    assertBytes(setDuty(18, 0.90f), 0x012, {0x00, 0x01, 0x5F, 0x90});  // 90000
    assertBytes(setDuty(18, 1.00f), 0x012, {0x00, 0x01, 0x86, 0xA0});  // 100000
    assertBytes(setDuty(18, 0.153f), 0x012, {0x00, 0x00, 0x3B, 0xC4}); // 15300
}

void test_set_duty_is_clamped() {
    assertBytes(setDuty(18, 1.7f), 0x012, {0x00, 0x01, 0x86, 0xA0});
    assertBytes(setDuty(18, -0.3f), 0x012, {0x00, 0x00, 0x00, 0x00}); // never backwards
    assertBytes(setDuty(18, NAN), 0x012, {0x00, 0x00, 0x00, 0x00});
    assertBytes(setDuty(18, INFINITY), 0x012, {0x00, 0x00, 0x00, 0x00});
}

void test_set_current_zero_is_four_bytes() {
    // 6 or more bytes would make the VESC read the first two as an off-delay.
    assertBytes(setCurrent(18, 0.0f), 0x112, {0x00, 0x00, 0x00, 0x00});
    assertBytes(setCurrent(19, 0.0f), 0x113, {0x00, 0x00, 0x00, 0x00});
    assertBytes(setCurrent(18, 1.5f), 0x112, {0x00, 0x00, 0x05, 0xDC}); // 1500 mA
    assertBytes(setCurrent(18, NAN), 0x112, {0x00, 0x00, 0x00, 0x00});
}

void test_set_rpm_frames() {
    // CAN_PACKET_SET_RPM = 3: identifier 0x3xx, int32 ERPM, not scaled (comm_can.c: buffer_get_float32(data8, 1e0)).
    assertBytes(setRpm(18, 67000), 0x312, {0x00, 0x01, 0x05, 0xB8});
    assertBytes(setRpm(19, 67000), 0x313, {0x00, 0x01, 0x05, 0xB8});
    assertBytes(setRpm(18, 33500), 0x312, {0x00, 0x00, 0x82, 0xDC});
    assertBytes(setRpm(18, 1000), 0x312, {0x00, 0x00, 0x03, 0xE8});
    assertBytes(setRpm(18, -500), 0x312, {0x00, 0x00, 0x00, 0x00}); // never backwards
}

void test_pump_power_to_erpm() {
    TEST_ASSERT_EQUAL_INT32(67000, powerToErpm(1.0f, 67000));
    TEST_ASSERT_EQUAL_INT32(33500, powerToErpm(0.5f, 67000));
    TEST_ASSERT_EQUAL_INT32(6700, powerToErpm(0.1f, 67000));
    TEST_ASSERT_EQUAL_INT32(0, powerToErpm(0.0f, 67000));
    TEST_ASSERT_EQUAL_INT32(0, powerToErpm(-0.2f, 67000));
    TEST_ASSERT_EQUAL_INT32(0, powerToErpm(NAN, 67000));
    TEST_ASSERT_EQUAL_INT32(67000, powerToErpm(1.4f, 67000));
    // The setting is held inside 5,000..75,000: above that the motor's back-EMF passes the 12 V supply.
    TEST_ASSERT_EQUAL_INT32(75000, powerToErpm(1.0f, 120000));
    TEST_ASSERT_EQUAL_INT32(5000, powerToErpm(1.0f, 0));
    TEST_ASSERT_EQUAL_INT32(MAX_ERPM_DEFAULT, clampMaxErpm(67000));
}

void test_speed_drive_switches_at_1000_erpm_with_a_gap() {
    // Under the VESC's own minimum (900) a speed command shorts the phases, so nothing under 950 is ever sent.
    TEST_ASSERT_TRUE(SPEED_OFF_ERPM > 900);
    TEST_ASSERT_FALSE(speedDriveOn(0, false));
    TEST_ASSERT_FALSE(speedDriveOn(999, false));
    TEST_ASSERT_TRUE(speedDriveOn(1000, false));
    TEST_ASSERT_TRUE(speedDriveOn(960, true)); // already running: stays on
    TEST_ASSERT_TRUE(speedDriveOn(950, true));
    TEST_ASSERT_FALSE(speedDriveOn(949, true));
    TEST_ASSERT_FALSE(speedDriveOn(960, false)); // released: needs 1,000 again
}

void test_fault_request_frame() {
    // From ID 42, send flag 0, COMM_GET_VALUES_SELECTIVE (50), mask bit 15.
    assertBytes(faultRequest(18, OWN), 0x812, {0x2A, 0x00, 0x32, 0x00, 0x00, 0x80, 0x00});
    assertBytes(faultRequest(19, OWN), 0x813, {0x2A, 0x00, 0x32, 0x00, 0x00, 0x80, 0x00});
}

// ---------------------------------------------------------------- frames in

void test_status1_parsed() {
    PumpReadings pumps[MAX_PUMPS];
    // 30,866 ERPM = 0x00007892, 2.1 A = 21, duty 0.500 = 500 = 0x01F4
    Frame f = make(0x912, {0x00, 0x00, 0x78, 0x92, 0x00, 0x15, 0x01, 0xF4});
    TEST_ASSERT_EQUAL_INT(0, applyFrame(f, 1000, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_TRUE(pumps[0].hasStatus1);
    TEST_ASSERT_EQUAL_UINT32(1000, pumps[0].status1At);
    TEST_ASSERT_EQUAL_INT32(30866, pumps[0].erpm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.1f, pumps[0].motorCurrent);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, pumps[0].duty);
    TEST_ASSERT_FALSE(pumps[1].hasStatus1);

    // Pump 2, turning backwards: -1,000 ERPM = 0xFFFFFC18, -0.3 A = 0xFFFD
    f = make(0x913, {0xFF, 0xFF, 0xFC, 0x18, 0xFF, 0xFD, 0x00, 0x00});
    TEST_ASSERT_EQUAL_INT(1, applyFrame(f, 1010, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT32(-1000, pumps[1].erpm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.3f, pumps[1].motorCurrent);
}

void test_status4_and_5_parsed() {
    PumpReadings pumps[MAX_PUMPS];
    // FET 31.5 C = 315 = 0x013B, motor 80.2 C = 802 = 0x0322, input 1.1 A = 11, pid pos ignored
    Frame f = make(0x1012, {0x01, 0x3B, 0x03, 0x22, 0x00, 0x0B, 0x12, 0x34});
    TEST_ASSERT_EQUAL_INT(0, applyFrame(f, 2000, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_TRUE(pumps[0].hasStatus4);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 31.5f, pumps[0].fetTemp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 80.2f, pumps[0].motorTemp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.1f, pumps[0].inputCurrent);

    // Tachometer 123456 = 0x0001E240, input 12.0 V = 120 = 0x0078
    f = make(0x1B13, {0x00, 0x01, 0xE2, 0x40, 0x00, 0x78, 0x00, 0x00});
    TEST_ASSERT_EQUAL_INT(1, applyFrame(f, 2000, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_TRUE(pumps[1].hasStatus5);
    TEST_ASSERT_EQUAL_INT32(123456, pumps[1].tachometer);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, pumps[1].inputVoltage);

    // A motor colder than 0 C: -5.0 C = -50 = 0xFFCE
    f = make(0x1012, {0x00, 0x64, 0xFF, 0xCE, 0x00, 0x00, 0x00, 0x00});
    applyFrame(f, 2100, OWN, IDS, pumps, MAX_PUMPS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.0f, pumps[0].motorTemp);
}

void test_fault_answer_parsed() {
    PumpReadings pumps[MAX_PUMPS];
    // Side 2 (ID 19) answers ID 42: [19][1][50][00 00 80 00][4 = ABS_OVER_CURRENT]
    Frame f = make(0x82A, {0x13, 0x01, 0x32, 0x00, 0x00, 0x80, 0x00, 0x04});
    TEST_ASSERT_EQUAL_INT(1, applyFrame(f, 3000, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_TRUE(pumps[1].hasFault);
    TEST_ASSERT_EQUAL_UINT8(4, pumps[1].fault);
    TEST_ASSERT_EQUAL_STRING("ABS_OVER_CURRENT", faultName(pumps[1].fault));
    TEST_ASSERT_FALSE(pumps[0].hasFault);

    f = make(0x82A, {0x12, 0x01, 0x32, 0x00, 0x00, 0x80, 0x00, 0x00});
    TEST_ASSERT_EQUAL_INT(0, applyFrame(f, 3000, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_TRUE(pumps[0].hasFault);
    TEST_ASSERT_EQUAL_UINT8(0, pumps[0].fault);
}

void test_foreign_frames_ignored() {
    PumpReadings pumps[MAX_PUMPS];
    // STATUS 1 from a VESC with another ID (side 2 before its ID is set to 19 sends as ID 0)
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x900, {0, 0, 0x78, 0x92, 0, 0x15, 0x01, 0xF4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    // Our own request seen back, a short answer addressed to someone else, a wrong mask, a short frame
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x812, {0x2A, 0x00, 0x32, 0x00, 0x00, 0x80, 0x00}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x812, {0x13, 0x01, 0x32, 0, 0, 0x80, 0, 4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x82A, {0x13, 0x01, 0x32, 0, 0, 0x40, 0, 4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x82A, {0x13, 0x00, 0x32, 0, 0, 0x80, 0, 4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x82A, {0x07, 0x01, 0x32, 0, 0, 0x80, 0, 4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x912, {0x00, 0x00, 0x78, 0x92}), 1, OWN, IDS, pumps, MAX_PUMPS));
    // Other packets (STATUS 2, PONG) and an ID with bits above the packet byte
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0xE12, {0, 0, 0, 0, 0, 0, 0, 0}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x1212, {0x12, 0x00}), 1, OWN, IDS, pumps, MAX_PUMPS));
    TEST_ASSERT_EQUAL_INT(-1, applyFrame(make(0x10912, {0, 0, 0x78, 0x92, 0, 0x15, 0x01, 0xF4}), 1, OWN, IDS, pumps, MAX_PUMPS));
    for (const PumpReadings &p : pumps) {
        TEST_ASSERT_FALSE(p.hasStatus1 || p.hasStatus4 || p.hasStatus5 || p.hasFault);
        TEST_ASSERT_EQUAL_UINT32(0, p.frames);
    }
}

// ---------------------------------------------------------------- stop rules

// A small bench: time moves in 10 ms cycles, the VESCs report as the real ones do unless a test silences them.
struct Bench {
    Guard guard;
    PumpReadings pumps[MAX_PUMPS];
    bool enabled[MAX_PUMPS] = {true, false};
    uint32_t now = 100000;
    int32_t erpm[MAX_PUMPS] = {0, 0};
    float motorTemp[MAX_PUMPS] = {30.0f, 30.0f};
    uint8_t fault[MAX_PUMPS] = {0, 0};
    bool status1[MAX_PUMPS] = {true, true};
    bool status4[MAX_PUMPS] = {true, true};
    bool faultPoll[MAX_PUMPS] = {true, true};
    int latches = 0;
    uint32_t cycle = 0; // counts 10 ms cycles; the frame cadence hangs on this, not on `now`, which may wrap
    bool latched = false;
    uint32_t latchedAfterMs = 0;

    // Speed control, as VescCan::loop() feeds the guard
    bool speedOn = false;
    int32_t askedErpm = 0;

    void frames() {
        now += 10;
        cycle++;
        for (size_t i = 0; i < MAX_PUMPS; i++) {
            if (status1[i] && cycle % 2 == 0) { // 50 a second
                pumps[i].erpm = erpm[i];
                pumps[i].hasStatus1 = true;
                pumps[i].status1At = now;
            }
            if (status4[i] && cycle % 10 == 0) { // 10 a second
                pumps[i].motorTemp = motorTemp[i];
                pumps[i].hasStatus4 = true;
                pumps[i].status4At = now;
            }
            if (faultPoll[i] && cycle % 20 == 0) { // 5 a second
                pumps[i].fault = fault[i];
                pumps[i].hasFault = true;
                pumps[i].faultAt = now;
            }
        }
    }

    void count(bool latchedNow, uint32_t t) {
        if (!latchedNow)
            return;
        latches++;
        if (!latched) {
            latched = true;
            latchedAfterMs = t;
        }
    }

    // Runs `ms` of cycles at one duty (duty control). Returns true when a stop latched in this run; latchedAfterMs
    // then holds how far into the run that was.
    bool run(uint32_t ms, float duty) {
        latched = false;
        latchedAfterMs = 0;
        for (uint32_t t = 10; t <= ms; t += 10) {
            frames();
            count(guard.update(now, duty, pumps, enabled, MAX_PUMPS), t);
        }
        return latched;
    }

    // The same in speed control: pump power 0..1 becomes an ERPM, driven from 1,000 ERPM up.
    bool runSpeed(uint32_t ms, float power, int32_t maxErpm = MAX_ERPM_DEFAULT) {
        latched = false;
        latchedAfterMs = 0;
        for (uint32_t t = 10; t <= ms; t += 10) {
            frames();
            askedErpm = powerToErpm(power, maxErpm);
            speedOn = speedDriveOn(askedErpm, speedOn);
            count(guard.update(now, guard.speedDemand(power > 0.0f, speedOn, askedErpm), pumps, enabled, MAX_PUMPS), t);
        }
        return latched;
    }
};

void test_healthy_shot_never_stops() {
    Bench b;
    b.run(3000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(40000, 0.5f));
    b.erpm[0] = 0;
    TEST_ASSERT_FALSE(b.run(5000, 0.0f)); // pump off and standing still is not a stall
    TEST_ASSERT_FALSE(b.guard.tripped());
    TEST_ASSERT_EQUAL_INT(0, b.latches);
}

void test_stall_after_grace_and_half_second() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 0; // never starts turning
    TEST_ASSERT_TRUE(b.run(5000, 0.5f));
    // 2 s grace after the duty rises, then 500 ms without a break
    TEST_ASSERT_UINT32_WITHIN(30, 2500, b.latchedAfterMs);
    TEST_ASSERT_TRUE(b.guard.tripped());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::STALL), static_cast<uint8_t>(b.guard.trip()));
    TEST_ASSERT_EQUAL_UINT8(1, b.guard.tripPump());
    TEST_ASSERT_EQUAL_INT(1, b.latches); // latches once, not every cycle
}

void test_stall_in_the_middle_of_a_shot() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(10000, 0.5f));
    b.erpm[0] = 150;
    TEST_ASSERT_TRUE(b.run(2000, 0.5f));
    TEST_ASSERT_UINT32_WITHIN(40, 500, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_INT32(150, b.guard.lastTripDetail());
}

void test_short_dip_is_not_a_stall() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.run(5000, 0.5f);
    for (int n = 0; n < 5; n++) {
        b.erpm[0] = 0;
        TEST_ASSERT_FALSE(b.run(400, 0.5f)); // under 500 ms
        b.erpm[0] = 30000;
        TEST_ASSERT_FALSE(b.run(100, 0.5f)); // one break restarts the count
    }
    TEST_ASSERT_FALSE(b.guard.tripped());
}

void test_no_stall_below_15_percent() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 0;
    TEST_ASSERT_FALSE(b.run(20000, 0.14f)); // low duty against pressure may hold still
    TEST_ASSERT_TRUE(b.run(5000, 0.15f));   // from 15 % it has to turn
}

void test_grace_rearmed_when_duty_rises_from_zero() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.run(5000, 0.5f);
    b.erpm[0] = 0;
    b.run(300, 0.0f);                    // a zero-power step in the profile
    TEST_ASSERT_TRUE(b.run(5000, 0.5f)); // pump jammed on the restart
    TEST_ASSERT_UINT32_WITHIN(30, 2500, b.latchedAfterMs);
}

void test_negative_erpm_counts_as_turning() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = -30000;
    TEST_ASSERT_FALSE(b.run(10000, 0.5f));
    b.erpm[0] = -150;
    TEST_ASSERT_TRUE(b.run(2000, 0.5f));
}

void test_hot_motor_stops_at_the_limit() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.motorTemp[0] = 79.9f;
    TEST_ASSERT_FALSE(b.run(5000, 0.8f));
    b.motorTemp[0] = 80.0f;
    TEST_ASSERT_TRUE(b.run(1000, 0.8f));
    TEST_ASSERT_TRUE(b.latchedAfterMs <= 110); // the next STATUS 4
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::HOT), static_cast<uint8_t>(b.guard.trip()));
    TEST_ASSERT_EQUAL_INT32(80, b.guard.lastTripDetail());
}

void test_hot_motor_blocks_the_next_start_until_cool() {
    Bench b;
    b.erpm[0] = 30000;
    b.motorTemp[0] = 85.0f;
    b.run(1000, 0.0f);
    TEST_ASSERT_FALSE(b.guard.tripped()); // nothing to stop while idle
    TEST_ASSERT_TRUE(b.run(1000, 0.5f));
    TEST_ASSERT_EQUAL_UINT32(10, b.latchedAfterMs); // first cycle
    b.run(1500, 0.0f);
    TEST_ASSERT_FALSE(b.guard.tripped());
    TEST_ASSERT_TRUE(b.run(1000, 0.5f)); // still hot: stops again at once
    TEST_ASSERT_EQUAL_UINT32(10, b.latchedAfterMs);
    b.motorTemp[0] = 60.0f;
    b.run(1500, 0.0f);
    TEST_ASSERT_FALSE(b.run(5000, 0.5f));
}

void test_temperature_check_can_be_switched_off() {
    Bench b;
    b.guard.config.motorTempLimit = 0.0f;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.motorTemp[0] = 150.0f;
    TEST_ASSERT_FALSE(b.run(5000, 0.5f));
}

void test_disabled_sensor_reads_zero_and_never_trips() {
    Bench b; // a VESC with the motor sensor set to Disabled reports 0 C
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.motorTemp[0] = 0.0f;
    TEST_ASSERT_FALSE(b.run(5000, 0.5f));
}

void test_fault_code_stops() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(5000, 0.6f));
    b.fault[0] = 4;
    TEST_ASSERT_TRUE(b.run(1000, 0.6f));
    TEST_ASSERT_TRUE(b.latchedAfterMs <= 210); // the next fault poll
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::FAULT), static_cast<uint8_t>(b.guard.trip()));
    TEST_ASSERT_EQUAL_INT32(4, b.guard.lastTripDetail());
}

void test_silent_vesc_stops_as_no_data_not_stall() {
    Bench b;
    b.status1[0] = b.status4[0] = b.faultPoll[0] = false; // Flipsky off, cable out, status messages not set up
    b.run(1000, 0.0f);
    TEST_ASSERT_TRUE(b.run(5000, 0.5f));
    TEST_ASSERT_UINT32_WITHIN(20, 500, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::NO_DATA), static_cast<uint8_t>(b.guard.trip()));
}

void test_vesc_going_silent_mid_shot() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(8000, 0.5f));
    b.status1[0] = false;
    TEST_ASSERT_TRUE(b.run(3000, 0.5f));
    TEST_ASSERT_UINT32_WITHIN(40, 510, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::NO_DATA), static_cast<uint8_t>(b.guard.trip()));
}

void test_missing_status4_or_fault_poll_does_not_stop() {
    Bench b; // without them the temperature and fault checks have nothing to judge; STATUS 1 still guards the stall
    b.status4[0] = b.faultPoll[0] = false;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(20000, 0.5f));
}

void test_stale_readings_are_not_used() {
    Bench b;
    b.erpm[0] = 30000;
    b.motorTemp[0] = 90.0f;
    b.fault[0] = 4;
    b.run(1000, 0.0f);                     // hot and faulted while idle
    b.status4[0] = b.faultPoll[0] = false; // then those messages stop
    b.run(3000, 0.0f);                     // the last ones are now 3 s old
    TEST_ASSERT_FALSE(b.run(5000, 0.5f));
}

void test_second_pump_ignored_until_enabled() {
    Bench b;
    b.status1[1] = b.status4[1] = b.faultPoll[1] = false; // no pump 2 on the bus
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(10000, 0.5f));
    b.run(2000, 0.0f);
    b.enabled[1] = true; // "second pump" switched on with nothing wired
    TEST_ASSERT_TRUE(b.run(3000, 0.5f));
    TEST_ASSERT_EQUAL_UINT8(2, b.guard.tripPump());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::NO_DATA), static_cast<uint8_t>(b.guard.trip()));
}

void test_either_pump_stalling_stops() {
    Bench b;
    b.enabled[1] = true;
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    b.erpm[1] = 30000;
    TEST_ASSERT_FALSE(b.run(10000, 0.5f));
    b.erpm[1] = 0;
    TEST_ASSERT_TRUE(b.run(2000, 0.5f));
    TEST_ASSERT_EQUAL_UINT8(2, b.guard.tripPump());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::STALL), static_cast<uint8_t>(b.guard.trip()));
}

void test_stop_holds_until_power_has_been_zero_for_a_second() {
    Bench b;
    b.run(1000, 0.0f);
    b.erpm[0] = 0;
    TEST_ASSERT_TRUE(b.run(5000, 0.5f));
    b.erpm[0] = 30000;  // whatever the readings say now,
    b.run(20000, 0.5f); // the stop holds while power is still asked for
    TEST_ASSERT_TRUE(b.guard.tripped());
    b.run(900, 0.0f);
    TEST_ASSERT_TRUE(b.guard.tripped());
    b.run(50, 0.3f); // a blip of power restarts the second
    b.run(900, 0.0f);
    TEST_ASSERT_TRUE(b.guard.tripped());
    b.run(200, 0.0f);
    TEST_ASSERT_FALSE(b.guard.tripped());
    TEST_ASSERT_EQUAL_INT(1, b.latches);
    // The last stop is still on record, and a new run starts clean.
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::STALL), static_cast<uint8_t>(b.guard.lastTrip()));
    TEST_ASSERT_FALSE(b.run(10000, 0.5f));
}

// ---------------------------------------------------------------- speed control (maxa4turbo.16)

void test_speed_healthy_shot_into_field_weakening() {
    Bench b;
    b.runSpeed(3000, 0.0f);
    b.erpm[0] = 33500;
    TEST_ASSERT_FALSE(b.runSpeed(20000, 0.5f));
    b.erpm[0] = 67000; // past the 58,400 ERPM that full duty gives: field weakening
    TEST_ASSERT_FALSE(b.runSpeed(20000, 1.0f));
    TEST_ASSERT_EQUAL_INT32(67000, b.askedErpm);
    b.erpm[0] = 0;
    TEST_ASSERT_FALSE(b.runSpeed(5000, 0.0f));
    TEST_ASSERT_EQUAL_INT(0, b.latches);
}

void test_speed_stall_judged_from_10000_erpm() {
    Bench b;
    b.runSpeed(1000, 0.0f);
    b.erpm[0] = 0;
    // 14.9 % of 67,000 = 9,983 ERPM asked: a pump holding still is not called a stall yet
    TEST_ASSERT_FALSE(b.runSpeed(20000, 0.149f));
    TEST_ASSERT_TRUE(b.askedErpm < 10000);
    // 15 % = 10,050 ERPM: it has to turn. The drive was on all along, so the 2 s grace is long over.
    TEST_ASSERT_TRUE(b.runSpeed(5000, 0.15f));
    TEST_ASSERT_UINT32_WITHIN(40, 500, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::STALL), static_cast<uint8_t>(b.guard.trip()));
}

void test_speed_stall_after_grace_from_a_start() {
    Bench b;
    b.runSpeed(1000, 0.0f);
    b.erpm[0] = 0; // never starts turning
    TEST_ASSERT_TRUE(b.runSpeed(5000, 1.0f));
    TEST_ASSERT_UINT32_WITHIN(30, 2500, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::STALL), static_cast<uint8_t>(b.guard.trip()));
}

void test_speed_under_1000_erpm_is_released_and_not_judged() {
    Bench b;
    b.runSpeed(1000, 0.0f);
    b.status1[0] = false; // a silent VESC
    // 1 % of 67,000 = 670 ERPM: under 1,000 the pump is released, so there is nothing to check
    TEST_ASSERT_FALSE(b.runSpeed(5000, 0.01f));
    TEST_ASSERT_FALSE(b.speedOn);
    // 2 % = 1,340 ERPM: driven, and now the silent VESC is "no data" after its 500 ms
    TEST_ASSERT_TRUE(b.runSpeed(3000, 0.02f));
    TEST_ASSERT_TRUE(b.speedOn);
    TEST_ASSERT_UINT32_WITHIN(30, 500, b.latchedAfterMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Trip::NO_DATA), static_cast<uint8_t>(b.guard.trip()));
}

void test_speed_grace_starts_when_the_drive_switches_on() {
    Bench b;
    b.runSpeed(1000, 0.0f);
    b.erpm[0] = 0;
    b.runSpeed(10000, 0.01f); // pump power above 0 for 10 s, but released (670 ERPM asked)
    TEST_ASSERT_FALSE(b.guard.tripped());
    // Now a real speed is asked: the stall count starts with its 2 s grace here, not 10 s ago
    TEST_ASSERT_FALSE(b.runSpeed(2400, 0.5f));
    TEST_ASSERT_TRUE(b.runSpeed(1000, 0.5f));
}

void test_speed_stop_holds_until_power_has_been_zero_for_a_second() {
    Bench b;
    b.runSpeed(1000, 0.0f);
    b.erpm[0] = 0;
    TEST_ASSERT_TRUE(b.runSpeed(5000, 1.0f));
    b.erpm[0] = 30000; // (the pump would turn again)
    b.runSpeed(5000, 1.0f);
    TEST_ASSERT_TRUE(b.guard.tripped()); // still asking for power: stays stopped
    // Power above 0 but under 1,000 ERPM is still the profile asking: no release
    b.runSpeed(3000, 0.01f);
    TEST_ASSERT_TRUE(b.guard.tripped());
    b.runSpeed(900, 0.0f);
    TEST_ASSERT_TRUE(b.guard.tripped());
    b.runSpeed(200, 0.0f);
    TEST_ASSERT_FALSE(b.guard.tripped());
    TEST_ASSERT_FALSE(b.runSpeed(10000, 0.5f)); // and the next shot runs
}

void test_speed_demand_flags() {
    Guard g;
    Demand d = g.speedDemand(true, true, 12000);
    TEST_ASSERT_TRUE(d.requested && d.driving && d.stallArmed);
    d = g.speedDemand(true, true, 5000);
    TEST_ASSERT_TRUE(d.requested && d.driving);
    TEST_ASSERT_FALSE(d.stallArmed);
    d = g.speedDemand(true, false, 700);
    TEST_ASSERT_TRUE(d.requested);
    TEST_ASSERT_FALSE(d.driving || d.stallArmed);
    d = g.speedDemand(false, true, 20000); // no pump power: nothing is driven whatever the rest says
    TEST_ASSERT_FALSE(d.requested || d.driving || d.stallArmed);
    d = g.dutyDemand(0.5f);
    TEST_ASSERT_TRUE(d.requested && d.driving && d.stallArmed);
    d = g.dutyDemand(0.1f);
    TEST_ASSERT_TRUE(d.requested && d.driving);
    TEST_ASSERT_FALSE(d.stallArmed);
}

void test_millis_wraparound() {
    Bench b;
    b.now = 0xFFFFFFFF - 5000; // about 5 s before the 49.7 day wrap
    b.run(1000, 0.0f);
    b.erpm[0] = 30000;
    TEST_ASSERT_FALSE(b.run(20000, 0.5f)); // straight through the wrap
    TEST_ASSERT_TRUE(b.now < 100000);
    b.erpm[0] = 0;
    TEST_ASSERT_TRUE(b.run(2000, 0.5f));
    TEST_ASSERT_UINT32_WITHIN(40, 500, b.latchedAfterMs);
}

void test_pump_state_for_the_web_ui() {
    Bench b;
    b.erpm[0] = 0;
    b.run(1000, 0.0f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::OFF),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, false, b.now)));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::NOT_USED),
                            static_cast<uint8_t>(pumpState(1, false, b.pumps[1], b.guard, false, b.now)));
    b.erpm[0] = 30000;
    b.run(3000, 0.5f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::RUNNING),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, true, b.now)));
    b.erpm[0] = 0;
    b.run(1000, 0.5f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::STALL),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, false, b.now)));
    b.run(1500, 0.0f); // released
    b.motorTemp[0] = 82.0f;
    b.run(300, 0.0f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::HOT),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, false, b.now)));
    b.motorTemp[0] = 40.0f;
    b.fault[0] = 2;
    b.run(300, 0.0f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::FAULT),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, false, b.now)));
    b.status1[0] = false;
    b.run(700, 0.0f);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PumpState::NO_DATA),
                            static_cast<uint8_t>(pumpState(0, true, b.pumps[0], b.guard, false, b.now)));
}

void test_fault_names_match_the_firmware_list() {
    TEST_ASSERT_EQUAL_STRING("NONE", faultName(0));
    TEST_ASSERT_EQUAL_STRING("OVER_VOLTAGE", faultName(1));
    TEST_ASSERT_EQUAL_STRING("UNDER_VOLTAGE", faultName(2));
    TEST_ASSERT_EQUAL_STRING("DRV", faultName(3));
    TEST_ASSERT_EQUAL_STRING("ABS_OVER_CURRENT", faultName(4));
    TEST_ASSERT_EQUAL_STRING("OVER_TEMP_FET", faultName(5));
    TEST_ASSERT_EQUAL_STRING("OVER_TEMP_MOTOR", faultName(6));
    TEST_ASSERT_EQUAL_STRING("ABS_OVERSPEED", faultName(33));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", faultName(34));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", faultName(255));
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_set_duty_frames);
    RUN_TEST(test_set_duty_is_clamped);
    RUN_TEST(test_set_current_zero_is_four_bytes);
    RUN_TEST(test_set_rpm_frames);
    RUN_TEST(test_pump_power_to_erpm);
    RUN_TEST(test_speed_drive_switches_at_1000_erpm_with_a_gap);
    RUN_TEST(test_fault_request_frame);
    RUN_TEST(test_status1_parsed);
    RUN_TEST(test_status4_and_5_parsed);
    RUN_TEST(test_fault_answer_parsed);
    RUN_TEST(test_foreign_frames_ignored);
    RUN_TEST(test_healthy_shot_never_stops);
    RUN_TEST(test_stall_after_grace_and_half_second);
    RUN_TEST(test_stall_in_the_middle_of_a_shot);
    RUN_TEST(test_short_dip_is_not_a_stall);
    RUN_TEST(test_no_stall_below_15_percent);
    RUN_TEST(test_grace_rearmed_when_duty_rises_from_zero);
    RUN_TEST(test_negative_erpm_counts_as_turning);
    RUN_TEST(test_hot_motor_stops_at_the_limit);
    RUN_TEST(test_hot_motor_blocks_the_next_start_until_cool);
    RUN_TEST(test_temperature_check_can_be_switched_off);
    RUN_TEST(test_disabled_sensor_reads_zero_and_never_trips);
    RUN_TEST(test_fault_code_stops);
    RUN_TEST(test_silent_vesc_stops_as_no_data_not_stall);
    RUN_TEST(test_vesc_going_silent_mid_shot);
    RUN_TEST(test_missing_status4_or_fault_poll_does_not_stop);
    RUN_TEST(test_stale_readings_are_not_used);
    RUN_TEST(test_second_pump_ignored_until_enabled);
    RUN_TEST(test_either_pump_stalling_stops);
    RUN_TEST(test_stop_holds_until_power_has_been_zero_for_a_second);
    RUN_TEST(test_speed_healthy_shot_into_field_weakening);
    RUN_TEST(test_speed_stall_judged_from_10000_erpm);
    RUN_TEST(test_speed_stall_after_grace_from_a_start);
    RUN_TEST(test_speed_under_1000_erpm_is_released_and_not_judged);
    RUN_TEST(test_speed_grace_starts_when_the_drive_switches_on);
    RUN_TEST(test_speed_stop_holds_until_power_has_been_zero_for_a_second);
    RUN_TEST(test_speed_demand_flags);
    RUN_TEST(test_millis_wraparound);
    RUN_TEST(test_pump_state_for_the_web_ui);
    RUN_TEST(test_fault_names_match_the_firmware_list);
    return UNITY_END();
}
