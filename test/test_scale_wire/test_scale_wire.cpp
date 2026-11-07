// Unit tests: the scale-board wire format ($W, $C) of the fork's J4 scale link.
// Host-side, STL only — pio test -e native -f test_scale_wire.

#include <unity.h>

#include <cmath>
#include <cstring>
#include <string>

#include "peripherals/LineProtocol.h"
#include "peripherals/ScaleWire.h"

using namespace lineproto;

// Frames a payload and runs it through a LineReader, as the link task does.
static bool roundTrip(const char *payload, Message &msg, LineReader &r) {
    char out[MAX_LINE + 8];
    if (frame(payload, out, sizeof(out)) == 0)
        return false;
    bool got = false;
    for (const char *p = out; *p; p++)
        if (r.feed(*p, msg))
            got = true;
    return got;
}

void test_weight_round_trip() {
    scalewire::Reading in;
    in.grams = 36.42f;
    in.cell1 = 17.10f;
    in.cell2 = 19.32f;
    in.cell1Valid = in.cell2Valid = true;
    in.seq = 1234;
    in.tareAck = 512345;
    char payload[MAX_LINE];
    TEST_ASSERT_TRUE(scalewire::formatWeight(in, payload, sizeof(payload)) > 0);
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip(payload, m, r));
    scalewire::Reading out;
    TEST_ASSERT_TRUE(scalewire::parseWeight(m, out));
    TEST_ASSERT_TRUE(out.available());
    TEST_ASSERT_FLOAT_WITHIN(0.006f, 36.42f, out.grams);
    TEST_ASSERT_FLOAT_WITHIN(0.006f, 17.10f, out.cell1);
    TEST_ASSERT_FLOAT_WITHIN(0.006f, 19.32f, out.cell2);
    TEST_ASSERT_EQUAL_UINT32(1234, out.seq);
    TEST_ASSERT_EQUAL_UINT32(512345, out.tareAck);
}

void test_negative_weight_kept() {
    scalewire::Reading in;
    in.grams = -0.3f;
    in.cell1 = -0.1f;
    in.cell2 = -0.2f;
    in.cell1Valid = in.cell2Valid = true;
    char payload[MAX_LINE];
    scalewire::formatWeight(in, payload, sizeof(payload));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip(payload, m, r));
    scalewire::Reading out;
    TEST_ASSERT_TRUE(scalewire::parseWeight(m, out));
    TEST_ASSERT_FLOAT_WITHIN(0.006f, -0.3f, out.grams);
}

void test_unavailable_is_stock_sentinel() {
    scalewire::Reading in; // defaults: no scale
    in.seq = 7;
    char payload[MAX_LINE];
    scalewire::formatWeight(in, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_STRING("W,-9999,0,0,0,7,0", payload);
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip(payload, m, r));
    scalewire::Reading out;
    TEST_ASSERT_TRUE(scalewire::parseWeight(m, out));
    TEST_ASSERT_FALSE(out.available());
    TEST_ASSERT_FALSE(out.cell1Valid);
    TEST_ASSERT_FALSE(out.cell2Valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -9999.0f, out.grams);
}

void test_weight_without_ack_field() {
    // Five fields (no tare ack) still parse; the ack reads as 0.
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip("W,12.00,6.00,6.00,3,9", m, r));
    scalewire::Reading out;
    TEST_ASSERT_TRUE(scalewire::parseWeight(m, out));
    TEST_ASSERT_TRUE(out.available());
    TEST_ASSERT_EQUAL_UINT32(0, out.tareAck);
}

void test_tare_ids() {
    LineReader r;
    Message m;
    uint32_t id = 1;
    TEST_ASSERT_TRUE(roundTrip("Z,734512", m, r));
    TEST_ASSERT_TRUE(scalewire::parseTare(m, id));
    TEST_ASSERT_EQUAL_UINT32(734512, id);
    TEST_ASSERT_TRUE(roundTrip("Z", m, r));
    TEST_ASSERT_TRUE(scalewire::parseTare(m, id));
    TEST_ASSERT_EQUAL_UINT32(0, id);
    TEST_ASSERT_TRUE(roundTrip("W,1,1,0,3,1", m, r));
    TEST_ASSERT_FALSE(scalewire::parseTare(m, id));
}

void test_one_cell_counts_as_no_scale() {
    // Only one cell good: the display must get the stock "unavailable" message, not half a weight.
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip("W,12.00,12.00,0.00,1,3", m, r));
    scalewire::Reading out;
    TEST_ASSERT_TRUE(scalewire::parseWeight(m, out));
    TEST_ASSERT_FALSE(out.available());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -9999.0f, out.grams);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.cell1);
}

void test_bad_weight_lines_rejected() {
    const char *bad[] = {
        "W,12.0,6.0,6.0,3",          // missing seq
        "W,abc,6.0,6.0,3,1",         // not a number
        "W,12.0,6.0,6.0,4,1",        // valid out of range
        "W,12.0,6.0,6.0,-1,1",       // valid negative
        "W,9000.0,4500,4500,3,1",    // beyond MAX_GRAMS
        "W,12.0,,6.0,3,1",           // empty cell with valid flags
        "WX,12.0,6.0,6.0,3,1",       // other type
    };
    for (const char *p : bad) {
        LineReader r;
        Message m;
        TEST_ASSERT_TRUE_MESSAGE(roundTrip(p, m, r), p);
        scalewire::Reading out;
        TEST_ASSERT_FALSE_MESSAGE(scalewire::parseWeight(m, out), p);
    }
}

void test_config_round_trip() {
    scalewire::Config in;
    in.factor1 = -2513.47f;
    in.factor2 = 2489.12f;
    in.sps = 80;
    in.idleAlpha = 0.40f;
    in.activeAlpha = 0.40f;
    char payload[MAX_LINE];
    TEST_ASSERT_TRUE(scalewire::formatConfig(in, payload, sizeof(payload)) > 0);
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(roundTrip(payload, m, r));
    scalewire::Config out;
    TEST_ASSERT_TRUE(scalewire::parseConfig(m, out));
    TEST_ASSERT_TRUE(scalewire::sameConfig(in, out));
    TEST_ASSERT_EQUAL_UINT16(80, out.sps);
}

void test_config_change_detected() {
    scalewire::Config a, b;
    TEST_ASSERT_TRUE(scalewire::sameConfig(a, b));
    b.factor2 = 2500.01f;
    TEST_ASSERT_FALSE(scalewire::sameConfig(a, b));
    b = a;
    b.sps = 80;
    TEST_ASSERT_FALSE(scalewire::sameConfig(a, b));
    b = a;
    b.idleAlpha = 0.79f;
    TEST_ASSERT_FALSE(scalewire::sameConfig(a, b));
}

void test_bad_config_rejected() {
    const char *bad[] = {"C,-2500,2500,10,0.8", "C,-2500,x,10,0.8,0.8", "C,-2500,2500,0,0.8,0.8", "C,-2500,2500,10,,0.8"};
    for (const char *p : bad) {
        LineReader r;
        Message m;
        TEST_ASSERT_TRUE_MESSAGE(roundTrip(p, m, r), p);
        scalewire::Config out;
        TEST_ASSERT_FALSE_MESSAGE(scalewire::parseConfig(m, out), p);
    }
}

void test_worst_case_lines_fit() {
    scalewire::Reading w;
    w.grams = -4999.99f;
    w.cell1 = -4999.99f;
    w.cell2 = -4999.99f;
    w.cell1Valid = w.cell2Valid = true;
    w.seq = 4294967295u;
    w.tareAck = 4294967295u;
    char payload[MAX_LINE];
    const int n = scalewire::formatWeight(w, payload, sizeof(payload));
    TEST_ASSERT_TRUE(n > 0 && static_cast<size_t>(n) + 4 <= MAX_LINE);
    scalewire::Config c;
    c.factor1 = -1234567.125f;
    c.factor2 = 1234567.125f;
    c.sps = 80;
    const int k = scalewire::formatConfig(c, payload, sizeof(payload));
    TEST_ASSERT_TRUE(k > 0 && static_cast<size_t>(k) + 4 <= MAX_LINE);
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_weight_round_trip);
    RUN_TEST(test_negative_weight_kept);
    RUN_TEST(test_unavailable_is_stock_sentinel);
    RUN_TEST(test_weight_without_ack_field);
    RUN_TEST(test_tare_ids);
    RUN_TEST(test_one_cell_counts_as_no_scale);
    RUN_TEST(test_bad_weight_lines_rejected);
    RUN_TEST(test_config_round_trip);
    RUN_TEST(test_config_change_detected);
    RUN_TEST(test_bad_config_rejected);
    RUN_TEST(test_worst_case_lines_fit);
    return UNITY_END();
}
