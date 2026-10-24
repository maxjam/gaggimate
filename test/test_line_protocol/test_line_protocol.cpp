// Unit tests: the checksummed line protocol of the meter/scale serial links (fork).
// Host-side, STL only — pio test -e native -f test_line_protocol.

#include <unity.h>

#include <cstring>
#include <string>

#include "peripherals/LineProtocol.h"

using namespace lineproto;

static bool feedAll(LineReader &r, const std::string &bytes, Message &msg) {
    bool got = false;
    for (char c : bytes) {
        if (r.feed(c, msg))
            got = true;
    }
    return got;
}

void test_frame_matches_nmea_xor() {
    char out[64];
    const size_t n = frame("F,2.000,36.50,17", out, sizeof(out));
    TEST_ASSERT_TRUE(n > 0);
    // XOR of "F,2.000,36.50,17" computed independently
    uint8_t x = 0;
    for (const char *p = "F,2.000,36.50,17"; *p; p++)
        x ^= static_cast<uint8_t>(*p);
    char expect[64];
    snprintf(expect, sizeof(expect), "$F,2.000,36.50,17*%02X\r\n", x);
    TEST_ASSERT_EQUAL_STRING(expect, out);
}

void test_round_trip_fields() {
    char out[64];
    frame("F,2.125,36.50,17", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
    TEST_ASSERT_EQUAL_STRING("F", m.type);
    TEST_ASSERT_EQUAL_UINT(3, m.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.125f, m.f(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 36.5f, m.f(1));
    TEST_ASSERT_EQUAL_INT(17, m.i(2));
    TEST_ASSERT_EQUAL_UINT32(1, r.goodLines());
    TEST_ASSERT_EQUAL_UINT32(0, r.badLines());
}

void test_bad_checksum_rejected() {
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, "$F,2.000,36.50,17*00\n", m));
    TEST_ASSERT_EQUAL_UINT32(1, r.badLines());
}

void test_corrupted_byte_rejected() {
    char out[64];
    frame("T,143", out, sizeof(out));
    out[3] = '9'; // flip a digit, checksum no longer matches
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, out, m));
}

void test_boot_text_and_noise_skipped() {
    char good[64];
    frame("H,12", good, sizeof(good));
    std::string stream = "ESP-ROM:esp32s3-20210327\r\nBuild:Mar 27 2021\r\nrst:0x1 (POWERON),boot:0x8\r\n\x00\xff garbage";
    stream += good;
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, stream, m));
    TEST_ASSERT_EQUAL_STRING("H", m.type);
    TEST_ASSERT_EQUAL_INT(12, m.i(0));
}

void test_partial_line_resyncs_on_dollar() {
    char good[64];
    frame("F,1.500,10.00,3", good, sizeof(good));
    std::string stream = "$F,0.900,5.0";
    stream += good; // the first line never ended; the '$' of the next one restarts the reader
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, stream, m));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, m.f(0));
    TEST_ASSERT_EQUAL_UINT32(1, r.goodLines());
}

void test_overlong_line_dropped() {
    std::string stream = "$";
    stream += std::string(200, 'A');
    stream += "*00\n";
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, stream, m));
}

void test_missing_fields_fallback() {
    char out[64];
    frame("Z", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
    TEST_ASSERT_EQUAL_STRING("Z", m.type);
    TEST_ASSERT_EQUAL_UINT(0, m.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, m.f(0, -1.0f));
}

void test_gains_line() {
    char out[96];
    frame("G,0.0500,0.1500,20,0.00,4.0,1.50,0.40", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
    TEST_ASSERT_EQUAL_STRING("G", m.type);
    TEST_ASSERT_EQUAL_UINT(7, m.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, m.f(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, m.f(2));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.0f, m.f(4));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.f(6));
}

void test_two_lines_back_to_back() {
    char a[64], b[64];
    frame("F,2.000,1.00,1", a, sizeof(a));
    frame("F,2.100,1.10,2", b, sizeof(b));
    LineReader r;
    Message m;
    int got = 0;
    float last = 0;
    for (const char *s : {a, b}) {
        for (const char *p = s; *p; p++) {
            if (r.feed(*p, m)) {
                got++;
                last = m.f(0);
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(2, got);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.1f, last);
}

void test_star_inside_field_truncates_but_checksum_rules() {
    // "$F,1*5,2*XX": the parser takes the LAST '*' as the checksum marker; the inner '*' stays in a field.
    char line[] = "F,1*5,2";
    char out[64];
    frame(line, out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
    TEST_ASSERT_EQUAL_UINT(2, m.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.f(0)); // strtof stops at '*'
}

void test_empty_fields_fall_back() {
    char out[64];
    frame("F,,,", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
    TEST_ASSERT_EQUAL_UINT(3, m.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, m.f(0, -1.0f));
    TEST_ASSERT_EQUAL_INT(7, m.i(2, 7));
}

void test_too_many_fields_rejected() {
    char out[96];
    frame("X,1,2,3,4,5,6,7,8,9,10", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, out, m));
}

void test_lowercase_hex_accepted() {
    char out[64];
    frame("H,5", out, sizeof(out));
    for (char *p = out; *p; p++)
        if (*p >= 'A' && *p <= 'F' && p > strchr(out, '*'))
            *p = static_cast<char>(*p - 'A' + 'a');
    LineReader r;
    Message m;
    TEST_ASSERT_TRUE(feedAll(r, out, m));
}

void test_lone_dollar_and_bare_newlines() {
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, "\r\n\r\n$\r\n$*\n", m));
    char out[64];
    frame("Z", out, sizeof(out));
    TEST_ASSERT_TRUE(feedAll(r, out, m)); // and the reader is still usable afterwards
}

void test_long_type_rejected() {
    char out[64];
    frame("FLOW,1", out, sizeof(out));
    LineReader r;
    Message m;
    TEST_ASSERT_FALSE(feedAll(r, out, m));
}

void setUp() {}
void tearDown() {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_frame_matches_nmea_xor);
    RUN_TEST(test_round_trip_fields);
    RUN_TEST(test_bad_checksum_rejected);
    RUN_TEST(test_corrupted_byte_rejected);
    RUN_TEST(test_boot_text_and_noise_skipped);
    RUN_TEST(test_partial_line_resyncs_on_dollar);
    RUN_TEST(test_overlong_line_dropped);
    RUN_TEST(test_missing_fields_fallback);
    RUN_TEST(test_gains_line);
    RUN_TEST(test_two_lines_back_to_back);
    RUN_TEST(test_star_inside_field_truncates_but_checksum_rules);
    RUN_TEST(test_empty_fields_fall_back);
    RUN_TEST(test_too_many_fields_rejected);
    RUN_TEST(test_lowercase_hex_accepted);
    RUN_TEST(test_lone_dollar_and_bare_newlines);
    RUN_TEST(test_long_type_rejected);
    return UNITY_END();
}
