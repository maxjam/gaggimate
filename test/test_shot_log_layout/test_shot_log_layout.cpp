#include <unity.h>
#include <cstddef>
#include <initializer_list>
#include <display/models/shot_log_format.h>

void setUp() {}
void tearDown() {}

void test_legacy_and_current_layouts() {
    ShotLogHeader h{};
    h.fieldsMask = 0x1FFF;
    h.version = 5;
    h.reserved0 = 26;
    TEST_ASSERT_FALSE(shotLogHasElapsedTimestamp(h));
    TEST_ASSERT_EQUAL_UINT8(26, shotLogSampleSize(h));
    h.version = 6;
    TEST_ASSERT_FALSE(shotLogHasElapsedTimestamp(h));
    TEST_ASSERT_EQUAL_UINT8(26, shotLogSampleSize(h));
    h.reserved0 = 28;
    TEST_ASSERT_TRUE(shotLogHasElapsedTimestamp(h));
    TEST_ASSERT_EQUAL_UINT8(28, shotLogSampleSize(h));
    h.fieldsMask = 0x3FFF;
    h.reserved0 = 30;
    for (uint8_t version : {7, 8}) {
        h.version = version;
        TEST_ASSERT_TRUE(shotLogHasElapsedTimestamp(h));
        TEST_ASSERT_EQUAL_UINT8(30, shotLogSampleSize(h));
    }
    h.reserved0 = 26;
    TEST_ASSERT_EQUAL_UINT8(0, shotLogSampleSize(h));
}

// Fork: pump power (bit 20) makes the record 32 bytes; the size follows the mask, so upstream files keep theirs.
void test_pump_power_layout() {
    TEST_ASSERT_EQUAL_UINT32(0x103FFF, SHOT_LOG_FIELDS_MASK_ALL);
    TEST_ASSERT_EQUAL_UINT32(32, SHOT_LOG_SAMPLE_SIZE);
    TEST_ASSERT_EQUAL_UINT32(32, sizeof(ShotLogSample));
    TEST_ASSERT_EQUAL_UINT32(30, offsetof(ShotLogSample, pp));
    ShotLogHeader h{};
    h.version = 8;
    h.fieldsMask = SHOT_LOG_FIELDS_MASK_ALL;
    h.reserved0 = 32;
    TEST_ASSERT_TRUE(shotLogHasElapsedTimestamp(h));
    TEST_ASSERT_EQUAL_UINT8(32, shotLogSampleSize(h));
    h.reserved0 = 0; // no recorded size: trust the mask
    TEST_ASSERT_EQUAL_UINT8(32, shotLogSampleSize(h));
    h.reserved0 = 30; // mask and recorded size disagree
    TEST_ASSERT_EQUAL_UINT8(0, shotLogSampleSize(h));
    h.fieldsMask = 0x3FFF; // upstream v8 file
    h.reserved0 = 30;
    TEST_ASSERT_EQUAL_UINT8(30, shotLogSampleSize(h));
    h.fieldsMask = 0;
    h.reserved0 = 0;
    TEST_ASSERT_EQUAL_UINT8(0, shotLogSampleSize(h));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_legacy_and_current_layouts);
    RUN_TEST(test_pump_power_layout);
    return UNITY_END();
}
