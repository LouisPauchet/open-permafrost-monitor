#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "geophone_parse.h"
#include "geophone_timing.h"

void setUp() {}
void tearDown() {}

static GeophonePacket pkt;

static bool parse(const char *s) {
  return geophoneParsePacket(s, strlen(s), &pkt);
}

// --- geophone_parse ---

static void test_parse_channel_packet() {
  TEST_ASSERT_TRUE(parse("{\"MA\": \"RS3D-11-4.19\", \"DF\": \"1.0\", \"CN\": \"EH1\", \"TS\": 0, "
                         "\"TSM\": 0, \"TQ\": 45, \"SI\": 5000, \"DS\": [\"3F62\", \"4051\", \"FFFFA804\"]}"));
  TEST_ASSERT_TRUE(pkt.isChannelData);
  TEST_ASSERT_FALSE(pkt.isMarker);
  TEST_ASSERT_EQUAL_STRING("EH1", pkt.channel);
  TEST_ASSERT_EQUAL_UINT32(5000, pkt.sampleIntervalUs);
  TEST_ASSERT_EQUAL(3, pkt.count);
  TEST_ASSERT_EQUAL_INT32(16226, pkt.samples[0]);
  TEST_ASSERT_EQUAL_INT32(0x4051, pkt.samples[1]);
  TEST_ASSERT_EQUAL_INT32(-22524, pkt.samples[2]); // all-8-digit negative, see raspberry_shake.md
}

static void test_parse_compact_and_lowercase() {
  TEST_ASSERT_TRUE(parse("{\"CN\":\"EH3\",\"SI\":5000,\"DS\":[\"0\",\"ffffffff\",\"7fffffff\"]}"));
  TEST_ASSERT_EQUAL_STRING("EH3", pkt.channel);
  TEST_ASSERT_EQUAL_INT32(0, pkt.samples[0]);
  TEST_ASSERT_EQUAL_INT32(-1, pkt.samples[1]);
  TEST_ASSERT_EQUAL_INT32(2147483647, pkt.samples[2]);
}

static void test_parse_fifty_samples() {
  char buf[1024];
  size_t n = (size_t)snprintf(buf, sizeof(buf), "{\"CN\": \"EH2\", \"SI\": 5000, \"DS\": [");
  for (int i = 0; i < 50; i++) {
    n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s\"%X\"", i ? ", " : "", (unsigned)(i * 1000 - 20000));
  }
  snprintf(buf + n, sizeof(buf) - n, "]}");
  TEST_ASSERT_TRUE(parse(buf));
  TEST_ASSERT_EQUAL(50, pkt.count);
  TEST_ASSERT_EQUAL_INT32(-20000, pkt.samples[0]);
  TEST_ASSERT_EQUAL_INT32(29000, pkt.samples[49]);
}

static void test_parse_marker() {
  TEST_ASSERT_TRUE(parse("{\"MSEC\": 64500, \"LS\": 0}"));
  TEST_ASSERT_TRUE(pkt.isMarker);
  TEST_ASSERT_FALSE(pkt.isChannelData);
  TEST_ASSERT_EQUAL_UINT32(64500, pkt.msec);
}

static void test_parse_rejects_malformed() {
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\", \"DS\": [\"3F62\", \"40"));       // truncated
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\", \"DS\": [\"3F62\", \"4051\""));  // no ']'
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\", \"DS\": [\"3G62\"]}"));          // non-hex
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\", \"DS\": [\"123456789\"]}"));     // > 32 bits
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\", \"DS\": [\"\"]}"));              // empty sample
  TEST_ASSERT_FALSE(parse("{\"CN\": \"EH1\"}"));                              // no DS
  TEST_ASSERT_FALSE(parse("{\"LS\": 0}"));                                    // neither shape
  TEST_ASSERT_FALSE(parse("{\"MSEC\": x}"));
}

static void test_parse_rejects_too_many_samples() {
  char buf[2048];
  size_t n = (size_t)snprintf(buf, sizeof(buf), "{\"CN\":\"EH1\",\"DS\":[");
  for (size_t i = 0; i < GEOPHONE_MAX_SAMPLES + 1; i++) {
    n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s\"1\"", i ? "," : "");
  }
  snprintf(buf + n, sizeof(buf) - n, "]}");
  TEST_ASSERT_FALSE(parse(buf));
}

// --- geophone_timing ---

static const int64_t STEP = 50000, RESYNC = 20000;
static const int64_t T0 = 1760000000000000LL; // some UTC epoch, us

static void test_clock_tracks_jitter_without_stepping() {
  GeoClock c;
  TEST_ASSERT_TRUE(geoClockOnMarker(&c, 1000, T0, STEP)); // first marker anchors
  int64_t first = geoClockRoundStartUs(&c);
  TEST_ASSERT_EQUAL_INT64(T0, first);
  for (int i = 1; i <= 400; i++) {
    int64_t jitter = (i % 3 - 1) * 8000; // +-8 ms of poll jitter
    TEST_ASSERT_FALSE(geoClockOnMarker(&c, 1000 + 250 * i, T0 + 250000LL * i + jitter, STEP));
    int64_t err = geoClockRoundStartUs(&c) - (T0 + 250000LL * i);
    TEST_ASSERT_TRUE(err < 10000 && err > -10000);
  }
}

static void test_clock_steps_on_big_jump() {
  GeoClock c;
  geoClockOnMarker(&c, 0, T0, STEP);
  TEST_ASSERT_TRUE(geoClockOnMarker(&c, 250, T0 + 250000 + 900000, STEP)); // GNSS re-anchor by 0.9 s
  TEST_ASSERT_EQUAL_INT64(T0 + 1150000, geoClockRoundStartUs(&c));
}

static void test_clock_msec_wraps() {
  GeoClock c;
  geoClockOnMarker(&c, 0xFFFFFF00u, T0, STEP);
  TEST_ASSERT_FALSE(geoClockOnMarker(&c, 0xFFFFFF00u + 250u, T0 + 250000, STEP)); // wraps past 2^32
  TEST_ASSERT_EQUAL_INT64(T0 + 250000, geoClockRoundStartUs(&c));
}

static void test_channel_continuity_and_resync() {
  int64_t next = 0;
  bool have = false, resynced = false;
  // First packet: snaps to the round start.
  TEST_ASSERT_EQUAL_INT64(T0, geoChannelPacketStartUs(&next, &have, T0, RESYNC, &resynced));
  TEST_ASSERT_TRUE(resynced);
  next += 50 * 5000; // writer advances by the packet's samples
  // Next round's start a few ms off: keep the continuous sample clock.
  TEST_ASSERT_EQUAL_INT64(T0 + 250000,
                          geoChannelPacketStartUs(&next, &have, T0 + 253000, RESYNC, &resynced));
  TEST_ASSERT_FALSE(resynced);
  next += 50 * 5000;
  // A lost round (marker 500 ms on): snap, leaving a real gap.
  TEST_ASSERT_EQUAL_INT64(T0 + 750000,
                          geoChannelPacketStartUs(&next, &have, T0 + 750000, RESYNC, &resynced));
  TEST_ASSERT_TRUE(resynced);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_channel_packet);
  RUN_TEST(test_parse_compact_and_lowercase);
  RUN_TEST(test_parse_fifty_samples);
  RUN_TEST(test_parse_marker);
  RUN_TEST(test_parse_rejects_malformed);
  RUN_TEST(test_parse_rejects_too_many_samples);
  RUN_TEST(test_clock_tracks_jitter_without_stepping);
  RUN_TEST(test_clock_steps_on_big_jump);
  RUN_TEST(test_clock_msec_wraps);
  RUN_TEST(test_channel_continuity_and_resync);
  return UNITY_END();
}
