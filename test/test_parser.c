/*
 * Framing tests.
 *
 * These exist to catch the failure the old 9600-baud reader actually had: a
 * dropped byte desynchronizing the stream with nothing to resynchronize
 * against.  So the interesting cases here are not "a good packet parses" but
 * the nasty ones, garbage in front, a byte lost mid-packet, a payload that
 * contains the header bytes, and a corrupted checksum.
 */

#include "sensorhub/sensorhub.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_failures++;                                                     \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

static sh_packet_t sample_packet(void)
{
    sh_packet_t p;
    memset(&p, 0, sizeof(p));
    p.speed = 23.5f;
    p.airspeed = 19.25f;
    p.engineTemp = 180.0f;
    p.radTemp = 148.5f;
    p.channel0 = 1;
    p.channel1 = 0;
    p.channel2 = 1;
    p.channel3 = 1;
    p.channel4 = 0;
    p.channelA0 = 812;
    return p;
}

static bool packets_equal(const sh_packet_t *a, const sh_packet_t *b)
{
    return memcmp(a, b, sizeof(sh_packet_t)) == 0;
}

/* Feed a buffer through a parser, returning how many packets came out. */
static int feed_all(sh_parser_t *parser, const uint8_t *data, size_t len,
                    sh_packet_t *last)
{
    int count = 0;
    sh_packet_t out;

    for (size_t i = 0; i < len; ++i) {
        if (sh_parser_feed(parser, data[i], &out)) {
            count++;
            if (last != NULL) {
                *last = out;
            }
        }
    }

    return count;
}

static void test_roundtrip(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_encode_frame(&in, frame);
    sh_parser_init(&parser);

    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 1,
          "one frame should yield one packet");
    CHECK(packets_equal(&in, &out), "round-tripped packet should match");
    CHECK(parser.stats.packets == 1, "packet counter should be 1");
    CHECK(parser.stats.checksum_errors == 0, "no checksum errors expected");
}

static void test_packet_size_is_on_the_wire_contract(void)
{
    CHECK(sizeof(sh_packet_t) == 23, "payload must stay 23 bytes");
    CHECK(SH_FRAME_SIZE == 26, "frame must stay 26 bytes");
}

static void test_leading_garbage(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[64];
    sh_parser_t parser;
    size_t n = 0;

    /* Junk, including a lone header byte, before a good frame. */
    buf[n++] = 0x00;
    buf[n++] = 0xFF;
    buf[n++] = SH_HEADER_1;
    buf[n++] = 0x12;
    buf[n++] = 0x34;

    sh_encode_frame(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, n, &out) == 1,
          "should recover from leading garbage");
    CHECK(packets_equal(&in, &out), "recovered packet should match");
}

static void test_payload_containing_header_bytes(void)
{
    /* A payload byte equal to 0xAA immediately before the real 0xAA 0x55 is
       the case that a naive two-state matcher gets wrong. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[8 + SH_FRAME_SIZE];
    sh_parser_t parser;
    size_t n = 0;

    buf[n++] = SH_HEADER_1;
    buf[n++] = SH_HEADER_1;
    buf[n++] = SH_HEADER_1;

    sh_encode_frame(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, n, &out) == 1,
          "repeated 0xAA before a header must not break framing");
    CHECK(packets_equal(&in, &out), "packet after repeated 0xAA should match");
}

static void test_bad_checksum_is_counted_not_silent(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_encode_frame(&in, frame);
    frame[SH_FRAME_SIZE - 1] ^= 0xFFu; /* corrupt the checksum */

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 0,
          "a bad checksum must not produce a packet");
    CHECK(parser.stats.checksum_errors == 1,
          "a bad checksum must be counted, not dropped silently");
}

static void test_corrupt_payload_is_rejected(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_encode_frame(&in, frame);
    frame[5] ^= 0x01u; /* flip a bit in the payload, checksum now wrong */

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 0,
          "a corrupted payload must be rejected");
    CHECK(parser.stats.checksum_errors == 1, "corruption should be counted");
}

static void test_resync_after_dropped_byte(void)
{
    /* The real-world failure: one byte lost in transit.  The truncated frame
       must be discarded and the NEXT frame must still parse.  Without a
       header this is exactly where the old reader went permanently wrong. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[SH_FRAME_SIZE * 3];
    sh_parser_t parser;
    size_t n = 0;

    sh_encode_frame(&in, buf);
    /* Copy all but the last byte of frame one: a dropped tail. */
    n = SH_FRAME_SIZE - 1;

    sh_encode_frame(&in, buf + n);
    n += SH_FRAME_SIZE;
    sh_encode_frame(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    int got = feed_all(&parser, buf, n, &out);

    CHECK(got >= 1, "must resynchronize after a dropped byte, got %d", got);
    CHECK(packets_equal(&in, &out), "post-resync packet should match");
}

static void test_split_across_reads(void)
{
    /* Serial reads arrive in arbitrary chunks; the parser is byte-at-a-time
       so this should be invisible, but prove it rather than assume it. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;
    int count = 0;

    sh_encode_frame(&in, frame);
    sh_parser_init(&parser);

    for (size_t i = 0; i < sizeof(frame); ++i) {
        if (sh_parser_feed(&parser, frame[i], &out)) {
            count++;
        }
    }

    CHECK(count == 1, "byte-at-a-time feeding should yield exactly one packet");
    CHECK(packets_equal(&in, &out), "split packet should match");
}

static void test_back_to_back_frames(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[SH_FRAME_SIZE * 5];
    sh_parser_t parser;

    for (int i = 0; i < 5; ++i) {
        in.speed = (float)i;
        sh_encode_frame(&in, buf + ((size_t)i * SH_FRAME_SIZE));
    }

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, sizeof(buf), &out) == 5,
          "five frames should yield five packets");
    CHECK(out.speed == 4.0f, "last packet should be the last one sent");
}

static void test_null_arguments_are_safe(void)
{
    sh_parser_t parser;
    sh_packet_t out;

    sh_parser_init(NULL); /* must not crash */
    sh_parser_init(&parser);

    CHECK(!sh_parser_feed(NULL, 0x00, &out), "NULL parser should return false");
    CHECK(!sh_parser_feed(&parser, 0x00, NULL), "NULL out should return false");
    CHECK(sh_checksum(NULL, 10) == 0, "NULL checksum input should return 0");
    sh_encode_frame(NULL, NULL); /* must not crash */
}

static void test_checksum_matches_the_arduino(void)
{
    /* The Arduino XORs every payload byte. Pin the algorithm with a value
       computed by hand so a "clever" rewrite cannot quietly change it. */
    const uint8_t data[] = { 0x01, 0x02, 0x03 }; /* 1^2^3 == 0 */
    const uint8_t data2[] = { 0xAA, 0x55 };      /* 0xAA^0x55 == 0xFF */

    CHECK(sh_checksum(data, sizeof(data)) == 0x00, "1^2^3 should be 0");
    CHECK(sh_checksum(data2, sizeof(data2)) == 0xFF, "0xAA^0x55 should be 0xFF");
}

static void test_wire_format_is_frozen(void)
{
    /*
     * A golden frame, byte for byte.
     *
     * This is the contract with carsensordriver.ino on the Arduino and with
     * every CSV already on disk. It was verified byte-identical against the
     * firmware's original hand-rolled send path over 200,000 random packets
     * before that code was deleted in favour of sh_encode_frame().
     *
     * If this test fails, the wire format changed. That is not a test to
     * update; it is a flash-the-Arduino-and-tell-everyone event.
     */
    static const uint8_t golden[SH_FRAME_SIZE] = {
        0xAA, 0x55,                          /* header                  */
        0x00, 0x00, 0xBC, 0x41,              /* speed        23.5       */
        0x00, 0x00, 0x9A, 0x41,              /* airspeed     19.25      */
        0x00, 0x00, 0x34, 0x43,              /* engineTemp   180.0      */
        0x00, 0x80, 0x14, 0x43,              /* radTemp      148.5      */
        0x01, 0x00, 0x01, 0x01, 0x00,        /* channel0..4             */
        0x2C, 0x03,                          /* channelA0    812        */
        0xA8                                 /* XOR checksum            */
    };

    sh_packet_t in = sample_packet();
    uint8_t frame[SH_FRAME_SIZE];

    sh_encode_frame(&in, frame);

    CHECK(memcmp(frame, golden, sizeof(golden)) == 0,
          "encoded frame must match the frozen wire format byte for byte");

    /* And the parser must accept its own golden frame, which catches an
       encoder and decoder that drifted together. */
    {
        sh_parser_t parser;
        sh_packet_t out;
        sh_parser_init(&parser);
        CHECK(feed_all(&parser, golden, sizeof(golden), &out) == 1,
              "the frozen frame must still decode");
        CHECK(packets_equal(&in, &out), "decoded golden frame should match");
    }
}

int main(void)
{
    test_packet_size_is_on_the_wire_contract();
    test_wire_format_is_frozen();
    test_checksum_matches_the_arduino();
    test_roundtrip();
    test_leading_garbage();
    test_payload_containing_header_bytes();
    test_bad_checksum_is_counted_not_silent();
    test_corrupt_payload_is_rejected();
    test_resync_after_dropped_byte();
    test_split_across_reads();
    test_back_to_back_frames();
    test_null_arguments_are_safe();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
