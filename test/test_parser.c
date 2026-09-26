/*
 * Framing tests.
 *
 * These exist to catch the failures that matter, not to demonstrate that a
 * good packet parses. The interesting cases are garbage in front of a frame,
 * a byte lost mid-packet, a payload containing the header bytes, a corrupted
 * checksum, and, since the format now carries a version, a sender speaking a
 * dialect we do not know.
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
    p.temps[SH_TEMP_ENGINE] = 180.0f;
    p.temps[SH_TEMP_RADIATOR] = 148.5f;
    p.temps[2] = 0.0f;
    p.temps[3] = 0.0f;
    p.analog[SH_ANALOG_BATTERY] = 812;
    p.digital_in = 0x0Du;  /* channels 0, 2, 3 */
    p.digital_out = 0x02u; /* output 1 */
    p.sequence = 7;
    return p;
}

static bool packets_equal(const sh_packet_t *a, const sh_packet_t *b)
{
    return memcmp(a, b, sizeof(sh_packet_t)) == 0;
}

static void encode(const sh_packet_t *p, uint8_t *buf)
{
    sh_status_t st = sh_encode_frame(p, buf, SH_FRAME_SIZE, NULL);
    CHECK(st == SH_OK, "encode should succeed, got %s", sh_strstatus(st));
}

/* Feed a buffer through a parser, returning how many packets came out. */
static int feed_all(sh_parser_t *parser, const uint8_t *data, size_t len,
                    sh_packet_t *last)
{
    int count = 0;
    sh_packet_t out;

    for (size_t i = 0; i < len; ++i) {
        if (sh_parser_feed(parser, data[i], &out) == SH_OK) {
            count++;
            if (last != NULL) *last = out;
        }
    }

    return count;
}

static void test_sizes_are_the_wire_contract(void)
{
    CHECK(sizeof(sh_packet_t) == 36, "payload must stay 36 bytes");
    CHECK(SH_FRAME_SIZE == 42, "frame must stay 42 bytes");
    CHECK(SH_FRAME_OVERHEAD == 6, "overhead is header, fmt, len, 2 CRC bytes");
}

static void test_roundtrip(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    encode(&in, frame);
    sh_parser_init(&parser);

    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 1,
          "one frame should yield one packet");
    CHECK(packets_equal(&in, &out), "round-tripped packet should match");
    CHECK(parser.stats.packets == 1, "packet counter should be 1");
    CHECK(parser.stats.checksum_errors == 0, "no checksum errors expected");
}

static void test_leading_garbage(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[80];
    sh_parser_t parser;
    size_t n = 0;

    buf[n++] = 0x00;
    buf[n++] = 0xFF;
    buf[n++] = SH_HEADER_1; /* a lone header byte going nowhere */
    buf[n++] = 0x12;
    buf[n++] = 0x34;

    encode(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, n, &out) == 1,
          "should recover from leading garbage");
    CHECK(packets_equal(&in, &out), "recovered packet should match");
}

static void test_payload_containing_header_bytes(void)
{
    /* A payload byte equal to 0xAA immediately before the real 0xAA 0x55 is
       the case a naive two-state matcher gets wrong. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[8 + SH_FRAME_SIZE];
    sh_parser_t parser;
    size_t n = 0;

    buf[n++] = SH_HEADER_1;
    buf[n++] = SH_HEADER_1;
    buf[n++] = SH_HEADER_1;

    encode(&in, buf + n);
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
    sh_status_t last = SH_OK;

    encode(&in, frame);
    frame[SH_FRAME_SIZE - 1] ^= 0xFFu;

    sh_parser_init(&parser);
    for (size_t i = 0; i < sizeof(frame); ++i) {
        last = sh_parser_feed(&parser, frame[i], &out);
    }

    CHECK(last == SH_E_CHECKSUM, "a bad CRC must report SH_E_CHECKSUM");
    CHECK(parser.stats.checksum_errors == 1, "and must be counted");
    CHECK(parser.stats.packets == 0, "and must not yield a packet");
}

static void test_corrupt_payload_is_rejected(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    encode(&in, frame);
    frame[8] ^= 0x01u; /* flip a bit inside the payload */

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 0,
          "a corrupted payload must be rejected");
    CHECK(parser.stats.checksum_errors == 1, "corruption should be counted");
}

static void test_resync_after_dropped_byte(void)
{
    /* The real-world failure: one byte lost in transit. The truncated frame
       must be discarded and the next one must still parse. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[SH_FRAME_SIZE * 3];
    sh_parser_t parser;
    size_t n;

    encode(&in, buf);
    n = SH_FRAME_SIZE - 1; /* drop the tail of frame one */

    encode(&in, buf + n);
    n += SH_FRAME_SIZE;
    encode(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    int got = feed_all(&parser, buf, n, &out);

    CHECK(got >= 1, "must resynchronise after a dropped byte, got %d", got);
    CHECK(packets_equal(&in, &out), "post-resync packet should match");
}

static void test_back_to_back_frames(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[SH_FRAME_SIZE * 5];
    sh_parser_t parser;

    for (int i = 0; i < 5; ++i) {
        in.speed = (float)i;
        in.sequence = (uint16_t)(100 + i);
        encode(&in, buf + ((size_t)i * SH_FRAME_SIZE));
    }

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, sizeof(buf), &out) == 5,
          "five frames should yield five packets");
    CHECK(out.speed == 4.0f, "last packet should be the last one sent");
    CHECK(parser.stats.dropped == 0, "a contiguous run drops nothing");
}

/* ---- the version and length bytes, which is why they are on the wire ---- */

static void test_unknown_format_is_refused_not_decoded(void)
{
    /* The whole point. A sender speaking a version we do not know must be
       rejected loudly, not decoded into plausible nonsense. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;
    sh_status_t seen = SH_OK;

    encode(&in, frame);
    frame[2] = 0x02u; /* a future format */
    /* fix the CRC so this is purely a version rejection, not a corruption
       that would have been caught anyway */
    {
        uint16_t crc = sh_crc16(frame + 2, SH_PAYLOAD_SIZE + 2u);
        frame[SH_FRAME_SIZE - 2] = (uint8_t)(crc & 0xFFu);
        frame[SH_FRAME_SIZE - 1] = (uint8_t)(crc >> 8);
    }

    sh_parser_init(&parser);
    for (size_t i = 0; i < sizeof(frame); ++i) {
        sh_status_t st = sh_parser_feed(&parser, frame[i], &out);
        if (st == SH_E_FORMAT) seen = st;
        CHECK(st != SH_OK, "an unknown format must never yield a packet");
    }

    CHECK(seen == SH_E_FORMAT, "should have reported SH_E_FORMAT");
    CHECK(parser.stats.format_errors == 1, "and counted it");
    CHECK(parser.stats.packets == 0, "and produced nothing");
}

static void test_length_lets_us_skip_a_newer_sender(void)
{
    /* An old receiver against a new sender: the length byte is what keeps it
       framed. After skipping a longer packet it does not understand, the very
       next packet it does understand must still parse. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t buf[128];
    sh_parser_t parser;
    size_t n = 0;

    /* A future frame: format 2, payload 50 bytes, then its two CRC bytes. */
    buf[n++] = SH_HEADER_1;
    buf[n++] = SH_HEADER_2;
    buf[n++] = 0x02u;
    buf[n++] = 50u;
    for (int i = 0; i < 50; ++i) buf[n++] = (uint8_t)i;
    buf[n++] = 0x00u;
    buf[n++] = 0x00u;

    /* Then a frame we do understand. */
    encode(&in, buf + n);
    n += SH_FRAME_SIZE;

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, buf, n, &out) == 1,
          "must stay framed across a packet from a newer sender");
    CHECK(packets_equal(&in, &out), "the packet after the skip should match");
    CHECK(parser.stats.format_errors == 1, "the unknown frame was counted");
}

static void test_wrong_length_for_our_own_format(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;
    sh_status_t seen = SH_OK;

    encode(&in, frame);
    frame[3] = 99u; /* our format, an impossible length */

    sh_parser_init(&parser);
    for (size_t i = 0; i < sizeof(frame); ++i) {
        sh_status_t st = sh_parser_feed(&parser, frame[i], &out);
        if (st == SH_E_LENGTH) seen = st;
    }

    CHECK(seen == SH_E_LENGTH, "a bad length must report SH_E_LENGTH");
    CHECK(parser.stats.packets == 0, "and must not yield a packet");
}

static void test_checksum_covers_the_header_fields(void)
{
    /* If the checksum only covered the payload, flipping the length byte
       would reframe the stream and still validate. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    encode(&in, frame);
    frame[3] = (uint8_t)(SH_PAYLOAD_SIZE - 1u); /* lie about the length */

    sh_parser_init(&parser);
    CHECK(feed_all(&parser, frame, sizeof(frame), &out) == 0,
          "a tampered length byte must not produce a packet");
}

static void test_dropped_packets_are_counted_from_the_sequence(void)
{
    /* A checksum cannot tell you about a packet that never arrived. The
       sequence number can. */
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_parser_init(&parser);

    in.sequence = 10;
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    in.sequence = 14; /* 11, 12, 13 never arrived */
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    CHECK(parser.stats.dropped == 3, "should have counted 3 dropped, got %lu",
          (unsigned long)parser.stats.dropped);
    CHECK(parser.stats.packets == 2, "two packets did arrive");
}

static void test_sequence_wrap_is_not_a_flood(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_parser_init(&parser);

    in.sequence = 65535;
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    in.sequence = 0; /* wrapped, not 65535 packets lost */
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    CHECK(parser.stats.dropped == 0, "a clean wrap drops nothing, got %lu",
          (unsigned long)parser.stats.dropped);
}

static void test_sender_restart_is_not_a_flood(void)
{
    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    sh_parser_init(&parser);

    in.sequence = 40000;
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    in.sequence = 0; /* the Arduino rebooted */
    encode(&in, frame);
    feed_all(&parser, frame, sizeof(frame), &out);

    CHECK(parser.stats.dropped == 0,
          "a restart should not report a fictional flood, got %lu",
          (unsigned long)parser.stats.dropped);
}

/* ---- the API surface itself ---- */

static void test_digital_accessors(void)
{
    sh_packet_t p;
    bool v;

    memset(&p, 0, sizeof(p));

    CHECK(sh_set_digital_in(&p, 3, true) == SH_OK, "set channel 3");
    CHECK(sh_digital_in(&p, 3, &v) == SH_OK && v, "channel 3 should read set");
    CHECK(sh_digital_in(&p, 4, &v) == SH_OK && !v, "channel 4 should be clear");

    CHECK(sh_set_digital_in(&p, 3, false) == SH_OK, "clear channel 3");
    CHECK(sh_digital_in(&p, 3, &v) == SH_OK && !v, "channel 3 should clear");

    CHECK(sh_set_digital_out(&p, 0, true) == SH_OK, "set output 0");
    CHECK(sh_digital_out(&p, 0, &v) == SH_OK && v, "output 0 should read set");
    /* outputs and inputs must not alias each other */
    CHECK(sh_digital_in(&p, 0, &v) == SH_OK && !v,
          "setting an output must not touch the inputs");
}

static void test_out_of_range_is_an_error_not_a_read(void)
{
    sh_packet_t p = sample_packet();
    bool v;
    float f;
    uint16_t a;

    CHECK(sh_digital_in(&p, 8, &v) == SH_E_RANGE, "channel 8 is out of range");
    CHECK(sh_digital_out(&p, 99, &v) == SH_E_RANGE, "so is 99");
    CHECK(sh_set_digital_in(&p, 8, true) == SH_E_RANGE, "and for writes");
    CHECK(sh_temp(&p, SH_TEMP_COUNT, &f) == SH_E_RANGE, "temp index bound");
    CHECK(sh_analog(&p, SH_ANALOG_COUNT, &a) == SH_E_RANGE, "analog bound");

    CHECK(sh_temp(&p, SH_TEMP_ENGINE, &f) == SH_OK && f == 180.0f,
          "a valid temp index still works");
    CHECK(sh_analog(&p, SH_ANALOG_BATTERY, &a) == SH_OK && a == 812,
          "a valid analog index still works");
}

static void test_encode_refuses_a_short_buffer(void)
{
    sh_packet_t p = sample_packet();
    uint8_t small[SH_FRAME_SIZE - 1];
    uint8_t ok[SH_FRAME_SIZE];
    size_t written = 0;

    CHECK(sh_encode_frame(&p, small, sizeof(small), &written) == SH_E_SPACE,
          "a short buffer must be refused, not overrun");
    CHECK(sh_encode_frame(&p, ok, sizeof(ok), &written) == SH_OK,
          "a correctly sized buffer works");
    CHECK(written == SH_FRAME_SIZE, "and reports what it wrote");
}

static void test_null_arguments_are_safe(void)
{
    sh_parser_t parser;
    sh_packet_t out;
    sh_packet_t p = sample_packet();
    uint8_t buf[SH_FRAME_SIZE];
    bool v;

    sh_parser_init(NULL); /* must not crash */
    sh_parser_init(&parser);

    CHECK(sh_parser_feed(NULL, 0, &out) == SH_E_NULL, "NULL parser");
    CHECK(sh_parser_feed(&parser, 0, NULL) == SH_E_NULL, "NULL out");
    CHECK(sh_encode_frame(NULL, buf, sizeof(buf), NULL) == SH_E_NULL,
          "NULL packet");
    CHECK(sh_encode_frame(&p, NULL, sizeof(buf), NULL) == SH_E_NULL,
          "NULL buffer");
    CHECK(sh_digital_in(NULL, 0, &v) == SH_E_NULL, "NULL packet to accessor");
    CHECK(sh_digital_in(&p, 0, NULL) == SH_E_NULL, "NULL out to accessor");
    CHECK(sh_crc16(NULL, 10) == 0xFFFFu,
          "NULL CRC input returns the initial value, not garbage");
}

static void test_status_strings_exist(void)
{
    /* A status with no message turns a diagnosable failure into "error 7". */
    static const sh_status_t all[] = {
        SH_OK, SH_INCOMPLETE, SH_E_NULL, SH_E_CHECKSUM, SH_E_FORMAT,
        SH_E_LENGTH, SH_E_RANGE, SH_E_SPACE, SH_E_OPEN, SH_E_IO,
        SH_E_INTERRUPTED, SH_E_CLOSED
    };

    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
        const char *s = sh_strstatus(all[i]);
        CHECK(s != NULL && s[0] != '\0' && strcmp(s, "unknown status") != 0,
              "status %d needs its own message", (int)all[i]);
    }

    CHECK(strcmp(sh_strstatus((sh_status_t)999), "unknown status") == 0,
          "a bogus status should still return a string, not NULL");
    CHECK(!sh_failed(SH_OK), "SH_OK is not a failure");
    CHECK(!sh_failed(SH_INCOMPLETE),
          "SH_INCOMPLETE is not a failure; it is the usual answer");
    CHECK(sh_failed(SH_E_CHECKSUM), "SH_E_CHECKSUM is a failure");
}

static void test_wire_format_is_frozen(void)
{
    /*
     * A golden frame, byte for byte. This is the contract with
     * carsensordriver.ino and with every consumer downstream.
     *
     * If this fails, the wire format changed. That is not a test to update,
     * it is a reflash-every-Arduino event: bump SH_FORMAT_CURRENT, regenerate
     * this, and tell everyone.
     */
    static const uint8_t golden[SH_FRAME_SIZE] = {
        0xAA, 0x55,             /* header                        */
        0x01,                   /* format 1                      */
        0x24,                   /* payload length, 36            */
        0x00, 0x00, 0xBC, 0x41, /* speed          23.5           */
        0x00, 0x00, 0x9A, 0x41, /* airspeed       19.25          */
        0x00, 0x00, 0x34, 0x43, /* temps[0]       180.0          */
        0x00, 0x80, 0x14, 0x43, /* temps[1]       148.5          */
        0x00, 0x00, 0x00, 0x00, /* temps[2]       0.0            */
        0x00, 0x00, 0x00, 0x00, /* temps[3]       0.0            */
        0x2C, 0x03,             /* analog[0]      812            */
        0x00, 0x00,             /* analog[1]                     */
        0x00, 0x00,             /* analog[2]                     */
        0x00, 0x00,             /* analog[3]                     */
        0x0D,                   /* digital_in     0b00001101     */
        0x02,                   /* digital_out    0b00000010     */
        0x07, 0x00,             /* sequence       7              */
        0x51, 0xF1              /* CRC-16 over fmt, len, payload, LE */
    };

    sh_packet_t in = sample_packet();
    sh_packet_t out;
    uint8_t frame[SH_FRAME_SIZE];
    sh_parser_t parser;

    encode(&in, frame);

    CHECK(memcmp(frame, golden, sizeof(golden)) == 0,
          "encoded frame must match the frozen wire format byte for byte");

    /* And the parser must accept its own golden frame, which catches an
       encoder and decoder that drifted together. */
    sh_parser_init(&parser);
    CHECK(feed_all(&parser, golden, sizeof(golden), &out) == 1,
          "the frozen frame must still decode");
    CHECK(packets_equal(&in, &out), "decoded golden frame should match");
}


/* ---- the corruptions an XOR checksum could not see ---- */

/* Feed a whole frame and report whether it was accepted. */
static bool frame_accepted(const uint8_t *frame, size_t len)
{
    sh_parser_t parser;
    sh_packet_t out;

    sh_parser_init(&parser);
    for (size_t i = 0; i < len; ++i) {
        if (sh_parser_feed(&parser, frame[i], &out) == SH_OK) return true;
    }
    return false;
}

static void test_two_flips_in_the_same_bit_position(void)
{
    /*
     * The reason this frame carries a CRC rather than an XOR sum.
     *
     * Two bit flips in the same bit position cancel exactly under XOR, so a
     * byte-sum accepted 100% of these in a two-million-frame simulation. It
     * is not an exotic case either: a ground bounce or a supply glitch
     * disturbing one data line flips the same bit in consecutive bytes,
     * which is exactly what an ignition system produces.
     */
    sh_packet_t in = sample_packet();
    uint8_t frame[SH_FRAME_SIZE];
    unsigned accepted = 0;

    for (unsigned bit = 0; bit < 8u; ++bit) {
        for (unsigned i = 4; i < 4u + SH_PAYLOAD_SIZE - 1u; ++i) {
            encode(&in, frame);
            frame[i] ^= (uint8_t)(1u << bit);
            frame[i + 1] ^= (uint8_t)(1u << bit);
            if (frame_accepted(frame, sizeof(frame))) accepted++;
        }
    }

    CHECK(accepted == 0,
          "paired same-bit flips must all be caught, %u slipped through",
          accepted);
}

static void test_swapped_bytes(void)
{
    /*
     * The other one. XOR is order-independent, so swapping two payload bytes
     * is completely invisible to it: also 100% undetected in simulation.
     */
    sh_packet_t in = sample_packet();
    uint8_t frame[SH_FRAME_SIZE];
    unsigned accepted = 0;
    unsigned tried = 0;

    for (unsigned i = 4; i < 4u + SH_PAYLOAD_SIZE; ++i) {
        for (unsigned j = i + 1u; j < 4u + SH_PAYLOAD_SIZE; ++j) {
            uint8_t tmp;

            encode(&in, frame);
            if (frame[i] == frame[j]) continue; /* a no-op swap */

            tmp = frame[i];
            frame[i] = frame[j];
            frame[j] = tmp;
            tried++;

            if (frame_accepted(frame, sizeof(frame))) accepted++;
        }
    }

    CHECK(tried > 100, "should have tried a decent number of swaps, got %u",
          tried);
    CHECK(accepted == 0, "swapped bytes must all be caught, %u slipped through",
          accepted);
}

static void test_every_single_bit_flip_is_caught(void)
{
    /* Across the whole frame, including the header fields and the CRC. */
    sh_packet_t in = sample_packet();
    uint8_t frame[SH_FRAME_SIZE];
    unsigned accepted = 0;

    for (unsigned i = 2; i < SH_FRAME_SIZE; ++i) {
        for (unsigned bit = 0; bit < 8u; ++bit) {
            encode(&in, frame);
            frame[i] ^= (uint8_t)(1u << bit);
            if (frame_accepted(frame, sizeof(frame))) accepted++;
        }
    }

    CHECK(accepted == 0, "every single-bit flip must be caught, %u were not",
          accepted);
}

static void test_crc_matches_the_reference_vector(void)
{
    /* CRC-16-CCITT (0x1021, init 0xFFFF) over "123456789" is 0x29B1. This is
       the standard check value; if it fails, the polynomial or the init
       constant drifted. */
    static const uint8_t check[] = "123456789";

    CHECK(sh_crc16(check, sizeof(check) - 1u) == 0x29B1u,
          "CRC-16-CCITT check value must be 0x29B1, got 0x%04X",
          (unsigned)sh_crc16(check, sizeof(check) - 1u));
}

int main(void)
{
    test_sizes_are_the_wire_contract();
    test_wire_format_is_frozen();
    test_roundtrip();
    test_leading_garbage();
    test_payload_containing_header_bytes();
    test_bad_checksum_is_counted_not_silent();
    test_corrupt_payload_is_rejected();
    test_resync_after_dropped_byte();
    test_back_to_back_frames();

    test_unknown_format_is_refused_not_decoded();
    test_length_lets_us_skip_a_newer_sender();
    test_wrong_length_for_our_own_format();
    test_checksum_covers_the_header_fields();
    test_dropped_packets_are_counted_from_the_sequence();
    test_sequence_wrap_is_not_a_flood();
    test_sender_restart_is_not_a_flood();

    test_digital_accessors();
    test_out_of_range_is_an_error_not_a_read();
    test_encode_refuses_a_short_buffer();
    test_null_arguments_are_safe();
    test_status_strings_exist();

    test_crc_matches_the_reference_vector();
    test_every_single_bit_flip_is_caught();
    test_two_flips_in_the_same_bit_position();
    test_swapped_bytes();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
