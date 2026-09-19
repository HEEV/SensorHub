/*
 * SensorHub: the Raspberry Pi side of the link to the Arduino Nano that reads
 * the car's sensors.
 *
 * The wire format is defined by SensorController/carsensordriver.ino and must
 * match it exactly:
 *
 *     0xAA  0x55  <23-byte packed payload>  <XOR checksum of payload>
 *
 * Twenty-six bytes on the wire, 115200 baud, 8N1.
 *
 * Parsing is deliberately separate from I/O.  sh_parser_feed() takes one byte
 * and never touches a file descriptor, so the framing can be tested against
 * synthetic input in CI, on any machine, with no car attached.  The serial
 * helpers below are a convenience for callers that do have hardware.
 */

#ifndef SENSORHUB_H
#define SENSORHUB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SH_HEADER_1 0xAAu
#define SH_HEADER_2 0x55u

/*
 * Field names mirror the Arduino struct character for character.  This is a
 * memcpy target off the wire, so the resemblance is load-bearing: if you
 * rename a field here, rename it there in the same commit.
 */
typedef struct __attribute__((packed)) {
    float speed;     /* ground speed, mph, from the wheel interrupt */
    float airspeed;  /* pitot, mph, zeroed at Arduino startup */
    float engineTemp; /* degrees F, DS18B20 */
    float radTemp;    /* degrees F, DS18B20 */

    uint8_t channel0; /* digital, pin 4  */
    uint8_t channel1; /* digital, pin 5  */
    uint8_t channel2; /* digital, pin 6  */
    uint8_t channel3; /* digital, pin 7  */
    uint8_t channel4; /* digital, pin 8  */

    uint16_t channelA0; /* analog, pin A7, currently battery voltage */
} sh_packet_t;

#define SH_PAYLOAD_SIZE 23u
#define SH_FRAME_SIZE   26u /* 2 header + payload + 1 checksum */

/* Both ends of this link are little-endian (AVR and aarch64).  A big-endian
   host would need byte swapping that no one has written, so say so loudly
   rather than decoding garbage. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#error "SensorHub assumes a little-endian host; the wire format is AVR-native"
#endif

typedef enum {
    SH_WAIT_HEADER_1 = 0,
    SH_WAIT_HEADER_2,
    SH_READ_PAYLOAD,
    SH_READ_CHECKSUM
} sh_state_t;

/* Counters worth watching on a bench test: a link that is losing packets
   shows up here long before it shows up on the dashboard.

   The width is configurable because this header also compiles for the
   ATmega328p, where 64-bit counters cost RAM a Nano does not have and every
   increment is a slow multi-word add. The AVR default is 32-bit rather than
   16: at roughly 40 packets a second, 16 bits wraps in under half an hour,
   and a diagnostic counter that quietly lies is worse than a larger one.
   32 bits lasts about three years of continuous running for six more bytes.
   Define SH_COUNTER_BITS before including to override. */
#ifndef SH_COUNTER_BITS
#  if defined(__AVR__) || defined(SH_EMBEDDED)
#    define SH_COUNTER_BITS 32
#  else
#    define SH_COUNTER_BITS 64
#  endif
#endif

#if SH_COUNTER_BITS == 16
typedef uint16_t sh_counter_t;
#elif SH_COUNTER_BITS == 32
typedef uint32_t sh_counter_t;
#elif SH_COUNTER_BITS == 64
typedef uint64_t sh_counter_t;
#else
#error "SH_COUNTER_BITS must be 16, 32, or 64"
#endif

typedef struct {
    sh_counter_t packets;         /* accepted */
    sh_counter_t checksum_errors; /* framed correctly, contents rejected */
    sh_counter_t resyncs;         /* fell back to hunting for a header */
} sh_stats_t;

typedef struct {
    sh_state_t state;
    uint8_t    payload[SH_PAYLOAD_SIZE];
    size_t     payload_index;
    sh_stats_t stats;
} sh_parser_t;

/* Reset a parser to hunting for a header.  Also zeroes the statistics. */
void sh_parser_init(sh_parser_t *parser);

/*
 * Feed exactly one received byte.
 *
 * Returns true when that byte completed a packet whose checksum matched, in
 * which case *out holds it.  Returns false every other time, including for a
 * packet that arrived intact but failed its checksum; that case increments
 * stats.checksum_errors so it can be distinguished from an idle link.
 *
 * Passing NULL for either argument is a no-op returning false.
 */
bool sh_parser_feed(sh_parser_t *parser, uint8_t byte, sh_packet_t *out);

/* XOR of every byte, the checksum the Arduino appends. Exposed so tests and
   any future transmitter can build frames without duplicating it. */
uint8_t sh_checksum(const uint8_t *data, size_t length);

/* Serialize a packet into a full 26-byte frame. buffer must have room for
   SH_FRAME_SIZE. Used by the tests to generate input, and by any simulator
   that wants to stand in for the Arduino. */
void sh_encode_frame(const sh_packet_t *packet, uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif /* SENSORHUB_H */
