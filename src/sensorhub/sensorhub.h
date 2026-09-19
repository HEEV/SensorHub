/*
 * SensorHub: the wire format for the link between the car's Arduino Nano and
 * the Raspberry Pi. Used by both ends so the two cannot drift apart.
 *
 * FRAME
 *
 *     0xAA 0x55  fmt  len  payload[len]  checksum
 *
 *   fmt       format version. A receiver refuses anything it does not know
 *             rather than decoding it confidently and wrongly.
 *   len       payload length. Lets an old receiver skip a packet from a newer
 *             sender and stay framed, instead of desynchronising.
 *   checksum  XOR of fmt, len, and every payload byte.
 *
 * The version and length bytes are the difference between "we changed the
 * format and everything broke loudly" and "we changed the format and the car
 * logged plausible nonsense for a season". They cost two bytes on a link
 * running at 7% utilisation.
 *
 * PARSING IS SEPARATE FROM I/O
 *
 * sh_parser_feed() takes one byte and never touches a file descriptor, so the
 * framing can be tested on any machine with no car attached. The serial
 * helpers in <sensorhub/serial.h> are a convenience for callers that do have
 * hardware.
 */

#ifndef SENSORHUB_H
#define SENSORHUB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ *
 *  Framing constants
 * ------------------------------------------------------------------ */

#define SH_HEADER_1 0xAAu
#define SH_HEADER_2 0x55u

/* Bump this when the payload layout changes, and reflash everything. */
#define SH_FORMAT_V1 0x01u
#define SH_FORMAT_CURRENT SH_FORMAT_V1

/* How many slots the payload carries. Spares are deliberate: adding the
   fifth temperature probe should be a wiring job, not a format change that
   invalidates every CSV on disk. */
#define SH_TEMP_COUNT   4u
#define SH_ANALOG_COUNT 4u

/* Named indices for the slots that are actually wired today. The rest are
   spare; give them names here when they get used. */
#define SH_TEMP_ENGINE   0u
#define SH_TEMP_RADIATOR 1u
#define SH_ANALOG_BATTERY 0u

/* ------------------------------------------------------------------ *
 *  The packet
 * ------------------------------------------------------------------ */

/*
 * Digital channels are bits, not bytes: a switch carries one bit of
 * information and sixteen of them fit in the space two bytes used to take.
 *
 * Outputs are reported as well as inputs. The firmware drives a radiator fan
 * and a water pump, and until now their state appeared nowhere in telemetry,
 * so you could not display or log what the car was doing to itself.
 */
typedef struct __attribute__((packed)) {
    float speed;    /* mph, from the wheel interrupt   */
    float airspeed; /* mph, pitot, zeroed at startup   */

    float temps[SH_TEMP_COUNT];       /* degrees F, see SH_TEMP_*   */
    uint16_t analog[SH_ANALOG_COUNT]; /* raw ADC counts, see SH_ANALOG_* */

    uint8_t digital_in;  /* bitfield, input channels 0..7  */
    uint8_t digital_out; /* bitfield, output channels 0..7 */

    /* Increments every packet and wraps. The receiver turns gaps in this
       into a count of packets that never arrived, which a checksum cannot
       tell you: a dropped packet leaves no trace otherwise. */
    uint16_t sequence;
} sh_packet_t;

#define SH_PAYLOAD_SIZE 36u /* sizeof(sh_packet_t) for SH_FORMAT_V1 */
#define SH_FRAME_OVERHEAD 5u /* 2 header + fmt + len + checksum */
#define SH_FRAME_SIZE (SH_PAYLOAD_SIZE + SH_FRAME_OVERHEAD)

/* The largest payload the parser will buffer. Bigger than V1 on purpose, so
   a newer sender's packet can be received and rejected cleanly rather than
   overrunning anything. */
#define SH_MAX_PAYLOAD 64u

/* Both ends are little-endian, AVR and aarch64. A big-endian host would need
   byte swapping nobody has written, so say so rather than decode garbage. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#error "SensorHub assumes a little-endian host; the wire format is AVR-native"
#endif

/* ------------------------------------------------------------------ *
 *  Status
 * ------------------------------------------------------------------ */

/*
 * Every call that can fail returns one of these. SH_OK is zero, so the
 * common shape reads naturally:
 *
 *     if (sh_parser_feed(&parser, byte, &packet) == SH_OK) { ... }
 *
 * SH_INCOMPLETE is not an error. It is the usual answer, returned for every
 * byte that did not happen to complete a packet.
 */
typedef enum {
    SH_OK = 0,       /* a packet is ready                          */
    SH_INCOMPLETE,   /* byte consumed, nothing complete yet        */
    SH_E_NULL,       /* a required argument was NULL               */
    SH_E_CHECKSUM,   /* framed correctly, contents rejected        */
    SH_E_FORMAT,     /* fmt byte names a version we do not know    */
    SH_E_LENGTH,     /* len disagrees with fmt, or exceeds the max */
    SH_E_RANGE,      /* a channel index was out of bounds          */
    SH_E_SPACE,      /* caller's buffer was too small              */
    SH_E_OPEN,       /* could not open the port, see errno         */
    SH_E_IO,         /* read failed, see errno                     */
    SH_E_INTERRUPTED,/* a signal arrived; check your stop flag     */
    SH_E_CLOSED      /* clean EOF; the adapter was unplugged       */
} sh_status_t;

/* A short human-readable description. Never NULL, even for a bogus value. */
const char *sh_strstatus(sh_status_t status);

/* True for statuses that mean something went wrong, so SH_INCOMPLETE does
   not get logged as a failure forty times a second. */
bool sh_failed(sh_status_t status);


/* ------------------------------------------------------------------ *
 *  Digital channel accessors
 * ------------------------------------------------------------------ *
 *
 * Bit twiddling at every call site is how off-by-one channel bugs happen,
 * so do it once, here, with bounds checking.
 */

/* Read one input channel. Returns SH_E_RANGE for channel > 7. */
sh_status_t sh_digital_in(const sh_packet_t *packet, unsigned channel,
                          bool *out);

/* Read one output channel. Returns SH_E_RANGE for channel > 7. */
sh_status_t sh_digital_out(const sh_packet_t *packet, unsigned channel,
                           bool *out);

/* Set an input channel, for senders building a packet. */
sh_status_t sh_set_digital_in(sh_packet_t *packet, unsigned channel,
                              bool value);

/* Set an output channel, for senders building a packet. */
sh_status_t sh_set_digital_out(sh_packet_t *packet, unsigned channel,
                               bool value);

/* Bounds-checked reads for the array slots, so an out-of-range index is an
   error rather than whatever was next in memory. */
sh_status_t sh_temp(const sh_packet_t *packet, unsigned index, float *out);
sh_status_t sh_analog(const sh_packet_t *packet, unsigned index,
                      uint16_t *out);

/* ------------------------------------------------------------------ *
 *  Link statistics
 * ------------------------------------------------------------------ *
 *
 * A link that is losing packets shows up here long before it shows up on the
 * dashboard. The three counters fail differently and are worth reading
 * separately:
 *
 *   checksum_errors rising   electrical: noise, a marginal cable, a bad ground
 *   resyncs rising           the sender is being interrupted mid-frame
 *   dropped rising           packets never arrived at all; the receiver is
 *                            not keeping up, or the sender is restarting
 *
 * Width is configurable because this header also compiles for an ATmega328p,
 * where 64-bit counters cost RAM a Nano does not have and every increment is
 * a slow multi-word add. The AVR default is 32-bit rather than 16: at roughly
 * 40 packets a second, 16 bits wraps in under half an hour, and a diagnostic
 * counter that quietly lies is worse than a larger one.
 *
 * Note this changes sizeof(sh_stats_t), so everything linked together must
 * agree on it.
 */
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
    sh_counter_t packets;         /* accepted                              */
    sh_counter_t checksum_errors; /* framed correctly, contents rejected   */
    sh_counter_t resyncs;         /* fell back to hunting for a header     */
    sh_counter_t format_errors;   /* a version we do not understand        */
    sh_counter_t dropped;         /* inferred from gaps in the sequence    */
} sh_stats_t;

/* ------------------------------------------------------------------ *
 *  Parser
 * ------------------------------------------------------------------ */

typedef enum {
    SH_WAIT_HEADER_1 = 0,
    SH_WAIT_HEADER_2,
    SH_READ_FORMAT,
    SH_READ_LENGTH,
    SH_READ_PAYLOAD,
    SH_READ_CHECKSUM,
    SH_SKIP_UNKNOWN /* draining a packet whose format we do not know */
} sh_state_t;

typedef struct {
    sh_state_t state;
    uint8_t    format;
    uint8_t    length;
    uint8_t    payload[SH_MAX_PAYLOAD];
    size_t     payload_index;
    bool       have_sequence;
    uint16_t   last_sequence;
    sh_stats_t stats;
} sh_parser_t;

/* Reset to hunting for a header. Also zeroes the statistics. */
void sh_parser_init(sh_parser_t *parser);

/*
 * Feed exactly one received byte.
 *
 *   SH_OK          *out now holds a valid packet
 *   SH_INCOMPLETE  byte consumed, nothing complete yet. The usual answer.
 *   SH_E_CHECKSUM  a frame arrived intact but its contents were rejected
 *   SH_E_FORMAT    a frame announced a version we do not know; the parser
 *                  uses the length byte to skip it cleanly and carries on
 *   SH_E_NULL      parser or out was NULL
 *
 * Errors are reported, not thrown: the parser stays usable and keeps
 * counting. A caller that only wants packets can compare against SH_OK and
 * read parser->stats occasionally.
 */
sh_status_t sh_parser_feed(sh_parser_t *parser, uint8_t byte,
                           sh_packet_t *out);

/* ------------------------------------------------------------------ *
 *  Encoding
 * ------------------------------------------------------------------ */

/* XOR of every byte. Exposed so tests and senders need not duplicate it. */
uint8_t sh_checksum(const uint8_t *data, size_t length);

/*
 * Serialise a packet into a complete frame.
 *
 * buffer must have room for at least SH_FRAME_SIZE bytes; pass its real size
 * in buffer_size and the function will refuse rather than overrun. On success
 * *written holds the frame length. written may be NULL if you do not care.
 *
 *   SH_OK       frame written
 *   SH_E_SPACE  buffer_size was too small
 *   SH_E_NULL   packet or buffer was NULL
 */
sh_status_t sh_encode_frame(const sh_packet_t *packet, uint8_t *buffer,
                            size_t buffer_size, size_t *written);

#ifdef __cplusplus
}
#endif

#endif /* SENSORHUB_H */
