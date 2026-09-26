/*
 * Framing, checksum, and accessors.
 *
 * No I/O lives here on purpose: everything in this file can be exercised from
 * a byte array, which is what makes the tests meaningful with no car attached.
 */

#include "sensorhub/sensorhub.h"

#include <string.h>

_Static_assert(sizeof(float) == 4, "32-bit float required");
_Static_assert(sizeof(sh_packet_t) == SH_PAYLOAD_SIZE,
               "packet size changed; bump SH_FORMAT_CURRENT and the golden test");
_Static_assert(SH_PAYLOAD_SIZE <= SH_MAX_PAYLOAD,
               "the parser cannot buffer its own format");

/* ------------------------------------------------------------------ *
 *  Status
 * ------------------------------------------------------------------ */

const char *sh_strstatus(sh_status_t status)
{
    switch (status) {
    case SH_OK:            return "ok";
    case SH_INCOMPLETE:    return "incomplete, need more bytes";
    case SH_E_NULL:        return "null argument";
    case SH_E_CHECKSUM:    return "bad checksum";
    case SH_E_FORMAT:      return "unsupported packet format";
    case SH_E_LENGTH:      return "bad payload length";
    case SH_E_RANGE:       return "channel index out of range";
    case SH_E_SPACE:       return "buffer too small";
    case SH_E_OPEN:        return "could not open serial port";
    case SH_E_IO:          return "serial read failed";
    case SH_E_INTERRUPTED: return "interrupted by a signal";
    case SH_E_CLOSED:      return "serial connection closed";
    }
    return "unknown status";
}

bool sh_failed(sh_status_t status)
{
    return status != SH_OK && status != SH_INCOMPLETE;
}

/* ------------------------------------------------------------------ *
 *  Accessors
 * ------------------------------------------------------------------ */

#define SH_DIGITAL_MAX 7u

static sh_status_t bit_get(uint8_t field, unsigned channel, bool *out)
{
    if (out == NULL)              return SH_E_NULL;
    if (channel > SH_DIGITAL_MAX) return SH_E_RANGE;

    *out = ((field >> channel) & 1u) != 0u;
    return SH_OK;
}

static sh_status_t bit_set(uint8_t *field, unsigned channel, bool value)
{
    if (field == NULL)            return SH_E_NULL;
    if (channel > SH_DIGITAL_MAX) return SH_E_RANGE;

    if (value) *field |= (uint8_t)(1u << channel);
    else       *field &= (uint8_t)~(1u << channel);
    return SH_OK;
}

sh_status_t sh_digital_in(const sh_packet_t *p, unsigned channel, bool *out)
{
    return (p == NULL) ? SH_E_NULL : bit_get(p->digital_in, channel, out);
}

sh_status_t sh_digital_out(const sh_packet_t *p, unsigned channel, bool *out)
{
    return (p == NULL) ? SH_E_NULL : bit_get(p->digital_out, channel, out);
}

sh_status_t sh_set_digital_in(sh_packet_t *p, unsigned channel, bool value)
{
    return (p == NULL) ? SH_E_NULL : bit_set(&p->digital_in, channel, value);
}

sh_status_t sh_set_digital_out(sh_packet_t *p, unsigned channel, bool value)
{
    return (p == NULL) ? SH_E_NULL : bit_set(&p->digital_out, channel, value);
}

sh_status_t sh_analog(const sh_packet_t *packet, unsigned index, uint16_t *out)
{
    if (packet == NULL || out == NULL) return SH_E_NULL;
    if (index >= SH_ANALOG_COUNT)      return SH_E_RANGE;

    *out = packet->analog[index];
    return SH_OK;
}

/* ------------------------------------------------------------------ *
 *  Checksum and encoding
 * ------------------------------------------------------------------ */

uint16_t sh_crc16_continue(uint16_t crc, const uint8_t *data, size_t length)
{
    if (data == NULL) return crc;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);

        for (unsigned bit = 0; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

uint16_t sh_crc16(const uint8_t *data, size_t length)
{
    return sh_crc16_continue(0xFFFFu, data, length);
}

sh_status_t sh_encode_frame(const sh_packet_t *packet, uint8_t *buffer,
                            size_t buffer_size, size_t *written)
{
    if (packet == NULL || buffer == NULL) return SH_E_NULL;
    if (buffer_size < SH_FRAME_SIZE)      return SH_E_SPACE;

    buffer[0] = (uint8_t)SH_HEADER_1;
    buffer[1] = (uint8_t)SH_HEADER_2;
    buffer[2] = (uint8_t)SH_FORMAT_CURRENT;
    buffer[3] = (uint8_t)SH_PAYLOAD_SIZE;
    memcpy(buffer + 4, packet, SH_PAYLOAD_SIZE);

    /* The CRC covers fmt and len as well as the payload, so a corrupted
       length byte cannot quietly reframe the stream and still validate. */
    {
        uint16_t crc = sh_crc16(buffer + 2, SH_PAYLOAD_SIZE + 2u);
        buffer[4 + SH_PAYLOAD_SIZE] = (uint8_t)(crc & 0xFFu);
        buffer[5 + SH_PAYLOAD_SIZE] = (uint8_t)(crc >> 8);
    }

    if (written != NULL) *written = SH_FRAME_SIZE;
    return SH_OK;
}

/* ------------------------------------------------------------------ *
 *  Parser
 * ------------------------------------------------------------------ */

void sh_parser_init(sh_parser_t *parser)
{
    if (parser == NULL) return;

    memset(parser, 0, sizeof(*parser));
    parser->state = SH_WAIT_HEADER_1;
}

/* Count how many packets went missing between two sequence numbers, allowing
   for the counter wrapping at 16 bits. */
static void note_sequence(sh_parser_t *parser, uint16_t sequence)
{
    if (parser->have_sequence) {
        uint16_t expected = (uint16_t)(parser->last_sequence + 1u);
        uint16_t gap = (uint16_t)(sequence - expected);

        /* A huge gap almost certainly means the sender restarted rather than
           that 60000 packets vanished, so do not report a fictional flood. */
        if (gap > 0u && gap < 1000u) {
            parser->stats.dropped += gap;
        }
    }

    parser->have_sequence = true;
    parser->last_sequence = sequence;
}

sh_status_t sh_parser_feed(sh_parser_t *parser, uint8_t byte, sh_packet_t *out)
{
    if (parser == NULL || out == NULL) return SH_E_NULL;

    switch (parser->state) {
    case SH_WAIT_HEADER_1:
        if (byte == SH_HEADER_1) parser->state = SH_WAIT_HEADER_2;
        break;

    case SH_WAIT_HEADER_2:
        if (byte == SH_HEADER_2) {
            parser->state = SH_READ_FORMAT;
        }
        else if (byte == SH_HEADER_1) {
            /* 0xAA 0xAA 0x55 is a valid start: a payload byte that happens
               to be 0xAA can precede the real header, so hold this state
               rather than discarding the candidate. */
        }
        else {
            parser->stats.resyncs++;
            parser->state = SH_WAIT_HEADER_1;
        }
        break;

    case SH_READ_FORMAT:
        parser->format = byte;
        parser->state = SH_READ_LENGTH;
        break;

    case SH_READ_LENGTH:
        parser->length = byte;
        parser->payload_index = 0;

        if (parser->format != SH_FORMAT_CURRENT) {
            /* Unknown version. The length byte is what lets us skip it
               cleanly instead of desynchronising, which is the entire
               reason it is on the wire. */
            parser->stats.format_errors++;
            parser->state = (byte > 0u) ? SH_SKIP_UNKNOWN : SH_WAIT_HEADER_1;
            return SH_E_FORMAT;
        }

        if (byte != (uint8_t)SH_PAYLOAD_SIZE) {
            /* Our own version with the wrong length: corruption, not a new
               sender. Skip what it claims and carry on. */
            parser->state = (byte > 0u && byte <= SH_MAX_PAYLOAD)
                                ? SH_SKIP_UNKNOWN
                                : SH_WAIT_HEADER_1;
            return SH_E_LENGTH;
        }

        parser->state = SH_READ_PAYLOAD;
        break;

    case SH_SKIP_UNKNOWN:
        /* Drain the payload and its checksum without interpreting either. */
        parser->payload_index++;
        if (parser->payload_index > (size_t)parser->length) {
            parser->state = SH_WAIT_HEADER_1;
        }
        break;

    case SH_READ_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        if (parser->payload_index == (size_t)parser->length) {
            parser->state = SH_READ_CRC_LO;
        }
        break;

    case SH_READ_CRC_LO:
        parser->crc_lo = byte;
        parser->state = SH_READ_CRC_HI;
        break;

    case SH_READ_CRC_HI: {
        uint16_t received = (uint16_t)(((uint16_t)byte << 8) | parser->crc_lo);
        uint16_t expected;
        uint8_t header[2];

        parser->state = SH_WAIT_HEADER_1;

        /* Recompute over fmt, len, and the payload, matching the encoder.
           Feeding the two header fields through the same routine keeps one
           definition of the polynomial rather than two. */
        header[0] = parser->format;
        header[1] = parser->length;
        expected = sh_crc16_continue(sh_crc16(header, sizeof(header)),
                                     parser->payload,
                                     (size_t)parser->length);

        if (expected == received) {
            memcpy(out, parser->payload, SH_PAYLOAD_SIZE);
            parser->stats.packets++;
            note_sequence(parser, out->sequence);
            return SH_OK;
        }

        parser->stats.checksum_errors++;
        return SH_E_CHECKSUM;
    }
    }

    return SH_INCOMPLETE;
}
