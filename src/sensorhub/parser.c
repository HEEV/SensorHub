/*
 * Framing and checksum.  No I/O lives here on purpose: everything in this
 * file can be exercised from a byte array, which is what makes the tests
 * meaningful without a car plugged in.
 */

#include "sensorhub/sensorhub.h"

#include <string.h>

_Static_assert(sizeof(float) == 4, "32-bit float required");
_Static_assert(sizeof(sh_packet_t) == SH_PAYLOAD_SIZE, "unexpected packet size");

uint8_t sh_checksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;

    if (data == NULL) {
        return 0;
    }

    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }

    return checksum;
}

void sh_encode_frame(const sh_packet_t *packet, uint8_t *buffer)
{
    if (packet == NULL || buffer == NULL) {
        return;
    }

    buffer[0] = (uint8_t)SH_HEADER_1;
    buffer[1] = (uint8_t)SH_HEADER_2;
    memcpy(buffer + 2, packet, SH_PAYLOAD_SIZE);
    buffer[2 + SH_PAYLOAD_SIZE] = sh_checksum(buffer + 2, SH_PAYLOAD_SIZE);
}

void sh_parser_init(sh_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = SH_WAIT_HEADER_1;
}

bool sh_parser_feed(sh_parser_t *parser, uint8_t byte, sh_packet_t *out)
{
    if (parser == NULL || out == NULL) {
        return false;
    }

    switch (parser->state) {
    case SH_WAIT_HEADER_1:
        if (byte == SH_HEADER_1) {
            parser->state = SH_WAIT_HEADER_2;
        }
        break;

    case SH_WAIT_HEADER_2:
        if (byte == SH_HEADER_2) {
            parser->payload_index = 0;
            parser->state = SH_READ_PAYLOAD;
        }
        else if (byte == SH_HEADER_1) {
            /* 0xAA 0xAA 0x55 is a valid start: a payload byte that happens to
               be 0xAA can precede the real header, so hold this state rather
               than throwing the candidate away. */
        }
        else {
            parser->stats.resyncs++;
            parser->state = SH_WAIT_HEADER_1;
        }
        break;

    case SH_READ_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;

        if (parser->payload_index == SH_PAYLOAD_SIZE) {
            parser->state = SH_READ_CHECKSUM;
        }
        break;

    case SH_READ_CHECKSUM:
        parser->state = SH_WAIT_HEADER_1;

        if (sh_checksum(parser->payload, SH_PAYLOAD_SIZE) == byte) {
            memcpy(out, parser->payload, SH_PAYLOAD_SIZE);
            parser->stats.packets++;
            return true;
        }

        parser->stats.checksum_errors++;
        break;
    }

    return false;
}
