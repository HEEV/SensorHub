// sm_serial.c
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define HEADER_1 0xAAU
#define HEADER_2 0x55U
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B115200
#define BAUD_TEXT "115200"

typedef struct __attribute__((packed)) {
    float speed;
    float airspeed;
    float engineTemp;
    float radTemp;

    uint8_t channel0;
    uint8_t channel1;
    uint8_t channel2;
    uint8_t channel3;
    uint8_t channel4;

    uint16_t channelA0;
} DataPacket;

_Static_assert(sizeof(float) == 4, "32-bit float required");
_Static_assert(sizeof(DataPacket) == 23, "Unexpected packet size");

typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_READ_PAYLOAD,
    STATE_READ_CHECKSUM
} ReceiverState;

typedef struct {
    ReceiverState state;
    uint8_t payload[sizeof(DataPacket)];
    size_t payloadIndex;
} PacketReceiver;

static uint8_t packet_checksum(
    const uint8_t *data,
    size_t length)
{
    uint8_t checksum = 0;

    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }

    return checksum;
}

static bool configure_serial(int fd, speed_t baud_rate)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, baud_rate) != 0 ||
        cfsetospeed(&tty, baud_rate) != 0) {
        perror("setting baud rate");
        return false;
    }

    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_cflag &= (tcflag_t)~PARENB;
    tty.c_cflag &= (tcflag_t)~PARODD;
    tty.c_cflag &= (tcflag_t)~CSTOPB;
    tty.c_cflag &= (tcflag_t)~CRTSCTS;

    /*
     * Prevent modem-control lines from being dropped on close.
     * This helps avoid repeated Arduino Nano resets.
     */
    tty.c_cflag &= (tcflag_t)~HUPCL;

    tty.c_iflag &= (tcflag_t)~IXON;
    tty.c_iflag &= (tcflag_t)~IXOFF;
    tty.c_iflag &= (tcflag_t)~IXANY;

    /*
     * Blocking mode: each read waits for at least one byte.
     */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }

    /*
     * Do not call tcflush() and do not manipulate DTR or RTS.
     */
    return true;
}

static bool read_byte(int fd, uint8_t *value)
{
    ssize_t count;

    if (value == NULL) {
        errno = EINVAL;
        return false;
    }

    do {
        count = read(fd, value, 1);
    } while (count < 0 && errno == EINTR);

    return count == 1;
}

static void packet_receiver_init(PacketReceiver *receiver)
{
    if (receiver == NULL) {
        return;
    }

    receiver->state = STATE_WAIT_HEADER_1;
    receiver->payloadIndex = 0;
    memset(receiver->payload, 0, sizeof(receiver->payload));
}

static bool receive_packet(
    PacketReceiver *receiver,
    int fd,
    DataPacket *packet)
{
    uint8_t received_byte;

    if (receiver == NULL || packet == NULL) {
        errno = EINVAL;
        return false;
    }

    while (read_byte(fd, &received_byte)) {
        switch (receiver->state) {
            case STATE_WAIT_HEADER_1:
                if (received_byte == HEADER_1) {
                    receiver->state = STATE_WAIT_HEADER_2;
                }
                break;

            case STATE_WAIT_HEADER_2:
                if (received_byte == HEADER_2) {
                    receiver->payloadIndex = 0;
                    receiver->state = STATE_READ_PAYLOAD;
                } else if (received_byte != HEADER_1) {
                    receiver->state = STATE_WAIT_HEADER_1;
                }
                /*
                 * If another HEADER_1 arrives, remain in this state.
                 */
                break;

            case STATE_READ_PAYLOAD:
                receiver->payload[receiver->payloadIndex++] =
                    received_byte;

                if (receiver->payloadIndex == sizeof(DataPacket)) {
                    receiver->state = STATE_READ_CHECKSUM;
                }
                break;

            case STATE_READ_CHECKSUM:
                receiver->state = STATE_WAIT_HEADER_1;

                if (packet_checksum(
                        receiver->payload,
                        sizeof(receiver->payload)) == received_byte) {
                    memcpy(
                        packet,
                        receiver->payload,
                        sizeof(*packet));

                    return true;
                }

                fprintf(
                    stderr,
                    "Discarded packet: bad checksum\n");
                break;
        }
    }

    return false;
}

static void print_packet(const DataPacket *packet)
{
    if (packet == NULL) {
        return;
    }

    printf(
        "\r\x1b[2K"
        "speed=%.2f mph"
        " | airspeed=%.2f"
        " | engine=%.2f F"
        " | radiator=%.2f F"
        " | channels=%u%u%u%u%u"
        " | A0=%u",
        (double)packet->speed,
        (double)packet->airspeed,
        (double)packet->engineTemp,
        (double)packet->radTemp,
        (unsigned)packet->channel0,
        (unsigned)packet->channel1,
        (unsigned)packet->channel2,
        (unsigned)packet->channel3,
        (unsigned)packet->channel4,
        (unsigned)packet->channelA0);

    fflush(stdout);
}

int main(void)
{
    PacketReceiver receiver;
    DataPacket packet;
    int fd;

    /*
     * Open only once. Reopening the port can repeatedly reset an
     * Arduino Nano through its DTR auto-reset circuit.
     */
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("open");
        return 1;
    }

    if (!configure_serial(fd, BAUD_RATE)) {
        close(fd);
        return 1;
    }

    packet_receiver_init(&receiver);

    printf(
        "Listening on %s at %s baud\n",
        SERIAL_PORT,
        BAUD_TEXT);

    while (true) {
        memset(&packet, 0, sizeof(packet));

        if (!receive_packet(&receiver, fd, &packet)) {
            if (errno != 0) {
                perror("serial read");
            } else {
                fprintf(stderr, "Serial connection closed\n");
            }

            break;
        }

        print_packet(&packet);
    }

    printf("\n");
    close(fd);
    return 0;
}