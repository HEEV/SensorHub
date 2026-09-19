#define _DEFAULT_SOURCE

/*
 * POSIX only. The Arduino build compiles every source under src/, including
 * this one, so it must reduce to nothing on an 8-bit target rather than drag
 * termios onto a Nano.
 */
#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)


#include "sensorhub/serial.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static bool configure_serial(int fd)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0) {
        return false;
    }

    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_cflag &= (tcflag_t)~PARENB;
    tty.c_cflag &= (tcflag_t)~PARODD;
    tty.c_cflag &= (tcflag_t)~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= (tcflag_t)~CRTSCTS;
#endif

    /* Do not drop the modem control lines on close: that is the Nano's
       auto-reset circuit, and tripping it costs a reboot mid-run. */
    tty.c_cflag &= (tcflag_t)~HUPCL;

    tty.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);

    /* Blocking: each read waits for at least one byte. */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }

    /* Deliberately no tcflush() and no DTR/RTS handling, for the same
       reason as HUPCL above. */
    return true;
}

int sh_serial_open(const char *device)
{
    int fd;

    if (device == NULL) {
        device = SH_DEFAULT_PORT;
    }

    fd = open(device, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        return -1;
    }

    if (!configure_serial(fd)) {
        int saved = errno;
        close(fd);
        errno = saved;
        return -1;
    }

    return fd;
}

void sh_serial_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

static bool read_byte(int fd, uint8_t *value)
{
    ssize_t count;

    count = read(fd, value, 1);

    if (count == 0) {
        errno = 0; /* clean EOF, distinguish from a real error */
    }

    /* EINTR is reported rather than retried. A caller with a shutdown flag
       needs a chance to look at it; swallowing the signal here is what makes
       a serial tool impossible to Ctrl-C. */
    return count == 1;
}

bool sh_serial_read_packet(int fd, sh_parser_t *parser, sh_packet_t *out)
{
    uint8_t byte;

    if (parser == NULL || out == NULL) {
        errno = EINVAL;
        return false;
    }

    while (read_byte(fd, &byte)) {
        if (sh_parser_feed(parser, byte, out)) {
            return true;
        }
    }

    return false;
}

#else  /* not POSIX */

/* Keep this a non-empty translation unit for compilers that dislike one. */
typedef int sh_serial_unsupported_on_this_target;

#endif
