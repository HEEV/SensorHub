/*
 * POSIX serial transport.  Separate from the parser so that the framing can be
 * built and tested on any host, including ones with no termios.
 *
 * The careful parts here are about not resetting the Arduino.  A Nano reboots
 * whenever DTR is asserted, which costs a couple of seconds of telemetry and,
 * worse, re-runs the airspeed zeroing with the car possibly moving.  So: open
 * once, clear HUPCL, and never touch the modem control lines.
 */

#ifndef SENSORHUB_SERIAL_H
#define SENSORHUB_SERIAL_H

#include "sensorhub/sensorhub.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Where the CH340 lands on the Pi.  /dev/serial/by-id is stabler than
   /dev/ttyUSB0, which renumbers when another serial device is present. */
#define SH_DEFAULT_PORT "/dev/ttyUSB0"
#define SH_DEFAULT_PORT_BY_ID \
    "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0"

/*
 * Open and configure a port at 115200 8N1 raw.
 *
 * Returns a file descriptor, or -1 with errno set.  Close it with
 * sh_serial_close().  Do not reopen in a retry loop without a delay: each
 * open can reset the Nano.
 */
int sh_serial_open(const char *device);

/* Close a descriptor from sh_serial_open(). Safe to call with -1. */
void sh_serial_close(int fd);

/*
 * Block until the next valid packet arrives, feeding bytes through parser.
 *
 * Returns true with *out populated.  Returns false on EOF, on a read error,
 * or when a signal interrupted the read:
 *
 *   errno == 0      clean EOF; on a serial port, the adapter was unplugged
 *   errno == EINTR  a signal arrived; check your shutdown flag and call again
 *   otherwise       a real error
 *
 * EINTR is surfaced rather than retried internally so that a caller can
 * actually be interrupted. Install handlers with sigaction() and no
 * SA_RESTART if you want Ctrl-C to work; plain signal() sets SA_RESTART on
 * most platforms, which prevents read() from ever returning EINTR.
 */
bool sh_serial_read_packet(int fd, sh_parser_t *parser, sh_packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SENSORHUB_SERIAL_H */
