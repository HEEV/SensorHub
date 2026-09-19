/*
 * sensorhub-monitor: print live packets from the car.
 *
 * This is the bench tool.  Point it at the Arduino and you can see whether the
 * link is healthy and whether a given sensor is actually wired, without
 * involving the display or any of the rest of the stack.
 *
 *     sensorhub-monitor [device]
 */

#include "sensorhub/serial.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int signum)
{
    (void)signum;
    g_stop = 1;
}

static void install_handler(int signum)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    /* No SA_RESTART on purpose: we want read() to return EINTR so the loop
       below can notice g_stop. signal() would set SA_RESTART and make this
       process impossible to Ctrl-C out of a blocking read. */
    sa.sa_flags = 0;
    (void)sigaction(signum, &sa, NULL);
}

static void print_packet(const sh_packet_t *packet, const sh_stats_t *stats)
{
    printf("\r\x1b[2K"
           "speed=%.2f mph"
           " | air=%.2f"
           " | engine=%.1fF"
           " | rad=%.1fF"
           " | ch=%u%u%u%u%u"
           " | A0=%u"
           " | ok=%llu bad=%llu resync=%llu",
           (double)packet->speed,
           (double)packet->airspeed,
           (double)packet->engineTemp,
           (double)packet->radTemp,
           (unsigned)packet->channel0,
           (unsigned)packet->channel1,
           (unsigned)packet->channel2,
           (unsigned)packet->channel3,
           (unsigned)packet->channel4,
           (unsigned)packet->channelA0,
           (unsigned long long)stats->packets,
           (unsigned long long)stats->checksum_errors,
           (unsigned long long)stats->resyncs);

    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *device = (argc > 1) ? argv[1] : SH_DEFAULT_PORT;
    sh_parser_t parser;
    sh_packet_t packet;
    int fd;

    signal(SIGPIPE, SIG_IGN);
    install_handler(SIGINT);
    install_handler(SIGTERM);

    /* Open once.  Reopening resets the Nano through its DTR circuit. */
    fd = sh_serial_open(device);

    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", device, strerror(errno));
        return 1;
    }

    sh_parser_init(&parser);
    printf("Listening on %s at 115200 baud\n", device);

    while (!g_stop) {
        memset(&packet, 0, sizeof(packet));

        if (!sh_serial_read_packet(fd, &parser, &packet)) {
            if (errno == EINTR) {
                continue; /* a signal; the while condition rechecks g_stop */
            }
            if (errno != 0) {
                fprintf(stderr, "\nserial read: %s\n", strerror(errno));
            }
            else {
                fprintf(stderr, "\nserial connection closed\n");
            }
            break;
        }

        print_packet(&packet, &parser.stats);
    }

    printf("\n%llu packets, %llu checksum errors, %llu resyncs\n",
           (unsigned long long)parser.stats.packets,
           (unsigned long long)parser.stats.checksum_errors,
           (unsigned long long)parser.stats.resyncs);

    sh_serial_close(fd);
    return 0;
}
