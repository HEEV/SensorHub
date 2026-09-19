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
    char digital[18];
    unsigned i;

    /* inputs then outputs, most significant channel on the left so the
       string reads the way the bitfield is written */
    for (i = 0; i < 8; ++i) {
        bool v = false;
        (void)sh_digital_in(packet, 7 - i, &v);
        digital[i] = v ? '1' : '0';
    }
    digital[8] = ' ';
    for (i = 0; i < 8; ++i) {
        bool v = false;
        (void)sh_digital_out(packet, 7 - i, &v);
        digital[9 + i] = v ? '1' : '0';
    }
    digital[17] = '\0';

    printf("\r\x1b[2K"
           "#%u speed=%.2f air=%.2f"
           " | eng=%.1fF rad=%.1fF"
           " | A0=%u"
           " | in/out=%s"
           " | ok=%llu bad=%llu fmt=%llu resync=%llu lost=%llu",
           (unsigned)packet->sequence,
           (double)packet->speed,
           (double)packet->airspeed,
           (double)packet->temps[SH_TEMP_ENGINE],
           (double)packet->temps[SH_TEMP_RADIATOR],
           (unsigned)packet->analog[SH_ANALOG_BATTERY],
           digital,
           (unsigned long long)stats->packets,
           (unsigned long long)stats->checksum_errors,
           (unsigned long long)stats->format_errors,
           (unsigned long long)stats->resyncs,
           (unsigned long long)stats->dropped);

    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *device = (argc > 1) ? argv[1] : SH_DEFAULT_PORT;
    sh_parser_t parser;
    sh_packet_t packet;
    sh_status_t status;
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

        status = sh_serial_read_packet(fd, &parser, &packet);

        if (status == SH_E_INTERRUPTED) {
            continue; /* a signal; the while condition rechecks g_stop */
        }

        if (status != SH_OK) {
            fprintf(stderr, "\n%s", sh_strstatus(status));
            if (status == SH_E_IO) fprintf(stderr, ": %s", strerror(errno));
            fprintf(stderr, "\n");
            break;
        }

        print_packet(&packet, &parser.stats);
    }

    printf("\n%llu packets, %llu bad checksums, %llu unknown formats, "
           "%llu resyncs, %llu never arrived\n",
           (unsigned long long)parser.stats.packets,
           (unsigned long long)parser.stats.checksum_errors,
           (unsigned long long)parser.stats.format_errors,
           (unsigned long long)parser.stats.resyncs,
           (unsigned long long)parser.stats.dropped);

    sh_serial_close(fd);
    return 0;
}
