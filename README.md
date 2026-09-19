# SensorHub

The wire format for the link between the car's Arduino Nano and the Raspberry
Pi, used by **both** ends so the two cannot drift apart.

The Arduino sends, the Pi receives, and there is exactly one definition of the
packet, the checksum, and the frame encoder.

## The wire format

Twenty-six bytes per frame, 115200 baud, 8N1:

```
0xAA  0x55  <23-byte packed payload>  <XOR checksum of the payload>
```

The payload is `sh_packet_t`, a packed struct of fixed-width types. It is 23
bytes on the Pi and on an ATmega328p alike, asserted at compile time on both.

| offset | type | field | notes |
|---|---|---|---|
| 0 | `float` | `speed` | mph, from the wheel interrupt |
| 4 | `float` | `airspeed` | mph, pitot, zeroed at Arduino startup |
| 8 | `float` | `engineTemp` | °F, DS18B20 |
| 12 | `float` | `radTemp` | °F, DS18B20 |
| 16 | `uint8_t` | `channel0` | digital, pin 4 |
| 17 | `uint8_t` | `channel1` | digital, pin 5 |
| 18 | `uint8_t` | `channel2` | digital, pin 6 |
| 19 | `uint8_t` | `channel3` | digital, pin 7 |
| 20 | `uint8_t` | `channel4` | digital, pin 8 |
| 21 | `uint16_t` | `channelA0` | analog, pin A7 |

The first four fields are fixed. The last six are deliberately anonymous:
what they *mean* is a wiring decision, and naming them is the consumer's job.

Both ends are little-endian. A big-endian host is rejected with an `#error`
rather than quietly decoding nonsense.

## Sending, on the Arduino

```sh
arduino-cli lib install --git-url https://github.com/HEEV/SensorHub
```

```c
#include <SensorHub.h>

void sendPacket(const sh_packet_t &packet) {
    uint8_t frame[SH_FRAME_SIZE];
    sh_encode_frame(&packet, frame);
    Serial.write(frame, sizeof(frame));
}
```

`sh_encode_frame()` writes header, payload, and checksum into a
`SH_FRAME_SIZE` buffer. One buffered `Serial.write` beats four small ones.

Costs about **22 bytes of flash** and no RAM over hand-rolling it.

Note the include is `<SensorHub.h>`, not the nested path. Arduino resolves
libraries by top-level header name, so that shim is what makes the library
discoverable from a sketch. Everywhere else, use `<sensorhub/sensorhub.h>`.

## Receiving, on the Pi

```c
#include <sensorhub/sensorhub.h>
#include <sensorhub/serial.h>

sh_parser_t parser;
sh_packet_t packet;

int fd = sh_serial_open("/dev/ttyUSB0");   /* NULL for the default port */
if (fd < 0) { perror("open"); return 1; }

sh_parser_init(&parser);

while (sh_serial_read_packet(fd, &parser, &packet)) {
    printf("%.2f mph\n", (double) packet.speed);
}

sh_serial_close(fd);
```

`sh_serial_open()` sets 115200 8N1 raw and, importantly, **clears `HUPCL` and
never touches DTR or RTS**. A Nano reboots when DTR is asserted, which costs
a couple of seconds of telemetry and re-runs the airspeed zeroing with the car
possibly moving. Open the port once and keep it open.

Prefer the stable device path over `/dev/ttyUSB0`, which renumbers when a
second serial device is present:

```c
sh_serial_open(SH_DEFAULT_PORT_BY_ID);
```

## Parsing without a serial port

The parser never touches a file descriptor. Feed it bytes from wherever you
got them: a socket, a file, a test fixture, a non-blocking read of your own.

```c
bool sh_parser_feed(sh_parser_t *parser, uint8_t byte, sh_packet_t *out);
```

Returns `true` exactly when that byte completed a packet whose checksum
matched, with the result in `*out`. Every other byte returns `false`.

This is what makes the library testable with no car attached, and it is how
CarDisplay drains a socket without blocking its UI thread:

```c
uint8_t buf[256];
ssize_t n;

while ((n = read(fd, buf, sizeof(buf))) > 0) {       /* O_NONBLOCK */
    for (ssize_t i = 0; i < n; ++i) {
        if (sh_parser_feed(&parser, buf[i], &packet)) {
            apply(&packet);   /* keep the newest; a backlog means you fell behind */
        }
    }
}
```

## Link health

The parser keeps counters. A link that is losing packets shows up here long
before it shows up on the dashboard.

```c
printf("ok=%llu bad=%llu resync=%llu\n",
       (unsigned long long) parser.stats.packets,
       (unsigned long long) parser.stats.checksum_errors,
       (unsigned long long) parser.stats.resyncs);
```

| counter | meaning |
|---|---|
| `packets` | accepted |
| `checksum_errors` | framed correctly, contents rejected |
| `resyncs` | fell back to hunting for a header |

A rising `checksum_errors` is electrical: noise, a marginal cable, a bad
ground. A rising `resyncs` with few checksum errors usually means the sender
is being interrupted mid-frame.

Counter width is `uint64_t` on a host and `uint32_t` on AVR. Not 16-bit: at
roughly 40 packets a second that wraps in under half an hour, and a diagnostic
counter that quietly lies is worse than a larger one. Override with
`-DSH_COUNTER_BITS=16|32|64` if you must, but note it changes
`sizeof(sh_stats_t)`, so everything linked together has to agree.

## Recovering from a bad byte

This is the failure the old 9600-baud reader actually had: one dropped byte
desynchronised the stream and there was nothing to resynchronise against.

The `0xAA 0x55` header fixes that, and the parser handles the subtle case too.
A payload byte that happens to be `0xAA` sitting immediately before a real
header does not break framing, because the state machine holds its candidate
rather than discarding it. A truncated frame is dropped and the next one
parses.

You do not have to do anything to get this. It is just worth knowing it is
there, and there are tests for each case.

## Building

CMake, as a subproject:

```cmake
add_subdirectory(SensorHub)
target_link_libraries(your_app PRIVATE sensorhub)
```

Standalone, with the bench tool and tests:

```sh
cmake -S . -B build && cmake --build build -j
ctest --test-dir build
```

Options: `SH_BUILD_MONITOR` and `SH_BUILD_TESTS`, both on when this is the
top-level project and off when it is a subproject.

Layout note: sources live under `src/sensorhub/` rather than the usual
`include/` and `src/` split, so that this directory is simultaneously a valid
Arduino library. `arduino-cli` puts `<library>/src` on the include path, which
makes `<sensorhub/sensorhub.h>` resolve identically on both targets.
`src/sensorhub/serial.c` is guarded so it compiles to zero bytes on AVR.

## The bench tool

```sh
./build/sensorhub-monitor [device]
```

Prints live packets and running counters. This is how you find out whether a
sensor is actually wired without involving the display or anything else:

```
speed=23.50 mph | air=19.25 | engine=180.0F | rad=148.5F | ch=10110 | A0=812 | ok=2 bad=1 resync=0
```

Ctrl-C works, which is less obvious than it sounds. `signal()` installs
handlers with `SA_RESTART` on most platforms, so a blocking `read()`
auto-restarts and never returns, making the tool impossible to interrupt. The
library surfaces `EINTR` rather than retrying it internally so the caller can
check its own shutdown flag:

```c
if (!sh_serial_read_packet(fd, &parser, &packet)) {
    if (errno == EINTR) continue;    /* a signal: check your stop flag */
    if (errno == 0)    break;        /* clean EOF: the adapter was unplugged */
    perror("serial read");
    break;
}
```

Install your handlers with `sigaction()` and no `SA_RESTART` if you want this
to work.

## API summary

From `<sensorhub/sensorhub.h>`, portable everywhere:

```c
void    sh_parser_init  (sh_parser_t *parser);
bool    sh_parser_feed  (sh_parser_t *parser, uint8_t byte, sh_packet_t *out);
uint8_t sh_checksum     (const uint8_t *data, size_t length);
void    sh_encode_frame (const sh_packet_t *packet, uint8_t *buffer);
```

From `<sensorhub/serial.h>`, POSIX only:

```c
int  sh_serial_open       (const char *device);
void sh_serial_close      (int fd);
bool sh_serial_read_packet(int fd, sh_parser_t *parser, sh_packet_t *out);
```

Every function tolerates `NULL` arguments by doing nothing and returning a
falsy value rather than crashing.

## Changing the wire format

Don't, casually. The format is frozen by a golden-frame test that compares
`sh_encode_frame()` output byte for byte against a known-good frame, verified
identical to the firmware's original hand-rolled encoder across 200,000 random
packets.

If that test fails, it is not a test to update. It means every Arduino in the
fleet needs reflashing and every consumer needs checking, including the CSVs
already on disk. Change it deliberately, in one commit, on both ends.
