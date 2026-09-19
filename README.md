# SensorHub

The wire format for the link between the car's Arduino Nano and the Raspberry
Pi, used by **both** ends so the two cannot drift apart.

The Arduino sends, the Pi receives, and there is exactly one definition of the
packet, the checksum, and the frame encoder.

## The wire format

Forty-one bytes per frame, 115200 baud, 8N1:

```
0xAA 0x55  fmt  len  payload[len]  checksum
```

| field | size | purpose |
|---|---|---|
| `0xAA 0x55` | 2 | header, for resynchronising |
| `fmt` | 1 | format version, currently `0x01` |
| `len` | 1 | payload length, currently 36 |
| payload | `len` | `sh_packet_t` |
| `checksum` | 1 | XOR of `fmt`, `len`, and every payload byte |

**The version and length bytes earn their keep.** Without a version, changing
what a field *means* while keeping the packet the same size still passes the
checksum, and the receiver decodes confidently wrong data forever. That is not
hypothetical: the old Python server's channel names silently changed in April
and nobody noticed for months. Without a length, an old receiver facing a
newer sender desynchronises instead of skipping one packet and carrying on.

Two bytes on a link running at 7% utilisation is a good trade.

The checksum deliberately covers `fmt` and `len` as well as the payload, so a
corrupted length byte cannot quietly reframe the stream and still validate.

### Payload, format 1

`sh_packet_t`, a packed struct of fixed-width types, 36 bytes on the Pi and on
an ATmega328p alike, asserted at compile time on both.

| offset | type | field | notes |
|---|---|---|---|
| 0 | `float` | `speed` | mph, from the wheel interrupt |
| 4 | `float` | `airspeed` | mph, pitot, zeroed at Arduino startup |
| 8 | `float[4]` | `temps` | °F. `[0]` engine, `[1]` radiator, 2 spare |
| 24 | `uint16_t[4]` | `analog` | raw ADC counts. `[0]` battery, 3 spare |
| 32 | `uint8_t` | `digital_in` | bitfield, input channels 0–7 |
| 33 | `uint8_t` | `digital_out` | bitfield, output channels 0–7 |
| 34 | `uint16_t` | `sequence` | wraps; gaps mean dropped packets |

Three things worth noting about that layout:

- **Spare slots are deliberate.** Adding a fourth temperature probe to the
  DS18B20 bus should be a wiring job, not a format change that invalidates
  every CSV on disk.
- **Digital channels are bits.** A switch carries one bit, and sixteen of them
  fit where two bytes used to go.
- **Outputs are reported, not just inputs.** The firmware drives a radiator
  fan and a water pump, and until now their state appeared nowhere in
  telemetry, so there was no way to see or log what the car was doing to
  itself.

Both ends are little-endian. A big-endian host is rejected with an `#error`
rather than quietly decoding nonsense.

## Sending, on the Arduino

```sh
arduino-cli lib install --git-url https://github.com/HEEV/SensorHub
```

```c
#include <SensorHub.h>

static uint16_t sequence = 0;

void sendPacket(sh_packet_t &packet) {
    uint8_t frame[SH_FRAME_SIZE];
    size_t written = 0;

    packet.sequence = sequence++;

    if (sh_encode_frame(&packet, frame, sizeof(frame), &written) != SH_OK) {
        return;  /* refuses rather than writing a half-built frame */
    }
    Serial.write(frame, written);
}
```

Build the packet with the accessors rather than poking bits by hand, which is
how off-by-one channel bugs happen:

```c
packet.temps[SH_TEMP_ENGINE] = engineTempF;
packet.analog[SH_ANALOG_BATTERY] = analogRead(A7);
sh_set_digital_in(&packet, 0, digitalRead(4));
sh_set_digital_out(&packet, 0, digitalRead(RAD_FAN_PIN));
```

Remember to increment `sequence` every packet. It is the only way the receiver
can tell that a packet never arrived; a checksum cannot.

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

while (sh_serial_read_packet(fd, &parser, &packet) == SH_OK) {
    printf("#%u %.2f mph\n", packet.sequence, (double) packet.speed);
}

sh_serial_close(fd);
```

`sh_serial_read_packet` keeps reading through recoverable framing errors. A
bad checksum or an unknown format is counted in `parser.stats` and the read
continues, because one corrupt packet is not a reason to make every caller
write a retry loop. Only a completed packet or an I/O condition ends the call.

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
sh_status_t sh_parser_feed(sh_parser_t *parser, uint8_t byte, sh_packet_t *out);
```

| returns | meaning |
|---|---|
| `SH_OK` | `*out` holds a packet |
| `SH_INCOMPLETE` | byte consumed, nothing complete yet. The usual answer |
| `SH_E_CHECKSUM` | a frame arrived intact but its contents were rejected |
| `SH_E_FORMAT` | a version we do not know; skipped cleanly using `len` |
| `SH_E_LENGTH` | our version, an impossible length |

Errors are reported, not thrown. The parser stays usable and keeps counting,
so a caller that only wants packets can compare against `SH_OK` and read
`parser.stats` occasionally.

This is what makes the library testable with no car attached, and it is how
CarDisplay drains a socket without blocking its UI thread:

```c
uint8_t buf[256];
ssize_t n;

while ((n = read(fd, buf, sizeof(buf))) > 0) {       /* O_NONBLOCK */
    for (ssize_t i = 0; i < n; ++i) {
        if (sh_parser_feed(&parser, buf[i], &packet) == SH_OK) {
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

| counter | meaning | what it usually indicates |
|---|---|---|
| `packets` | accepted | — |
| `checksum_errors` | framed correctly, contents rejected | electrical: noise, a marginal cable, a bad ground |
| `resyncs` | fell back to hunting for a header | the sender is being interrupted mid-frame |
| `format_errors` | a version we do not understand | firmware and receiver are out of step |
| `dropped` | inferred from gaps in `sequence` | packets never arrived; the receiver is not keeping up, or the sender restarted |

`dropped` is the one a checksum cannot give you. A packet that never arrives
leaves no trace at all, so without a sequence number a link losing half its
traffic looks identical to a healthy one.

The counter deliberately does not report a flood when the sequence wraps at
65535, nor when the Arduino restarts and begins again at zero.

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

From `<sensorhub/sensorhub.h>`, portable everywhere including AVR:

```c
/* framing */
void        sh_parser_init  (sh_parser_t *parser);
sh_status_t sh_parser_feed  (sh_parser_t *parser, uint8_t byte,
                             sh_packet_t *out);
uint8_t     sh_checksum     (const uint8_t *data, size_t length);
sh_status_t sh_encode_frame (const sh_packet_t *packet, uint8_t *buffer,
                             size_t buffer_size, size_t *written);

/* channels, all bounds-checked */
sh_status_t sh_digital_in      (const sh_packet_t *p, unsigned ch, bool *out);
sh_status_t sh_digital_out     (const sh_packet_t *p, unsigned ch, bool *out);
sh_status_t sh_set_digital_in  (sh_packet_t *p, unsigned ch, bool value);
sh_status_t sh_set_digital_out (sh_packet_t *p, unsigned ch, bool value);
sh_status_t sh_temp            (const sh_packet_t *p, unsigned i, float *out);
sh_status_t sh_analog          (const sh_packet_t *p, unsigned i, uint16_t *out);

/* status */
const char *sh_strstatus (sh_status_t status);   /* never NULL */
bool        sh_failed    (sh_status_t status);   /* SH_INCOMPLETE is not a failure */
```

From `<sensorhub/serial.h>`, POSIX only:

```c
int         sh_serial_open       (const char *device);
void        sh_serial_close      (int fd);
sh_status_t sh_serial_read_packet(int fd, sh_parser_t *parser,
                                  sh_packet_t *out);
```

Conventions worth knowing:

- **`SH_OK` is zero**, so `if (call(...) == SH_OK)` reads naturally and a
  status can be tested for truth if you prefer.
- **`SH_INCOMPLETE` is not an error.** It is the answer for every byte that
  did not happen to finish a packet, which is most of them. Use `sh_failed()`
  rather than `!= SH_OK` when deciding whether to log something.
- **Every function tolerates `NULL`** by returning `SH_E_NULL` rather than
  crashing, and every index is bounds-checked into `SH_E_RANGE`.
- **`sh_encode_frame` takes the buffer size** and refuses rather than
  overrunning.
- **Every status has a message.** `sh_strstatus()` never returns `NULL`, even
  for a value that is not a real status.

## Changing the wire format

Don't, casually. The format is frozen by a golden-frame test that compares
`sh_encode_frame()` output byte for byte against a known-good frame, verified
identical to the firmware's original hand-rolled encoder across 200,000 random
packets.

If that test fails, it is not a test to update. It means every Arduino in the
fleet needs reflashing and every consumer needs checking, including the CSVs
already on disk. Change it deliberately, in one commit, on both ends.
