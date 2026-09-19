#!/usr/bin/env python3
"""
End-to-end check against a fake Arduino.

Opens a pty, speaks the exact wire format carsensordriver.ino speaks, and runs
sensorhub-monitor against the other end.  This is the test that covers the
parts a unit test cannot: termios setup, blocking reads, signal handling, and
the framing under a realistic mix of good frames, line noise, and corruption.

It also pins behaviour that is easy to regress: a corrupt packet must be
counted rather than silently dropped, and the tool must exit on a signal
instead of hanging in a blocking read.

    ./test/test_serial_pty.py build/sensorhub-monitor
"""

import os
import pty
import struct
import subprocess
import sys
import time


FORMAT_V1 = 1
PAYLOAD_SIZE = 36


def frame(speed, airspeed, temps, analog, digital_in, digital_out, sequence,
          fmt=FORMAT_V1):
    """Build one 41-byte frame.

    0xAA 0x55, format, length, 36-byte payload, XOR checksum over the
    format and length bytes as well as the payload.
    """
    payload = struct.pack(
        "<ff4f4HBBH", speed, airspeed, *temps, *analog,
        digital_in, digital_out, sequence
    )
    assert len(payload) == PAYLOAD_SIZE, (
        "payload must be %d bytes, got %d" % (PAYLOAD_SIZE, len(payload))
    )

    header = bytes([fmt, len(payload)])
    checksum = 0
    for byte in header + payload:
        checksum ^= byte

    return bytes([0xAA, 0x55]) + header + payload + bytes([checksum])


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: test_serial_pty.py <path to sensorhub-monitor>")

    monitor = sys.argv[1]
    master, slave = pty.openpty()
    device = os.ttyname(slave)

    proc = subprocess.Popen(
        [monitor, device], stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    time.sleep(0.5)

    good = dict(temps=(180.0, 148.5, 0.0, 0.0), analog=(812, 0, 0, 0),
                digital_in=0x0D, digital_out=0x02)

    # A good frame.
    os.write(master, frame(23.5, 19.25, sequence=1, **good))
    time.sleep(0.2)

    # Line noise, then a frame with a deliberately corrupted checksum.
    os.write(master, b"\x00\xff\xde\xad\xbe\xef")
    corrupt = bytearray(frame(11.0, 2.0, sequence=2, **good))
    corrupt[-1] ^= 0xFF
    os.write(master, bytes(corrupt))
    time.sleep(0.2)

    # A packet from a "newer" sender. The length byte must let the receiver
    # skip it and stay framed rather than desynchronising.
    future = bytes([0xAA, 0x55, 0x02, 50]) + bytes(range(50)) + bytes([0x00])
    os.write(master, future)
    time.sleep(0.2)

    # A good frame after all of that. Sequence jumps 1 -> 5, so 2, 3 and 4
    # never arrived and the receiver should say so. (The corrupt frame above
    # carried sequence 2, but it was rejected, so it does not count as
    # received.)
    os.write(master, frame(31.25, 5.5, sequence=5,
                           temps=(190.0, 150.0, 0.0, 0.0),
                           analog=(900, 0, 0, 0),
                           digital_in=0x16, digital_out=0x01))
    time.sleep(0.8)

    proc.terminate()

    try:
        out = proc.communicate(timeout=5)[0].decode(errors="replace")
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        sys.exit("FAIL: monitor did not exit on SIGTERM (stuck in a blocking read?)")

    print(out)

    failures = []
    if "speed=23.50" not in out:
        failures.append("first good packet was not decoded")
    if "speed=31.25" not in out:
        failures.append("did not recover after corruption and a skip")
    if "ok=2" not in out:
        failures.append("expected exactly 2 accepted packets")
    if "bad=1" not in out:
        failures.append("corrupt packet was not counted as a checksum error")
    if "fmt=1" not in out:
        failures.append("packet from a newer sender was not counted as an "
                        "unknown format")
    if "lost=3" not in out:
        failures.append("gap in the sequence numbers was not counted as "
                        "dropped packets")

    if failures:
        for f in failures:
            print("FAIL:", f)
        sys.exit(1)

    print("PASS: framing, recovery, and shutdown all behaved")


if __name__ == "__main__":
    main()
