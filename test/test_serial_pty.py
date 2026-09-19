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


def frame(speed, airspeed, engine_temp, rad_temp, channels, a0):
    """Build one 26-byte frame: 0xAA 0x55, 23-byte payload, XOR checksum."""
    payload = struct.pack(
        "<ffff5BH", speed, airspeed, engine_temp, rad_temp, *channels, a0
    )
    assert len(payload) == 23, "payload must be 23 bytes, got %d" % len(payload)

    checksum = 0
    for byte in payload:
        checksum ^= byte

    return bytes([0xAA, 0x55]) + payload + bytes([checksum])


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

    # A good frame.
    os.write(master, frame(23.5, 19.25, 180.0, 148.5, [1, 0, 1, 1, 0], 812))
    time.sleep(0.2)

    # Line noise, then a frame with a deliberately corrupted checksum. The
    # parser must reject the second and still recover for the third.
    os.write(master, b"\x00\xff\xde\xad\xbe\xef")
    corrupt = bytearray(frame(11.0, 2.0, 1.0, 2.0, [0, 0, 0, 0, 0], 5))
    corrupt[-1] ^= 0xFF
    os.write(master, bytes(corrupt))
    time.sleep(0.2)

    # A good frame after the corruption: this is the resync case.
    os.write(master, frame(31.25, 5.5, 190.0, 150.0, [0, 1, 0, 1, 1], 900))
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
        failures.append("did not recover after corruption")
    if "ok=2" not in out:
        failures.append("expected exactly 2 accepted packets")
    if "bad=1" not in out:
        failures.append("corrupt packet was not counted as a checksum error")

    if failures:
        for f in failures:
            print("FAIL:", f)
        sys.exit(1)

    print("PASS: framing, recovery, and shutdown all behaved")


if __name__ == "__main__":
    main()
