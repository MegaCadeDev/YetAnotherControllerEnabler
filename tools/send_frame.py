#!/usr/bin/env python3
"""Send one held DualSense-style frame to a running YetAnotherControllerEnabler payload."""

import socket
import struct
import sys
import time

BUTTONS = {
    "cross": 0x4000,
    "circle": 0x2000,
    "square": 0x8000,
    "triangle": 0x1000,
    "up": 0x10,
    "down": 0x40,
    "left": 0x80,
    "right": 0x20,
    "options": 0x8,
    "create": 0x1,
    "l1": 0x400,
    "r1": 0x800,
}


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: send_frame.py HOST [button]", file=sys.stderr)
        sys.exit(2)
    host = sys.argv[1]
    button = BUTTONS.get(sys.argv[2] if len(sys.argv) > 2 else "cross", 0)
    frame = struct.pack("<IBBBBBB6x", button, 128, 128, 128, 128, 0, 0)
    with socket.create_connection((host, 9045), timeout=5) as sock:
        while True:
            sock.sendall(frame)
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
