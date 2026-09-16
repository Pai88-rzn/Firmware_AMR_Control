#!/usr/bin/env python3
"""
Real-time Terminal Monitor for STM32_Sensor_Test
Reads ASCII output from /dev/ttyACM0 and prints formatted diagnostic stream.
"""

import sys
import os
import time
import termios

import glob

if len(sys.argv) > 1:
    PORT = sys.argv[1]
else:
    acm_ports = sorted(glob.glob("/dev/ttyACM*"))
    PORT = acm_ports[0] if acm_ports else "/dev/ttyACM0"

print("=" * 60)
print(f" Connecting to STM32 Sensor Monitor on {PORT} ...")
print(" Press Ctrl+C to stop.")
print("=" * 60)

if not os.path.exists(PORT):
    print(f"Error: {PORT} not found. Please connect the STM32 USB cable.")
    sys.exit(1)

# Configure raw terminal mode on the serial device
try:
    # Use stty to set baud rate 115200 and raw mode
    os.system(f"stty -F {PORT} 115200 raw -echo -hupcl")
    fd = os.open(PORT, os.O_RDONLY)
except Exception as e:
    print(f"Error opening port {PORT}: {e}")
    print("Ensure your user has permission: sudo usermod -a -G dialout $USER")
    sys.exit(1)

try:
    while True:
        data = os.read(fd, 512)
        if data:
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()
except KeyboardInterrupt:
    print("\n[Stopped by user]")
finally:
    try:
        os.close(fd)
    except:
        pass
