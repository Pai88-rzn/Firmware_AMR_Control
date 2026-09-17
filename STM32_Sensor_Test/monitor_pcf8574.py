#!/usr/bin/env python3
"""
Interactive Terminal Monitor & Keyboard Controller for PCF8574 Diagnostics
Supports real-time bidirectional communication over USB CDC (/dev/ttyACM0).
No third-party packages required (pure Python standard library).
"""

import sys
import os
import glob
import select
import termios
import tty

def get_serial_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else "/dev/ttyACM0"

def main():
    port = get_serial_port()

    print("=" * 66)
    print(f" Connecting to PCF8574 Interactive Monitor on {port} ...")
    print("=" * 66)
    print(" KEYBOARD SHORTCUTS (Live Interactive Control):")
    print("   [0] : Mode Auto Demo (Relay Chaser + Input check cycle)")
    print("   [1] : Mode Input Monitor (0xFF - Live ground detection)")
    print("   [2] : Mode Relay Chaser (Active-LOW walking zero RY1..RY8)")
    print("   [3] : Mode LED Chaser (Active-HIGH walking one P0..P7)")
    print("   [4] : Mode All Toggle (Alternates 0x00 <--> 0xFF)")
    print("   [5..9, p..w] : Manual toggle individual pin P0..P7")
    print("   [a] : Force ALL LOW  (0x00 / All Relays ON)")
    print("   [f] : Force ALL HIGH (0xFF / All Relays OFF)")
    print("   [s] : Re-scan I2C Bus")
    print("   [h] : Show help menu")
    print("   [Ctrl+C] : Exit monitor")
    print("=" * 66 + "\n")

    if not os.path.exists(port):
        print(f"Error: {port} not found. Please connect STM32 USB cable.")
        sys.exit(1)

    # Set stty options on serial port
    os.system(f"stty -F {port} 115200 raw -echo -hupcl")

    try:
        ser_fd = os.open(port, os.O_RDWR | os.O_NONBLOCK)
    except Exception as e:
        print(f"Error opening port {port}: {e}")
        print("Tip: Run 'sudo chmod 666 /dev/ttyACM*' or add user to dialout group.")
        sys.exit(1)

    # Save original terminal settings for stdin
    stdin_fd = sys.stdin.fileno()
    old_stdin_attr = termios.tcgetattr(stdin_fd)

    try:
        # Put stdin in cbreak mode (keys sent immediately, Ctrl+C still generates SIGINT)
        tty.setcbreak(stdin_fd)

        while True:
            rlist, _, _ = select.select([ser_fd, stdin_fd], [], [], 0.05)

            # Incoming serial data from STM32
            if ser_fd in rlist:
                data = os.read(ser_fd, 1024)
                if data:
                    sys.stdout.write(data.decode("utf-8", errors="replace"))
                    sys.stdout.flush()

            # Key pressed by user on host keyboard
            if stdin_fd in rlist:
                key = sys.stdin.read(1)
                if key:
                    os.write(ser_fd, key.encode("utf-8"))

    except KeyboardInterrupt:
        print("\n\n[Exited by user (Ctrl+C)]")
    finally:
        # Restore terminal settings
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_stdin_attr)
        try:
            os.close(ser_fd)
        except:
            pass

if __name__ == "__main__":
    main()
