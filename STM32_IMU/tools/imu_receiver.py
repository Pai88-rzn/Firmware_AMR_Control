#!/usr/bin/env python3
"""
IMU Telemetry Receiver & Validator (AMR GPRP Platform)
Reads 36-byte binary telemetry packets streamed from STM32_IMU via FT231XS USB COM Port.
"""

import sys
import time
import struct
import argparse

try:
    import serial
except ImportError:
    print("[ERROR] 'pyserial' is required. Install via: pip install pyserial")
    sys.exit(1)

SYNC_HEADER = bytes([0x55, 0xAA])
PACKET_SIZE = 36
# Struct format: 2B sync, 1I timestamp_ms, 4h quat(w,x,y,z), 3h accel(x,y,z), 3h gyro(x,y,z), 3h euler(yaw,roll,pitch), 1B calib, 1b temp, 1H crc16
PACKET_FORMAT = "<2s I 4h 3h 3h 3h B b H"


def compute_crc16_ccitt(data: bytes) -> int:
    """Computes CRC-16-CCITT (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_packet(raw: bytes):
    if len(raw) != PACKET_SIZE:
        return None

    unpacked = struct.unpack(PACKET_FORMAT, raw)
    sync, ts_ms, qw_raw, qx_raw, qy_raw, qz_raw, ax_raw, ay_raw, az_raw, \
        gx_raw, gy_raw, gz_raw, yaw_raw, roll_raw, pitch_raw, calib_byte, temp_c, rx_crc = unpacked

    if sync != SYNC_HEADER:
        return None

    # Verify CRC
    calc_crc = compute_crc16_ccitt(raw[:PACKET_SIZE - 2])
    crc_valid = (calc_crc == rx_crc)

    # Conversion factors
    # Quaternion: 1 LSB = 2^-14
    SCALE_QUAT = 1.0 / (1 << 14)
    qw = qw_raw * SCALE_QUAT
    qx = qx_raw * SCALE_QUAT
    qy = qy_raw * SCALE_QUAT
    qz = qz_raw * SCALE_QUAT

    # Linear Accel: 100 LSB = 1 m/s^2
    ax = ax_raw / 100.0
    ay = ay_raw / 100.0
    az = az_raw / 100.0

    # Gyro: 16 LSB = 1 dps
    gx = gx_raw / 16.0
    gy = gy_raw / 16.0
    gz = gz_raw / 16.0

    # Euler: 16 LSB = 1 degree
    yaw = yaw_raw / 16.0
    roll = roll_raw / 16.0
    pitch = pitch_raw / 16.0

    # Calibration Status
    sys_cal = (calib_byte >> 6) & 0x03
    gyr_cal = (calib_byte >> 4) & 0x03
    acc_cal = (calib_byte >> 2) & 0x03
    mag_cal = calib_byte & 0x03

    return {
        "timestamp_ms": ts_ms,
        "quat": (qw, qx, qy, qz),
        "accel": (ax, ay, az),
        "gyro": (gx, gy, gz),
        "euler": (yaw, roll, pitch),
        "calib": {"sys": sys_cal, "gyro": gyr_cal, "accel": acc_cal, "mag": mag_cal},
        "temperature": temp_c,
        "crc_ok": crc_valid
    }


def main():
    parser = argparse.ArgumentParser(description="AMR STM32_IMU Telemetry Receiver")
    parser.add_argument("-p", "--port", type=str, default="COM3", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"[*] Opening serial port {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"[!] Failed to open port {args.port}: {e}")
        return

    print("[*] Listening for IMU packets @ 100 Hz. Press Ctrl+C to stop.\n")
    buffer = bytearray()
    pkt_count = 0
    err_count = 0
    last_print = time.time()

    try:
        while True:
            byte = ser.read(1)
            if not byte:
                continue

            buffer.append(byte[0])

            # Synchronize buffer on 0x55 0xAA header
            while len(buffer) >= 2 and (buffer[0] != 0x55 or buffer[1] != 0xAA):
                buffer.pop(0)

            if len(buffer) >= PACKET_SIZE:
                raw_frame = bytes(buffer[:PACKET_SIZE])
                buffer = buffer[PACKET_SIZE:]

                data = parse_packet(raw_frame)
                if data and data["crc_ok"]:
                    pkt_count += 1
                    now = time.time()
                    if now - last_print >= 0.1:  # 10 Hz console update
                        last_print = now
                        e = data["euler"]
                        q = data["quat"]
                        a = data["accel"]
                        g = data["gyro"]
                        c = data["calib"]
                        print(f"\r[T: {data['timestamp_ms']:8d} ms] "
                              f"RPY: [{e[1]:6.1f}°, {e[2]:6.1f}°, {e[0]:6.1f}°] | "
                              f"Quat: ({q[0]:5.2f}, {q[1]:5.2f}, {q[2]:5.2f}, {q[3]:5.2f}) | "
                              f"Acc: [{a[0]:5.1f}, {a[1]:5.1f}, {a[2]:5.1f}] m/s² | "
                              f"Cal: S:{c['sys']} G:{c['gyro']} A:{c['accel']} M:{c['mag']} | "
                              f"Temp: {data['temperature']}°C", end="", flush=True)
                else:
                    err_count += 1

    except KeyboardInterrupt:
        print(f"\n[*] Stopped. Total valid packets received: {pkt_count}, Checksum errors: {err_count}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
