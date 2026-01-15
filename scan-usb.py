#!/usr/bin/env python3

import glob
import subprocess
import os
import sys

SERIAL_PROBE_BYTES = 256
SERIAL_TIMEOUT = 1.0

def run(cmd):
    try:
        return subprocess.check_output(
            cmd, stderr=subprocess.DEVNULL, text=True
        )
    except subprocess.CalledProcessError:
        return ""

def udev_info(dev):
    out = run(["udevadm", "info", "-q", "all", "-n", dev])
    info = {}
    for line in out.splitlines():
        if line.startswith("E: "):
            k, _, v = line[3:].partition("=")
            info[k] = v
    return info

def probe_nmea(dev):
    try:
        import serial
        ser = serial.Serial(
            dev,
            baudrate=9600,
            timeout=SERIAL_TIMEOUT
        )
        data = ser.read(SERIAL_PROBE_BYTES).decode(errors="ignore")
        ser.close()
        for line in data.splitlines():
            if line.startswith("$GP") or line.startswith("$GN"):
                return True
        return False
    except Exception:
        return None

def scan():
    devices = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/pps*"):
        devices.extend(glob.glob(pattern))

    results = []
    for dev in sorted(set(devices)):
        info = udev_info(dev)
        entry = {
            "device": dev,
            "vendor": info.get("ID_VENDOR"),
            "model": info.get("ID_MODEL"),
            "vendor_id": info.get("ID_VENDOR_ID"),
            "model_id": info.get("ID_MODEL_ID"),
            "driver": info.get("ID_USB_DRIVER"),
            "nmea_detected": None,
        }

        if dev.startswith("/dev/tty"):
            entry["nmea_detected"] = probe_nmea(dev)

        results.append(entry)
    return results

def main():
    if os.geteuid() != 0:
        print("warning: run as root for full access", file=sys.stderr)

    for r in scan():
        print(r["device"])
        print(f"  vendor: {r['vendor']}")
        print(f"  model: {r['model']}")
        print(f"  vendor_id: {r['vendor_id']}")
        print(f"  model_id: {r['model_id']}")
        print(f"  driver: {r['driver']}")
        print(f"  nmea_detected: {r['nmea_detected']}")
        print()

if __name__ == "__main__":
    main()

