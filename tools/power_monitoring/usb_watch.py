#!/usr/bin/env python3
"""Poll `lsusb` and log connect/disconnect events (not a full dump every
time -- just the diff), with a timestamp and fsync per event.

A peripheral (the thermal camera, the Logitech receiver, etc.) browning
out and re-enumerating is a useful early-warning sign distinct from the
PMIC's own rails, and USB disconnect/reconnect around the same moment as
a voltage droop is a strong clue about where current is actually being
pulled.
"""
import os
import subprocess
import time

LOG_PATH = '/home/vish/power_logs/usb_events.log'
INTERVAL_S = 1.0


def snapshot():
    out = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5).stdout
    return set(line.strip() for line in out.splitlines() if line.strip())


def main():
    with open(LOG_PATH, 'a') as f:
        f.write(f'\n--- usb_watch started {time.strftime("%Y-%m-%d %H:%M:%S")} ---\n')
        f.flush()
        os.fsync(f.fileno())

        prev = snapshot()
        for line in sorted(prev):
            f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}  PRESENT AT START  {line}\n')
        f.flush()
        os.fsync(f.fileno())

        while True:
            time.sleep(INTERVAL_S)
            try:
                cur = snapshot()
            except Exception as exc:
                f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}  lsusb error: {exc}\n')
                f.flush()
                os.fsync(f.fileno())
                continue

            gone = prev - cur
            new = cur - prev
            if gone or new:
                ts = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                for line in sorted(gone):
                    f.write(f'{ts}  DISCONNECTED  {line}\n')
                for line in sorted(new):
                    f.write(f'{ts}  CONNECTED     {line}\n')
                f.flush()
                os.fsync(f.fileno())
            prev = cur


if __name__ == '__main__':
    main()
