#!/usr/bin/env python3
"""Short, high-rate PMIC sampling burst (no sleep -- runs as fast as
vcgencmd allows, ~30ms/sample = ~33Hz) to try to catch a voltage/current
transient that the normal 0.2s steady logger is too coarse to see.

Not meant to run continuously (no sleep = constant subprocess spawning,
real CPU/IO cost) -- run it for a short deliberate test window, e.g.
while triggering a cold pipeline start to simulate the device-enumeration
burst that happens at boot.

Usage: burst_monitor.py <output.csv> <duration_seconds>
"""
import csv
import os
import re
import subprocess
import sys
import time

ADC_LINE_RE = re.compile(r'^\s*(\S+)\s+(current|volt)\((\d+)\)=([\d.]+)([AV])\s*$')


def read_pmic():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc'], capture_output=True, text=True, timeout=5).stdout
    rails = {}
    for line in out.splitlines():
        m = ADC_LINE_RE.match(line)
        if not m:
            continue
        name, kind, _idx, value, _unit = m.groups()
        base = name.rsplit('_', 1)[0]
        rails.setdefault(base, {})[kind] = float(value)
    return rails


def main():
    out_path = sys.argv[1]
    duration = float(sys.argv[2])

    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_mono', 'ext5v_v', 'vdd_core_v', 'vdd_core_a', 'wl_sw_v', 'wl_sw_a'])
        f.flush()
        os.fsync(f.fileno())

        t_start = time.monotonic()
        n = 0
        while time.monotonic() - t_start < duration:
            t = time.monotonic() - t_start
            try:
                rails = read_pmic()
            except Exception as exc:
                print(f'read error: {exc}', file=sys.stderr)
                continue
            ext5v = rails.get('EXT5V', {}).get('volt', float('nan'))
            vdd_core_v = rails.get('VDD_CORE', {}).get('volt', float('nan'))
            vdd_core_a = rails.get('VDD_CORE', {}).get('current', float('nan'))
            wl_sw_v = rails.get('3V7_WL_SW', {}).get('volt', float('nan'))
            wl_sw_a = rails.get('3V7_WL_SW', {}).get('current', float('nan'))
            writer.writerow([f'{t:.4f}', ext5v, vdd_core_v, vdd_core_a, wl_sw_v, wl_sw_a])
            f.flush()
            os.fsync(f.fileno())
            n += 1

        print(f'{n} samples in {duration}s -> {n/duration:.1f} Hz actual')


if __name__ == '__main__':
    main()
