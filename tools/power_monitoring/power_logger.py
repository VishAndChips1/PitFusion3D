#!/usr/bin/env python3
"""Continuously log the Pi 5 PMIC's rail voltages/currents + throttle state.

Polls `vcgencmd pmic_read_adc` and `vcgencmd get_throttled` on an interval
and appends one CSV row per sample. Runs forever (for use under systemd
with Restart=always) so it keeps logging across crash-reboots -- the
whole point is to catch a cutout in the act, and a cutout is exactly the
moment this process itself might die.

Each row is fsync'd immediately: if the board loses power mid-write, the
OS page cache would otherwise lose whatever hasn't hit the SD card yet,
which is precisely the data we care most about (the samples right before
a crash).

CSV columns: timestamp, ext5v_v (the actual 5V input rail -- the one to
watch for drooping under load), throttled (raw hex from get_throttled;
bits 0-3 = currently throttled/undervolted, bits 16-19 = has happened
since boot -- see https://www.raspberrypi.com/documentation/computers/os.html#get_throttled),
total_w (sum of V*I across all current-bearing rails -- the board's own
internal power draw), est_total_i_a (total_w / ext5v_v -- an ESTIMATED
input current, not a direct measurement: this PMIC has no separate
input-current channel, only input *voltage* (EXT5V_V) plus each internal
rail's own current, so this divides the internal-power proxy by the
input voltage to get a current figure in the units asked for; it omits
regulator conversion losses, so true input current is somewhat higher,
and it still doesn't include downstream USB peripheral draw -- there is
no separate "USB peripheral current" rail on this PMIC at all, see the
chat explanation for what to use instead for that), temp_c (SoC temp),
cpu_freq_hz (ARM clock -- drops under thermal/power throttling even when
the throttled flag doesn't latch), loadavg_1m, then one <rail>_v /
<rail>_a column pair per PMIC rail.

Also appends one line to boot_events.log every time this process starts
(including every systemd-triggered restart after a crash), with the
current boot_id -- an explicit, unambiguous "the system rebooted here"
marker rather than inferring it from gaps in the CSV timestamps.
"""
import csv
import os
import re
import subprocess
import sys
import time

INTERVAL_S = 0.2  # vcgencmd pmic_read_adc takes ~30ms, so this is comfortably sustainable
LOG_PATH = '/home/vish/power_logs/power_log.csv'
BOOT_EVENTS_PATH = '/home/vish/power_logs/boot_events.log'


def get_boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            return f.read().strip()
    except OSError:
        return 'unknown'


def record_boot_event():
    try:
        with open('/proc/uptime') as f:
            uptime_s = float(f.read().split()[0])
    except OSError:
        uptime_s = float('nan')
    line = (f'{time.strftime("%Y-%m-%d %H:%M:%S")}  logger (re)started  '
            f'boot_id={get_boot_id()}  system_uptime={uptime_s:.1f}s\n')
    with open(BOOT_EVENTS_PATH, 'a') as f:
        f.write(line)
        f.flush()
        os.fsync(f.fileno())


def read_temp():
    out = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True, timeout=5).stdout
    m = re.search(r'([\d.]+)', out)
    return float(m.group(1)) if m else float('nan')


def read_cpu_freq():
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') as f:
            return int(f.read().strip()) * 1000  # file is in kHz
    except OSError:
        return -1


def read_loadavg():
    try:
        with open('/proc/loadavg') as f:
            return float(f.read().split()[0])
    except OSError:
        return float('nan')

ADC_LINE_RE = re.compile(r'^\s*(\S+)\s+(current|volt)\((\d+)\)=([\d.]+)([AV])\s*$')


def read_pmic():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc'], capture_output=True, text=True, timeout=5).stdout
    rails = {}
    for line in out.splitlines():
        m = ADC_LINE_RE.match(line)
        if not m:
            continue
        name, kind, _idx, value, _unit = m.groups()
        base = name.rsplit('_', 1)[0]  # e.g. "EXT5V_V" -> "EXT5V", "VDD_CORE_A" -> "VDD_CORE"
        rails.setdefault(base, {})[kind] = float(value)
    return rails


def read_throttled():
    out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=5).stdout
    m = re.search(r'0x([0-9a-fA-F]+)', out)
    return m.group(0) if m else 'unknown'


def main():
    record_boot_event()

    rails = read_pmic()
    rail_names = sorted(rails.keys())

    header = ['timestamp', 'ext5v_v', 'throttled', 'total_w', 'est_total_i_a',
              'temp_c', 'cpu_freq_hz', 'loadavg_1m']
    for r in rail_names:
        header += [f'{r}_v', f'{r}_a']

    write_header = True
    try:
        with open(LOG_PATH, 'r') as f:
            if f.readline():
                write_header = False
    except FileNotFoundError:
        pass

    with open(LOG_PATH, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(header)
            f.flush()
            import os
            os.fsync(f.fileno())

        while True:
            t0 = time.monotonic()
            try:
                rails = read_pmic()
                throttled = read_throttled()
                temp_c = read_temp()
                cpu_freq_hz = read_cpu_freq()
                loadavg_1m = read_loadavg()
            except Exception as exc:
                print(f'read error: {exc}', file=sys.stderr)
                time.sleep(INTERVAL_S)
                continue

            ext5v = rails.get('EXT5V', {}).get('volt', float('nan'))
            total_w = sum(
                v.get('volt', 0.0) * v.get('current', 0.0)
                for v in rails.values()
            )

            est_i = (total_w / ext5v) if ext5v else float('nan')
            row = [time.strftime('%Y-%m-%d %H:%M:%S.') + f'{time.time()%1:.3f}'[2:],
                   ext5v, throttled, round(total_w, 4), round(est_i, 4),
                   temp_c, cpu_freq_hz, loadavg_1m]
            for r in rail_names:
                v = rails.get(r, {})
                row += [v.get('volt', ''), v.get('current', '')]

            writer.writerow(row)
            f.flush()
            import os
            os.fsync(f.fileno())

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, INTERVAL_S - elapsed))


if __name__ == '__main__':
    main()
