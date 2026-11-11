#!/usr/bin/env python3
"""Continuously log per-core CPU%, per-core clock, and SoC temp.

Mirrors power_logger.py's shape (same CSV-append/fsync/systemd-restart-
friendly pattern) so the two logs can be joined on timestamp later -- see
join_power_cpu_output.py. power_logger.py already has temp_c, cpu_freq_hz
(core 0 only) and an aggregate loadavg_1m, but nothing broken down by
core: this fills that one gap (the thesis report's "CPU usage: core and
percentage" ask).

Per-core % is computed from /proc/stat deltas between samples (the only
way to get a true percentage rather than an instantaneous snapshot) --
not psutil.cpu_percent(percpu=True), which would need its own internal
sleep/interval and double the subprocess-free sampling this is meant to
be cheap at.

Usage: cpu_logger.py [interval_seconds] [output_csv]
"""
import csv
import os
import sys
import time

LOG_PATH = '/home/vish/power_logs/cpu_log.csv'
INTERVAL_S = 0.5


def read_proc_stat():
    """Returns {core_label: (user+nice+system+irq+softirq+steal, total)}
    cumulative-since-boot jiffies, core_label like 'cpu0'; 'cpu' itself
    (the aggregate line) is skipped -- aggregate is derivable by summing
    the per-core rows, and keeping only per-core avoids double-plotting
    the same information under two different logical columns."""
    out = {}
    with open('/proc/stat') as f:
        for line in f:
            if not line.startswith('cpu') or line.startswith('cpu '):
                continue
            parts = line.split()
            label = parts[0]
            fields = [int(x) for x in parts[1:11]]
            # user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice
            idle = fields[3] + fields[4]
            busy = fields[0] + fields[1] + fields[2] + fields[5] + fields[6] + fields[7]
            out[label] = (busy, busy + idle)
    return out


def read_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read().strip()) / 1000.0
    except OSError:
        return float('nan')


def read_core_freqs(n_cores):
    freqs = []
    for i in range(n_cores):
        try:
            with open(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq') as f:
                freqs.append(int(f.read().strip()) * 1000)  # kHz -> Hz
        except OSError:
            freqs.append(-1)
    return freqs


def read_loadavg():
    try:
        with open('/proc/loadavg') as f:
            parts = f.read().split()
            return parts[0], parts[1], parts[2]
    except OSError:
        return ('nan', 'nan', 'nan')


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else INTERVAL_S
    out_path = sys.argv[2] if len(sys.argv) > 2 else LOG_PATH

    prev = read_proc_stat()
    cores = sorted(prev.keys(), key=lambda c: int(c[3:]))
    n_cores = len(cores)

    header = (['timestamp'] + [f'{c}_pct' for c in cores]
              + [f'{c}_freq_hz' for c in cores]
              + ['temp_c', 'load_1m', 'load_5m', 'load_15m'])

    write_header = True
    try:
        with open(out_path, 'r') as f:
            if f.readline():
                write_header = False
    except FileNotFoundError:
        pass

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(header)
            f.flush()
            os.fsync(f.fileno())

        while True:
            t0 = time.monotonic()
            cur = read_proc_stat()

            pct = []
            for c in cores:
                busy0, total0 = prev.get(c, (0, 0))
                busy1, total1 = cur.get(c, (0, 0))
                dtotal = total1 - total0
                dbusy = busy1 - busy0
                pct.append(round(100.0 * dbusy / dtotal, 2) if dtotal > 0 else 0.0)
            prev = cur

            freqs = read_core_freqs(n_cores)
            temp_c = read_temp()
            load1, load5, load15 = read_loadavg()

            row = ([time.strftime('%Y-%m-%d %H:%M:%S.') + f'{time.time() % 1:.3f}'[2:]]
                   + pct + freqs + [temp_c, load1, load5, load15])
            writer.writerow(row)
            f.flush()
            os.fsync(f.fileno())

            # Deltas need a real gap between samples to divide by -- this
            # paces that gap to ~interval regardless of how long the reads/
            # write above took, same approach as power_logger.py.
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))


if __name__ == '__main__':
    main()
