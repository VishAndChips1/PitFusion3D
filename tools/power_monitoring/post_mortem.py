#!/usr/bin/env python3
"""Find power-loss gaps in power_log.csv and cross-reference boot_events.log,
kernel_log.txt, and usb_events.log around each one -- a one-shot post-mortem
instead of manually comparing four files by hand.

Usage: post_mortem.py [min_gap_seconds]  (default 5.0 -- real reboots take
tens of seconds; anything shorter is just sampling jitter, not a crash)
"""
import csv
import sys
from datetime import datetime, timedelta

DIR = '/home/vish/power_logs'
MIN_GAP_S = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def load_power_rows():
    rows = []
    with open(f'{DIR}/power_log.csv') as f:
        for row in csv.DictReader(f):
            try:
                row['_ts'] = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S.%f')
                row['ext5v_v'] = float(row['ext5v_v'])
            except (ValueError, KeyError):
                continue
            rows.append(row)
    return rows


def load_text_log(path, ts_parser):
    lines = []
    try:
        with open(path) as f:
            for line in f:
                ts = ts_parser(line)
                if ts:
                    lines.append((ts, line.rstrip()))
    except FileNotFoundError:
        pass
    return lines


def parse_boot_event_ts(line):
    try:
        return datetime.strptime(line[:19], '%Y-%m-%d %H:%M:%S')
    except ValueError:
        return None


def parse_boot_id(line):
    if 'boot_id=' in line:
        return line.split('boot_id=')[1].split()[0]
    return None


def parse_usb_event_ts(line):
    try:
        return datetime.strptime(line[:23], '%Y-%m-%d %H:%M:%S.%f')
    except ValueError:
        return None


def parse_kernel_ts(line):
    # journalctl -o short-iso lines start like "2026-10-23T13:17:25+1100 host kernel: ..."
    try:
        return datetime.strptime(line[:19], '%Y-%m-%dT%H:%M:%S')
    except ValueError:
        return None


def window(entries, start, end, pad=timedelta(seconds=2)):
    return [(ts, line) for ts, line in entries if start - pad <= ts <= end + pad]


def main():
    rows = load_power_rows()
    if len(rows) < 2:
        print('Not enough samples yet.')
        return

    boot_events = load_text_log(f'{DIR}/boot_events.log', parse_boot_event_ts)
    usb_events = load_text_log(f'{DIR}/usb_events.log', parse_usb_event_ts)
    kernel_events = load_text_log(f'{DIR}/kernel_log.txt', parse_kernel_ts)

    gaps = []
    for i in range(1, len(rows)):
        dt = (rows[i]['_ts'] - rows[i - 1]['_ts']).total_seconds()
        if dt >= MIN_GAP_S:
            gaps.append((rows[i - 1], rows[i], dt))

    print(f'{len(rows)} power samples, {rows[0]["_ts"]} -> {rows[-1]["_ts"]}')
    print(f'{len(gaps)} gap(s) >= {MIN_GAP_S}s found (likely real power-loss events)\n')

    for n, (before, after, dt) in enumerate(gaps, 1):
        boot_id_before = next(
            (parse_boot_id(l) for ts, l in reversed(boot_events) if ts <= before['_ts'] and parse_boot_id(l)),
            None)
        boot_id_after = next(
            (parse_boot_id(l) for ts, l in boot_events if ts >= before['_ts'] and parse_boot_id(l)),
            None)
        real_reboot = boot_id_before is not None and boot_id_after is not None and boot_id_before != boot_id_after
        kind = 'FULL SYSTEM REBOOT (real power loss)' if real_reboot else \
               'just a logger-process restart, NOT a real power loss' if boot_id_before else \
               'unknown (no boot_id history yet)'

        print('=' * 70)
        print(f'EVENT {n}: lost power ~{before["_ts"]}, back at {after["_ts"]}  ({dt:.1f}s gap)  -- {kind}')
        print('-' * 70)

        lead_up = [r for r in rows if before['_ts'] - timedelta(seconds=10) <= r['_ts'] <= before['_ts']]
        print('Last ~10s before the gap (voltage / temp / cpu freq / load):')
        for r in lead_up[-10:]:
            print(f'  {r["_ts"]}  {r["ext5v_v"]:.3f}V  '
                  f'temp={r.get("temp_c","?")}C  freq={r.get("cpu_freq_hz","?")}  '
                  f'load={r.get("loadavg_1m","?")}  throttled={r.get("throttled","?")}')

        be = window(boot_events, before['_ts'], after['_ts'])
        if be:
            print('\nBoot events in window:')
            for ts, line in be:
                print(f'  {line}')

        ue = window(usb_events, before['_ts'], after['_ts'])
        if ue:
            print('\nUSB events in window:')
            for ts, line in ue:
                print(f'  {line}')

        ke = window(kernel_events, before['_ts'], after['_ts'])
        if ke:
            print(f'\nKernel log lines in window ({len(ke)}):')
            for ts, line in ke[:30]:
                print(f'  {line}')
            if len(ke) > 30:
                print(f'  ... and {len(ke)-30} more')
        print()


if __name__ == '__main__':
    main()
