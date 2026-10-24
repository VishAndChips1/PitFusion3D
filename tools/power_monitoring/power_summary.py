#!/usr/bin/env python3
"""Summarize power_log.csv: min/max input voltage, peak power, any
undervoltage/throttle events, and the biggest voltage drops (the signal
to look at when hunting for what's correlated with a cutout).

Usage: power_summary.py [path to power_log.csv]
"""
import csv
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/home/vish/power_logs/power_log.csv'

rows = []
with open(path) as f:
    for row in csv.DictReader(f):
        try:
            row['ext5v_v'] = float(row['ext5v_v'])
            row['total_w'] = float(row['total_w'])
        except (ValueError, KeyError):
            continue
        rows.append(row)

if not rows:
    print('No samples yet.')
    sys.exit(0)

print(f'{len(rows)} samples, {rows[0]["timestamp"]} -> {rows[-1]["timestamp"]}')

for r in rows:
    try:
        r['est_total_i_a'] = float(r.get('est_total_i_a', 'nan'))
    except ValueError:
        r['est_total_i_a'] = float('nan')

v = [r['ext5v_v'] for r in rows]
w = [r['total_w'] for r in rows]
i = [r['est_total_i_a'] for r in rows if r['est_total_i_a'] == r['est_total_i_a']]  # drop NaN
min_v_row = min(rows, key=lambda r: r['ext5v_v'])
max_w_row = max(rows, key=lambda r: r['total_w'])

print(f'\nInput voltage (EXT5V): min={min(v):.3f}V @ {min_v_row["timestamp"]}  max={max(v):.3f}V  mean={sum(v)/len(v):.3f}V')
print(f'Internal power draw (proxy, not true USB-C input power): '
      f'max={max(w):.2f}W @ {max_w_row["timestamp"]}  mean={sum(w)/len(w):.2f}W')
if i:
    print(f'Estimated total current (power/voltage, excludes regulator losses + USB peripheral draw): '
          f'max={max(i):.3f}A  mean={sum(i)/len(i):.3f}A')

# Pi 5 undervoltage brownout threshold is ~4.6-4.8V; flag anything below 4.8V as a warning sign
low = [r for r in rows if r['ext5v_v'] < 4.8]
if low:
    print(f'\n*** {len(low)} samples below 4.80V (brownout warning zone) ***')
    for r in low[:10]:
        print(f'  {r["timestamp"]}: {r["ext5v_v"]:.3f}V')
    if len(low) > 10:
        print(f'  ... and {len(low)-10} more')

throttled_events = [r for r in rows if r['throttled'] not in ('0x0', '')]
if throttled_events:
    seen = set()
    print(f'\n*** throttled flag was non-zero at {len(throttled_events)} samples ***')
    print('(bit 0=undervolted now, bit 1=freq capped, bit 2=throttled now, bit 3=soft temp limit now,')
    print(' bits 16-19 = same but "has happened since boot" -- these persist until reboot)')
    for r in throttled_events:
        if r['throttled'] not in seen:
            print(f'  {r["timestamp"]}: {r["throttled"]}')
            seen.add(r['throttled'])
else:
    print('\nNo undervoltage/throttle events recorded since this log started '
          '(or since the last reboot cleared the "since boot" bits).')

# Biggest sample-to-sample voltage drops, win useful for correlating with a cutout time
diffs = [(rows[i]['timestamp'], rows[i-1]['ext5v_v'] - rows[i]['ext5v_v'])
         for i in range(1, len(rows))]
diffs.sort(key=lambda d: -d[1])
print('\nBiggest single-step voltage drops:')
for ts, d in diffs[:5]:
    if d > 0.01:
        print(f'  {ts}: dropped {d:.3f}V from previous sample')
