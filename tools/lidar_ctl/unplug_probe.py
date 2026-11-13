#!/usr/bin/env python3
"""Measure how much warning the Pi gets when its power is pulled.
  python3 ~/lidar_ctl/unplug_probe.py            # start logging EXT5V as fast as possible; every sample is flushed AND fsync'd to the SD card
  ... pull the power cable ... plug it back in, let the Pi boot ...
  python3 ~/lidar_ctl/unplug_probe.py --report   # shows the last samples before the Pi died
Output: ~/lidar_ctl/unplug_probe.csv  (t_epoch, EXT5V volts).  Run the logger in a terminal; stop it with Ctrl+C."""
import os, sys, time, subprocess
F = os.path.expanduser('~/lidar_ctl/unplug_probe.csv')
def read():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'], capture_output=True, text=True).stdout
    return float(out.split('=')[1].rstrip('V\n'))
if '--report' in sys.argv:
    rows = [l.split(',') for l in open(F).read().splitlines()[1:] if l.count(',') == 1 and l.split(',')[1]]
    t = [float(r[0]) for r in rows]; v = [float(r[1]) for r in rows]
    dts = [b - a for a, b in zip(t, t[1:])]
    print(f'{len(rows)} samples, mean interval {1000 * sum(dts) / len(dts):.0f} ms; log ended {time.strftime("%H:%M:%S", time.localtime(t[-1]))}')
    print('last 25 samples (ms before last, volts):')
    for tt, vv in zip(t[-25:], v[-25:]): print(f'  -{1000 * (t[-1] - tt):6.0f} ms  {vv:.3f} V')
    low = [(1000 * (t[-1] - tt), vv) for tt, vv in zip(t, v) if vv < 4.8 and t[-1] - tt < 5]
    print('samples below 4.8 V in the final 5 s:', low if low else 'NONE -> the supply collapsed faster than one sample interval (no software warning possible at this sampling rate)')
    sys.exit()
with open(F, 'w') as f:
    f.write('t_epoch,ext5v_v\n'); f.flush(); print('logging EXT5V to', F, '- Ctrl+C to stop'); n = 0; t0 = time.time()
    while True:
        try: v = read()
        except Exception: continue
        f.write(f'{time.time():.4f},{v}\n'); f.flush(); os.fsync(f.fileno()); n += 1
        if n % 100 == 0: print(f'{n} samples, {n / (time.time() - t0):.0f} Hz, last {v:.3f} V', flush=True)
