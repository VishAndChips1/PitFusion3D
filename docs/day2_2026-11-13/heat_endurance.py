#!/usr/bin/env python3
"""Sustained-heat test of the whole system. baseline -> N busy workers for STRESS_MIN minutes (pipeline running) -> cool-down.
Samples every 3 s: SoC temp, fan RPM + PWM, ARM clock, throttle flags, EXT5V, estimated input current. Pipeline rates every 90 s.
Safety gates: abort the stress if est. input > 4.4 A, EXT5V < 4.75 V or SoC > 84 C. Output: data/heat_endurance.csv + data/heat_endurance_rates.json"""
import sys, os, time, csv, json, re, subprocess, signal, glob
sys.path.insert(0, '/home/vish/thesis_tests/final_campaign_20261112')
import lib
BASE, STRESS, COOL = [float(x) for x in sys.argv[1:4]]; WORKERS = int(sys.argv[4]) if len(sys.argv) > 4 else 4
def fan():
    try: return int(open(glob.glob('/sys/class/hwmon/hwmon*/fan1_input')[0]).read()), int(open(glob.glob('/sys/class/hwmon/hwmon*/pwm1')[0]).read())
    except Exception: return -1, -1
def clk(): return int(re.search(r'=(\d+)', subprocess.run(['vcgencmd', 'measure_clock', 'arm'], capture_output=True, text=True).stdout).group(1)) / 1e6
def rates():
    r = lib.sh(f'{lib.ROS} python3 /home/vish/thesis_tests/final_campaign_20261112/probe_topics.py --topics /landfill/fusion/points_rgbt /landfill/fusion/points_merged --secs 15', 60)
    try: d = json.loads([l for l in r.stdout.splitlines() if l.startswith('{')][-1]); return {k.split('/')[-1]: round(v['hz'], 2) for k, v in d.items()}
    except Exception: return {}
f = open('data/heat_endurance.csv', 'w', newline=''); w = csv.writer(f); w.writerow(['t_s', 'phase', 'soc_c', 'fan_rpm', 'fan_pwm', 'arm_mhz', 'throttled_hex', 'ext5v_v', 'est_in_a', 'est_cons_a', 'usb_drops_cum'])
RATES = []; t0 = time.time(); proc = None; abort = None; phase = 'baseline'; last_rates = 0
def usb_drops(): return int(lib.sh("journalctl -k --no-pager --since '-6 hours' | grep -c 'USB disconnect'", 20).stdout.strip() or 0)
while True:
    el = time.time() - t0
    if el < BASE * 60: want = 'baseline'
    elif el < (BASE + STRESS) * 60 and not abort: want = 'stress'
    elif el < (BASE + STRESS + COOL) * 60 + (0 if not abort else 0): want = 'cooldown'
    else: break
    if abort and want == 'stress': want = 'cooldown'
    if want != phase:
        phase = want; print(f'[{time.strftime("%H:%M:%S")}] phase -> {phase}', flush=True)
        if phase == 'stress': proc = subprocess.Popen(['python3', '/home/vish/thesis_tests/final_campaign_20261112/stress.py', str(WORKERS), str(STRESS * 60 + 30), '/tmp/claude-1000/HEAT_ABORT', '256'])
        if phase == 'cooldown' and proc and proc.poll() is None: open('/tmp/claude-1000/HEAT_ABORT', 'w').write('x'); proc.wait(timeout=30); os.remove('/tmp/claude-1000/HEAT_ABORT')
    d = lib.pmic(); nom, cons, _ = est_i = lib.est_input(d) if d else (float('nan'),) * 3; ext = d.get(('EXT5V', 'V'), float('nan')); temp = lib.soc_temp(); rpm, pwm = fan()
    thr = lib.throttled(); w.writerow([f'{el:.0f}', phase, f'{temp:.1f}', rpm, pwm, f'{clk():.0f}', hex(thr), f'{ext:.3f}', f'{nom:.2f}', f'{cons:.2f}', usb_drops() if int(el) % 60 < 3 else '']); f.flush()
    if phase == 'stress' and (nom > 4.4 or ext < 4.75 or temp > 84): abort = f'SAFETY at {el:.0f}s: est {nom:.2f} A, ext5v {ext:.3f} V, temp {temp:.1f} C'; print(abort, flush=True); open('/tmp/claude-1000/HEAT_ABORT', 'w').write('x')
    if time.time() - last_rates > 90: RATES.append(dict(t_s=round(el), phase=phase, **rates())); last_rates = time.time()
    time.sleep(3)
if os.path.exists('/tmp/claude-1000/HEAT_ABORT'): os.remove('/tmp/claude-1000/HEAT_ABORT')
json.dump(dict(rates=RATES, abort=abort, workers=WORKERS, stress_min=STRESS), open('data/heat_endurance_rates.json', 'w'), indent=1); print('done', abort)
