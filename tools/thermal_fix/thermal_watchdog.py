#!/usr/bin/env python3
"""Thermal camera watchdog.  Detects the 'stuck camera' state (camera re-enumerates over and over, ~every 34 s, never streams) from the
USB event log and, if it persists, tries to recover it with an escalating ladder: (0) ganged VBUS power cut of the whole onboard USB group (the only real power loss), (1) port power-cycle, (2) deauthorise/reauthorise,
(3) USB reset.  Each step needs write permission (see install_once.sh); without it the watchdog only logs.  Never touches ROS or the node.
Status: ~/thermal_fix/status.json   Log: ~/thermal_fix/watchdog.log"""
import os, re, sys, time, json, glob, subprocess
VID, PID = '3474', '43e2'; EVENTS = '/home/vish/power_logs/usb_events.log'; HERE = os.path.dirname(os.path.abspath(__file__))
STUCK_N, WINDOW_S = 4, 300          # >=4 camera disconnects within 5 min = stuck
PERSIST_S, MIN_GAP_S, MAX_PER_H = 600, 300, 6   # give it 10 min to heal itself, then act at most every 5 min, 6x/hour
DRY = '--dry-run' in sys.argv
def log(m):
    l = f'[{time.strftime("%Y-%m-%d %H:%M:%S")}] {m}'; print(l, flush=True); open(f'{HERE}/watchdog.log', 'a').write(l + '\n')
def recent_disconnects(window):
    if not os.path.exists(EVENTS): return []
    with open(EVENTS, 'rb') as f:
        f.seek(0, 2); size = f.tell(); f.seek(max(0, size - 60000)); lines = f.read().decode('utf-8', 'replace').splitlines()
    out = []; now = time.time()
    for l in lines:
        if 'Thermal' in l and 'DISCONNECTED' in l:
            try: t = time.mktime(time.strptime(l[:19], '%Y-%m-%d %H:%M:%S'))
            except ValueError: continue
            if now - t <= window: out.append(t)
    return out
def camera_sysdir():
    for d in glob.glob('/sys/bus/usb/devices/*'):
        try:
            if open(f'{d}/idVendor').read().strip() == VID and open(f'{d}/idProduct').read().strip() == PID: return os.path.realpath(d)
        except OSError: pass
def port_dir(dev):
    name = os.path.basename(dev); parent = os.path.dirname(dev); hub = os.path.basename(parent); n = re.split(r'[-.]', name)[-1]
    if hub.startswith('usb'): bus = hub[3:]; return f'{parent}/{bus}-0:1.0/usb{bus}-port{n}'
    return f'{parent}/{hub}:1.0/{hub}-port{n}'
def writable(p): return os.path.exists(p) and os.access(p, os.W_OK)
def w(p, v):
    with open(p, 'w') as f: f.write(str(v))
VBUS_HOLD_S = 30     # how long the whole onboard USB group stays unpowered (lets the camera fully discharge / cool)
def onboard_ports():
    """The disable knobs of every port on the Pi 5's four onboard hubs (RP1 xHCI controllers). VBUS only drops when ALL of them are off."""
    out = []
    for p in glob.glob('/sys/bus/usb/devices/usb[0-9]*/[0-9]*-0:1.0/usb*-port*/disable'):
        if '1000120000.pcie' in os.path.realpath(p): out.append(p)      # RP1 xHCI hubs; excludes the dwc2 OTG hub
    return sorted(out)
def vbus_cycle(hold=VBUS_HOLD_S):
    """Real power cut to everything on the Pi 5's USB ports (ganged VBUS): disable all ports, wait, enable all.  Briefly drops keyboard/mouse too."""
    ports = onboard_ports()
    if not ports or not all(writable(p) for p in ports): raise PermissionError('onboard USB port knobs not writable (run: sudo bash ~/thermal_fix/install_ganged_power.sh)')
    for p in ports: w(p, 1)
    time.sleep(hold)
    for p in ports: w(p, 0)
def settle_check(wait=90):
    """After an action: success if the camera is present and does NOT disconnect again for `wait` seconds."""
    t0 = time.time(); time.sleep(5)
    while time.time() - t0 < wait:
        time.sleep(5)
        if any(t > t0 + 3 for t in recent_disconnects(wait + 10)): return False
    return camera_sysdir() is not None
def ladder(status):
    dev = camera_sysdir(); tried = []
    ports = onboard_ports()
    VBUS_GAP_S = 6 * 3600                                   # a power cut also drops keyboard/mouse: at most once per 6 h unless it has ever worked
    skip_vbus = status.get('last_vbus', 0) and time.time() - status['last_vbus'] < VBUS_GAP_S and not status['recoveries'].get('vbus_cycle')
    if skip_vbus: tried.append(('vbus_cycle', 'skipped (done <6 h ago and never worked)'))
    elif ports and all(writable(p) for p in ports):          # step 0: the only method that really removes power from the camera
        log(f'ACTION vbus_cycle (all {len(ports)} onboard USB ports off for {VBUS_HOLD_S}s)' + (' [dry run]' if DRY else ''))
        status['last_vbus'] = time.time()
        if not DRY:
            try: vbus_cycle()
            except OSError as e: log(f'vbus_cycle failed: {e}'); tried.append(('vbus_cycle', repr(e)))
            else:
                time.sleep(8); ok = settle_check(); tried.append(('vbus_cycle', 'recovered' if ok else 'still unstable'))
                status['attempts']['vbus_cycle'] = status['attempts'].get('vbus_cycle', 0) + 1
                if ok: status['recoveries']['vbus_cycle'] = status['recoveries'].get('vbus_cycle', 0) + 1; log('RECOVERED by vbus_cycle'); return 'vbus_cycle'
    else: tried.append(('vbus_cycle', 'no permission (sudo bash ~/thermal_fix/install_ganged_power.sh)'))
    dev = camera_sysdir()
    if not dev: log(f'camera not on USB at all (cable/port/power) - nothing more software can do: {tried}'); return 'absent'
    port = port_dir(dev)
    steps = [('port_power_cycle', f'{port}/disable', lambda: (w(f'{port}/disable', 1), time.sleep(8), w(f'{port}/disable', 0))),
             ('deauthorize_reauthorize', f'{dev}/authorized', lambda: (w(f'{dev}/authorized', 0), time.sleep(4), w(f'{dev}/authorized', 1)))]
    for name, path, fn in steps:
        if not writable(path): tried.append((name, 'no permission')); continue
        log(f'ACTION {name} on {path}' + (' [dry run]' if DRY else ''))
        if DRY: continue
        try: fn()
        except OSError as e: log(f'{name} failed: {e}'); tried.append((name, repr(e))); continue
        ok = settle_check(); tried.append((name, 'recovered' if ok else 'still unstable')); status['attempts'][name] = status['attempts'].get(name, 0) + 1
        if ok: status['recoveries'][name] = status['recoveries'].get(name, 0) + 1; log(f'RECOVERED by {name}'); return name
    node = None
    for g in glob.glob('/dev/bus/usb/*/*'):
        try:
            b, d = int(g.split('/')[-2]), int(g.split('/')[-1])
            if b == int(open(f'{dev}/busnum').read()) and d == int(open(f'{dev}/devnum').read()): node = g
        except (OSError, ValueError): pass
    if node and writable(node) and not DRY:
        log(f'ACTION usb_reset via usbreset {VID}:{PID}'); subprocess.run(['usbreset', f'{VID}:{PID}'], capture_output=True)
        ok = settle_check(); tried.append(('usb_reset', 'recovered' if ok else 'still unstable'))
        if ok: status['recoveries']['usb_reset'] = status['recoveries'].get('usb_reset', 0) + 1; log('RECOVERED by usb_reset'); return 'usb_reset'
    else: tried.append(('usb_reset', 'no permission' if not DRY else 'dry run'))
    log(f'no method recovered the camera: {tried}'); return tried
def main():
    if '--vbus-test' in sys.argv:
        log('manual vbus_cycle test'); vbus_cycle(); log('vbus_cycle done'); return
    status = dict(state='starting', stuck_since=None, attempts={}, recoveries={}, last_action=0, actions_this_hour=[], permission=False); last_perm_note = 0
    log('watchdog started' + (' (dry run)' if DRY else ''))
    while True:
        n = len(recent_disconnects(WINDOW_S)); now = time.time(); dev = camera_sysdir()
        stuck = n >= STUCK_N; status['recent_disconnects_5min'] = n; status['camera_present'] = dev is not None
        _op = onboard_ports(); status['vbus_permission'] = bool(_op) and all(writable(p) for p in _op)
        if dev: status['permission'] = status['vbus_permission'] or writable(f'{port_dir(dev)}/disable') or writable(f'{dev}/authorized')
        elif status['vbus_permission']: status['permission'] = True
        if stuck and status['stuck_since'] is None: status['stuck_since'] = now; status['state'] = 'stuck'; log(f'camera STUCK: {n} disconnects in {WINDOW_S}s')
        if not stuck and status['stuck_since'] is not None and n == 0:
            log(f'camera recovered by itself/after action after {(now - status["stuck_since"]) / 60:.1f} min'); status['stuck_since'] = None; status['state'] = 'ok'
        if not stuck and status['stuck_since'] is None: status['state'] = 'ok'
        if stuck and now - status['stuck_since'] >= PERSIST_S and now - status['last_action'] >= MIN_GAP_S:
            status['actions_this_hour'] = [t for t in status['actions_this_hour'] if now - t < 3600]
            if len(status['actions_this_hour']) < MAX_PER_H:
                if not status['permission'] and not DRY:
                    if now - last_perm_note > 3600: log('NO PERMISSION to reset the camera. One-time fix:  sudo bash ~/thermal_fix/install_ganged_power.sh'); last_perm_note = now
                else: status['actions_this_hour'].append(now); status['last_action'] = now; ladder(status)
        status['updated'] = time.strftime('%Y-%m-%d %H:%M:%S'); json.dump(status, open(f'{HERE}/status.json', 'w'), indent=1)
        if '--once' in sys.argv: print(json.dumps(status)); return
        time.sleep(30)
if __name__ == '__main__': main()
