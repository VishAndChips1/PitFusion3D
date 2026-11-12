#!/usr/bin/env python3
"""Master unattended measurement campaign (2026-11-12). Usage: campaign.py [--force] [phase ...]   (no phase = all, in order).
Resumable: a phase whose data/phase_<name>.json exists is skipped unless --force. Hard safety gates are enforced by PowerSampler."""
import os, sys, json, time, glob, subprocess, threading, signal
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/vish/ros2_ws/install/pitfusion3d_bringup/share/pitfusion3d_bringup/config/fastdds_large_buffers.xml'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *
PS = None; SS = None
def phase_begin(name):
    PS.phase = name; SS.phase = name; mark(name, 'begin'); log(f'=== PHASE {name} begin')
def phase_end(name, result=None):
    mark(name, 'end'); log(f'=== PHASE {name} end')
    if result is not None: save_json(f'phase_{name}', result)
def probe(topics, secs, save=None):
    r = ros_py('probe_topics.py', f'--topics {" ".join(topics)} --secs {secs}' + (f' --save {save}' if save else ''), secs + 40)
    try: return json.loads([l for l in r.stdout.splitlines() if l.startswith('{')][-1])
    except Exception: return dict(error=(r.stdout + r.stderr)[-300:])
def last_json(r):
    try: return json.loads([l for l in r.stdout.splitlines() if l.startswith('{')][-1])
    except Exception: return dict(error=(r.stdout + r.stderr)[-400:])
def service(action, timeout=90): return sh(f'systemctl --user {action} pitfusion3d.service', timeout)
def pipeline_pids(): return [p.pid for p in psutil.process_iter(['cmdline']) if any(k in ' '.join(p.info['cmdline'] or []) for k in ('lib/livox_ros2_avia', 'lib/pitfusion3d_bringup', 'lib/landfill_camera_sim', 'rpicam-vid')) and 'campaign' not in ' '.join(p.info['cmdline'] or [])]
def wait_data(timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = sh('curl -s -m 4 http://localhost:8080/merged.bin | gunzip 2>/dev/null | python3 -c "import sys,struct;b=sys.stdin.buffer.read();print(struct.unpack_from(\'<I\',b,0)[0] if len(b)>=4 else 0)"', 12)
        try:
            if int(r.stdout.strip() or 0) > 0: return time.time() - t0
        except ValueError: pass
        time.sleep(2)
    return None
def abort_file(): return f'{DATA}/ABORT_STRESS'

def P_meta():
    phase_begin('meta')
    info = dict(start=now(), start_epoch=time.time(), git=sh('cd /home/vish/PitFusion3D_repo; git log --oneline | head -3').stdout, uptime=sh('uptime -s').stdout.strip(), camera=camera_state(),
                caps=sh('ros2 param get /fusion_web_streamer max_points_merged; ros2 param get /fusion_web_streamer max_points_live').stdout, power_source=sh('cat /proc/device-tree/chosen/power/max_current | xxd -p').stdout.strip())
    phase_end('meta', info)
def P_inventory():
    phase_begin('inventory'); r = sh(f'nice -n 5 python3 {HERE}/inventory.py', 400); phase_end('inventory', dict(log=r.stdout[-300:]))
def P_code():
    phase_begin('code'); r = sh(f'python3 {HERE}/code_metrics.py', 120); phase_end('code', dict(out=r.stdout[:600]))
def P_idle_os():
    """Pi with the pipeline STOPPED: pure OS baseline (power, temp, CPU)."""
    phase_begin('idle_os'); cs = camera_state(); service('stop'); t0 = time.time()
    while pipeline_pids() and time.time() - t0 < 60: time.sleep(1)
    stop_s = time.time() - t0; time.sleep(30); mark('idle_os', 'steady_begin'); time.sleep(150); mark('idle_os', 'steady_end')
    phase_end('idle_os', dict(stop_seconds=stop_s, leftover_pids=pipeline_pids(), camera_before=cs))
def P_startup():
    """Start the pipeline from stopped: time-to-first-message per topic + inrush current (power sampler records it)."""
    phase_begin('startup'); out = {}
    pr = subprocess.Popen(['bash', '-c', ROS + f'python3 {HERE}/startup_probe.py 100'], stdout=subprocess.PIPE, text=True); time.sleep(4)
    t0 = time.time(); mark('startup', 'service_start_issued'); service('start'); d = wait_data(120); out['viewer_first_merged_data_s'] = d
    o = pr.communicate(timeout=140)[0]; res = last_json(subprocess.CompletedProcess('', 0, o, ''))
    out['first_message_s_after_start'] = {k: v - t0 for k, v in res.get('first', {}).items()}; out['message_counts_100s'] = res.get('counts'); out['t_start'] = t0
    out['nodes_up'] = len(pipeline_pids()); out['camera_after'] = camera_state(); phase_end('startup', out)
def P_idle_pipeline():
    phase_begin('idle_pipeline'); out = {}; time.sleep(60); mark('idle_pipeline', 'steady_begin')
    out['topics_group_lidar'] = probe(['/livox/lidar', '/landfill/livox/points'], 40)
    out['topics_rgb'] = probe(['/landfill/rgb/image_raw'], 30); out['topics_thermal'] = probe(['/landfill/thermal/image_raw'], 20)
    out['topics_fused_merged'] = probe(['/landfill/fusion/points_rgbt', '/landfill/fusion/points_merged'], 40)
    out['lidar_loss'] = [last_json(ros_py('lidar_loss.py', '25', 60)) for _ in range(3)]; out['camera'] = camera_state(); time.sleep(100); mark('idle_pipeline', 'steady_end'); phase_end('idle_pipeline', out)
def P_sensor():
    phase_begin('sensor'); out = {}
    for m, s in (('lidar', 60), ('rgb', 30), ('thermal', 20), ('fused', 60), ('merged', 45)):
        out[m] = last_json(ros_py('chars.py', f'{m} {s}', s + 120)); log(f'sensor {m}: {list(out[m])[:4]}')
    out['camera'] = camera_state(); phase_end('sensor', out)
def P_latency():
    """Stamp-based latency per stage + sync accuracy (stamps saved for offline analysis)."""
    phase_begin('latency'); out = {}
    for name, tp in (('lidar_restamp_fused', ['/livox/lidar', '/landfill/livox/points', '/landfill/fusion/points_rgbt']), ('rgb_thermal', ['/landfill/rgb/image_raw', '/landfill/thermal/image_raw']), ('merged', ['/landfill/fusion/points_merged'])):
        out[name] = probe(tp, 60, save=f'{DATA}/stamps_{name}.npz')
    phase_end('latency', out)
def P_threads():
    phase_begin('threads'); out = [last_json(sh(f'python3 {HERE}/threads.py 60', 90)) for _ in range(2)]; phase_end('threads', dict(runs=out))
def P_transport():
    phase_begin('transport'); out = {}; prof = os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']
    def socks():
        r = sh("ss -uamn -p 2>/dev/null | paste - - | awk '{ if (match($0,/,d[0-9]+\\)/)) { d=substr($0,RSTART+2,RLENGTH-3); split($0,a,\"pid=\"); split(a[2],b,\",\"); print b[1], $5, d } }'", 20).stdout
        return {(l.split()[0], l.split()[1]): int(l.split()[2]) for l in r.splitlines() if len(l.split()) == 3}
    def lo(): return [int(x) for x in sh("awk '/ lo:/ {print $2, $3}' /proc/net/dev").stdout.split()]
    def snmp(): return [int(x) for x in sh("awk '/^Udp:/ && !/InDatagrams/ {print $2,$3,$4,$5,$6,$7}' /proc/net/snmp").stdout.split()]
    a = socks(); l0 = lo(); u0 = snmp(); time.sleep(60); b = socks(); l1 = lo(); u1 = snmp()
    names = {}
    for p in {k[0] for k in b}: names[p] = sh(f'ps -o args= -p {p} | cut -c1-110').stdout.strip()
    out['live_pipeline_60s'] = dict(loopback_MB_per_s=(l1[0] - l0[0]) / 60 / 1e6, loopback_pkts_per_s=(l1[1] - l0[1]) / 60, udp_in_datagrams=u1[0] - u0[0], udp_rcvbuf_errors=u1[4] - u0[4], udp_in_errors=u1[2] - u0[2],
                                    socket_drop_growth={f'{k[0]}:{k[1]} ({names.get(k[0], "?")[-60:]})': b[k] - a.get(k, 0) for k in b if b[k] - a.get(k, 0) > 0})
    shm = sh('ls -la /dev/shm | grep -c fastrtps; du -sh /dev/shm').stdout; out['shm'] = shm
    for name, pf in (('no_profile', 'none'), ('current_profile', prof), ('shm_only_profile', f'{HERE}/shm_only.xml')):
        out[name] = last_json(sh(f'python3 {HERE}/transport_test.py {pf} 25 82', 120))
    phase_end('transport', out)
def P_http():
    phase_begin('http'); out = {}
    for n in (1, 2, 4, 8, 16):
        mark('http', f'clients_{n}_begin'); out[f'clients_{n}_localhost'] = last_json(sh(f'python3 {HERE}/http_bench.py localhost:8080 {n} 30', 90)); out[f'clients_{n}_localhost']['fused_rate'] = None; mark('http', f'clients_{n}_end'); time.sleep(5)
    ip = sh("ip -4 -br addr show wlan0 | awk '{print $3}' | cut -d/ -f1").stdout.strip()
    for n in (1, 4):
        out[f'clients_{n}_wifi_ip'] = last_json(sh(f'python3 {HERE}/http_bench.py {ip}:8080 {n} 30', 90)); time.sleep(5)
    out['pipeline_rates_under_load_note'] = 'see system_1hz.csv between http markers'; phase_end('http', out)
def P_net():
    phase_begin('net'); out = last_json(sh(f'python3 {HERE}/net_bench.py', 240)); phase_end('net', out)
def P_sweep():
    phase_begin('sweep'); PS.paused = True; out = {}
    for view, caps, tag in (('merged', '20000,40000,60000,80000,105000', 'final_merged'), ('live', '5000,10000,15000,25000', 'final_live')):
        r = sh(f'cd {HERE} && python3 sweep.py {view} {caps} 90 {tag}', 1800); out[tag] = r.stdout[-1500:]
    PS.paused = False; sh('ros2 param set /fusion_web_streamer max_points_merged 20000; ros2 param set /fusion_web_streamer max_points_live 5000', 40); phase_end('sweep', out)
def P_accuracy():
    phase_begin('accuracy'); out = dict(wire=last_json(ros_py('accuracy.py', '', 120)))
    j = sh(f'journalctl --user -u pitfusion3d.service --no-pager --since "-60 min" | grep "window_jitter" | tail -n 40').stdout
    import re; jit = [float(m.group(1)) for m in re.finditer(r'window_jitter=([\d.]+)ms', j)]; off = [float(m.group(1)) for m in re.finditer(r'offset=([\d.]+)s', j)]
    out['restamp_window_jitter_ms'] = dict(n=len(jit), mean=sum(jit) / len(jit) if jit else None, max=max(jit) if jit else None); out['restamp_offset_s'] = dict(n=len(off), min=min(off) if off else None, max=max(off) if off else None)
    mj = sh('journalctl --user -u pitfusion3d.service --no-pager --since "-60 min" | grep -E "registration_merge_node" | grep -vE "process started" | cut -c60-240 | tail -n 15').stdout; out['merge_log_tail'] = mj
    phase_end('accuracy', out)
def P_stress():
    phase_begin('stress'); out = {}; abort = abort_file()
    if os.path.exists(abort): os.remove(abort)
    steps = [(0, 0, 'baseline_pipeline_only'), (1, 0, 'cpu_1_worker'), (2, 0, 'cpu_2_workers'), (3, 0, 'cpu_3_workers'), (4, 0, 'cpu_4_workers'), (4, 512, 'cpu_4_workers_plus_512MB_mem')]
    for n, mem, name in steps:
        if PS.abort_reason or (PS.latest.get('est', 0) > 4.0): out[name] = dict(skipped=f'safety: {PS.abort_reason or PS.latest}'); continue
        mark('stress', f'{name}_begin'); p = subprocess.Popen(['python3', f'{HERE}/stress.py', str(n), '75', abort, str(mem)]) if n else None
        t0 = time.time(); trip = None
        while time.time() - t0 < 75:
            time.sleep(1)
            if PS.abort_reason or PS.latest.get('est', 0) > 4.2: open(abort, 'w').write('x'); trip = PS.abort_reason or f"est {PS.latest.get('est')}"; break
            if time.time() - t0 == 0: pass
            if 30 < time.time() - t0 < 32 and name not in out: out[name] = dict(rates=probe(['/landfill/fusion/points_rgbt', '/landfill/fusion/points_merged'], 15))
        if p: p.wait(timeout=30)
        if os.path.exists(abort): os.remove(abort)
        out.setdefault(name, {})['aborted_by_safety'] = trip; mark('stress', f'{name}_end'); time.sleep(20)
    mark('stress', 'cooldown_begin'); time.sleep(180); mark('stress', 'cooldown_end'); phase_end('stress', out)
def P_soak():
    phase_begin('soak'); out = dict(probes=[]); mins = 30; hb = subprocess.Popen(['python3', f'{HERE}/http_bench.py', 'localhost:8080', '2', str(mins * 60)], stdout=subprocess.PIPE, text=True)
    t0 = time.time(); k = 0
    while time.time() - t0 < mins * 60 - 60:
        time.sleep(300 if k else 240); k += 1
        out['probes'].append(dict(t_min=(time.time() - t0) / 60, rates=probe(['/landfill/fusion/points_rgbt', '/landfill/fusion/points_merged', '/landfill/rgb/image_raw'], 20), camera=camera_state(),
                                  journal_errors=int(sh('journalctl --user -u pitfusion3d.service --no-pager --since "-6 min" | grep -cE "ERROR|Traceback"').stdout.strip() or 0)))
    hb.wait(timeout=240); out['viewer_clients'] = last_json(subprocess.CompletedProcess('', 0, hb.stdout.read(), '')); phase_end('soak', out)
def run_suite(cmd, timeout, logname):
    r = sh(f'cd {HERE}/corner && {ROS} {cmd}', timeout); open(f'{LOGD}/{logname}.txt', 'w').write(r.stdout + r.stderr); return r.stdout[-6000:]
def P_corner():
    phase_begin('corner'); out = {}
    for t in ('t_merge', 't_fusion', 't_streamer', 't_restamp', 't_merge_stall'): out[t] = run_suite(f'nice -n 10 python3 {t}.py', 700, t); log(f'corner {t} done')
    phase_end('corner', out)
def P_recovery():
    phase_begin('recovery'); out = {}; fl = subprocess.Popen(['bash', '-c', ROS + f'python3 {HERE}/e2e_fake_lidar.py 0'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) if False else None
    nodes = 'livox_restamp_node calibration_publisher sensor_fusion_node registration_merge_node fusion_web_streamer rgb_camera_node livox_ros2_avia_node'
    out['crash_recovery_sigkill_all_but_thermal'] = run_suite(f'python3 crash_recovery.py {nodes}', 900, 'crash_recovery'); out['rpicam_after_crash'] = sh('ps -eo args | grep -c "^rpicam-vid"').stdout.strip()
    out['restart_cases'] = run_suite('python3 restart_cases.py', 600, 'restart_cases'); out['camera'] = camera_state(); phase_end('recovery', out)
def P_restart_timing():
    phase_begin('restart_timing'); out = []
    for i in range(3):
        pr = subprocess.Popen(['bash', '-c', ROS + f'python3 {HERE}/startup_probe.py 80'], stdout=subprocess.PIPE, text=True); time.sleep(4); t0 = time.time(); mark('restart_timing', f'restart_{i}'); service('restart')
        d = wait_data(100); res = last_json(subprocess.CompletedProcess('', 0, pr.communicate(timeout=120)[0], ''))
        out.append(dict(run=i, viewer_data_s=d, first_message_s={k: v - t0 for k, v in res.get('first', {}).items()}, nodes_up=len(pipeline_pids())))
    phase_end('restart_timing', dict(runs=out, camera=camera_state()))
def P_final():
    phase_begin('final'); out = dict(camera=camera_state(), throttled=sh('vcgencmd get_throttled').stdout.strip(), temp=sh('vcgencmd measure_temp').stdout.strip(), caps=sh('ros2 param get /fusion_web_streamer max_points_merged; ros2 param get /fusion_web_streamer max_points_live').stdout,
        journal_errors_last_6h=sh('journalctl --user -u pitfusion3d.service --no-pager --since "-6 hours" | grep -E "\\[ERROR\\]" | sed -E "s/.*\\[ERROR\\] //" | cut -c1-90 | sort | uniq -c | sort -rn | head -15').stdout,
        kernel_usb_events=sh('journalctl -k --no-pager --since "-6 hours" | grep -cE "USB disconnect"').stdout.strip(), final_inventory_pmic=sh('vcgencmd pmic_read_adc').stdout)
    phase_end('final', out)


def P_thermal_supplement():
    """Thermal camera is streaming again (after a full power loss): capture everything that needs live thermal, then compare with the no-thermal results."""
    phase_begin('thermal_supplement'); out = dict(camera_start=camera_state()); time.sleep(30); mark('thermal_supplement', 'steady_begin')
    out['topics_thermal'] = probe(['/landfill/thermal/image_raw'], 40); out['topics_rgb'] = probe(['/landfill/rgb/image_raw'], 30); out['topics_lidar_fused_merged'] = probe(['/landfill/livox/points', '/landfill/fusion/points_rgbt', '/landfill/fusion/points_merged'], 60, save=f'{DATA}/stamps_thermal_supplement.npz')
    out['sensor'] = {}
    for m, s in (('thermal', 40), ('fused', 90), ('merged', 45)): out['sensor'][m] = last_json(ros_py('chars.py', f'{m} {s}', s + 120)); log(f'thermal_supplement {m} done')
    out['lidar_loss'] = [last_json(ros_py('lidar_loss.py', '25', 60)) for _ in range(2)]
    out['threads'] = last_json(sh(f'python3 {HERE}/threads.py 60', 90)); time.sleep(120); mark('thermal_supplement', 'steady_end')
    out['camera_end'] = camera_state(); out['usb_drops_during'] = int(sh(f'journalctl -k --no-pager --since "@{int(time.time()-1200)}" | grep -c "USB disconnect"').stdout.strip() or 0); phase_end('thermal_supplement', out)

def P_bench():
    """Offline per-kernel benchmark (baseline vs optimised code) with the pipeline STOPPED so nothing competes for the cores."""
    phase_begin('bench'); service('stop'); t0 = time.time()
    while pipeline_pids() and time.time() - t0 < 60: time.sleep(1)
    r = sh(f'cd /home/vish/ros2_ws/src/landfill_camera_sim/benchmark && {ROS} python3 bench_offline.py --repeat 15 --json {DATA}/bench_offline.json', 1500)
    service('start'); wait_data(120); phase_end('bench', dict(stdout=r.stdout[-3000:], rc=r.returncode))
def P_timing():
    """Per-stage fusion timing (decode/project/colour/publish) + RGB/thermal-to-LiDAR stamp offsets, from a 2nd instrumented fusion node on the live inputs."""
    phase_begin('timing'); csvp = f'{DATA}/fusion_stage_timing.csv'
    if os.path.exists(csvp): os.remove(csvp)
    env = dict(os.environ, PF3D_TIMING='1', PF3D_TIMING_PATH=csvp)
    p = subprocess.Popen(['bash', '-c', ROS + 'python3 /home/vish/ros2_ws/src/landfill_camera_sim/scripts/sensor_fusion_node.py --ros-args -r __node:=timing_fusion -p output_topic:=/timing/points_rgbt'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(90); p.send_signal(signal.SIGINT)
    try: p.wait(15)
    except Exception: p.kill()
    rows = []
    if os.path.exists(csvp):
        import csv; rows = list(csv.DictReader(open(csvp)))
    out = dict(rows=len(rows), columns=list(rows[0].keys()) if rows else [])
    if rows:
        import numpy as np
        for k in rows[0]:
            try: v = np.array([float(r[k]) for r in rows if r[k] not in ('', 'nan')]); out[k] = dict(mean=float(v.mean()), p50=float(np.percentile(v, 50)), p95=float(np.percentile(v, 95)), max=float(v.max())) if len(v) else None
            except Exception: pass
    phase_end('timing', out)
ORDER_EXTRA = ['thermal_supplement', 'timing', 'recovery', 'restart_timing', 'bench', 'final']

ORDER = ['meta', 'inventory', 'code', 'idle_os', 'startup', 'idle_pipeline', 'sensor', 'latency', 'threads', 'transport', 'accuracy', 'http', 'net', 'sweep', 'stress', 'soak', 'corner', 'recovery', 'restart_timing', 'final']
if __name__ == '__main__':
    force = '--force' in sys.argv; want = [a for a in sys.argv[1:] if not a.startswith('--')] or ORDER
    PS = PowerSampler(); SS = SystemSampler(); PS.start(); SS.start(); time.sleep(3)
    log(f'campaign start; phases={want}; power sampler running')
    for name in want:
        if os.path.exists(f'{DATA}/phase_{name}.json') and not force: log(f'skip {name} (done)'); continue
        try: globals()[f'P_{name}']()
        except Exception as e:
            import traceback; log(f'PHASE {name} FAILED: {e!r}\n{traceback.format_exc()}'); mark(name, 'failed', error=repr(e)); save_json(f'phase_{name}', dict(failed=repr(e)))
        if PS.abort_reason: log(f'SAFETY ABORT FLAG: {PS.abort_reason}'); PS.abort_reason = None
    PS.stop_flag = True; SS.stop_flag = True; time.sleep(2); log('CAMPAIGN COMPLETE'); open(f'{DATA}/CAMPAIGN_DONE', 'w').write(now())
