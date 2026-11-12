import os, signal, subprocess, sys, time, psutil
NODES = [('livox_restamp_node','restamp'),('calibration_publisher','calib_pub'),('sensor_fusion_node','fusion'),
         ('registration_merge_node','merge'),('fusion_web_streamer','streamer'),('thermal_camera_node','thermal'),
         ('rgb_camera_node','rgb'),('livox_ros2_avia_node','livox_drv')]
def pids(pat):
    out = set()
    for p in psutil.process_iter(['pid','cmdline']):
        cl = p.info['cmdline'] or []
        if p.info['pid'] != os.getpid() and '--ros-args' in cl and any(pat in t for t in cl[:3]) and 'launch_params' in ' '.join(cl):
            out.add(p.info['pid'])
    return out
def viewer_pts():
    r = subprocess.run("curl -sS -m 4 http://localhost:8080/live.bin | gunzip 2>/dev/null | python3 -c \"import sys,struct;b=sys.stdin.buffer.read();print(struct.unpack_from('<I',b,0)[0] if len(b)>=4 else 0)\"",shell=True,capture_output=True,text=True)
    try: return int(r.stdout.strip())
    except: return -1
sel = sys.argv[1:] or [n for n,_ in NODES]
for pat,label in NODES:
    if pat not in sel: continue
    before = pids(pat)
    if not before: print(f'{label:10s} NOT RUNNING before test', flush=True); continue
    for p in before: os.kill(p, signal.SIGKILL)
    t0 = time.time(); back = None
    while time.time()-t0 < 45:
        time.sleep(1)
        if pids(pat) - before: back = time.time()-t0; break
    time.sleep(10 if back else 0)
    dup = len(pids(pat))
    print(f'{label:10s} kill -9 -> {"respawned in %2.0fs"%back if back else "NOT RESPAWNED (45s)"}; copies now: {dup}; viewer live pts: {viewer_pts()}', flush=True)
