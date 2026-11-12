import subprocess, time, struct, gzip, urllib.request, glob, psutil
def merged_pts():
    try:
        b = gzip.decompress(urllib.request.urlopen('http://localhost:8080/merged.bin', timeout=3).read())
        return struct.unpack_from('<I', b, 0)[0]
    except Exception: return -1
def live_pts():
    try:
        b = gzip.decompress(urllib.request.urlopen('http://localhost:8080/live.bin', timeout=3).read())
        return struct.unpack_from('<I', b, 0)[0]
    except Exception: return -1
def nodes():
    return sorted({k for p in psutil.process_iter(['cmdline']) for k in ('livox_ros2_avia_node','livox_restamp_node','calibration_publisher','rgb_camera_node','thermal_camera_node','sensor_fusion_node','registration_merge_node','fusion_web_streamer') if 'launch_params' in ' '.join(p.info['cmdline'] or []) and any(k in t for t in (p.info['cmdline'] or [])[:3])})
def sh(c): return subprocess.run(c, shell=True, capture_output=True, text=True)
def timed(label, action):
    shm0 = len(glob.glob('/dev/shm/*'))
    t0 = time.time(); action()
    first_live = first_merged = None
    while time.time() - t0 < 90:
        time.sleep(1)
        if first_live is None and live_pts() > 0: first_live = time.time() - t0
        if first_merged is None and merged_pts() > 0: first_merged = time.time() - t0
        if first_live and first_merged: break
    time.sleep(3)
    n = nodes(); rp = len(sh('pgrep -x rpicam-vid').stdout.split())
    print(f'{label:34s} live data after {first_live and round(first_live)}s, merged after {first_merged and round(first_merged)}s | nodes up {len(n)}/8 | rpicam-vid {rp} | /dev/shm files {shm0}->{len(glob.glob("/dev/shm/*"))}', flush=True)
timed('1. SIGKILL entire service (crash)', lambda: sh('systemctl --user kill -s SIGKILL pitfusion3d.service'))
timed('2. systemctl restart', lambda: sh('systemctl --user restart pitfusion3d.service'))
timed('3. stop, then start', lambda: (sh('systemctl --user stop pitfusion3d.service'), time.sleep(2), sh('systemctl --user start pitfusion3d.service')))
timed('4. restart again (repeat)', lambda: sh('systemctl --user restart pitfusion3d.service'))
