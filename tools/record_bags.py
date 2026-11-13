#!/usr/bin/env python3
"""Record 5-second ROS 2 bags (mcap) of every sensor, one per board arrangement.

  python3 ~/checkerboard_bags/record_bags.py              # all five arrangements, asks you to position the board before each
  python3 ~/checkerboard_bags/record_bags.py centre far   # only these arrangements
  python3 ~/checkerboard_bags/record_bags.py --yes ...    # no "press Enter" prompts

Each bag is a folder  ~/checkerboard_bags/bags/<arrangement>/  holding  <arrangement>_0.mcap + metadata.yaml  (a normal rosbag2 bag:
play it with `ros2 bag play`, inspect with `ros2 bag info`). After recording, the bag is read back and checked (verify_bags.py).
The pipeline (pitfusion3d.service) must be running: it owns the sensors.
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

try:
    import rosbag2_py
except ImportError:                      # not run from a sourced ROS shell: re-run this script inside one
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash 2>/dev/null; exec python3 "$0" "$@"',
                       os.path.abspath(__file__), *sys.argv[1:]])

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(HERE, 'bags')
DURATION_S = 5.0
ARRANGEMENTS = {
    'close': 'board about 1.5-2 m from the rig, facing it',
    'far': 'board about 4-5 m from the rig, facing it',
    'left': 'board about 3 m away, to the LEFT of centre, angled slightly towards the rig',
    'right': 'board about 3 m away, to the RIGHT of centre, angled slightly towards the rig',
    'centre': 'board about 3 m away, directly in front, facing the rig',
}
TOPICS = ['/landfill/livox/points',      # LiDAR, host-clock stamps (same clock as the cameras)
          '/livox/lidar',                # LiDAR, raw driver output
          '/livox/imu',
          '/landfill/rgb/image_raw', '/landfill/rgb/camera_info',
          '/landfill/thermal/image_raw', '/landfill/thermal/camera_info',
          '/tf_static']
ENV_SETUP = ('source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash 2>/dev/null; '
             'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; '
             'export FASTRTPS_DEFAULT_PROFILES_FILE=$(find ~/ros2_ws/src -name fastdds_large_buffers.xml | head -1); ')


def pipeline_running():
    """True if the sensor pipeline service is active."""
    return subprocess.run(['systemctl', '--user', 'is-active', '--quiet', 'pitfusion3d.service']).returncode == 0


def record(name):
    """Record one bag for DURATION_S seconds; returns the bag directory."""
    path = os.path.join(BAGS, name)
    raw = os.path.join(BAGS, '_raw_' + name)
    for p in (path, raw):
        if os.path.exists(p):
            shutil.rmtree(p)
    os.makedirs(BAGS, exist_ok=True)
    cmd = ENV_SETUP + 'exec ros2 bag record -s mcap -o "%s" %s' % (raw, ' '.join(TOPICS))
    proc = subprocess.Popen(['bash', '-c', cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            text=True, start_new_session=True)
    # Start the clock only once the recorder has subscribed to every topic.
    ready, deadline, lines = set(), time.time() + 40, []
    while time.time() < deadline and len(ready) < len(TOPICS):
        line = proc.stdout.readline()
        if not line:
            break
        lines.append(line)
        if 'Subscribed to topic' in line:
            ready.add(line.split("'")[1] if "'" in line else line)
    if len(ready) < len(TOPICS):
        print('  note: the recorder subscribed to %d of %d topics (a missing sensor topic is recorded as empty)' % (len(ready), len(TOPICS)))
    time.sleep(1.0)                      # let the first data arrive on all topics (the start is trimmed anyway)
    print('  RECORDING ...', flush=True)
    time.sleep(DURATION_S)
    stop(proc)
    print('  stopped; trimming to exactly %.0f s ...' % DURATION_S)
    trim(raw, path)
    shutil.rmtree(raw)
    return path


def trim(raw_path, final_path):
    """Copy the part of a raw recording that is exactly DURATION_S long, starting once both LiDAR and RGB are flowing.

    The recorder takes a moment to stop, so the raw bag is a little longer than needed. The result is again a normal bag.
    """
    def open_reader():
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=raw_path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        return reader

    first = {}
    reader = open_reader()
    while reader.has_next():
        topic, _, stamp = reader.read_next()
        first.setdefault(topic, stamp)
    t0 = max(first.get('/landfill/livox/points', 0), first.get('/landfill/rgb/image_raw', 0))
    t1 = t0 + int(DURATION_S * 1e9)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=final_path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader = open_reader()
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if t0 <= stamp < t1:
            writer.write(topic, data, stamp)
        elif topic == '/tf_static' and stamp < t0:
            writer.write(topic, data, t0)          # static transforms are sent once, before the window starts
    del writer                           # closes and finalises the file


def stop(proc):
    """Stop the recorder cleanly (SIGINT, then SIGTERM) and wait for the file to be finalised; never SIGKILL."""
    for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 8)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        for _ in range(wait * 10):
            if proc.poll() is not None:
                return
            time.sleep(0.1)
    print('  WARNING: recorder did not stop; the bag may be incomplete. Check it with verify_bags.py')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('arrangements', nargs='*', default=list(ARRANGEMENTS), help='names to record (default: all five)')
    ap.add_argument('--yes', action='store_true', help="don't wait for Enter before each recording")
    args = ap.parse_args()
    if not pipeline_running():
        sys.exit('The sensor pipeline is not running (systemctl --user start pitfusion3d.service). Start it, wait 30 s, and try again.')
    bad = []
    for name in args.arrangements:
        print('\n=== arrangement: %s ===\n  %s' % (name, ARRANGEMENTS.get(name, 'custom arrangement')))
        if not args.yes:
            input('  Position the board, hold still, then press Enter to record %.0f s ... ' % DURATION_S)
        for n in (3, 2, 1):
            print('  %d ...' % n, flush=True)
            time.sleep(1)
        path = record(name)
        ok = subprocess.run([sys.executable, os.path.join(HERE, 'verify_bags.py'), path]).returncode == 0
        if not ok:
            bad.append(name)
            print('  -> this arrangement has a problem: record it again with:  python3 %s %s' % (os.path.abspath(__file__), name))
    print('\nDone. Bags are in %s' % BAGS)
    if bad:
        print('Needs re-recording: %s' % ', '.join(bad))
        sys.exit(1)


if __name__ == '__main__':
    main()
