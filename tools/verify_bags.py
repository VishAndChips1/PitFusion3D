#!/usr/bin/env python3
"""Check that each directory given is a real, readable ROS 2 bag and print what is in it.

For every bag: the storage format, duration, message count per topic, whether every message can be read back,
and a decoded sample (image size / point count). Exit code 1 if any bag is unreadable or missing the LiDAR or RGB data.
usage: verify_bags.py BAG_DIR [BAG_DIR ...]
"""
import os
import sys

import yaml

if 'rosbag2_py' not in sys.modules:
    try:
        import rosbag2_py
    except ImportError:
        os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash 2>/dev/null; '
                           'exec python3 "$0" "$@"', os.path.abspath(__file__), *sys.argv[1:]])
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

REQUIRED = {'/landfill/livox/points': 40, '/landfill/rgb/image_raw': 40}      # minimum messages expected in 5 s (10 Hz sensors)


def check(path):
    """Return (ok, report lines) for one bag directory."""
    lines, ok = [], True
    meta_file = os.path.join(path, 'metadata.yaml')
    if not os.path.isfile(meta_file):
        return False, [f'{path}: NOT A ROS 2 BAG (no metadata.yaml)']
    info = yaml.safe_load(open(meta_file))['rosbag2_bagfile_information']
    storage = info['storage_identifier']
    files = info['relative_file_paths']
    size_mb = sum(os.path.getsize(os.path.join(path, f)) for f in files) / 1e6
    lines.append(f'{path}\n  format: ROS 2 bag, storage={storage}, files={files}, {size_mb:.0f} MB, duration {info["duration"]["nanoseconds"] / 1e9:.2f} s')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id=storage), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts, sample = dict.fromkeys(types, 0), {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] += 1
        if topic not in sample:
            sample[topic] = deserialize_message(data, get_message(types[topic]))
    for topic in sorted(counts):
        extra = ''
        m = sample.get(topic)
        if m is not None and hasattr(m, 'width') and hasattr(m, 'encoding'):
            extra = f'  first image {m.width}x{m.height} {m.encoding}'
        elif m is not None and hasattr(m, 'point_step'):
            extra = f'  first cloud {m.width * m.height} points'
        lines.append(f'  {counts[topic]:5d} msgs  {topic}{extra}')
    for topic, need in REQUIRED.items():
        if counts.get(topic, 0) < need:
            ok = False
            lines.append(f'  PROBLEM: {topic} has {counts.get(topic, 0)} messages, expected at least {need}')
    thermal = counts.get('/landfill/thermal/image_raw', 0)
    lines.append(f'  thermal frames: {thermal}' + ('' if thermal else '  (thermal camera not streaming: no thermal data in this bag)'))
    return ok, lines


if __name__ == '__main__':
    all_ok = True
    for bag in sys.argv[1:]:
        good, report = check(bag.rstrip('/'))
        all_ok &= good
        print('\n'.join(report) + ('\n  RESULT: OK' if good else '\n  RESULT: PROBLEM'))
    sys.exit(0 if all_ok else 1)
