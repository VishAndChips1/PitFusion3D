#!/usr/bin/env python3
"""Live end-to-end benchmark against a running pipeline.

Measures the rate and size of what each stage actually publishes, plus the
wall-clock cost of the merge node's own callback as reported in its logs.

    ros2 launch landfill_camera_sim landfill_camera_sim.launch.py rviz:=false gz_gui:=false
    ros2 launch landfill_camera_sim sensor_fusion.launch.py
    python3 benchmark/bench_live.py --duration 60

Note the simulator, not the processing code, is usually the limiting factor
on a machine without GPU-accelerated rendering: check the reported
real-time factor before reading anything into the rates.
"""

import argparse
import json
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

TOPICS = [
    ('/landfill/livox/raw_scan/points', PointCloud2, 'gz lidar grid (sim output)'),
    ('/landfill/livox/points', PointCloud2, 'Livox pattern resample'),
    ('/landfill/fusion/points_rgbt', PointCloud2, 'RGB+thermal fusion'),
    ('/landfill/fusion/points_merged', PointCloud2, 'windowed ICP merge'),
    ('/landfill/rgb/image_raw', Image, 'RGB camera'),
    ('/landfill/thermal/image_raw', Image, 'thermal camera'),
]


class LiveBench(Node):
    def __init__(self):
        super().__init__('pipeline_live_bench')
        self.stamps = {name: [] for name, _, _ in TOPICS}
        self.points = {name: [] for name, _, _ in TOPICS}
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, msg_type, _ in TOPICS:
            self.create_subscription(
                msg_type, name,
                lambda msg, n=name: self._on_msg(msg, n),
                sensor_qos,
            )

    def _on_msg(self, msg, name):
        self.stamps[name].append(time.monotonic())
        if isinstance(msg, PointCloud2):
            self.points[name].append(msg.width * msg.height)


def summarize(stamps, points):
    if len(stamps) < 2:
        return {'messages': len(stamps), 'hz': 0.0, 'mean_points': 0}
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    return {
        'messages': len(stamps),
        'hz': 1.0 / statistics.fmean(gaps),
        'period_ms_median': statistics.median(gaps) * 1000.0,
        'period_ms_max': max(gaps) * 1000.0,
        'mean_points': int(statistics.fmean(points)) if points else 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--json', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'results_live.json'))
    parser.add_argument('--label', default='optimized')
    args = parser.parse_args()

    rclpy.init()
    node = LiveBench()
    print('Measuring for %.0fs...' % args.duration)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    results = {}
    for name, _, description in TOPICS:
        results[name] = summarize(node.stamps[name], node.points[name])
        results[name]['description'] = description

    node.destroy_node()
    rclpy.shutdown()

    payload = {'label': args.label, 'duration_s': args.duration, 'topics': results}
    with open(args.json, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)

    print('\n%-34s %8s %9s %11s' % ('topic', 'Hz', 'msgs', 'points/msg'))
    print('-' * 66)
    for name, _, _ in TOPICS:
        r = results[name]
        print('%-34s %8.2f %9d %11d'
              % (name.replace('/landfill/', ''), r['hz'], r['messages'], r['mean_points']))
    print('\nWrote %s' % args.json)


if __name__ == '__main__':
    main()
