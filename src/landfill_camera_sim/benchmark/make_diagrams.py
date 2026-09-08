#!/usr/bin/env python3
"""Render the README's architecture and platform figures.

    python3 benchmark/make_diagrams.py

Writes benchmark/report/pipeline.png and benchmark/report/truck_platform.png.
The platform figure is drawn from the world template itself, so it stays
honest if the model changes.
"""

import argparse
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, 'report')

INK = '#1b1f24'
MUTED = '#5b6470'
SIM_C = '#3b6ea5'
PROC_C = '#2f7d6d'
OUT_C = '#b26a1f'
GRID = '#d7dbe0'


def _node(ax, x, y, w, h, title, subtitle, color):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.06',
        facecolor=color, edgecolor='none', alpha=0.13, zorder=2))
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.06',
        facecolor='none', edgecolor=color, linewidth=1.6, zorder=3))
    ax.text(x + w / 2, y + h * 0.62, title, ha='center', va='center',
            fontsize=9.5, fontweight='bold', color=INK, zorder=4)
    ax.text(x + w / 2, y + h * 0.26, subtitle, ha='center', va='center',
            fontsize=7.6, color=MUTED, zorder=4)


def _arrow(ax, x0, y0, x1, y1, label=None, color=MUTED):
    ax.add_patch(FancyArrowPatch(
        (x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=13,
        color=color, linewidth=1.3, zorder=1,
        shrinkA=2, shrinkB=2))
    if label:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + 0.11, label, ha='center',
                va='bottom', fontsize=7.1, color=MUTED, style='italic', zorder=5)


def fig_pipeline():
    fig, ax = plt.subplots(figsize=(12, 5.4))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.4)
    ax.axis('off')

    ax.text(0.15, 5.15, 'PitFusion3D pipeline', fontsize=15, fontweight='bold', color=INK)
    ax.text(0.15, 4.85, 'Gazebo sensors -> Livox resample -> RGB/thermal fusion -> '
                        'windowed ICP merge -> browser', fontsize=9, color=MUTED)

    w, h = 2.05, 0.95
    _node(ax, 0.15, 3.35, w, h, 'Gazebo Sim', 'RGB / thermal / gpu_lidar', SIM_C)
    _node(ax, 0.15, 1.95, w, h, 'ros_gz_bridge', 'gz topics -> ROS 2', SIM_C)
    _node(ax, 0.15, 0.55, w, h, 'platform_motion', 'truck cmd_vel', SIM_C)

    _node(ax, 2.75, 1.95, w, h, 'livox_pattern_filter', 'true-angle resample', PROC_C)
    _node(ax, 5.35, 1.95, w, h, 'sensor_fusion_node', 'colour + temperature', PROC_C)
    _node(ax, 7.95, 1.95, w, h, 'registration_merge', 'ICP over 1s window', PROC_C)
    _node(ax, 10.45, 1.95, 1.4, h, 'web streamer', ':8080', OUT_C)

    _node(ax, 5.35, 3.35, w, h, 'calibration_publisher', 'intrinsics + extrinsics', SIM_C)

    _arrow(ax, 1.18, 3.35, 1.18, 2.90, None)
    _arrow(ax, 2.20, 2.42, 2.75, 2.42, 'raw_scan/points')
    _arrow(ax, 4.80, 2.42, 5.35, 2.42, 'livox/points')
    _arrow(ax, 7.40, 2.42, 7.95, 2.42, 'points_rgbt')
    _arrow(ax, 10.00, 2.42, 10.45, 2.42, 'points_merged')
    _arrow(ax, 6.37, 3.35, 6.37, 2.90, 'camera_info')
    # The controller publishes cmd_vel up into the bridge, which forwards it
    # to Gazebo's VelocityControl plugin.
    _arrow(ax, 1.18, 1.50, 1.18, 1.95, 'cmd_vel')
    ax.text(2.45, 1.02, 'drives the truck through the scene', fontsize=7.4,
            color=MUTED, style='italic', va='center')

    ax.text(0.15, 0.12, 'blue: simulation + bridge     green: processing nodes '
                        '(the Raspberry Pi 5 workload)     amber: output',
            fontsize=7.8, color=MUTED)
    fig.tight_layout()
    return fig


def _parse_visuals(sdf_text):
    """Pull box/cylinder visuals with numeric poses out of the platform model."""
    model = re.search(r'<model name="camera_platform">(.*?)\n    </model>',
                      sdf_text, re.DOTALL)
    if not model:
        return []
    body = model.group(1)
    out = []
    for block in re.finditer(r'<visual name="([^"]+)">(.*?)</visual>', body, re.DOTALL):
        name, inner = block.group(1), block.group(2)
        pose = re.search(r'<pose>([^<]+)</pose>', inner)
        if not pose:
            continue
        values = pose.group(1).split()
        if len(values) < 6 or any('__' in v for v in values):
            continue
        px, py, pz = (float(v) for v in values[:3])
        diffuse = re.search(r'<diffuse>([^<]+)</diffuse>', inner)
        color = [float(c) for c in diffuse.group(1).split()[:3]] if diffuse else [0.5] * 3
        box = re.search(r'<box>\s*<size>([^<]+)</size>', inner, re.DOTALL)
        cyl = re.search(r'<cylinder>\s*<radius>([^<]+)</radius>\s*<length>([^<]+)</length>',
                        inner, re.DOTALL)
        if box:
            sx, sy, sz = (float(v) for v in box.group(1).split())
            out.append((name, 'box', (px, py, pz), (sx, sy, sz), color))
        elif cyl:
            r = float(cyl.group(1))
            length = float(cyl.group(2))
            rot_y = abs(float(values[4]) - 1.5708) < 0.2
            size = (length, 2 * r, 2 * r) if rot_y else (2 * r, 2 * r, length)
            out.append((name, 'cyl', (px, py, pz), size, color))
    return out


def fig_truck(sdf_path):
    fig = plt.figure(figsize=(9.5, 5.2))
    ax = fig.add_subplot(111, projection='3d')

    if not os.path.exists(sdf_path):
        ax.text(0.5, 0.5, 0.5, 'world template not found')
        return fig

    # Resolve the mast placeholders to what launch computes for the default
    # 3.2 m sensor height, so the mast shows up in the figure.
    sdf_text = open(sdf_path, encoding='utf-8').read()
    sdf_text = (sdf_text
                .replace('__TRUCK_MAST_CENTER_Z__', '2.195')
                .replace('__TRUCK_MAST_HEIGHT__', '2.01'))
    visuals = _parse_visuals(sdf_text)
    for name, _kind, (px, py, pz), (sx, sy, sz), color in visuals:
        x0, x1 = px - sx / 2, px + sx / 2
        y0, y1 = py - sy / 2, py + sy / 2
        z0, z1 = pz - sz / 2, pz + sz / 2
        faces = [
            [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
            [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
            [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
            [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
            [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
            [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
        ]
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        ax.add_collection3d(Poly3DCollection(
            faces, facecolor=color, edgecolor='#2b2f35', linewidths=0.25, alpha=0.96))

    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-4.2, 4.2)
    ax.set_zlim(0, 4.2)
    ax.set_box_aspect((5.2, 8.4, 4.2))
    ax.view_init(elev=20, azim=-58)
    ax.set_xlabel('x (m)', fontsize=8, color=MUTED)
    ax.set_ylabel('y (m)  forward ->', fontsize=8, color=MUTED)
    ax.set_zlabel('z (m)', fontsize=8, color=MUTED)
    ax.tick_params(labelsize=7, colors=MUTED)
    ax.set_title('Sensor truck (camera_platform) - %d visuals from the world template'
                 % len(visuals), fontsize=11, fontweight='bold', color=INK, pad=6)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor('white')
        pane.pane.set_edgecolor(GRID)
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sdf', default=os.path.join(
        os.path.dirname(HERE), 'worlds', 'landfill_camera_world.sdf.in'))
    parser.add_argument('--out-dir', default=OUT_DIR)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    for name, fig in (('pipeline', fig_pipeline()), ('truck_platform', fig_truck(args.sdf))):
        path = os.path.join(args.out_dir, '%s.png' % name)
        fig.savefig(path, dpi=150, facecolor='white', bbox_inches='tight')
        plt.close(fig)
        print('Wrote %s' % path)


if __name__ == '__main__':
    main()
