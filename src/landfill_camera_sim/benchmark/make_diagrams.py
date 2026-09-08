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
        facecolor='none', edgecolor=color, linewidth=2.2, zorder=3))
    ax.text(x + w / 2, y + h * 0.63, title, ha='center', va='center',
            fontsize=15, fontweight='bold', color=INK, zorder=4)
    ax.text(x + w / 2, y + h * 0.27, subtitle, ha='center', va='center',
            fontsize=11.5, color=MUTED, zorder=4)


def _arrow(ax, x0, y0, x1, y1, label=None, color=MUTED, label_above=True):
    ax.add_patch(FancyArrowPatch(
        (x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=20,
        color=color, linewidth=2.0, zorder=1,
        shrinkA=3, shrinkB=3))
    if label:
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        offset = 0.18 if label_above else -0.30
        ax.text(mx, my + offset, label, ha='center', va='bottom' if label_above else 'top',
                fontsize=11, color=INK, style='italic', zorder=5,
                bbox=dict(boxstyle='round,pad=0.22', facecolor='white',
                          edgecolor='none', alpha=0.95))


def fig_pipeline():
    """Two-row serpentine flow: wide boxes so the labels stay readable."""
    fig, ax = plt.subplots(figsize=(15, 9.5))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 9.5)
    ax.axis('off')

    ax.text(0.4, 9.05, 'PitFusion3D pipeline', fontsize=25, fontweight='bold', color=INK)
    ax.text(0.4, 8.55, 'Gazebo sensors  ->  Livox resample  ->  RGB/thermal fusion  ->  '
                       'windowed ICP merge  ->  browser', fontsize=13, color=MUTED)

    w, h = 4.2, 1.5
    top, bottom = 6.3, 3.3
    xa, xb, xc = 0.4, 5.4, 10.4

    _node(ax, xa, top, w, h, 'Gazebo Sim', 'RGB / thermal / gpu_lidar', SIM_C)
    _node(ax, xb, top, w, h, 'ros_gz_bridge', 'gz topics  ->  ROS 2', SIM_C)
    _node(ax, xc, top, w, h, 'livox_pattern_filter', 'true-angle resample', PROC_C)

    _node(ax, xa, bottom, w, h, 'sensor_fusion_node', 'colour + temperature', PROC_C)
    _node(ax, xb, bottom, w, h, 'registration_merge', 'ICP over 1s window', PROC_C)
    _node(ax, xc, bottom, w, h, 'web streamer', 'browser on :8080', OUT_C)

    _node(ax, xa, 0.75, w, 1.25, 'calibration_publisher', 'intrinsics + extrinsics', SIM_C)
    _node(ax, xb, 0.75, w, 1.25, 'platform_motion', 'drives the truck', SIM_C)

    mid_top, mid_bottom = top + h / 2, bottom + h / 2
    _arrow(ax, xa + w, mid_top, xb, mid_top, 'sensor topics')
    _arrow(ax, xb + w, mid_top, xc, mid_top, 'raw_scan/points')
    _arrow(ax, xa + w, mid_bottom, xb, mid_bottom, 'points_rgbt')
    _arrow(ax, xb + w, mid_bottom, xc, mid_bottom, 'points_merged')

    # Wrap from the end of the top row down and back to the start of the
    # bottom row, the way a serpentine flow chart reads. Drawn as an explicit
    # elbow: matplotlib's 'angle' connector rejects a 180-degree pair.
    wrap_y = (top + bottom + h) / 2
    ax.plot([xc + w / 2, xc + w / 2], [top - 0.05, wrap_y],
            color=MUTED, linewidth=2.0, zorder=1, solid_capstyle='round')
    ax.plot([xc + w / 2, xa + w / 2], [wrap_y, wrap_y],
            color=MUTED, linewidth=2.0, zorder=1, solid_capstyle='round')
    ax.add_patch(FancyArrowPatch(
        (xa + w / 2, wrap_y), (xa + w / 2, bottom + h),
        arrowstyle='-|>', mutation_scale=20, color=MUTED, linewidth=2.0,
        zorder=1, shrinkA=0, shrinkB=3))
    ax.text((xa + xc) / 2 + w / 2, wrap_y + 0.14, 'livox/points', fontsize=11,
            color=INK, style='italic', ha='center',
            bbox=dict(boxstyle='round,pad=0.22', facecolor='white',
                      edgecolor='none', alpha=0.95), zorder=5)

    _arrow(ax, xa + w / 2, 2.0, xa + w / 2, bottom, 'camera_info', label_above=False)
    # Short stub rather than a long line: cmd_vel goes back through the bridge
    # into Gazebo, and drawing that literally would cut across the whole chart.
    _arrow(ax, xb + w / 2, 2.0, xb + w / 2, 2.75, None)
    ax.text(xb + w / 2 + 0.15, 2.5, 'cmd_vel  ->  ros_gz_bridge  ->  Gazebo',
            fontsize=11, color=INK, style='italic', va='center', ha='left',
            bbox=dict(boxstyle='round,pad=0.22', facecolor='white',
                      edgecolor='none', alpha=0.95), zorder=5)

    legend = [('blue: simulation + bridge', SIM_C),
              ('green: processing nodes (the Raspberry Pi 5 workload)', PROC_C),
              ('amber: output', OUT_C)]
    x = 0.4
    for label, color in legend:
        ax.text(x, 0.25, label, fontsize=11.5, color=color, fontweight='bold')
        x += len(label) * 0.107 + 0.5
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
