#!/usr/bin/env python3
"""Verify the Avia scan pattern against Livox-SDK/livox_laser_simulation.

The upstream plugin is ROS 1 / Gazebo Classic and cannot be used directly
under ROS 2 + gz-sim, so livox_pattern_filter.py reimplements its geometry.
These tests pin that reimplementation to the upstream reference:

    // livox_points_plugin.cpp
    avia_infos.back().azimuth = data[1] * deg_2_rad;
    avia_infos.back().zenith  = data[2] * deg_2_rad - M_PI_2;
    ...
    ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
    auto axis  = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
    auto point = range * axis;

Run with:  python3 -m pytest test/test_scan_pattern.py -v
"""

import math
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV_PATH = os.path.join(PACKAGE_ROOT, 'config', 'scan_patterns', 'avia.csv')

# Avia envelope, from the sensor's datasheet and config/sensors_sim.yaml.
HORIZONTAL_FOV_DEG = 70.4
VERTICAL_FOV_DEG = 77.2


def load_pattern(max_rows=50000):
    if not os.path.exists(CSV_PATH):
        pytest.skip('avia.csv not present')
    raw = np.loadtxt(CSV_PATH, delimiter=',', skiprows=1, usecols=(1, 2),
                     max_rows=max_rows)
    azimuth = np.radians(raw[:, 0])
    zenith = np.radians(raw[:, 1]) - math.pi / 2.0
    return azimuth, zenith


def package_directions(azimuth, zenith):
    """The closed form used by livox_pattern_filter.py."""
    cos_zenith = np.cos(zenith)
    dirs = np.empty((azimuth.shape[0], 3))
    dirs[:, 0] = cos_zenith * np.cos(azimuth)
    dirs[:, 1] = cos_zenith * np.sin(azimuth)
    dirs[:, 2] = -np.sin(zenith)
    return dirs


def upstream_directions(azimuth, zenith):
    """Reference: the quaternion the Gazebo Classic plugin builds."""
    rotation = Rotation.from_euler(
        'xyz', np.stack([np.zeros_like(azimuth), zenith, azimuth], axis=1))
    return rotation.apply(np.array([1.0, 0.0, 0.0]))


def test_directions_match_upstream_quaternion():
    azimuth, zenith = load_pattern()
    difference = np.abs(package_directions(azimuth, zenith)
                        - upstream_directions(azimuth, zenith))
    assert difference.max() < 1e-12, 'max deviation %.3e' % difference.max()


def test_directions_are_unit_length():
    azimuth, zenith = load_pattern()
    norms = np.linalg.norm(package_directions(azimuth, zenith), axis=1)
    assert np.allclose(norms, 1.0)


def test_elevation_and_azimuth_are_separable():
    """A ray's elevation is exactly -zenith and its azimuth exactly azimuth.

    This is what makes interpolating the gz-sim range image on an
    (azimuth, elevation) grid valid.
    """
    azimuth, zenith = load_pattern(max_rows=20000)
    dirs = package_directions(azimuth, zenith)
    elevation = np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0))
    recovered_azimuth = np.arctan2(dirs[:, 1], dirs[:, 0])
    assert np.allclose(elevation, -zenith, atol=1e-12)
    assert np.allclose(recovered_azimuth, azimuth, atol=1e-9)


def test_pattern_covers_the_avia_field_of_view():
    azimuth, zenith = load_pattern(max_rows=120000)
    azimuth_span = np.degrees(azimuth.max() - azimuth.min())
    elevation_span = np.degrees((-zenith).max() - (-zenith).min())
    assert azimuth_span == pytest.approx(HORIZONTAL_FOV_DEG, abs=0.5)
    assert elevation_span == pytest.approx(VERTICAL_FOV_DEG, abs=0.5)


def test_scan_slice_is_two_dimensional():
    """One scan must sweep a 2D rosette, not a handful of scan lines.

    Snapping rays to the gz-sim grid used to collapse each scan onto
    vertical_samples distinct elevations, which renders as banding.
    """
    azimuth, zenith = load_pattern(max_rows=24000)
    elevation = -zenith
    assert np.degrees(azimuth.max() - azimuth.min()) > 65.0
    assert np.degrees(elevation.max() - elevation.min()) > 70.0
    # Distinct elevations far exceed the 120-row grid the old code quantised to.
    assert len(np.unique(np.round(np.degrees(elevation), 2))) > 2000


def test_consecutive_scans_do_not_repeat():
    """The Avia is non-repetitive: successive scans must walk new rays."""
    azimuth, zenith = load_pattern(max_rows=48000)
    first = np.stack([azimuth[:24000], zenith[:24000]], axis=1)
    second = np.stack([azimuth[24000:48000], zenith[24000:48000]], axis=1)
    shared = set(map(tuple, np.round(first, 9))) & set(map(tuple, np.round(second, 9)))
    assert len(shared) < 0.02 * len(first)
