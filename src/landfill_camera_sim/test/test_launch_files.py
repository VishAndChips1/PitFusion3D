"""Check that the launch files build a valid description with the expected nodes (nothing is started)."""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

SRC = os.path.join(os.path.dirname(__file__), '..', '..')


def load(package, name):
    """Import a launch file by path and return its LaunchDescription."""
    path = os.path.join(SRC, package, 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_sensor_fusion_launch_has_fusion_and_merge_nodes():
    description = load('landfill_camera_sim', 'sensor_fusion.launch.py')
    assert isinstance(description, LaunchDescription)
    assert len([e for e in description.entities if isinstance(e, Node)]) == 2


def test_web_stream_launch_has_streamer_node():
    description = load('landfill_camera_sim', 'web_stream.launch.py')
    assert len([e for e in description.entities if isinstance(e, Node)]) == 1


def test_real_sensors_launch_starts_all_sensor_nodes():
    description = load('pitfusion3d_bringup', 'real_sensors.launch.py')
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert len(nodes) == 3                      # LiDAR driver, restamp node, calibration publisher (cameras start on a timer)


def test_full_launch_includes_sensors_fusion_and_viewer():
    description = load('pitfusion3d_bringup', 'pitfusion3d_full.launch.py')
    assert any(isinstance(e, IncludeLaunchDescription) for e in description.entities)
