"""Shared launch helpers for the D-Racer bring-up pipeline.

`find_config()` locates repo config by walking parent dirs (works with
`--symlink-install`; falls back to the D3-G default path otherwise).
`base_nodes()` returns the nodes common to EVERY pipeline launch — camera,
actuator, joystick, monitor, battery — so the launches stay DRY and consistent.
Mode-specific nodes (perception / control / recorder) are added on top by each
launch.
"""
from pathlib import Path

from launch_ros.actions import Node


def find_config(rel, fallback):
    for base in Path(__file__).resolve().parents:
        cand = base / rel
        if cand.exists():
            return str(cand)
    return fallback


def vehicle_config_path():
    return find_config(
        'src/config/vehicle_config.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml')


def default_profile_path():
    return find_config(
        'src/config/profiles/track2025.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/profiles/track2025.yaml')


def default_record_dir():
    return find_config('bagfile', str(Path.home() / 'bagfile'))


def base_nodes(vehicle_config, *, calibration_mode, use_joystick_control, image_topic):
    """Nodes common to every pipeline launch.

    - camera    : publishes /camera/image/compressed
    - actuator  : /control -> servo (joystick mode when use_joystick_control)
    - joystick  : /joystick (calibration_mode enables trim/accel edits)
    - monitor   : web dashboard (:5000), streams `image_topic` low-latency
    - battery   : /battery_status (feeds the monitor battery panel)
    """
    return [
        Node(package='camera', executable='camera_node', name='camera_node',
             output='screen',
             parameters=[{'vehicle_config_file': vehicle_config}]),
        Node(package='actuator', executable='actuator_node', name='actuator_node',
             output='screen',
             parameters=[{'use_joystick_control': use_joystick_control,
                          'vehicle_config_file': vehicle_config}]),
        Node(package='joystick', executable='joystick_node', name='joystick_node',
             output='screen',
             parameters=[{'calibration_mode': calibration_mode,
                          'vehicle_config_file': vehicle_config}]),
        Node(package='monitor', executable='monitor_node', name='monitor_node',
             output='screen',
             parameters=[{'vehicle_config_file': vehicle_config,
                          'image_topic': image_topic}]),
        Node(package='battery', executable='battery_node', name='battery_node',
             output='screen'),
    ]
