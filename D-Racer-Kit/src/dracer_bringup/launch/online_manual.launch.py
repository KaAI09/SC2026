"""Online MANUAL bring-up: joystick-driven car + live perception + recording.

No autonomous actuation. control_node runs in joystick mode (the human drives);
perception_node runs the offline-selected profile and publishes /lane/state +
a debug overlay; recorder_node writes mp4 + csv on the joystick START button.

Use for: perception validation on the real track and driving-data collection.

    ros2 launch control online_manual.launch.py
    ros2 launch control online_manual.launch.py profile:=/abs/path/track2025.yaml
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find(rel, fallback):
    for base in Path(__file__).resolve().parents:
        cand = base / rel
        if cand.exists():
            return str(cand)
    return fallback


def generate_launch_description():
    vehicle_config = _find(
        'src/config/vehicle_config.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml')
    default_profile = _find(
        'src/config/profiles/track2025.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/profiles/track2025.yaml')
    default_record_dir = _find('bagfile', str(Path.home() / 'bagfile'))

    profile = LaunchConfiguration('profile')
    record_dir = LaunchConfiguration('record_dir')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile,
                              description='driving profile YAML (perception section applied)'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir,
                              description='directory for recorder mp4/csv output'),
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='actuator', executable='actuator_node', name='actuator_node',
            output='screen',
            parameters=[{'use_joystick_control': True,
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='joystick', executable='joystick_node', name='joystick_node',
            output='screen',
            parameters=[{'calibration_mode': True,
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            parameters=[{'profile': profile}],
        ),
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            parameters=[{'record_dir': record_dir}],
        ),
        Node(
            package='monitor', executable='monitor_node', name='monitor_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config,
                         # perception running -> stream the debug overlay (tuning view)
                         'image_topic': '/lane/debug/compressed'}],
        ),
    ])
