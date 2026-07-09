"""Online AUTONOMOUS bring-up: perception -> driving -> /control -> actuator.

⚠ ACTUATION. control_node publishes /control ONLY when engage:=true. Bring up
with the wheels OFF THE GROUND first, verify perception + steering direction,
then set engage. E-STOP = joystick X button.

Pipeline: camera -> perception_node (/lane/state) -> control_node (/control,
gated) -> control_node (PWM). recorder_node logs mp4 + csv on START. Both
perception and control load the offline-selected profile.

    ros2 launch control online_auto.launch.py                 # engage stays false
    ros2 param set /control_node engage true                  # after wheels-off check
    ros2 launch control online_auto.launch.py profile:=/abs/path/track2025.yaml
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    engage = LaunchConfiguration('engage')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile,
                              description='driving profile YAML (perception + control)'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir,
                              description='directory for recorder mp4/csv output'),
        DeclareLaunchArgument('engage', default_value='false',
                              description='start autonomously actuating (keep false; '
                                          'set true only after wheels-off checks)'),
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='actuator', executable='actuator_node', name='actuator_node',
            output='screen',
            parameters=[{'use_joystick_control': False,
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='joystick', executable='joystick_node', name='joystick_node',
            output='screen',
            parameters=[{'calibration_mode': False,
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            parameters=[{'profile': profile}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{'profile': profile,
                         'engage': ParameterValue(engage, value_type=bool)}],
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
