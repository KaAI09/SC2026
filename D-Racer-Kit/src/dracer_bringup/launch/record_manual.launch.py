"""Track-test pipeline — Launch 2 (manual drive + raw camera recording).

2nd manual run, with the camera condition already fixed by Launch 1. Manual
driving (control + joystick) + recorder; NO perception. The recorder captures the
RAW camera stream (/camera/image/compressed, not the perception overlay) so the
offline perception work sees unannotated frames. Joystick START toggles each
recording session.

Produces drive video for offline perception (pipeline steps 4-6): the confirmed
7-label BEV method (offline/lane7_probe.py). The old front-view track-condition
tool (track_analyze.py) was removed.

    ros2 launch dracer_bringup record_manual.launch.py
    ros2 launch dracer_bringup record_manual.launch.py record_dir:=$HOME/bagfile
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
    default_record_dir = _find('bagfile', str(Path.home() / 'bagfile'))

    record_dir = LaunchConfiguration('record_dir')

    return LaunchDescription([
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
            parameters=[{'calibration_mode': False,   # camera already calibrated in Launch 1
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            parameters=[{'record_dir': record_dir,
                         'image_topic': '/camera/image/compressed',  # RAW, not overlay
                         'name_prefix': 'raw'}],
        ),
        Node(
            package='monitor', executable='monitor_node', name='monitor_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config,
                         # no perception here -> show the RAW camera feed
                         'image_topic': '/camera/image/compressed'}],
        ),
    ])
