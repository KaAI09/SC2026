"""Pipeline launch 2 — record: manual drive + RAW camera recording.

Base nodes + recorder capturing the RAW camera stream (not the perception
overlay) so offline replay (offline/panel_replay.py) sees unannotated
frames. No perception -> raw IS the main stream, so the recorder writes only
raw/ + csv/ (no panel/). Joystick START toggles each recording session.

    ros2 launch dracer_bringup record.launch.py
    ros2 launch dracer_bringup record.launch.py record_dir:=$HOME/recorder
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dracer_bringup.launch_common import base_nodes, default_record_dir, vehicle_config_path


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    record_dir = LaunchConfiguration('record_dir')

    return LaunchDescription([
        DeclareLaunchArgument('record_dir', default_value=default_record_dir(),
                              description='directory for recorder mp4/csv output'),
        *base_nodes(
            vehicle_config,
            calibration_mode=False,
            use_joystick_control=True,
            image_topic='/camera/image/compressed',   # no perception -> raw camera view
        ),
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            parameters=[{'record_dir': record_dir,
                         'image_topic': '/camera/image/compressed',  # RAW, not overlay
                         'name_prefix': 'raw'}],
        ),
    ])
