"""Pipeline launch 3 — perceive: manual drive + live perception + recording.

No autonomous actuation (actuator in joystick mode; the human drives). Base
nodes + perception_node (publishes /lane/state + the debug overlay the monitor
streams) + recorder (on joystick START: panel/ overlay mp4 + raw/ camera mp4 +
csv/ log, one shared basename). calibration_mode stays on so
trim/accel can still be tuned. Tune perception live via `ros2 param set
/perception_node <field> <value>` and watch the monitor (:5000).

Use for: perception validation on the real track and driving-data collection.

    ros2 launch dracer_bringup perceive.launch.py
    ros2 launch dracer_bringup perceive.launch.py profile:=$HOME/SC2026/D-Racer-Kit/src/config/profiles/track2025.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dracer_bringup.launch_common import (base_nodes, default_profile_path,
                                          default_record_dir, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    record_dir = LaunchConfiguration('record_dir')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile_path(),
                              description='driving profile YAML ([perception] applied)'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir(),
                              description='directory for recorder mp4/csv output'),
        *base_nodes(
            vehicle_config,
            calibration_mode=True,          # allow trim/accel tuning during validation
            use_joystick_control=True,
            image_topic='/lane/debug/compressed',   # perception running -> debug overlay
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
    ])
