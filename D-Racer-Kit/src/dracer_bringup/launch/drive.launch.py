"""Pipeline launch 4 — drive: autonomous perception -> control -> /control.

⚠ ACTUATION. control_node publishes /control ONLY when engaged (joystick A
button OR `engage:=true` / `ros2 param set /control_node engage true`). Bring up
with the wheels OFF THE GROUND first, verify perception + steering direction,
then engage. E-STOP = joystick X.

Base nodes (actuator NOT in joystick mode) + perception_node + control_node +
recorder (on joystick START: panel/ overlay mp4 + raw/ camera mp4 + csv/ log, one
shared basename). Tune live via `ros2 param set /perception_node <field>` and
`/control_node <gain>`; watch the monitor (:5000).

    ros2 launch dracer_bringup drive.launch.py                    # engage stays false
    ros2 param set /control_node engage true                      # after wheels-off check
    ros2 launch dracer_bringup drive.launch.py profile:=$HOME/SC2026/D-Racer-Kit/src/config/profiles/track2025.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dracer_bringup.launch_common import (base_nodes, default_profile_path,
                                          default_record_dir, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    record_dir = LaunchConfiguration('record_dir')
    engage = LaunchConfiguration('engage')

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile_path(),
                              description='driving profile YAML ([perception] + [control])'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir(),
                              description='directory for recorder mp4/csv output'),
        DeclareLaunchArgument('engage', default_value='false',
                              description='start autonomously actuating (keep false; '
                                          'set true only after wheels-off checks)'),
        *base_nodes(
            vehicle_config,
            calibration_mode=False,
            use_joystick_control=False,     # actuator takes /control from control_node
            image_topic='/lane/debug/compressed',   # perception running -> debug overlay
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
    ])
