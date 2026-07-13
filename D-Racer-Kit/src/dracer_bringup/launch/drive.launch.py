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

from dracer_bringup.launch_common import (base_nodes, default_camera_path,
                                          default_profile_path,
                                          default_record_dir, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    camera = LaunchConfiguration('camera')
    record_dir = LaunchConfiguration('record_dir')
    engage = LaunchConfiguration('engage')
    command_hz = LaunchConfiguration('command_hz')
    publish_rate = LaunchConfiguration('publish_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value=default_camera_path(),
            description='camera calibration YAML -> metric BEV. REQUIRED: the pipeline '
                        'raises ValueError without it (front-view path was removed)'),
        DeclareLaunchArgument('profile', default_value=default_profile_path(),
                              description='driving profile YAML ([perception] + [control])'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir(),
                              description='directory for recorder mp4/csv output'),
        DeclareLaunchArgument('engage', default_value='false',
                              description='start autonomously actuating (keep false; '
                                          'set true only after wheels-off checks)'),
        # Both rates are launch args, NOT constants, because they are set once at node
        # construction (each creates a timer) and so cannot be changed with `ros2 param set`
        # on a running car.
        DeclareLaunchArgument('command_hz', default_value='30.0',
                              description='actuator servo write rate. Gates E-STOP / watchdog '
                                          'latency. Ceiling is the 50Hz PWM.'),
        DeclareLaunchArgument('publish_rate', default_value='30.0',
                              description='control_node /control rate. Should be >= the '
                                          'perception rate or commands are dropped.'),
        *base_nodes(
            vehicle_config,
            calibration_mode=False,
            use_joystick_control=False,     # actuator takes /control from control_node
            # RAW camera, not the debug panel. Every subscriber of /lane/debug/compressed
            # forces perception to composite + JPEG-encode a 4-panel strip on the hot path;
            # the monitor only needs to show the driver the road.
            image_topic='/camera/image/compressed',
            command_hz=ParameterValue(command_hz, value_type=float),
        ),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            parameters=[{'profile': profile, 'camera': camera}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{'profile': profile,
                         'publish_rate': ParameterValue(publish_rate, value_type=float),
                         'engage': ParameterValue(engage, value_type=bool)}],
        ),
        # Record the RAW camera + the LaneState csv, NOT the annotated panel. Together they
        # reconstruct the panel exactly, offline, on a machine with cycles to spare — so the
        # car never pays for rendering while it drives. With no panel subscriber left,
        # perception's own subscriber check turns the overlay off by itself.
        # To review the live overlay instead: image_topic:=/lane/debug/compressed
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            parameters=[{'record_dir': record_dir,
                         'image_topic': '/camera/image/compressed',
                         'raw_topic': ''}],
        ),
    ])
