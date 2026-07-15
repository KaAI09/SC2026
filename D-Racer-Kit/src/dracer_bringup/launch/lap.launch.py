"""Launch 4/4 — lap: autonomous driving, nothing else. LAP-TIME MEASUREMENT.

⚠ ACTUATION. Same rules as `drive`: /control only when engaged (joystick A or
`engage:=true`), E-STOP = joystick X, wheels off the ground first.

`drive` minus the monitor minus the recorder. That is the entire difference, and it is
the entire point: a lap time is a measurement of the car, and a car that is also
JPEG-streaming a web dashboard and H.264-encoding an mp4 is not the car that will run
the timed lap. Both of those cost the hot path a frame's worth of CPU each, on the same
board perception runs on, so leaving them in makes the measured lap slower than the real
one -- which is the one failure mode a lap-time launch may not have.

What is NOT dropped:
  joystick   E-STOP (X) and engage (A) live here. Dropping it to save cycles would be
             removing the brakes to go faster.
  actuator   the only path to the servo.
  battery    ~free, and it is what tells you a slow lap was a sagging pack rather than
             a slow car.

Perception's debug render is off (`publish_debug: false`) and there is no subscriber
that could switch it back on.

    ros2 launch dracer_bringup lap.launch.py                        # engage stays false
    ros2 param set /control_node engage true                        # after wheels-off check

To watch or record a run, use `drive`. To compare two routes, run this one twice.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dracer_bringup.launch_common import (battery_node, core_nodes, default_camera_path,
                                          default_profile_path, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    camera = LaunchConfiguration('camera')
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
        DeclareLaunchArgument('engage', default_value='false',
                              description='start autonomously actuating (keep false; '
                                          'set true only after wheels-off checks)'),
        DeclareLaunchArgument('command_hz', default_value='30.0',
                              description='actuator servo write rate. Gates E-STOP / watchdog '
                                          'latency. Ceiling is the 50Hz PWM.'),
        DeclareLaunchArgument('publish_rate', default_value='30.0',
                              description='control_node /control rate. Should be >= the '
                                          'perception rate or commands are dropped.'),
        # THE THROTTLE GATE. Same switch as `drive`, same default (off), and it belongs here
        # for the same reason it belongs there: `perception_node` always DETECTS, and this only
        # says whether `control_node` ACTS. Without the argument a timed lap could not obey a
        # red light even if you wanted it to -- and "the lap launch cannot do missions" is a
        # decision nobody made, it was just a missing line.
        #
        # ⚠ ON = the car does not move until it is shown a GREEN. The gate starts STOPPED.
        DeclareLaunchArgument('mission_gate', default_value='false',
                              description='control_node gates throttle on GREEN/RED/MARK. '
                                          'ON = the car will not move until it sees a GREEN. '
                                          'Detection runs either way -- this is only whether '
                                          'the throttle listens.'),
        DeclareLaunchArgument('mission_config', default_value='',
                              description='venue mission YAML overriding MissionCfg. Rarely needed -- the '
                                          'gates measure PHYSICS (does it emit?), not venue '
                                          'thresholds; see MissionCfg.'),
        *core_nodes(
            vehicle_config,
            calibration_mode=False,
            use_joystick_control=False,     # actuator takes /control from control_node
            command_hz=ParameterValue(command_hz, value_type=float),
        ),
        battery_node(),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            # debug_view 'off': not merely unsubscribed, but unable to render at all. In a
            # timed lap "nobody happens to be watching" is not a guarantee; this is.
            parameters=[{'profile': profile, 'camera': camera,
                         'publish_debug': False, 'debug_view': 'off',
                         'use_mission': True,
                         'mission_config': LaunchConfiguration('mission_config')}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{'profile': profile,
                         'publish_rate': ParameterValue(publish_rate, value_type=float),
                         'engage': ParameterValue(engage, value_type=bool),
                         'use_mission': ParameterValue(
                             LaunchConfiguration('mission_gate'), value_type=bool)}],
        ),
    ])
