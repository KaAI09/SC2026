"""Launch 3/4 — drive: autonomous perception -> control -> /control. TUNING & VALIDATION.

⚠ ACTUATION. control_node publishes /control ONLY when engaged (joystick A button OR
`engage:=true` / `ros2 param set /control_node engage true`). Bring up with the wheels
OFF THE GROUND first, verify perception + steering direction, then engage.
E-STOP = joystick X.

This is the launch you WATCH. Core nodes (actuator NOT in joystick mode) + perception
+ control + monitor + recorder. Tune live via `ros2 param set /perception_node <field>`
and `/control_node <gain>`.

`monitor_topic` is what separates this from `lap`. Default is the RAW camera, which
costs nothing. Point it at /lane/debug/compressed and perception's own subscriber check
switches the debug render on -- that is a real cost on the hot path, and it is why `lap`
has no monitor at all.

⚠ The debug panel is not just CPU, it is BANDWIDTH, and the two fail differently. The
render costs frames; the stream costs LATENCY, and the MJPEG stream does not shed frames
the link cannot carry -- they queue, so the lag grows instead of levelling off. On a
congested venue Wi-Fi the panel will be seconds behind while `ros2 topic hz /lane/state`
still says 30. Watch the topic, not the picture.

Recording is the joystick START toggle. Only the RAW camera + the csv are written; the
panel is reconstructed offline (`offline/panel_replay.py`), so the car never pays for
rendering a video it is not watching:

    <record_dir>/raw/drive_<stamp>.mp4    RAW camera, never annotated
    <record_dir>/csv/drive_<stamp>.csv    per-frame LaneState + autonomous command

    ros2 launch dracer_bringup drive.launch.py                      # engage stays false
    ros2 param set /control_node engage true                        # after wheels-off check
    ros2 launch dracer_bringup drive.launch.py monitor_topic:=/lane/debug/compressed
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dracer_bringup.launch_common import (battery_node, core_nodes, default_camera_path,
                                          default_profile_path, default_record_dir,
                                          monitor_node, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    camera = LaunchConfiguration('camera')
    record_dir = LaunchConfiguration('record_dir')
    monitor_topic = LaunchConfiguration('monitor_topic')
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
        # RAW camera by DEFAULT, and this is the safe default rather than the useful one.
        # The debug panel is what you want to look at -- and it is also the one thing in
        # this launch that can push the car's frame budget and the venue Wi-Fi over at the
        # same time. Ask for it explicitly, on a link you have checked:
        #
        #     ros2 launch dracer_bringup drive.launch.py monitor_topic:=/lane/debug/compressed
        DeclareLaunchArgument('monitor_topic',
                              default_value='/camera/image/compressed',
                              description='what the web monitor streams. RAW costs nothing '
                                          '(perception renders only when the debug topic has a '
                                          'subscriber). Point at /lane/debug/compressed to see '
                                          'the panel -- and to pay for it.'),
        DeclareLaunchArgument('debug_view', default_value='bev',
                              description="bev = BEV(lanes+corridors) | camera(mission boxes), "
                                          '552x240. panels = the old 4-panel, 1280x240. '
                                          'off = never render.'),
        DeclareLaunchArgument('debug_scale', default_value='1.0',
                              description='debug panel upscale before JPEG. 1.0 = native. 2.0 '
                                          'quadruples the bytes on the wire for pixels the '
                                          'browser would have scaled for free.'),
        DeclareLaunchArgument('mission_config', default_value='',
                              description='venue mission YAML from scripts/mission_tune.py'),
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
        *core_nodes(
            vehicle_config,
            calibration_mode=False,
            use_joystick_control=False,     # actuator takes /control from control_node
            command_hz=ParameterValue(command_hz, value_type=float),
        ),
        monitor_node(vehicle_config, monitor_topic),
        battery_node(),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            parameters=[{'profile': profile, 'camera': camera,
                         'use_mission': True,
                         'debug_view': LaunchConfiguration('debug_view'),
                         'debug_scale': ParameterValue(LaunchConfiguration('debug_scale'),
                                                       value_type=float),
                         'mission_config': LaunchConfiguration('mission_config')}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{'profile': profile,
                         'publish_rate': ParameterValue(publish_rate, value_type=float),
                         'engage': ParameterValue(engage, value_type=bool)}],
        ),
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            parameters=[{'record_dir': record_dir,
                         'image_topic': '/camera/image/compressed',
                         'raw_topic': '',
                         'name_prefix': 'drive'}],
        ),
    ])
