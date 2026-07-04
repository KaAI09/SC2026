from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_workspace_paths():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate), str(base_path / 'bagfile')
    root = Path('/home/topst/SC2026/D-Racer-Kit')
    return str(root / 'src' / 'config' / 'vehicle_config.yaml'), str(root / 'bagfile')


def generate_launch_description():
    """CLOSED-LOOP autonomous lane following (perception + control).

    ⚠ ACTUATION. lane_follow_node publishes /control ONLY when engage:=true.
       control_node runs in DIRECT mode; joystick_node provides the X-button
       E-STOP. START records the overlay + autonomous-command CSV.

    SAFETY LADDER:
      1) wheels OFF the ground, engage:=false -> bring up, verify overlay.
      2) `ros2 param set /lane_follow_node engage true` -> confirm the wheels
         steer with center_error and E-STOP (joystick X) cuts throttle.
      3) only then, low-speed track. Keep a hand on the E-STOP.

    Start disengaged by default. Example:
      ros2 launch control lane_follow.launch.py mode:=O2 throttle_base:=0.12
      ros2 param set /lane_follow_node engage true      # (after wheels-off check)
    """
    vehicle_config_path, bagfile_dir = get_workspace_paths()
    mode = LaunchConfiguration('mode')
    fp = {n: ParameterValue(LaunchConfiguration(n), value_type=float)
          for n in ('kp', 'kd', 'center_target', 'steer_max', 'slew_rate',
                    'throttle_base', 'conf_gate', 'trap_top_w', 'lane_width_default')}

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='O2'),
        DeclareLaunchArgument('control_timeout', default_value='0.5'),
        DeclareLaunchArgument('kp', default_value='0.5'),
        DeclareLaunchArgument('kd', default_value='0.1'),
        DeclareLaunchArgument('center_target', default_value='0.0'),
        DeclareLaunchArgument('steer_max', default_value='0.8'),
        DeclareLaunchArgument('slew_rate', default_value='0.15'),
        DeclareLaunchArgument('throttle_base', default_value='0.13'),
        DeclareLaunchArgument('conf_gate', default_value='0.4'),
        DeclareLaunchArgument('trap_top_w', default_value='-1.0'),
        DeclareLaunchArgument('lane_width_default', default_value='-1.0'),
        DeclareLaunchArgument('record_dir', default_value=bagfile_dir),
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config_path}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{
                'use_joystick_control': False,   # DIRECT mode: /control drives
                'control_timeout': ParameterValue(LaunchConfiguration('control_timeout'),
                                                   value_type=float),
                'vehicle_config_file': vehicle_config_path,
            }],
        ),
        Node(
            package='joystick', executable='joystick_node', name='joystick_node',
            output='screen',
            parameters=[{
                'calibration_mode': False,       # X button E-STOP backup
                'vehicle_config_file': vehicle_config_path,
            }],
        ),
        Node(
            package='opencv', executable='lane_follow_node', name='lane_follow_node',
            output='screen',
            parameters=[{
                'mode': mode,
                'engage': False,                 # start DISENGAGED
                'record_dir': LaunchConfiguration('record_dir'),
                **fp,
            }],
        ),
    ])
