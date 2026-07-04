from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_workspace_paths():
    """Return (vehicle_config_path, bagfile_dir) by walking up to the src root."""
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate), str(base_path / 'bagfile')
    root = Path('/home/topst/SC2026/D-Racer-Kit')
    return str(root / 'src' / 'config' / 'vehicle_config.yaml'), str(root / 'bagfile')


def generate_launch_description():
    """Manual (joystick) driving + live lane-detection overlay.

    Drive the car with the joystick on the real track while watching lane
    detection on /lane/debug/compressed (e.g. rqt_image_view / web_video_server).

    - camera_node        publishes /camera/image/compressed
    - control_node       joystick mode (use_joystick_control=True)  <- manual only
    - joystick_node      manual steering/throttle + X-button E-STOP
    - lane_detect_node   PERCEPTION ONLY; publishes overlay, never touches /control

    SAFETY: this launch performs NO autonomous actuation. Steering/throttle come
    solely from the joystick. lane_detect_node does not publish /control.
    """
    vehicle_config_path, bagfile_dir = get_workspace_paths()
    mode = LaunchConfiguration('mode')
    record_dir = LaunchConfiguration('record_dir')

    # Live-tunable overrides (sentinels < 0 mean "use the preset default").
    # e.g. ros2 launch ... mode:=O2 trap_top_w:=0.85 orange_s_min:=80
    fparam = {n: ParameterValue(LaunchConfiguration(n), value_type=float)
              for n in ('trap_top_w', 'trap_bot_w', 'lane_width_default', 'roi_top_frac')}
    iparam = {n: ParameterValue(LaunchConfiguration(n), value_type=int)
              for n in ('orange_h_lo', 'orange_h_hi', 'orange_s_min', 'orange_v_min')}

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='M2',
                              description='lane_core preset: M1..M6 / O1..O3'),
        DeclareLaunchArgument('record_dir', default_value=bagfile_dir,
                              description='where START-button overlay MP4s are saved'),
        DeclareLaunchArgument('trap_top_w', default_value='-1.0',
                              description='trapezoid ROI top width (fraction of W); widen for 2-line sections'),
        DeclareLaunchArgument('trap_bot_w', default_value='-1.0',
                              description='trapezoid ROI bottom width (fraction of W)'),
        DeclareLaunchArgument('lane_width_default', default_value='-1.0',
                              description='single-line fallback lane width (fraction of W)'),
        DeclareLaunchArgument('roi_top_frac', default_value='-1.0',
                              description='fraction of image height cut from the top'),
        DeclareLaunchArgument('orange_h_lo', default_value='-1'),
        DeclareLaunchArgument('orange_h_hi', default_value='-1'),
        DeclareLaunchArgument('orange_s_min', default_value='-1'),
        DeclareLaunchArgument('orange_v_min', default_value='-1'),
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config_path}],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen',
            parameters=[{
                'use_joystick_control': True,
                'vehicle_config_file': vehicle_config_path,
            }],
        ),
        Node(
            package='joystick', executable='joystick_node', name='joystick_node',
            output='screen',
            parameters=[{
                'calibration_mode': False,
                'vehicle_config_file': vehicle_config_path,
            }],
        ),
        Node(
            package='opencv', executable='lane_detect_node', name='lane_detect_node',
            output='screen',
            parameters=[{'mode': mode, 'record_dir': record_dir, **fparam, **iparam}],
        ),
    ])
