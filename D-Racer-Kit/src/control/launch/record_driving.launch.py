from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def get_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return str(Path('/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml'))


def generate_launch_description():
    """Manual (joystick) driving with the camera running so that the
    joystick START button can record practice-track video into bagfiles.

    - camera_node publishes /camera/image/compressed (included in the bag).
    - control_node runs in joystick mode (use_joystick_control=True).
    - joystick_node START button toggles `ros2 bag record -a`; each
      START->STOP cycle creates a new bag_<timestamp> directory, so a
      single drive can be split into multiple recordings.
    """
    vehicle_config_path = get_vehicle_config_path()

    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {
                    'use_joystick_control': True,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
        Node(
            package='joystick',
            executable='joystick_node',
            name='joystick_node',
            output='screen',
            parameters=[
                {
                    'calibration_mode': True,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
    ])
