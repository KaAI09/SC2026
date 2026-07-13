from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return str(Path('/home/topst/D-Racer/src/config/vehicle_config.yaml'))


def generate_launch_description():
    """Bring-up for the interactive steering/throttle test.

    - control_node runs in DIRECT mode (use_joystick_control=False) so that
      scripts/actuation_test.py can publish raw /control commands covering the
      full range (including reverse). A control_timeout watchdog stops the
      vehicle if the command stream stalls.
    - joystick_node runs only as a hardware E-STOP backup: when
      use_joystick_control=False the joystick X button still latches E-STOP,
      while its steering/throttle inputs are ignored.

    SAFETY: start every test with the wheels off the ground.
    """
    vehicle_config_path = get_vehicle_config_path()
    control_timeout = LaunchConfiguration('control_timeout')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_timeout',
            default_value='0.5',
            description='Seconds without a /control message before the '
                        'vehicle is auto-stopped (direct mode only).',
        ),
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {
                    'use_joystick_control': False,
                    'control_timeout': control_timeout,
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
                    'calibration_mode': False,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
    ])
