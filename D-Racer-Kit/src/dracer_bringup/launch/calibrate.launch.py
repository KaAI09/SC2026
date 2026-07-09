"""Track-test pipeline — Launch 1 (calibration & setup).

1st manual run. Brings up camera + manual driving (control + joystick) + the live
web monitor (+ battery publisher feeding the monitor battery panel) so the
operator can (a) adjust camera angle/height watching the live feed and (b) tune
steering_trim (joystick Y/B) and accel_ratio (joystick L1/R1). Both are
calibration_mode edits and are PERSISTED to vehicle_config.yaml
(STEER_TRIM + ACCEL_RATIO), so every later launch loads them automatically.

The slim monitor shows only the live camera, storage, and battery — this is the
only pipeline launch that runs the web monitor, so battery_node runs here too.

No perception, no recorder, no autonomous actuation — setup only.

    ros2 launch control calibrate.launch.py
"""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _find(rel, fallback):
    for base in Path(__file__).resolve().parents:
        cand = base / rel
        if cand.exists():
            return str(cand)
    return fallback


def generate_launch_description():
    vehicle_config = _find(
        'src/config/vehicle_config.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml')

    return LaunchDescription([
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='actuator', executable='actuator_node', name='actuator_node',
            output='screen',
            parameters=[{'use_joystick_control': True,
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='joystick', executable='joystick_node', name='joystick_node',
            output='screen',
            parameters=[{'calibration_mode': True,   # Y/B: steer trim, L1/R1: accel_ratio
                         'vehicle_config_file': vehicle_config}],
        ),
        Node(
            package='monitor', executable='monitor_node', name='monitor_node',
            output='screen',
            parameters=[{'vehicle_config_file': vehicle_config,
                         # calibration has no perception -> show the RAW camera feed
                         # (overrides any vehicle_config IMAGE_TOPIC drift to /lane/debug)
                         'image_topic': '/camera/image/compressed'}],
        ),
        Node(
            package='battery', executable='battery_node', name='battery_node',
            output='screen',
            # publishes /battery_status -> monitor battery panel
        ),
    ])
