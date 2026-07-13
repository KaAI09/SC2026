"""Pipeline launch 1 — calibrate: camera setup + STEER_TRIM / ACCEL_RATIO.

1st manual run. Base nodes only (no perception/control/recorder). The operator
(a) adjusts camera angle/height on the live web monitor and (b) tunes
steering_trim (joystick Y/B) and accel_ratio (joystick L1/R1) — calibration_mode
edits PERSISTED to vehicle_config.yaml so every later launch loads them.

    ros2 launch dracer_bringup calibrate.launch.py

Also used for CAMERA (lens) calibration. Point the monitor at the capture script's
overlay so the operator sees live chessboard-corner detection while shooting:

    ros2 launch dracer_bringup calibrate.launch.py image_topic:=/calib/preview/compressed
    python3 scripts/capture_camera_calib.py --out ~/calib/intr --count 20
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from dracer_bringup.launch_common import base_nodes, vehicle_config_path


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    image_topic = LaunchConfiguration('image_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image/compressed',
            description='monitor web stream. Use /calib/preview/compressed to watch '
                        'capture_camera_calib.py corner detection live.'),
        *base_nodes(
            vehicle_config,
            calibration_mode=True,      # Y/B: steer trim, L1/R1: accel_ratio
            use_joystick_control=True,
            image_topic=image_topic,
        ),
    ])
