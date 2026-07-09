"""Pipeline launch 1 — calibrate: camera setup + STEER_TRIM / ACCEL_RATIO.

1st manual run. Base nodes only (no perception/control/recorder). The operator
(a) adjusts camera angle/height on the live web monitor and (b) tunes
steering_trim (joystick Y/B) and accel_ratio (joystick L1/R1) — calibration_mode
edits PERSISTED to vehicle_config.yaml so every later launch loads them.

    ros2 launch dracer_bringup calibrate.launch.py
"""
from launch import LaunchDescription

from dracer_bringup.launch_common import base_nodes, vehicle_config_path


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    return LaunchDescription(base_nodes(
        vehicle_config,
        calibration_mode=True,          # Y/B: steer trim, L1/R1: accel_ratio
        use_joystick_control=True,
        image_topic='/camera/image/compressed',   # no perception -> raw camera view
    ))
