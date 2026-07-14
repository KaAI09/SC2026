"""Launch 1/4 — calibrate: camera setup + STEER_TRIM / ACCEL_RATIO.

The ONLY launch that edits the vehicle config, and the only one with
`calibration_mode` on. Core nodes + monitor + battery: no perception, no control, no
recorder. The operator (a) adjusts camera angle/height on the live web monitor and
(b) tunes steering_trim (joystick Y/B) and accel_ratio (joystick L1/R1). Those edits
are PERSISTED to vehicle_config.yaml, so every later launch loads them and none of
them needs to be able to change them -- a trim that moves mid-run would silently
change what the steering column of a recorded csv MEANS.

    ros2 launch dracer_bringup calibrate.launch.py

Also used for CAMERA (lens) calibration. Point the monitor at the capture script's
overlay so the operator sees live chessboard-corner detection while shooting:

    ros2 launch dracer_bringup calibrate.launch.py image_topic:=/calib/preview/compressed
    python3 scripts/capture_camera_calib.py --out ~/calib/intr --count 20
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from dracer_bringup.launch_common import (battery_node, core_nodes, monitor_node,
                                          vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    image_topic = LaunchConfiguration('image_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image/compressed',
            description='monitor web stream. Use /calib/preview/compressed to watch '
                        'capture_camera_calib.py corner detection live.'),
        *core_nodes(
            vehicle_config,
            calibration_mode=True,      # Y/B: steer trim, L1/R1: accel_ratio
            use_joystick_control=True,
        ),
        monitor_node(vehicle_config, image_topic),
        battery_node(),
    ])
