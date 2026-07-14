"""Shared launch helpers for the D-Racer bring-up pipeline.

`find_config()` locates repo config by walking parent dirs (works with
`--symlink-install`; falls back to the D3-G default path otherwise).

The node helpers are SPLIT rather than bundled into one `base_nodes()`, because
`lap` exists. A lap-time run has to be able to drop the monitor -- a Flask server
that JPEG-streams a frame at camera rate -- WITHOUT dropping the joystick, which
carries E-STOP. One bundle made every launch pay for every node, so the leanest
launch was as heavy as the heaviest.

  core_nodes()    camera + actuator + joystick. NEVER omit: the joystick is the
                  E-STOP (X) / engage (A) / record (START) path, and the actuator
                  is the only thing that reaches the servo.
  monitor_node()  web dashboard (:5000). OPTIONAL, and the expensive one.
  battery_node()  /battery_status. OPTIONAL, and nearly free.
"""
from pathlib import Path

from launch_ros.actions import Node


def find_config(rel, fallback):
    for base in Path(__file__).resolve().parents:
        cand = base / rel
        if cand.exists():
            return str(cand)
    return fallback


def vehicle_config_path():
    return find_config(
        'src/config/vehicle_config.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml')


def default_profile_path():
    return find_config(
        'src/config/profiles/track2025.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/profiles/track2025.yaml')


def default_camera_path():
    """Camera calibration -> metric BEV. TRACK-INDEPENDENT (reuse at any venue);
    re-run `offline/calibrate.py --ground` if the camera mount was moved."""
    return find_config(
        'src/config/camera.yaml',
        '/home/topst/SC2026/D-Racer-Kit/src/config/camera.yaml')


def default_record_dir():
    """Recorder output root. Sessions land in <root>/{raw,csv}/ (recorder_node).

    NOT via find_config(): a repo-relative search for 'recorder' would resolve to the
    SOURCE PACKAGE `src/recorder/` and write recordings into the source tree. Recording
    output is machine-local and git-untracked, so anchor it at $HOME and let a launch
    argument override it.
    """
    return str(Path.home() / 'recorder')


def core_nodes(vehicle_config, *, calibration_mode, use_joystick_control,
               command_hz=30.0):
    """The three nodes the car can neither move nor stop without.

    - camera    : publishes /camera/image/compressed
    - actuator  : /control -> servo (joystick mode when use_joystick_control)
    - joystick  : /joystick -- E-STOP (X), engage (A), record toggle (START).
                  `calibration_mode` additionally enables the trim/accel EDITS
                  (Y/B, L1/R1). Driving works either way; only the edits are gated.

    `command_hz` is a LAUNCH ARGUMENT (not baked in): it creates a timer at node
    construction, so it cannot be changed with `ros2 param set` on a running car.
    """
    return [
        Node(package='camera', executable='camera_node', name='camera_node',
             output='screen',
             parameters=[{'vehicle_config_file': vehicle_config}]),
        # command_hz 30 (the actuator node's own default is still 10). The actuator timer is
        # a stateless zero-order hold -- it just re-writes the latest /control to the servo --
        # so its rate is NOT coupled to perception's: a slow input simply gets written more
        # than once, which is identical to writing it once. What the rate DOES gate is every
        # safety path, because E-STOP, the /control dead-man and the neutral fallback all
        # reach the servo only on this tick. At 10Hz an E-STOP took up to 100ms to arrive.
        # The PCA9685 runs a 50Hz PWM, so 50 is the physical ceiling (the servo cannot consume
        # a new pulse width faster than one per period); 30 sits under it at ~7% I2C load.
        Node(package='actuator', executable='actuator_node', name='actuator_node',
             output='screen',
             parameters=[{'use_joystick_control': use_joystick_control,
                          'command_hz': command_hz,
                          'vehicle_config_file': vehicle_config}]),
        Node(package='joystick', executable='joystick_node', name='joystick_node',
             output='screen',
             parameters=[{'calibration_mode': calibration_mode,
                          'vehicle_config_file': vehicle_config}]),
    ]


def monitor_node(vehicle_config, image_topic):
    """Web dashboard (:5000), streaming `image_topic`.

    The cost is not the dashboard, it is what the SUBSCRIPTION switches on: perception
    renders and JPEG-encodes its debug image only while something is listening
    (`get_subscription_count() > 0`). Point this at the raw camera and that cost is
    zero; point it at a debug topic and you have asked for it.
    """
    return Node(package='monitor', executable='monitor_node', name='monitor_node',
                output='screen',
                parameters=[{'vehicle_config_file': vehicle_config,
                             'image_topic': image_topic}])


def battery_node():
    """/battery_status. Cheap enough to keep even in `lap`: a lap that got slower because
    the pack sagged is not a slower car, and only this node can tell the two apart."""
    return Node(package='battery', executable='battery_node', name='battery_node',
                output='screen')
