import os
from pathlib import Path

import rclpy
from rclpy.node import Node
import yaml

from dracer_msgs.msg import Control
from dracer_msgs.msg import Joystick
from topst_utils.d3racer import D3Racer


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml'


class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')

        # ROS parameters
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('pca9685_addr', 0x40)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('use_joystick_control', False)
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('command_hz', 10.0)
        # Dead-man watchdog: in direct (/control) mode, auto-stop when the
        # command stream stalls. <= 0 disables it. Ignored in joystick mode.
        self.declare_parameter('control_timeout', 0.5)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        pca9685_addr = int(self.get_parameter('pca9685_addr').value)
        steering_channel = int(self.get_parameter('steering_channel').value)
        throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        self.use_joystick_control = bool(self.get_parameter('use_joystick_control').value)
        joystick_topic = str(self.get_parameter('joystick_topic').value)
        control_topic = str(self.get_parameter('control_topic').value)
        command_hz = float(self.get_parameter('command_hz').value)
        if command_hz <= 0.0:
            raise ValueError('command_hz must be greater than 0')
        self.control_timeout = float(self.get_parameter('control_timeout').value)

        self.command_hz = command_hz
        self.steer_trim = self.load_steer_trim()
        # Timestamp of the most recent /control message (direct mode watchdog).
        self.last_control_time = None

        self.d3_racer = D3Racer(
            i2c_bus=i2c_bus,
            pca9685_addr=pca9685_addr,
            steering_channel=steering_channel,
            throttle_channel=throttle_channel,
        )

        self.get_logger().info(
            'd3_racer configured:\n'
            f'  i2c_bus={i2c_bus}\n'
            f'  pca9685_addr=0x{pca9685_addr:02X}\n'
            f'  steering_channel={steering_channel}\n'
            f'  throttle_channel={throttle_channel}\n'
            f'  steer_trim={self.steer_trim}\n'
            f'  use_joystick_control={self.use_joystick_control}\n'
            f'  joystick_topic={joystick_topic}\n'
            f'  control_topic={control_topic}\n'
            f'  command_hz={self.command_hz}\n'
            f'  control_timeout={self.control_timeout}\n'
            f'  vehicle_config_file={self.vehicle_config_file}'
        )

        self.throttle = 0.0
        self.steering = self.steer_trim
        self.e_stop_active = False

        # Control inputs
        self.create_subscription(
            Joystick,
            joystick_topic,
            self.joystick_callback,
            10,
        )
        self.create_subscription(
            Control,
            control_topic,
            self.control_callback,
            10,
        )

        # Command output loop
        self.timer = self.create_timer(1.0 / self.command_hz, self.timer_callback)

    def timer_callback(self):
        if self.e_stop_active:
            self.apply_actuation(self.steering, 0.0)
            return

        if self.is_control_stale():
            # Command stream stalled in direct mode: hold steering neutral and
            # cut throttle until fresh /control messages resume.
            self.apply_actuation(self.steer_trim, 0.0)
            return

        self.apply_actuation(self.steering, self.throttle)

    def is_control_stale(self):
        # Watchdog only applies to direct (/control) mode. Joystick mode
        # refreshes self.throttle continuously, so it never goes stale.
        if self.use_joystick_control or self.control_timeout <= 0.0:
            return False
        if self.last_control_time is None:
            # No /control command received yet: stay in the safe neutral state.
            return True
        elapsed = (self.get_clock().now() - self.last_control_time).nanoseconds / 1e9
        return elapsed > self.control_timeout

    def apply_actuation(self, steering, throttle):
        self.d3_racer.set_steering_percent(float(steering))
        self.d3_racer.set_throttle_percent(float(throttle))

    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            self.engage_e_stop()
            return

        if self.e_stop_active or not self.use_joystick_control:
            return

        self.steering = float(msg.control_msg.steering)
        self.throttle = float(msg.control_msg.throttle)

    def control_callback(self, msg: Control):
        if self.e_stop_active or self.use_joystick_control:
            return

        # The autonomous command is symmetric about 0 (0 = straight). Add the servo
        # trim so 0 maps to mechanical-straight, matching manual mode (which sends
        # axis + trim); otherwise every command sits STEER_TRIM off center. Clamp.
        raw = float(msg.steering) + self.steer_trim
        self.steering = max(-1.0, min(1.0, raw))
        # The command and the trim share ONE [-1, 1] servo budget, so a controller whose
        # steer_max exceeds 1 - |trim| loses authority on one side only, and loses it
        # silently: the servo just stops moving while the controller keeps asking for more.
        # That reads as a one-sided understeer and sends you hunting through the gains.
        # control_node's steer_max is set to 1 - |STEER_TRIM| for exactly this reason; if
        # that ever drifts, say so rather than clipping in the dark.
        if abs(raw - self.steering) > 1e-6:
            self.get_logger().warning(
                f'steering saturated: {raw:+.3f} -> {self.steering:+.3f} '
                f'(cmd {msg.steering:+.3f} + trim {self.steer_trim:+.3f}). '
                f'steer_max 를 {1.0 - abs(self.steer_trim):.2f} 이하로 낮춰라 — '
                '지금 한쪽 조향 권한만 깎이고 있다.',
                throttle_duration_sec=2.0)
        self.throttle = float(msg.throttle)
        self.last_control_time = self.get_clock().now()

    def engage_e_stop(self):
        if self.e_stop_active:
            return

        self.e_stop_active = True
        self.throttle = 0.0
        self.apply_actuation(self.steering, 0.0)
        self.get_logger().warning('E-STOP engaged. Ignoring incoming throttle commands.')

    def load_steer_trim(self):
        if not os.path.exists(self.vehicle_config_file):
            return 0.0

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return 0.0

        return float(config_data.get('STEER_TRIM', 0.0))

    def destroy_node(self):
        try:
            if hasattr(self, 'd3_racer') and self.d3_racer is not None:
                self.apply_actuation(self.steer_trim, 0.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
