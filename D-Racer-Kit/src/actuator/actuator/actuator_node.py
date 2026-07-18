import os
from pathlib import Path

import rclpy
from rclpy.node import Node
import yaml

from dracer_msgs.msg import Control
from dracer_msgs.msg import Joystick
from topst_utils.d3racer import D3Racer, ServoCalib


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
        self.servo = self.load_servo_calib()
        # Timestamp of the most recent /control message (direct mode watchdog).
        self.last_control_time = None

        self.d3_racer = D3Racer(
            i2c_bus=i2c_bus,
            pca9685_addr=pca9685_addr,
            steering_channel=steering_channel,
            throttle_channel=throttle_channel,
            steering=self.servo,
        )

        self.get_logger().info(
            'd3_racer configured:\n'
            f'  i2c_bus={i2c_bus}\n'
            f'  pca9685_addr=0x{pca9685_addr:02X}\n'
            f'  steering_channel={steering_channel}\n'
            f'  throttle_channel={throttle_channel}\n'
            f'  servo: center={self.servo.center_us}us span={self.servo.span_us}us '
            f'range={self.servo.min_us}~{self.servo.max_us}us\n'
            f'  use_joystick_control={self.use_joystick_control}\n'
            f'  joystick_topic={joystick_topic}\n'
            f'  control_topic={control_topic}\n'
            f'  command_hz={self.command_hz}\n'
            f'  control_timeout={self.control_timeout}\n'
            f'  vehicle_config_file={self.vehicle_config_file}'
        )

        self.throttle = 0.0
        # 0 IS straight now -- the trim lives in the servo's centre, not in the command.
        self.steering = 0.0
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
            self.apply_actuation(0.0, 0.0)
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

        # Live trim: the joystick owns the Y/B buttons, the actuator owns the servo. The
        # centre rides along on the message (same as accel_ratio) so a trim adjustment moves
        # the wheels NOW -- a calibration you must restart to see is not a calibration.
        # Applies in BOTH modes: the servo centre is a property of the car, not of who is
        # driving it.
        c = float(msg.steer_center_us)
        if c > 0.0 and abs(c - self.servo.center_us) > 1e-6:
            old_eff = self.servo.effective_span()
            eff = self.d3_racer.set_steering_center(c)    # span_us 는 그대로, 실효만 재계산
            msg_ = (f'servo centre -> {c:.0f}us  (±{eff * 25.0 / 300.0:.1f}도, 좌우 대칭)')
            if eff < self.servo.span_us - 1e-6:
                msg_ += (f'  ⚠ 중립이 치우쳐 실효 span 이 {self.servo.span_us:.0f} -> '
                         f'{eff:.0f}us 로 줄었다')
            elif eff > old_eff + 1e-6:
                msg_ += f'  (실효 span 회복: {old_eff:.0f} -> {eff:.0f}us)'
            self.get_logger().info(msg_)

        if self.e_stop_active or not self.use_joystick_control:
            return

        self.steering = float(msg.control_msg.steering)
        self.throttle = float(msg.control_msg.throttle)

    def control_callback(self, msg: Control):
        if self.e_stop_active or self.use_joystick_control:
            return

        # 0 = straight, full stop. The servo's own centre (ServoCalib.center_us) carries the
        # mechanical trim now, so nothing is added here and the command uses the whole [-1, 1].
        self.steering = max(-1.0, min(1.0, float(msg.steering)))
        self.throttle = float(msg.throttle)
        self.last_control_time = self.get_clock().now()

    def engage_e_stop(self):
        if self.e_stop_active:
            return

        self.e_stop_active = True
        self.throttle = 0.0
        self.apply_actuation(self.steering, 0.0)
        self.get_logger().warning('E-STOP engaged. Ignoring incoming throttle commands.')

    def load_servo_calib(self):
        """Measured servo calibration (scripts/servo_sweep.py) -> ServoCalib.

        These are MEASUREMENTS, not conventions. The dataclass defaults (1500/500/1000/2000)
        are the RC-servo convention and this car's servo does not match them: its real range
        is 1300~1900us, so the old settings drove p=+-1.0 straight past the mechanical stops.
        If the config is missing the keys we fall back to those defaults -- but say so, loudly,
        because "the servo defaults" is exactly the assumption that was wrong.
        """
        cfg = {}
        if os.path.exists(self.vehicle_config_file):
            try:
                with open(self.vehicle_config_file, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f) or {}
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}')

        d = ServoCalib()          # convention defaults, only as a last resort
        if 'SERVO_CENTER_US' not in cfg:
            self.get_logger().warning(
                f'{self.vehicle_config_file} 에 SERVO_CENTER_US 가 없다 — RC 관례 기본값'
                f'({d.center_us}/{d.span_us}/{d.min_us}~{d.max_us}us)으로 돈다. 그 값은 이 차의 '
                '서보를 잰 것이 아니다: scripts/servo_sweep.py 로 실측하라. '
                '(옛 STEER_TRIM 은 더 이상 쓰이지 않는다 — 트림은 서보 중립으로 옮겼다.)')

        servo = ServoCalib(
            center_us=float(cfg.get('SERVO_CENTER_US', d.center_us)),
            span_us=float(cfg.get('SERVO_SPAN_US', d.span_us)),
            min_us=float(cfg.get('SERVO_MIN_US', d.min_us)),
            max_us=float(cfg.get('SERVO_MAX_US', d.max_us)),
        )
        # span 은 실측(조향이 일어나는 반경)이고 min/max 는 서보 하드 clip 이다. 중립이 치우쳐
        # 좌우 여유가 span 보다 좁아지면 effective_span() 이 알아서 좁은 쪽에 맞춘다 — 여기서
        # 미리 줄이지 않는다(중립을 되돌리면 복구돼야 한다). 다만 조용히 넘어가지는 않는다.
        eff = servo.effective_span()
        if eff < servo.span_us - 1e-6:
            self.get_logger().warning(
                f'중립({servo.center_us:.0f})이 치우쳐 실효 span 이 {servo.span_us:.0f} -> '
                f'{eff:.0f}us 로 줄었다 (하드 clip {servo.min_us:.0f}~{servo.max_us:.0f}). '
                f'조향각이 ±{eff * 25.0 / 300.0:.1f}도로 좁아진다 — 좌우는 대칭이다.')
        return servo

    def destroy_node(self):
        try:
            if hasattr(self, 'd3_racer') and self.d3_racer is not None:
                self.apply_actuation(0.0, 0.0)
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
