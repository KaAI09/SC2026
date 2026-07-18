import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml

from dracer_msgs.msg import Control
from dracer_msgs.msg import Joystick
from topst_utils.gamepads import ShanWanGamepad


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/SC2026/D-Racer-Kit/src/config/vehicle_config.yaml'


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # ROS parameters
        self.declare_parameter('publish_topic', 'joystick')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('throttle_scale', 0.22)
        self.declare_parameter('throttle_deadzone', 0.05)
        self.declare_parameter('steering_deadzone', 0.05)
        self.declare_parameter('steering_axis', 'auto')
        # 서보 중립(us). 트림은 서보의 중립이지 명령의 일부가 아니다 — 명령에 더하면
        # u 와 트림이 같은 [-1,1] 예산을 나눠 쓰고 한쪽 조향 권한이 조용히 깎인다.
        self.declare_parameter('steer_center_us', 1500.0)
        # 중립 조정 범위 = 서보 하드 clip (1250~2050). 서보는 여기까지 무리 없이 돈다.
        #
        # ⚠ 중립을 실측값(1650)에서 크게 옮기면 한쪽 여유가 조향 반경(300us)보다 좁아지고,
        # actuator 가 대칭을 지키려고 실효 span 을 그만큼 줄인다 = 조향각이 줄어든다.
        # 예: 중립 1500 -> 좌우 여유 min(550, 250) = 250us -> ±20.8도.
        # 트림 보정은 실측값 근처 몇 스텝이면 된다. 그보다 크게 벗어나야 한다면 그것은
        # 트림이 아니라 서보 혼/링키지를 기계적으로 다시 맞추라는 신호다.
        self.declare_parameter('steer_center_min_us', 1250.0)
        self.declare_parameter('steer_center_max_us', 2050.0)
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('calibration_step', 10.0)        # us
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('accel_ratio_step', 0.005)
        self.declare_parameter('accel_ratio_min', 0.12)
        self.declare_parameter('accel_ratio_max', 0.4)
        self.declare_parameter('debug_log_enable', True)
        self.declare_parameter('debug_log_hz', 5.0)

        publish_topic = str(self.get_parameter('publish_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')

        self.throttle_scale = float(self.get_parameter('throttle_scale').value)
        self.throttle_deadzone = float(self.get_parameter('throttle_deadzone').value)
        self.steering_deadzone = float(self.get_parameter('steering_deadzone').value)
        self.steering_axis = str(self.get_parameter('steering_axis').value)
        self.steer_center_us = float(self.get_parameter('steer_center_us').value)
        self.center_min_us = float(self.get_parameter('steer_center_min_us').value)
        self.center_max_us = float(self.get_parameter('steer_center_max_us').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.calibration_step = float(self.get_parameter('calibration_step').value)
        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        self.accel_ratio_step = float(self.get_parameter('accel_ratio_step').value)
        self.accel_ratio_min = float(self.get_parameter('accel_ratio_min').value)
        self.accel_ratio_max = float(self.get_parameter('accel_ratio_max').value)
        self.debug_log_enable = bool(self.get_parameter('debug_log_enable').value)
        self.debug_log_hz = float(self.get_parameter('debug_log_hz').value)
        self.publish_hz = publish_hz

        if self.accel_ratio_min > self.accel_ratio_max:
            raise ValueError('accel_ratio_min must be less than or equal to accel_ratio_max')

        self.accel_ratio = self.clamp(
            self.throttle_scale,
            self.accel_ratio_min,
            self.accel_ratio_max,
        )

        self._prev_l1_pressed = False
        self._prev_r1_pressed = False
        self._prev_y_pressed = False
        self._prev_b_pressed = False
        self._prev_x_pressed = False
        self._prev_start_pressed = False
        self._prev_a_pressed = False
        self.e_stop_latched = False
        self.is_recording = False
        self.engage_latched = False       # A button toggles autonomous engage
        self._debug_left_y = 0.0
        self._debug_right_x = 0.0
        self._debug_right_y = 0.0
        self._debug_steering = 0.0
        self._debug_throttle = 0.0

        self.load_saved_calibration()

        self.joystick_pub = self.create_publisher(Joystick, publish_topic, 10)
        self.gamepad = ShanWanGamepad()
        self.latest_input = None
        self.lock = threading.Lock()

        self.running = True
        self.reader_thread = threading.Thread(
            target=self.gamepad_read_loop,
            daemon=True,
        )
        self.reader_thread.start()

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        if self.debug_log_enable and self.debug_log_hz > 0.0:
            self.debug_timer = self.create_timer(
                1.0 / self.debug_log_hz,
                self.debug_timer_callback,
            )

        self.get_logger().info(
            f'Joystick node started: topic={publish_topic}, publish_hz={self.publish_hz}, '
            f'throttle_scale={self.throttle_scale}, throttle_deadzone={self.throttle_deadzone}, '
            f'steering_deadzone={self.steering_deadzone}, steering_axis={self.steering_axis}, '
            f'steer_center_us={self.steer_center_us:.0f}, calibration_mode={self.calibration_mode}, '
            f'calibration_step={self.calibration_step}us, vehicle_config_file={self.vehicle_config_file}, '
            f'accel_ratio={self.accel_ratio}, accel_ratio_step={self.accel_ratio_step}, '
            f'accel_ratio_min={self.accel_ratio_min}, accel_ratio_max={self.accel_ratio_max}, '
            f'debug_log_enable={self.debug_log_enable}, debug_log_hz={self.debug_log_hz}'
        )

    @staticmethod
    def clamp(value, min_v=-1.0, max_v=1.0):
        return max(min(value, max_v), min_v)

    @staticmethod
    def deadzone(value, deadzone_value=0.05):
        return 0.0 if abs(value) < deadzone_value else value

    def load_saved_calibration(self):
        if not os.path.exists(self.vehicle_config_file):
            return

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as calibration_stream:
                calibration_data = yaml.safe_load(calibration_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return

        saved_center = calibration_data.get('SERVO_CENTER_US')
        if saved_center is not None:
            self.steer_center_us = float(saved_center)
            self.set_parameters([
                Parameter('steer_center_us', Parameter.Type.DOUBLE, self.steer_center_us),
            ])
        elif calibration_data.get('STEER_TRIM') is not None:
            # 옛 캘리브레이션. 조용히 무시하면 차가 예전과 다르게 서서 원인을 못 찾는다.
            self.get_logger().warning(
                'vehicle_config 에 옛 STEER_TRIM 이 있고 SERVO_CENTER_US 가 없다. '
                '트림은 서보 중립으로 옮겼다 — scripts/servo_sweep.py 로 실측하고 '
                'SERVO_CENTER_US 를 넣어라. 지금은 중립을 기본값으로 둔다.')

        saved_accel = calibration_data.get('ACCEL_RATIO')
        if saved_accel is not None:
            self.accel_ratio = self.clamp(
                float(saved_accel), self.accel_ratio_min, self.accel_ratio_max)
            self.get_logger().info(
                f'loaded ACCEL_RATIO={self.accel_ratio:.3f} from vehicle config')

    def save_calibration(self):
        calibration_dir = os.path.dirname(self.vehicle_config_file)
        if calibration_dir:
            os.makedirs(calibration_dir, exist_ok=True)

        config_data = {}
        if os.path.exists(self.vehicle_config_file):
            try:
                with open(self.vehicle_config_file, 'r', encoding='utf-8') as calibration_stream:
                    config_data = yaml.safe_load(calibration_stream) or {}
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to merge existing vehicle config {self.vehicle_config_file}: {exc}'
                )

        config_data['SERVO_CENTER_US'] = float(self.steer_center_us)
        config_data['ACCEL_RATIO'] = float(self.accel_ratio)

        with open(self.vehicle_config_file, 'w', encoding='utf-8') as calibration_stream:
            yaml.safe_dump(config_data, calibration_stream, sort_keys=False)

    def update_steering_trim_from_buttons(self, data):
        """Y/B -> 서보 중립을 ±step us 옮긴다. actuator 가 메시지로 받아 즉시 반영한다.

        서보의 중립 자체를 움직이므로, 맞추고 나면 u=0 이 진짜 직진이고 u 는 ±1.0
        전체를 대칭으로 쓴다.
        """
        if not self.calibration_mode:
            return

        y_pressed = bool(data.button_y)
        b_pressed = bool(data.button_b)
        changed = False

        if y_pressed and not self._prev_y_pressed:
            self.steer_center_us = self.clamp(
                self.steer_center_us - self.calibration_step,
                self.center_min_us, self.center_max_us)
            changed = True

        if b_pressed and not self._prev_b_pressed:
            self.steer_center_us = self.clamp(
                self.steer_center_us + self.calibration_step,
                self.center_min_us, self.center_max_us)
            changed = True

        if changed:
            self.set_parameters([
                Parameter('steer_center_us', Parameter.Type.DOUBLE, self.steer_center_us),
            ])
            try:
                self.save_calibration()
                self.get_logger().info(
                    f'steer_center_us updated to {self.steer_center_us:.0f} us')
            except Exception as exc:
                self.get_logger().error(f'Failed to save steering calibration: {exc}')

        self._prev_y_pressed = y_pressed
        self._prev_b_pressed = b_pressed

    def update_accel_ratio_from_buttons(self, data):
        l1_pressed = bool(data.button_L1)
        r1_pressed = bool(data.button_R1)
        accel_changed = False

        if l1_pressed and not self._prev_l1_pressed:
            self.accel_ratio = self.clamp(
                self.accel_ratio - self.accel_ratio_step,
                self.accel_ratio_min,
                self.accel_ratio_max,
            )
            accel_changed = True
            self.get_logger().info(f'accel_ratio decreased to {self.accel_ratio:.3f}')

        if r1_pressed and not self._prev_r1_pressed:
            self.accel_ratio = self.clamp(
                self.accel_ratio + self.accel_ratio_step,
                self.accel_ratio_min,
                self.accel_ratio_max,
            )
            accel_changed = True
            self.get_logger().info(f'accel_ratio increased to {self.accel_ratio:.3f}')

        if accel_changed:                       # persist to vehicle config
            try:
                self.save_calibration()
            except Exception as exc:
                self.get_logger().error(f'Failed to save accel_ratio calibration: {exc}')

        self._prev_l1_pressed = l1_pressed
        self._prev_r1_pressed = r1_pressed

    def update_e_stop_from_buttons(self, data):
        x_pressed = bool(data.button_x)

        if x_pressed and not self._prev_x_pressed and not self.e_stop_latched:
            self.e_stop_latched = True
            self.get_logger().warning('E-STOP latched by joystick X button')

        self._prev_x_pressed = x_pressed

    def update_recording_from_buttons(self, data):
        # START toggles is_recording, published in the Joystick msg. recorder_node
        # mirrors this flag to start/stop its mp4 + csv. No rosbag is spawned here.
        start_pressed = bool(data.button_start)

        if start_pressed and not self._prev_start_pressed:
            self.is_recording = not self.is_recording
            self.get_logger().info(
                'recording started (recorder mp4+csv)' if self.is_recording
                else 'recording stopped')

        self._prev_start_pressed = start_pressed

    def update_engage_from_buttons(self, data):
        # A toggles autonomous engage, published in the Joystick msg. driving_node
        # OR-combines this with its `engage` param (either can engage). E-STOP wins:
        # a latched X forces engage off here so the flag can never re-enable output.
        if self.e_stop_latched:
            self.engage_latched = False
        a_pressed = bool(data.button_a)
        if a_pressed and not self._prev_a_pressed and not self.e_stop_latched:
            self.engage_latched = not self.engage_latched
            self.get_logger().warning(
                'ENGAGE ON (joystick A) — autonomous output enabled' if self.engage_latched
                else 'engage off (joystick A)')
        self._prev_a_pressed = a_pressed

    def read_steering_axis(self, data):
        right_x = self.clamp(data.analog_stick_right.x)
        right_y = self.clamp(data.analog_stick_right.y)

        if self.steering_axis == 'right_x':
            return right_x
        if self.steering_axis == 'right_y':
            return right_y
        return right_x if abs(right_x) >= abs(right_y) else right_y

    def gamepad_read_loop(self):
        while rclpy.ok() and self.running:
            try:
                data = self.gamepad.read_data()
                self.update_accel_ratio_from_buttons(data)
                self.update_steering_trim_from_buttons(data)
                self.update_e_stop_from_buttons(data)
                self.update_engage_from_buttons(data)
                self.update_recording_from_buttons(data)
                with self.lock:
                    self.latest_input = data
            except Exception as exc:
                self.get_logger().error(f'Gamepad read error: {exc}')
                time.sleep(0.1)

    def timer_callback(self):
        with self.lock:
            data = self.latest_input

        if data is None:
            return

        throttle_axis = self.deadzone(
            self.clamp(data.analog_stick_left.y),
            self.throttle_deadzone,
        )
        throttle = self.clamp(throttle_axis * self.accel_ratio)
        throttle = max(0.0, throttle)

        steering = self.deadzone(
            self.read_steering_axis(data),
            self.steering_deadzone,
        )
        # 트림을 더하지 않는다. 0 = 직진이고, 그것은 서보의 중립(steer_center_us)이 만든다.
        steering = self.clamp(steering)
        if self.e_stop_latched:
            throttle = 0.0

        self._debug_left_y = float(data.analog_stick_left.y)
        self._debug_right_x = float(data.analog_stick_right.x)
        self._debug_right_y = float(data.analog_stick_right.y)
        self._debug_steering = float(steering)
        self._debug_throttle = float(throttle)

        msg = Joystick()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joystick'
        control_msg = Control()
        control_msg.header.stamp = msg.header.stamp
        control_msg.header.frame_id = 'joystick'
        control_msg.steering = float(steering)
        control_msg.throttle = float(throttle)
        msg.control_msg = control_msg
        msg.accel_ratio = float(self.accel_ratio)
        # 서보 중립을 함께 싣는다 — actuator 가 이걸로 서보를 즉시 갱신한다.
        # 자율 주행 중에도 유효하다: 서보 중립은 '누가 운전하는가' 가 아니라
        # '이 차가 어떻게 생겼는가' 의 속성이다.
        msg.steer_center_us = float(self.steer_center_us)
        msg.e_stop_en = bool(self.e_stop_latched)
        msg.is_recording = bool(self.is_recording)
        msg.engage = bool(self.engage_latched)
        self.joystick_pub.publish(msg)

    def debug_timer_callback(self):
        with self.lock:
            data = self.latest_input

        l1_state = int(bool(data.button_L1)) if data else 0
        r1_state = int(bool(data.button_R1)) if data else 0
        self.get_logger().info(
            f'[Joystick DBG] \n'
            f'left_y={self._debug_left_y:.2f} \n'
            f'right_x={self._debug_right_x:.2f} right_y={self._debug_right_y:.2f} \n'
            f'steering={self._debug_steering:.2f} throttle={self._debug_throttle:.2f} \n'
            f'accel_ratio={self.accel_ratio:.3f} \n'
            f'steer_center={self.steer_center_us:.0f}us \n'
            f'e_stop={int(self.e_stop_latched)} \n'
            f'recording={int(self.is_recording)} \n'
            f'L1={l1_state} R1={r1_state}\n'
        )

    def destroy_node(self):
        self.running = False
        reader_thread = getattr(self, 'reader_thread', None)
        if reader_thread is not None and reader_thread.is_alive():
            reader_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
