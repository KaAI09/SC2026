"""Recorder node: synchronized driving-video (mp4) + driving-log (csv).

Standalone recording, decoupled from perception/control. Mirrors the joystick
START button (Joystick.is_recording) and, per START->STOP cycle, writes:
  * drive_<timestamp>.mp4  -- the recorded image stream (annotated debug overlay
    by default, or raw camera), and
  * drive_<timestamp>.csv  -- per-frame LaneState paired with BOTH the autonomous
    /control command and the manual joystick command (for imitation eval).

Rosbag recording is intentionally NOT done here: joystick_node already owns it
(START runs data_acquisition.sh -> ros2 bag). This node only adds mp4 + csv.

Topics (all subscribe):
  image_topic  (sensor_msgs/CompressedImage)  default /lane/debug/compressed
  /lane/state  (lane_msgs/LaneState)
  /control     (control_msgs/Control)          autonomous command
  joystick     (joystick_msgs/Joystick)        is_recording + manual command
Params: image_topic, record_dir, record_fps, name_prefix.
"""
import csv
import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
from lane_msgs.msg import LaneState


def _f(v, nd=4):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return ''
    return round(float(v), nd)


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        self.declare_parameter('image_topic', '/lane/debug/compressed')
        self.declare_parameter('state_topic', '/lane/state')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('record_dir', '')
        self.declare_parameter('record_fps', 30.0)
        self.declare_parameter('name_prefix', 'drive')

        gp = self.get_parameter
        image_topic = str(gp('image_topic').value)
        state_topic = str(gp('state_topic').value)
        control_topic = str(gp('control_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        self.record_dir = os.path.expanduser(str(gp('record_dir').value)) or os.getcwd()
        self.record_fps = float(gp('record_fps').value)
        self.name_prefix = str(gp('name_prefix').value)

        # latest signals (written per image frame)
        self._state = None
        self._ctrl = (0.0, 0.0)
        self._manual = (0.0, 0.0)
        self._e_stop = False
        self._want_record = False

        # writer state
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        self.create_subscription(LaneState, state_topic, self.state_callback, 10)
        self.create_subscription(Control, control_topic, self.control_callback, 10)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)

        self.get_logger().info(
            f'recorder_node: image={image_topic} state={state_topic} '
            f'control={control_topic} dir={self.record_dir} '
            '(mp4 + csv on joystick START; bag is owned by joystick_node)'
        )

    # ---- signal inputs ----------------------------------------------------
    def state_callback(self, msg: LaneState):
        self._state = msg

    def control_callback(self, msg: Control):
        self._ctrl = (float(msg.steering), float(msg.throttle))

    def joystick_callback(self, msg: Joystick):
        self._want_record = bool(msg.is_recording)
        self._e_stop = bool(msg.e_stop_en)
        self._manual = (float(msg.control_msg.steering), float(msg.control_msg.throttle))

    # ---- image + recording ------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        if self._want_record and self._writer is None:
            frame = self._decode(msg)
            if frame is not None:
                self._start_writer(frame.shape[1], frame.shape[0])
        elif not self._want_record and self._writer is not None:
            self._stop_writer()

        if self._writer is None:
            return
        frame = self._decode(msg)
        if frame is None:
            return
        self._writer.write(frame)
        frame_t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._log_row(frame_t)

    @staticmethod
    def _decode(msg):
        return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _log_row(self, frame_t):
        if self._csv_writer is None:
            return
        s = self._state
        row = [round(frame_t, 4)]
        if s is not None:
            row += [
                int(s.valid), _f(s.center_error), _f(s.ema),
                int(s.heading_valid), _f(s.heading), round(float(s.confidence), 3),
                round(float(s.left_conf), 3), round(float(s.right_conf), 3),
                s.state, int(s.used_fallback),
            ]
        else:
            row += ['', '', '', '', '', '', '', '', '', '']
        row += [_f(self._ctrl[0]), _f(self._ctrl[1]),
                _f(self._manual[0]), _f(self._manual[1]), int(self._e_stop)]
        self._csv_writer.writerow(row)

    def _start_writer(self, w, h):
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._rec_path = os.path.join(self.record_dir, f'{self.name_prefix}_{stamp}.mp4')
            self._writer = cv2.VideoWriter(
                self._rec_path, cv2.VideoWriter_fourcc(*'mp4v'), self.record_fps, (w, h))
            if not self._writer.isOpened():
                self.get_logger().error(f'Failed to open VideoWriter: {self._rec_path}')
                self._writer = None
                return
            self._csv_file = open(os.path.splitext(self._rec_path)[0] + '.csv',
                                  'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'frame_time', 'valid', 'center_error', 'ema', 'heading_valid',
                'heading', 'confidence', 'left_conf', 'right_conf', 'state',
                'used_fallback', 'ctrl_steering', 'ctrl_throttle',
                'manual_steering', 'manual_throttle', 'e_stop',
            ])
            self.get_logger().info(f'Recording started: {self._rec_path} (+ .csv)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to start recording: {exc}')
            self._writer = None
            self._csv_file = None
            self._csv_writer = None

    def _stop_writer(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Recording saved: {self._rec_path} (+ .csv)')
        if self._csv_file is not None:
            self._csv_file.close()
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None

    def destroy_node(self):
        self._stop_writer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
