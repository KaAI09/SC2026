"""Perception-only lane-detection node for live D3-G testing.

Subscribes to the camera, runs the shared lane_core pipeline, and publishes a
debug overlay image plus periodic lane-state logs. It NEVER publishes /control
or commands the vehicle. Intended to run alongside MANUAL joystick driving so
the driver can watch lane detection on the real track.

Recording: the joystick START button toggles Joystick.is_recording (in
joystick_node). This node mirrors that flag to record the overlay video to an
MP4 file. Each START->STOP cycle writes a new lane_<mode>_<timestamp>.mp4, so a
single drive can produce multiple clips (same UX as the bag recorder).

Topics:
  sub : /camera/image/compressed  (sensor_msgs/CompressedImage)
  sub : joystick                  (joystick_msgs/Joystick)  # is_recording flag
  pub : /lane/debug/compressed    (sensor_msgs/CompressedImage)  # overlay
Params:
  mode (M1..M6) + per-axis overrides, debug_scale, jpeg_quality, log_hz,
  record (bool), record_dir, record_fps.
"""
import csv
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from joystick_msgs.msg import Joystick

from driving_core.lane_core import LanePipeline, make_cfg


class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')

        self.declare_parameter('subscribe_topic', '/camera/image/compressed')
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('debug_topic', '/lane/debug/compressed')
        self.declare_parameter('mode', 'M2')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('debug_scale', 2.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('publish_debug', True)
        # recording (START button on the joystick toggles Joystick.is_recording)
        self.declare_parameter('record', True)
        self.declare_parameter('record_dir', '')
        self.declare_parameter('record_fps', 30.0)
        # per-axis overrides (sentinels mean "unset -> preset default")
        self.declare_parameter('roi_top_frac', -1.0)
        self.declare_parameter('split_ref', '')
        self.declare_parameter('heading_method', '')
        self.declare_parameter('min_aspect', -1.0)
        self.declare_parameter('min_length', -1.0)
        self.declare_parameter('dynamic_roi', False)
        self.declare_parameter('per_lane_conf', False)
        self.declare_parameter('use_median', False)
        self.declare_parameter('do_polyfit', False)
        # ROI shape + single-line fallback width (tune 2-line sections live)
        self.declare_parameter('trap_top_w', -1.0)
        self.declare_parameter('trap_bot_w', -1.0)
        self.declare_parameter('lane_width_default', -1.0)
        self.declare_parameter('min_contour_area', -1)
        self.declare_parameter('morph_kernel', -1)
        # orange hue band (for orange-tape tracks; tune to track lighting)
        self.declare_parameter('use_orange', False)
        self.declare_parameter('orange_h_lo', -1)
        self.declare_parameter('orange_h_hi', -1)
        self.declare_parameter('orange_s_min', -1)
        self.declare_parameter('orange_v_min', -1)

        gp = self.get_parameter
        subscribe_topic = str(gp('subscribe_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        self.debug_topic = str(gp('debug_topic').value)
        mode = str(gp('mode').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.debug_scale = float(gp('debug_scale').value)
        self.log_hz = float(gp('log_hz').value)
        self.publish_debug = bool(gp('publish_debug').value)
        self.record_enabled = bool(gp('record').value)
        self.record_dir = os.path.expanduser(str(gp('record_dir').value)) or os.getcwd()
        self.record_fps = float(gp('record_fps').value)

        overrides = {}
        if float(gp('roi_top_frac').value) >= 0:
            overrides['roi_top_frac'] = float(gp('roi_top_frac').value)
        if str(gp('split_ref').value):
            overrides['split_ref'] = str(gp('split_ref').value)
        if str(gp('heading_method').value):
            overrides['heading_method'] = str(gp('heading_method').value)
        if float(gp('min_aspect').value) >= 0:
            overrides['min_aspect'] = float(gp('min_aspect').value)
        if float(gp('min_length').value) >= 0:
            overrides['min_length'] = float(gp('min_length').value)
        if bool(gp('dynamic_roi').value):
            overrides['dynamic_roi'] = True
        if bool(gp('per_lane_conf').value):
            overrides['per_lane_conf'] = True
        if bool(gp('use_median').value):
            overrides['use_median'] = True
        if bool(gp('do_polyfit').value):
            overrides['do_polyfit'] = True
        if float(gp('trap_top_w').value) >= 0:
            overrides['trap_top_w'] = float(gp('trap_top_w').value)
        if float(gp('trap_bot_w').value) >= 0:
            overrides['trap_bot_w'] = float(gp('trap_bot_w').value)
        if float(gp('lane_width_default').value) >= 0:
            overrides['lane_width_default'] = float(gp('lane_width_default').value)
        if int(gp('min_contour_area').value) >= 0:
            overrides['min_contour_area'] = int(gp('min_contour_area').value)
        if int(gp('morph_kernel').value) >= 0:
            overrides['morph_kernel'] = int(gp('morph_kernel').value)
        if bool(gp('use_orange').value):
            overrides['use_orange'] = True
        if int(gp('orange_h_lo').value) >= 0:
            overrides['orange_h_lo'] = int(gp('orange_h_lo').value)
        if int(gp('orange_h_hi').value) >= 0:
            overrides['orange_h_hi'] = int(gp('orange_h_hi').value)
        if int(gp('orange_s_min').value) >= 0:
            overrides['orange_s_min'] = int(gp('orange_s_min').value)
        if int(gp('orange_v_min').value) >= 0:
            overrides['orange_v_min'] = int(gp('orange_v_min').value)

        self.mode = mode
        self.cfg = make_cfg(mode, **overrides)
        self.pipeline = LanePipeline(self.cfg)

        # recording state
        self._want_record = False
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None
        # latest MANUAL joystick command (ground-truth for control design)
        self._joy_steering = 0.0
        self._joy_throttle = 0.0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            CompressedImage, subscribe_topic, self.image_callback, image_qos)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, image_qos)
        if self.record_enabled:
            self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)

        self._last_log = self.get_clock().now()
        self._log_period = 1.0 / self.log_hz if self.log_hz > 0 else 0.0

        self.get_logger().info(
            f'lane_detect_node: sub={subscribe_topic} debug={self.debug_topic} '
            f'mode={self.cfg.name} overrides={overrides} publish_debug={self.publish_debug} '
            f'record={self.record_enabled} record_dir={self.record_dir} '
            '(perception-only; never publishes /control)'
        )

    def joystick_callback(self, msg: Joystick):
        # Mirror the START-button recording flag from joystick_node, and capture
        # the manual steering/throttle command as ground truth for control design.
        self._want_record = bool(msg.is_recording)
        self._joy_steering = float(msg.control_msg.steering)
        self._joy_throttle = float(msg.control_msg.throttle)

    def image_callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return

        overlay, state = self.pipeline.process(frame)
        out = overlay
        if self.debug_scale and self.debug_scale != 1.0:
            out = cv2.resize(out, None, fx=self.debug_scale, fy=self.debug_scale,
                             interpolation=cv2.INTER_NEAREST)

        if self.publish_debug:
            ok, enc = cv2.imencode('.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                out_msg = CompressedImage()
                out_msg.header.stamp = msg.header.stamp
                out_msg.header.frame_id = 'lane_debug'
                out_msg.format = 'jpeg'
                out_msg.data = enc.tobytes()
                self.debug_pub.publish(out_msg)

        frame_t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._handle_recording(out, state, frame_t)
        self._maybe_log(state)

    def _handle_recording(self, frame, state, frame_t):
        if not self.record_enabled:
            return
        if self._want_record and self._writer is None:
            self._start_writer(frame.shape[1], frame.shape[0])
        elif not self._want_record and self._writer is not None:
            self._stop_writer()
        if self._writer is not None:
            self._writer.write(frame)
            self._log_row(state, frame_t)

    def _log_row(self, s, frame_t):
        if self._csv_writer is None:
            return

        def n(v):
            return '' if v is None else round(float(v), 4)
        # per-frame: perception state paired with the MANUAL joystick command
        self._csv_writer.writerow([
            round(frame_t, 4), n(s['center_error']), n(s['ema']), n(s['heading']),
            s['heading_label'], round(s['confidence'], 3),
            round(s['left_conf'], 3), round(s['right_conf'], 3), s['state'],
            round(self._joy_steering, 4), round(self._joy_throttle, 4),
        ])

    def _start_writer(self, w, h):
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._rec_path = os.path.join(self.record_dir, f'lane_{self.mode}_{stamp}.mp4')
            self._writer = cv2.VideoWriter(
                self._rec_path, cv2.VideoWriter_fourcc(*'mp4v'),
                self.record_fps, (w, h))
            if not self._writer.isOpened():
                self.get_logger().error(f'Failed to open VideoWriter: {self._rec_path}')
                self._writer = None
                return
            csv_path = os.path.splitext(self._rec_path)[0] + '.csv'
            self._csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'frame_time', 'center_error', 'ema', 'heading', 'heading_label',
                'confidence', 'left_conf', 'right_conf', 'state',
                'manual_steering', 'manual_throttle',
            ])
            self.get_logger().info(f'Overlay recording started: {self._rec_path} (+ .csv)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to start overlay recording: {exc}')
            self._writer = None
            self._csv_file = None
            self._csv_writer = None

    def _stop_writer(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Overlay recording saved: {self._rec_path} (+ .csv)')
        if self._csv_file is not None:
            self._csv_file.close()
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None

    def _maybe_log(self, s):
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log = now

        def f(v):
            return f'{v:+.2f}' if isinstance(v, (int, float)) else 'n/a'
        rec = ' REC' if self._writer is not None else ''
        self.get_logger().info(
            f"[lane]{rec} state={s['state']} center={f(s['center_error'])} ema={f(s['ema'])} "
            f"heading={f(s['heading'])}[{s['heading_label']}] conf={s['confidence']:.2f} "
            f"L/R={s['left_conf']:.2f}/{s['right_conf']:.2f}"
            + (' FALLBACK' if s['used_fallback'] else '')
        )

    def destroy_node(self):
        self._stop_writer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
