"""Perception node: camera -> shared lane pipeline -> LaneState (+ debug overlay).

Pure perception. Runs the shared `dracer_core.perception_core` pipeline and publishes
the lane state on `/lane/state` for the control node, plus an optional debug
overlay image for monitoring. It NEVER commands the vehicle and does NOT record
(recording is a separate recorder node).

Topics:
  sub : /camera/image/compressed  (sensor_msgs/CompressedImage)
  pub : /lane/state               (dracer_msgs/LaneState)
  pub : /lane/debug/compressed    (sensor_msgs/CompressedImage)  # 다패널 디버그(입력+ROI|mask|검출)
Params:
  profile ([perception] section applied), debug_scale, jpeg_quality, log_hz,
  publish_debug.
"""
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from dracer_msgs.msg import LaneState

from dracer_core.perception_core import LanePipeline, cfg_from_profile, render_panels
from dracer_core.profile import load_profile, section


def _nan(v):
    return float('nan') if v is None else float(v)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('subscribe_topic', '/camera/image/compressed')
        self.declare_parameter('state_topic', '/lane/state')
        self.declare_parameter('debug_topic', '/lane/debug/compressed')
        self.declare_parameter('profile', '')      # [perception] section applied
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('debug_scale', 2.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('publish_debug', True)

        gp = self.get_parameter
        subscribe_topic = str(gp('subscribe_topic').value)
        state_topic = str(gp('state_topic').value)
        self.debug_topic = str(gp('debug_topic').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.debug_scale = float(gp('debug_scale').value)
        self.log_hz = float(gp('log_hz').value)
        self.publish_debug = bool(gp('publish_debug').value)

        # One confirmed perception pipeline; the profile [perception] section
        # supplies all tuning. (Experiment presets/mode/per-axis overrides removed;
        # live tuning via ROS param callbacks comes in the tuning phase.)
        profile_path = os.path.expanduser(str(gp('profile').value))
        if profile_path:
            self.cfg = cfg_from_profile(section(load_profile(profile_path), 'perception'))
            self.get_logger().info(f'perception: loaded profile {profile_path}')
        else:
            self.cfg = cfg_from_profile()
        self.pipeline = LanePipeline(self.cfg)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            CompressedImage, subscribe_topic, self.image_callback, image_qos)
        self.state_pub = self.create_publisher(LaneState, state_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, image_qos)

        self._last_log = self.get_clock().now()
        self._log_period = 1.0 / self.log_hz if self.log_hz > 0 else 0.0

        self.get_logger().info(
            f'perception_node: sub={subscribe_topic} state={state_topic} '
            f'debug={self.debug_topic} cfg={self.cfg.name} '
            '(perception-only; publishes LaneState, never /control)'
        )

    def image_callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return

        if self.publish_debug:
            # 다패널(입력+ROI | mask | 검출+상태) 합성 발행 → recorder가 이 영상을 저장
            _, state, dbg = self.pipeline.process(frame, debug=True)
            self._publish_state(state, msg.header.stamp)
            self._publish_debug(render_panels(frame, dbg, self.cfg), msg.header.stamp)
        else:
            _, state = self.pipeline.process(frame)
            self._publish_state(state, msg.header.stamp)
        self._maybe_log(state)

    def _publish_state(self, s, stamp):
        m = LaneState()
        m.header.stamp = stamp
        m.header.frame_id = 'lane'
        m.valid = s['center_error'] is not None
        m.center_error = _nan(s['center_error'])
        m.ema = _nan(s['ema'])
        m.heading_valid = s['heading'] is not None
        m.heading = _nan(s['heading'])
        m.heading_label = str(s['heading_label'])
        m.confidence = float(s['confidence'])
        m.left_conf = float(s['left_conf'])
        m.right_conf = float(s['right_conf'])
        m.has_curvature = s['curvature'] is not None
        m.curvature = _nan(s['curvature'])
        m.state = str(s['state'])
        m.used_fallback = bool(s['used_fallback'])
        self.state_pub.publish(m)

    def _publish_debug(self, overlay, stamp):
        out = overlay
        if self.debug_scale and self.debug_scale != 1.0:
            out = cv2.resize(out, None, fx=self.debug_scale, fy=self.debug_scale,
                             interpolation=cv2.INTER_NEAREST)
        ok, enc = cv2.imencode('.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = 'lane_debug'
            m.format = 'jpeg'
            m.data = enc.tobytes()
            self.debug_pub.publish(m)

    def _maybe_log(self, s):
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log = now

        def f(v):
            return f'{v:+.2f}' if isinstance(v, (int, float)) else 'n/a'
        self.get_logger().info(
            f"[lane] state={s['state']} center={f(s['center_error'])} ema={f(s['ema'])} "
            f"heading={f(s['heading'])}[{s['heading_label']}] conf={s['confidence']:.2f} "
            f"L/R={s['left_conf']:.2f}/{s['right_conf']:.2f}"
            + (' FALLBACK' if s['used_fallback'] else '')
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
