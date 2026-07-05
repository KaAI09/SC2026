"""Perception node: camera -> shared lane pipeline -> LaneState (+ debug overlay).

Pure perception. Runs the shared `driving_core.lane_core` pipeline and publishes
the lane state on `/lane/state` for the control node, plus an optional debug
overlay image for monitoring. It NEVER commands the vehicle and does NOT record
(recording is a separate recorder node).

Topics:
  sub : /camera/image/compressed  (sensor_msgs/CompressedImage)
  pub : /lane/state               (lane_msgs/LaneState)
  pub : /lane/debug/compressed    (sensor_msgs/CompressedImage)  # overlay
Params:
  mode (G1..G6 condition groups) + per-axis overrides, debug_scale, jpeg_quality,
  log_hz, publish_debug.
"""
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from lane_msgs.msg import LaneState

from driving_core.lane_core import LanePipeline, make_cfg
from driving_core.profile import load_profile, section


def _nan(v):
    return float('nan') if v is None else float(v)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('subscribe_topic', '/camera/image/compressed')
        self.declare_parameter('state_topic', '/lane/state')
        self.declare_parameter('debug_topic', '/lane/debug/compressed')
        # offline-selected profile (authoritative when set); else use mode+overrides
        self.declare_parameter('profile', '')
        self.declare_parameter('mode', 'G1')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('debug_scale', 2.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('publish_debug', True)
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
        self.declare_parameter('trap_top_w', -1.0)
        self.declare_parameter('trap_bot_w', -1.0)
        self.declare_parameter('lane_width_default', -1.0)
        self.declare_parameter('min_contour_area', -1)
        self.declare_parameter('morph_kernel', -1)
        # color set: comma-separated subset of white,yellow ('' -> preset default)
        self.declare_parameter('colors', '')
        self.declare_parameter('yellow_h_lo', -1)
        self.declare_parameter('yellow_h_hi', -1)
        self.declare_parameter('yellow_s_min', -1)
        self.declare_parameter('yellow_v_min', -1)

        gp = self.get_parameter
        subscribe_topic = str(gp('subscribe_topic').value)
        state_topic = str(gp('state_topic').value)
        self.debug_topic = str(gp('debug_topic').value)
        mode = str(gp('mode').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.debug_scale = float(gp('debug_scale').value)
        self.log_hz = float(gp('log_hz').value)
        self.publish_debug = bool(gp('publish_debug').value)

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
        colors_p = str(gp('colors').value).strip()
        if colors_p:
            overrides['colors'] = tuple(x.strip() for x in colors_p.split(',') if x.strip())
        if int(gp('yellow_h_lo').value) >= 0:
            overrides['yellow_h_lo'] = int(gp('yellow_h_lo').value)
        if int(gp('yellow_h_hi').value) >= 0:
            overrides['yellow_h_hi'] = int(gp('yellow_h_hi').value)
        if int(gp('yellow_s_min').value) >= 0:
            overrides['yellow_s_min'] = int(gp('yellow_s_min').value)
        if int(gp('yellow_v_min').value) >= 0:
            overrides['yellow_v_min'] = int(gp('yellow_v_min').value)

        # A profile (offline-selected) is authoritative: it replaces mode +
        # per-axis overrides so the car runs exactly what offline picked.
        profile_path = os.path.expanduser(str(gp('profile').value))
        if profile_path:
            psec = section(load_profile(profile_path), 'perception')
            mode = str(psec.pop('mode', mode))
            overrides = psec
            self.get_logger().info(f'perception: loaded profile {profile_path}')

        self.cfg = make_cfg(mode, **overrides)
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
            f'debug={self.debug_topic} mode={self.cfg.name} overrides={overrides} '
            '(perception-only; publishes LaneState, never /control)'
        )

    def image_callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return

        overlay, state = self.pipeline.process(frame)
        self._publish_state(state, msg.header.stamp)
        if self.publish_debug:
            self._publish_debug(overlay, msg.header.stamp)
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
