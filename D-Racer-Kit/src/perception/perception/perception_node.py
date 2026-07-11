"""Perception node: camera -> shared lane pipeline -> LaneState (+ debug overlay).

Pure perception. Runs the shared `dracer_core.perception_core` pipeline and publishes
the lane state on `/lane/state` for the control node, plus an optional debug
overlay image for monitoring. It NEVER commands the vehicle and does NOT record
(recording is a separate recorder node).

Topics:
  sub : /camera/image/compressed  (sensor_msgs/CompressedImage)  BEST_EFFORT/1 = newest only
  pub : /lane/state               (dracer_msgs/LaneState)
  pub : /lane/debug/compressed    (sensor_msgs/CompressedImage)  # 다패널 디버그(BEV footprint|mask|검출)
Params:
  profile ([perception] section seeds tuning), debug_scale, jpeg_quality, log_hz,
  publish_debug, plus every perception_core.Cfg field as a LIVE param
  (`ros2 param set` rebuilds the pipeline on change).

The debug panel is built ONLY when `publish_debug` and someone is subscribed to it — it
costs more than the whole detector, so it must never run just because it might be wanted.
The CameraModel is rescaled to whatever resolution the camera actually publishes, so
camera.yaml and vehicle_config can no longer silently disagree.
"""
import math
import os
from dataclasses import fields as dc_fields

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage
from dracer_msgs.msg import LaneState

from dracer_core.calib import CameraModel
from dracer_core.perception_core import Cfg, LanePipeline, cfg_from_profile, render_panels
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
        self.declare_parameter('camera', '')       # camera.yaml -> metric BEV ('' = front-view)
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

        # Perception tuning: every Cfg field is a live ROS param. The profile
        # [perception] section seeds them; `ros2 param set` (or the monitor
        # sliders) rebuilds the pipeline live via the on-set callback.
        self._cfg_fields = [f.name for f in dc_fields(Cfg) if f.name != 'name']
        profile_path = os.path.expanduser(str(gp('profile').value))
        psec = section(load_profile(profile_path), 'perception') if profile_path else {}
        base = Cfg()
        for name in self._cfg_fields:
            default = getattr(base, name)
            val = psec.get(name, default)
            if name == 'colors':
                val = [str(c) for c in val]
            elif isinstance(default, bool):
                val = bool(val)
            elif isinstance(default, int):
                val = int(val)
            elif isinstance(default, float):
                val = float(val)
            self.declare_parameter(name, val)
        if profile_path:
            self.get_logger().info(f'perception: loaded profile {profile_path}')

        # Calibrated camera -> metric BEV. Absent/unreadable -> front-view (legacy),
        # so a missing camera.yaml degrades instead of taking the vehicle down.
        cam_path = os.path.expanduser(str(gp('camera').value))
        self.cam = None
        if cam_path:
            try:
                self.cam = CameraModel.load(cam_path)
                self.get_logger().info(f'perception: BEV {self.cam.summary()}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'perception: camera.yaml 로드 실패 ({cam_path}): {exc} '
                    '— front-view 로 계속한다')
        else:
            self.get_logger().warning('perception: camera 파라미터 없음 → front-view '
                                      '(BEV 미적용, PERCEPTION.md §6 한계 그대로)')
        self.cfg = self._build_cfg()
        self.pipeline = LanePipeline(self.cfg, self.cam)
        self.add_on_set_parameters_callback(self._on_set_params)

        # Camera IN: BEST_EFFORT / depth 1. Perception must always work on the NEWEST
        # frame. With RELIABLE / depth 10 a slow frame does not drop -- it queues, and the
        # node then steers the car from an image up to 10 frames stale. Dropping is the
        # correct behaviour here; latency is the thing we cannot afford.
        cam_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
        )
        # Debug OUT stays RELIABLE: the recorder subscribes RELIABLE, and a BEST_EFFORT
        # publisher would be QoS-incompatible with it (= silently no recording).
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            CompressedImage, subscribe_topic, self.image_callback, cam_qos)
        self.state_pub = self.create_publisher(LaneState, state_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, debug_qos)

        self._last_log = self.get_clock().now()
        self._log_period = 1.0 / self.log_hz if self.log_hz > 0 else 0.0

        self.get_logger().info(
            f'perception_node: sub={subscribe_topic} state={state_topic} '
            f'debug={self.debug_topic} cfg={self.cfg.name} '
            '(perception-only; publishes LaneState, never /control)'
        )

    # ---- perception params (profile seed + live tuning) ------------------
    def _build_cfg(self, overrides=None):
        """Build the perception Cfg from current param values (with optional
        pending overrides applied on top, for the pre-set callback)."""
        ov = overrides or {}
        gp = self.get_parameter
        d = {}
        for name in self._cfg_fields:
            val = ov[name] if name in ov else gp(name).value
            d[name] = list(val) if name == 'colors' else val
        return cfg_from_profile(d)

    def _on_set_params(self, params):
        """Live: rebuild the pipeline when any perception param is set."""
        if {p.name for p in params} & set(self._cfg_fields):
            ov = {p.name: p.value for p in params if p.name in self._cfg_fields}
            self.cfg = self._build_cfg(ov)
            self.pipeline = LanePipeline(self.cfg, self.cam)
            self.get_logger().info(f'perception live-update: {list(ov)}')
        return SetParametersResult(successful=True)

    def _match_camera(self, frame):
        """Keep the CameraModel on the resolution the camera ACTUALLY sends.

        camera.yaml stores whatever size it was calibrated at; vehicle_config decides what
        the camera publishes. When those drifted apart (yaml 320x240, camera 320x160) the
        BEV LUT sampled rows the frame did not have and `to_bev` silently filled them with
        black — it wiped out the near field, 26-37cm, precisely where the sliding window
        hunts its base peaks, and detection collapsed with nothing in any log.

        `rescale()` is exact (the GStreamer path is a pure videoscale, no crop), so simply
        adapting is free. The runtime resolution becomes a knob you can A/B on the board
        without touching the calibration.
        """
        h, w = frame.shape[:2]
        if self.cam is None or tuple(self.cam.image_size) == (w, h):
            return
        old = self.cam.image_size
        self.cam = self.cam.match((w, h))
        self.pipeline = LanePipeline(self.cfg, self.cam)
        self.get_logger().warning(
            f'perception: 카메라가 {w}x{h} 를 보내는데 camera.yaml 은 '
            f'{old[0]}x{old[1]} 기준이다 → 모델을 {w}x{h} 로 정확 rescale 했다. '
            f'{self.cam.summary()}')

    def image_callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return
        self._match_camera(frame)

        # Render ONLY when something is actually listening. The 4-panel composite plus its
        # JPEG encode dwarfs the entire detection pipeline (which works on a 232x207 BEV),
        # and it was running every frame even with no subscriber — that was the frame drop.
        # drive.launch now points the monitor and the recorder at the raw camera, so this
        # count is 0 while driving and the cost disappears; attach rqt (or point the
        # recorder back at /lane/debug/compressed) and it switches itself on.
        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            state, dbg = self.pipeline.process(frame, debug=True)
            self._publish_state(state, msg.header.stamp)
            self._publish_debug(render_panels(frame, dbg, self.cfg), msg.header.stamp)
        else:
            state = self.pipeline.process(frame)
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
