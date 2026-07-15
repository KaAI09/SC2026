"""Perception node: camera -> lane pipeline + mission detection -> LaneState + MissionState.

ONE node, ONE frame. Lane detection and mission-object detection are two independent
detectors (`dracer_core.perception_core` / `dracer_core.mission_core`), but they answer
questions about the SAME image, so they run in the same callback on the same decoded
frame. As two nodes they each JPEG-decoded the frame, each converted it to HSV, each
rendered and encoded a debug image, and each saw a different frame with a different
stamp -- which made "the sign was seen HERE and the branch appeared THERE" unanswerable.

What the merge buys, concretely:
  imdecode   once   (was twice)
  BGR->HSV   once   on mission frames -- the lane band is a slice of it (was twice)
  BGR->GRAY  once   and only when ArUco actually runs
  debug      one canvas, one JPEG encode, gated on a subscriber (was two, one ungated)
  stamps     lane state and mission state carry the SAME frame stamp

It NEVER commands the vehicle and does NOT record (recording is the recorder node).

Topics:
  sub : /camera/image/compressed  (sensor_msgs/CompressedImage)  BEST_EFFORT/1 = newest only
  pub : /lane/state               (dracer_msgs/LaneState)
  pub : /mission/state            (dracer_msgs/MissionState)
  pub : /lane/debug/compressed    (sensor_msgs/CompressedImage)  BEV|camera, or the 4-panel
Params:
  camera (camera.yaml -> metric BEV; REQUIRED, the node refuses to start without it),
  profile ([perception] section seeds tuning), debug_view, debug_scale, jpeg_quality,
  log_hz, publish_debug, rate_floor_hz,
  use_mission, mission_frame_skip, mission_config,
  plus every perception_core.Cfg field AND every mission_core.MissionCfg field as a LIVE
  param -- EXCEPT the DERIVED_PX ones, which cfg_to_px computes and which used to accept a
  `param set` and silently ignore it.

The debug panel is built ONLY when `publish_debug` and someone is subscribed to it — it
costs more than both detectors together, so it must never run just because it might be
wanted. `lap` therefore has no monitor at all.
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
from dracer_msgs.msg import LaneState, MissionState
import yaml

from dracer_core.calib import CameraModel
from dracer_core.mission_core import CLASS_NAMES, MissionCfg, MissionDetector
from dracer_core.perception_core import (DERIVED_PX, Cfg, LanePipeline, cfg_from_profile,
                                         render_bev, render_panels)
from dracer_core.profile import load_profile, section

DEBUG_VIEWS = ('bev', 'panels', 'off')


def _nan(v):
    return float('nan') if v is None else float(v)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('subscribe_topic', '/camera/image/compressed')
        self.declare_parameter('state_topic', '/lane/state')
        self.declare_parameter('mission_topic', '/mission/state')
        self.declare_parameter('debug_topic', '/lane/debug/compressed')
        self.declare_parameter('profile', '')      # [perception] section applied
        self.declare_parameter('camera', '')       # camera.yaml -> metric BEV. REQUIRED.
        self.declare_parameter('jpeg_quality', 80)
        # 1.0 = render at native panel size. This USED to be 2.0, and 2.0 is a 4x pixel
        # bill on a link that has to carry it: the BEV view is 552x240, so the car was
        # JPEG-encoding 1104x480 -- SEVEN times the raw camera frame (320x240) -- thirty
        # times a second, and pushing it over the venue Wi-Fi. The stream does not drop
        # frames when the link cannot keep up; they queue in the socket buffer, so the
        # latency does not plateau, it GROWS. Upscaling is the viewer's job: the browser
        # already scales the <img> to IMAGE_DISPLAY_*, and it does it for free.
        self.declare_parameter('debug_scale', 1.0)
        self.declare_parameter('log_hz', 2.0)
        self.declare_parameter('publish_debug', True)
        # 'bev'    BEV (lanes + every corridor) | camera (mission boxes). 552x240.
        # 'panels' the old 4-panel strip, 1280x240. 2.3x the pixels; use it to debug the
        #          sliding window, which is the one thing the BEV view drops.
        # 'off'    never render, even with a subscriber.
        self.declare_parameter('debug_view', 'bev')
        self.declare_parameter('use_mission', True)
        # Process every (skip+1)-th frame for MISSION only; lane runs on every frame,
        # because control's dt comes from lane stamps and the gains were tuned at 30Hz.
        # ⚠ MissionCfg.confirm_n counts PROCESSED frames, so this scales the debounce
        # WINDOW: skip=2, confirm_n=5 -> 15 camera frames -> 0.5s at 30Hz.
        self.declare_parameter('mission_frame_skip', 2)
        self.declare_parameter('mission_config', '')    # venue YAML from scripts/mission_tune.py

        gp = self.get_parameter
        subscribe_topic = str(gp('subscribe_topic').value)
        state_topic = str(gp('state_topic').value)
        mission_topic = str(gp('mission_topic').value)
        self.debug_topic = str(gp('debug_topic').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.debug_scale = float(gp('debug_scale').value)
        self.log_hz = float(gp('log_hz').value)
        self.publish_debug = bool(gp('publish_debug').value)
        self.debug_view = str(gp('debug_view').value)
        if self.debug_view not in DEBUG_VIEWS:
            raise ValueError(
                f"perception: debug_view={self.debug_view!r} 는 없다. {DEBUG_VIEWS} 중 하나여야 "
                '한다. (조용히 폴백하지 않는다 — 오타 하나가 보고 있다고 믿는 것과 다른 화면을 '
                '띄우는 것이 이 노드에서 가장 비싼 실수다.)')
        self.use_mission = bool(gp('use_mission').value)
        self.mission_skip = max(0, int(gp('mission_frame_skip').value))

        # Perception tuning: every Cfg field is a live ROS param -- EXCEPT the ones
        # `cfg_to_px` computes (DERIVED_PX). Those are outputs, not inputs, and declaring
        # them was a trap: `ros2 param set /perception_node sw_margin 40` reported success,
        # logged a live-update, and changed NOTHING, because cfg_to_px overwrote it on the way
        # in -- it is a value cfg_to_px WRITES. And `sw_margin` /
        # `jump_max` are exactly what you reach for when detection misbehaves trackside.
        # Not declaring them means ROS refuses the `param set` outright, loudly. The real
        # knobs are the `_cm` twins.
        self._cfg_fields = [f.name for f in dc_fields(Cfg)
                            if f.name != 'name' and f.name not in DERIVED_PX]
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

        # Mission tuning: every MissionCfg field is a live ROS param too. The mission node
        # used to keep its OWN copies of these numbers and drifted from the core -- the core
        # got fixed (housing gate removed, red brightness gate added) while the node kept
        # passing the old 78 and 0.55, so the offline analysis and the car were not running
        # the same detector. One source of truth, or none.
        self._mcfg_fields = [f.name for f in dc_fields(MissionCfg)]
        clash = set(self._mcfg_fields) & set(self._cfg_fields)
        if clash:
            raise RuntimeError(
                f'perception: Cfg 와 MissionCfg 의 필드 이름이 겹친다 {sorted(clash)} — '
                '하나의 ROS 파라미터가 두 설정을 뜻하게 된다. 한쪽 이름을 바꿔라.')
        mbase = MissionCfg()
        for name in self._mcfg_fields:
            default = getattr(mbase, name)
            if isinstance(default, tuple):
                default = list(default)          # ROS has no tuple parameter type
            self.declare_parameter(name, default)
        self.mcfg = self._build_mission_cfg()
        self.mdet = MissionDetector(self.mcfg) if self.use_mission else None

        # Calibrated camera -> metric BEV. REQUIRED. There is no front-view fallback any
        # more, and "degrade gracefully" was the wrong instinct here: the fallback was a
        # different pipeline with screen-space thresholds nobody had tuned, and it would
        # have driven the car. Failing to start is the safe outcome -- control_node's
        # perception watchdog then sees no /lane/state and publishes neutral, so the car
        # simply does not move.
        cam_path = os.path.expanduser(str(gp('camera').value))
        if not cam_path:
            raise RuntimeError(
                'perception: `camera` 파라미터가 비어 있다. camera.yaml 은 필수다 — '
                'front-view 경로는 삭제됐고, 모든 문턱값이 cm 단위라 BEV 없이는 의미가 없다. '
                'drive.launch 는 기본값으로 넘긴다.')
        self.cam = CameraModel.load(cam_path)     # 실패하면 여기서 죽는다. 그게 맞다.
        self.get_logger().info(f'perception: BEV {self.cam.summary()}')
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
        self.mission_pub = self.create_publisher(MissionState, mission_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, debug_qos)

        # Mission results survive the frames mission does NOT run on (frame_skip), so the
        # debug panel draws a steady box instead of one that blinks at 10Hz inside a 30Hz
        # video. They are at most `mission_skip` frames stale -- 66ms at 30Hz / skip 2.
        self._frame_i = 0
        self._mdets = []
        self._mcls = None
        self._sign = None            # 'L'|'R' — last confirmed direction sign (held briefly)
        self._sign_age = 10 ** 6     # frames since the sign was last CONFIRMED

        self._last_log = self.get_clock().now()
        self._log_period = 1.0 / self.log_hz if self.log_hz > 0 else 0.0
        # Achieved rate. Nothing measured this, so a slowdown was invisible. The perception
        # thresholds no longer stretch with it (they are durations now, fed by `_tick`), but
        # the rate still matters on its own: it IS latency, control's `dt` comes from these
        # stamps, and the gains were tuned at 30Hz. The 0711 runs held 30.0Hz with a worst gap
        # of 38ms over 1502 frames and zero drops; anything materially under that is a
        # different machine from the one the gains were tuned on, and you should hear about it.
        self._rate_floor = float(self.declare_parameter('rate_floor_hz', 24.0).value)
        self._t_prev = None
        self._dt_ema = None
        self._slow_since = None

        if self.mdet is not None:
            n = self.mission_skip + 1
            self.get_logger().info(
                f'mission: ON  frame_skip={self.mission_skip} (매 {n}프레임) '
                f'aruco={self.mcfg.aruco_dict} ids={list(self.mcfg.aruco_ids)} '
                f'green_s>={self.mcfg.green_s_min} red_s>={self.mcfg.red_s_min} '
                f'confirm={self.mcfg.confirm_m}/{self.mcfg.confirm_n} '
                f'(stop {self.mcfg.confirm_m_stop}) '
                f'-> 디바운스 창 = {n * self.mcfg.confirm_n} 카메라 프레임')
        else:
            self.get_logger().info('mission: OFF (use_mission=false) — /mission/state 는 안 나온다')
        self.get_logger().info(
            f'perception_node: sub={subscribe_topic} state={state_topic} '
            f'mission={mission_topic} debug={self.debug_topic}[{self.debug_view}] '
            f'cfg={self.cfg.name} '
            '(perception-only; publishes LaneState + MissionState, never /control)'
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

    def _build_mission_cfg(self, overrides=None):
        """MissionCfg from params, then the venue YAML ON TOP.

        `mission_config` is what `scripts/mission_tune.py` writes at the venue after looking
        at the actual traffic light through this actual camera. It wins over the params
        because it is the only one of the two that has SEEN the lamp -- the defaults were
        measured under practice lighting, and an LED's saturation is exactly the thing a
        venue changes.
        """
        ov = overrides or {}
        gp = self.get_parameter
        d = {}
        for name in self._mcfg_fields:
            val = ov[name] if name in ov else gp(name).value
            default = getattr(MissionCfg(), name)
            d[name] = tuple(val) if isinstance(default, tuple) else val
        path = os.path.expanduser(str(gp('mission_config').value))
        if path:
            with open(path) as f:
                venue = yaml.safe_load(f) or {}
            for k, v in venue.items():
                if k not in d:
                    continue
                d[k] = tuple(v) if isinstance(getattr(MissionCfg(), k), tuple) else v
            self.get_logger().info(f'perception: mission venue config {path} applied')
        return MissionCfg(**d)

    def _on_set_params(self, params):
        """Live: swap the config INTO the running pipeline. State survives.

        This used to do `self.pipeline = LanePipeline(...)`, which reset the Tracker's
        left/right identity, its width EMA and the centre EMA -- so every live tune punched a
        perception discontinuity into the drive, and the only advice we could give was "stop
        the car first". `reconfigure` keeps the measurement and swaps only the judgement.
        """
        names = {p.name for p in params}
        if names & set(self._cfg_fields):
            ov = {p.name: p.value for p in params if p.name in self._cfg_fields}
            self.cfg = self._build_cfg(ov)
            self.pipeline.reconfigure(self.cfg)
            self.get_logger().info(
                f'perception live-update: {sorted(ov)} (Tracker/EMA 상태 유지)')
        if names & set(self._mcfg_fields) and self.mdet is not None:
            # The detector IS rebuilt (unlike the lane pipeline), which drops its debounce
            # history. That history is `confirm_n` PROCESSED frames -- 5, i.e. ~0.5s -- and
            # the ArUco dictionary / error-correction rate are baked into the detector at
            # construction anyway. Half a second of vote history is not worth a second code
            # path that only half-applies a change.
            ov = {p.name: p.value for p in params if p.name in self._mcfg_fields}
            self.mcfg = self._build_mission_cfg(ov)
            self.mdet = MissionDetector(self.mcfg)
            self.get_logger().info(f'mission live-update: {sorted(ov)} (디바운스 이력 리셋)')
        # Render knobs. These were read ONCE at construction, so `ros2 param set
        # /perception_node debug_scale 1.0` answered "successful" and changed nothing --
        # the same silent-success failure this callback was written to end. They are the
        # knobs you reach for when the monitor lags mid-session, which is exactly when
        # restarting the node is the thing you cannot afford to do.
        for p in params:
            if p.name == 'debug_view':
                if p.value not in DEBUG_VIEWS:
                    return SetParametersResult(
                        successful=False,
                        reason=f'debug_view={p.value!r} 는 없다. {DEBUG_VIEWS} 중 하나여야 한다.')
                self.debug_view = str(p.value)
            elif p.name == 'debug_scale':
                self.debug_scale = float(p.value)
            elif p.name == 'jpeg_quality':
                self.jpeg_quality = int(p.value)
            elif p.name == 'publish_debug':
                self.publish_debug = bool(p.value)
            else:
                continue
            self.get_logger().info(f'perception render-update: {p.name} -> {p.value}')
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
        if tuple(self.cam.image_size) == (w, h):
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

        # Perception's clock. Every filter and failsafe threshold in the pipeline is a
        # physical duration now, so dt must be measured BEFORE the frame is processed and
        # handed in -- the pipeline owns neither the clock nor the stamps.
        dt_s = self._tick()
        self._frame_i += 1
        run_mission = (self.mdet is not None
                       and self._frame_i % (self.mission_skip + 1) == 0)

        # ONE conversion, shared. Only on mission frames: the lane pipeline needs HSV of the
        # BEV band ONLY (`cam.src_rows`), the mission detectors need the whole frame, and a
        # full-frame convert on a frame mission is not going to look at would be paying for
        # nothing. On a mission frame the band is just a slice of what we already have.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) if run_mission else None

        if run_mission:
            # GRAY here, not inside detect_aruco: it is the only consumer, but building it at
            # the call site keeps the "convert the frame once" rule in ONE place, where it can
            # be read.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self._mdets, self._mcls, newly = self.mdet.process(frame, hsv=hsv, gray=gray)
            self._publish_mission(msg.header.stamp, newly)
            if newly and self._mcls is not None:
                self.get_logger().info(
                    f'[mission] {self._mcls}:{CLASS_NAMES.get(self._mcls, "?")} 확정')

            # 표지판(RIGHT=3/LEFT=4) → 갈림길 회피 hint 래치. `_mcls` 는 디바운스된 값이라
            # 표지판이 확정된 동안만 잡고, 실제로 hint 를 미는 것은 프레임마다 도는
            # `_sign_age` 쪽이다 (mission 이 스킵되는 프레임에도 나이는 먹어야 한다).
            if self._mcls in (3, 4):
                self._sign = 'R' if self._mcls == 3 else 'L'
                self._sign_age = 0

        # 표지판(sign_live_hold)의 생존 판정은 mission_frame_skip 과 무관하게 매 프레임
        # 돌아야 한다 — 그래야 mission 이 스킵된 프레임에서도 hint 가 제때 꺼진다.
        self._sign_age += 1
        if self.pipeline is not None:
            live = self._sign_age <= self.cfg.sign_live_hold
            self.pipeline.set_branch_hint(self._sign if live else None)

        # Render ONLY when something is actually listening. The debug composite plus its JPEG
        # encode dwarfs both detectors put together (they work on a 320x240 frame and a
        # 232x207 BEV), and it used to run every frame even with no subscriber — that was the
        # frame drop. `collect`/`lap` point the monitor at the raw camera (or have none), so
        # this count is 0 and the cost disappears; point it at the debug topic and it switches
        # itself on, which is the same as asking for it.
        want_debug = (self.publish_debug and self.debug_view != 'off'
                      and self.debug_pub.get_subscription_count() > 0)
        if want_debug:
            state, dbg = self.pipeline.process(frame, dt_s, debug=True, hsv=hsv)
            self._publish_state(state, msg.header.stamp)
            img = (render_bev(frame, dbg, self.cfg, self._mdets, self._mcls)
                   if self.debug_view == 'bev' else render_panels(frame, dbg, self.cfg))
            self._publish_debug(img, msg.header.stamp)
        else:
            state = self.pipeline.process(frame, dt_s, hsv=hsv)
            self._publish_state(state, msg.header.stamp)
        self._maybe_log(state)

    def _tick(self):
        """Advance perception's clock: return dt (s) since the previous frame, and complain
        if the achieved rate sags.

        First frame returns 0.0 -- no time has passed, so the pipeline's EMAs seed instead of
        blending. A LONG gap is passed through as-is, deliberately: after a dropout the stored
        EMAs are older than their own time constant and the honest thing is to forget them
        (`_ema_alpha` -> 1) and to let `lost_stop_s` fire. Clamping the gap would hide a
        dropout from exactly the failsafes that exist to catch it.
        """
        now = self.get_clock().now()
        dt = 0.0
        if self._t_prev is not None:
            dt = (now - self._t_prev).nanoseconds / 1e9
            if 0.0 < dt < 1.0:      # the RATE estimate ignores outliers; `dt` itself does not
                self._dt_ema = dt if self._dt_ema is None else 0.9 * self._dt_ema + 0.1 * dt
        self._t_prev = now
        dt = max(0.0, dt)
        if self._dt_ema is None or self._rate_floor <= 0.0:
            return dt
        hz = 1.0 / self._dt_ema
        if hz < self._rate_floor:
            if self._slow_since is None:
                self._slow_since = now
                self.get_logger().warning(
                    f'perception {hz:.1f}Hz < rate_floor {self._rate_floor:.0f}Hz. '
                    '인지 문턱값은 이제 시간 단위라 이 속도에서도 같은 뜻을 갖는다 — 하지만 '
                    '제어 게인(kp 0.45)은 30Hz 에서 검증된 값이고, 무엇보다 이 레이트 자체가 '
                    '지연이다.')
        elif self._slow_since is not None:
            self._slow_since = None
            self.get_logger().info(f'perception rate recovered: {hz:.1f}Hz')
        return dt

    @property
    def rate_hz(self):
        return 0.0 if not self._dt_ema else 1.0 / self._dt_ema

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
        m.n_corridors = int(min(255, s['n_corridors']))
        m.ego_rule = str(s['ego_rule'])
        m.fork_type = str(s.get('fork_type', '') or '')
        m.n_islands = int(s.get('n_islands', 0))
        self.state_pub.publish(m)

    def _publish_mission(self, stamp, newly):
        """CONFIRMED class + the raw top detection of this frame.

        Both, because they answer different questions. `cls` is what a consumer acts on: it
        survived an M-of-N vote, and STOP classes (RED / MARK) confirm at a lower threshold
        than GO because the two errors are not symmetric. `det_*` is what fired THIS frame,
        with its box -- it is what the panel draws and what a csv needs to say why `cls` is,
        or is not, what it is. A recording with only `cls` cannot tell a missed detection
        from a rejected one.
        """
        top = max(self._mdets, key=lambda d: d[1], default=None)
        m = MissionState()
        m.header.stamp = stamp
        m.header.frame_id = 'mission'
        m.cls = int(self._mcls) if self._mcls is not None else -1
        m.newly_confirmed = bool(newly)
        if top is None:
            m.det_cls = -1
            m.det_conf = 0.0
        else:
            cls_id, conf, (x, y, w, h) = top
            m.det_cls = int(cls_id)
            m.det_conf = float(conf)
            m.det_x, m.det_y, m.det_w, m.det_h = int(x), int(y), int(w), int(h)
        self.mission_pub.publish(m)

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
        mission = ('' if self.mdet is None
                   else f" mission={CLASS_NAMES.get(self._mcls, '--')}")
        self.get_logger().info(
            f"[lane] {self.rate_hz:4.1f}Hz state={s['state']} center={f(s['center_error'])} "
            f"ema={f(s['ema'])} "
            f"heading={f(s['heading'])}[{s['heading_label']}] conf={s['confidence']:.2f} "
            f"L/R={s['left_conf']:.2f}/{s['right_conf']:.2f} "
            f"corridors={s['n_corridors']}[{s['ego_rule']}]"
            + (' FALLBACK' if s['used_fallback'] else '') + mission
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
