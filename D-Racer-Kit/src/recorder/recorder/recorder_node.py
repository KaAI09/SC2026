"""Recorder node: synchronized driving video (mp4) + driving log (csv).

Standalone recording, decoupled from perception/control. Mirrors the joystick
START button (Joystick.is_recording). One START->STOP cycle = one session, and a
session writes files that SHARE ONE basename `<prefix>_<timestamp>`:

    <record_dir>/
      raw/<prefix>_<stamp>.mp4     RAW camera (/camera/image/compressed)
      csv/<prefix>_<stamp>.csv     per-frame LaneState + autonomous + manual command
      panel/<prefix>_<stamp>.mp4   annotated overlay -- ONLY if a launch asks for it

WHAT THE LAUNCHES ACTUALLY DO: every one of them points `image_topic` at the raw
camera and leaves `raw_topic` empty, so raw IS the main stream and only raw/ + csv/
are written. The panel mp4 is reconstructed offline (`offline/panel_replay.py`) from
exactly those two files -- the car must not spend a frame's worth of CPU rendering a
video it is not watching. The dual-stream path below stays because the capability is
free to keep, not because anything uses it.

Panel frames (when recorded) are stamped top-right with `<name> f<idx> t<sec>` so a
single screenshot identifies its source clip, video frame, and csv row (frame_idx is
1:1 with the csv data row). RAW frames are NEVER stamped -- an overlay would corrupt
the very pixels offline perception and camera calibration must read.

Topics (all subscribe):
  image_topic    (sensor_msgs/CompressedImage)  main stream; drives csv + stamping
  raw_topic      (sensor_msgs/CompressedImage)  extra RAW stream (skipped if == image_topic)
  /lane/state    (dracer_msgs/LaneState)
  /mission/state (dracer_msgs/MissionState)     object detection: confirmed class + raw box
  /control       (dracer_msgs/Control)          autonomous command
  joystick       (dracer_msgs/Joystick)         is_recording + manual command
Params: image_topic, raw_topic, state_topic, mission_topic, control_topic, joystick_topic,
        record_dir, record_fps, name_prefix.
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
from dracer_msgs.msg import Control
from dracer_msgs.msg import Joystick
from dracer_msgs.msg import LaneState
from dracer_msgs.msg import MissionState

PANEL_DIR, RAW_DIR, CSV_DIR = 'panel', 'raw', 'csv'


def _f(v, nd=4):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return ''
    return round(float(v), nd)


def _stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        self.declare_parameter('image_topic', '/lane/debug/compressed')
        self.declare_parameter('raw_topic', '/camera/image/compressed')
        self.declare_parameter('state_topic', '/lane/state')
        self.declare_parameter('mission_topic', '/mission/state')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('record_dir', '')
        self.declare_parameter('record_fps', 30.0)
        self.declare_parameter('name_prefix', 'drive')

        gp = self.get_parameter
        image_topic = str(gp('image_topic').value)
        raw_topic = str(gp('raw_topic').value)
        state_topic = str(gp('state_topic').value)
        mission_topic = str(gp('mission_topic').value)
        control_topic = str(gp('control_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        self.record_dir = os.path.expanduser(str(gp('record_dir').value)) or os.getcwd()
        self.record_fps = float(gp('record_fps').value)
        self.name_prefix = str(gp('name_prefix').value)

        # Raw IS the main stream whenever a launch points image_topic at the camera and
        # leaves raw_topic empty -- which every launch now does. Only then is there nothing
        # extra to record. Give the node a distinct raw_topic and it records both.
        self._dual = bool(raw_topic) and raw_topic != image_topic
        self._main_dir = PANEL_DIR if self._dual else RAW_DIR

        # latest signals (written per main-stream frame)
        self._state = None
        self._mission = None
        self._ctrl = (0.0, 0.0)
        self._manual = (0.0, 0.0)
        self._e_stop = False
        self._want_record = False

        # session state
        self._session = None        # shared timestamp; None = not recording
        self._vid = {}              # subdir -> dict(writer, path, idx, t0)
        self._csv_file = None
        self._csv_writer = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        if self._dual:
            self.create_subscription(CompressedImage, raw_topic, self.raw_callback, image_qos)
        self.create_subscription(LaneState, state_topic, self.state_callback, 10)
        self.create_subscription(MissionState, mission_topic, self.mission_callback, 10)
        self.create_subscription(Control, control_topic, self.control_callback, 10)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)

        streams = f'{image_topic} -> {self._main_dir}/'
        if self._dual:
            streams += f' + {raw_topic} -> {RAW_DIR}/'
        self.get_logger().info(
            f'recorder_node: {streams} state={state_topic} control={control_topic} '
            f'dir={self.record_dir} (mp4 + csv on joystick START)'
        )

    # ---- signal inputs ----------------------------------------------------
    def state_callback(self, msg: LaneState):
        self._state = msg

    def mission_callback(self, msg: MissionState):
        self._mission = msg

    def control_callback(self, msg: Control):
        self._ctrl = (float(msg.steering), float(msg.throttle))

    def joystick_callback(self, msg: Joystick):
        self._want_record = bool(msg.is_recording)
        self._e_stop = bool(msg.e_stop_en)
        self._manual = (float(msg.control_msg.steering), float(msg.control_msg.throttle))

    # ---- image + recording ------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        """Main stream: opens/closes the session, drives the csv, gets stamped."""
        if self._want_record and self._session is None:
            self._open_session()
        elif not self._want_record and self._session is not None:
            self._close_session()
        if self._session is None:
            return
        frame = self._decode(msg)
        if frame is None:
            return
        frame_t = _stamp_of(msg)
        # stamp ONLY the panel overlay; a raw main stream must stay unannotated
        self._write(self._main_dir, frame, frame_t, annotate=self._dual)
        self._log_row(frame_t)

    def raw_callback(self, msg: CompressedImage):
        """Extra RAW stream. The main stream owns the session lifecycle."""
        if self._session is None:
            return
        frame = self._decode(msg)
        if frame is not None:
            self._write(RAW_DIR, frame, _stamp_of(msg), annotate=False)

    @staticmethod
    def _decode(msg):
        return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _write(self, subdir, frame, frame_t, annotate):
        v = self._vid.get(subdir)
        if v is None:
            v = self._open_writer(subdir, frame.shape[1], frame.shape[0])
            if v is None:
                return
        if v['t0'] is None:
            v['t0'] = frame_t
        if annotate:
            self._annotate(frame, os.path.splitext(os.path.basename(v['path']))[0],
                           v['idx'], frame_t - v['t0'])
        v['writer'].write(frame)
        v['idx'] += 1

    @staticmethod
    def _annotate(frame, name, idx, elapsed):
        """`<name> f<idx> t<sec>` top-right. idx is 1:1 with the csv data row."""
        txt = f'{name}  f{idx}  t{elapsed:.1f}s'
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        x = max(0, frame.shape[1] - tw - 6)
        cv2.rectangle(frame, (x - 5, 0), (frame.shape[1], th + 9), (0, 0, 0), -1)
        cv2.putText(frame, txt, (x, th + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 255), 1)

    def _log_row(self, frame_t):
        if self._csv_writer is None:
            return
        s, m = self._state, self._mission
        row = [round(frame_t, 4)]
        if s is not None:
            row += [
                int(s.valid), _f(s.center_error), _f(s.center_error_cm), _f(s.ema),
                int(s.heading_valid), _f(s.heading), round(float(s.confidence), 3),
                round(float(s.left_conf), 3), round(float(s.right_conf), 3),
                s.state, int(s.used_fallback),
                int(s.n_corridors), s.ego_rule,
            ]
        else:
            row += ['', '', '', '', '', '', '', '', '', '', '', '', '']
        # Mission runs on every (frame_skip+1)-th frame, so this is the LATEST result, held
        # across the frames it did not run on -- at most `mission_skip` frames old. It is not
        # resampled to the main stream and does not pretend to be: `mission_cls` repeating
        # for three rows means three video frames shared one detection, which is exactly what
        # happened.
        if m is not None:
            row += [int(m.cls), int(m.det_cls), round(float(m.det_conf), 3),
                    int(m.det_x), int(m.det_y), int(m.det_w), int(m.det_h)]
        else:
            row += ['', '', '', '', '', '', '']
        row += [_f(self._ctrl[0]), _f(self._ctrl[1]),
                _f(self._manual[0]), _f(self._manual[1]), int(self._e_stop)]
        self._csv_writer.writerow(row)

    # ---- session ----------------------------------------------------------
    def _open_session(self):
        """One START = one timestamp shared by panel/raw/csv (same basename)."""
        try:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self.record_dir, CSV_DIR, f'{self.name_prefix}_{stamp}.csv')
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self._csv_file = open(path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'frame_time', 'valid', 'center_error', 'center_error_cm', 'ema', 'heading_valid',
                'heading', 'confidence', 'left_conf', 'right_conf', 'state',
                'used_fallback', 'n_corridors', 'ego_rule',
                # mission_cls  = CONFIRMED (M-of-N debounced) class, -1 = none.
                # mission_det_* = the RAW top detection of that frame, with its camera-pixel
                # box. Both, because only the pair distinguishes "nothing was there" from
                # "something fired and the vote rejected it".
                'mission_cls', 'mission_det_cls', 'mission_det_conf',
                'mission_det_x', 'mission_det_y', 'mission_det_w', 'mission_det_h',
                'ctrl_steering', 'ctrl_throttle',
                'manual_steering', 'manual_throttle', 'e_stop',
            ])
            self._session = stamp
            self.get_logger().info(f'Recording started: {self.name_prefix}_{stamp} '
                                   f'({self._main_dir}/' + (f' + {RAW_DIR}/' if self._dual else '')
                                   + f' + {CSV_DIR}/) in {self.record_dir}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to start recording: {exc}')
            self._close_session()

    def _open_writer(self, subdir, w, h):
        """Lazily open one mp4 writer (frame size known only at first frame)."""
        try:
            path = os.path.join(self.record_dir, subdir,
                                f'{self.name_prefix}_{self._session}.mp4')
            os.makedirs(os.path.dirname(path), exist_ok=True)
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     self.record_fps, (w, h))
            if not writer.isOpened():
                self.get_logger().error(f'Failed to open VideoWriter: {path}')
                return None
            self._vid[subdir] = {'writer': writer, 'path': path, 'idx': 0, 't0': None}
            return self._vid[subdir]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to open {subdir} writer: {exc}')
            return None

    def _close_session(self):
        for subdir, v in self._vid.items():
            v['writer'].release()
            self.get_logger().info(f'Recording saved: {v["path"]} ({v["idx"]} frames)')
        self._vid = {}
        if self._csv_file is not None:
            self._csv_file.close()
        self._csv_file = None
        self._csv_writer = None
        self._session = None

    def destroy_node(self):
        self._close_session()
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
