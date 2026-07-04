"""Closed-loop lane-following node (perception + control).

Runs the shared lane_core perception and control_core controller, then publishes
control_msgs/Control to /control **only when engaged**. Everything else is a
safety layer.

SAFETY MODEL
  * engage: a boolean parameter (default False). Autonomous commands are
    published only while `engage:=true` (toggle live via `ros2 param set`).
  * E-STOP: joystick X button (Joystick.e_stop_en) latches a stop here and in
    control_node; while stopped the node publishes neutral (0, 0).
  * When idle / E-stopped / low-confidence / lane lost -> throttle 0.
  * steering is clamped (steer_max) and slew-rate limited; throttle is a low
    fixed base. The node always publishes at publish_rate so control_node's
    control_timeout watchdog stays fresh; kill the node and the car stops.
  * START button records the overlay + a CSV of the AUTONOMOUS command.

This node DOES actuate. Bring it up only with the wheels off the ground first,
then low speed, with a clear stop path.

Topics:
  sub : /camera/image/compressed (CompressedImage), joystick (Joystick)
  pub : /control (Control), /lane/debug/compressed (CompressedImage)
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
from control_msgs.msg import Control
from joystick_msgs.msg import Joystick

from opencv.lane_core import LanePipeline, make_cfg
from opencv.control_core import Controller, make_ctrl


class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow_node')

        p = self.declare_parameter
        p('subscribe_topic', '/camera/image/compressed')
        p('joystick_topic', 'joystick')
        p('control_topic', '/control')
        p('debug_topic', '/lane/debug/compressed')
        p('publish_rate', 20.0)
        p('engage', False)
        # perception
        p('mode', 'O2')
        p('roi_top_frac', -1.0)
        p('trap_top_w', -1.0)
        p('trap_bot_w', -1.0)
        p('lane_width_default', -1.0)
        p('orange_s_min', -1)
        p('orange_v_min', -1)
        # control (C2 PD defaults, conservative)
        p('controller', 'C2')
        p('kp', 0.5)
        p('kd', 0.1)
        p('ki', 0.0)
        p('center_target', 0.0)
        p('steer_max', 0.8)
        p('steer_sign', 1.0)
        p('slew_rate', 0.15)
        p('out_ema', 0.0)
        p('throttle_base', 0.13)
        p('throttle_min', 0.0)
        p('curv_slow', 0.0)
        p('conf_gate', 0.4)
        # recording / debug
        p('record', True)
        p('record_dir', '')
        p('record_fps', 30.0)
        p('jpeg_quality', 80)
        p('debug_scale', 2.0)
        p('publish_debug', True)

        gp = self.get_parameter
        subscribe_topic = str(gp('subscribe_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        control_topic = str(gp('control_topic').value)
        self.debug_topic = str(gp('debug_topic').value)
        self.publish_rate = float(gp('publish_rate').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.debug_scale = float(gp('debug_scale').value)
        self.publish_debug = bool(gp('publish_debug').value)
        self.record_enabled = bool(gp('record').value)
        self.record_dir = os.path.expanduser(str(gp('record_dir').value)) or os.getcwd()
        self.record_fps = float(gp('record_fps').value)

        # perception config
        pov = {}
        if float(gp('roi_top_frac').value) >= 0:
            pov['roi_top_frac'] = float(gp('roi_top_frac').value)
        if float(gp('trap_top_w').value) >= 0:
            pov['trap_top_w'] = float(gp('trap_top_w').value)
        if float(gp('trap_bot_w').value) >= 0:
            pov['trap_bot_w'] = float(gp('trap_bot_w').value)
        if float(gp('lane_width_default').value) >= 0:
            pov['lane_width_default'] = float(gp('lane_width_default').value)
        if int(gp('orange_s_min').value) >= 0:
            pov['orange_s_min'] = int(gp('orange_s_min').value)
        if int(gp('orange_v_min').value) >= 0:
            pov['orange_v_min'] = int(gp('orange_v_min').value)
        self.mode = str(gp('mode').value)
        self.pipeline = LanePipeline(make_cfg(self.mode, **pov))

        # control config
        self.controller = Controller(make_ctrl(
            str(gp('controller').value),
            kp=float(gp('kp').value), kd=float(gp('kd').value), ki=float(gp('ki').value),
            center_target=float(gp('center_target').value),
            steer_max=float(gp('steer_max').value), steer_sign=float(gp('steer_sign').value),
            slew_rate=float(gp('slew_rate').value), out_ema=float(gp('out_ema').value),
            throttle_base=float(gp('throttle_base').value),
            throttle_min=float(gp('throttle_min').value),
            curv_slow=float(gp('curv_slow').value), conf_gate=float(gp('conf_gate').value),
        ))
        self.conf_gate = float(gp('conf_gate').value)

        # state
        self.e_stop = False
        self.latest_cmd = (0.0, 0.0)
        self.latest_state = None
        self.prev_t = None
        self._want_record = False
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, subscribe_topic, self.image_callback, image_qos)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)
        self.control_pub = self.create_publisher(Control, control_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, self.debug_topic, image_qos)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().warning(
            'lane_follow_node up. ACTUATION: publishes /control ONLY when engage:=true. '
            'E-STOP=joystick X. Wheels off the ground first. '
            f'perception={self.mode} controller={gp("controller").value} '
            f'throttle_base={gp("throttle_base").value} steer_max={gp("steer_max").value}'
        )

    # ---- inputs -----------------------------------------------------------
    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            if not self.e_stop:
                self.get_logger().error('E-STOP latched (joystick X). Autonomous output disabled.')
            self.e_stop = True
        self._want_record = bool(msg.is_recording)

    def image_callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed image')
            return

        overlay, state = self.pipeline.process(frame)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t

        st = {'center_error': state['center_error'], 'ema': state['ema'],
              'heading': state['heading'], 'confidence': state['confidence']}
        steer, thr, _ = self.controller.step(st, dt)
        # node-level conservative safety: stop throttle on low conf / lost / hold
        if (state['center_error'] is None or state['confidence'] < self.conf_gate
                or state['state'] in ('LOST', 'HOLD')):
            thr = 0.0
        self.latest_cmd = (float(steer), float(thr))
        self.latest_state = state

        out = self._annotate(overlay, steer, thr)
        if self.publish_debug:
            self._publish_debug(out, msg.header.stamp)
        self._handle_recording(out, state, steer, thr, t)

    # ---- output (safety-gated) -------------------------------------------
    def publish_cmd(self):
        engage = bool(self.get_parameter('engage').value)   # live-toggleable
        if self.e_stop or not engage:
            steer, thr = 0.0, 0.0
        else:
            steer, thr = self.latest_cmd
        m = Control()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'lane_follow'
        m.steering = float(steer)
        m.throttle = float(thr)
        self.control_pub.publish(m)

    # ---- overlay / debug --------------------------------------------------
    def _annotate(self, overlay, steer, thr):
        img = overlay
        engage = bool(self.get_parameter('engage').value)
        if self.e_stop:
            tag, col = 'E-STOP', (0, 0, 255)
        elif engage:
            tag, col = 'AUTO ENGAGED', (0, 200, 0)
        else:
            tag, col = 'IDLE (engage:=false)', (0, 180, 255)
        h = overlay.shape[0]
        cv2.putText(img, f'{tag}  steer={steer:+.2f} thr={thr:.2f}',
                    (2, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1)
        return img

    def _publish_debug(self, out, stamp):
        if self.debug_scale and self.debug_scale != 1.0:
            out = cv2.resize(out, None, fx=self.debug_scale, fy=self.debug_scale,
                             interpolation=cv2.INTER_NEAREST)
        ok, enc = cv2.imencode('.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            m = CompressedImage()
            m.header.stamp = stamp
            m.header.frame_id = 'lane_follow_debug'
            m.format = 'jpeg'
            m.data = enc.tobytes()
            self.debug_pub.publish(m)

    # ---- recording (overlay mp4 + autonomous-command CSV) -----------------
    def _handle_recording(self, frame, state, steer, thr, t):
        if not self.record_enabled:
            return
        if self._want_record and self._writer is None:
            self._start_writer(frame.shape[1], frame.shape[0])
        elif not self._want_record and self._writer is not None:
            self._stop_writer()
        if self._writer is not None:
            self._writer.write(frame)
            if self._csv_writer is not None:
                engage = bool(self.get_parameter('engage').value)

                def n(v):
                    return '' if v is None else round(float(v), 4)
                self._csv_writer.writerow([
                    round(t, 4), n(state['center_error']), n(state['ema']), n(state['heading']),
                    round(state['confidence'], 3), state['state'],
                    int(engage), int(self.e_stop), round(steer, 4), round(thr, 4),
                ])

    def _start_writer(self, w, h):
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._rec_path = os.path.join(self.record_dir, f'follow_{self.mode}_{stamp}.mp4')
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
                'frame_time', 'center_error', 'ema', 'heading', 'confidence', 'state',
                'engaged', 'e_stop', 'auto_steering', 'auto_throttle',
            ])
            self.get_logger().info(f'Follow recording started: {self._rec_path} (+ .csv)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to start recording: {exc}')
            self._writer = None
            self._csv_file = None
            self._csv_writer = None

    def _stop_writer(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Follow recording saved: {self._rec_path} (+ .csv)')
        if self._csv_file is not None:
            self._csv_file.close()
        self._writer = None
        self._rec_path = None
        self._csv_file = None
        self._csv_writer = None

    def destroy_node(self):
        # best-effort: command neutral on shutdown
        try:
            m = Control()
            m.header.stamp = self.get_clock().now().to_msg()
            m.steering = 0.0
            m.throttle = 0.0
            self.control_pub.publish(m)
        except Exception:  # noqa: BLE001
            pass
        self._stop_writer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
