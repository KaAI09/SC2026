"""Driving (control) node: LaneState -> shared controller -> /control.

Consumes the perception node's LaneState, runs the shared `driving_core`
controller, and publishes control_msgs/Control **only when engaged**. Everything
else is a safety layer. This node does NOT do perception and does NOT record.

SAFETY MODEL
  * engage: a boolean parameter (default False). Autonomous commands are
    published only while `engage:=true` (toggle live via `ros2 param set`).
  * E-STOP: joystick X button (Joystick.e_stop_en) latches a stop; while
    stopped the node publishes neutral (0, 0).
  * When idle / E-stopped / low-confidence / lane lost -> throttle 0.
  * steering is clamped + slew-limited (in the controller); the node always
    publishes at publish_rate so the actuator's control_timeout watchdog stays
    fresh. Kill this node and the car coasts to the watchdog stop.

Bring up only with the wheels off the ground first, then low speed, with a
clear stop path.

Topics:
  sub : /lane/state (lane_msgs/LaneState), joystick (joystick_msgs/Joystick)
  pub : /control    (control_msgs/Control)
"""
import math
import os

import rclpy
from rclpy.node import Node
from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
from lane_msgs.msg import LaneState

from driving_core.control_core import Controller, make_ctrl
from driving_core.profile import load_profile, section


def _num(v, valid):
    """LaneState float -> Python float or None (guarding NaN / invalid flag)."""
    if not valid or v is None or math.isnan(v):
        return None
    return float(v)


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

        p = self.declare_parameter
        p('state_topic', '/lane/state')
        p('joystick_topic', 'joystick')
        p('control_topic', '/control')
        p('publish_rate', 20.0)
        p('engage', False)
        # offline-selected profile (authoritative for the fields it specifies)
        p('profile', '')
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

        gp = self.get_parameter
        state_topic = str(gp('state_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        control_topic = str(gp('control_topic').value)
        self.publish_rate = float(gp('publish_rate').value)

        ctrl_name = str(gp('controller').value)
        ctrl_kw = dict(
            kp=float(gp('kp').value), kd=float(gp('kd').value), ki=float(gp('ki').value),
            center_target=float(gp('center_target').value),
            steer_max=float(gp('steer_max').value), steer_sign=float(gp('steer_sign').value),
            slew_rate=float(gp('slew_rate').value), out_ema=float(gp('out_ema').value),
            throttle_base=float(gp('throttle_base').value),
            throttle_min=float(gp('throttle_min').value),
            curv_slow=float(gp('curv_slow').value), conf_gate=float(gp('conf_gate').value),
        )
        # profile (offline-selected) overrides the fields it specifies
        profile_path = os.path.expanduser(str(gp('profile').value))
        if profile_path:
            csec = section(load_profile(profile_path), 'control')
            ctrl_name = str(csec.pop('controller', ctrl_name))
            ctrl_kw.update(csec)
            self.get_logger().info(f'driving: loaded profile {profile_path}')
        self.controller = Controller(make_ctrl(ctrl_name, **ctrl_kw))
        self.conf_gate = float(ctrl_kw['conf_gate'])

        # state
        self.e_stop = False
        self.latest_cmd = (0.0, 0.0)
        self.prev_t = None

        self.create_subscription(LaneState, state_topic, self.state_callback, 10)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)
        self.control_pub = self.create_publisher(Control, control_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().warning(
            'driving_node up. ACTUATION: publishes /control ONLY when engage:=true. '
            'E-STOP=joystick X. Wheels off the ground first. '
            f'controller={gp("controller").value} '
            f'throttle_base={gp("throttle_base").value} steer_max={gp("steer_max").value}'
        )

    # ---- inputs -----------------------------------------------------------
    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            if not self.e_stop:
                self.get_logger().error('E-STOP latched (joystick X). Autonomous output disabled.')
            self.e_stop = True

    def state_callback(self, msg: LaneState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t

        center = _num(msg.center_error, msg.valid)
        st = {
            'center_error': center,
            'ema': _num(msg.ema, msg.valid),
            'heading': _num(msg.heading, msg.heading_valid),
            'confidence': float(msg.confidence),
        }
        steer, thr, _ = self.controller.step(st, dt)
        # node-level conservative safety: stop throttle on low conf / lost / hold
        if (center is None or msg.confidence < self.conf_gate
                or msg.state in ('LOST', 'HOLD')):
            thr = 0.0
        self.latest_cmd = (float(steer), float(thr))

    # ---- output (safety-gated) -------------------------------------------
    def publish_cmd(self):
        engage = bool(self.get_parameter('engage').value)   # live-toggleable
        if self.e_stop or not engage:
            steer, thr = 0.0, 0.0
        else:
            steer, thr = self.latest_cmd
        m = Control()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'driving'
        m.steering = float(steer)
        m.throttle = float(thr)
        self.control_pub.publish(m)

    def destroy_node(self):
        try:
            m = Control()
            m.header.stamp = self.get_clock().now().to_msg()
            m.steering = 0.0
            m.throttle = 0.0
            self.control_pub.publish(m)
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
