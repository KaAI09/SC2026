"""Driving (control) node: LaneState -> shared controller -> /control.

Consumes the perception node's LaneState, runs the shared `dracer_core`
controller, and publishes dracer_msgs/Control **only when engaged**. Everything
else is a safety layer. This node does NOT do perception and does NOT record.

SAFETY MODEL
  * engage: autonomous commands are published only while engaged. Two OR'd
    sources: the `engage` parameter (default False, `ros2 param set`) OR the
    joystick A button (Joystick.engage, toggles live). Either enables output.
  * E-STOP: joystick X button (Joystick.e_stop_en) latches a stop; while
    stopped the node publishes neutral (0, 0) and A-engage is forced off.
  * When idle / E-stopped / low-confidence / lane lost -> throttle 0.
  * steering is clamped + slew-limited (in the controller); the node always
    publishes at publish_rate so the actuator's control_timeout watchdog stays
    fresh. Kill this node and the car coasts to the watchdog stop.

Bring up only with the wheels off the ground first, then low speed, with a
clear stop path.

Topics:
  sub : /lane/state (dracer_msgs/LaneState), joystick (dracer_msgs/Joystick)
  pub : /control    (dracer_msgs/Control)
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from dracer_msgs.msg import Control
from dracer_msgs.msg import Joystick
from dracer_msgs.msg import LaneState

from dracer_core.control_core import Controller, make_ctrl
from dracer_core.profile import load_profile, section


# CtrlCfg fields exposed as live-tunable ROS params (rebuilt on `ros2 param set`).
_CTRL_FLOATS = ('kp', 'kd', 'ki', 'center_target', 'steer_max', 'steer_sign',
                'slew_rate', 'out_ema', 'throttle_base', 'throttle_min',
                'curv_slow', 'conf_gate')
_CTRL_PARAMS = ('controller',) + _CTRL_FLOATS


def _num(v, valid):
    """LaneState float -> Python float or None (guarding NaN / invalid flag)."""
    if not valid or v is None or math.isnan(v):
        return None
    return float(v)


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

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

        # profile (offline-selected) -> push into the ROS params so params are the
        # single source of truth; then live `ros2 param set` tuning stays correct.
        profile_path = os.path.expanduser(str(gp('profile').value))
        if profile_path:
            self._apply_profile_params(section(load_profile(profile_path), 'control'))
            self.get_logger().info(f'control: loaded profile {profile_path}')
        self.controller, self.conf_gate = self._build_controller()
        # live tuning: rebuild the controller whenever a control param is set
        self.add_on_set_parameters_callback(self._on_set_params)

        # state
        self.e_stop = False
        self.js_engage = False        # joystick A-button engage (OR'd with param)
        self.latest_cmd = (0.0, 0.0)
        self.prev_t = None

        self.create_subscription(LaneState, state_topic, self.state_callback, 10)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)
        self.control_pub = self.create_publisher(Control, control_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().warning(
            'control_node up. ACTUATION: publishes /control ONLY when engaged '
            '(engage:=true OR joystick A). E-STOP=joystick X. Wheels off the ground first. '
            f'controller={gp("controller").value} '
            f'throttle_base={gp("throttle_base").value} steer_max={gp("steer_max").value}'
        )

    # ---- control params (profile load + live tuning) ---------------------
    def _apply_profile_params(self, csec):
        """Push a profile [control] section into the declared ROS params."""
        updates = []
        for k, v in csec.items():
            if k == 'controller':
                updates.append(Parameter('controller', Parameter.Type.STRING, str(v)))
            elif k in _CTRL_FLOATS:
                updates.append(Parameter(k, Parameter.Type.DOUBLE, float(v)))
        if updates:
            self.set_parameters(updates)

    def _build_controller(self, overrides=None):
        """Build the Controller from current param values (with optional pending
        overrides applied on top, for the pre-set callback)."""
        ov = overrides or {}
        gp = self.get_parameter

        def val(name):
            return ov[name] if name in ov else gp(name).value
        kw = {k: float(val(k)) for k in _CTRL_FLOATS}
        return Controller(make_ctrl(str(val('controller')), **kw)), kw['conf_gate']

    def _on_set_params(self, params):
        """Live: rebuild the controller when any control param is set."""
        if any(p.name in _CTRL_PARAMS for p in params):
            ov = {p.name: p.value for p in params if p.name in _CTRL_PARAMS}
            self.controller, self.conf_gate = self._build_controller(ov)
            self.get_logger().info(f'control live-update: {ov}')
        return SetParametersResult(successful=True)

    # ---- inputs -----------------------------------------------------------
    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            if not self.e_stop:
                self.get_logger().error('E-STOP latched (joystick X). Autonomous output disabled.')
            self.e_stop = True
        self.js_engage = bool(msg.engage)

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
        # engage if EITHER the param (ros2 param set) OR the joystick A-button is on
        engage = bool(self.get_parameter('engage').value) or self.js_engage
        if self.e_stop or not engage:
            steer, thr = 0.0, 0.0
        else:
            steer, thr = self.latest_cmd
        m = Control()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'control'
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
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
