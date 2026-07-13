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
  * PERCEPTION WATCHDOG (`state_timeout`): no /lane/state for this long -> publish
    neutral (0, 0). See below -- this is the one that keeps the car on the ground.
  * JOYSTICK WATCHDOG (`joystick_timeout`): no /joystick for this long -> the joystick
    engage is forced OFF (the `engage` param is untouched). `js_engage` mirrors a message
    field, so without this it stays True forever once the messages stop -- the car drives
    on with nobody holding the pad and no button left to press.
    CAVEAT, and it is a real one: this fires when joystick_node DIES. It does NOT fire when
    the PAD is merely unplugged, because joystick_node's read loop swallows the error and
    its timer keeps republishing the last input at 50Hz -- engage flag and all. /joystick
    stays perfectly fresh. Closing that needs joystick_node to stop asserting a pad it can
    no longer read; it is not fixed here.
  * steering is clamped + slew-limited (in the controller); the node always
    publishes at publish_rate so the actuator's control_timeout watchdog stays
    fresh. Kill this node and the car coasts to the watchdog stop.

WHY THE PERCEPTION WATCHDOG EXISTS
  The actuator has a dead-man on /control, and this node feeds it at publish_rate
  from a timer -- from `latest_cmd`, which only `state_callback` ever writes. So if
  the CAMERA or PERCEPTION dies, /control does not stop: this node keeps republishing
  the last command it ever computed, at full rate, forever. It is perfectly fresh, and
  it is a steering angle and a throttle from a world that no longer exists. The
  actuator's watchdog cannot see the difference and never fires.

      camera/perception dies -> /lane/state stops -> latest_cmd freezes
      -> /control stays FRESH -> actuator watchdog never fires
      -> car drives away on its last command

  A dead-man is only as good as the staleness it can actually observe. The actuator
  watches its own input; nobody was watching ours.

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
_CTRL_FLOATS = ('kp', 'kd', 'ki', 'i_clamp', 'center_target', 'steer_max', 'steer_sign',
                'slew_rate_per_sec', 'out_ema', 'throttle_base', 'throttle_min',
                'curv_slow', 'conf_gate', 'throttle_outlier', 'dt_max')
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
        # >= the fastest perception can run. At 20Hz this timer sampled a `latest_cmd` that
        # perception was rewriting at 30Hz: a third of every command computed was overwritten
        # before it was ever published, and the two free-running clocks beat against each
        # other so the effective delay wandered. Publishing FASTER than perception costs
        # nothing -- an unchanged value is simply republished -- so the rate should track the
        # ceiling, not the average.
        p('publish_rate', 30.0)
        p('engage', False)
        # Perception dead-man. 0.25s = ~8 frames at the 30Hz perception runs at; a gap that
        # long is already an anomaly, not jitter (measured worst gap over the 0711 runs:
        # 38ms, across 1502 frames, zero drops). Must stay well UNDER the actuator's own
        # control_timeout (0.5s) -- ours is the only watchdog that can see this failure, so
        # it has to fire first. <= 0 disables (do not).
        p('state_timeout', 0.25)
        # Joystick dead-man. `js_engage` MIRRORS Joystick.engage, so it holds its last value
        # forever if the messages stop -- and if that value was True, the car keeps driving
        # with nobody holding the pad. Same failure as the perception one above: a flag that
        # only ever gets refreshed, never expired. joystick_node publishes at 50Hz, so 0.3s
        # is 15 missed messages -- a disconnection, not jitter.
        # This expires the JOYSTICK engage only; the `engage` PARAM path (headless bring-up
        # with no pad at all) is untouched.
        p('joystick_timeout', 0.3)
        # offline-selected profile (authoritative for the fields it specifies)
        p('profile', '')
        # control (PD defaults, conservative)
        p('controller', 'PD')   # 'PD' | 'PID' -- an unknown name raises at build time
        p('kp', 0.5)
        p('kd', 0.1)
        p('ki', 0.0)
        p('i_clamp', 0.5)       # PID anti-windup bound (unused by PD)
        p('center_target', 0.0)
        p('steer_max', 1.0)   # the trim lives in the servo centre now -> +-1.0 is symmetric
        p('steer_sign', 1.0)
        p('slew_rate_per_sec', 7.5)   # 0712 ran 187.5 deg/s; at the post-A1 scale (25 deg/u)
                                      # that is 7.5 u/s. It is a u-rate, not a degree-rate, so
                                      # the servo rescale moves it exactly like kp.
        p('dt_max', 0.1)
        p('out_ema', 0.0)
        p('throttle_base', 0.13)
        p('throttle_min', 0.0)
        p('curv_slow', 0.0)
        p('conf_gate', 0.4)
        p('throttle_outlier', 0.0)

        gp = self.get_parameter
        state_topic = str(gp('state_topic').value)
        joystick_topic = str(gp('joystick_topic').value)
        control_topic = str(gp('control_topic').value)
        self.publish_rate = float(gp('publish_rate').value)
        self.state_timeout = float(gp('state_timeout').value)
        self.joystick_timeout = float(gp('joystick_timeout').value)

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
        self.last_state_time = None   # wall time of the last /lane/state (perception dead-man)
        self.state_stale = True       # nothing received yet -> not safe to drive
        self.last_js_time = None      # wall time of the last /joystick (joystick dead-man)
        self.js_stale = False         # True only after a joystick we HAD went silent

        self.create_subscription(LaneState, state_topic, self.state_callback, 10)
        self.create_subscription(Joystick, joystick_topic, self.joystick_callback, 10)
        self.control_pub = self.create_publisher(Control, control_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().warning(
            'control_node up. ACTUATION: publishes /control ONLY when engaged '
            '(engage:=true OR joystick A). E-STOP=joystick X. Wheels off the ground first. '
            f'controller={gp("controller").value} '
            f'throttle_base={gp("throttle_base").value} steer_max={gp("steer_max").value} '
            f'state_timeout={self.state_timeout}s '
            f'joystick_timeout={self.joystick_timeout}s '
            f'throttle_outlier={gp("throttle_outlier").value}'
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
        self.last_js_time = self.get_clock().now()
        if self.js_stale:
            self.js_stale = False
            self.get_logger().warning(
                'joystick recovered. engage stays OFF until you press A again.')
        if bool(msg.e_stop_en):
            if not self.e_stop:
                self.get_logger().error('E-STOP latched (joystick X). Autonomous output disabled.')
            self.e_stop = True
        self.js_engage = bool(msg.engage)

    def _joystick_is_stale(self):
        """Has the joystick gone silent? Only meaningful once we have actually seen one --
        a headless bring-up (engage param, no pad) must not be treated as a lost pad."""
        if self.joystick_timeout <= 0.0 or self.last_js_time is None:
            return False
        age = (self.get_clock().now() - self.last_js_time).nanoseconds / 1e9
        return age > self.joystick_timeout

    def state_callback(self, msg: LaneState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t
        first = self.last_state_time is None
        self.last_state_time = self.get_clock().now()
        if self.state_stale and not first:
            # Coming back from a perception dropout. Do NOT resume the controller's
            # internal state across the gap: prev_u is the command from before the
            # blackout, and prev_e would make the first derivative a step over a hole of
            # unknown length. Restart from neutral and let the slew limit walk it back up.
            self.controller.reset()
            self.get_logger().warning('lane state recovered — controller reset, resuming.')
        self.state_stale = False

        center = _num(msg.center_error, msg.valid)
        st = {
            'center_error': center,
            'ema': _num(msg.ema, msg.valid),
            'heading': _num(msg.heading, msg.heading_valid),
            'confidence': float(msg.confidence),
            'state': str(msg.state),      # OUTLIER -> throttle_outlier (control_core)
        }
        steer, thr, _ = self.controller.step(st, dt)
        # node-level conservative safety: stop throttle on low conf / lost / hold
        if (center is None or msg.confidence < self.conf_gate
                or msg.state in ('LOST', 'HOLD')):
            thr = 0.0
        self.latest_cmd = (float(steer), float(thr))

    def _state_is_stale(self):
        """Has perception stopped talking to us? See the module docstring."""
        if self.state_timeout <= 0.0:
            return False
        if self.last_state_time is None:
            return True               # never heard from perception -> do not drive
        age = (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9
        return age > self.state_timeout

    # ---- output (safety-gated) -------------------------------------------
    def publish_cmd(self):
        # Expire the joystick engage BEFORE it is read. `js_engage` is a mirror of a message
        # field, so silence leaves it frozen at whatever it last was -- and if that was True
        # the car drives on with nobody holding the pad, and no button left to press. Latch
        # it off: a recovered joystick does NOT re-engage by itself, the driver presses A.
        if self._joystick_is_stale() and not self.js_stale:
            self.js_stale = True
            if self.js_engage:
                self.get_logger().error(
                    f'JOYSTICK STALE: no /joystick for >{self.joystick_timeout:.2f}s while '
                    'ENGAGED. Forcing engage OFF. Press A to re-engage once it is back.')
            self.js_engage = False

        # engage if EITHER the param (ros2 param set) OR the joystick A-button is on
        engage = bool(self.get_parameter('engage').value) or self.js_engage
        stale = self._state_is_stale()
        if stale and not self.state_stale and engage:
            self.get_logger().error(
                f'PERCEPTION STALE: no /lane/state for >{self.state_timeout:.2f}s. '
                'Publishing neutral (0, 0). The car will NOT keep driving on the last '
                'command — check camera_node / perception_node.')
        if stale:
            self.state_stale = True

        if self.e_stop or not engage or stale:
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
