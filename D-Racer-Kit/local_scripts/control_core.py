"""ROS-independent lane-following controllers (principle fixed, all conditions
parameterized).

Mirrors the perception `lane_core` pattern: one config dataclass, controller
"modes" as presets C1..C5, and a stateful `Controller.step()` that maps a lane
state to a (steering, throttle) command. The PRINCIPLE of each controller is
fixed; everything that depends on the vehicle, track, or camera/resolution is a
parameter so the same code retunes to any track.

    from control_core import Controller, make_ctrl
    ctrl = Controller(make_ctrl('C2', kp=0.7, center_target=-0.15))
    steer, thr, info = ctrl.step({'center_error': ce, 'ema': ema,
                                  'heading': hd_deg, 'confidence': conf,
                                  'speed': throttle_proxy}, dt)

This module NEVER actuates. It only computes a command; a separate ROS node
would publish it, gated by the vehicle-safety layer. Sign conventions match the
recorded data (center_error < 0  ->  steer right/+), and can be flipped per
vehicle with `steer_sign`.

Controllers (C6 bang-bang intentionally omitted; C7 learning is optional/later):
  C1 P            steering = -Kp*e
  C2 PD           steering = -(Kp*e + Kd*e_dot)                [RC-car baseline]
  C3 PID          steering = -(Kp*e + Kd*e_dot + Ki*integral)
  C4 PurePursuit  geometry: curvature to a lookahead lane point
  C5 Stanley      cross-track + heading (needs a reliable heading)
"""
import math
from dataclasses import dataclass, replace


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


@dataclass
class CtrlCfg:
    name: str = 'C1'
    controller: str = 'P'          # 'P'|'PD'|'PID'|'PURE_PURSUIT'|'STANLEY'

    # --- error source / setpoint (DETECTION + TRACK dependent) ---
    use_ema: bool = True           # use smoothed center (ema) vs raw center_error
    center_target: float = 0.0     # calibrated center_error when going straight

    # --- linear gains (TUNE per track/speed) ---
    kp: float = 0.6
    kd: float = 0.15
    ki: float = 0.0
    i_clamp: float = 0.5           # anti-windup bound on the integral

    # --- pure pursuit (GEOMETRY; lookahead in normalized view-depth 0..1) ---
    lookahead: float = 0.6
    pp_gain: float = 0.4

    # --- stanley ---
    stanley_k: float = 1.0
    stanley_soft: float = 0.15     # softening term (avoids blow-up at low speed)
    heading_gain: float = 1.0

    # --- vehicle / output shaping (VEHICLE dependent) ---
    steer_max: float = 1.0
    steer_sign: float = 1.0        # flip to -1.0 if steering wiring is reversed
    slew_rate: float = 0.0         # max |delta steering| per step (0 = off)
    out_ema: float = 0.0           # smoothing on the output command (0 = off)

    # --- throttle policy (VEHICLE + TRACK dependent) ---
    throttle_base: float = 0.18
    throttle_min: float = 0.10
    curv_slow: float = 0.0         # throttle -= curv_slow*|steering| (slow in curves)

    # --- safety gating (DETECTION dependent) ---
    conf_gate: float = 0.3         # confidence below this -> hold last steer, coast


PRESETS = {
    'C1': CtrlCfg(name='C1 P',           controller='P'),
    'C2': CtrlCfg(name='C2 PD',          controller='PD'),
    'C3': CtrlCfg(name='C3 PID',         controller='PID', ki=0.05),
    'C4': CtrlCfg(name='C4 PurePursuit', controller='PURE_PURSUIT'),
    'C5': CtrlCfg(name='C5 Stanley',     controller='STANLEY'),
    # C6 bang-bang: intentionally omitted.
    # C7 learning/behavior-cloning: optional, trained on-site later.
}


def make_ctrl(mode='C1', **overrides):
    base = PRESETS.get(mode, PRESETS['C1'])
    ov = {k: v for k, v in overrides.items() if v is not None}
    return replace(base, **ov) if ov else base


class Controller:
    """Stateful controller. Feed one lane state per frame with the frame dt."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.prev_e = None
        self.integ = 0.0
        self.prev_u = 0.0

    def _center(self, st):
        c = self.cfg
        if c.use_ema and st.get('ema') is not None:
            return st['ema']
        return st.get('center_error')

    def step(self, st, dt):
        c = self.cfg
        center = self._center(st)
        conf = st.get('confidence', 1.0)
        if center is None:
            # no lane -> keep last steering, coast slow (safety layer decides more)
            return self.prev_u, c.throttle_min, {'gated': 'no_center'}

        e = center - c.center_target
        de = 0.0 if (self.prev_e is None or dt <= 0) else (e - self.prev_e) / dt
        self.prev_e = e
        heading = math.radians(st.get('heading') or 0.0)
        speed = max(1e-3, float(st.get('speed', c.throttle_base)))

        if c.controller == 'P':
            u = -(c.kp * e)
        elif c.controller == 'PD':
            u = -(c.kp * e + c.kd * de)
        elif c.controller == 'PID':
            self.integ = clamp(self.integ + e * dt, -c.i_clamp, c.i_clamp)
            u = -(c.kp * e + c.kd * de + c.ki * self.integ)
        elif c.controller == 'PURE_PURSUIT':
            # lateral offset of the lane at the lookahead, projected via heading;
            # pure-pursuit curvature ~ 2*x_L / Ld^2  (normalized image geometry)
            x_l = e + math.tan(heading) * c.lookahead
            u = -c.pp_gain * (2.0 * x_l) / (c.lookahead ** 2 + 1e-6)
        elif c.controller == 'STANLEY':
            cross = math.atan2(c.stanley_k * e, c.stanley_soft + speed)
            u = -(c.heading_gain * heading + cross)
        else:
            u = 0.0

        gated = None
        if conf < c.conf_gate:
            u = self.prev_u          # low confidence -> hold last command
            gated = 'low_conf_hold'

        u *= c.steer_sign
        u = clamp(u, -c.steer_max, c.steer_max)
        if c.slew_rate > 0:
            u = clamp(u, self.prev_u - c.slew_rate, self.prev_u + c.slew_rate)
        if c.out_ema > 0:
            u = c.out_ema * self.prev_u + (1.0 - c.out_ema) * u
        self.prev_u = u

        thr = c.throttle_base - c.curv_slow * abs(u)
        thr = max(c.throttle_min, thr)
        if conf < c.conf_gate:
            thr = c.throttle_min

        return u, thr, {'e': e, 'de': de, 'gated': gated}
