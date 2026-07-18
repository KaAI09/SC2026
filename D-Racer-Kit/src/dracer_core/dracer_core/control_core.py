"""ROS-independent lane-following controllers (principle fixed, all conditions
parameterized).

Mirrors the perception `perception_core` pattern: one config dataclass, controller
"modes" as presets, and a stateful `Controller.step()` that maps a lane state to a
(steering, throttle) command. The PRINCIPLE of each controller is fixed; everything
that depends on the vehicle, track, or camera/resolution is a parameter so the same
code retunes to any track.

    from dracer_core.control_core import Controller, make_ctrl
    ctrl = Controller(make_ctrl('PD', kp=0.7, center_target=-0.15))
    steer, thr, info = ctrl.step({'center_error': ce, 'ema': ema,
                                  'confidence': conf, 'state': st}, dt)

This module NEVER actuates. It only computes a command; a separate ROS node
would publish it, gated by the vehicle-safety layer.

SIGNS. center_error is the corridor centre's offset from the vehicle axis, + = the
corridor lies to the RIGHT. The law is u = -Kp*e, so a corridor to the LEFT (e < 0)
gives u > 0: on this vehicle (steer_sign = +1.0) **u > 0 steers LEFT** -- the car turns
toward the corridor, which is the only thing that can be true of a controller that laps
a track. `steer_sign` flips the emitted value for a vehicle wired the other way -- and
it is applied in exactly one place, `_emit()`.

Controllers:
  PD    steering = -(Kp*e + Kd*e_dot)                [default -- the completion runs]
  PID   steering = -(Kp*e + Kd*e_dot + Ki*integral)

PURE PURSUIT IS NOT HERE, AND THAT IS DELIBERATE. A pure-pursuit law needs the lateral
error at a lookahead point as a REAL DISTANCE. Computing it from a normalized image
coordinate gives you a "lookahead" that is not a distance and a "curvature" that is not
a curvature -- a lateral-error regulator wearing a geometry costume, i.e. a PD with
extra steps. Shipping that as an option only lets someone believe the car is doing pure
pursuit when it is not.
"""
from dataclasses import dataclass, replace


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


@dataclass
class CtrlCfg:
    name: str = 'PD'
    controller: str = 'PD'         # 'PD'|'PID'

    # --- error source / setpoint (DETECTION + TRACK dependent) ---
    use_ema: bool = True           # use smoothed center (ema) vs raw center_error
    center_target: float = 0.0     # calibrated center_error when going straight

    # --- linear gains (TUNE per track/speed) ---
    kp: float = 0.6
    kd: float = 0.15
    ki: float = 0.0
    i_clamp: float = 0.5           # anti-windup bound on the integral

    # --- vehicle / output shaping (VEHICLE dependent) ---
    steer_max: float = 1.0
    steer_sign: float = 1.0        # flip to -1.0 if steering wiring is reversed
    slew_rate_per_sec: float = 0.0    # max |d(steering)|/dt, PER SECOND (0 = off).
                                      # Per-second so steering authority does not depend on
                                      # the perception frame rate. See `step()`.
    out_ema: float = 0.0           # smoothing on the output command (0 = off)
    dt_max: float = 0.1            # dt is clamped to this before it scales anything.
                                   # The frame after a perception dropout carries a dt of
                                   # the whole gap; without a cap it would buy a slew
                                   # allowance big enough to make the limiter a no-op
                                   # exactly once -- on the least trustworthy frame there is.

    # --- throttle policy (VEHICLE + TRACK dependent) ---
    throttle_base: float = 0.18
    throttle_min: float = 0.10
    curv_slow: float = 0.0         # throttle -= curv_slow*|steering| (slow in curves)

    # --- safety gating (DETECTION dependent) ---
    conf_gate: float = 0.3         # confidence below this -> hold last steer, coast
    throttle_outlier: float = 0.0  # throttle while perception reports state == 'OUTLIER'.
                                   # NOT floored by throttle_min: with throttle_base 0.23 and
                                   # throttle_min 0.22 there is no headroom to slow down at
                                   # all, so a floored reduction would be a no-op. 0.0 = coast
                                   # (momentum carries the car, friction bleeds speed).


PRESETS = {
    'PD':  CtrlCfg(name='PD',  controller='PD'),
    'PID': CtrlCfg(name='PID', controller='PID', ki=0.05),
}


def make_ctrl(mode='PD', **overrides):
    """Build a CtrlCfg from a preset name.

    An unknown name RAISES rather than falling back to a default: a typo in the
    profile (`controller: PD2`) would otherwise put the car on a different controller
    than the one written down, silently, at speed.
    """
    if mode not in PRESETS:
        raise ValueError(f'unknown controller {mode!r}; expected one of {sorted(PRESETS)}')
    ov = {k: v for k, v in overrides.items() if v is not None}
    return replace(PRESETS[mode], **ov) if ov else PRESETS[mode]


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

    def _emit(self, u):
        """Internal command -> actuator command. `steer_sign` is applied HERE and ONLY here.

        `prev_u` holds the INTERNAL value, not the emitted (sign-applied) one. If it held the
        emitted value, the low-confidence hold would read it back and multiply by `steer_sign`
        a SECOND time -- so on a steer_sign=-1 vehicle "hold the last command" would invert it
        every frame instead. Keeping `prev_u` internal is what keeps the hold correct.
        """
        return u * self.cfg.steer_sign

    def step(self, st, dt):
        c = self.cfg
        center = self._center(st)
        conf = st.get('confidence', 1.0)
        state = str(st.get('state') or 'OK')
        if center is None:
            # no lane -> keep last steering, coast slow (safety layer decides more)
            return self._emit(self.prev_u), c.throttle_min, {'gated': 'no_center'}

        # dt drives BOTH the derivative and the slew limit, and both are wrong on the
        # first frame after a gap: the derivative divides by it (harmless -- a big dt
        # shrinks de) and the slew limit MULTIPLIES by it (not harmless -- a 0.3s gap
        # would grant a 1.35 allowance and let the command jump anywhere it likes).
        dt_s = min(max(dt, 0.0), c.dt_max)

        e = center - c.center_target
        de = 0.0 if (self.prev_e is None or dt_s <= 0) else (e - self.prev_e) / dt_s
        self.prev_e = e

        if c.controller == 'PD':
            u = -(c.kp * e + c.kd * de)
        elif c.controller == 'PID':
            self.integ = clamp(self.integ + e * dt_s, -c.i_clamp, c.i_clamp)
            u = -(c.kp * e + c.kd * de + c.ki * self.integ)
        else:
            # make_ctrl() rejects unknown names, so reaching this means the cfg was
            # built by hand. Refuse to steer rather than invent a command.
            raise ValueError(f'unknown controller {c.controller!r}')

        gated = None
        if conf < c.conf_gate:
            u = self.prev_u          # low confidence -> hold last command (internal space)
            gated = 'low_conf_hold'

        # Saturation / slew / smoothing all act on the INTERNAL command, so they stay
        # symmetric and `steer_sign` never enters the state. Emitted once, at the bottom.
        u = clamp(u, -c.steer_max, c.steer_max)
        if c.slew_rate_per_sec > 0 and dt_s > 0:
            # PER SECOND, not per callback. A per-step limit would make the car's maximum
            # turn rate depend on the perception frame rate:
            #
            #     0.15/step @ 10.7Hz = 1.6 /s      (full swing 1.0s)
            #     0.15/step @ 30Hz   = 4.5 /s      (full swing 0.36s)
            #
            # so a faster camera would silently make the steering more agile, and a drop to
            # 20Hz would quietly cut the turn rate to 3.0/s -- the car physically could not
            # corner as hard. Pinning the PHYSICAL rate reproduces the same car at any frame
            # rate.
            slew = c.slew_rate_per_sec * dt_s
            u = clamp(u, self.prev_u - slew, self.prev_u + slew)
        if c.out_ema > 0:
            u = c.out_ema * self.prev_u + (1.0 - c.out_ema) * u
        self.prev_u = u

        thr = c.throttle_base - c.curv_slow * abs(u)
        thr = max(c.throttle_min, thr)
        if conf < c.conf_gate:
            thr = c.throttle_min
        if state == 'OUTLIER':
            # Perception is telling us its own EMA is not to be trusted: the raw measurement
            # has disagreed past `outlier_jump` and has not yet been believed (see
            # perception_core._Stabilizer). We are steering on a value that is stale and --
            # after a corridor flip -- WRONG-SIGNED, for up to `outlier_relatch_s`.
            #
            # This guards against inverted steering while perception is reporting the truth
            # the whole time -- steering on a stale, wrong-signed value. Perception knew;
            # this makes the control side listen.
            #
            # Steering needs no extra hold: a frozen EMA is a constant `e`, so `de` decays to
            # zero and u is already held. What we buy here is GROUND NOT COVERED.
            thr = min(thr, c.throttle_outlier)
            gated = gated or 'outlier_slow'

        return self._emit(u), thr, {'e': e, 'de': de, 'state': state, 'gated': gated}
