"""Pure (ROS-independent) lane-detection pipeline. Single source of truth.

This is the FRONT-VIEW port of the offline 7-label probe (offline/lane7_probe.py):
detect -> sliding-window multi-lane tracking -> 7-label classification -> ego
corridor centerline (+ Tracker/coast). The ONE difference from the probe is that
the probe's perspective warp (an uncalibrated ROI-trapezoid -> rectangle
homography, a rough BEV) is REMOVED: the sliding window runs directly on the
ROI-cropped front-view mask, so lane polynomials x=f(y) are already in image
coordinates (no warp-back needed to draw).

A properly camera-calibrated BEV is a LATER, separate stage. When it lands, warp
the color masks right before `sliding_window_lanes` and unwarp the fitted coeffs
for drawing; every stage after detection is coordinate-agnostic and stays as-is.
The seam is marked in `LanePipeline.process`.

Imported by BOTH the online perception node and the offline control tools
(offline/control_predict.py). Depends only on cv2 + numpy.

Usage:
    from dracer_core.perception_core import LanePipeline, Cfg, cfg_from_profile
    pipe = LanePipeline(cfg_from_profile(profile['perception']))   # or Cfg()
    overlay_bgr, state = pipe.process(frame_bgr)   # state: dict (center_error, ...)

The pipeline NEVER commands the vehicle; it only produces a lane-state estimate
(consumed by the control stage) and a debug overlay.
"""
from collections import deque
from dataclasses import dataclass, fields, replace
import math

import cv2
import numpy as np


# ==========================================================================
# Config + presets (single source; imported by online node + offline tools)
# ==========================================================================
@dataclass
class Cfg:
    name: str = 'lane'
    # A: segmentation — WHITE + YELLOW; `colors` is a subset, masks OR-combined.
    colors: tuple = ('white', 'yellow')
    white_s_max: int = 60
    white_v_min: int = 185
    yellow_h_lo: int = 18
    yellow_h_hi: int = 36
    yellow_s_min: int = 65
    yellow_v_min: int = 100
    morph_v: int = 5             # vertical CLOSE kernel (bridge dashed gaps)
    color_gate: float = 0.15     # minority-color fraction < this -> drop it wholesale
    gate_min_px: int = 80
    # B: ROI trapezoid (front-view crop; was the probe's BEV source region)
    roi_top_frac: float = 0.30
    trap_top_w: float = 0.80
    trap_bot_w: float = 1.0
    # C: sliding window (image coords)
    sw_nwin: int = 12            # vertical window count
    sw_margin: int = 28          # window half-width (px)
    sw_minpix: int = 18          # min pixels in a window to recenter
    sw_max_miss: int = 3         # consecutive empty windows -> lane ends
    sw_dir_ema: float = 0.6      # per-window lateral-step EMA (curve following)
    sw_min_hits: int = 2         # fewer successful windows -> not a lane
    sw_min_span: float = 0.30    # vertical span < this*H -> not a lane (stop line)
    sw_max_lanes: int = 3        # max base peaks per color
    sw_peak_min: int = 10        # min histogram peak (rows)
    sw_peak_sep: int = 45        # min peak separation (px)
    merge_dx: float = 30.0       # two fits with MAX |dx| over their y-overlap < this -> same lane
    # D: classification (per-instance heading + curvature)
    heading_frac: float = 0.06   # |top-bottom x| >= this*W -> turn by heading
    curv_strong: float = 0.0015  # near-vertical but |a| >= this -> turn by curvature
    straight_thresh: float = 0.0006
    # pairing (adjacent left/right boundary -> centerline)
    pair_overlap_min: float = 0.30
    pair_gap_min: float = 12.0
    # E: tracker (ego L/R persistence + width coast)
    lane_width_default: float = 0.5
    jump_max: float = 120.0
    lost_reset: int = 8
    # scalar output stabilizer (smooths center_error + names the failsafe state)
    ema_alpha: float = 0.4
    outlier_jump: float = 0.5
    use_median: bool = False
    median_window: int = 5
    conf_low: float = 0.25
    lost_stop_frames: int = 8


_FIELDS = {f.name for f in fields(Cfg)}


def cfg_from_profile(section=None):
    """Build the single perception Cfg from a profile [perception] dict. Unknown
    keys (e.g. a leftover `mode:` or a param no longer modelled) are ignored;
    `colors` list -> tuple. One confirmed pipeline — no experiment presets."""
    ov = {}
    for k, v in (section or {}).items():
        if v is None or k not in _FIELDS:
            continue
        ov[k] = tuple(v) if (k == 'colors' and isinstance(v, list)) else v
    return replace(Cfg(), **ov) if ov else Cfg()


LABEL_COLORS = {  # BGR
    'W-L': (255, 255, 255), 'W-R': (170, 170, 170),   # white / gray
    'YR-L': (0, 140, 255), 'YR-R': (0, 90, 200),      # orange (right turn)
    'YL-L': (255, 0, 200), 'YL-R': (200, 0, 150),     # magenta (left turn)
    'YS-L': (0, 230, 0), 'YS-R': (0, 150, 0),         # green (straight)
}
EGO_CENTER_COLOR = (255, 255, 0)   # cyan — ego corridor centerline (control value)


# ==========================================================================
# A: detection (HSV white/yellow masks + morphology + color-dominance gate)
# ==========================================================================
def detect(frame, c):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    white = ((S <= c.white_s_max) & (V >= c.white_v_min)).astype(np.uint8) * 255
    yellow = ((H >= c.yellow_h_lo) & (H <= c.yellow_h_hi) &
              (S >= c.yellow_s_min) & (V >= c.yellow_v_min)).astype(np.uint8) * 255
    if 'white' not in c.colors:
        white[:] = 0
    if 'yellow' not in c.colors:
        yellow[:] = 0
    kv = cv2.getStructuringElement(cv2.MORPH_RECT, (3, max(3, c.morph_v)))
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kv)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kv)
    wc, yc = int(cv2.countNonZero(white)), int(cv2.countNonZero(yellow))
    tot = wc + yc
    if tot >= c.gate_min_px:
        if wc / tot < c.color_gate:
            white[:] = 0
        if yc / tot < c.color_gate:
            yellow[:] = 0
    return white, yellow


# ==========================================================================
# B: ROI trapezoid (front-view crop)
# ==========================================================================
def _roi_polygon(h, w, c):
    y0 = int(h * c.roi_top_frac)
    cx = w / 2.0
    tw, bw = c.trap_top_w * w, c.trap_bot_w * w
    return y0, np.float32([[cx - bw / 2, h - 1], [cx - tw / 2, y0],
                           [cx + tw / 2, y0], [cx + bw / 2, h - 1]])


def _roi_mask(h, w, c):
    y0, poly = _roi_polygon(h, w, c)
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [poly.astype(np.int32)], 255)
    return m, y0, poly


# ==========================================================================
# feature build (2nd-order fit -> base x, trend x, turn)
# ==========================================================================
def _build_instance(ys, xs, color, c, W):
    ins = {'color': color, 'xs': xs, 'ys': ys, 'coeffs': None,
           'x_bottom': float(xs[np.argmax(ys)]),
           'x_mean': float(xs.mean()), 'turn': 0}
    if np.unique(ys).size >= 5:
        a, b, cc = np.polyfit(ys.astype(float), xs.astype(float), 2)
        yb, yt = float(ys.max()), float(ys.min())
        ins['coeffs'] = (a, b, cc)
        ins['x_bottom'] = float(a * yb * yb + b * yb + cc)
        hd = (a * yt * yt + b * yt + cc) - ins['x_bottom']
        if abs(hd) >= c.heading_frac * W:
            ins['turn'] = 1 if hd > 0 else -1
        elif abs(a) >= c.curv_strong:
            ins['turn'] = 1 if a > 0 else -1
    return ins


# ==========================================================================
# C: sliding-window separation
# ==========================================================================
def _find_peaks(hist, c):
    h = hist.astype(float).copy()
    peaks = []
    for _ in range(c.sw_max_lanes):
        i = int(h.argmax())
        if h[i] < c.sw_peak_min:
            break
        peaks.append(i)
        h[max(0, i - c.sw_peak_sep):i + c.sw_peak_sep] = 0
    return sorted(peaks)


def _same_lane(a, b, c):
    """Two instances are the SAME physical lane only if their 2nd-order fits stay
    close over the WHOLE shared y-range (MAX |dx| < merge_dx). Uses max, not mean,
    so a pair that coincides at one end but SEPARATES at the other is NOT merged:
      - Y-branch: arms share a stem (bottom) but diverge toward the top,
      - perspective-converging pair: apart near the car, converge at the vanishing
        point up top.
    A thick/diagonal line split into parallel stacks stays close everywhere -> max
    is small -> merged (which window-overlap merging misses, since diagonal stacks
    never share a window column). Under-merging (staying two instances) is the safe
    failure; wrongly merging a real branch is not."""
    if a['coeffs'] is None or b['coeffs'] is None:
        return False
    ylo = max(float(a['ys'].min()), float(b['ys'].min()))
    yhi = min(float(a['ys'].max()), float(b['ys'].max()))
    if yhi - ylo < 1.0:                      # no shared y-range -> cannot judge
        return False
    yy = np.linspace(ylo, yhi, 7)
    dx = float(np.abs(_ebottom(a['coeffs'], yy) - _ebottom(b['coeffs'], yy)).max())
    return dx < c.merge_dx


def sliding_window_lanes(mask, color, c, windows_out=None):
    """ROI-cropped mask -> lane instances. Base peaks -> stack windows upward,
    following curvature -> drop vertically-short stacks -> merge duplicates."""
    H, W = mask.shape
    hist = (mask[H // 2:] > 0).sum(axis=0)
    win_h = max(1, H // c.sw_nwin)
    raw = []
    for base in _find_peaks(hist, c):
        cur = float(base)
        step = 0.0
        prev_cx = None
        xs_all, ys_all, miss, hits = [], [], 0, 0
        for i in range(c.sw_nwin):
            ci = int(round(cur))
            ylo, yhi = H - (i + 1) * win_h, H - i * win_h
            xlo, xhi = max(0, ci - c.sw_margin), min(W, ci + c.sw_margin)
            if windows_out is not None:
                windows_out.append((xlo, ylo, xhi, yhi))
            wy, wx = np.nonzero(mask[ylo:yhi, xlo:xhi] > 0)
            if wx.size > c.sw_minpix:
                mx = float(wx.mean()) + xlo
                xs_all.append(wx + xlo)
                ys_all.append(wy + ylo)
                if prev_cx is not None:
                    obs = max(-c.sw_margin, min(c.sw_margin, mx - prev_cx))
                    step = c.sw_dir_ema * obs + (1 - c.sw_dir_ema) * step
                prev_cx = mx
                cur = mx + step
                miss = 0
                hits += 1
            else:
                cur += step
                miss += 1
                if miss >= c.sw_max_miss:
                    break
            cur = min(max(cur, 0.0), float(W - 1))
        if not xs_all:
            continue
        xs, ys = np.concatenate(xs_all), np.concatenate(ys_all)
        y_span = float(ys.max() - ys.min())
        if hits < c.sw_min_hits or y_span < c.sw_min_span * H:
            continue
        if np.unique(ys).size >= 5 and xs.size >= c.sw_minpix:
            raw.append({'xs': xs, 'ys': ys, 'npix': int(xs.size), 'W': W})
    # strongest (most pixels) first, then drop near-duplicate fits (a thick/diagonal
    # line split into parallel stacks). Merge by polynomial proximity, not window
    # overlap: diagonal splits never share window columns but their fits coincide.
    raw.sort(key=lambda r: -r['npix'])
    kept = []
    for r in raw:
        ins = _build_instance(r['ys'], r['xs'], color, c, r['W'])
        if ins['coeffs'] is not None and any(_same_lane(ins, k, c) for k in kept):
            continue
        kept.append(ins)
    return kept


# ==========================================================================
# D: classification (7 labels)
# ==========================================================================
def _side(ins, w):
    cx = w / 2.0
    pos = -1 if ins['x_bottom'] < cx else 1
    dirv = -1 if ins['x_mean'] < cx else 1
    vote = pos + dirv
    if vote != 0:
        return 'L' if vote < 0 else 'R'
    return 'L' if ins['x_bottom'] < cx else 'R'


def classify(ins, w):
    side = _side(ins, w)
    if ins['color'] == 'W':
        return f'W-{side}'
    tw = 'S' if ins['turn'] == 0 else ('R' if ins['turn'] > 0 else 'L')
    return f'Y{tw}-{side}'


# ==========================================================================
# centerline (adjacent parallel boundary pairs -> ego corridor center)
# ==========================================================================
def _shift(coeffs, dx):
    a, b, c = coeffs
    return (a, b, c + dx)


def _ebottom(coeffs, yb):
    a, b, c = coeffs
    return a * yb * yb + b * yb + c


def _pair_gate(a, b, h, c):
    ylo = max(int(a['ys'].min()), int(b['ys'].min()))
    yhi = min(int(a['ys'].max()), int(b['ys'].max()))
    if yhi - ylo < c.pair_overlap_min * h:
        return None
    yy = np.linspace(ylo, yhi, 7)
    gaps = _ebottom(b['coeffs'], yy) - _ebottom(a['coeffs'], yy)
    if float(gaps.min()) < c.pair_gap_min:
        return None
    return ylo, yhi


def lane_centers(lanes, w, h, c):
    cx = w / 2.0
    ls = sorted([x for x in lanes if x['coeffs'] is not None],
                key=lambda x: x['x_bottom'])
    out = []
    for a, b in zip(ls, ls[1:]):
        if _side(a, w) == _side(b, w):
            continue
        ov = _pair_gate(a, b, h, c)
        if ov is None:
            continue
        ylo, yhi = ov
        coeffs = tuple((p + q) / 2.0 for p, q in zip(a['coeffs'], b['coeffs']))
        x_bottom = float(_ebottom(coeffs, yhi))
        ego = a['x_bottom'] < cx <= b['x_bottom']
        out.append({'coeffs': coeffs, 'x_bottom': x_bottom, 'offset': x_bottom - cx,
                    'ego': ego, 'a': a, 'b': b, 'y_lo': ylo, 'y_hi': yhi})
    return out


def ego_center(centers, lanes, w, width):
    for cc in centers:
        if cc['ego']:
            return cc
    cx = w / 2.0
    cand = [x for x in lanes if x['coeffs'] is not None]
    if not cand or width <= 0:
        return None
    near = min(cand, key=lambda x: abs(x['x_bottom'] - cx))
    if abs(near['x_bottom'] - cx) > width:
        return None
    dx = width / 2.0 if near['x_bottom'] < cx else -width / 2.0
    coeffs = _shift(near['coeffs'], dx)
    xb = near['x_bottom'] + dx
    # clamp to the source lane's observed span so heading/curvature/drawing never
    # extrapolate the parabola beyond where the lane was actually detected.
    return {'coeffs': coeffs, 'x_bottom': xb, 'offset': xb - cx, 'ego': True,
            'a': near, 'b': None, 'coast': True,
            'y_lo': float(near['ys'].min()), 'y_hi': float(near['ys'].max())}


# ==========================================================================
# E: tracker (ego L/R persistence + lane-width coast + turn)
# ==========================================================================
class Tracker:
    def __init__(self, c, h, w):
        self.c = c
        self.h, self.w = h, w
        self.L = self.R = self.width = None
        self.lost = 0
        self.turn = 1

    def _pick(self, cands, tracked, want_side):
        cands = [x for x in cands if x['coeffs'] is not None]
        if not cands:
            return None
        yb = self.h - 1
        if tracked is not None:
            best = min(cands, key=lambda x: abs(_ebottom(x['coeffs'], yb) - _ebottom(tracked, yb)))
            return best if abs(_ebottom(best['coeffs'], yb) - _ebottom(tracked, yb)) <= self.c.jump_max else None
        cx = self.w / 2
        pool = [x for x in cands if (x['x_bottom'] < cx) == (want_side == 'L')]
        if not pool:
            return None
        return (max if want_side == 'L' else min)(pool, key=lambda x: x['x_bottom'])

    def _ema(self, prev, meas):
        if prev is None:
            return meas
        a = self.c.ema_alpha
        return tuple(a * m + (1 - a) * p for m, p in zip(meas, prev))

    def update(self, instances):
        drive = list(instances)
        Lc = [x for x in drive if x['x_bottom'] < self.w / 2]
        Rc = [x for x in drive if x['x_bottom'] >= self.w / 2]
        mL, mR = self._pick(Lc, self.L, 'L'), self._pick(Rc, self.R, 'R')
        yb = self.h - 1
        gL, gR = mL is not None, mR is not None
        width = self.width if self.width is not None else self.c.lane_width_default * self.w
        if gL and gR:
            self.L = self._ema(self.L, mL['coeffs'])
            self.R = self._ema(self.R, mR['coeffs'])
            wdt = _ebottom(mR['coeffs'], yb) - _ebottom(mL['coeffs'], yb)
            self.width = wdt if self.width is None else 0.6 * self.width + 0.4 * wdt
            self.lost = 0
        elif gL:
            self.L = self._ema(self.L, mL['coeffs'])
            self.R = _shift(self.L, width)
            self.lost = 0
        elif gR:
            self.R = self._ema(self.R, mR['coeffs'])
            self.L = _shift(self.R, -width)
            self.lost = 0
        else:
            self.lost += 1
        if self.lost >= self.c.lost_reset:
            self.L = self.R = self.width = None
        if self.L is not None and self.R is not None:
            ca = (self.L[0] + self.R[0]) / 2.0
            self.turn = 0 if abs(ca) < self.c.straight_thresh else (1 if ca > 0 else -1)
        return mL, mR


# ==========================================================================
# F: scalar output stabilizer (center_error EMA + failsafe state name)
# ==========================================================================
class _Stabilizer:
    def __init__(self, c):
        self.c = c
        self.ema = None
        self.lost = 0
        self.hist = deque(maxlen=max(1, c.median_window))

    def update(self, center_error, conf):
        c = self.c
        if center_error is None or conf < c.conf_low:
            self.lost += 1
            return self.ema, ('LOST' if self.lost >= c.lost_stop_frames else 'HOLD')
        if self.ema is not None and abs(center_error - self.ema) > c.outlier_jump:
            self.lost += 1
            return self.ema, 'OUTLIER'
        self.lost = 0
        val = center_error
        if c.use_median:
            self.hist.append(center_error)
            val = float(np.median(self.hist))
        self.ema = val if self.ema is None else c.ema_alpha * val + (1 - c.ema_alpha) * self.ema
        return self.ema, ('LOW_CONF' if conf < c.conf_low * 1.6 else 'OK')


def _heading_deg(coeffs, y_lo, y_hi):
    """Ego centerline tangent as a signed angle (deg). +: recedes rightward.
    y grows downward, so y_hi is the near (bottom) end, y_lo the far (top) end."""
    x_near, x_far = _ebottom(coeffs, y_hi), _ebottom(coeffs, y_lo)
    denom = (y_hi - y_lo) or 1e-6
    return float(math.degrees(math.atan2(x_far - x_near, denom)))


# ==========================================================================
# Public pipeline (stateful across frames)
# ==========================================================================
class LanePipeline:
    def __init__(self, cfg):
        self.cfg = cfg
        self.stab = _Stabilizer(cfg)
        self.trk = None
        self._size = None

    def process(self, bgr, debug=False):
        """Run one frame. Returns (overlay, state); with debug=True also a third
        dict of intermediates for render_panels (masks, lanes, windows, ec, ...)."""
        c = self.cfg
        h, w = bgr.shape[:2]
        if self._size != (h, w):
            self._size = (h, w)
            self.trk = Tracker(c, h, w)

        white, yellow = detect(bgr, c)
        roi, y0, trap = _roi_mask(h, w, c)
        mw = cv2.bitwise_and(white, roi)
        my = cv2.bitwise_and(yellow, roi)
        # --- BEV seam: a calibrated BEV would warp (mw, my) here and unwarp the
        #     fitted coeffs before drawing. Front-view interim keeps image coords.
        windows = []
        lanes = (sliding_window_lanes(mw, 'W', c, windows) +
                 sliding_window_lanes(my, 'Y', c, windows))
        mL, mR = self.trk.update(lanes)

        centers = lane_centers(lanes, w, h, c)
        width = self.trk.width if self.trk.width else c.lane_width_default * w
        ec = ego_center(centers, lanes, w, width)

        if ec is not None:
            center_error = float(ec['offset'] / (w / 2))
            y_lo, y_hi = ec.get('y_lo', y0), ec.get('y_hi', h - 1)
            heading = _heading_deg(ec['coeffs'], y_lo, y_hi)
            curvature = float(ec['coeffs'][0] * 1000.0)
            confidence = 0.5 if ec.get('coast') else 0.9
        else:
            center_error = heading = curvature = None
            confidence = 0.0
        left_conf = 1.0 if mL is not None else 0.0
        right_conf = 1.0 if mR is not None else 0.0
        used_fb = bool(ec is not None and ec.get('coast'))

        ema, fstate = self.stab.update(center_error, confidence)

        state = {
            'center_error': center_error, 'ema': ema,
            'heading': heading, 'heading_label': 'lane7',
            'confidence': confidence, 'left_conf': left_conf,
            'right_conf': right_conf, 'curvature': curvature,
            'state': fstate, 'used_fallback': used_fb,
        }
        overlay = _render_single(bgr, lanes, ec, w, ema, heading, fstate)
        if debug:
            dbg = {'mw': mw, 'my': my, 'lanes': lanes, 'windows': windows, 'ec': ec,
                   'trap': trap, 'y0': y0, 'ema': ema, 'fstate': fstate,
                   'center_error': center_error, 'heading': heading,
                   'confidence': confidence, 'used_fb': used_fb}
            return overlay, state, dbg
        return overlay, state


# ==========================================================================
# rendering (front-view; no BEV panels)
# ==========================================================================
def _draw_curve(img, coeffs, color, thick, y0, y1):
    h, w = img.shape[:2]
    ys = np.arange(int(y0), int(y1) + 1)
    xs = coeffs[0] * ys * ys + coeffs[1] * ys + coeffs[2]
    pts = np.array([[int(x), int(y)] for x, y in zip(xs, ys) if 0 <= x < w], np.int32)
    if len(pts) > 1:
        cv2.polylines(img, [pts], False, color, thick)


def _lane_span(ins):
    return int(ins['ys'].min()), int(ins['ys'].max())


def _render_single(bgr, lanes, ec, w, ema, heading, fstate):
    img = bgr.copy()
    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        col = LABEL_COLORS.get(classify(ins, w), (0, 255, 0))
        _draw_curve(img, ins['coeffs'], col, 2, *_lane_span(ins))
    if ec is not None:
        _draw_curve(img, ec['coeffs'], EGO_CENTER_COLOR, 2,
                    ec.get('y_lo', 0), ec.get('y_hi', img.shape[0] - 1))
    off = f"{ec['offset']:+.0f}px{'(coast)' if ec.get('coast') else ''}" if ec else '--'
    txt = f"off {off} ema {'n/a' if ema is None else f'{ema:+.2f}'} {fstate}"
    cv2.putText(img, txt, (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                EGO_CENTER_COLOR, 1)
    return img


def _panel(img, title):
    cv2.rectangle(img, (0, 0), (img.shape[1], 14), (0, 0, 0), -1)
    cv2.putText(img, title, (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    return img


def _inst_color(idx):
    bgr = cv2.cvtColor(np.uint8([[[int((idx * 47) % 180), 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def render_panels(bgr, dbg, cfg):
    """Online debug multi-panel (front-view, 4 panels):
      [ input+ROI | white/yellow mask | sliding windows+lanes | label+ego overlay ].
    Built only from `LanePipeline.process(debug=True)` intermediates (no
    re-implementation). perception_node publishes this on /lane/debug/compressed;
    recorder saves it. BEV (6-panel) is deferred to the calibrated-BEV stage.
    """
    h, w = bgr.shape[:2]
    mw, my, lanes = dbg['mw'], dbg['my'], dbg['lanes']
    windows, ec, trap = dbg['windows'], dbg['ec'], dbg['trap']

    # P1: input + ROI trapezoid
    p1 = bgr.copy()
    cv2.polylines(p1, [trap.astype(np.int32)], True, (0, 200, 255), 1)
    _panel(p1, f'1 input+ROI  {cfg.name}')

    # P2: white/yellow mask
    p2 = np.zeros((h, w, 3), np.uint8)
    p2[mw > 0] = (200, 200, 200)
    p2[my > 0] = (0, 220, 255)
    _panel(p2, '2 mask (W/Y)')

    # P3: sliding windows + lane instance points
    p3 = np.zeros((h, w, 3), np.uint8)
    for (xlo, ylo, xhi, yhi) in windows:
        cv2.rectangle(p3, (xlo, ylo), (xhi, yhi), (55, 55, 55), 1)
    for idx, ins in enumerate(lanes):
        col = _inst_color(idx)
        for x, y in zip(ins['xs'][::3], ins['ys'][::3]):
            cv2.circle(p3, (int(x), int(y)), 1, col, -1)
    _panel(p3, f'3 sliding ({len(lanes)} lane)')

    # P4: label overlay + ego center on original
    p4 = bgr.copy()
    ty = 26
    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        lab = classify(ins, w)
        col = LABEL_COLORS.get(lab, (0, 255, 0))
        _draw_curve(p4, ins['coeffs'], col, 2, *_lane_span(ins))
        cv2.putText(p4, lab, (4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)
        ty += 14
    if ec is not None:
        _draw_curve(p4, ec['coeffs'], EGO_CENTER_COLOR, 2,
                    ec.get('y_lo', 0), ec.get('y_hi', h - 1))
    off = (f"off={ec['offset']:+.0f}px{'(coast)' if ec.get('coast') else ''}"
           if ec is not None else 'off=--')
    _panel(p4, f"4 labels+ego  {off}  {dbg['fstate']}")

    return np.hstack([p1, p2, p3, p4])
