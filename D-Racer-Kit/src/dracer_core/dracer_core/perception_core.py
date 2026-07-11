"""Pure (ROS-independent) lane-detection pipeline. Single source of truth.

colour -> calibrated BEV -> sliding-window multi-lane tracking -> 7-label
classification -> ego corridor centerline (+ Tracker/coast).

Stage order, and why (`LanePipeline.process` marks the seam):

  colour   on the RAW frame, cropped to `cam.src_rows` -- the band the BEV actually
           reads. A per-pixel colour test is cheaper and sharper before any resample,
           and rows above that band feed no BEV cell at all.
  warp     the two 1-channel masks (not the 3-channel frame).
  shape    morphology, area gates, window sizes, lane width, pairing, coast, heading --
           ALL of it after the warp, where a pixel IS a centimetre and a threshold means
           the same thing at 26cm and at 78cm. `cfg_to_px` resolves the cm config to BEV
           px once; every stage below it keeps taking plain pixels.

With a CameraModel the LUT is itself a calibrated trapezoid crop, so the legacy
hand-tuned `roi_*` trapezoid is redundant and is skipped. Without one (cam=None) the
front-view path is unchanged, thresholds keep their legacy screen-space meaning, and
PERCEPTION.md §6 limitations apply.

Imported by BOTH the online perception node and the offline control tools
(offline/control_predict.py). Depends only on cv2 + numpy.

Usage:
    from dracer_core.perception_core import LanePipeline, Cfg, cfg_from_profile
    pipe = LanePipeline(cfg_from_profile(profile['perception']), cam)   # cam: CameraModel
    state = pipe.process(frame_bgr)                 # dict (center_error, ...)
    state, dbg = pipe.process(frame_bgr, debug=True)   # dbg -> render_panels()

`process()` returns ONLY the lane state. Rendering is never on its hot path: build the
debug image with `render_panels(frame, dbg, cfg)`, and only when something is watching.

The pipeline NEVER commands the vehicle; it only produces a lane-state estimate
(consumed by the control stage).
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
    outlier_relatch: int = 6     # consecutive rejections -> believe the new value, re-seed.
                                 # Without it `outlier_jump` latches forever (see _Stabilizer).
                                 # 6 @30Hz = 0.2s: still swallows any spike, but a real
                                 # corridor flip is acted on in a fifth of a second.
    use_median: bool = False
    median_window: int = 5
    conf_low: float = 0.25
    lost_stop_frames: int = 8

    # ==================================================================
    # METRIC (BEV) parameters — cm, used ONLY when a CameraModel is given.
    # ------------------------------------------------------------------
    # In a calibrated BEV a pixel IS a physical length, so every geometric
    # threshold above becomes a real distance instead of a screen fraction that
    # silently changes meaning with resolution, camera angle, or track. These cm
    # values are resolved to px ONCE by `cfg_to_px()`; the stages below it keep
    # taking plain pixels and are unchanged.
    lane_width_cm: float = 35.0      # TRACK: centre-to-centre of the two lane tapes.
                                     # (2025: inner 32 / outer 38, tape 3 -> centres 35.
                                     #  The fits follow tape CENTRES, so it is neither 32 nor 38.)
    sw_margin_cm: float = 5.0        # window half-width: lane-position slack + curve step
    sw_peak_sep_cm: float = 15.0     # two base peaks closer than this = one lane (< lane width)
    merge_dx_cm: float = 5.0         # two fits within this everywhere = the same tape
    pair_gap_min_cm: float = 8.0     # below this the pair has collapsed = a crossing, reject
    jump_max_cm: float = 15.0        # tracker: max lane jump between frames
    pair_width_tol: float = 0.25     # PHYSICAL pairing gate: |gap - lane_width| <= this*width.
                                     # Impossible in the front view (gap varies with y); it is
                                     # what stops a white/yellow or cross-corridor mismatch.
    heading_cm: float = 5.0          # |top-bottom x| >= this -> turn by heading.
                                     # In the BEV this is a REAL sideways drift; in the front
                                     # view perspective convergence faked it (limitation §6a).

    # Thresholds that count PIXELS rather than measuring a distance. They were the last
    # scale trap left in the config: a pixel COUNT scales with px_per_cm SQUARED (an area),
    # so halving the BEV scale quarters every count while the threshold stays put, and
    # detection dies with nothing in the log. Expressed as areas/lengths they are immune.
    # Defaults reproduce the old px values EXACTLY at px_per_cm = 4.
    morph_cm: float = 1.25           # vertical CLOSE kernel  (was morph_v = 5 px)
    sw_minpix_cm2: float = 1.125     # min lane area in a window (was sw_minpix = 18 px)
    sw_peak_min_cm: float = 2.5      # min histogram peak, a LENGTH (was sw_peak_min = 10 px)
    gate_min_cm2: float = 5.0        # colour-gate floor, an area (was gate_min_px = 80 px)


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


def cfg_to_px(cfg, cam):
    """Resolve the metric (cm) params into BEV pixels — ONCE, here.

    This is the whole trick that keeps the BEV port small: the detector, the sliding
    window, the pairing and the tracker keep speaking plain pixels and are untouched;
    only their THRESHOLDS change meaning, and that mapping lives in one place.

    cam=None -> front-view: return cfg unchanged (legacy px thresholds).
    """
    if cam is None:
        return cfg
    s = cam.px_per_cm
    bev_w = cam.bev_size[0]
    lane_px = cfg.lane_width_cm * s
    return replace(
        cfg,
        sw_margin=max(4, int(round(cfg.sw_margin_cm * s))),
        sw_peak_sep=max(4, int(round(cfg.sw_peak_sep_cm * s))),
        merge_dx=cfg.merge_dx_cm * s,
        pair_gap_min=cfg.pair_gap_min_cm * s,
        jump_max=cfg.jump_max_cm * s,
        # Pixel COUNTS -> areas (s^2); the histogram peak is a length (s).
        morph_v=max(3, int(round(cfg.morph_cm * s))),
        sw_minpix=max(4, int(round(cfg.sw_minpix_cm2 * s * s))),
        sw_peak_min=max(3, int(round(cfg.sw_peak_min_cm * s))),
        gate_min_px=max(8, int(round(cfg.gate_min_cm2 * s * s))),
        # Tracker reads `lane_width_default * W` as its fallback width, so express the
        # REAL lane width as that fraction. The old 0.5 was a guess with no physical
        # meaning — it is what made every coast produce |center_error| ~= 0.5 (§6c).
        lane_width_default=lane_px / bev_w,
        # In the BEV, heading is a true lateral drift over the lane's span (no perspective
        # convergence to fake it), so express the threshold as a real distance.
        heading_frac=(cfg.heading_cm * s) / bev_w,
    )


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
def color_masks(frame, c):
    """Per-pixel HSV colour test. No morphology, no gate -- those are SHAPE and AREA
    judgements, and in the front view neither has a stable meaning (a 5px kernel spans
    1cm of tarmac up close and 5cm at the far edge). They belong after the warp."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    white = ((S <= c.white_s_max) & (V >= c.white_v_min)).astype(np.uint8) * 255
    yellow = ((H >= c.yellow_h_lo) & (H <= c.yellow_h_hi) &
              (S >= c.yellow_s_min) & (V >= c.yellow_v_min)).astype(np.uint8) * 255
    if 'white' not in c.colors:
        white[:] = 0
    if 'yellow' not in c.colors:
        yellow[:] = 0
    return white, yellow


def morph_gate(white, yellow, c):
    """Vertical CLOSE + colour-dominance gate.

    Run on the BEV, `morph_v` is a REAL length (cfg.morph_cm) that means the same thing
    at 26cm and at 78cm, and it also heals the row-replication the nearest-neighbour warp
    leaves behind. `gate_min_px` likewise becomes a real area. On the front view (cam=None)
    both keep their legacy screen-space meaning.
    """
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


def detect(frame, c):
    """Front-view detection (legacy path + offline probes): colour, then shape."""
    return morph_gate(*color_masks(frame, c), c)


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
                # Only a gap BETWEEN hits is a miss. Empty windows BEFORE the first hit are
                # the climb up to the lane, not the end of it, and `sw_max_miss` must not
                # spend itself on them: the base peak has already proven there are pixels in
                # this column: they are simply not at the very bottom of the BEV.
                #
                # They routinely are not. The near BEV rows go blind whenever the lane leaves
                # the camera's lateral FOV -- on a curve, or with the car merely offset -- and
                # the BEV floor (26cm) sits below where this lens can see a lane at all (both
                # boundaries only appear from ~30cm, and the near field is asymmetric:
                # -19.7..+2.0cm at 26cm). Counting that climb as misses killed EVERY stack
                # three windows in, and the whole frame came out as 0 lanes with a perfectly
                # good mask sitting right there in the debug panel.
                if hits:
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


def _pair_gate(a, b, h, c, lane_w_px=0.0):
    """Can these two fits be the left/right boundary of ONE corridor?

    `lane_w_px` > 0 (BEV only) adds the PHYSICAL gate: the gap must actually equal a
    lane width. In the front view the gap shrinks with y, so no such test exists and
    `pair_gap_min` degenerates into "> 0" — which is why a white line could pair with a
    yellow one across the wrong corridor (§6b). In a metric BEV the gap is constant, so
    "is this really a lane width apart?" is a question we can finally ask.
    """
    ylo = max(int(a['ys'].min()), int(b['ys'].min()))
    yhi = min(int(a['ys'].max()), int(b['ys'].max()))
    if yhi - ylo < c.pair_overlap_min * h:
        return None
    yy = np.linspace(ylo, yhi, 7)
    gaps = _ebottom(b['coeffs'], yy) - _ebottom(a['coeffs'], yy)
    if float(gaps.min()) < c.pair_gap_min:
        return None
    if lane_w_px > 0:
        if abs(float(gaps.mean()) - lane_w_px) > c.pair_width_tol * lane_w_px:
            return None                      # not a lane width apart -> not one corridor
    return ylo, yhi


def lane_centers(lanes, w, h, c, lane_w_px=0.0):
    cx = w / 2.0
    ls = sorted([x for x in lanes if x['coeffs'] is not None],
                key=lambda x: x['x_bottom'])
    out = []
    for a, b in zip(ls, ls[1:]):
        if _side(a, w) == _side(b, w):
            continue
        ov = _pair_gate(a, b, h, c, lane_w_px)
        if ov is None:
            continue
        ylo, yhi = ov
        coeffs = tuple((p + q) / 2.0 for p, q in zip(a['coeffs'], b['coeffs']))
        x_bottom = float(_ebottom(coeffs, yhi))
        ego = a['x_bottom'] < cx <= b['x_bottom']
        out.append({'coeffs': coeffs, 'x_bottom': x_bottom, 'offset': x_bottom - cx,
                    'ego': ego, 'a': a, 'b': b, 'y_lo': ylo, 'y_hi': yhi})
    return out


def ego_center(centers, lanes, w, width, mL=None, mR=None):
    """Ego corridor centreline: a real left/right pair, else COAST off one boundary.

    `mL`/`mR` are the Tracker's picks for this frame. Prefer them for the coast side:
    the Tracker carries the left/right identity ACROSS frames (it matches against the
    lane it tracked last frame), whereas deciding the side afresh from `x_bottom < cx`
    flips as soon as a curving lane crosses the image centre — and a flip inverts the
    shift, so the centreline jumps to the WRONG SIDE of the car. That is what produced
    the sign reversals: 96 of them, 80% while coasting, and the steering command pointed
    INTO the lane instead of away from it (§6c/§6e, §7.4 f250).
    """
    for cc in centers:
        if cc['ego']:
            return cc
    cx = w / 2.0
    if width <= 0:
        return None

    # Side from the Tracker (persistent) — not from this frame's geometry.
    near, dx = None, 0.0
    if mL is not None and mL.get('coeffs') is not None:
        near, dx = mL, +width / 2.0          # left boundary -> corridor lies to its RIGHT
    elif mR is not None and mR.get('coeffs') is not None:
        near, dx = mR, -width / 2.0          # right boundary -> corridor lies to its LEFT
    else:
        cand = [x for x in lanes if x['coeffs'] is not None]
        if not cand:
            return None
        near = min(cand, key=lambda x: abs(x['x_bottom'] - cx))
        if abs(near['x_bottom'] - cx) > width:
            return None                       # too far to be an ego boundary
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
            if self.lost < c.outlier_relatch:
                return self.ema, 'OUTLIER'
            # The rejection had no way out. `outlier_jump` exists to swallow a ONE-frame
            # spike, but the test is against a frozen EMA, so a SUSTAINED move past it
            # re-rejects itself forever: every later frame is measured against the same
            # stale value and fails the same way. That is a latch, not a filter.
            #
            # It fires on a legitimate event. When the tracker swaps which boundary it
            # can see, the corridor centre steps by a full lane width -- 35cm over a 29cm
            # half-width = 1.2 normalised, way past the 0.5 gate. Session 145617 did this
            # at frame 419 (left-coast +0.49 -> right-coast -0.38) and never recovered:
            # the EMA stayed pinned at +0.428 for the last 44 frames (1.47s) while
            # perception kept correctly reporting -0.38. control_core reads `ema` and
            # ignores `state`, so the car steered on the WRONG SIGN for 1.47s.
            #
            # So: reject a spike, but believe a fact. If the new value holds for
            # `outlier_relatch` frames straight it is not noise -- re-seed onto it.
            self.hist.clear()
            self.ema = None
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
    """Lane pipeline. `cam` (dracer_core.calib.CameraModel) switches on the metric BEV.

    cam=None  -> front-view (legacy). Lane fits are in image coords; every geometric
                 threshold is a screen fraction; coast shifts by a constant pixel offset
                 that is only correct in a BEV. Known limitations: PERCEPTION.md §6.
    cam given -> masks are warped to a calibrated top-down view, so a pixel IS a length.
                 Lane width, pairing, coast and heading all become physical. `cfg_to_px`
                 resolves the cm thresholds once; every stage below stays pixel-based.

    The scalar contract (`center_error` normalised to [-1,1]) is UNCHANGED in both modes,
    so control needs no retune to A/B this. Switching it to cm is a separate step (P5).
    """

    def __init__(self, cfg, cam=None):
        self.cfg = cfg
        self.cam = cam
        self.c = cfg_to_px(cfg, cam)          # cm -> BEV px, once
        self.stab = _Stabilizer(self.c)
        self.trk = None
        self._size = None

    def process(self, bgr, debug=False):
        """Run one frame. Returns `state`; with debug=True returns (state, dbg), the
        second being intermediates for render_panels (masks, lanes, windows, ec, ...)."""
        c = self.c
        h, w = bgr.shape[:2]

        # --- BEV seam --------------------------------------------------------
        # Warp the COLOR MASKS, not the frame: colour is a per-pixel test, so thresholding
        # before the resample is both cheaper (2 x 1ch out vs 1 x 3ch) and sharper (no
        # interpolated in-between colours to fool the HSV gates). Everything after the warp
        # is coordinate-agnostic — it just starts getting metric pixels.
        if self.cam is not None:
            # The LUT already crops the ground to a calibrated trapezoid, so the hand-tuned
            # `roi_*` one is redundant: measured against this camera it removes a further
            # 3.7% of the valid BEV and nothing else. Dropping it also drops a fillPoly and
            # two full-frame bitwise_ands. Rows above the LUT's top feed NO BEV cell, so
            # colour-test only the band it reads (~38% fewer pixels through cvtColor).
            r0, r1 = self.cam.src_rows
            mw = np.zeros((h, w), np.uint8)
            my = np.zeros((h, w), np.uint8)
            mw[r0:r1], my[r0:r1] = color_masks(bgr[r0:r1], c)
            mw = self.cam.to_bev(mw)
            my = self.cam.to_bev(my)
            # Shape + area judgements now that a pixel IS a length.
            mw, my = morph_gate(mw, my, c)
            w, h = self.cam.bev_size
            lane_w_px = self.cam.lane_width_px(c.lane_width_cm)
            trap, y0 = self.cam.footprint_poly(), r0
        else:
            white, yellow = detect(bgr, c)
            roi, y0, trap = _roi_mask(h, w, c)
            mw = cv2.bitwise_and(white, roi)
            my = cv2.bitwise_and(yellow, roi)
            lane_w_px = 0.0                   # no physical gate in the front view
        if self._size != (h, w):
            self._size = (h, w)
            self.trk = Tracker(c, h, w)

        windows = []
        lanes = (sliding_window_lanes(mw, 'W', c, windows) +
                 sliding_window_lanes(my, 'Y', c, windows))
        mL, mR = self.trk.update(lanes)

        centers = lane_centers(lanes, w, h, c, lane_w_px)
        width = self.trk.width if self.trk.width else c.lane_width_default * w
        ec = ego_center(centers, lanes, w, width, mL, mR)

        if ec is not None:
            center_error = float(ec['offset'] / (w / 2))
            y_lo, y_hi = ec.get('y_lo', 0), ec.get('y_hi', h - 1)
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
        if debug:
            dbg = {'mw': mw, 'my': my, 'lanes': lanes, 'windows': windows, 'ec': ec,
                   'trap': trap, 'y0': y0, 'ema': ema, 'fstate': fstate,
                   'center_error': center_error, 'heading': heading,
                   'confidence': confidence, 'used_fb': used_fb, 'cam': self.cam,
                   'bev_size': (w, h)}
            return state, dbg
        return state


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


def _draw_fit(img, coeffs, y0, y1, color, thick, cam):
    """Draw a lane fit. In BEV mode the fit lives in ground coords -> unwarp it back
    onto the camera frame, so the overlay a human looks at is always the real view."""
    if cam is None:
        _draw_curve(img, coeffs, color, thick, y0, y1)
        return
    pts = cam.bev_coeffs_to_image(coeffs, y0, y1)
    h, w = img.shape[:2]
    p = np.array([[int(x), int(y)] for x, y in pts
                  if 0 <= x < w and 0 <= y < h], np.int32)
    if len(p) > 1:
        cv2.polylines(img, [p], False, color, thick)


def _panel(img, title):
    cv2.rectangle(img, (0, 0), (img.shape[1], 14), (0, 0, 0), -1)
    cv2.putText(img, title, (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    return img


def _inst_color(idx):
    bgr = cv2.cvtColor(np.uint8([[[int((idx * 47) % 180), 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def render_panels(bgr, dbg, cfg):
    """Online debug multi-panel (4 panels):
      [ input+ROI | mask | sliding windows+lanes | label+ego overlay ].
    Built only from `LanePipeline.process(debug=True)` intermediates (no
    re-implementation). perception_node publishes this on /lane/debug/compressed.

    In BEV mode panels 2-3 show the METRIC top-down view (that is where the lanes are
    actually found), letter-boxed to the frame size so the strip stays uniform, while
    panels 1 and 4 stay in the camera view a human can read.
    """
    h, w = bgr.shape[:2]
    mw, my, lanes = dbg['mw'], dbg['my'], dbg['lanes']
    windows, ec, trap = dbg['windows'], dbg['ec'], dbg['trap']
    cam = dbg.get('cam')
    bw, bh = dbg.get('bev_size', (w, h))     # lane coords live in THIS frame

    def _fit_panel(img):
        """BEV canvas -> frame-sized panel (uniform strip, aspect preserved)."""
        if (img.shape[1], img.shape[0]) == (w, h):
            return img
        s = min(w / img.shape[1], h / img.shape[0])
        r = cv2.resize(img, (max(1, int(img.shape[1] * s)), max(1, int(img.shape[0] * s))),
                       interpolation=cv2.INTER_NEAREST)
        out = np.zeros((h, w, 3), np.uint8)
        y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
        out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
        return out

    # P1: input + ROI trapezoid (camera view)
    p1 = bgr.copy()
    cv2.polylines(p1, [trap.astype(np.int32)], True, (0, 200, 255), 1)
    _panel(p1, f'1 input+ROI  {cfg.name}')

    # P2: mask — in BEV mode this is the warped, metric mask
    p2 = np.zeros((bh, bw, 3), np.uint8)
    p2[mw > 0] = (200, 200, 200)
    p2[my > 0] = (0, 220, 255)
    if cam is not None:                       # vehicle axis: what center_error is measured from
        cv2.line(p2, (int(cam.axis_u), 0), (int(cam.axis_u), bh - 1), (0, 0, 120), 1)
    p2 = _fit_panel(p2)
    _panel(p2, '2 mask' + ('  BEV metric' if cam is not None else '  (W/Y)'))

    # P3: sliding windows + lane points (same coords as the fits)
    p3 = np.zeros((bh, bw, 3), np.uint8)
    for (xlo, ylo, xhi, yhi) in windows:
        cv2.rectangle(p3, (xlo, ylo), (xhi, yhi), (55, 55, 55), 1)
    for idx, ins in enumerate(lanes):
        col = _inst_color(idx)
        for x, y in zip(ins['xs'][::3], ins['ys'][::3]):
            cv2.circle(p3, (int(x), int(y)), 1, col, -1)
    p3 = _fit_panel(p3)
    _panel(p3, f'3 sliding ({len(lanes)} lane)')

    # P4: labels + ego centreline, unwarped back onto the camera view
    p4 = bgr.copy()
    ty = 26
    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        lab = classify(ins, bw)
        col = LABEL_COLORS.get(lab, (0, 255, 0))
        _draw_fit(p4, ins['coeffs'], *_lane_span(ins), col, 2, cam)
        cv2.putText(p4, lab, (4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)
        ty += 14
    if ec is not None:
        _draw_fit(p4, ec['coeffs'], ec.get('y_lo', 0), ec.get('y_hi', bh - 1),
                  EGO_CENTER_COLOR, 2, cam)
    if ec is None:
        off = 'off=--'
    elif cam is not None:
        off = f"off={ec['offset'] / cam.px_per_cm:+.1f}cm{'(coast)' if ec.get('coast') else ''}"
    else:
        off = f"off={ec['offset']:+.0f}px{'(coast)' if ec.get('coast') else ''}"
    _panel(p4, f"4 labels+ego  {off}  {dbg['fstate']}")

    return np.hstack([p1, p2, p3, p4])
