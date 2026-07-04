"""Pure (ROS-independent) lane-detection pipeline shared by the onboard node.

Mirrors the offline experimentation tool (local_scripts/lane_preview.py):
composable axes A..F with mode presets M1..M6. Depends only on cv2 + numpy so
it can run inside the ROS2 node and be unit-tested off-board.

Usage:
    from opencv.lane_core import LanePipeline, PRESETS, make_cfg
    pipe = LanePipeline(make_cfg('M2', roi_top_frac=0.6))
    overlay_bgr, state = pipe.process(frame_bgr)   # state: dict (center_error, ...)

The pipeline NEVER commands the vehicle; it only produces a lane state estimate
and a debug overlay. Control mapping is a separate, later stage.
"""
from collections import deque
from dataclasses import dataclass, replace

import cv2
import numpy as np


# ==========================================================================
# Config + presets (kept in sync with local_scripts/lane_preview.py)
# ==========================================================================
@dataclass
class Cfg:
    name: str = 'M1'
    # A: segmentation
    use_hsv: bool = True
    use_lab: bool = False
    fuse: str = 'or'
    edge_validate: bool = False
    seg_fallback_adaptive: bool = False
    fallback_min_frac: float = 0.004
    hsv_s_max: int = 80
    hsv_v_min: int = 160
    lab_l_min: int = 170
    # colored-tape lane (yellow/orange tape). HSV hue band.
    # Tuned to the 2025 test track: measured tape hue H~22-32, S p5=80, V p5=120
    # (was h 5-30 / s 90 / v 80, tuned for the older bag_20260703_145235 clip).
    use_orange: bool = False
    orange_h_lo: int = 15
    orange_h_hi: int = 38
    orange_s_min: int = 70
    orange_v_min: int = 90
    canny_lo: int = 50
    canny_hi: int = 150
    edge_dilate: int = 2
    morph_kernel: int = 3
    adaptive_block: int = 21
    adaptive_c: int = -5
    # B: ROI
    roi_top_frac: float = 0.55
    trap_top_w: float = 0.55
    trap_bot_w: float = 1.0
    dynamic_roi: bool = False
    dynamic_roi_gain: float = 1.0
    # C: extraction
    min_contour_area: int = 15
    min_aspect: float = 0.0
    min_length: float = 0.0
    lane_width_default: float = 0.6
    lane_width_tol: float = 0.0
    split_ref: str = 'center'
    do_polyfit: bool = False
    curvature: bool = False
    per_lane_conf: bool = False
    # D: heading
    heading_method: str = 'slope'
    near_frac: float = 0.30
    far_frac: float = 0.30
    # E: temporal
    ema_alpha: float = 0.4
    outlier_jump: float = 0.5
    use_median: bool = False
    median_window: int = 5
    # F: failsafe (annotated)
    conf_low: float = 0.25
    lost_stop_frames: int = 8


PRESETS = {
    'M1': Cfg(name='M1 Basic', use_hsv=True, use_lab=False),
    'M2': Cfg(name='M2 Brightness', use_hsv=True, use_lab=True),
    'M3': Cfg(name='M3 Strict', use_hsv=True, use_lab=True, edge_validate=True),
    'M4': Cfg(name='M4 Heading', use_hsv=True, use_lab=True,
              heading_method='hough', split_ref='prev_row'),
    'M5': Cfg(name='M5 Curve', use_hsv=True, use_lab=True,
              do_polyfit=True, curvature=True, use_median=True,
              heading_method='two_point', split_ref='prev_row'),
    'M6': Cfg(name='M6 Fallback', use_hsv=True, use_lab=False,
              seg_fallback_adaptive=True),
    # Yellow/orange-tape tracks (shortcut + roundabout). White masks fail;
    # segment the tape hue band only (band defaults tuned for the 2025 test track).
    'O1': Cfg(name='O1 Orange', use_hsv=False, use_lab=False, use_orange=True,
              split_ref='prev_row', roi_top_frac=0.45, min_contour_area=8),
    'O2': Cfg(name='O2 OrangeCurve', use_hsv=False, use_lab=False, use_orange=True,
              split_ref='prev_row', roi_top_frac=0.45, min_contour_area=8,
              do_polyfit=True, curvature=True, use_median=True,
              heading_method='two_point'),
    'O3': Cfg(name='O3 OrangeStrict', use_hsv=False, use_lab=False, use_orange=True,
              split_ref='prev_row', roi_top_frac=0.45, min_contour_area=8,
              min_aspect=1.8, min_length=12, morph_kernel=5),
}


def make_cfg(mode='M1', **overrides):
    base = PRESETS.get(mode, PRESETS['M1'])
    # drop None overrides so callers can pass through unset ROS params
    ov = {k: v for k, v in overrides.items() if v is not None}
    return replace(base, **ov) if ov else base


# ==========================================================================
# A: segmentation
# ==========================================================================
def _seg_hsv(bgr, c):
    _, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    return ((s <= c.hsv_s_max) & (v >= c.hsv_v_min)).astype(np.uint8) * 255


def _seg_lab(bgr, c):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    return (lab[:, :, 0] >= c.lab_l_min).astype(np.uint8) * 255


def _seg_orange(bgr, c):
    h, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    return ((h >= c.orange_h_lo) & (h <= c.orange_h_hi)
            & (s >= c.orange_s_min) & (v >= c.orange_v_min)).astype(np.uint8) * 255


def _seg_edges(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    return cv2.Canny(gray, c.canny_lo, c.canny_hi)


def _seg_adaptive(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, c.adaptive_block | 1, c.adaptive_c)


def _segmentation(bgr, c):
    masks = []
    if c.use_hsv:
        masks.append(_seg_hsv(bgr, c))
    if c.use_lab:
        masks.append(_seg_lab(bgr, c))
    if c.use_orange:
        masks.append(_seg_orange(bgr, c))
    if not masks:
        masks.append(_seg_hsv(bgr, c))
    mask = masks[0]
    for m in masks[1:]:
        mask = cv2.bitwise_or(mask, m) if c.fuse == 'or' else cv2.bitwise_and(mask, m)
    if c.edge_validate:
        edges = cv2.dilate(_seg_edges(bgr, c), np.ones((3, 3), np.uint8),
                           iterations=max(1, c.edge_dilate))
        mask = cv2.bitwise_and(mask, edges)
    return mask


def _filter_contours(mask, c):
    if c.min_contour_area <= 0 and c.min_aspect <= 0 and c.min_length <= 0:
        return mask
    out = np.zeros_like(mask)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        if cv2.contourArea(cnt) < c.min_contour_area:
            continue
        if c.min_aspect > 0 or c.min_length > 0:
            (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
            long_side, short_side = max(rw, rh), max(1.0, min(rw, rh))
            if long_side < c.min_length or long_side / short_side < c.min_aspect:
                continue
        cv2.drawContours(out, [cnt], -1, 255, -1)
    return out


def _compute_mask(bgr, c, rmask):
    mask = cv2.bitwise_and(_segmentation(bgr, c), rmask)
    used_fb = False
    if c.seg_fallback_adaptive:
        roi_area = max(1, int(cv2.countNonZero(rmask)))
        if cv2.countNonZero(mask) / roi_area < c.fallback_min_frac:
            mask = cv2.bitwise_and(_seg_adaptive(bgr, c), rmask)
            used_fb = True
    if c.morph_kernel > 0:
        k = np.ones((c.morph_kernel, c.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return _filter_contours(mask, c), used_fb


# ==========================================================================
# B: ROI
# ==========================================================================
def _roi_mask(h, w, c, center_px=None):
    m = np.zeros((h, w), np.uint8)
    y0 = int(h * c.roi_top_frac)
    cx = w / 2 if center_px is None else center_px
    tw, bw = c.trap_top_w * w, c.trap_bot_w * w
    pts = np.array([[cx - bw / 2, h - 1], [cx - tw / 2, y0],
                    [cx + tw / 2, y0], [cx + bw / 2, h - 1]], np.int32)
    cv2.fillPoly(m, [pts], 255)
    return m, y0, pts


# ==========================================================================
# C: extraction
# ==========================================================================
def _extract(mask, c, y0, prev_center_px, prev_width):
    h, w = mask.shape
    cx = w / 2
    seed = cx
    if c.split_ref in ('prev_frame', 'prev_row') and prev_center_px is not None:
        seed = prev_center_px
    ref = seed
    lane_w = prev_width if prev_width else c.lane_width_default * w
    scan_rows = list(range(h - 1, y0 - 1, -2))
    rows, centers, widths = [], [], []
    left_hits = right_hits = 0
    for y in scan_rows:
        xs = np.where(mask[y] > 0)[0]
        if xs.size == 0:
            continue
        left = xs[xs < ref]
        right = xs[xs >= ref]
        width = None
        if left.size and right.size:
            lx, rx = left.max(), right.min()
            center = (lx + rx) / 2.0
            width = float(rx - lx)
            lane_w = width
            left_hits += 1
            right_hits += 1
        elif left.size:
            center = left.max() + lane_w / 2.0
            left_hits += 1
        else:
            center = right.min() - lane_w / 2.0
            right_hits += 1
        rows.append(y)
        centers.append(center)
        widths.append(width)
        if c.split_ref == 'prev_row':
            ref = center
    rows = np.array(rows, float)
    centers = np.array(centers, float)
    if c.lane_width_tol > 0 and rows.size:
        valid_w = np.array([wd for wd in widths if wd is not None], float)
        if valid_w.size:
            med = np.median(valid_w)
            keep = np.array([(wd is None) or abs(wd - med) <= c.lane_width_tol * w
                             for wd in widths])
            rows, centers = rows[keep], centers[keep]
    total = max(1, len(scan_rows))
    return {'rows': rows, 'centers': centers, 'lane_width': lane_w,
            'conf': len(rows) / total, 'left_conf': left_hits / total,
            'right_conf': right_hits / total, 'seed': seed}


# ==========================================================================
# D: state
# ==========================================================================
def _hough_heading(mask, y0):
    lines = cv2.HoughLinesP(mask[y0:], 1, np.pi / 180, threshold=20,
                            minLineLength=15, maxLineGap=10)
    if lines is None:
        return None
    angs = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        ang = np.degrees(np.arctan2(float(x2 - x1), float(y1 - y2)))
        if abs(ang) < 70:
            angs.append(ang)
    return float(np.median(angs)) if angs else None


def _compute_state(det, c, h, w, y0, mask):
    rows, centers = det['rows'], det['centers']
    cx = w / 2
    out = {'center_error': None, 'heading': None, 'heading_label': c.heading_method,
           'poly': None, 'curvature': None}
    if rows.size < 3:
        return out
    order = np.argsort(rows)
    rows, centers = rows[order], centers[order]
    n = rows.size
    k = max(3, int(n * c.near_frac))
    out['center_error'] = float((centers[-k:].mean() - cx) / (w / 2))
    if c.heading_method == 'hough':
        out['heading'] = _hough_heading(mask, y0)
    else:
        kn = max(2, int(n * c.near_frac))
        kf = max(2, int(n * c.far_frac))
        x_near, y_near = centers[-kn:].mean(), rows[-kn:].mean()
        x_far, y_far = centers[:kf].mean(), rows[:kf].mean()
        if c.heading_method == 'norm_slope':
            out['heading'] = float((x_far - x_near) / (w / 2))
            out['heading_label'] = 'norm_slope'
        else:
            if c.heading_method == 'slope':
                a, b = np.polyfit(rows, centers, 1)
                x_near, x_far = a * rows.max() + b, a * rows.min() + b
                y_near, y_far = rows.max(), rows.min()
            denom = (y_near - y_far) or 1e-6
            out['heading'] = float(np.degrees(np.arctan2(x_far - x_near, denom)))
    if (c.do_polyfit or c.curvature) and n >= 6:
        poly = np.polyfit(rows, centers, 2)
        out['poly'] = poly
        if c.curvature:
            out['curvature'] = float(poly[0] * 1000.0)
    return out


# ==========================================================================
# E/F: temporal + failsafe
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


# ==========================================================================
# Public pipeline (stateful across frames)
# ==========================================================================
class LanePipeline:
    def __init__(self, cfg):
        self.cfg = cfg
        self.stab = _Stabilizer(cfg)
        self.prev_center_px = None
        self.prev_width = None
        self._static_roi = None
        self._size = None

    def process(self, bgr):
        c = self.cfg
        h, w = bgr.shape[:2]
        if self._size != (h, w):
            self._size = (h, w)
            self._static_roi = _roi_mask(h, w, c)
        if c.dynamic_roi and self.prev_center_px is not None:
            cpx = w / 2 + (self.prev_center_px - w / 2) * c.dynamic_roi_gain
            rmask, y0, trap = _roi_mask(h, w, c, center_px=cpx)
        else:
            rmask, y0, trap = self._static_roi

        mask, used_fb = _compute_mask(bgr, c, rmask)
        det = _extract(mask, c, y0, self.prev_center_px, self.prev_width)
        st = _compute_state(det, c, h, w, y0, mask)
        ema, fstate = self.stab.update(st['center_error'], det['conf'])

        self.prev_width = det['lane_width']
        if ema is not None:
            self.prev_center_px = w / 2 + ema * (w / 2)

        overlay = self._draw(bgr, mask, det, st, y0, trap, ema, fstate, used_fb)
        state = {
            'center_error': st['center_error'], 'ema': ema,
            'heading': st['heading'], 'heading_label': st['heading_label'],
            'confidence': det['conf'], 'left_conf': det['left_conf'],
            'right_conf': det['right_conf'], 'curvature': st['curvature'],
            'state': fstate, 'used_fallback': used_fb,
        }
        return overlay, state

    def _draw(self, bgr, mask, det, st, y0, trap, ema, fstate, used_fb):
        c = self.cfg
        h, w = bgr.shape[:2]
        img = bgr.copy()
        cv2.polylines(img, [trap], True, (0, 200, 255), 1)
        cv2.line(img, (int(det['seed']), y0), (int(det['seed']), h - 1), (200, 120, 0), 1)
        for y, xc in zip(det['rows'], det['centers']):
            cv2.circle(img, (int(xc), int(y)), 1, (0, 255, 0), -1)
        if st['poly'] is not None and c.do_polyfit:
            ys = np.arange(y0, h)
            xs = np.polyval(st['poly'], ys)
            pts = np.array([[int(x), int(y)] for x, y in zip(xs, ys) if 0 <= x < w], np.int32)
            if len(pts) > 1:
                cv2.polylines(img, [pts], False, (255, 0, 255), 1)
        if ema is not None:
            ex = int(w / 2 + ema * (w / 2))
            cv2.line(img, (ex, y0), (ex, h - 1), (0, 0, 255), 2)

        def f(v, s=''):
            return f'{v:+.2f}{s}' if v is not None else 'n/a'
        col = (0, 255, 0) if fstate == 'OK' else (0, 200, 255)
        txt = [f"{c.name} {fstate}{' FB' if used_fb else ''}",
               f"cen {f(st['center_error'])} ema {f(ema)}",
               f"hd {f(st['heading'])}[{st['heading_label']}] cf {det['conf']:.2f}"]
        for i, t in enumerate(txt):
            cv2.putText(img, t, (2, 10 + i * 10), cv2.FONT_HERSHEY_SIMPLEX, 0.30, col, 1)
        return img
