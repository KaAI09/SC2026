"""Pure (ROS-independent) lane-detection pipeline. Single source of truth.

Composable axes A..F with condition-based groups G1..G6 (white / yellow, per the
two-track scope; see offline/LANE_DETECTION.md §4), imported by BOTH the online
perception node and the offline tools (offline/). Depends only on cv2 + numpy so
it runs inside the ROS2 node and off-board.

Usage:
    from driving_core.lane_core import LanePipeline, PRESETS, make_cfg
    pipe = LanePipeline(make_cfg('G5', colors=('white', 'yellow')))
    overlay_bgr, state = pipe.process(frame_bgr)   # state: dict (center_error, ...)

The pipeline NEVER commands the vehicle; it only produces a lane state estimate
and a debug overlay. Control mapping is a separate, later stage.
"""
from collections import deque
from dataclasses import dataclass, replace

import cv2
import numpy as np


# ==========================================================================
# Config + presets (single source; imported by online node + offline tools)
# ==========================================================================
@dataclass
class Cfg:
    name: str = 'G1'
    # A: segmentation — two tracks use WHITE + YELLOW only ('orange' retired).
    # `colors` is a subset of ('white','yellow'); masks are OR-combined.
    colors: tuple = ('white',)
    white_use_lab: bool = False        # OR LAB L-channel for brightness robustness
    edge_validate: bool = False
    seg_fallback_adaptive: bool = False
    fallback_min_frac: float = 0.004
    # white marking band (2025 dashcam measured: S p95~16, V p5~207 -> tight)
    white_s_max: int = 60
    white_v_min: int = 185
    lab_l_min: int = 170
    # yellow marking band (2025 dashcam measured: H 22-32, S>=66, V>=112)
    yellow_h_lo: int = 18
    yellow_h_hi: int = 36
    yellow_s_min: int = 65
    yellow_v_min: int = 100
    canny_lo: int = 50
    canny_hi: int = 150
    edge_dilate: int = 2
    morph_kernel: int = 3
    adaptive_block: int = 21
    adaptive_c: int = -5
    # B: ROI (2025 dashcam heatmap: keep bottom ~65-70%, wide top)
    roi_top_frac: float = 0.35
    trap_top_w: float = 0.80
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


# Condition-based experiment groups (2025 dashcam analysis; two-track scope).
# Bands/ROI are measured defaults (see offline/LANE_DETECTION.md §4); 2026 car
# camera re-tunes them. Groups are specialists per observed condition.
PRESETS = {
    # white boundary, straight / two-line (near lanes)
    'G1': Cfg(name='G1 white_line', colors=('white',),
              roi_top_frac=0.35, trap_top_w=0.80, split_ref='center'),
    # white boundary, curve / single-line (lane_width fallback + polyfit)
    'G2': Cfg(name='G2 white_curve', colors=('white',), white_use_lab=True,
              white_v_min=175, roi_top_frac=0.35, trap_top_w=0.80,
              split_ref='prev_row', do_polyfit=True, curvature=True,
              use_median=True, heading_method='two_point'),
    # yellow solid, roundabout curve
    'G3': Cfg(name='G3 yellow_solid', colors=('yellow',),
              roi_top_frac=0.30, trap_top_w=0.80, split_ref='prev_row',
              do_polyfit=True, curvature=True, use_median=True,
              heading_method='two_point', min_contour_area=8),
    # yellow dashed guide line (bridge gaps; do NOT aspect/length-filter dashes)
    'G4': Cfg(name='G4 yellow_dashed', colors=('yellow',),
              roi_top_frac=0.30, trap_top_w=0.80, split_ref='prev_row',
              morph_kernel=5, min_contour_area=8),
    # white + yellow coexisting (junction / roundabout entry) -> fused mask
    'G5': Cfg(name='G5 white_yellow', colors=('white', 'yellow'),
              roi_top_frac=0.30, trap_top_w=0.80, split_ref='prev_row'),
    # low-contrast / blur safety net (LAB assist + adaptive fallback + hold)
    'G6': Cfg(name='G6 robust_lowlight', colors=('white',), white_use_lab=True,
              white_s_max=70, white_v_min=160, seg_fallback_adaptive=True,
              roi_top_frac=0.35, trap_top_w=0.80),
}


def make_cfg(mode='G1', **overrides):
    base = PRESETS.get(mode, PRESETS['G1'])
    # drop None overrides so callers can pass through unset ROS params
    ov = {k: v for k, v in overrides.items() if v is not None}
    return replace(base, **ov) if ov else base


# ==========================================================================
# A: segmentation
# ==========================================================================
def _seg_white(bgr, c):
    _, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    return ((s <= c.white_s_max) & (v >= c.white_v_min)).astype(np.uint8) * 255


def _seg_lab(bgr, c):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    return (lab[:, :, 0] >= c.lab_l_min).astype(np.uint8) * 255


def _seg_yellow(bgr, c):
    h, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    return ((h >= c.yellow_h_lo) & (h <= c.yellow_h_hi)
            & (s >= c.yellow_s_min) & (v >= c.yellow_v_min)).astype(np.uint8) * 255


def _seg_edges(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    return cv2.Canny(gray, c.canny_lo, c.canny_hi)


def _seg_adaptive(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, c.adaptive_block | 1, c.adaptive_c)


def _segmentation(bgr, c):
    masks = []
    if 'white' in c.colors:
        wm = _seg_white(bgr, c)
        if c.white_use_lab:
            wm = cv2.bitwise_or(wm, _seg_lab(bgr, c))
        masks.append(wm)
    if 'yellow' in c.colors:
        masks.append(_seg_yellow(bgr, c))
    if not masks:
        masks.append(_seg_white(bgr, c))
    mask = masks[0]
    for m in masks[1:]:      # multiple colors -> OR (white OR yellow)
        mask = cv2.bitwise_or(mask, m)
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

    def process(self, bgr, debug=False):
        """Run one frame. Returns (overlay, state); with debug=True also returns
        a third dict of intermediates (mask, det, st, y0, trap, ema, fstate,
        used_fb) so offline tools can render their own panels from this single
        source instead of re-implementing the pipeline."""
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
        if debug:
            dbg = {'mask': mask, 'det': det, 'st': st, 'y0': y0, 'trap': trap,
                   'ema': ema, 'fstate': fstate, 'used_fb': used_fb}
            return overlay, state, dbg
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


def render_panels(bgr, dbg, cfg):
    """온라인 디버그용 다패널(3패널) 합성: [ 입력+ROI+split | mask | 검출+상태 ].
    `LanePipeline.process(debug=True)`의 dbg 중간산물만 사용(파이프라인 재구현 없음).
    perception_node가 이 합성 이미지를 /lane/debug/compressed 로 발행 → recorder가 저장.
    (오프라인 _common.three_panel과 동일 스타일; ROS 노드가 import 가능하도록 코어에 둠.)
    """
    h, w = bgr.shape[:2]
    mask, det, st = dbg['mask'], dbg['det'], dbg['st']
    y0, trap, ema, fstate, used_fb = (dbg['y0'], dbg['trap'], dbg['ema'],
                                      dbg['fstate'], dbg['used_fb'])

    def _label(img, text):
        cv2.rectangle(img, (0, 0), (img.shape[1], 14), (0, 0, 0), -1)
        cv2.putText(img, text, (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
        return img

    def _f(v, s=''):
        return f'{v:+.2f}{s}' if v is not None else 'n/a'

    # P1: 입력 + ROI 사다리꼴 + split 기준선
    p1 = bgr.copy()
    cv2.polylines(p1, [trap], True, (0, 200, 255), 1)
    cv2.line(p1, (int(det['seed']), y0), (int(det['seed']), h - 1), (200, 120, 0), 1)
    _label(p1, f'1 input+ROI  {cfg.name}')

    # P2: segmentation mask
    p2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    _label(p2, '2 mask' + (' [adaptive fb]' if used_fb else ''))

    # P3: 행 중심점 + polyfit + ema + 상태 텍스트
    p3 = bgr.copy()
    for y, xc in zip(det['rows'], det['centers']):
        cv2.circle(p3, (int(xc), int(y)), 1, (0, 255, 0), -1)
    if st['poly'] is not None and cfg.do_polyfit:
        ys = np.arange(y0, h)
        xs = np.polyval(st['poly'], ys)
        pts = np.array([[int(x), int(y)] for x, y in zip(xs, ys) if 0 <= x < w], np.int32)
        if len(pts) > 1:
            cv2.polylines(p3, [pts], False, (255, 0, 255), 1)
    if ema is not None:
        ex = int(w / 2 + ema * (w / 2))
        cv2.line(p3, (ex, y0), (ex, h - 1), (0, 0, 255), 2)
    _label(p3, '3 detection')
    lines = [f"cen {_f(st['center_error'])} ema {_f(ema)}",
             f"hd {_f(st['heading'])}[{st['heading_label']}]",
             f"conf {det['conf']:.2f} L/R {det['left_conf']:.2f}/{det['right_conf']:.2f}",
             f"state {fstate}" + (' FB' if used_fb else '')]
    col = (0, 255, 0) if 'OK' in fstate else (0, 200, 255)
    for i, t in enumerate(lines):
        cv2.putText(p3, t, (3, 26 + i * 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)

    return np.hstack([p1, p2, p3])
