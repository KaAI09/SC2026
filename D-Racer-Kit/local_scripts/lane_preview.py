#!/usr/bin/env python3
"""Offline lane-detection preview / comparison over recorded clips.

LOCAL-ONLY experimentation tool. Config-driven pipeline so each axis is
composable and "modes" are just presets:

    A segmentation -> B ROI -> C extraction -> D state -> E temporal -> F failsafe(annotated)

Every axis option is toggisable via CLI so you can test many combinations:
    ../../.venv/bin/python lane_preview.py CLIP.mp4 --mode M2 \
        --split prev_row --heading two_point --aspect 2.0 --length 20 \
        --dynamic-roi --per-lane-conf --median

for m in M1 M2 M3 M4 M5 M6; do ../../.venv/bin/python lane_preview.py "../bagfile/bag_20260703_145235_camera_image_compressed.mp4" --mode $m; done

Output: side-by-side panel MP4  [ original+ROI | mask | detection+stats ].
BEV/IPM is intentionally NOT here (optional / low priority).
"""
import argparse
import os
from collections import deque
from dataclasses import dataclass, replace

import cv2
import numpy as np


# ==========================================================================
# Config (one dataclass; presets are overrides)
# ==========================================================================
@dataclass
class Cfg:
    name: str = 'M1'
    # --- A: segmentation ---
    use_hsv: bool = True
    use_lab: bool = False
    fuse: str = 'or'                    # combine hsv+lab: 'or'|'and'
    edge_validate: bool = False         # cascade: keep white near Canny edges
    seg_fallback_adaptive: bool = False  # if primary mask too sparse -> adaptive
    fallback_min_frac: float = 0.004    # white fraction in ROI to trigger fallback
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
    # --- B: ROI ---
    roi_top_frac: float = 0.55
    trap_top_w: float = 0.55
    trap_bot_w: float = 1.0
    dynamic_roi: bool = False           # center ROI on previous lane center
    dynamic_roi_gain: float = 1.0
    # --- C: extraction ---
    min_contour_area: int = 15
    min_aspect: float = 0.0             # 0 = off; keep elongated contours only
    min_length: float = 0.0            # 0 = off; keep long contours only
    lane_width_default: float = 0.6    # fraction of W (single-line fallback)
    lane_width_tol: float = 0.0        # 0 = off; reject rows w/ width far from median
    split_ref: str = 'center'          # 'center'|'prev_frame'|'prev_row'
    do_polyfit: bool = False
    curvature: bool = False
    per_lane_conf: bool = False
    # --- D: heading ---
    heading_method: str = 'slope'      # 'slope'|'two_point'|'norm_slope'|'hough'
    near_frac: float = 0.30            # bottom fraction of valid rows -> "near"
    far_frac: float = 0.30             # top fraction of valid rows -> "far"
    # --- E: temporal ---
    ema_alpha: float = 0.4
    outlier_jump: float = 0.5
    use_median: bool = False
    median_window: int = 5
    # --- F: failsafe (annotated only) ---
    conf_low: float = 0.25
    lost_stop_frames: int = 8


PRESETS = {
    'M1': Cfg(name='M1 Basic',      use_hsv=True, use_lab=False),
    'M2': Cfg(name='M2 Brightness', use_hsv=True, use_lab=True),
    'M3': Cfg(name='M3 Strict',     use_hsv=True, use_lab=True, edge_validate=True),
    'M4': Cfg(name='M4 Heading',    use_hsv=True, use_lab=True,
              heading_method='hough', split_ref='prev_row'),
    'M5': Cfg(name='M5 Curve',      use_hsv=True, use_lab=True,
              do_polyfit=True, curvature=True, use_median=True,
              heading_method='two_point', split_ref='prev_row'),
    'M6': Cfg(name='M6 Fallback',   use_hsv=True, use_lab=False,
              seg_fallback_adaptive=True),
    # Yellow/orange-tape tracks (shortcut + roundabout) — white masks fail, so
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


# ==========================================================================
# A: segmentation
# ==========================================================================
def seg_hsv_white(bgr, c):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _, s, v = cv2.split(hsv)
    return ((s <= c.hsv_s_max) & (v >= c.hsv_v_min)).astype(np.uint8) * 255


def seg_lab_l(bgr, c):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    return (lab[:, :, 0] >= c.lab_l_min).astype(np.uint8) * 255


def seg_orange(bgr, c):
    h, s, v = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    return ((h >= c.orange_h_lo) & (h <= c.orange_h_hi)
            & (s >= c.orange_s_min) & (v >= c.orange_v_min)).astype(np.uint8) * 255


def seg_edges(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    return cv2.Canny(gray, c.canny_lo, c.canny_hi)


def seg_adaptive(bgr, c):
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    block = c.adaptive_block | 1  # must be odd
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, block, c.adaptive_c)


def segmentation(bgr, c):
    masks = []
    if c.use_hsv:
        masks.append(seg_hsv_white(bgr, c))
    if c.use_lab:
        masks.append(seg_lab_l(bgr, c))
    if c.use_orange:
        masks.append(seg_orange(bgr, c))
    if not masks:
        masks.append(seg_hsv_white(bgr, c))
    mask = masks[0]
    for m in masks[1:]:
        mask = cv2.bitwise_or(mask, m) if c.fuse == 'or' else cv2.bitwise_and(mask, m)
    if c.edge_validate:
        edges = seg_edges(bgr, c)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=max(1, c.edge_dilate))
        mask = cv2.bitwise_and(mask, edges)
    return mask


def compute_mask(bgr, c, rmask):
    mask = cv2.bitwise_and(segmentation(bgr, c), rmask)
    roi_area = max(1, int(cv2.countNonZero(rmask)))
    used_fallback = False
    if c.seg_fallback_adaptive and cv2.countNonZero(mask) / roi_area < c.fallback_min_frac:
        mask = cv2.bitwise_and(seg_adaptive(bgr, c), rmask)
        used_fallback = True
    if c.morph_kernel > 0:
        k = np.ones((c.morph_kernel, c.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = filter_contours(mask, c)
    return mask, used_fallback


# ==========================================================================
# B: ROI (bottom fraction + trapezoid, optionally re-centered)
# ==========================================================================
def roi_mask(h, w, c, center_px=None):
    m = np.zeros((h, w), np.uint8)
    y0 = int(h * c.roi_top_frac)
    cx = w / 2 if center_px is None else center_px
    tw, bw = c.trap_top_w * w, c.trap_bot_w * w
    pts = np.array([[cx - bw / 2, h - 1], [cx - tw / 2, y0],
                    [cx + tw / 2, y0], [cx + bw / 2, h - 1]], np.int32)
    cv2.fillPoly(m, [pts], 255)
    return m, y0, pts


# ==========================================================================
# C: extraction  (contour filter -> per-row left/right -> lane center)
# ==========================================================================
def filter_contours(mask, c):
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
            if long_side < c.min_length:
                continue
            if long_side / short_side < c.min_aspect:
                continue
        cv2.drawContours(out, [cnt], -1, 255, -1)
    return out


def extract(mask, c, y0, prev_center_px, prev_width):
    h, w = mask.shape
    cx = w / 2
    seed = prev_center_px if (c.split_ref == 'prev_frame' and prev_center_px is not None) else cx
    if c.split_ref == 'prev_row' and prev_center_px is not None:
        seed = prev_center_px
    ref = seed
    lane_w = prev_width if prev_width else c.lane_width_default * w

    scan_rows = list(range(h - 1, y0 - 1, -2))   # bottom -> up
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

    # lane-width outlier rejection (only where both lines were seen)
    if c.lane_width_tol > 0 and rows.size:
        valid_w = np.array([wd for wd in widths if wd is not None], float)
        if valid_w.size:
            med = np.median(valid_w)
            keep = np.array([(wd is None) or abs(wd - med) <= c.lane_width_tol * w
                             for wd in widths])
            rows, centers = rows[keep], centers[keep]

    total = max(1, len(scan_rows))
    return {
        'rows': rows, 'centers': centers, 'lane_width': lane_w,
        'conf': len(rows) / total,
        'left_conf': left_hits / total, 'right_conf': right_hits / total,
        'seed': seed,
    }


# ==========================================================================
# D: state  (center_error, heading_error, curvature, poly)
# ==========================================================================
def hough_heading(mask, y0):
    roi = mask[y0:]
    lines = cv2.HoughLinesP(roi, 1, np.pi / 180, threshold=20,
                            minLineLength=15, maxLineGap=10)
    if lines is None:
        return None
    angs = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):  # handle (N,1,4) and (N,4)
        ang = np.degrees(np.arctan2(float(x2 - x1), float(y1 - y2)))  # vs vertical, + = right/fwd
        if abs(ang) < 70:                                  # drop near-horizontal noise
            angs.append(ang)
    return float(np.median(angs)) if angs else None


def compute_state(det, c, h, w, y0, mask):
    rows, centers = det['rows'], det['centers']
    cx = w / 2
    out = {'center_error': None, 'heading': None, 'heading_label': c.heading_method,
           'poly': None, 'curvature': None}
    if rows.size < 3:
        return out

    order = np.argsort(rows)
    rows, centers = rows[order], centers[order]      # ascending y (top -> bottom)
    n = rows.size

    # center_error: nearest (bottom) rows, normalized to [-1, 1]
    k = max(3, int(n * c.near_frac))
    out['center_error'] = float((centers[-k:].mean() - cx) / (w / 2))

    # heading (unified convention: + = lane bends right going forward/up)
    if c.heading_method == 'hough':
        hd = hough_heading(mask, y0)
        out['heading'] = hd
    else:
        kn = max(2, int(n * c.near_frac))
        kf = max(2, int(n * c.far_frac))
        x_near, y_near = centers[-kn:].mean(), rows[-kn:].mean()
        x_far, y_far = centers[:kf].mean(), rows[:kf].mean()
        if c.heading_method == 'norm_slope':
            out['heading'] = float((x_far - x_near) / (w / 2))   # unitless
            out['heading_label'] = 'norm_slope(unitless)'
        else:  # 'slope' (via linear fit endpoints) or 'two_point' (raw endpoints)
            if c.heading_method == 'slope':
                a, b = np.polyfit(rows, centers, 1)
                x_near = a * rows.max() + b
                x_far = a * rows.min() + b
                y_near, y_far = rows.max(), rows.min()
            denom = (y_near - y_far) if (y_near - y_far) != 0 else 1e-6
            out['heading'] = float(np.degrees(np.arctan2(x_far - x_near, denom)))

    if (c.do_polyfit or c.curvature) and n >= 6:
        poly = np.polyfit(rows, centers, 2)
        out['poly'] = poly
        if c.curvature:
            out['curvature'] = float(poly[0] * 1000.0)  # relative (scaled 2A/2)
    return out


# ==========================================================================
# E/F: temporal + failsafe
# ==========================================================================
class Stabilizer:
    def __init__(self, c):
        self.c = c
        self.ema = None
        self.lost = 0
        self.hist = deque(maxlen=max(1, c.median_window))

    def update(self, center_error, conf):
        c = self.c
        if center_error is None or conf < c.conf_low:
            self.lost += 1
            return self.ema, ('LOST(stop)' if self.lost >= c.lost_stop_frames else 'HOLD(prev)')
        if self.ema is not None and abs(center_error - self.ema) > c.outlier_jump:
            self.lost += 1
            return self.ema, 'OUTLIER(reject)'
        self.lost = 0
        val = center_error
        if c.use_median:
            self.hist.append(center_error)
            val = float(np.median(self.hist))
        self.ema = val if self.ema is None else c.ema_alpha * val + (1 - c.ema_alpha) * self.ema
        return self.ema, ('LOW_CONF(slow)' if conf < c.conf_low * 1.6 else 'OK')


# ==========================================================================
# Visualization: side-by-side panels
# ==========================================================================
def label(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
    cv2.putText(img, text, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return img


def make_panels(bgr, mask, det, st, cfg, y0, trap_pts, ema, fstate, used_fb):
    h, w = bgr.shape[:2]
    p1 = bgr.copy()
    cv2.polylines(p1, [trap_pts], True, (0, 200, 255), 1)
    cv2.line(p1, (int(det['seed']), y0), (int(det['seed']), h - 1), (200, 120, 0), 1)
    label(p1, f'{cfg.name} | ROI(orange) split-ref(blue)')

    p2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    label(p2, 'mask' + (' [adaptive fallback]' if used_fb else ''))

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
    label(p3, 'detection')

    def f(v, s=''):
        return f'{v:+.2f}{s}' if v is not None else 'n/a'
    lines = [f"center: {f(st['center_error'])}  ema: {f(ema)}",
             f"heading: {f(st['heading'])} [{st['heading_label']}]",
             f"conf: {det['conf']:.2f}"]
    if cfg.per_lane_conf:
        lines.append(f"L/R conf: {det['left_conf']:.2f}/{det['right_conf']:.2f}")
    if cfg.curvature:
        lines.append(f"curv: {f(st['curvature'])}")
    lines.append(f"state: {fstate}")
    for i, t in enumerate(lines):
        col = (0, 255, 0) if 'OK' in fstate else (0, 200, 255)
        cv2.putText(p3, t, (3, 30 + i * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1)

    s = 3
    panels = [cv2.resize(p, (w * s, h * s), interpolation=cv2.INTER_NEAREST)
              for p in (p1, p2, p3)]
    return np.hstack(panels)


# ==========================================================================
def run(path, cfg, out_path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    ok, frame = cap.read()
    if not ok:
        raise SystemExit('empty video')
    h, w = frame.shape[:2]
    static_roi = roi_mask(h, w, cfg)

    writer = None
    stab = Stabilizer(cfg)
    prev_center_px = None
    prev_width = None
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if cfg.dynamic_roi and prev_center_px is not None:
            cpx = w / 2 + (prev_center_px - w / 2) * cfg.dynamic_roi_gain
            rmask, y0, trap = roi_mask(h, w, cfg, center_px=cpx)
        else:
            rmask, y0, trap = static_roi

        mask, used_fb = compute_mask(frame, cfg, rmask)
        det = extract(mask, cfg, y0, prev_center_px, prev_width)
        st = compute_state(det, cfg, h, w, y0, mask)
        ema, fstate = stab.update(st['center_error'], det['conf'])

        prev_width = det['lane_width']
        if ema is not None:
            prev_center_px = w / 2 + ema * (w / 2)

        panel = make_panels(frame, mask, det, st, cfg, y0, trap, ema, fstate, used_fb)
        if writer is None:
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     fps, (panel.shape[1], panel.shape[0]))
        writer.write(panel)
        n += 1
    if writer:
        writer.release()
    cap.release()
    print(f'{cfg.name}: {n} frames -> {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input')
    ap.add_argument('--mode', default='M1', choices=list(PRESETS))
    ap.add_argument('--roi-top', type=float)
    ap.add_argument('--split', choices=['center', 'prev_frame', 'prev_row'])
    ap.add_argument('--heading', choices=['slope', 'two_point', 'norm_slope', 'hough'])
    ap.add_argument('--aspect', type=float, help='min contour aspect ratio (0=off)')
    ap.add_argument('--length', type=float, help='min contour long-side length (0=off)')
    ap.add_argument('--lane-width-tol', type=float, help='lane-width outlier tol (0=off)')
    ap.add_argument('--dynamic-roi', action='store_true')
    ap.add_argument('--per-lane-conf', action='store_true')
    ap.add_argument('--median', action='store_true')
    ap.add_argument('--polyfit', action='store_true')
    ap.add_argument('--orange', action='store_true', help='enable orange hue mask')
    ap.add_argument('--orange-h', type=int, nargs=2, metavar=('LO', 'HI'),
                    help='orange hue band (OpenCV H 0-180)')
    ap.add_argument('--orange-s-min', type=int, help='orange min saturation')
    ap.add_argument('--orange-v-min', type=int, help='orange min value')
    ap.add_argument('--output')
    args = ap.parse_args()

    cfg = PRESETS[args.mode]
    ov = {}
    if args.roi_top is not None:
        ov['roi_top_frac'] = args.roi_top
    if args.split:
        ov['split_ref'] = args.split
    if args.heading:
        ov['heading_method'] = args.heading
    if args.aspect is not None:
        ov['min_aspect'] = args.aspect
    if args.length is not None:
        ov['min_length'] = args.length
    if args.lane_width_tol is not None:
        ov['lane_width_tol'] = args.lane_width_tol
    if args.dynamic_roi:
        ov['dynamic_roi'] = True
    if args.per_lane_conf:
        ov['per_lane_conf'] = True
    if args.median:
        ov['use_median'] = True
    if args.polyfit:
        ov['do_polyfit'] = True
    if args.orange:
        ov['use_orange'] = True
    if args.orange_h:
        ov['orange_h_lo'], ov['orange_h_hi'] = args.orange_h
    if args.orange_s_min is not None:
        ov['orange_s_min'] = args.orange_s_min
    if args.orange_v_min is not None:
        ov['orange_v_min'] = args.orange_v_min
    if ov:
        cfg = replace(cfg, **ov)

    base = os.path.splitext(os.path.basename(args.input))[0]
    out = args.output or f'{base}__{args.mode}.mp4'
    run(args.input, cfg, out)


if __name__ == '__main__':
    main()
