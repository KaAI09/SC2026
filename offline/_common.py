"""Shared offline helpers: video IO, panel rendering, perception metrics, and
profile writing. Kept thin — all detection/control LOGIC lives in driving_core;
here we only orchestrate, visualize, score, and hand off.

Used by perception_preview / perception_select (and later control_predict /
control_select). Rendering reads the SAME `dbg` intermediates that
LanePipeline.process(debug=True) returns, so panels never re-implement the pipeline.
"""
import os

import cv2
import numpy as np
import yaml


# ------------------------------------------------------------------ video IO
def open_clip(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    return cap, fps, n


def iter_frames(path):
    cap, fps, n = open_clip(path)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        yield frame
    cap.release()


def clip_name(path):
    return os.path.splitext(os.path.basename(path))[0]


# ------------------------------------------------------------------ drawing
def label(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
    cv2.putText(img, text, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return img


def _draw_detection(p, dbg, cfg):
    """Overlay green row-centers, magenta polyfit, red EMA on a frame copy."""
    h, w = p.shape[:2]
    det, st = dbg['det'], dbg['st']
    y0, ema = dbg['y0'], dbg['ema']
    for y, xc in zip(det['rows'], det['centers']):
        cv2.circle(p, (int(xc), int(y)), 1, (0, 255, 0), -1)
    if st['poly'] is not None and cfg.do_polyfit:
        ys = np.arange(y0, h)
        xs = np.polyval(st['poly'], ys)
        pts = np.array([[int(x), int(y)] for x, y in zip(xs, ys) if 0 <= x < w], np.int32)
        if len(pts) > 1:
            cv2.polylines(p, [pts], False, (255, 0, 255), 1)
    if ema is not None:
        ex = int(w / 2 + ema * (w / 2))
        cv2.line(p, (ex, y0), (ex, h - 1), (0, 0, 255), 2)
    return p


def _fmt(v, s=''):
    return f'{v:+.2f}{s}' if v is not None else 'n/a'


def three_panel(bgr, dbg, cfg, scale=3):
    """Preview 3-panel: [ orig+ROI+split | mask | detection+stats ]."""
    h, w = bgr.shape[:2]
    mask, det, st = dbg['mask'], dbg['det'], dbg['st']
    y0, trap, ema, fstate, used_fb = (dbg['y0'], dbg['trap'], dbg['ema'],
                                      dbg['fstate'], dbg['used_fb'])

    p1 = bgr.copy()
    cv2.polylines(p1, [trap], True, (0, 200, 255), 1)
    cv2.line(p1, (int(det['seed']), y0), (int(det['seed']), h - 1), (200, 120, 0), 1)
    label(p1, f'{cfg.name} | ROI(orange) split(blue)')

    p2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    label(p2, 'mask' + (' [adaptive fb]' if used_fb else ''))

    p3 = _draw_detection(bgr.copy(), dbg, cfg)
    label(p3, 'detection')
    lines = [f"center: {_fmt(st['center_error'])}  ema: {_fmt(ema)}",
             f"heading: {_fmt(st['heading'])} [{st['heading_label']}]",
             f"conf: {det['conf']:.2f}"]
    if cfg.per_lane_conf:
        lines.append(f"L/R: {det['left_conf']:.2f}/{det['right_conf']:.2f}")
    if cfg.curvature:
        lines.append(f"curv: {_fmt(st['curvature'])}")
    lines.append(f"state: {fstate}")
    for i, t in enumerate(lines):
        col = (0, 255, 0) if 'OK' in fstate else (0, 200, 255)
        cv2.putText(p3, t, (3, 30 + i * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1)

    panels = [cv2.resize(p, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
              for p in (p1, p2, p3)]
    return np.hstack(panels)


def detection_tile(bgr, dbg, cfg, scale=3):
    """Single detection panel + compact stats, for comparison grids."""
    h, w = bgr.shape[:2]
    det, st, fstate = dbg['det'], dbg['st'], dbg['fstate']
    p = _draw_detection(bgr.copy(), dbg, cfg)
    cv2.polylines(p, [dbg['trap']], True, (0, 200, 255), 1)
    p = cv2.resize(p, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    txt = [cfg.name,
           f"cen{_fmt(st['center_error'])} cf{det['conf']:.2f}",
           f"hd{_fmt(st['heading'])} {fstate.split('(')[0]}"]
    cv2.rectangle(p, (0, 0), (p.shape[1], 12 + 13 * len(txt)), (0, 0, 0), -1)
    col = (0, 255, 0) if 'OK' in fstate else (0, 200, 255)
    for i, t in enumerate(txt):
        cv2.putText(p, t, (3, 12 + i * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1)
    return p


# ------------------------------------------------------------------ metrics
def perception_metrics(states, tau=0.3):
    """Perception-quality metrics over one clip's per-frame states (detector-only,
    controller-independent). Returns a flat dict of scalars."""
    n = max(1, len(states))
    ce = np.array([s['center_error'] for s in states if s['center_error'] is not None], float)
    hd = np.array([s['heading'] for s in states if s['heading'] is not None], float)
    conf = np.array([s.get('confidence', 0.0) or 0.0 for s in states], float)
    lr = np.array([abs((s.get('left_conf') or 0.0) - (s.get('right_conf') or 0.0))
                   for s in states], float)
    outliers = sum(1 for s in states if s.get('state') in ('OUTLIER', 'HOLD', 'LOST'))

    def jit(a):
        return float(np.mean(np.abs(np.diff(a)))) if a.size > 1 else float('nan')

    return {
        'coverage': float(np.mean(conf >= tau)),          # frac frames conf>=tau
        'valid_frac': float(ce.size / n),                 # frac with a center
        'center_bias': float(ce.mean()) if ce.size else float('nan'),
        'center_jitter': jit(ce),
        'heading_jitter': jit(hd),
        'lr_imbalance': float(lr.mean()),
        'outlier_rate': outliers / n,
        'frames': len(states),
    }


# jitter weight in the composite: center_error is in [-1,1]; a stable clip has
# frame-to-frame |Δ| ~0.02-0.1, a noisy one ~0.3+, so 8*jitter spreads them well.
JITTER_W = 8.0


def quality_score(m):
    """Composite perception quality in [0,1]: reward detecting a lot (coverage),
    stably (low center jitter), without temporal breakdown (low outlier rate).
    Discriminates 'found something' from 'tracked the lane well' — coverage alone
    does not (a brightness mask lights up on any bright marking)."""
    cov = m['coverage']
    cj = m['center_jitter']
    cj = 1.0 if cj != cj else cj                # nan (never valid) -> worst
    stability = 1.0 / (1.0 + JITTER_W * cj)
    return float(cov * stability * (1.0 - m['outlier_rate']))


# ------------------------------------------------------------------ profile IO
def read_profile(path):
    if not os.path.exists(path):
        return {}
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def write_profile_section(path, key, data):
    """In-place update of one profile section (perception|control), preserving the
    other sections. NOTE: PyYAML does not preserve comments; a header is re-added."""
    prof = read_profile(path)
    prof[key] = data
    header = (f'# Driving profile: {prof.get("name", clip_name(path))}\n'
              f'# offline -> online contract (perception_select / control_select).\n'
              f'# Keys map 1:1 to driving_core Cfg (perception) / CtrlCfg (control).\n')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.safe_dump(prof, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    return path
