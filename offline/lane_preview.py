#!/usr/bin/env python3
"""Offline lane-detection preview over recorded clips (single-source pipeline).

Runs the SHARED `driving_core.lane_core` pipeline — the exact same code the
online perception node executes — and only adds offline concerns: panel
visualization and video IO. So whatever combination you pick here is what the
car runs. Config-driven: "modes" are `Cfg` presets (M1..M6 white, O1..O3
yellow/orange tape); every axis is CLI-overridable for combination sweeps.

    python lane_preview.py CLIP.mp4 --mode M2 --roi-top 0.6 --polyfit
    for m in M1 M2 M3; do python lane_preview.py CLIP.mp4 --mode $m; done

Output: side-by-side panel MP4  [ original+ROI | mask | detection+stats ].
BEV/IPM is intentionally NOT here (optional / low priority).

Requires driving_core importable (e.g. `pip install -e D-Racer-Kit/src/driving_core`).
"""
import argparse
import os

import cv2
import numpy as np

from driving_core.lane_core import LanePipeline, PRESETS, make_cfg


def label(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
    cv2.putText(img, text, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return img


def make_panels(bgr, dbg, state, cfg):
    """Build the 3-panel view from the shared pipeline's debug intermediates."""
    h, w = bgr.shape[:2]
    mask, det, st = dbg['mask'], dbg['det'], dbg['st']
    y0, trap, ema, fstate, used_fb = (dbg['y0'], dbg['trap'], dbg['ema'],
                                      dbg['fstate'], dbg['used_fb'])

    p1 = bgr.copy()
    cv2.polylines(p1, [trap], True, (0, 200, 255), 1)
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


def run(path, cfg, out_path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    pipe = LanePipeline(cfg)
    writer = None
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        _, state, dbg = pipe.process(frame, debug=True)
        panel = make_panels(frame, dbg, state, cfg)
        if writer is None:
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     fps, (panel.shape[1], panel.shape[0]))
        writer.write(panel)
        n += 1
    if writer:
        writer.release()
    cap.release()
    print(f'{cfg.name}: {n} frames -> {out_path}')


def build_cfg(args):
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
    return make_cfg(args.mode, **ov)


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
    ap.add_argument('--orange', action='store_true', help='enable orange/yellow hue mask')
    ap.add_argument('--orange-h', type=int, nargs=2, metavar=('LO', 'HI'),
                    help='orange/yellow hue band (OpenCV H 0-180)')
    ap.add_argument('--orange-s-min', type=int, help='orange min saturation')
    ap.add_argument('--orange-v-min', type=int, help='orange min value')
    ap.add_argument('--output')
    args = ap.parse_args()

    cfg = build_cfg(args)
    base = os.path.splitext(os.path.basename(args.input))[0]
    out = args.output or f'rslt/{base}__{args.mode}.mp4'
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    run(args.input, cfg, out)


if __name__ == '__main__':
    main()
