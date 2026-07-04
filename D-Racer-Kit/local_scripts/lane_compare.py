#!/usr/bin/env python3
"""Side-by-side comparison of lane-detection presets on the SAME frames.

LOCAL-ONLY tool built on top of lane_preview.py. Instead of one MP4 per mode,
this samples a few frames from a clip, runs each preset's stateful pipeline
(temporal state preserved by iterating every frame), and tiles the *detection*
panels into a single PNG grid:

        rows  = sampled frames (time)
        cols  = presets / parameter combinations

so every combination is judged on identical frames. Purpose: pick the best
mode + threshold combo for a given clip at a glance.

    ../../.venv/bin/python lane_compare.py CLIP.mp4 --modes M1,M2,M3,M4,M5,M6
    ../../.venv/bin/python lane_compare.py CLIP.mp4 --modes O1,O2,O3,Y1 --frames 5

Extra candidate presets tuned for the 2025 test track (yellow tape, measured
hue ~22-32) are registered below as Y1..Y3 without touching lane_preview's
shipped PRESETS.
"""
import argparse
import os
from dataclasses import replace

import cv2
import numpy as np

import lane_preview as lp


# Explicit-band reference presets. The shipped O1-O3 band defaults are now tuned
# to the 2025 test track (H 15-38, S>=70, V>=90), so Y1-Y3 currently match O1-O3;
# they stay as a fixed reference if the shipped defaults are later changed.
EXTRA = {
    'Y1': replace(lp.PRESETS['O1'], name='Y1 Yellow25',
                  orange_h_lo=15, orange_h_hi=38, orange_s_min=70, orange_v_min=90),
    'Y2': replace(lp.PRESETS['O2'], name='Y2 Yellow25Curve',
                  orange_h_lo=15, orange_h_hi=38, orange_s_min=70, orange_v_min=90),
    'Y3': replace(lp.PRESETS['O3'], name='Y3 Yellow25Strict',
                  orange_h_lo=15, orange_h_hi=38, orange_s_min=70, orange_v_min=90),
}


def all_presets():
    d = dict(lp.PRESETS)
    d.update(EXTRA)
    return d


def detection_panel(frame, mask, det, st, cfg, y0, trap, ema, fstate, used_fb, scale):
    """The 3rd (detection) panel only, with a compact stats overlay."""
    h, w = frame.shape[:2]
    p = frame.copy()
    cv2.polylines(p, [trap], True, (0, 200, 255), 1)
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
    p = cv2.resize(p, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

    def f(v):
        return f'{v:+.2f}' if v is not None else 'n/a'
    col = (0, 255, 0) if 'OK' in fstate else (0, 200, 255)
    txt = [cfg.name,
           f"cen{f(st['center_error'])} cf{det['conf']:.2f}",
           f"hd{f(st['heading'])} {fstate.split('(')[0]}"]
    cv2.rectangle(p, (0, 0), (p.shape[1], 12 + 13 * len(txt)), (0, 0, 0), -1)
    for i, t in enumerate(txt):
        cv2.putText(p, t, (3, 12 + i * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1)
    return p


def run_mode(path, cfg, sample_idx, scale):
    """Iterate the whole clip (temporal state) but keep only sampled panels."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {path}')
    ok, frame = cap.read()
    if not ok:
        raise SystemExit('empty video')
    h, w = frame.shape[:2]
    static_roi = lp.roi_mask(h, w, cfg)
    stab = lp.Stabilizer(cfg)
    prev_center_px = prev_width = None
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    want = set(int(i) for i in sample_idx)
    out = {}
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if cfg.dynamic_roi and prev_center_px is not None:
            cpx = w / 2 + (prev_center_px - w / 2) * cfg.dynamic_roi_gain
            rmask, y0, trap = lp.roi_mask(h, w, cfg, center_px=cpx)
        else:
            rmask, y0, trap = static_roi
        mask, used_fb = lp.compute_mask(frame, cfg, rmask)
        det = lp.extract(mask, cfg, y0, prev_center_px, prev_width)
        st = lp.compute_state(det, cfg, h, w, y0, mask)
        ema, fstate = stab.update(st['center_error'], det['conf'])
        prev_width = det['lane_width']
        if ema is not None:
            prev_center_px = w / 2 + ema * (w / 2)
        if i in want:
            out[i] = detection_panel(frame, mask, det, st, cfg, y0, trap,
                                     ema, fstate, used_fb, scale)
        i += 1
    cap.release()
    return out, i


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input')
    ap.add_argument('--modes', default='M1,M2,M3,M4,M5,M6')
    ap.add_argument('--frames', type=int, default=4, help='number of sampled frames (rows)')
    ap.add_argument('--scale', type=int, default=3)
    ap.add_argument('--output')
    args = ap.parse_args()

    presets = all_presets()
    modes = [m.strip() for m in args.modes.split(',') if m.strip()]
    for m in modes:
        if m not in presets:
            raise SystemExit(f'unknown mode {m}; have {list(presets)}')

    cap = cv2.VideoCapture(args.input)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    # skip head/tail, evenly sample
    sample_idx = np.linspace(n * 0.1, n * 0.9, args.frames).astype(int)

    cols = []
    for m in modes:
        panels, _ = run_mode(args.input, presets[m], sample_idx, args.scale)
        col = np.vstack([panels[int(i)] for i in sample_idx if int(i) in panels])
        cols.append(col)
    grid = np.hstack(cols)

    base = os.path.splitext(os.path.basename(args.input))[0]
    out = args.output or f'rslt/{base}__cmp_{"_".join(modes)}.png'
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    cv2.imwrite(out, grid)
    print(f'{base}: {len(modes)} modes x {args.frames} frames (of {n}) -> {out}')


if __name__ == '__main__':
    main()
