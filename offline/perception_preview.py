#!/usr/bin/env python3
"""Stage 1 (perception) — APPLY one experiment group to a clip and visualize.

Runs the SHARED driving_core.lane_core pipeline (exact online code) for a single
condition group (G1..G6) and renders a side-by-side 3-panel MP4:
    [ original + ROI + split-ref | mask | detection + stats ]

This file only APPLIES + visualizes one group. Comparing/ranking groups is stage 2
(perception_select.py). Groups and their measured band/ROI defaults are defined in
driving_core.lane_core.PRESETS (see offline/LANE_DETECTION.md §4).

    python perception_preview.py CLIP.mp4 --group G3
    python perception_preview.py CLIP.mp4 --group G2 --roi-top 0.30 --polyfit
    for g in G1 G2 G5; do python perception_preview.py CLIP.mp4 --group $g; done

Requires driving_core importable (pip install -e D-Racer-Kit/src/driving_core).
"""
import argparse
import os

import cv2

from driving_core.lane_core import LanePipeline, PRESETS, make_cfg

import _common as cm


def build_cfg(args):
    ov = {}
    if args.colors:
        ov['colors'] = tuple(x.strip() for x in args.colors.split(',') if x.strip())
    for cli, field in [('roi_top', 'roi_top_frac'), ('trap_top', 'trap_top_w'),
                       ('split', 'split_ref'), ('heading', 'heading_method'),
                       ('white_s_max', 'white_s_max'), ('white_v_min', 'white_v_min'),
                       ('yellow_s_min', 'yellow_s_min'), ('yellow_v_min', 'yellow_v_min'),
                       ('morph', 'morph_kernel')]:
        v = getattr(args, cli)
        if v is not None:
            ov[field] = v
    if args.yellow_h:
        ov['yellow_h_lo'], ov['yellow_h_hi'] = args.yellow_h
    for flag in ('do_polyfit', 'curvature', 'use_median', 'dynamic_roi',
                 'per_lane_conf', 'white_use_lab'):
        if getattr(args, flag):
            ov[flag] = True
    return make_cfg(args.group, **ov)


def run(path, cfg, out_path):
    pipe = LanePipeline(cfg)
    cap, fps, n = cm.open_clip(path)
    writer, cnt = None, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        _, _, dbg = pipe.process(frame, debug=True)
        panel = cm.three_panel(frame, dbg, cfg)
        if writer is None:
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     fps, (panel.shape[1], panel.shape[0]))
        writer.write(panel)
        cnt += 1
    cap.release()
    if writer:
        writer.release()
    print(f'{cfg.name}: {cnt} frames -> {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input')
    ap.add_argument('--group', default='G1', choices=list(PRESETS))
    ap.add_argument('--colors', help='override color set, e.g. white,yellow')
    ap.add_argument('--roi-top', dest='roi_top', type=float)
    ap.add_argument('--trap-top', dest='trap_top', type=float)
    ap.add_argument('--split', choices=['center', 'prev_frame', 'prev_row'])
    ap.add_argument('--heading', choices=['slope', 'two_point', 'norm_slope', 'hough'])
    ap.add_argument('--white-s-max', dest='white_s_max', type=int)
    ap.add_argument('--white-v-min', dest='white_v_min', type=int)
    ap.add_argument('--yellow-h', type=int, nargs=2, metavar=('LO', 'HI'))
    ap.add_argument('--yellow-s-min', dest='yellow_s_min', type=int)
    ap.add_argument('--yellow-v-min', dest='yellow_v_min', type=int)
    ap.add_argument('--morph', type=int, help='morph_kernel (dashed-gap close)')
    ap.add_argument('--polyfit', dest='do_polyfit', action='store_true')
    ap.add_argument('--curvature', action='store_true')
    ap.add_argument('--median', dest='use_median', action='store_true')
    ap.add_argument('--dynamic-roi', dest='dynamic_roi', action='store_true')
    ap.add_argument('--per-lane-conf', dest='per_lane_conf', action='store_true')
    ap.add_argument('--lab', dest='white_use_lab', action='store_true',
                    help='OR LAB L-channel into the white mask')
    ap.add_argument('--output')
    args = ap.parse_args()

    cfg = build_cfg(args)
    base = cm.clip_name(args.input)
    out = args.output or f'rslt/{base}__{args.group}.mp4'
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    run(args.input, cfg, out)


if __name__ == '__main__':
    main()
