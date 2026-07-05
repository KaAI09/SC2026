#!/usr/bin/env python3
"""Stage 2 (perception) — COMPARE experiment groups across clips, then hand off.

Because the 6 groups are condition SPECIALISTS, this does not crown one overall
winner. It runs every group on every clip (full temporal state), aggregates
detector-quality metrics into a group x clip matrix, and emits BOTH:
  1. a metric matrix report (printed table + coverage heatmap PNG)
  2. a per-condition detection grid PNG (rows = sampled frames, cols = groups)
so a human decides single-config vs section-switching and then exports.

Export is NOT automatic: pass --export GROUP --profile PATH to write that group's
perception section (in place, other sections preserved).

    python perception_select.py "Dashcam(2025 Track)"/0704*.mp4
    python perception_select.py CLIPS... --groups G1,G2,G5
    python perception_select.py CLIPS... --export G5 \
        --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml

Requires driving_core importable (pip install -e D-Racer-Kit/src/driving_core).
"""
import argparse
import os

import cv2
import numpy as np

from driving_core.lane_core import LanePipeline, PRESETS, make_cfg

import _common as cm


# Manual clip -> condition labels (2025 dashcam analysis; see PIPELINE.md §5b).
DEFAULT_LABELS = {
    '070401': 'white_line', '070403': 'white_line',
    '070411': 'white_curve', '070413': 'white_curve', '070412': 'white_curve',
    '070408': 'yellow_solid', '070409': 'yellow_solid', '070410': 'yellow_solid',
    '070404': 'yellow_dashed', '070405': 'yellow_dashed',
    '070406': 'white_yellow', '070407': 'robust', '070402': 'white_line',
}

METRIC_KEYS = ['coverage', 'valid_frac', 'center_bias', 'center_jitter',
               'heading_jitter', 'lr_imbalance', 'outlier_rate']


def run_group_on_clip(path, cfg, want_idx=None):
    """Iterate the whole clip (temporal state); return per-frame states and, for
    frames in want_idx, the (frame, dbg) for panel rendering."""
    pipe = LanePipeline(cfg)
    states, panels = [], {}
    want = set(int(i) for i in (want_idx if want_idx is not None else []))
    for i, frame in enumerate(cm.iter_frames(path)):
        _, state, dbg = pipe.process(frame, debug=True)
        states.append(state)
        if i in want:
            panels[i] = (frame, dbg)
    return states, panels


def score_heatmap(matrix, groups, clips, out):
    """matrix[g][c] = composite quality score. Render a labelled group x clip PNG."""
    rows, cols = len(groups), len(clips)
    cell, top, left = 46, 60, 150
    img = np.full((top + rows * cell, left + cols * cell, 3), 30, np.uint8)
    for r, g in enumerate(groups):
        cv2.putText(img, g, (4, top + r * cell + cell // 2 + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1)
    for cc, cl in enumerate(clips):
        x = left + cc * cell
        cv2.putText(img, cl[-4:], (x + 4, top - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (200, 200, 200), 1)
        cv2.putText(img, DEFAULT_LABELS.get(cl, '?')[:7], (x + 2, top - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 180, 220), 1)
    for r in range(rows):
        for cc in range(cols):
            v = matrix[r][cc]
            v = 0.0 if v != v else v            # nan -> 0
            color = cv2.applyColorMap(np.uint8([[int(v * 255)]]), cv2.COLORMAP_VIRIDIS)[0][0]
            x, y = left + cc * cell, top + r * cell
            cv2.rectangle(img, (x, y), (x + cell - 2, y + cell - 2),
                          tuple(int(z) for z in color), -1)
            cv2.putText(img, f'{v:.2f}', (x + 5, y + cell // 2 + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    cv2.imwrite(out, img)


def detection_grid(clips, groups, cfgs, frames_per_clip, out):
    """rows = sampled frames across clips, cols = groups; cells = detection tiles."""
    col_imgs = {g: [] for g in groups}
    for cl_path, cl in clips:
        cap, fps, n = cm.open_clip(cl_path)
        cap.release()
        idx = np.linspace(n * 0.2, n * 0.8, frames_per_clip).astype(int)
        for g in groups:
            _, panels = run_group_on_clip(cl_path, cfgs[g], idx)
            for i in idx:
                if int(i) in panels:
                    frame, dbg = panels[int(i)]
                    col_imgs[g].append(cm.detection_tile(frame, dbg, cfgs[g], scale=2))
    cols = [np.vstack(col_imgs[g]) for g in groups if col_imgs[g]]
    if cols:
        cv2.imwrite(out, np.hstack(cols))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clips', nargs='+', help='clip mp4 paths')
    ap.add_argument('--groups', default=','.join(PRESETS))
    ap.add_argument('--frames', type=int, default=3, help='sampled frames/clip for grid')
    ap.add_argument('--outdir', default='rslt')
    ap.add_argument('--export', help='write this GROUP as the perception profile section')
    ap.add_argument('--profile', help='profile YAML path for --export')
    args = ap.parse_args()

    groups = [g.strip() for g in args.groups.split(',') if g.strip()]
    for g in groups:
        if g not in PRESETS:
            raise SystemExit(f'unknown group {g}; have {list(PRESETS)}')
    cfgs = {g: make_cfg(g) for g in groups}
    clips = [(p, cm.clip_name(p)) for p in args.clips]
    os.makedirs(args.outdir, exist_ok=True)

    # --- metric matrix: groups x clips ---
    print(f'# perception_select: {len(groups)} groups x {len(clips)} clips')
    print('# score = coverage x stability(1/(1+8*jitter)) x (1-outlier_rate)\n')
    metrics = {}   # (group, clip) -> dict
    score = [[0.0] * len(clips) for _ in groups]
    for r, g in enumerate(groups):
        for cc, (path, cl) in enumerate(clips):
            states, _ = run_group_on_clip(path, cfgs[g])
            m = cm.perception_metrics(states)
            m['score'] = cm.quality_score(m)
            metrics[(g, cl)] = m
            score[r][cc] = m['score']

    # per-clip best group by composite score
    hdr = f'{"clip":10s} {"cond":13s} ' + ' '.join(f'{g:>6s}' for g in groups) + '   best'
    print(hdr); print('-' * len(hdr))
    for cc, (path, cl) in enumerate(clips):
        scs = [metrics[(g, cl)]['score'] for g in groups]
        best = groups[int(np.argmax(scs))]
        row = ' '.join(f'{v:6.2f}' for v in scs)
        print(f'{cl:10s} {DEFAULT_LABELS.get(cl, "?"):13s} {row}   {best}')
    # detail: coverage / center_jitter behind the score (to expose 'found vs tracked')
    print('\n# detail  coverage | center_jitter  (per group, mean over clips):')
    for r, g in enumerate(groups):
        cov = np.mean([metrics[(g, cl)]['coverage'] for _, cl in clips])
        cj = np.nanmean([metrics[(g, cl)]['center_jitter'] for _, cl in clips])
        sc = np.array(score[r])
        print(f'  {g:18s} score mean={sc.mean():.2f} worst={sc.min():.2f}'
              f'  | cov={cov:.2f} jitter={cj:.3f}')

    heat = os.path.join(args.outdir, 'perception_matrix.png')
    grid = os.path.join(args.outdir, 'perception_grid.png')
    score_heatmap(score, groups, [c for _, c in clips], heat)
    detection_grid(clips, groups, cfgs, args.frames, grid)
    print(f'\nscore matrix heatmap -> {heat}\ndetection grid -> {grid}')

    # --- optional hand-off (human-triggered) ---
    if args.export:
        if args.export not in PRESETS or not args.profile:
            raise SystemExit('--export GROUP requires a valid group and --profile PATH')
        c = make_cfg(args.export)
        sec = {'mode': args.export, 'colors': list(c.colors),
               'roi_top_frac': c.roi_top_frac, 'trap_top_w': c.trap_top_w,
               'white_s_max': c.white_s_max, 'white_v_min': c.white_v_min,
               'yellow_h_lo': c.yellow_h_lo, 'yellow_h_hi': c.yellow_h_hi,
               'yellow_s_min': c.yellow_s_min, 'yellow_v_min': c.yellow_v_min}
        cm.write_profile_section(args.profile, 'perception', sec)
        print(f'exported {args.export} -> {args.profile} [perception]')


if __name__ == '__main__':
    main()
