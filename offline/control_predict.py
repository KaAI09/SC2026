#!/usr/bin/env python3
"""Stage 3 (control) — PREDICT controller commands offline (open-loop, no vehicle).

Inputs: a drive VIDEO + the paired manual CSV (recorder's frame-aligned log) + the
perception PROFILE chosen in stage 2. For each frame we RE-RUN the selected
perception on the video (fresh lane state, not the record-time columns), then step
every candidate controller (dracer_core.control_core.Controller) to predict a
(steering, throttle) command. Output is a wide predicted CSV consumed by stage 4
(control_select.py).

This never actuates — it only computes what each controller WOULD command on the
human's lane states. See offline/CONTROL_DESIGN.md §5 for the open-loop rationale.

    python control_predict.py drive.mp4 --csv drive.csv \
        --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml \
        --controllers C1,C2,C3,C4,C5

Requires dracer_core importable (pip install -e D-Racer-Kit/src/dracer_core).
"""
import argparse
import os

import numpy as np

from dracer_core.perception_core import LanePipeline, cfg_from_profile
from dracer_core.control_core import Controller, make_ctrl
from dracer_core.profile import load_profile, section

import _common as cm


def perception_from_profile(profile_path):
    """Build the perception Cfg from a profile's [perception] section."""
    if not profile_path:
        return cfg_from_profile()
    psec = section(load_profile(profile_path), 'perception')
    psec.pop('mode', None)          # legacy key, no longer used
    return cfg_from_profile(psec)


def run(video, csv_path, cfg, controllers, out_path):
    rows = cm.read_csv(csv_path)
    ftime = cm.col(rows, 'frame_time')
    steer = cm.col(rows, 'manual_steering', default=0.0)
    throttle = cm.col(rows, 'manual_throttle', default=0.0)

    pipe = LanePipeline(cfg)
    ctrls = {c: Controller(make_ctrl(c)) for c in controllers}

    fields = (['frame_time', 'center_error', 'ema', 'heading', 'confidence',
               'manual_steering', 'manual_throttle']
              + [f'{p}_{c}' for c in controllers for p in ('pred_steer', 'pred_thr', 'gated')])
    out_rows = []
    prev_t = None
    for i, frame in enumerate(cm.iter_frames(video)):
        _, st, _ = pipe.process(frame, debug=True)
        t = ftime[i] if i < len(ftime) and ftime[i] == ftime[i] else float(i)
        dt = (t - prev_t) if (prev_t is not None and t > prev_t) else 0.0
        prev_t = t
        m_thr = throttle[i] if i < len(throttle) else 0.0
        st_in = dict(st)
        st_in['speed'] = m_thr if m_thr == m_thr else None
        row = {'frame_time': t, 'center_error': st['center_error'], 'ema': st['ema'],
               'heading': st['heading'], 'confidence': st['confidence'],
               'manual_steering': steer[i] if i < len(steer) else '',
               'manual_throttle': m_thr}
        for c in controllers:
            u, thr, info = ctrls[c].step(st_in, dt)
            row[f'pred_steer_{c}'] = round(u, 5)
            row[f'pred_thr_{c}'] = round(thr, 5)
            row[f'gated_{c}'] = info.get('gated') or ''
        out_rows.append(row)

    cm.write_csv(out_path, fields, out_rows)
    print(f'{len(out_rows)} frames x {len(controllers)} controllers -> {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('video')
    ap.add_argument('--csv', required=True, help='paired manual drive log (recorder csv)')
    ap.add_argument('--profile', help='perception profile (stage-2 selected)')
    ap.add_argument('--controllers', default='C1,C2,C3,C4,C5')
    ap.add_argument('--output')
    args = ap.parse_args()

    controllers = [c.strip() for c in args.controllers.split(',') if c.strip()]
    cfg = perception_from_profile(args.profile)
    base = cm.clip_name(args.video)
    out = args.output or f'rslt/pred_{base}.csv'
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    run(args.video, args.csv, cfg, controllers, out)


if __name__ == '__main__':
    main()
