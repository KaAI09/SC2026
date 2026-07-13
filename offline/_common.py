"""Shared offline IO helpers: video read, CSV read, profile read.
Kept thin — all detection/control LOGIC lives in dracer_core.

Used by panel_replay.py and calibrate.py. Rendering/metrics helpers were removed with
the perception exploration tools; the perception debug overlay now lives in the
pipeline itself (perception_core.render_panels).
"""
import csv
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


# ------------------------------------------------------------------ csv IO
def read_csv(path):
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


def col(rows, key, default=np.nan):
    """Extract one column as a float array; blank/missing -> default (NaN)."""
    out = []
    for r in rows:
        v = (r.get(key) or '').strip()
        out.append(default if v == '' else float(v))
    return np.array(out)


# ------------------------------------------------------------------ profile IO
def read_profile(path):
    if not os.path.exists(path):
        return {}
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


# NOTE: read-only on purpose. The profile is hand-edited -- PyYAML does not preserve
# comments, and this profile's comments (why kp is 0.45, why steer_max is 0.7) are worth
# more than any automated rewrite.
