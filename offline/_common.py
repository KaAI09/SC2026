"""Shared offline IO helpers: video read, CSV read/write, and profile writing.
Kept thin — all detection/control LOGIC lives in dracer_core.

Used by the control tools (control_predict / control_select). Rendering/metrics
helpers were removed with the perception exploration tools; the perception debug
overlay now lives in the pipeline itself (perception_core.render_panels).
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


def write_csv(path, fieldnames, rows):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return path


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
              f'# offline -> online contract (control_select writes [control]).\n'
              f'# Keys map 1:1 to dracer_core Cfg (perception) / CtrlCfg (control).\n')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.safe_dump(prof, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    return path
