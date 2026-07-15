"""신규 calib·클립으로 BEV/calib 품질을 오프라인 평가하는 단일 도구.

서브커맨드:
  compare  두 camera.yaml 로 같은 클립을 재생 → center_error 스케일이 x_half 비만큼
           달라지는지(정규화는 calib 결합) + center_error_cm 은 불변인지(디커플링) 확인.
  quality  구간(seg.csv)별 BEV 품질 — 직선 구간은 CameraModel.validate 로 폭·수직·평행 오차.
  coast    coast→pair 전환 진지 오차 + side 뒤집힘률.

panel_replay.py 와 같은 재생 하네스(같은 dracer_core 파이프라인)를 공유한다.

    python offline/bev_eval.py compare <raw.mp4> \
        --camera-a D-Racer-Kit/src/config/camera_current.yaml \
        --camera-b D-Racer-Kit/src/config/camera_fresh.yaml \
        --profile D-Racer-Kit/src/config/profiles/track.yaml
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'D-Racer-Kit', 'src', 'dracer_core'))
import _common as cm                                             # noqa: E402
from dracer_core.calib import CameraModel                        # noqa: E402
from dracer_core.perception_core import (LanePipeline,           # noqa: E402
                                         cfg_from_profile)


# ----------------------------------------------------------------- 재생 하네스
def load_cfg(profile_path):
    prof = cm.read_profile(os.path.expanduser(profile_path)) if profile_path else {}
    return cfg_from_profile(prof.get('perception') or {})


def replay(mp4, cam, cfg, want_dbg=False):
    """panel_replay 와 동일: 실제 프레임 크기에 모델 정합, 클립 fps 를 dt 로.
    (i, state, dbg) 를 순차 yield. dbg 는 want_dbg=True 일 때만 채워진다."""
    cap, fps, n = cm.open_clip(mp4)
    ok, first = cap.read()
    if not ok:
        raise SystemExit(f'빈 영상: {mp4}')
    h, w = first.shape[:2]
    cam = cam.match((w, h))
    cap.release()
    pipe = LanePipeline(cfg, cam)
    dt_s = 1.0 / fps
    for i, frame in enumerate(cm.iter_frames(mp4)):
        out = pipe.process(frame, 0.0 if i == 0 else dt_s, debug=want_dbg)
        if want_dbg:
            st, dbg = out
        else:
            st, dbg = out, None
        yield i, st, dbg


# --------------------------------------------------------------- compare 순수함수
def scale_stats(ce_a, ce_b):
    """정규화 center_error 두 벌의 비(a/b) 중앙값. |b|<1e-3 과 NaN 은 제외(0 근처 불안정)."""
    a = np.asarray(ce_a, dtype=float)
    b = np.asarray(ce_b, dtype=float)
    good = np.isfinite(a) & np.isfinite(b) & (np.abs(b) > 1e-3)
    ratio = float(np.median(a[good] / b[good])) if good.any() else float('nan')
    return {'n': int(good.sum()), 'ratio_median': ratio}


def _cam_row(tag, c):
    return (f'  [{tag}] rms={c.rms_px:.3f}px ground_rms={c.ground_rms_px:.3f}px  '
            f'x_half={c.x_half_cm:.2f}cm px/cm={c.px_per_cm:.2f}  '
            f'y={c.y_near_cm:.1f}..{c.y_far_cm:.1f}cm')


def cmd_compare(a):
    cfg = load_cfg(a.profile)
    cam_a = CameraModel.load(os.path.expanduser(a.camera_a))
    cam_b = CameraModel.load(os.path.expanduser(a.camera_b))
    ce_a, cm_a = [], []
    for _, st, _ in replay(a.raw, cam_a, cfg):
        ce_a.append(st['center_error'] if st['center_error'] is not None else np.nan)
        cm_a.append(st.get('center_error_cm') if st.get('center_error_cm') is not None else np.nan)
    ce_b, cm_b = [], []
    for _, st, _ in replay(a.raw, cam_b, cfg):
        ce_b.append(st['center_error'] if st['center_error'] is not None else np.nan)
        cm_b.append(st.get('center_error_cm') if st.get('center_error_cm') is not None else np.nan)
    m = min(len(ce_a), len(ce_b))
    s = scale_stats(ce_a[:m], ce_b[:m])
    # center_error = 횡오차/x_half → 같은 물리 횡오차에서 ce_a/ce_b = x_half_b/x_half_a.
    r_expected = cam_b.x_half_cm / cam_a.x_half_cm
    da = np.asarray(cm_a[:m], float)
    db = np.asarray(cm_b[:m], float)
    goodcm = np.isfinite(da) & np.isfinite(db)
    cm_diff = float(np.median(np.abs(da[goodcm] - db[goodcm]))) if goodcm.any() else float('nan')
    print(cm.clip_name(a.raw))
    print(_cam_row('A', cam_a))
    print(_cam_row('B', cam_b))
    print(f'  정규화 center_error 비 a/b: 실측중앙 {s["ratio_median"]:.3f} vs 기대 '
          f'x_half_b/x_half_a={r_expected:.3f}  (n={s["n"]})')
    print(f'    → 일치하면 정규화 center_error 가 calib 스케일에 결합됨을 실측 확정.')
    print(f'  center_error_cm |A-B| 중앙값: {cm_diff:.3f}cm  (n={int(goodcm.sum())})')
    print(f'    → 0 근처면 cm 값은 calib 불변 = 디커플링이 유효함을 확정.')


# ----------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('compare', help='두 calib 비교 + 디커플링 확인')
    p.add_argument('raw')
    p.add_argument('--camera-a', dest='camera_a', required=True)
    p.add_argument('--camera-b', dest='camera_b', required=True)
    p.add_argument('--profile', default='')
    p.set_defaults(func=cmd_compare)

    a = ap.parse_args()
    a.func(a)


if __name__ == '__main__':
    main()
