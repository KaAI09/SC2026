"""주행 raw 영상 -> 디버그 4패널 영상 재구성 (+ 실차 csv 와 대조).

`drive.launch` 는 패널을 녹화하지 않는다. 패널 합성 + JPEG 인코딩이 검출의 4배를 먹어서
그게 프레임 드랍의 진범이었기 때문이다(offline/README.md). 대신 raw 카메라 영상과
LaneState csv 를 남기고, 그 둘로 패널을 **여기서, 사후에** 정확히 되살린다. 차는 주행 중
렌더링 비용을 한 푼도 내지 않는다.

같은 `dracer_core` 파이프라인을 그대로 돌리므로 재구성 결과는 보드가 봤을 화면과 같다.
`--csv` 를 주면 실차가 그때 실제로 발행한 LaneState 와 프레임 단위로 대조해서, 재현이
맞는지(= 오프라인 튜닝을 믿어도 되는지) 검증한다.

    # 성공 주행 한 세션 되살리기
    python offline/panel_replay.py offline/rslt/recorder/raw/drive_20260711_145515.mp4 \
        --camera D-Racer-Kit/src/config/camera.yaml \
        --profile D-Racer-Kit/src/config/profiles/track2025.yaml \
        --csv offline/rslt/recorder/csv/drive_20260711_145515.csv

    # 파라미터 바꿔서 A/B (원본은 건드리지 않는다)
    python offline/panel_replay.py <raw.mp4> --camera ... --profile ... \
        --set sw_max_miss=4 --set lane_width_cm=35 --no-video
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'D-Racer-Kit', 'src', 'dracer_core'))

import _common as cm                                            # noqa: E402
from dracer_core.calib import CameraModel                       # noqa: E402
from dracer_core.perception_core import (LanePipeline, cfg_from_profile,  # noqa: E402
                                         render_panels)


def _coerce(cfg, key, raw):
    """'sw_max_miss=4' 의 문자열 값을 Cfg 필드의 실제 타입으로."""
    cur = getattr(cfg, key)
    if isinstance(cur, bool):
        return raw.lower() in ('1', 'true', 'yes')
    if isinstance(cur, int):
        return int(raw)
    if isinstance(cur, float):
        return float(raw)
    if isinstance(cur, (tuple, list)):
        return tuple(s.strip() for s in raw.split(','))
    return raw


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('raw', help='recorder 가 남긴 raw/<session>.mp4')
    ap.add_argument('--camera', required=True, help='camera.yaml -> metric BEV')
    ap.add_argument('--profile', default='', help='track profile yaml ([perception] 적용)')
    ap.add_argument('--csv', default='', help='같은 세션의 csv -> 실차 LaneState 와 대조')
    ap.add_argument('--out', default='', help='패널 mp4 출력 (기본: <raw>_panel.mp4)')
    ap.add_argument('--scale', type=float, default=2.0, help='패널 확대 (보기용)')
    ap.add_argument('--set', action='append', default=[], metavar='K=V',
                    help='Cfg 필드 덮어쓰기 (반복 가능) — 오프라인 A/B 용')
    ap.add_argument('--no-video', action='store_true', help='통계만, mp4 안 씀')
    a = ap.parse_args()

    cam = CameraModel.load(os.path.expanduser(a.camera))
    prof = cm.read_profile(os.path.expanduser(a.profile)) if a.profile else {}
    cfg = cfg_from_profile(prof.get('perception') or {})
    for kv in a.set:
        k, _, v = kv.partition('=')
        if not hasattr(cfg, k):
            raise SystemExit(f'알 수 없는 Cfg 필드: {k}')
        cfg = type(cfg)(**{**cfg.__dict__, k: _coerce(cfg, k, v)})
        print(f'  override  {k} = {getattr(cfg, k)!r}')

    cap, fps, n = cm.open_clip(a.raw)
    ok, first = cap.read()
    if not ok:
        raise SystemExit(f'빈 영상: {a.raw}')
    h, w = first.shape[:2]
    # 보드가 그랬듯 실제 프레임 크기에 모델을 정합시킨다 (rescale 은 정확하다).
    cam = cam.match((w, h))
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    print(f'{cm.clip_name(a.raw)}: {w}x{h} {n}프레임 @{fps:.1f}fps | {cam.summary()}')

    pipe = LanePipeline(cfg, cam)
    writer, states = None, []
    out_path = a.out or os.path.join(os.path.dirname(a.raw) or '.',
                                     cm.clip_name(a.raw) + '_panel.mp4')
    # 인지 문턱값·EMA 가 전부 시간 단위라 파이프라인은 dt 를 요구한다. 재생에서는 클립의
    # fps 가 그 dt 다 — 보드가 그 프레임들을 실제로 받은 간격이 그것이기 때문이다.
    dt_s = 1.0 / fps
    for i, frame in enumerate(cm.iter_frames(a.raw)):
        st, dbg = pipe.process(frame, 0.0 if i == 0 else dt_s, debug=True)
        states.append(st)
        if a.no_video:
            continue
        panel = render_panels(frame, dbg, cfg)
        if a.scale != 1.0:
            panel = cv2.resize(panel, None, fx=a.scale, fy=a.scale,
                               interpolation=cv2.INTER_NEAREST)
        if writer is None:
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     fps, (panel.shape[1], panel.shape[0]))
        writer.write(panel)
    if writer is not None:
        writer.release()
        print(f'패널 -> {out_path}')

    # ---- 재구성 통계 -------------------------------------------------------
    val = np.array([s['center_error'] is not None for s in states])
    both = np.array([s['left_conf'] == 1.0 and s['right_conf'] == 1.0 for s in states])
    coast = np.array([s['used_fallback'] for s in states])
    stt = [s['state'] for s in states]
    print(f'재구성  n={len(states)}  valid={100*val.mean():.0f}%  쌍검출={100*both.mean():.0f}%  '
          f'coast={100*coast.mean():.0f}%  ' +
          '  '.join(f'{k}={100*np.mean([s == k for s in stt]):.0f}%'
                    for k in ('OK', 'HOLD', 'OUTLIER', 'LOST')))

    # ---- 실차 csv 와 대조 (재현이 맞는지) ----------------------------------
    if a.csv:
        rows = cm.read_csv(os.path.expanduser(a.csv))
        m = min(len(rows), len(states))
        if m == 0:
            return
        live = cm.col(rows[:m], 'center_error')
        mine = np.array([s['center_error'] if s['center_error'] is not None else np.nan
                         for s in states[:m]])
        good = ~np.isnan(live) & ~np.isnan(mine)
        lb = cm.col(rows[:m], 'left_conf') == 1.0
        rb = cm.col(rows[:m], 'right_conf') == 1.0
        print(f'실차    n={len(rows)}  valid={100*np.mean(cm.col(rows, "valid")):.0f}%  '
              f'쌍검출={100*np.mean(lb & rb):.0f}%  '
              f'coast={100*np.mean(cm.col(rows, "used_fallback")):.0f}%')
        if good.any():
            d = np.abs(live[good] - mine[good])
            # 중앙값으로 판정한다. raw mp4 는 보드가 디코드한 JPEG 를 **다시 손실 인코딩**한
            # 것이라 프레임이 비트 단위로 같지 않다. HSV 임계 근처 픽셀이 몇 개 뒤집히고,
            # 그것이 코리도어 선택처럼 양자택일인 지점에서 가끔 결과를 통째로 갈라놓는다.
            # 그 소수 프레임이 평균을 지배하므로 평균은 재현성 지표로 못 쓴다.
            print(f'대조    겹친 {m}프레임 중 {good.sum()}개 비교: |Δcenter_error| '
                  f'중앙값 {np.median(d):.4f}  p90 {np.percentile(d, 90):.4f}  최대 {d.max():.4f}')
            print(f'        |Δ|>0.3 (코리도어 선택이 갈린 프레임): {100 * (d > 0.3).mean():.1f}%')
            print('        ' + ('✅ 재현 일치 — 오프라인 튜닝 결과를 믿어도 된다'
                                if np.median(d) < 0.02 else
                                '⚠️ 재현 불일치 — 보드와 다른 코드/설정으로 돌고 있다'))
        if len(rows) != len(states):
            print(f'        ※ 프레임 수가 다르다 (실차 {len(rows)} vs raw {len(states)}): '
                  'csv 는 발행된 LaneState, raw 는 카메라 프레임 — 인지가 프레임을 '
                  '흘렸다면 어긋난다')


if __name__ == '__main__':
    main()
