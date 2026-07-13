"""color_gate 가 진짜 차선을 지우고 있나 — 세는 도구 (README 남은 작업 B4).

B4 는 "노란선을 지울 수 있다 / 치명적일 수 있다" 라는 **가설**이다. 고치기 전에 센다.

게이트는 morph CLOSE **이후**의 픽셀 수로 판정하므로, 같은 `morph_gate` 를 `color_gate=0`
으로 한 번 더 돌려 "게이트가 세는 바로 그 카운트" 를 얻는다. 게이트 로직을 복제하지 않고,
실제 반환 마스크가 전부 0 인지로 kill 을 관찰한다 — 복제한 판정은 원본과 어긋나는 순간
거짓말이 된다.

두 패스를 프레임 단위로 대조한다:

    A(baseline)   color_gate = profile 값
    B(gate off)   color_gate = 0.0

B 에서는 코리도어가 2개인데 A 에서는 1개 이하인 프레임 = **게이트가 분기를 숨긴 프레임**.
그것이 0 이면 게이트는 이 데이터에서 무해하다.

0712 성공 주행 결과: kill 14프레임(1.1%), 숨긴 분기 **0**, 게이트 on/off 시 `n_corridors`
**완전 동일**. 정작 죽는 건 흰색(57.7%) 이었다 — 이 트랙은 두 색이 거의 배타적이라
게이트가 "다수색 구간의 소수색 잔재 지우기" 로 작동한다. **노란 분기 진입 구간을 담은
데이터로 다시 돌려야 판정이 된다.**

    .venv/bin/python offline/color_gate_probe.py offline/rslt/<세션>/raw/*.mp4 \
        --camera D-Racer-Kit/src/config/camera.yaml \
        --profile D-Racer-Kit/src/config/profiles/track2025.yaml
"""
import argparse
import csv
import os
import sys
from dataclasses import replace

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'D-Racer-Kit', 'src', 'dracer_core'))

import _common as cm                                            # noqa: E402
import dracer_core.perception_core as pc                        # noqa: E402
from dracer_core.calib import CameraModel                       # noqa: E402
from dracer_core.perception_core import (LanePipeline,          # noqa: E402
                                         cfg_from_profile)

_orig_morph_gate = pc.morph_gate
_rec = []


def _spy(white, yellow, c):
    """morph_gate 를 감싼다. 파이프라인 동작은 원본 그대로 유지한다."""
    # 게이트가 세는 대상 = morph CLOSE 이후 / 게이트 적용 이전의 마스크.
    # color_gate=0 이면 `wc/tot < 0` 이 결코 참이 아니므로 게이트만 무력화된다.
    wm, ym = _orig_morph_gate(white.copy(), yellow.copy(), replace(c, color_gate=0.0))
    wc, yc = int(cv2.countNonZero(wm)), int(cv2.countNonZero(ym))

    out_w, out_y = _orig_morph_gate(white, yellow, c)           # 실제 결과 (원본 로직)

    _rec.append({
        'wc': wc, 'yc': yc, 'tot': wc + yc, 'gate_min_px': int(c.gate_min_px),
        'w_frac': (wc / (wc + yc)) if (wc + yc) else 0.0,
        'y_frac': (yc / (wc + yc)) if (wc + yc) else 0.0,
        # "게이트가 죽였다" = morph 후엔 픽셀이 있었는데 게이트 통과 후 전부 0.
        'w_killed': int(wc > 0 and cv2.countNonZero(out_w) == 0),
        'y_killed': int(yc > 0 and cv2.countNonZero(out_y) == 0),
    })
    return out_w, out_y


pc.morph_gate = _spy


def run_pass(clips, cfg, cam0, label):
    rows = []
    for path in clips:
        cap, fps, n = cm.open_clip(path)
        ok, first = cap.read()
        cap.release()
        if not ok:
            print(f'  ! 빈 영상: {path}')
            continue
        h, w = first.shape[:2]
        cam = cam0.match((w, h))              # 보드가 그랬듯 실제 프레임 크기에 정합
        pipe = LanePipeline(cfg, cam)         # 세션마다 새 파이프라인 (보드와 동일)
        name = cm.clip_name(path)
        dt_s = 1.0 / fps                      # 인지 문턱값은 시간 단위다 (README B2)

        _rec.clear()
        for i, frame in enumerate(cm.iter_frames(path)):
            before = len(_rec)
            st = pipe.process(frame, 0.0 if i == 0 else dt_s)
            if len(_rec) != before + 1:       # spy 가 안 걸렸다 = 이 측정은 거짓말이다
                raise SystemExit('morph_gate spy 미발동 — 파이프라인 경로가 바뀌었다')
            rows.append({
                'pass': label, 'clip': name, 'i': i, **_rec[-1],
                'n_corridors': st['n_corridors'], 'ego_rule': st['ego_rule'],
                'state': st['state'], 'used_fallback': int(st['used_fallback']),
                'center_error': (st['center_error']
                                 if st['center_error'] is not None else ''),
            })
        print(f'  {label:9s} {name}: {len(rows)}프레임 누적')
    return rows


def pct(x, n):
    return f'{100.0 * x / n:.1f}%' if n else '—'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clips', nargs='+', help='raw/<세션>.mp4 (여러 개 가능)')
    ap.add_argument('--camera', required=True, help='camera.yaml -> metric BEV')
    ap.add_argument('--profile', required=True, help='track profile yaml')
    ap.add_argument('--out', default='', help='프레임별 CSV (기본: <raw 폴더>/color_gate_probe.csv)')
    a = ap.parse_args()

    cam0 = CameraModel.load(os.path.expanduser(a.camera))
    prof = cm.read_profile(os.path.expanduser(a.profile))
    cfg_a = cfg_from_profile(prof.get('perception') or {})
    cfg_b = replace(cfg_a, color_gate=0.0)
    print(f'A(baseline) color_gate={cfg_a.color_gate}   '
          f'B(gate off) color_gate={cfg_b.color_gate}\n')

    rows_a = run_pass(a.clips, cfg_a, cam0, 'A-base')
    rows_b = run_pass(a.clips, cfg_b, cam0, 'B-gateoff')
    if not rows_a:
        raise SystemExit('프레임이 하나도 없다')

    out = a.out or os.path.join(os.path.dirname(a.clips[0]) or '.', 'color_gate_probe.csv')
    with open(out, 'w', newline='', encoding='utf-8') as f:
        wri = csv.DictWriter(f, fieldnames=list(rows_a[0].keys()))
        wri.writeheader()
        wri.writerows(rows_a + rows_b)
    print(f'\n프레임별 기록 -> {out}')

    # ---------------------------------------------------------------- 요약
    n = len(rows_a)
    yc = np.array([r['yc'] for r in rows_a])
    wc = np.array([r['wc'] for r in rows_a])
    yf = np.array([r['y_frac'] for r in rows_a])
    tot = np.array([r['tot'] for r in rows_a])
    gmin = np.array([r['gate_min_px'] for r in rows_a])
    ykill = np.array([r['y_killed'] for r in rows_a], bool)
    wkill = np.array([r['w_killed'] for r in rows_a], bool)
    nc_a = np.array([r['n_corridors'] for r in rows_a])
    nc_b = np.array([r['n_corridors'] for r in rows_b])

    print(f'\n=== 1. 게이트 발동 (A, n={n}) ===')
    print(f'  gate_min_px          : {int(gmin[0])} px (총합이 이 미만이면 게이트 미발동)')
    print(f'  판정 대상 프레임      : {int((tot >= gmin).sum())} ({pct(int((tot >= gmin).sum()), n)})')
    print(f'  노란 픽셀 존재 (yc>0) : {int((yc > 0).sum())} ({pct(int((yc > 0).sum()), n)})')
    print(f'  ★ 노란색이 죽은 프레임 : {int(ykill.sum())} ({pct(int(ykill.sum()), n)})')
    print(f'    흰색이 죽은 프레임   : {int(wkill.sum())} ({pct(int(wkill.sum()), n)})')

    seen = yc > 0
    if seen.any():
        q = np.percentile(yf[seen], [5, 25, 50, 75, 95])
        print(f'\n=== 2. 노란색이 보이는 프레임의 y_frac (n={int(seen.sum())}) ===')
        print(f'  p5 {q[0]:.3f} | p25 {q[1]:.3f} | med {q[2]:.3f} | p75 {q[3]:.3f} | '
              f'p95 {q[4]:.3f}   (문턱 {cfg_a.color_gate})')
        print('  med 가 1.0 에 가까우면 두 색이 배타적이다 = 게이트는 잔재 청소부로 일한다')
    if ykill.any():
        print(f'\n=== 2b. 죽은 노란색의 크기 ===')
        print(f'  yc     : min {int(yc[ykill].min())} | med {int(np.median(yc[ykill]))} | '
              f'max {int(yc[ykill].max())} px  (노이즈면 작고, 진짜 선이면 크다)')
        print(f'  y_frac : max {yf[ykill].max():.3f}')

    br = nc_a >= 2
    if br.any():
        print(f'\n=== 2c. 분기 프레임에서 (n={int(br.sum())}) ===')
        print(f'  흰색이 죽음 {int(wkill[br].sum())} | 노란색이 죽음 {int(ykill[br].sum())}')

    print(f'\n=== 3. 분기 검출 A vs B ===')
    print(f'  A(baseline) n_corridors>=2 : {int((nc_a >= 2).sum())} '
          f'({pct(int((nc_a >= 2).sum()), n)})')
    print(f'  B(gate off) n_corridors>=2 : {int((nc_b >= 2).sum())} '
          f'({pct(int((nc_b >= 2).sum()), n)})')
    hidden = (nc_a < 2) & (nc_b >= 2)
    print(f'  ★ 게이트가 분기를 숨긴 프레임 : {int(hidden.sum())} '
          f'({pct(int(hidden.sum()), n)})   ← B4 가 실재하는가')

    if hidden.any():
        print('\n  숨겨진 프레임:')
        for r_a, r_b, h_ in zip(rows_a, rows_b, hidden):
            if h_:
                print(f'    {r_a["clip"]}#{r_a["i"]:4d}  wc={r_a["wc"]:5d} yc={r_a["yc"]:5d} '
                      f'y_frac={r_a["y_frac"]:.3f} y_killed={r_a["y_killed"]} '
                      f'w_killed={r_a["w_killed"]}  n_corridors {r_a["n_corridors"]}'
                      f'→{r_b["n_corridors"]}')

    print('\n=== 4. 판정 ===')
    if not (ykill.any() or wkill.any()):
        print('  게이트가 아무 색도 죽이지 않았다 → 이 데이터에서 B4 는 재현되지 않는다.')
    elif not hidden.any():
        print('  색이 죽은 적은 있으나, 그 때문에 분기를 놓친 프레임은 없다.')
        print('  → 게이트는 이 데이터에서 무해하다. 노란 분기 진입 구간으로 다시 확인하라.')
    else:
        print(f'  게이트가 {int(hidden.sum())}프레임에서 분기를 숨겼다 → B4 는 실재한다.')
        print('  → 절대 면적 예외(color_keep_cm2) 를 검토하라: 비율이 낮아도 면적이 크면 살린다.')


if __name__ == '__main__':
    main()
