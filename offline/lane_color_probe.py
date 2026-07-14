"""차선 색 임계값을 **측정**한다 — 대회장 조명에서 흰/노랑이 실제로 어디 있는가.

색 임계를 눈으로 맞추면 순환에 빠진다: 임계가 틀려서 테이프를 못 잡으면, 그 테이프의
HSV 분포도 볼 수 없다. 그래서 이 도구는 **느슨한 탐색 임계**(`--probe-*`)로 후보 픽셀을
먼저 건지고, 거기서 나온 분포를 근거로 **운영 임계를 제안**한다. 제안값은 분포의 분위수이지
누군가의 감이 아니다.

측정은 전부 **BEV 위에서** 한다. 원본 프레임에는 관중·천장·옆 트랙이 같이 찍히고, 그것들의
HSV 를 섞어 놓은 히스토그램은 노면에 대해 아무 말도 하지 않는다. BEV 는 캘리브레이션된
지면 크롭이므로, 거기 있는 픽셀은 정의상 노면이다. (그러니 `camera.yaml` 이 틀리면 이
도구의 답도 틀린다 — 캘리브를 먼저 확정하라.)

판정은 **차가 쓰는 바로 그 함수**로 한다 (`color_masks` · `morph_gate`). 판정 로직을
복제하지 않는다 — 복제한 판정은 원본과 어긋나는 순간 거짓말이 된다.

    # 지면 사진 1장 (트랙에 서기 전, 조명만 확인)
    .venv/bin/python offline/lane_color_probe.py offline/calib/ground_01.png \
        --camera D-Racer-Kit/src/config/camera.yaml \
        --profile D-Racer-Kit/src/config/profiles/track.yaml

    # 수동 주행 녹화 (권장 — 트랙 한 바퀴가 조명 변화를 다 담는다)
    .venv/bin/python offline/lane_color_probe.py offline/rslt/<세션>/raw/collect_*.mp4 \
        --camera D-Racer-Kit/src/config/camera.yaml \
        --profile D-Racer-Kit/src/config/profiles/track.yaml --stride 5

제안값은 **가설이다.** 프로파일에 넣기 전에 `panel_replay.py --set` 으로 같은 클립에
A/B 하라 — 이 도구는 "테이프가 HSV 어디 있나" 를 답하고, 그것이 "차선이 잡히나" 를
자동으로 뜻하지는 않는다.
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
from dracer_core.perception_core import (cfg_from_profile,      # noqa: E402
                                         cfg_to_px, color_masks, morph_gate)

IMG_EXT = ('.png', '.jpg', '.jpeg', '.bmp')


def _frames(path, stride):
    """이미지 1장이든 클립이든 프레임을 흘려준다."""
    if path.lower().endswith(IMG_EXT):
        img = cv2.imread(path)
        if img is None:
            raise SystemExit(f'cannot read {path}')
        yield img
        return
    for i, f in enumerate(cm.iter_frames(path)):
        if i % stride == 0:
            yield f


def _q(arr, *qs):
    return [float(np.percentile(arr, q)) for q in qs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip', help='수동 주행 mp4 (권장) 또는 지면 사진 1장')
    ap.add_argument('--camera', required=True, help='camera.yaml — BEV 지면 크롭')
    ap.add_argument('--profile', required=True, help='현재 프로파일 (운영 임계값)')
    ap.add_argument('--stride', type=int, default=5, help='클립 프레임 간격 (기본 5)')
    # 탐색 임계 — 운영 임계보다 넓다. 여기 걸리는 것이 "테이프 후보" 이고, 분포는 이 안에서 본다.
    ap.add_argument('--probe-white-s', type=int, default=90, help='탐색: 흰 후보 S 상한')
    ap.add_argument('--probe-white-v', type=int, default=140, help='탐색: 흰 후보 V 하한')
    ap.add_argument('--probe-chroma-s', type=int, default=50, help='탐색: 유채색 후보 S 하한')
    ap.add_argument('--probe-chroma-v', type=int, default=80, help='탐색: 유채색 후보 V 하한')
    a = ap.parse_args()

    cam = CameraModel.load(a.camera)
    cfg = cfg_from_profile(cm.read_profile(a.profile).get('perception', {}))

    hs, ws, vs = [], [], []          # 유채색(노랑/주황) 후보 분포
    wS, wV = [], []                  # 흰 후보 분포
    n = 0
    hist = np.zeros(180, dtype=np.int64)
    # 게이트 통계: 소수색 비율과, 게이트가 실제로 죽인 색
    fracs_w, fracs_y, kill_w, kill_y, gated = [], [], 0, 0, 0

    for frame in _frames(a.clip, a.stride):
        if n == 0:
            cam = cam.match((frame.shape[1], frame.shape[0]))
            cfg_px = cfg_to_px(cfg, cam)
        n += 1

        # --- 분포: BEV 컬러 워프 = 노면만. 배경은 정의상 여기 없다.
        bev = cam.to_bev(frame, nearest=False)
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
        ground = bev.any(axis=2)      # BEV 유효 영역 (워프 밖은 0)

        w_cand = ground & (S <= a.probe_white_s) & (V >= a.probe_white_v)
        if w_cand.any():
            wS.append(S[w_cand]); wV.append(V[w_cand])

        c_cand = ground & (S >= a.probe_chroma_s) & (V >= a.probe_chroma_v)
        if c_cand.any():
            hs.append(H[c_cand]); ws.append(S[c_cand]); vs.append(V[c_cand])
            hist += np.bincount(H[c_cand], minlength=180)

        # --- 게이트: 차가 쓰는 그 함수로. 운영 임계 그대로.
        white, yellow = color_masks(frame, cfg_px)
        bw, by = cam.to_bev(white), cam.to_bev(yellow)
        pre_w, pre_y = int(cv2.countNonZero(bw)), int(cv2.countNonZero(by))
        post_w, post_y = morph_gate(bw.copy(), by.copy(), cfg_px)
        post_w, post_y = int(cv2.countNonZero(post_w)), int(cv2.countNonZero(post_y))
        tot = pre_w + pre_y
        if tot:
            fracs_w.append(pre_w / tot); fracs_y.append(pre_y / tot)
        if tot >= cfg_px.gate_min_px:
            gated += 1
            if pre_w and not post_w:
                kill_w += 1
            if pre_y and not post_y:
                kill_y += 1

    if not n:
        raise SystemExit('프레임이 없다')

    print(f'\n프레임 {n}개  |  BEV {cam.describe() if hasattr(cam, "describe") else ""}')
    print(f'운영 임계 (현재 프로파일): white S<={cfg.white_s_max} V>={cfg.white_v_min}  |  '
          f'yellow H {cfg.yellow_h_lo}-{cfg.yellow_h_hi} S>={cfg.yellow_s_min} '
          f'V>={cfg.yellow_v_min}  |  color_gate {cfg.color_gate}')

    # ---------------------------------------------------------------- 흰색
    print('\n[흰 테이프]')
    if wV:
        S_, V_ = np.concatenate(wS), np.concatenate(wV)
        # 노랑과 같은 병: 탐색 임계(V>=140)에 **노면도 걸린다.** 파란 카펫과 회색 바닥은
        # 저채도라 S 게이트를 그냥 통과하고, 그 픽셀들이 V 분포의 하단 꼬리를 아래로 끌어
        # 내린다 -- 실측: 흰 차선이 거의 없는 클립에서 V p05 가 140 으로 나오고, 도구는
        # `white_v_min 130` 을 제안했다. 그 임계를 넣으면 노면이 통째로 차선이 된다.
        # 테이프는 노면보다 **확실히 밝다**. 그 조건을 따로 세고, 그것으로만 제안한다.
        TAPE_V = 200
        tape = V_ >= TAPE_V
        print(f'  후보 {V_.size}px (탐색 S<={a.probe_white_s} V>={a.probe_white_v})  '
              f'| 그중 테이프(V>={TAPE_V}) {int(tape.sum())}px')
        if tape.sum() < 200:
            print(f'  V p50={np.median(V_):.0f} — **테이프로 보기엔 너무 어둡다.** 이 클립의 '
                  f'BEV 안에 흰 차선이 거의 없다 (잡은 것은 노면/카펫이다). 제안하지 않는다.')
        else:
            St, Vt = S_[tape], V_[tape]
            s50, s90, s98 = _q(St, 50, 90, 98)
            v02, v05, v10, v50 = _q(Vt, 2, 5, 10, 50)
            print(f'  테이프 S: p50={s50:.0f}  p90={s90:.0f}  p98={s98:.0f}')
            print(f'  테이프 V: p02={v02:.0f}  p05={v05:.0f}  p10={v10:.0f}  p50={v50:.0f}')
            # 하한은 테이프 분포의 하단 꼬리에 마진을 준다 — 그늘진 부분이 거기 산다.
            # 단 TAPE_V 아래로는 내려가지 않는다 (그 아래는 노면이라고 방금 정의했다).
            sug_v = int(max(TAPE_V - 40, v05 - 15))
            sug_s = int(max(cfg.white_s_max, min(120, s98 + 10)))
            print(f'  제안: white_v_min {cfg.white_v_min} -> {sug_v}   '
                  f'white_s_max {cfg.white_s_max} -> {sug_s}')
            if cfg.white_v_min > v10:
                print(f'  ⚠ 현재 V 하한({cfg.white_v_min})이 테이프 분포의 p10({v10:.0f})보다 '
                      f'높다 — 그늘진 테이프가 잘려나가고 있다')
    else:
        print('  후보 없음 — 탐색 임계를 더 낮춰라 (--probe-white-v)')

    # ------------------------------------------------------------- 노랑/주황
    print('\n[노랑/주황 테이프]')
    if hs:
        H_, S_, V_ = np.concatenate(hs), np.concatenate(ws), np.concatenate(vs)
        print(f'  유채색 후보 {H_.size}px (탐색 S>={a.probe_chroma_s} V>={a.probe_chroma_v})')
        # 파란 매트/카펫(H~100-130)을 빼고 난색 대역만 본다. 노란 테이프는 여기 없을 수 없다.
        warm = (H_ <= 45)
        # 난색이라고 다 테이프가 아니다. 파란 카펫·노면은 **탁한** 난색 잡음을 대량으로 낸다
        # (실측: S 61 / V 87) 반면 진짜 테이프는 밝고 진하다 (S 130 / V 145). 그 둘을 섞어
        # 히스토그램을 그리면 잡음이 표본을 압도하고, 도구는 **노면 색에 맞춘 임계**를
        # 자신 있게 제안한다 — 노란 테이프가 프레임에 하나도 없는 클립에서도.
        # 그래서 테이프의 최소 조건(밝고 진하다)을 별도로 세고, 그것이 없으면 **제안하지
        # 않는다.** 답이 없을 때 답을 지어내지 않는 것이 이 도구의 유일한 안전장치다.
        TAPE_S, TAPE_V = 80, 120
        tape = warm & (S_ >= TAPE_S) & (V_ >= TAPE_V)
        if warm.sum() < 50:
            print('  ⚠ 난색(H<=45) 픽셀이 거의 없다 — 이 클립에 노란 테이프가 안 찍혔거나, '
                  '노출이 눌러버렸다')
        elif tape.sum() < 200:
            Hn, Sn, Vn = H_[warm], S_[warm], V_[warm]
            print(f'  난색 {warm.sum()}px 이 있지만 **테이프로 보기엔 너무 탁하다** '
                  f'(S p50={np.median(Sn):.0f}  V p50={np.median(Vn):.0f}; '
                  f'테이프 기준 S>={TAPE_S} V>={TAPE_V} 를 넘는 픽셀 {tape.sum()}개).')
            print('  → **이 클립에 노란 테이프가 찍히지 않았다.** 잡은 것은 노면/카펫의 저채도 '
                  '난색 잡음이다. 임계를 제안하지 않는다 — 여기서 나온 제안은 노면 색에 맞춘 '
                  '것이지 테이프에 맞춘 것이 아니다.')
            print('  → 노란 차선(중앙 점선·분기)이 **BEV 안에 들어오는** 구간을 찍어서 다시 돌려라.')
        else:
            Hw, Sw, Vw = H_[tape], S_[tape], V_[tape]
            print(f'  테이프 픽셀 {tape.sum()}px (S>={TAPE_S} V>={TAPE_V} — 노면 잡음 제외)')
            h02, h50, h98 = _q(Hw, 2, 50, 98)
            s05, s50 = _q(Sw, 5, 50)
            v05, v50 = _q(Vw, 5, 50)
            print(f'  난색 {Hw.size}px:  H p02={h02:.0f} p50={h50:.0f} p98={h98:.0f}  '
                  f'S p05={s05:.0f} p50={s50:.0f}  V p05={v05:.0f} p50={v50:.0f}')
            print('  H 히스토그램 (테이프 픽셀만):')
            warm_hist = np.bincount(Hw, minlength=46)[:46]
            if warm_hist.max():
                for b in range(0, 46):
                    if warm_hist[b] * 60 // max(1, warm_hist.max()) > 0:
                        mark = ' <- 현재 하한' if b == cfg.yellow_h_lo else ''
                        print(f'    H={b:3d} {warm_hist[b]:7d} '
                              f'{"#" * int(60 * warm_hist[b] / warm_hist.max())}{mark}')
            lo = int(max(0, h02 - 4))
            hi = int(min(45, h98 + 4))
            # S/V 하한은 **테이프 필터를 걸기 전** 분포에서 낸다. 필터가 S>=TAPE_S 를 강제해
            # 놓고 그 결과에서 s_min 을 뽑으면 자기가 넣은 값을 자기가 다시 읽는 순환이 된다
            # (실측: 필터 S>=80 -> 제안 s_min 80). H 대역은 위에서 확정됐으니, 이제 그 대역
            # 안의 **모든** 후보를 보면 된다 — 색이 맞는 픽셀 중 가장 어둡고 옅은 것까지가
            # 임계가 품어야 할 범위다.
            band = warm & (H_ >= lo) & (H_ <= hi)
            sb05, vb05 = _q(S_[band], 5)[0], _q(V_[band], 5)[0]
            print(f'  제안: yellow_h_lo {cfg.yellow_h_lo} -> {lo}   '
                  f'yellow_h_hi {cfg.yellow_h_hi} -> {hi}   '
                  f'yellow_s_min {cfg.yellow_s_min} -> {int(max(30, sb05 - 10))}   '
                  f'yellow_v_min {cfg.yellow_v_min} -> {int(max(60, vb05 - 10))}')
            if cfg.yellow_h_lo > h02:
                cut = (Hw < cfg.yellow_h_lo).sum() / Hw.size
                print(f'  ⚠ 현재 H 하한({cfg.yellow_h_lo})이 난색 분포의 하단을 자른다 '
                      f'— 난색 픽셀의 {cut*100:.0f}% 가 버려진다. '
                      f'주황에 가까운 테이프는 H 가 노랑보다 낮다.')
    else:
        print('  후보 없음')

    # ------------------------------------------------------------ color_gate
    print(f'\n[color_gate = {cfg.color_gate}]  (소수색 비율이 이 미만이면 그 색을 통째로 버린다)')
    if fracs_y:
        fy, fw = np.array(fracs_y), np.array(fracs_w)
        print(f'  게이트가 판정한 프레임: {gated}/{n}  (gate_min_px={cfg_px.gate_min_px} 이상)')
        print(f'  노랑 비율 y_frac: p50={np.median(fy):.3f}  p90={np.percentile(fy, 90):.3f}')
        print(f'  흰   비율 w_frac: p50={np.median(fw):.3f}  p10={np.percentile(fw, 10):.3f}')
        print(f'  게이트가 죽인 프레임 — 노랑 {kill_y} ({kill_y/max(1,n)*100:.1f}%)  '
              f'흰 {kill_w} ({kill_w/max(1,n)*100:.1f}%)')
        if np.median(fy) < cfg.color_gate:
            print(f'  ⚠ 노랑 비율의 중앙값({np.median(fy):.3f})이 게이트({cfg.color_gate})보다 '
                  f'낮다 — 노란 테이프가 상시 버려진다. 점선 중앙선처럼 면적이 작은 차선은 '
                  f'게이트를 구조적으로 통과할 수 없다. color_gate 를 낮추거나 0 으로 꺼라.')
    else:
        print('  판정할 프레임 없음')

    print('\n제안은 가설이다. 프로파일에 넣기 전에 같은 클립으로 A/B 하라:')
    print(f'  .venv/bin/python offline/panel_replay.py {a.clip} --camera {a.camera} '
          f'--profile {a.profile} --set yellow_h_lo=<제안> --no-video')


if __name__ == '__main__':
    main()
