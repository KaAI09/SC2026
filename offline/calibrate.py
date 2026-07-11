"""카메라 캘리브레이션 도구 — 체커보드 사진 → `camera.yaml` (K·D·H·BEV 격자).

로컬(macOS, ROS 불필요)에서 돈다. 수학은 전부 `dracer_core.calib`에 있고 이 파일은
CLI/리포트만 담당한다 (온라인 노드와 같은 코드 = 오프라인/온라인 불일치 없음).

**촬영은 런타임(320x160)이 아니라 고해상도(640x480)로 한다.** 320x160에서는 지면에 눕힌
보드의 코너가 사실상 안 잡힌다 — 원근이 세로를 뭉개고 2:1 비등방 스케일이 한 번 더 뭉개서
칸이 7~11px까지 내려간다(합성 스윕: 24개 틸트/거리 조합 중 2개만 검출). 그래서 코너가
서브픽셀로 잡히는 해상도에서 캘리브레이션하고 `--runtime-size`로 **정확히** 옮긴다.

  이 변환은 근사가 아니다. 두 해상도가 같은 센서 프레임의 순수 리스케일(크롭/레터박스
  없음)이므로 640x480 -> 320x160 은 네이티브 크기와 무관하게 정확히 (sx,sy)=(0.5, 1/3):
      K 는 축별 선형 스케일,  D 는 불변(정규화 좌표에 작용),  H 는 H @ S^-1 로 합성.
  합성 검증에서 rescale 오차 0.00e+00 px 확인. (dracer_core.calib.CameraModel.rescale)

촬영: D3-G에서 `scripts/capture_camera_calib.py` (해상도 변경 절차는 그 파일 참조).

  # 1) K·D — 보드를 카메라 정면으로 들고 여러 각도/거리 10~20장.
  #    렌즈 고유 -> 트랙이 바뀌어도, 카메라를 재조준해도 영구 재사용.
  ../.venv/bin/python calibrate.py --intrinsics shots/intr --square-mm 24.8

  # 2) H — 지면에 눕힌 보드 1장. 장착 자세 -> 카메라를 건드리면 이것만 다시 찍는다.
  #    거리를 잴 필요가 없다: 보드가 지면 평면을 정의하므로 solvePnP 가 카메라 위치를
  #    풀어주고, 카메라~보드 거리는 산출된다(손으로는 광학중심의 지면 투영점도, 첫
  #    '내부 코너'도 정확히 짚기 어렵다 — 실측 두 해석 모두 Cy≠0 으로 틀렸었다).
  #    보드는 화면에 다 들어오게, 좌우 중앙에, 노면에 평평하게만 두면 된다.
  ../.venv/bin/python calibrate.py --intrinsics shots/intr --ground shots/ground.png \
      --square-mm 25.0 --lane-width-cm 35 --cam-height-cm 23 \
      --out ../D-Racer-Kit/src/config/camera.yaml

  # 3) 검증 — 직선 구간 프레임에서 BEV가 실제로 metric 한지 (카메라를 만진 뒤엔 항상)
  ../.venv/bin/python calibrate.py --check ../D-Racer-Kit/src/config/camera.yaml \
      --straight shots/straight.png --lane-width-cm 60

`--square-mm`는 **인쇄물을 자로 실측한 값**을 넣어야 한다. 프린터가 자동 축소하면
공칭 25mm가 아니게 되고, 그 오차가 BEV의 cm 전체를 그대로 틀어지게 한다.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'D-Racer-Kit', 'src', 'dracer_core'))
from dracer_core.calib import (  # noqa: E402
    CameraModel, build_model, calibrate_intrinsics, find_corners, ground_extent,
    ground_pose,
)


def _pattern(s):
    c, r = s.lower().split('x')
    return int(c), int(r)


def _load(paths):
    imgs, names = [], []
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  ! 읽기 실패: {p}")
            continue
        imgs.append(img)
        names.append(os.path.basename(p))
    return imgs, names


def do_intrinsics(a):
    files = sorted(glob.glob(os.path.join(a.intrinsics, '*.png')) +
                   glob.glob(os.path.join(a.intrinsics, '*.jpg')))
    imgs, names = _load(files)
    if not imgs:
        sys.exit(f"체커보드 사진이 없다: {a.intrinsics}")

    # 어느 장이 쓰였는지 = 촬영 품질 피드백 (검출 실패가 많으면 패턴이 너무 촘촘한 것)
    used = []
    for img, nm in zip(imgs, names):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        used.append((nm, find_corners(gray, a.pattern) is not None))
    ok_n = sum(1 for _, o in used if o)
    print(f"\n[K·D] {len(imgs)}장 중 코너 검출 {ok_n}장 (패턴 {a.pattern[0]}x{a.pattern[1]} 내부코너)")
    for nm, o in used:
        if not o:
            print(f"  - 검출 실패: {nm}")
    if ok_n < 5:
        sys.exit("\n검출 성공이 5장 미만이다. 보드를 화면에 더 크게(폭의 70~80%) 담거나,\n"
                 "칸이 더 큰 패턴으로 재출력할 것 (320x160은 세로 여유가 적다).")

    K, D, rms, n, size = calibrate_intrinsics(imgs, a.pattern, a.square_mm)
    print(f"  해상도   : {size[0]}x{size[1]}")
    print(f"  재투영 RMS: {rms:.3f} px   ({'양호' if rms < 1.0 else '⚠ 1px 초과 — 보드 평탄도/초점 확인'})")
    print(f"  fx,fy    : {K[0,0]:.1f}, {K[1,1]:.1f}   (2:1 비등방 스케일 → fx≠fy 가 정상)")
    print(f"  cx,cy    : {K[0,2]:.1f}, {K[1,2]:.1f}")
    print(f"  D        : {np.array2string(D, precision=4)}")
    return K, D, rms, size


def do_ground(a, K, D, size):
    img = cv2.imread(a.ground)
    if img is None:
        sys.exit(f"지면 사진을 못 읽었다: {a.ground}")
    if (img.shape[1], img.shape[0]) != tuple(size):
        sys.exit(f"지면 사진 해상도 {img.shape[1]}x{img.shape[0]} != K·D 해상도 "
                 f"{size[0]}x{size[1]} — 같은 파이프라인으로 찍어야 한다")

    # 보드가 지면 평면을 정의하므로 solvePnP 가 카메라 위치를 준다 -> near_cm 실측 불필요.
    # (손으로 재면 광학중심의 지면 투영점도, 첫 '내부 코너'도 정확히 짚기 어렵다)
    Hg, cam_h, near_cm, lat_cm, grms = ground_pose(img, K, D, a.pattern, a.square_mm)
    print(f"\n[H] 지면 보드 → 호모그래피  (near_cm 은 실측이 아니라 산출)")
    print(f"  카메라 높이 : {cam_h:.1f} cm" +
          (f"   (실측 {a.cam_height_cm} cm, 오차 {abs(cam_h - a.cam_height_cm):.1f} cm"
           f" — {'정합' if abs(cam_h - a.cam_height_cm) < 3 else '⚠ 불일치: 보드 평탄도/칸 크기 확인'})"
           if a.cam_height_cm else "   (--cam-height-cm 을 주면 교차검증한다)"))
    print(f"  near        : {near_cm:.1f} cm  (카메라 수직 투영점 → 가장 가까운 코너열)")
    print(f"  보드 횡위치 : {lat_cm:+.1f} cm  (중앙에 놓았다면 0 근처)")
    print(f"  재투영 RMS  : {grms:.3f} cm  "
          f"({'양호' if grms < 1.0 else '⚠ 1cm 초과 — 보드 평탄도/칸 크기 확인'})")

    x_half, y_near, y_far = ground_extent(Hg, K, D, size)
    if a.x_half:
        x_half = a.x_half
    if a.y_far:
        y_far = a.y_far
    print(f"  가시 지면 : x=±{x_half:.0f}cm, y={y_near:.0f}~{y_far:.0f}cm  (이미지 경계를 지면에 투영)")

    m = build_model(K, D, Hg, size, a.px_per_cm, x_half, y_near, y_far,
                    lateral_offset_cm=a.axis_offset_cm, rms=a.rms, ground_rms=grms,
                    square_mm=a.square_mm)
    print(f"  촬영 : {m.summary()}")

    # 런타임 해상도로 정확히 이전 (K 선형 스케일 / D 불변 / H 합성). 근사가 아니다.
    if tuple(a.runtime_size) != tuple(size):
        m = m.rescale(a.runtime_size)
        print(f"  런타임: {m.summary()}")
        print(f"         ({size[0]}x{size[1]} → {a.runtime_size[0]}x{a.runtime_size[1]} "
              f"순수 리스케일: fx·cx ×{a.runtime_size[0]/size[0]:.4f}, "
              f"fy·cy ×{a.runtime_size[1]/size[1]:.4f}, D 불변)")

    if a.lane_width_cm:
        lw = m.lane_width_px(a.lane_width_cm)
        print(f"  차선폭 {a.lane_width_cm}cm → BEV {lw:.0f}px "
              f"(BEV 폭 {m.bev_size[0]}px의 {100*lw/m.bev_size[0]:.0f}%)")
        if lw > m.bev_size[0] * 0.8:
            print("  ⚠ 차선폭이 BEV 폭의 80%를 넘는다 — --x-half 를 키우거나 px-per-cm 을 낮출 것")
    return m


def do_check(a):
    m = CameraModel.load(a.check)
    print(f"\n[검증] {m.summary()}")
    img = cv2.imread(a.straight)
    if img is None:
        sys.exit(f"직선 구간 프레임을 못 읽었다: {a.straight}")

    # 인지 코어와 '같은' 검출로 확인해야 의미가 있다
    from dracer_core.perception_core import Cfg, detect, sliding_window_lanes
    c = Cfg()
    white, yellow = detect(img, c)
    bev_w, bev_y = m.to_bev(white), m.to_bev(yellow)
    lanes = (sliding_window_lanes(bev_w, 'W', c) + sliding_window_lanes(bev_y, 'Y', c))
    r = m.validate(lanes, a.lane_width_cm)

    print(f"  BEV 차선 {len(lanes)}개 검출")
    if 'width_px' in r:
        print(f"  차선폭     : {r['width_px']:.0f}px  (기대 {r['width_expected_px']:.0f}px, "
              f"오차 {r['width_err_cm']:+.1f}cm)")
        print(f"  평행성     : 간격 변동 {r['parallel_spread_cm']:.1f}cm")
        print(f"  수직성     : 좌우 흐름 {r['vertical_skew_cm']:.1f}cm")
    print(f"  → {'OK — 캘리브레이션 유효' if r['ok'] else '✗ FAIL: ' + r['reason']}")

    out = os.path.join('rslt', 'calib_check.png')
    os.makedirs('rslt', exist_ok=True)
    vis = np.zeros((*bev_w.shape, 3), np.uint8)
    vis[bev_w > 0] = (200, 200, 200)
    vis[bev_y > 0] = (0, 220, 255)
    cv2.line(vis, (int(m.axis_u), 0), (int(m.axis_u), vis.shape[0] - 1), (0, 0, 255), 1)
    cv2.imwrite(out, np.hstack([
        cv2.resize(img, (vis.shape[1], vis.shape[0])), vis]))
    print(f"  시각화     : offline/{out}  (좌: 원본, 우: BEV 마스크 + 차량축 빨강)")
    return 0 if r['ok'] else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--intrinsics', help='체커보드 사진 폴더 (K·D)')
    ap.add_argument('--ground', help='지면에 눕힌 보드 사진 1장 (H)')
    ap.add_argument('--check', help='검증할 camera.yaml')
    ap.add_argument('--straight', help='--check 용 직선 구간 프레임')
    ap.add_argument('--out', help='camera.yaml 출력 경로')
    ap.add_argument('--pattern', type=_pattern, default=(8, 5),
                    help='내부 코너 수 ColsxRows (기본 8x5 = 9x6 사각형 보드)')
    ap.add_argument('--square-mm', type=float, default=25.0,
                    help='체커 한 칸 실측 mm (인쇄물을 자로 잰 값!)')
    ap.add_argument('--cam-height-cm', type=float,
                    help='카메라 높이 실측 (cm). 주면 복원값과 교차검증한다 (필수 아님)')
    ap.add_argument('--runtime-size', type=_pattern, default=(320, 160),
                    help='런타임(인지) 해상도 WxH — 여기로 정확히 rescale 해서 저장')
    ap.add_argument('--lateral-cm', type=float, default=0.0,
                    help='지면 보드 중심의 차량축 대비 횡방향 offset (cm, + = 오른쪽)')
    ap.add_argument('--axis-offset-cm', type=float, default=0.0,
                    help='카메라가 차량 중심에 안 붙었을 때의 축 보정 (cm, + = 축을 오른쪽으로)')
    ap.add_argument('--px-per-cm', type=float, default=2.0, help='BEV 스케일 (기본 1px=5mm)')
    ap.add_argument('--x-half', type=float, help='BEV 횡방향 반폭 cm (기본: 가시범위 자동)')
    ap.add_argument('--y-far', type=float, help='BEV 최대 전방 cm (기본: 가시범위 자동)')
    ap.add_argument('--lane-width-cm', type=float, help='트랙 차선폭 (검증/리포트용)')
    a = ap.parse_args()

    if a.check:
        if not (a.straight and a.lane_width_cm):
            sys.exit('--check 에는 --straight 와 --lane-width-cm 이 필요하다')
        sys.exit(do_check(a))

    if not a.intrinsics:
        sys.exit('--intrinsics (K·D) 또는 --check 중 하나가 필요하다')
    K, D, a.rms, size = do_intrinsics(a)

    if not a.ground:
        print("\n--ground 가 없어 K·D 까지만 계산했다. camera.yaml 을 쓰려면 지면 사진이 필요하다.")
        return
    m = do_ground(a, K, D, size)

    if a.out:
        m.save(a.out)
        print(f"\n저장: {a.out}")
        print("  → 트랙 무관. 카메라를 재조준하면 --ground 만 다시 돌리면 된다.")


if __name__ == '__main__':
    main()
