#!/usr/bin/env python3
"""체커보드 캘리브레이션 사진 캡처 (D3-G에서 실행).

웹 모니터(:5000)로 **실시간 코너 검출을 보면서** 터미널에서 Enter로 한 장씩 찍는다.
`/camera/image/compressed`를 **PNG 무손실**로 저장한다 — 코너를 서브픽셀로 잡아야 하는데
동영상/JPEG 재압축을 거치면 그 정밀도가 나오지 않고, K·D가 오염되면 BEV 전체가 무너진다.

  ros2 launch dracer_bringup calibrate.launch.py image_topic:=/calib/preview/compressed
  python3 scripts/capture_camera_calib.py --out ~/calib/intr --count 20

  웹 :5000 에서 오버레이를 보며   Enter = 저장 / q = 종료
      초록 코너 = 검출됨.  min gap = 인접 코너 최소 간격(px) = 이 촬영의 품질 지표.

★ 해상도를 올려야 하는가? — 스크립트가 판정해 준다 ★
    핵심 지표는 **min gap**(인접 코너 최소 간격, px)이다.
      >= 14px  좋음      — 그대로 촬영
      10~14px  아슬함    — 보드를 더 크게 담거나 해상도를 올리는 게 안전
      <  10px  위험      — 코너가 안 잡히거나 잡혀도 부정확 -> 해상도를 올릴 것

    320x160 이 되느냐 안 되느냐는 **카메라 FOV에 달렸다**. FOV가 넓으면 보드가 화면에 다
    들어오면서도 칸이 충분히 클 수 있고, 그러면 320x160 으로도 된다. 다만 검출이 '되는' 것과
    '정밀한' 것은 다르다 — 코너 오차가 픽셀 단위로 비슷하다면 320px 폭에서의 1px 는 640px
    에서의 2px 에 해당해 상대 오차가 2배가 되고, 그게 K·D·H 에 그대로 실린다.
    `CameraModel.rescale()` 이 해상도를 정확히(오차 0) 되돌리므로 고해상도로 찍는 데 드는
    비용은 없다. 지면 보드는 원근으로 세로가 눌려 특히 불리하니 min gap 을 꼭 볼 것.

  해상도를 올릴 경우 (촬영 전):
    cd ~/SC2026/D-Racer-Kit
    cp src/config/vehicle_config.yaml /tmp/vehicle_config.bak      # ★ 백업
    sed -i 's/^IMAGE_WIDTH:.*/IMAGE_WIDTH: 640/; s/^IMAGE_HEIGHT:.*/IMAGE_HEIGHT: 480/' \\
        src/config/vehicle_config.yaml
    # 런치 재시작 (카메라 파이프라인이 다시 열려야 한다)
  촬영 후 (★ 원복 — 안 하면 주행 인지가 640x480 으로 돌아간다):
    cp /tmp/vehicle_config.bak src/config/vehicle_config.yaml
    grep -E "IMAGE_(WIDTH|HEIGHT)" src/config/vehicle_config.yaml   # 320 / 160 확인

  1) K·D : 보드를 카메라 정면으로 들고 여러 각도·거리로 20장.
           화면 **가장자리**도 반드시 덮을 것 (왜곡이 가장 큰 곳이자 coast 가 나는 곳).
    python3 scripts/capture_camera_calib.py --out ~/calib/intr --count 20

  2) H   : 보드를 노면에 **평평하게 눕히고**, 카메라 앞 **18~22cm**, 좌우 중앙에서 1장.
           카메라 → 가장 가까운 코너열 거리를 **자로 재서 적어둘 것** (--near-cm 에 쓴다).
    python3 scripts/capture_camera_calib.py --out ~/calib --name ground --count 1

  로컬로 옮겨 처리:
    scp -r topst@<D3-G_IP>:~/calib ./
    cd offline && ../.venv/bin/python calibrate.py --intrinsics ../calib/intr \\
        --ground ../calib/ground_00.png --square-mm 25.0 --near-cm <실측> \\
        --lane-width-cm <실측> --out ../D-Racer-Kit/src/config/camera.yaml
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

PATTERN = (8, 5)        # 내부 코너 (9x6 사각형 보드). offline/calibrate.py --pattern 과 일치
GAP_GOOD, GAP_WARN = 14.0, 10.0     # min gap (px) 품질 임계
FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
         cv2.CALIB_CB_FAST_CHECK)


def min_corner_gap(corners):
    """인접 코너 최소 간격(px). 이 촬영이 쓸 만한지를 한 숫자로 요약한다."""
    c = corners.reshape(PATTERN[1], PATTERN[0], 2)
    dx = np.linalg.norm(np.diff(c, axis=1), axis=2)     # 가로 인접
    dy = np.linalg.norm(np.diff(c, axis=0), axis=2)     # 세로 인접 (지면 보드는 여기가 뭉갠다)
    return float(min(dx.min(), dy.min()))


class Capture(Node):
    def __init__(self, a):
        super().__init__('capture_camera_calib')
        self.a = a
        self.n = 0
        self.gaps = []
        self.last_auto = 0.0
        self.shot = threading.Event()
        self.quit = threading.Event()
        os.makedirs(a.out, exist_ok=True)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CompressedImage, a.topic, self.cb, qos)
        self.pub = self.create_publisher(CompressedImage, a.preview_topic, qos)

        mode = f'{a.interval}s 자동' if a.interval else 'Enter 수동'
        self.get_logger().info(
            f'\n  입력   : {a.topic}\n'
            f'  미리보기: {a.preview_topic}  → 웹 :5000 에서 확인\n'
            f'           (런치를 image_topic:={a.preview_topic} 로 띄울 것)\n'
            f'  저장   : {a.out}/{a.name}_*.png  ({a.count}장, {mode})\n'
            f'  조작   : Enter = 저장,  q = 종료')
        threading.Thread(target=self._stdin, daemon=True).start()

    def _stdin(self):
        for line in sys.stdin:
            if line.strip().lower() in ('q', 'quit', 'exit'):
                self.quit.set()
                return
            self.shot.set()

    # ---------------------------------------------------------------- preview
    def _overlay(self, img, corners, gap):
        vis = cv2.resize(img, None, fx=self.a.preview_scale, fy=self.a.preview_scale,
                         interpolation=cv2.INTER_NEAREST)
        s = self.a.preview_scale
        if corners is not None:
            cv2.drawChessboardCorners(vis, PATTERN, corners * s, True)
        h, w = vis.shape[:2]
        # ASCII only: cv2.putText (Hershey) cannot render Hangul -- it would come out as '?'.
        if corners is None:
            head, col = 'NO CORNERS - board bigger / flatter', (0, 0, 255)
        elif gap >= GAP_GOOD:
            head, col = f'OK   min gap {gap:.0f}px', (0, 220, 0)
        elif gap >= GAP_WARN:
            head, col = f'TIGHT   min gap {gap:.0f}px  (>={GAP_GOOD:.0f} preferred)', (0, 200, 255)
        else:
            head, col = f'TOO SMALL   min gap {gap:.0f}px  - raise resolution', (0, 0, 255)
        bar = 34
        cv2.rectangle(vis, (0, 0), (w, bar), (0, 0, 0), -1)
        cv2.putText(vis, head, (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        cv2.putText(vis, f'{img.shape[1]}x{img.shape[0]}   saved {self.n}/{self.a.count}'
                         f'   [Enter]=save  [q]=quit',
                    (5, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        return vis

    def _publish(self, vis, stamp):
        ok, enc = cv2.imencode('.jpg', vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        m = CompressedImage()
        m.header.stamp = stamp
        m.header.frame_id = 'calib'
        m.format = 'jpeg'
        m.data = enc.tobytes()
        self.pub.publish(m)

    # ---------------------------------------------------------------- main
    def cb(self, msg):
        if self.quit.is_set():
            raise SystemExit(0)
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN, FLAGS)
        gap = min_corner_gap(corners) if found else 0.0
        self._publish(self._overlay(img, corners if found else None, gap), msg.header.stamp)

        want = self.shot.is_set()
        if self.a.interval and (time.time() - self.last_auto) >= self.a.interval:
            want = True
        if not want:
            return
        self.shot.clear()
        self.last_auto = time.time()

        # 코너가 없는 장은 캘리브레이션에서 어차피 버려진다 -> 저장하지 않는다
        if not found:
            self.get_logger().warn('코너 미검출 — 저장 안 함')
            return

        path = os.path.join(self.a.out, f'{self.a.name}_{self.n:02d}.png')
        cv2.imwrite(path, img)                       # PNG = 무손실 (재압축 없음)
        self.n += 1
        self.gaps.append(gap)
        flag = 'OK' if gap >= GAP_GOOD else ('TIGHT' if gap >= GAP_WARN else 'TOO SMALL')
        self.get_logger().info(f'[{self.n}/{self.a.count}] {path}  '
                               f'{img.shape[1]}x{img.shape[0]}  min gap {gap:.1f}px  {flag}')
        if self.n >= self.a.count:
            raise SystemExit(0)

    def report(self):
        if not self.gaps:
            print('\n저장된 장이 없다.')
            return
        g = np.array(self.gaps)
        print(f'\n{"="*64}\n저장 {self.n}장  |  min gap: 최소 {g.min():.1f}px  '
              f'중앙 {np.median(g):.1f}px  최대 {g.max():.1f}px')
        if g.min() >= GAP_GOOD:
            print('→ 이 해상도로 충분하다. 그대로 진행.')
        elif g.min() >= GAP_WARN:
            print('→ 아슬하다. 쓸 수는 있으나 해상도를 올리면 K·D·H 정밀도가 올라간다.')
        else:
            print('→ 너무 작다. 해상도를 640x480 으로 올려 다시 찍는 것을 권한다\n'
                  '   (rescale 이 정확히 되돌리므로 손해가 없다).')
        print(f'{"="*64}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/calib'), help='저장 폴더')
    ap.add_argument('--name', default='intr', help='파일명 prefix (K·D=intr, 지면=ground)')
    ap.add_argument('--topic', default='/camera/image/compressed')
    ap.add_argument('--preview-topic', default='/calib/preview/compressed',
                    help='코너 오버레이 발행 토픽 (런치의 image_topic 에 지정해 웹으로 본다)')
    ap.add_argument('--preview-scale', type=float, default=2.0)
    ap.add_argument('--count', type=int, default=20, help='저장할 장수')
    ap.add_argument('--interval', type=float, default=0.0,
                    help='>0 이면 이 간격(초)으로 자동 저장. 기본 0 = Enter 수동')
    a = ap.parse_args()

    rclpy.init()
    node = Capture(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
