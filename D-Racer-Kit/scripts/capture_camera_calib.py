#!/usr/bin/env python3
"""체커보드 캘리브레이션 사진 캡처 (D3-G에서 실행).

`/camera/image/compressed`를 **PNG 무손실**로 저장한다. mp4가 아니라 PNG인 이유: 체커보드
코너를 서브픽셀로 잡아야 하는데 동영상 압축을 거치면 그 정밀도가 나오지 않고, K·D가
오염되면 이후 BEV 전체가 무너진다.

★ 촬영은 주행 해상도(320x160)가 아니라 **640x480** 으로 한다 ★
    320x160 에서는 지면에 눕힌 보드의 코너가 사실상 안 잡힌다. 원근이 세로를 뭉개고
    2:1 비등방 스케일이 한 번 더 뭉개서 칸이 7~11px 까지 내려간다(합성 스윕: 24개
    틸트/거리 조합 중 2개만 검출). 640x480 이면 near 18~22cm 에서 틸트와 무관하게 잡힌다.
    해상도 차이는 offline/calibrate.py 가 `--runtime-size` 로 **정확히** 되돌린다
    (K 선형 스케일 / D 불변 / H 합성 — 근사 아님, 검증 완료).

  촬영 전 (해상도 올리기):
    cd ~/SC2026/D-Racer-Kit
    cp src/config/vehicle_config.yaml /tmp/vehicle_config.bak     # ★ 반드시 백업
    sed -i 's/^IMAGE_WIDTH:.*/IMAGE_WIDTH: 640/;  s/^IMAGE_HEIGHT:.*/IMAGE_HEIGHT: 480/' \\
        src/config/vehicle_config.yaml
    ros2 launch dracer_bringup calibrate.launch.py                # 카메라 재시작 필요

  1) K·D — 보드를 카메라 정면으로 들고 여러 각도·거리로 기울여가며 20장.
     화면 **가장자리**도 반드시 덮을 것 (왜곡이 가장 큰 곳이자 coast 가 나는 곳).
    python3 scripts/capture_camera_calib.py --out ~/calib/intr --count 20 --interval 1.5

  2) H — 보드를 트랙 노면에 **평평하게 눕히고**, 카메라 앞 **18~22cm**, 좌우 중앙에.
     그 거리(가장 가까운 코너열까지)를 자로 재서 --near-cm 으로 넘긴다.
    python3 scripts/capture_camera_calib.py --out ~/calib --name ground --count 1

  촬영 후 (★ 해상도 되돌리기 — 안 하면 주행 인지가 640x480 으로 돌아간다):
    cp /tmp/vehicle_config.bak src/config/vehicle_config.yaml
    grep -E "IMAGE_(WIDTH|HEIGHT)" src/config/vehicle_config.yaml   # 320 / 160 확인

  로컬로 옮겨 처리:
    scp -r topst@<D3-G_IP>:~/calib ./
    cd offline && ../.venv/bin/python calibrate.py --intrinsics ../calib/intr \\
        --ground ../calib/ground_00.png --square-mm <자로 실측한 값> --near-cm <실측> \\
        --out ../D-Racer-Kit/src/config/camera.yaml
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

PATTERN = (8, 5)   # 내부 코너 (9x6 사각형 보드). offline/calibrate.py --pattern 과 맞출 것


class Capture(Node):
    def __init__(self, a):
        super().__init__('capture_camera_calib')
        self.a = a
        self.n = 0
        self.last = 0.0
        os.makedirs(a.out, exist_ok=True)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CompressedImage, a.topic, self.cb, qos)
        self.get_logger().info(
            f'capture: {a.topic} -> {a.out}/{a.name}_*.png  '
            f'({a.count}장, {a.interval}s 간격, 코너검출 {"필수" if a.require_corners else "무시"})')

    def cb(self, msg):
        now = time.time()
        if now - self.last < self.a.interval:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return

        # 코너가 안 잡히는 장은 저장해봐야 캘리브레이션에서 버려진다 -> 촬영 중 즉시 피드백
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(
            gray, PATTERN,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if self.a.require_corners and not found:
            self.get_logger().warn('코너 미검출 — 보드를 더 크게/평평하게. 저장 안 함', throttle_duration_sec=2.0)
            return

        path = os.path.join(self.a.out, f'{self.a.name}_{self.n:02d}.png')
        cv2.imwrite(path, img)                      # PNG = 무손실
        self.last = now
        self.n += 1
        self.get_logger().info(f'[{self.n}/{self.a.count}] {path}  '
                               f'{img.shape[1]}x{img.shape[0]}  코너={"O" if found else "X"}')
        if self.n >= self.a.count:
            self.get_logger().info('완료. scp 로 로컬에 옮겨 offline/calibrate.py 실행.')
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/calib'), help='저장 폴더')
    ap.add_argument('--name', default='intr', help='파일명 prefix (K·D=intr, 지면=ground)')
    ap.add_argument('--topic', default='/camera/image/compressed')
    ap.add_argument('--count', type=int, default=20, help='저장할 장수')
    ap.add_argument('--interval', type=float, default=1.5, help='저장 간격(초) — 보드를 옮길 시간')
    ap.add_argument('--require-corners', action='store_true', default=True,
                    help='코너가 검출된 프레임만 저장 (기본 on)')
    ap.add_argument('--any-frame', dest='require_corners', action='store_false',
                    help='코너 검출과 무관하게 저장 (지면 사진 디버깅용)')
    a = ap.parse_args()

    rclpy.init()
    node = Capture(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
