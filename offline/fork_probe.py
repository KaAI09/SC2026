"""갈림길 감지 검증: 클립별 island 감지율. 일반주행에서 오검출률(목표 <1%)."""
import os, sys
import cv2
sys.path.insert(0, os.path.join('D-Racer-Kit', 'src', 'dracer_core'))
from dracer_core.calib import CameraModel
from dracer_core.perception_core import LanePipeline, cfg_from_profile
from dracer_core.profile import load_profile, section


def rate(clips, cam, c0):
    n = hit = 0
    for clip in clips:
        cap = cv2.VideoCapture(clip); pipe = LanePipeline(c0, cam)
        while True:
            ok, f = cap.read()
            if not ok:
                break
            n += 1
            _, dbg = pipe.process(f, 1 / 30.0, debug=True)
            if any(cc.get('fork_type') == 'island' for cc in (dbg.get('centers') or [])):
                hit += 1
        cap.release()
    return hit, n


if __name__ == '__main__':
    cam = CameraModel.load('D-Racer-Kit/src/config/camera.yaml')
    c0 = cfg_from_profile(section(load_profile(
        'D-Racer-Kit/src/config/profiles/track.yaml'), 'perception'))
    fork = ['offline/rslt/07140515/raw/raw_20260714_081354.mp4',
            'offline/rslt/07140515/raw/raw_20260714_081458.mp4']
    normal = ['offline/rslt/07142315/raw/drive_20260714_141234.mp4',
              'offline/rslt/07142315/raw/drive_20260714_141332.mp4']
    h, n = rate(fork, cam, c0);   print(f'갈림길 클립  island 감지율: {h}/{n} = {100*h/n:.1f}%')
    h, n = rate(normal, cam, c0); print(f'일반주행    island 오검출: {h}/{n} = {100*h/n:.1f}%  (목표 <1%)')
