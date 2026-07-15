"""center_error_cm 은 calib 불변(cm) 물리오차여야 한다: offset/px_per_cm == center_error*x_half_cm.
pytest 없이 `python3 test_center_error_cm.py` 로 돈다(하단 러너). 클립 없으면 SKIP."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import cv2  # noqa: E402
from dracer_core.calib import CameraModel  # noqa: E402
from dracer_core.perception_core import Cfg, LanePipeline  # noqa: E402

_HERE = os.path.dirname(__file__)
CAM = os.path.join(_HERE, '..', '..', 'config', 'camera.yaml')
CLIP = os.path.join(_HERE, '..', '..', '..', '..', 'offline', 'rslt',
                    '07142315', 'raw', 'drive_20260714_141218.mp4')


def test_center_error_cm_identity():
    if not (os.path.exists(CAM) and os.path.exists(CLIP)):
        print('SKIP test_center_error_cm_identity (no cam/clip)')
        return
    cam = CameraModel.load(CAM)
    cap = cv2.VideoCapture(CLIP)
    ok, first = cap.read()
    assert ok, 'empty clip'
    cam = cam.match((first.shape[1], first.shape[0]))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    pipe = LanePipeline(Cfg(), cam)
    checked, i = 0, 0
    while checked < 20:
        ok, frame = cap.read()
        if not ok:
            break
        st = pipe.process(frame, 0.0 if i == 0 else 1.0 / fps)
        i += 1
        if st['center_error'] is not None:
            assert 'center_error_cm' in st, 'state dict missing center_error_cm'
            expect = st['center_error'] * cam.x_half_cm
            got = st['center_error_cm']
            assert got is not None and abs(got - expect) < 1e-6, (got, expect)
            checked += 1
    cap.release()
    assert checked > 0, 'no valid frame produced a center_error'
    print(f'PASS identity on {checked} valid frames')


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
    print('all ok')
