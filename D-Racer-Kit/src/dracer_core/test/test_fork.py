"""갈림길 판단 순수함수 유닛테스트. pytest 스타일(assert)이되, pytest 없이도
`.venv/bin/python test_fork.py` 로 직접 돈다(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from dracer_core.perception_core import classify  # noqa: E402
from dracer_core.perception_core import lane_centers, Cfg, LanePipeline  # noqa: E402
from dracer_core.calib import CameraModel  # noqa: E402
import numpy as np  # noqa: E402


def _ins(color, turn, x_bottom):
    return {'color': color, 'turn': turn, 'x_bottom': x_bottom,
            'x_mean': x_bottom, 'coeffs': (0.0, 0.0, x_bottom), 'ys': None}


def test_side_from_corridor_ab():
    # 차가 오른쪽으로 치우쳐 두 벽이 모두 화면 왼쪽(<116)이어도 corridor 로 갈린다
    a = _ins('W', -1, 40.0)   # 왼벽(x작음)
    b = _ins('W', +1, 90.0)   # 오른벽(x큼)
    corridor = {'a': a, 'b': b}
    assert classify(a, 232, corridor=corridor) == 'WL-L'   # 왼벽 → L
    assert classify(b, 232, corridor=corridor) == 'WR-R'   # 오른벽 → R


def test_side_from_tracker_identity():
    a = _ins('W', 0, 40.0)
    b = _ins('W', 0, 90.0)
    assert classify(a, 232, mL=a, mR=b) == 'WS-L'
    assert classify(b, 232, mL=a, mR=b) == 'WS-R'


def test_side_fallback_screen():
    # corridor/tracker 없으면 기존 화면위치(_side)
    left = _ins('Y', +1, 40.0)    # 축 116 왼쪽
    assert classify(left, 232) == 'YR-L'


def _line(color, x0, slope, turn):
    ys = np.arange(0, 189, 4.0)
    xs = x0 + slope * (188 - ys)          # v 클수록(아래=가까움) x0
    return {'color': color, 'turn': turn, 'coeffs': (0.0, -slope, x0 + slope * 188),
            'x_bottom': float(x0), 'x_mean': float(xs.mean()), 'ys': ys, 'xs': xs}


def test_fork_island_lr_flagged():
    c = Cfg(); c.fork_spread_min = 100.0   # 25cm*4px; 테스트는 px 직접
    c.lane_width_default = 0.6; c.pair_same_color = True
    c.pair_parallel = 0.0; c.pair_gap_min = 0.0; c.pair_width_tol = 0.0; c.pair_overlap_min = 0.0
    # gap(y) = 27.2 + 0.6y: never crosses zero (min 27.2 at y=0) and spreads to 140.0 at
    # y=188, so spread(=113) clears fork_spread_min without tripping pair_gap_min.
    # lane_w_px=0 disables the physical-width gate -- this test is about the spread/turn
    # gate, not about matching a real lane width to a zero-tolerance band.
    left = _line('W', 60.0, +0.30, -1)     # 왼벽; turn=-1 → 'L'
    right = _line('W', 200.0, -0.30, +1)   # 오른벽; turn=+1 → 'R'
    cors = lane_centers([left, right], 232, 189, c, lane_w_px=0.0)
    assert cors, 'corridor 하나는 나와야'
    isl = [x for x in cors if x.get('is_fork')]
    assert isl and isl[0]['fork_type'] == 'island'
    assert isl[0]['turn_pair'] == ('L', 'R')


def test_pipeline_set_branch_hint():
    cam = CameraModel.load('D-Racer-Kit/src/config/camera.yaml') if os.path.exists(
        'D-Racer-Kit/src/config/camera.yaml') else CameraModel.load(
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'camera.yaml'))
    p = LanePipeline(Cfg(), cam)
    p.set_branch_hint('L'); assert p._branch_hint == 'L'
    p.set_branch_hint('bogus'); assert p._branch_hint is None


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
