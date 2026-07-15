"""갈림길 판단 순수함수 유닛테스트. pytest 스타일(assert)이되, pytest 없이도
`.venv/bin/python test_fork.py` 로 직접 돈다(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from dracer_core.perception_core import classify  # noqa: E402


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


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
