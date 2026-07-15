"""bev_eval 순수함수 유닛테스트. pytest 없이 `python3 test_bev_eval.py` 로 돈다(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from bev_eval import scale_stats  # noqa: E402


def test_scale_stats_ratio():
    # b = a/2 → 정규화 center_error 비 a/b = 2 (작은 x_half 쪽이 값이 2배)
    a = [0.2, 0.4, -0.6]
    b = [0.1, 0.2, -0.3]
    s = scale_stats(a, b)
    assert s['n'] == 3, s
    assert abs(s['ratio_median'] - 2.0) < 1e-9, s


def test_scale_stats_skips_small_and_nan():
    a = [0.2, float('nan'), 0.0005]
    b = [0.1, 0.2, 0.0002]   # b[2] 도 <1e-3 → 제외, a[1] nan 제외
    s = scale_stats(a, b)
    assert s['n'] == 1, s
    assert abs(s['ratio_median'] - 2.0) < 1e-9, s


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
