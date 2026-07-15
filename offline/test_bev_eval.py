"""bev_eval 순수함수 유닛테스트. pytest 없이 `python3 test_bev_eval.py` 로 돈다(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from bev_eval import (scale_stats, coast_transitions,   # noqa: E402 (import 줄 교체)
                      parse_segments, label_for_time)


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


def _st(cm_cm, coast, both):
    return {'center_error_cm': cm_cm, 'used_fallback': coast,
            'left_conf': 1.0 if both else 0.0, 'right_conf': 1.0 if both else 0.0}


def test_coast_transition_error():
    states = [_st(5.0, True, False),    # coast
              _st(6.2, False, True)]    # → pair, 답 6.2 vs 직전 5.0 = 1.2cm
    tr = coast_transitions(states)
    assert len(tr) == 1, tr
    assert abs(tr[0]['err_cm'] - 1.2) < 1e-9, tr
    assert tr[0]['side_flip'] is False, tr


def test_coast_side_flip():
    states = [_st(-20.0, True, False),  # coast 는 좌(-)
              _st(19.0, False, True)]   # pair 는 우(+), |Δ|=39>17.4 & 부호반전 → flip
    tr = coast_transitions(states)
    assert len(tr) == 1 and tr[0]['side_flip'] is True, tr


def test_label_for_time():
    segs = [(0.0, 2.0, '직선'), (2.0, 5.0, '좌커브'), (5.0, 9.0, '우커브')]
    assert label_for_time(segs, 1.0) == '직선'
    assert label_for_time(segs, 2.0) == '좌커브'    # 경계는 start 포함
    assert label_for_time(segs, 8.9) == '우커브'
    assert label_for_time(segs, 9.0) == ''          # 범위 밖


def test_parse_segments(tmp_path=None):
    import tempfile
    txt = 'start_s,end_s,label\n0,2.0,직선\n2.0,5,좌커브\n'
    fd, path = tempfile.mkstemp(suffix='.csv')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(txt)
    segs = parse_segments(path)
    os.remove(path)
    assert segs == [(0.0, 2.0, '직선'), (2.0, 5.0, '좌커브')], segs


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
