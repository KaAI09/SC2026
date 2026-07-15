"""ctrl_feedback 순수함수 유닛테스트. pytest 없이 `python3 test_ctrl_feedback.py`(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from ctrl_feedback import curve_error, load_params, DEFAULT_PARAMS  # noqa: E402


def _row(t, ce_cm, steer):
    return {'frame_time': t, 'center_error_cm': ce_cm, 'ctrl_steering': steer,
            'ctrl_throttle': 0.23, 'state': 'OK', 'used_fallback': 0, 'center_error': ce_cm / 29.0}


def test_curve_error_auto_segment():
    # 앞 2개는 직선(steer~0), 뒤 3개는 커브(steer 0.4) — 커브에서만 집계
    rows = [_row(0.0, 1.0, 0.02), _row(0.03, -1.0, 0.02),
            _row(0.06, 8.0, 0.40), _row(0.09, 10.0, 0.42), _row(0.12, 6.0, 0.40)]
    r = curve_error(rows, segments=[])
    assert r['n'] == 3, r
    assert abs(r['e_ss_cm'] - 8.0) < 1e-9, r      # median(|8,10,6|) = 8
    assert abs(r['u_ss'] - 0.40) < 1e-9, r        # median(|0.40,0.42,0.40|) = 0.40


def test_load_params_defaults_when_no_sidecar():
    # 존재하지 않는 csv 경로 → 사이드카 없음 → overrides+기본값
    p = load_params('/nonexistent/drive_x.csv', overrides={'kp': 1.3})
    assert p['kp'] == 1.3, p
    assert p['x_half_cm'] == DEFAULT_PARAMS['x_half_cm'], p


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
