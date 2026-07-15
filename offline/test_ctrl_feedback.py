"""ctrl_feedback 순수함수 유닛테스트. pytest 없이 `python3 test_ctrl_feedback.py`(하단 러너)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from ctrl_feedback import (curve_error, load_params, DEFAULT_PARAMS,   # noqa: E402 (교체)
                           oscillation_index, saturation_rates,
                           suggest_kp, diagnose, fit_throttle_trend)


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


def test_oscillation_index_straight():
    # 직선(steer~0)에서 center_error 가 +,-,+,- 로 3번 부호변화, 0.09초간
    rows = [_row(0.0, 2.0, 0.01), _row(0.03, -2.0, 0.01),
            _row(0.06, 2.0, 0.01), _row(0.09, -2.0, 0.01)]
    idx = oscillation_index(rows, segments=[])
    # 부호변화 3회 / (0.09-0.0)초 = 33.33/s
    assert abs(idx - 3 / 0.09) < 1e-6, idx


def test_saturation_rates():
    p = dict(DEFAULT_PARAMS, slew_rate_per_sec=7.5)
    # steer: 0.3, 0.7(포화), 0.7 → |steer|>=0.6 은 3프레임 중 2 = 2/3.
    # slew: 전환 2개(i=1,2) 중 i=1 만 Δ=0.4/0.03s=13.3>=0.95·7.5 → 1/2.
    rows = [_row(0.0, 0.0, 0.30), _row(0.03, 0.0, 0.70), _row(0.06, 0.0, 0.70)]
    r = saturation_rates(rows, p)
    assert abs(r['steer_sat'] - 2 / 3) < 1e-9, r
    assert abs(r['slew_sat'] - 1 / 2) < 1e-9, r    # 전환 2개 중 1개가 slew 한계 초과


def test_suggest_kp():
    # u_ss=0.5, x_half=29, target=17.4 → kp = 0.5*29/17.4 = 0.8333...
    kp = suggest_kp(e_ss_cm=25.0, u_ss=0.5, x_half_cm=29.0, target_cm=17.4)
    assert abs(kp - 0.5 * 29.0 / 17.4) < 1e-9, kp
    # u_ss NaN 이면 제안 불가 → None
    assert suggest_kp(25.0, float('nan'), 29.0, 17.4) is None


def test_diagnose_binding_ess():
    p = dict(DEFAULT_PARAMS)
    # 커브에서 e_ss 25cm(>17.4=반폭) → binding 에 'e_ss' 포함
    rows = [_row(0.0, 25.0, 0.40), _row(0.03, 25.0, 0.42), _row(0.06, 25.0, 0.40)]
    d = diagnose(rows, p, segments=[], target_cm=17.4)
    assert 'e_ss' in d['binding'], d
    assert d['suggest_kp'] is not None, d


def test_fit_throttle_trend_slope():
    # throttle 0.20/0.24/0.28 에서 steer_sat 0.1/0.2/0.3 → slope=2.5/1단위throttle
    log_rows = [{'throttle_base': '0.20', 'steer_sat': '0.1'},
                {'throttle_base': '0.24', 'steer_sat': '0.2'},
                {'throttle_base': '0.28', 'steer_sat': '0.3'}]
    r = fit_throttle_trend(log_rows, metric='steer_sat')
    assert r['n'] == 3, r
    assert abs(r['slope'] - (0.2 / 0.08)) < 1e-6, r     # ΔY/ΔX = 0.2/0.08 = 2.5


def test_fit_throttle_trend_insufficient():
    r = fit_throttle_trend([{'throttle_base': '0.2', 'steer_sat': '0.1'}], metric='steer_sat')
    assert r['n'] == 1 and r['slope'] is None, r        # 점 부족 → 미확정


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'PASS {name}')
    print('all ok')
