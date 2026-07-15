"""주행 로그 기반 제어 파라미터 피드백 — throttle 상승에 따른 게인 스케줄링.

진단(곡선 e_ss·진동·조향포화·slew포화) + 권장값(PD 관계식) + 병목 식별. 매 실행이 누적 로그에
1행 append 하여, throttle 다른 주행이 쌓이면 throttle↔파라미터 추세를 피팅한다.

    python offline/ctrl_feedback.py <drive.csv> [--segments seg.csv] [--target-cm 17.4] [--no-log]

파라미터는 <drive>.meta.json 사이드카(recorder 기록)에서 읽는다. 없으면 --params k=v 로.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import _common as cm                                              # noqa: E402

DEFAULT_PARAMS = {
    'session': '', 'controller': 'PD', 'kp': 1.3, 'kd': 0.145,
    'throttle_base': 0.23, 'throttle_min': 0.22, 'slew_rate_per_sec': 7.5,
    'steer_max': 1.0, 'conf_gate': 0.4, 'curv_slow': 0.0, 'x_half_cm': 29.0,
}


def load_params(csv_path, overrides=None):
    """<basename>.meta.json 사이드카 우선, 없으면 기본값+overrides(경고)."""
    side = os.path.splitext(csv_path)[0] + '.meta.json'
    out = dict(DEFAULT_PARAMS)
    if os.path.exists(side):
        with open(side, encoding='utf-8') as f:
            out.update(json.load(f))
    else:
        out['_no_sidecar'] = True
    if overrides:
        out.update(overrides)
    return out


def _fval(row, key):
    v = row.get(key)
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def _is_curve(row, segments, t, steer_thresh):
    if segments:
        from bev_eval import label_for_time
        return label_for_time(segments, t) in ('좌커브', '우커브')
    return abs(_fval(row, 'ctrl_steering')) >= steer_thresh


def curve_error(rows, segments, steer_thresh=0.15):
    """커브 프레임의 정상상태 횡오차(cm)와 유지 조향. e_ss = median|center_error_cm|."""
    es, us = [], []
    for r in rows:
        t = _fval(r, 'frame_time')
        if _is_curve(r, segments, t, steer_thresh):
            ce, st = _fval(r, 'center_error_cm'), _fval(r, 'ctrl_steering')
            if np.isfinite(ce) and np.isfinite(st):
                es.append(abs(ce))
                us.append(abs(st))
    return {'e_ss_cm': float(np.median(es)) if es else float('nan'),
            'u_ss': float(np.median(us)) if us else float('nan'),
            'n': len(es)}


def oscillation_index(rows, segments, steer_thresh=0.15):
    """직선 프레임에서 center_error 부호변화 횟수 / 지속시간(초). 저감쇠 신호."""
    ce, ts = [], []
    for r in rows:
        t = _fval(r, 'frame_time')
        if not _is_curve(r, segments, t, steer_thresh):
            c = _fval(r, 'center_error_cm')
            if np.isfinite(c):
                ce.append(c)
                ts.append(t)
    if len(ce) < 2:
        return float('nan')
    flips = sum(1 for i in range(1, len(ce)) if ce[i] * ce[i - 1] < 0)
    dur = ts[-1] - ts[0]
    return float(flips / dur) if dur > 0 else float('nan')


def saturation_rates(rows, params):
    """조향 포화율(|steer|>=0.6)과 slew 포화율(|Δsteer/Δt|>=0.95·slew_rate)."""
    steers = np.array([_fval(r, 'ctrl_steering') for r in rows], dtype=float)
    times = np.array([_fval(r, 'frame_time') for r in rows], dtype=float)
    good = np.isfinite(steers)
    steer_sat = float(np.mean(np.abs(steers[good]) >= 0.6)) if good.any() else float('nan')
    slew = params.get('slew_rate_per_sec', 7.5)
    n_slew, hit = 0, 0
    for i in range(1, len(steers)):
        dt = times[i] - times[i - 1]
        if np.isfinite(steers[i]) and np.isfinite(steers[i - 1]) and dt > 0:
            n_slew += 1
            if abs(steers[i] - steers[i - 1]) / dt >= 0.95 * slew:
                hit += 1
    slew_sat = float(hit / n_slew) if n_slew else float('nan')
    return {'steer_sat': steer_sat, 'slew_sat': slew_sat}
