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


def suggest_kp(e_ss_cm, u_ss, x_half_cm, target_cm):
    """e_ss 를 target_cm 이하로 만드는 kp. u_ss·x_half/target. u_ss 불명이면 None."""
    if not (np.isfinite(u_ss) and np.isfinite(x_half_cm) and target_cm > 0):
        return None
    return float(abs(u_ss) * x_half_cm / target_cm)


def diagnose(rows, params, segments, target_cm=17.4, osc_thresh=5.0,
             steer_sat_thresh=0.10, slew_sat_thresh=0.10):
    """모든 지표 계산 + throttle 병목 식별 + kp 제안."""
    ce = curve_error(rows, segments)
    osc = oscillation_index(rows, segments)
    sat = saturation_rates(rows, params)
    binding = []
    if np.isfinite(ce['e_ss_cm']) and ce['e_ss_cm'] > target_cm:
        binding.append('e_ss')          # 곡선 이탈 → kp 부족/속도 과다
    if np.isfinite(osc) and osc > osc_thresh:
        binding.append('oscillation')   # 저감쇠 → kd 부족/kp 과다
    if np.isfinite(sat['steer_sat']) and sat['steer_sat'] > steer_sat_thresh:
        binding.append('steer_sat')     # 조향 포화 → 코너속도 하드리밋
    if np.isfinite(sat['slew_sat']) and sat['slew_sat'] > slew_sat_thresh:
        binding.append('slew_sat')      # 조향속도 병목
    kp_new = (suggest_kp(ce['e_ss_cm'], ce['u_ss'], params.get('x_half_cm', 29.0), target_cm)
              if 'e_ss' in binding else None)
    return {'throttle_base': params.get('throttle_base'), 'kp': params.get('kp'),
            'kd': params.get('kd'), 'e_ss_cm': ce['e_ss_cm'], 'u_ss': ce['u_ss'],
            'curve_n': ce['n'], 'oscillation': osc, 'steer_sat': sat['steer_sat'],
            'slew_sat': sat['slew_sat'], 'binding': binding, 'suggest_kp': kp_new}


_LOG_COLS = ['session', 'throttle_base', 'kp', 'kd', 'e_ss_cm', 'u_ss',
             'oscillation', 'steer_sat', 'slew_sat', 'binding', 'suggest_kp']


def append_log(log_path, session, diag):
    """누적 로그에 1행 append(없으면 헤더 생성). throttle↔파라미터 A/B 데이터셋."""
    import csv as _csv
    new = not os.path.exists(log_path)
    with open(log_path, 'a', newline='', encoding='utf-8') as f:
        w = _csv.writer(f)
        if new:
            w.writerow(_LOG_COLS)
        w.writerow([session, diag['throttle_base'], diag['kp'], diag['kd'],
                    diag['e_ss_cm'], diag['u_ss'], diag['oscillation'],
                    diag['steer_sat'], diag['slew_sat'], '|'.join(diag['binding']),
                    diag['suggest_kp'] if diag['suggest_kp'] is not None else ''])


def fit_throttle_trend(log_rows, metric='steer_sat', limit=0.3):
    """throttle_base vs metric 선형 피팅(점≥3). metric 이 limit 도달하는 throttle 추정."""
    xs, ys = [], []
    for r in log_rows:
        try:
            x, y = float(r['throttle_base']), float(r[metric])
        except (TypeError, ValueError, KeyError):
            continue
        if np.isfinite(x) and np.isfinite(y):
            xs.append(x)
            ys.append(y)
    if len(xs) < 3:
        return {'n': len(xs), 'slope': None, 'throttle_at_limit': None}
    slope, intercept = np.polyfit(xs, ys, 1)
    t_lim = ((limit - intercept) / slope) if slope != 0 else None
    return {'n': len(xs), 'slope': float(slope),
            'throttle_at_limit': float(t_lim) if t_lim is not None else None}


def _parse_overrides(pairs):
    ov = {}
    for kv in pairs:
        k, _, v = kv.partition('=')
        try:
            ov[k] = float(v)
        except ValueError:
            ov[k] = v
    return ov


def cmd_run(a):
    rows = cm.read_csv(os.path.expanduser(a.csv))
    segments = []
    if a.segments:
        from bev_eval import parse_segments
        segments = parse_segments(a.segments)
    params = load_params(os.path.expanduser(a.csv), _parse_overrides(a.params))
    d = diagnose(rows, params, segments, target_cm=a.target_cm)
    print(cm.clip_name(a.csv), '|', f"throttle={d['throttle_base']} kp={d['kp']} kd={d['kd']}"
          + ('  ⚠ 사이드카 없음(파라미터 추정)' if params.get('_no_sidecar') else ''))
    print(f"  곡선 e_ss {d['e_ss_cm']:.1f}cm (n={d['curve_n']}, u_ss {d['u_ss']:.2f}) | "
          f"진동 {d['oscillation']:.1f}/s | 조향포화 {100*d['steer_sat']:.1f}% | "
          f"slew포화 {100*d['slew_sat']:.1f}%")
    print(f"  병목: {', '.join(d['binding']) if d['binding'] else '없음(여유 있음)'}")
    if d['suggest_kp'] is not None:
        print(f"  권장 kp {d['kp']} → {d['suggest_kp']:.2f} (e_ss 를 {a.target_cm:.1f}cm 이하로)")
    if not a.no_log:
        log_path = os.path.join(os.path.dirname(__file__), 'ctrl_feedback_log.csv')
        session = params.get('session') or cm.clip_name(a.csv)
        append_log(log_path, session, d)
        rows_log = cm.read_csv(log_path)
        tr = fit_throttle_trend(rows_log, metric='steer_sat')
        if tr['slope'] is not None:
            msg = (f"throttle_at_steer_sat=0.3 ≈ {tr['throttle_at_limit']:.3f}"
                   if tr['throttle_at_limit'] else '기울기 0')
            print(f"  [누적 n={tr['n']}] 조향포화 추세 slope {tr['slope']:.2f}/throttle → {msg}")
        else:
            print(f"  [누적 n={tr['n']}] throttle 추세 미확정(점 3개 이상 필요)")
    return d, params


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--segments', default='')
    ap.add_argument('--target-cm', dest='target_cm', type=float, default=17.4)
    ap.add_argument('--params', action='append', default=[], metavar='K=V',
                    help='사이드카 없을 때 파라미터 수동 지정 (반복 가능)')
    ap.add_argument('--no-log', action='store_true', help='누적 로그 append 생략')
    a = ap.parse_args()
    cmd_run(a)


if __name__ == '__main__':
    main()
