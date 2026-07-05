#!/usr/bin/env python3
"""Stage 4 (control) — RANK controllers by open-loop command quality, then hand off.

Reads a predicted CSV (control_predict.py output) and scores each candidate
controller's command sequence with OPEN-LOOP metrics only — offline cannot measure
the closed-loop trajectory (covariate shift; see offline/CONTROL_DESIGN.md §5.0).
Human steering is a reference, NOT ground truth.

Metrics per controller:
  smoothness  mean|Δu|, RMS jerk        (lower = smoother)
  oscillation sign-change rate of u     (lower = less wobble)
  response    corr(u, -center_error)    (higher = steers to reduce error)
  saturation  frac |u| >= 0.95          (lower)
  gating      frac low-conf holds       (lower)
  human(ref)  corr / MAE vs manual      (reference only)
  score       response x stability x (1-oscillation)

Export is human-triggered: --export CTRL --profile PATH writes the control section.

    python control_select.py rslt/pred_drive.csv
    python control_select.py rslt/pred_drive.csv --export C2 \
        --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml

Requires driving_core importable (pip install -e D-Racer-Kit/src/driving_core).
"""
import argparse

import numpy as np

from driving_core.control_core import make_ctrl

import _common as cm


def _corr(a, b):
    if a.size < 2 or np.std(a) == 0 or np.std(b) == 0:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


def control_metrics(u, e, human):
    """Open-loop command-quality metrics for one controller's steering sequence u,
    given lane error e (center_error) and human steering (reference)."""
    u = np.nan_to_num(u)
    du = np.diff(u)
    slew = float(np.mean(np.abs(du))) if du.size else float('nan')
    jerk = float(np.sqrt(np.mean(np.diff(du) ** 2))) if du.size > 1 else float('nan')
    sign = np.sign(u)
    osc = float(np.mean(sign[1:] != sign[:-1])) if u.size > 1 else float('nan')
    sat = float(np.mean(np.abs(u) >= 0.95))
    valid = ~np.isnan(e)
    resp = _corr(u[valid], -np.nan_to_num(e[valid])) if valid.sum() > 2 else float('nan')
    hcorr = _corr(u, human) if human.size else float('nan')
    hmae = float(np.mean(np.abs(u - human))) if human.size else float('nan')
    stability = 1.0 / (1.0 + 8.0 * (slew if slew == slew else 1.0))
    r = 0.0 if resp != resp else max(0.0, resp)
    o = 0.0 if osc != osc else osc
    score = float(r * stability * (1.0 - o))
    return {'slew': slew, 'jerk': jerk, 'oscillation': osc, 'saturation': sat,
            'response': resp, 'human_corr': hcorr, 'human_mae': hmae, 'score': score}


def control_section(ctrl):
    c = make_ctrl(ctrl)
    return {'controller': ctrl, 'kp': c.kp, 'kd': c.kd, 'ki': c.ki,
            'center_target': c.center_target, 'steer_max': c.steer_max,
            'steer_sign': c.steer_sign, 'slew_rate': c.slew_rate,
            'throttle_base': c.throttle_base, 'throttle_min': c.throttle_min,
            'conf_gate': c.conf_gate}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pred_csv', help='control_predict.py output')
    ap.add_argument('--controllers', help='comma list (default: auto-detect columns)')
    ap.add_argument('--export', help='write this CONTROLLER as the control profile section')
    ap.add_argument('--profile', help='profile YAML path for --export')
    args = ap.parse_args()

    rows = cm.read_csv(args.pred_csv)
    if args.controllers:
        ctrls = [c.strip() for c in args.controllers.split(',') if c.strip()]
    else:
        ctrls = [k[len('pred_steer_'):] for k in rows[0] if k.startswith('pred_steer_')]
    if not ctrls:
        raise SystemExit('no pred_steer_* columns found')

    e = cm.col(rows, 'center_error')
    human = np.nan_to_num(cm.col(rows, 'manual_steering', default=0.0))
    print(f'# control_select: {len(ctrls)} controllers, {len(rows)} frames')
    print('# score = max(0,response) x 1/(1+8*slew) x (1-oscillation)\n')
    hdr = (f'{"controller":12s} {"score":>6s} {"resp":>6s} {"slew":>6s} {"jerk":>6s} '
           f'{"osc":>6s} {"sat":>6s} {"h_corr":>7s} {"h_mae":>6s}')
    print(hdr); print('-' * len(hdr))
    results = []
    for c in ctrls:
        u = cm.col(rows, f'pred_steer_{c}', default=0.0)
        m = control_metrics(u, e, human)
        results.append((c, m))
        print(f'{c:12s} {m["score"]:6.2f} {m["response"]:6.2f} {m["slew"]:6.3f} '
              f'{m["jerk"]:6.3f} {m["oscillation"]:6.2f} {m["saturation"]:6.2f} '
              f'{m["human_corr"]:7.2f} {m["human_mae"]:6.3f}')
    best = max(results, key=lambda r: (r[1]['score'] if r[1]['score'] == r[1]['score'] else -1))
    print(f'\nbest by score: {best[0]}  (score={best[1]["score"]:.2f})')
    print('note: open-loop only; confirm on-vehicle (wheels-off -> low speed). '
          'human is a reference, not ground truth.')

    if args.export:
        if not args.profile:
            raise SystemExit('--export CTRL requires --profile PATH')
        cm.write_profile_section(args.profile, 'control', control_section(args.export))
        print(f'exported {args.export} -> {args.profile} [control]')


if __name__ == '__main__':
    main()
