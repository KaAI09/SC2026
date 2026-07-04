#!/usr/bin/env python3
"""Offline controller comparison over a recorded lane CSV (imitation eval).

The perception side compares detectors on recorded video; this compares
CONTROLLERS on the recorded (perception state <-> human steering) log produced
by lane_detect_node. No vehicle needed: for each controller structure we find
its best linear scale+bias to the human steering and report how well that
structure explains the human's driving (R^2, MAE) plus the fitted gains.

    ../../.venv/bin/python control_eval.py ../rslt/lane_O2_20260703_160323.csv

Why fit a scale/bias per controller?  A controller is a *feature transform* of
the lane state (P -> e ; PD -> [e, e_dot] ; PurePursuit -> curvature(e, heading)).
Fitting the optimal linear scaling makes the comparison fair and directly yields
candidate gains. The bias term absorbs the constant-curvature feedforward of a
one-directional loop, so we also report R^2 WITHOUT bias to expose how much a
controller relies on that (non-generalizing) offset.

Limitations: from the current CSV, C4/C5 use approximations (no polyfit lookahead
point or reliable heading logged yet). Treat their scores as indicative; log
curvature + a lookahead lateral in perception to evaluate them properly.
"""
import argparse
import csv
import math

import numpy as np


def load(csv_path):
    rows = list(csv.DictReader(open(csv_path, encoding='utf-8')))

    def col(k):
        out = []
        for r in rows:
            v = r.get(k, '').strip()
            out.append(np.nan if v == '' else float(v))
        return np.array(out)

    d = {k: col(k) for k in ('frame_time', 'center_error', 'ema', 'heading',
                             'confidence', 'manual_steering', 'manual_throttle')}
    return d


def derivative(x, t):
    dx = np.zeros_like(x)
    dt = np.gradient(t)
    dt[dt <= 0] = np.nan
    dx[1:] = (x[1:] - x[:-1]) / np.where(dt[1:] > 0, dt[1:], np.nan)
    dx = np.nan_to_num(dx)
    return dx


def integral(x, t):
    dt = np.gradient(t)
    out = np.zeros_like(x)
    acc = 0.0
    for i in range(len(x)):
        acc = float(np.clip(acc + x[i] * (dt[i] if dt[i] > 0 else 0.0), -1.0, 1.0))
        out[i] = acc
    return out


def fit(features, target, with_bias=True):
    """Least-squares fit target ~ features (+bias). Returns weights, R^2, MAE."""
    X = np.column_stack(features + ([np.ones(len(target))] if with_bias else []))
    w, *_ = np.linalg.lstsq(X, target, rcond=None)
    pred = X @ w
    ss_res = np.sum((target - pred) ** 2)
    ss_tot = np.sum((target - target.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    mae = float(np.mean(np.abs(target - pred)))
    return w, r2, mae


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--use-ema', action='store_true', help='use ema instead of center_error')
    ap.add_argument('--lookahead', type=float, default=0.6)
    args = ap.parse_args()

    d = load(args.csv)
    t = d['frame_time']
    e = d['ema'] if args.use_ema else d['center_error']
    e = np.nan_to_num(e)
    hd = np.radians(np.nan_to_num(d['heading']))
    conf = np.nan_to_num(d['confidence'], nan=1.0)
    speed = np.nan_to_num(d['manual_throttle'], nan=0.18)
    human = np.nan_to_num(d['manual_steering'])

    e_dot = derivative(e, t)
    e_int = integral(e, t)
    # controller feature transforms (unit-gain; fit finds the scaling)
    pp = (2.0 * (e + np.tan(hd) * args.lookahead)) / (args.lookahead ** 2 + 1e-6)
    stanley = hd + np.arctan2(1.0 * e, 0.15 + speed)

    controllers = {
        'C1 P': [e],
        'C2 PD': [e, e_dot],
        'C3 PID': [e, e_dot, e_int],
        'C4 PurePursuit~': [pp],
        'C5 Stanley~': [stanley],
    }

    n = len(human)
    dur = float(t[-1] - t[0]) if n > 1 else float('nan')
    print(f'# controller imitation eval  ({args.csv})')
    print(f'# frames={n} duration={dur:.1f}s fps~={n/dur:.1f} '
          f'error_source={"ema" if args.use_ema else "center_error"}')
    print(f'# human steering: mean={human.mean():+.3f} std={human.std():.3f} '
          f'saturated(|.|>=0.99)={np.mean(np.abs(human)>=0.99)*100:.0f}%')
    print()
    header = f'{"controller":16s} {"R2(+bias)":>10s} {"R2(no bias)":>12s} {"MAE":>7s}  fitted'
    print(header)
    print('-' * len(header))
    results = []
    for name, feats in controllers.items():
        w, r2, mae = fit(feats, human, with_bias=True)
        _, r2nb, _ = fit(feats, human, with_bias=False)
        gains = ', '.join(f'{g:+.3f}' for g in w[:-1])
        bias = w[-1]
        results.append((name, r2, r2nb, mae, gains, bias))
        print(f'{name:16s} {r2:>10.3f} {r2nb:>12.3f} {mae:>7.3f}  [{gains}] bias={bias:+.3f}')

    print()
    best = max(results, key=lambda r: r[1])
    print(f'best by R2(+bias): {best[0]}  (R2={best[1]:.3f})')
    print('note: high R2(+bias) but low R2(no bias) => relies on the constant-'
          'curve offset (one-directional loop); collect both directions + straights.')


if __name__ == '__main__':
    main()
