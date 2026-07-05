#!/usr/bin/env python3
"""Pipeline step 4 (offline) — analyze TRACK CONDITIONS from manual drive video.

Measures, from the raw clips of a track, the parameters that seed the G1..G6
groups so detection is tuned to THIS track's actual colors/geometry:
  - white marking band   -> white_s_max, white_v_min
  - yellow marking band  -> yellow_h_lo/hi, yellow_s_min, yellow_v_min
  - ROI (lane heatmap)   -> roi_top_frac, trap_top_w
  - lane width           -> lane_width_default (fraction of width)

Outputs a printed report of suggested driving_core.lane_core.Cfg values plus two
PNGs (ROI heatmap with suggested trapezoid; white/yellow mask samples). Run it on
the 2nd-run manual clips (Launch 2 output); re-run on the 2026 car-camera clips on
competition day to re-tune. It only measures — it does not write any profile.

    python track_analyze.py "Dashcam(2025 Track)"/0704*.mp4
    python track_analyze.py CLIPS... --frames 8 --outdir rslt

Requires driving_core importable (pip install -e D-Racer-Kit/src/driving_core).
"""
import argparse
import os

import cv2
import numpy as np

from driving_core.lane_core import LanePipeline, make_cfg

import _common as cm

W, H = 320, 160


def _pct(a, p):
    return float(np.percentile(a, p)) if a.size else float('nan')


def sample_hsv(clips, k):
    """Sample k frames/clip, return (bgr_frames, hsv_frames) resized to W x H."""
    bgrs, hsvs = [], []
    for path in clips:
        cap, fps, n = cm.open_clip(path)
        for i in np.linspace(n * 0.1, n * 0.9, k).astype(int):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, fr = cap.read()
            if not ok:
                continue
            fr = cv2.resize(fr, (W, H), interpolation=cv2.INTER_AREA)
            bgrs.append(fr)
            hsvs.append(cv2.cvtColor(fr, cv2.COLOR_BGR2HSV))
        cap.release()
    return bgrs, hsvs


def white_band(hsvs):
    """Isolate the white-marking cluster (bright, low-sat) and suggest a band."""
    S, V = [], []
    for hsv in hsvs:
        m = (hsv[..., 1] <= 60) & (hsv[..., 2] >= 170)   # loose isolation gate
        if m.sum() > 20:
            S.append(hsv[..., 1][m]); V.append(hsv[..., 2][m])
    if not S:
        return None
    S = np.concatenate(S); V = np.concatenate(V)
    s_max = int(min(90, round((_pct(S, 95) + 8) / 5) * 5))
    v_min = int(max(120, round((_pct(V, 5) - 8) / 5) * 5))
    return {'S_p50': _pct(S, 50), 'S_p95': _pct(S, 95),
            'V_p5': _pct(V, 5), 'V_p50': _pct(V, 50),
            'white_s_max': s_max, 'white_v_min': v_min}


def yellow_band(hsvs):
    """Isolate the yellow-marking cluster (yellow hue, saturated) and suggest a band."""
    Hh, S, V = [], [], []
    for hsv in hsvs:
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = (h >= 10) & (h <= 45) & (s >= 55) & (v >= 80)
        if m.sum() > 20:
            Hh.append(h[m]); S.append(s[m]); V.append(v[m])
    if not Hh:
        return None
    Hh = np.concatenate(Hh); S = np.concatenate(S); V = np.concatenate(V)
    return {'H_p5': _pct(Hh, 5), 'H_p95': _pct(Hh, 95), 'S_p5': _pct(S, 5),
            'V_p5': _pct(V, 5),
            'yellow_h_lo': int(max(0, _pct(Hh, 5) - 2)),
            'yellow_h_hi': int(min(179, _pct(Hh, 95) + 2)),
            'yellow_s_min': int(max(30, _pct(S, 5))),
            'yellow_v_min': int(max(60, _pct(V, 5)))}


def _lane_mask(hsv, wb, yb):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    white = (s <= (wb['white_s_max'] if wb else 60)) & (v >= (wb['white_v_min'] if wb else 185))
    if yb:
        yellow = ((h >= yb['yellow_h_lo']) & (h <= yb['yellow_h_hi'])
                  & (s >= yb['yellow_s_min']) & (v >= yb['yellow_v_min']))
    else:
        yellow = np.zeros_like(white)
    return white | yellow


def roi_analysis(hsvs, wb, yb, out):
    """Accumulate a lane heatmap -> suggest roi_top_frac / trap widths; render PNG."""
    heat = np.zeros((H, W), np.float64)
    for hsv in hsvs:
        heat += _lane_mask(hsv, wb, yb).astype(np.float64)
    heat /= max(len(hsvs), 1)
    row_cov = heat.mean(axis=1)
    tot = row_cov.sum() or 1.0

    # top of ROI = row above which <5% of signal remains (from the top downward)
    cum, roi_top_row = 0.0, 0
    for y in range(H):
        cum += row_cov[y]
        if cum >= 0.05 * tot:
            roi_top_row = y
            break

    def x_extent(y0, y1):
        cols = heat[y0:y1].sum(axis=0)
        if cols.sum() == 0:
            return 0, W
        cdf = np.cumsum(cols) / cols.sum()
        return int(np.searchsorted(cdf, 0.05)), int(np.searchsorted(cdf, 0.95))
    tl, tr = x_extent(roi_top_row, roi_top_row + 20)
    bl, br = x_extent(H - 20, H)
    trap_top_w = round(min(0.95, (tr - tl) / W + 0.05), 2)
    trap_bot_w = round(min(1.0, (br - bl) / W + 0.05), 2)
    roi_top_frac = round(roi_top_row / H, 2)

    hm = cv2.applyColorMap((255 * heat / max(heat.max(), 1e-6)).astype(np.uint8),
                           cv2.COLORMAP_JET)
    trap = np.array([[bl, H - 1], [tl, roi_top_row], [tr, roi_top_row], [br, H - 1]], np.int32)
    cv2.polylines(hm, [trap], True, (255, 255, 255), 1)
    cv2.imwrite(out, cv2.resize(hm, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST))
    return {'roi_top_frac': roi_top_frac, 'trap_top_w': trap_top_w, 'trap_bot_w': trap_bot_w}


def lane_width(clips, k):
    """Median detected lane width (fraction of W) via the real extraction (G5)."""
    cfg = make_cfg('G5')
    widths = []
    for path in clips:
        pipe = LanePipeline(cfg)
        for i, frame in enumerate(cm.iter_frames(path)):
            _, _, dbg = pipe.process(frame, debug=True)
            det = dbg['det']
            if det['left_conf'] > 0 and det['right_conf'] > 0:
                widths.append(det['lane_width'] / frame.shape[1])
    return round(float(np.median(widths)), 3) if widths else float('nan')


def mask_samples(bgrs, wb, yb, out, n=4):
    step = max(1, len(bgrs) // n)
    rows = []
    for bgr in bgrs[::step][:n]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        if wb:
            wm = ((s <= wb['white_s_max']) & (v >= wb['white_v_min'])).astype(np.uint8) * 255
        else:
            wm = np.zeros((H, W), np.uint8)
        if yb:
            ym = ((h >= yb['yellow_h_lo']) & (h <= yb['yellow_h_hi'])
                  & (s >= yb['yellow_s_min']) & (v >= yb['yellow_v_min'])).astype(np.uint8) * 255
        else:
            ym = np.zeros((H, W), np.uint8)
        o = cm.label(bgr.copy(), 'orig')
        wimg = cm.label(cv2.cvtColor(wm, cv2.COLOR_GRAY2BGR), 'white')
        yimg = cm.label(cv2.cvtColor(ym, cv2.COLOR_GRAY2BGR), 'yellow')
        rows.append(np.hstack([o, wimg, yimg]))
    if rows:
        cv2.imwrite(out, cv2.resize(np.vstack(rows), (W * 3 * 2, len(rows) * H * 2),
                                    interpolation=cv2.INTER_NEAREST))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--frames', type=int, default=8, help='sampled frames/clip')
    ap.add_argument('--outdir', default='rslt')
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    bgrs, hsvs = sample_hsv(args.clips, args.frames)
    wb = white_band(hsvs)
    yb = yellow_band(hsvs)
    roi = roi_analysis(hsvs, wb, yb, os.path.join(args.outdir, 'track_analyze_heat.png'))
    lw = lane_width(args.clips, args.frames)
    mask_samples(bgrs, wb, yb, os.path.join(args.outdir, 'track_analyze_masks.png'))

    print(f'# track_analyze: {len(args.clips)} clips, {len(hsvs)} sampled frames\n')
    if wb:
        print(f'white : S[p50={wb["S_p50"]:.0f} p95={wb["S_p95"]:.0f}] '
              f'V[p5={wb["V_p5"]:.0f} p50={wb["V_p50"]:.0f}]  '
              f'-> white_s_max={wb["white_s_max"]} white_v_min={wb["white_v_min"]}')
    if yb:
        print(f'yellow: H[{yb["H_p5"]:.0f}..{yb["H_p95"]:.0f}] S[p5={yb["S_p5"]:.0f}] '
              f'V[p5={yb["V_p5"]:.0f}]  -> yellow_h_lo={yb["yellow_h_lo"]} '
              f'yellow_h_hi={yb["yellow_h_hi"]} yellow_s_min={yb["yellow_s_min"]} '
              f'yellow_v_min={yb["yellow_v_min"]}')
    print(f'ROI   : roi_top_frac={roi["roi_top_frac"]} trap_top_w={roi["trap_top_w"]} '
          f'trap_bot_w={roi["trap_bot_w"]}')
    print('        (roi_top_frac = 신호 시작 행; 근접·지평선없는 영상에선 낮게 나옴 → '
          'heatmap PNG로 확인. 차량 카메라(지평선 有)에선 자동값이 적절.)')
    print(f'lane  : lane_width_default={lw}')
    print('\n# suggested Cfg overrides (seed for G1..G6; verify with perception_preview):')
    sug = {}
    if wb:
        sug.update({'white_s_max': wb['white_s_max'], 'white_v_min': wb['white_v_min']})
    if yb:
        sug.update({k: yb[k] for k in ('yellow_h_lo', 'yellow_h_hi', 'yellow_s_min', 'yellow_v_min')})
    sug.update(roi)
    if lw == lw:
        sug['lane_width_default'] = lw
    print('  ' + '\n  '.join(f'{k}: {v}' for k, v in sug.items()))
    print(f'\nheatmap -> {os.path.join(args.outdir, "track_analyze_heat.png")}')
    print(f'masks   -> {os.path.join(args.outdir, "track_analyze_masks.png")}')


if __name__ == '__main__':
    main()
