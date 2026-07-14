"""Pure (ROS-independent) lane-detection pipeline. Single source of truth.

colour -> calibrated BEV -> sliding-window multi-lane tracking -> 7-label
classification -> ego corridor centerline (+ Tracker/coast).

Stage order, and why (`LanePipeline.process` marks the seam):

  colour   on the RAW frame, cropped to `cam.src_rows` -- the band the BEV actually
           reads. A per-pixel colour test is cheaper and sharper before any resample,
           and rows above that band feed no BEV cell at all.
  warp     the two 1-channel masks (not the 3-channel frame).
  shape    morphology, area gates, window sizes, lane width, pairing, coast, heading --
           ALL of it after the warp, where a pixel IS a centimetre and a threshold means
           the same thing at 26cm and at 78cm. `cfg_to_px` resolves the cm config to BEV
           px once; every stage below it keeps taking plain pixels.

A CameraModel is REQUIRED. The front-view path is gone -- it was a second pipeline, not a
fallback: every threshold here is in cm, and the physical gates the pipeline now leans on
(lane width, pairing, coast side) cannot even be ASKED in a perspective image. The BEV LUT
is itself a calibrated trapezoid crop, so the old hand-tuned `roi_*` one went with it.

Imported by BOTH the online perception node and the offline tools. Depends only on cv2 + numpy.

Usage:
    from dracer_core.perception_core import LanePipeline, Cfg, cfg_from_profile
    pipe = LanePipeline(cfg_from_profile(profile['perception']), cam)   # cam: REQUIRED
    state = pipe.process(frame_bgr, dt_s)                 # dict (center_error, ...)
    state, dbg = pipe.process(frame_bgr, dt_s, debug=True)   # dbg -> render_panels()

`process()` returns ONLY the lane state. Rendering is never on its hot path: build the
debug image with `render_panels(frame, dbg, cfg)`, and only when something is watching.

The pipeline NEVER commands the vehicle; it only produces a lane-state estimate
(consumed by the control stage).
"""
from collections import deque
from dataclasses import dataclass, fields, replace
import itertools
import math
import random

import cv2
import numpy as np

# Rendering only. The lane pipeline itself never touches mission_core -- the two run as
# independent detectors on the same frame, and `render_bev` is the one place they meet,
# because a human watching the car wants one picture, not two.
from .mission_core import annotate, CLASS_NAMES


# ==========================================================================
# Config + presets (single source; imported by online node + offline tools)
# ==========================================================================
@dataclass
class Cfg:
    name: str = 'lane'
    # A: segmentation — WHITE + YELLOW; `colors` is a subset, masks OR-combined.
    colors: tuple = ('white', 'yellow')
    white_s_max: int = 60
    white_v_min: int = 185
    yellow_h_lo: int = 18
    yellow_h_hi: int = 36
    yellow_s_min: int = 65
    yellow_v_min: int = 100
    morph_v: int = 5             # vertical CLOSE kernel (bridge dashed gaps)
    color_gate: float = 0.15     # minority-color fraction < this -> drop it wholesale
    gate_min_px: int = 80
    # C: sliding window (image coords)
    sw_nwin: int = 12            # vertical window count
    sw_margin: int = 28          # window half-width (px)
    sw_minpix: int = 18          # min pixels in a window to recenter
    sw_max_miss: int = 3         # consecutive empty windows -> lane ends
    sw_dir_ema: float = 0.6      # per-window lateral-step EMA (curve following)
    sw_min_hits: int = 2         # fewer successful windows -> not a lane
    sw_min_span: float = 0.30    # vertical span < this*H -> not a lane (stop line)
    sw_max_lanes: int = 3        # max base peaks per color
    sw_peak_min: int = 10        # min histogram peak (rows)
    sw_peak_sep: int = 45        # min peak separation (px)
    merge_dx: float = 30.0       # two fits with MAX |dx| over their y-overlap < this -> same lane
    # D: classification (per-instance heading + curvature)
    heading_frac: float = 0.06   # |top-bottom x| >= this*W -> turn by heading
    curv_strong: float = 0.0015  # near-vertical but |a| >= this -> turn by curvature
    straight_thresh: float = 0.0006
    # pairing (left/right boundary -> centerline)
    pair_overlap_min: float = 0.30
    pair_gap_min: float = 12.0
    pair_same_color: bool = True   # a white line and a yellow line are NOT the two boundaries
                                   # of one lane. On this track the main lane is bounded
                                   # white-white and the yellow shortcut yellow-yellow; a
                                   # white/yellow pair is two boundaries of DIFFERENT routes
                                   # that happen to be a lane width apart where they cross.
                                   # `pair_width_tol` cannot catch it -- both really are 35cm.
                                   # Measured: W-Y = 6.2% of all corridors, but 42.7% of the
                                   # corridors formed AT A BRANCH, which is exactly where a
                                   # wrong corridor costs a route.
    pair_parallel_cm: float = 8.0  # "am I IN this corridor", NOT "is this a corridor".
                                   # `spread` = max gap - min gap over the overlap: how far
                                   # from parallel the two boundaries are. A corridor whose
                                   # boundaries SPLAY is not a bad corridor -- it is a FORK,
                                   # a route peeling away from us, and we have not taken it.
                                   # Measured at the yellow branch: 21 of 21 two-corridor
                                   # frames give [A,B] + [B,C] SHARING a boundary (= a fork),
                                   # with the one we are in parallel (spread p50 3.2cm) and
                                   # the one we are not splaying (p50 32.7cm).
                                   #
                                   # So it is NOT a pairing gate. Gating on it deleted the
                                   # branch outright -- 12cm took the yellow fork from 52% of
                                   # its frames to 0%, and the judgment layer would have had
                                   # nothing to judge. `lane_centers` keeps every corridor;
                                   # `ego_center` only refuses to call a splaying one the lane
                                   # we are currently DRIVING DOWN.
                                   #
                                   # Same-colour p90 = 5.7cm, so 8cm admits the lane we are in
                                   # and refuses the fork. <= 0 disables.
    pair_parallel: float = 32.0  # DERIVED from pair_parallel_cm by cfg_to_px. Do not set.
    # branch policy — PLACEHOLDER for the judgment layer that does not exist yet
    branch_policy: str = 'keep'  # what to do when >1 corridor is physically available:
                                 #   keep   - carry on with the corridor the Tracker already
                                 #            has (measured: this is what happens today, 98%
                                 #            of branch frames). The car can NEVER take the
                                 #            yellow shortcut. Current behaviour; default.
                                 #   random - pick one at random. A PLACEHOLDER, for seeing
                                 #            what a route change does. Seeded (branch_seed)
                                 #            so a replay is reproducible.
                                 #   nearest- the corridor closest to the vehicle axis, i.e.
                                 #            by proximity, i.e. by accident.
                                 # Whatever picks, it is LATCHED for the length of the branch
                                 # and pushed into the Tracker (adopt). A per-frame choice
                                 # oscillates by a full lane width -- measured, not feared:
                                 # that is exactly what a per-frame `coast_side` flip did
                                 # (§8+), 0 oscillations -> 3.
    branch_seed: int = 0         # branch_policy='random' seed. Replays must be reproducible.
    ego_tol: float = 0.6         # a corridor is the EGO corridor if its centreline sits within
                                 # this many lane widths of the vehicle axis. 0.5 = "the axis is
                                 # strictly inside it"; 0.6 leaves slack for a car running wide.
                                 # Replaces the straddle test `a.x_bottom < cx <= b.x_bottom`,
                                 # which is a screen-space question, not a physical one.
                                 #
                                 # Sized to MEAN something. 0.75 x a 39cm tracked width is 29cm,
                                 # and the BEV half-width is 29cm -- i.e. every corridor in the
                                 # frame passes and the gate is decorative. 0.6 x 39 = 23cm, so
                                 # the corridor one lane over (centre ~35cm away) is refused.
                                 # Measured identical to 0.5/0.75 on the 0711 runs (the nearest-
                                 # corridor pick and `adopt` already get it right); this is for
                                 # the 4-lane roundabout, where several corridors are visible and
                                 # picking the wrong one is a lane change nobody asked for.
    # coast side check (falsify the coast's left/right against the mask; `coast_side`)
    coast_flip_support: float = 0.15   # the MIRROR assumption needs at least this fraction of
                                       # its rows backed by mask pixels before we believe it
                                       # over the tracker. <= 0 disables the check entirely.
                                       #
                                       # Measured over the 20 coast->pair transitions in the
                                       # 0711 runs (the pair that ends a coast reveals that
                                       # coast's true error):
                                       #   mirror support > 0 in 0 of the 16 coasts that were
                                       #     RIGHT (< 15cm error)  -> zero false positives
                                       #   mirror support > 0 in 2 of the 4 coasts that were
                                       #     WRONG (18cm and 29cm out)
                                       # 0.15 sits above the noise and below both real hits
                                       # (0.18, 0.27). This is a LEFT/RIGHT check, not a
                                       # general coast validator -- one 15cm error had a
                                       # well-supported phantom and is a width bug, not a
                                       # side bug.
    coast_flip_empty: float = 0.05     # ...AND our own phantom must be this unsupported. The
                                       # flip is only allowed in the unambiguous case: we are
                                       # pointing at bare tarmac and there is tape opposite.
                                       # Without this, `s_alt > s_now` flips on 0.35-vs-0.30,
                                       # a coin toss dressed as evidence -- and a false flip
                                       # is not a bad frame, it is a bad TRACKER, because
                                       # `reseat_coast` persists it until a pair rescues us.
    # E: tracker (ego L/R persistence + width coast)
    lane_width_default: float = 0.5
    jump_max: float = 120.0
    lost_reset_s: float = 0.26      # drop the tracked identity after this long with no lane
                                    # (= 8 frames @30Hz, the rate it was tuned at)
    track_width_tol: float = 0.25   # Tracker: accept a width MEASUREMENT only within this
                                    # fraction of lane_width_cm. <= 0 accepts anything.
                                    # Deliberately NOT the same knob as `pair_width_tol`
                                    # (which gates lane_centers' pairing): turning one off
                                    # must not turn the other off.
    # scalar output stabilizer (smooths center_error + names the failsafe state)
    #
    # TIME CONSTANTS, not per-frame weights. An EMA weight of 0.4 "per frame" makes the
    # filter's real memory a function of the day's FPS: the same 0.4 smooths over 0.16s at
    # 30Hz and 0.47s at 10Hz. Perception went 10.7 -> 30Hz and every one of these silently
    # became 3x shorter. `tau` is the physical memory; the per-frame weight is DERIVED from
    # it and the measured dt (`_ema_alpha`), so the filter behaves the same at any rate.
    ema_tau_s: float = 0.065     # = alpha 0.4 @30Hz. Shared by the lane-coeff EMA, the
                                 # corridor-width EMA and the center_error EMA -- all three
                                 # ran at 0.4 before, so one tau reproduces all three.
    outlier_jump: float = 0.5
    outlier_relatch_s: float = 0.16  # rejections held this long -> believe the new value,
                                 # re-seed. Without it `outlier_jump` latches forever (see
                                 # _Stabilizer). It bounds how long the car steers on a stale
                                 # (and, after a corridor flip, WRONG-SIGNED) value -- which
                                 # is a duration, so a duration is what it must be expressed
                                 # as. Was a 5-FRAME count that meant 0.17s @30Hz and 0.25s
                                 # @20Hz -- longest exactly when perception was worst.
                                 #
                                 # 5 frames @30Hz, measured on the 0711 raw (relatch 6->5->4):
                                 #   145617  OUTLIER 3.5% -> 2.8%,  max EMA freeze 5 -> 4 fr,
                                 #           |ema - truth| mean .063 -> .058, p95 .145 -> .137
                                 #   145515  identical (its bursts top out at 3 fr, below both)
                                 #   4 crosses the line: it relatches on one of 145515's
                                 #   3-frame transients -- believing a disagreement that was
                                 #   about to resolve itself. That is the spike defence going.
    use_median: bool = False
    median_window: int = 5       # SAMPLES, not time. A median needs a sample count; leaving
                                 # it in frames is honest (and `use_median` is off).
    conf_low: float = 0.25
    lost_stop_s: float = 0.26    # no usable measurement for this long -> LOST (= 8fr @30Hz)

    # ==================================================================
    # METRIC (BEV) parameters — cm, used ONLY when a CameraModel is given.
    # ------------------------------------------------------------------
    # In a calibrated BEV a pixel IS a physical length, so every geometric
    # threshold above becomes a real distance instead of a screen fraction that
    # silently changes meaning with resolution, camera angle, or track. These cm
    # values are resolved to px ONCE by `cfg_to_px()`; the stages below it keep
    # taking plain pixels and are unchanged.
    lane_width_cm: float = 35.0      # TRACK: centre-to-centre of the two lane tapes.
                                     # (2025: inner 32 / outer 38, tape 3 -> centres 35.
                                     #  The fits follow tape CENTRES, so it is neither 32 nor 38.)
    sw_margin_cm: float = 5.0        # window half-width: lane-position slack + curve step
    sw_peak_sep_cm: float = 15.0     # two base peaks closer than this = one lane (< lane width)
    merge_dx_cm: float = 5.0         # two fits within this everywhere = the same tape
    pair_gap_min_cm: float = 8.0     # below this the pair has collapsed = a crossing, reject
    jump_max_cm: float = 15.0        # tracker: max lane jump between frames
    pair_width_tol: float = 0.25     # PHYSICAL pairing gate: |gap - lane_width| <= this*width.
                                     # Impossible in the front view (gap varies with y); it is
                                     # what stops a white/yellow or cross-corridor mismatch.
    heading_cm: float = 5.0          # |top-bottom x| >= this -> turn by heading.
                                     # In the BEV this is a REAL sideways drift; in the front
                                     # view perspective convergence faked it (limitation §6a).

    # Thresholds that count PIXELS rather than measuring a distance. They were the last
    # scale trap left in the config: a pixel COUNT scales with px_per_cm SQUARED (an area),
    # so halving the BEV scale quarters every count while the threshold stays put, and
    # detection dies with nothing in the log. Expressed as areas/lengths they are immune.
    # Defaults reproduce the old px values EXACTLY at px_per_cm = 4.
    morph_cm: float = 1.25           # vertical CLOSE kernel  (was morph_v = 5 px)
    sw_minpix_cm2: float = 1.125     # min lane area in a window (was sw_minpix = 18 px)
    sw_peak_min_cm: float = 2.5      # min histogram peak, a LENGTH (was sw_peak_min = 10 px)
    gate_min_cm2: float = 5.0        # colour-gate floor, an area (was gate_min_px = 80 px)


_FIELDS = {f.name for f in fields(Cfg)}


def cfg_from_profile(section=None):
    """Build the single perception Cfg from a profile [perception] dict. Unknown
    keys (e.g. a leftover `mode:` or a param no longer modelled) are ignored;
    `colors` list -> tuple. One confirmed pipeline — no experiment presets."""
    ov = {}
    for k, v in (section or {}).items():
        if v is None or k not in _FIELDS:
            continue
        ov[k] = tuple(v) if (k == 'colors' and isinstance(v, list)) else v
    return replace(Cfg(), **ov) if ov else Cfg()


# Cfg fields that `cfg_to_px` COMPUTES from the cm parameters. They are outputs, not
# inputs: whatever you put in them is overwritten before a single frame is processed.
#
# This list exists so `perception_node` can refuse to declare them as ROS parameters. It
# used to declare every field, so `ros2 param set /perception_node sw_margin 40` would
# report success, log a live-update, rebuild the pipeline (throwing away the Tracker's
# state), and change NOTHING -- because cfg_to_px overwrote it on the way in. They look
# exactly like the knobs you would reach for when detection misbehaves trackside. The real
# knobs are the `_cm` twins: sw_margin_cm, jump_max_cm, merge_dx_cm, ...
DERIVED_PX = frozenset({
    'sw_margin', 'sw_peak_sep', 'merge_dx', 'pair_gap_min', 'jump_max', 'pair_parallel',
    'morph_v', 'sw_minpix', 'sw_peak_min', 'gate_min_px',
    'lane_width_default', 'heading_frac',
})


def cfg_to_px(cfg, cam):
    """Resolve the metric (cm) params into BEV pixels — ONCE, here.

    This is the whole trick that keeps the BEV port small: the detector, the sliding
    window, the pairing and the tracker keep speaking plain pixels and are untouched;
    only their THRESHOLDS change meaning, and that mapping lives in one place.

    Every field it writes is in `DERIVED_PX`.
    """
    if cam is None:
        raise ValueError(
            'cfg_to_px: CameraModel 이 필요하다. front-view 경로는 삭제됐다 — '
            '모든 문턱값이 cm 단위이고 물리 게이트(차선폭·페어링·coast)는 metric BEV '
            '에서만 성립한다. camera.yaml 을 넘겨라.')
    s = cam.px_per_cm
    bev_w = cam.bev_size[0]
    lane_px = cfg.lane_width_cm * s
    return replace(
        cfg,
        sw_margin=max(4, int(round(cfg.sw_margin_cm * s))),
        sw_peak_sep=max(4, int(round(cfg.sw_peak_sep_cm * s))),
        merge_dx=cfg.merge_dx_cm * s,
        pair_gap_min=cfg.pair_gap_min_cm * s,
        pair_parallel=cfg.pair_parallel_cm * s,
        jump_max=cfg.jump_max_cm * s,
        # Pixel COUNTS -> areas (s^2); the histogram peak is a length (s).
        morph_v=max(3, int(round(cfg.morph_cm * s))),
        sw_minpix=max(4, int(round(cfg.sw_minpix_cm2 * s * s))),
        sw_peak_min=max(3, int(round(cfg.sw_peak_min_cm * s))),
        gate_min_px=max(8, int(round(cfg.gate_min_cm2 * s * s))),
        # Tracker reads `lane_width_default * W` as its fallback width, so express the
        # REAL lane width as that fraction. The old 0.5 was a guess with no physical
        # meaning — it is what made every coast produce |center_error| ~= 0.5 (§6c).
        lane_width_default=lane_px / bev_w,
        # In the BEV, heading is a true lateral drift over the lane's span (no perspective
        # convergence to fake it), so express the threshold as a real distance.
        heading_frac=(cfg.heading_cm * s) / bev_w,
    )


def _ema_alpha(dt_s, tau_s):
    """Per-frame EMA weight from a TIME constant and the measured frame interval.

    `alpha = 1 - exp(-dt/tau)` is the discrete sampling of a first-order lag with memory
    `tau`. The point is that the filter's behaviour stops depending on the frame rate:

      dt = 0      -> 0    no time passed, so the filter does not move (a re-seed, not a blend)
      dt = tau    -> 0.63 one time constant
      dt >> tau   -> 1    the stored value is older than the filter's memory. Take the
                          measurement. After a long dropout that is the ONLY honest answer --
                          blending against a value from a second ago is worse than forgetting.

    A weight of 0.4 @30Hz is tau = -(1/30) / ln(0.6) = 0.065s, which is where the default
    comes from: the same filter the successful drive ran, now expressed in seconds.
    """
    if tau_s <= 0.0:
        return 1.0                       # no smoothing: always take the measurement
    if dt_s <= 0.0:
        return 0.0                       # no time passed: hold
    return 1.0 - math.exp(-dt_s / tau_s)


LABEL_COLORS = {  # BGR
    'W-L': (255, 255, 255), 'W-R': (170, 170, 170),   # white / gray
    'YR-L': (0, 140, 255), 'YR-R': (0, 90, 200),      # orange (right turn)
    'YL-L': (255, 0, 200), 'YL-R': (200, 0, 150),     # magenta (left turn)
    'YS-L': (0, 230, 0), 'YS-R': (0, 150, 0),         # green (straight)
}
EGO_CENTER_COLOR = (255, 255, 0)   # cyan — ego corridor centerline (control value)
# A corridor's boundaries are always ONE colour (`pair_same_color`), and that colour is the
# ROUTE'S IDENTITY: the main line and the sign island are bounded white-white, the roundabout
# yellow-yellow. So colouring a corridor by its boundaries makes the branch TYPE visible --
# {W,Y} = main-vs-roundabout, {Y,Y} = the roundabout junction, {W,W} = the sign island.
CORRIDOR_COLORS = {'W': (170, 170, 170), 'Y': (0, 170, 255)}


# ==========================================================================
# A: detection (HSV white/yellow masks + morphology + color-dominance gate)
# ==========================================================================
def color_masks(frame, c, hsv=None):
    """Per-pixel HSV colour test. No morphology, no gate -- those are SHAPE and AREA
    judgements, and in the front view neither has a stable meaning (a 5px kernel spans
    1cm of tarmac up close and 5cm at the far edge). They belong after the warp.

    `hsv`, if given, is the already-converted HSV of `frame` (the merged node computes it
    once for the whole frame and the mission detectors share it). A per-pixel conversion of
    a slice is bit-identical to the slice of a conversion, so this changes nothing except
    how many times the frame gets walked.
    """
    if hsv is None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    white = ((S <= c.white_s_max) & (V >= c.white_v_min)).astype(np.uint8) * 255
    yellow = ((H >= c.yellow_h_lo) & (H <= c.yellow_h_hi) &
              (S >= c.yellow_s_min) & (V >= c.yellow_v_min)).astype(np.uint8) * 255
    if 'white' not in c.colors:
        white[:] = 0
    if 'yellow' not in c.colors:
        yellow[:] = 0
    return white, yellow


def morph_gate(white, yellow, c):
    """Vertical CLOSE + colour-dominance gate.

    Run on the BEV, `morph_v` is a REAL length (cfg.morph_cm) that means the same thing
    at 26cm and at 78cm, and it also heals the row-replication the nearest-neighbour warp
    leaves behind. `gate_min_px` likewise becomes a real area.
    """
    kv = cv2.getStructuringElement(cv2.MORPH_RECT, (3, max(3, c.morph_v)))
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kv)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kv)
    wc, yc = int(cv2.countNonZero(white)), int(cv2.countNonZero(yellow))
    tot = wc + yc
    if tot >= c.gate_min_px:
        if wc / tot < c.color_gate:
            white[:] = 0
        if yc / tot < c.color_gate:
            yellow[:] = 0
    return white, yellow


def detect(frame, c):
    """Colour, then shape, on a RAW frame. Not on the driving path -- the pipeline runs
    `color_masks` before the warp and `morph_gate` after it, because a morphology kernel
    only means one thing in a metric BEV. Kept for `offline/calibrate.py`, which needs
    lane pixels out of a raw ground photo before any BEV exists."""
    return morph_gate(*color_masks(frame, c), c)


# The hand-tuned ROI trapezoid (`roi_top_frac`, `trap_*`) and the whole front-view path
# are GONE. The BEV LUT is itself a calibrated trapezoid crop of the ground, so the ROI
# was a crop on top of a crop -- and a hand-guessed one shaving 3.7% off the calibrated
# one. `cam.src_rows` (the row band the LUT actually samples) replaced it: derived from
# the calibration, so it cannot disagree with the BEV by construction.


# ==========================================================================
# feature build (2nd-order fit -> base x, trend x, turn)
# ==========================================================================
def _build_instance(ys, xs, color, c, W):
    ins = {'color': color, 'xs': xs, 'ys': ys, 'coeffs': None,
           'x_bottom': float(xs[np.argmax(ys)]),
           'x_mean': float(xs.mean()), 'turn': 0}
    if np.unique(ys).size >= 5:
        a, b, cc = np.polyfit(ys.astype(float), xs.astype(float), 2)
        yb, yt = float(ys.max()), float(ys.min())
        ins['coeffs'] = (a, b, cc)
        ins['x_bottom'] = float(a * yb * yb + b * yb + cc)
        hd = (a * yt * yt + b * yt + cc) - ins['x_bottom']
        if abs(hd) >= c.heading_frac * W:
            ins['turn'] = 1 if hd > 0 else -1
        elif abs(a) >= c.curv_strong:
            ins['turn'] = 1 if a > 0 else -1
    return ins


# ==========================================================================
# C: sliding-window separation
# ==========================================================================
def _find_peaks(hist, c):
    h = hist.astype(float).copy()
    peaks = []
    for _ in range(c.sw_max_lanes):
        i = int(h.argmax())
        if h[i] < c.sw_peak_min:
            break
        peaks.append(i)
        h[max(0, i - c.sw_peak_sep):i + c.sw_peak_sep] = 0
    return sorted(peaks)


def _same_lane(a, b, c):
    """Two instances are the SAME physical lane only if their 2nd-order fits stay
    close over the WHOLE shared y-range (MAX |dx| < merge_dx). Uses max, not mean,
    so a pair that coincides at one end but SEPARATES at the other is NOT merged:
      - Y-branch: arms share a stem (bottom) but diverge toward the top,
      - perspective-converging pair: apart near the car, converge at the vanishing
        point up top.
    A thick/diagonal line split into parallel stacks stays close everywhere -> max
    is small -> merged (which window-overlap merging misses, since diagonal stacks
    never share a window column). Under-merging (staying two instances) is the safe
    failure; wrongly merging a real branch is not."""
    if a['coeffs'] is None or b['coeffs'] is None:
        return False
    ylo = max(float(a['ys'].min()), float(b['ys'].min()))
    yhi = min(float(a['ys'].max()), float(b['ys'].max()))
    if yhi - ylo < 1.0:                      # no shared y-range -> cannot judge
        return False
    yy = np.linspace(ylo, yhi, 7)
    dx = float(np.abs(_ebottom(a['coeffs'], yy) - _ebottom(b['coeffs'], yy)).max())
    return dx < c.merge_dx


def sliding_window_lanes(mask, color, c, windows_out=None):
    """ROI-cropped mask -> lane instances. Base peaks -> stack windows upward,
    following curvature -> drop vertically-short stacks -> merge duplicates."""
    H, W = mask.shape
    hist = (mask[H // 2:] > 0).sum(axis=0)
    win_h = max(1, H // c.sw_nwin)
    raw = []
    for base in _find_peaks(hist, c):
        cur = float(base)
        step = 0.0
        prev_cx = None
        xs_all, ys_all, miss, hits = [], [], 0, 0
        for i in range(c.sw_nwin):
            ci = int(round(cur))
            ylo, yhi = H - (i + 1) * win_h, H - i * win_h
            xlo, xhi = max(0, ci - c.sw_margin), min(W, ci + c.sw_margin)
            if windows_out is not None:
                windows_out.append((xlo, ylo, xhi, yhi))
            wy, wx = np.nonzero(mask[ylo:yhi, xlo:xhi] > 0)
            if wx.size > c.sw_minpix:
                mx = float(wx.mean()) + xlo
                xs_all.append(wx + xlo)
                ys_all.append(wy + ylo)
                if prev_cx is not None:
                    obs = max(-c.sw_margin, min(c.sw_margin, mx - prev_cx))
                    step = c.sw_dir_ema * obs + (1 - c.sw_dir_ema) * step
                prev_cx = mx
                cur = mx + step
                miss = 0
                hits += 1
            else:
                cur += step
                # Only a gap BETWEEN hits is a miss. Empty windows BEFORE the first hit are
                # the climb up to the lane, not the end of it, and `sw_max_miss` must not
                # spend itself on them: the base peak has already proven there are pixels in
                # this column: they are simply not at the very bottom of the BEV.
                #
                # They routinely are not. The near BEV rows go blind whenever the lane leaves
                # the camera's lateral FOV -- on a curve, or with the car merely offset -- and
                # the BEV floor (26cm) sits below where this lens can see a lane at all (both
                # boundaries only appear from ~30cm, and the near field is asymmetric:
                # -19.7..+2.0cm at 26cm). Counting that climb as misses killed EVERY stack
                # three windows in, and the whole frame came out as 0 lanes with a perfectly
                # good mask sitting right there in the debug panel.
                if hits:
                    miss += 1
                    if miss >= c.sw_max_miss:
                        break
            cur = min(max(cur, 0.0), float(W - 1))
        if not xs_all:
            continue
        xs, ys = np.concatenate(xs_all), np.concatenate(ys_all)
        y_span = float(ys.max() - ys.min())
        if hits < c.sw_min_hits or y_span < c.sw_min_span * H:
            continue
        if np.unique(ys).size >= 5 and xs.size >= c.sw_minpix:
            raw.append({'xs': xs, 'ys': ys, 'npix': int(xs.size), 'W': W})
    # strongest (most pixels) first, then drop near-duplicate fits (a thick/diagonal
    # line split into parallel stacks). Merge by polynomial proximity, not window
    # overlap: diagonal splits never share window columns but their fits coincide.
    raw.sort(key=lambda r: -r['npix'])
    kept = []
    for r in raw:
        ins = _build_instance(r['ys'], r['xs'], color, c, r['W'])
        if ins['coeffs'] is not None and any(_same_lane(ins, k, c) for k in kept):
            continue
        kept.append(ins)
    return kept


# ==========================================================================
# D: classification (7 labels)
# ==========================================================================
def _side(ins, w):
    cx = w / 2.0
    pos = -1 if ins['x_bottom'] < cx else 1
    dirv = -1 if ins['x_mean'] < cx else 1
    vote = pos + dirv
    if vote != 0:
        return 'L' if vote < 0 else 'R'
    return 'L' if ins['x_bottom'] < cx else 'R'


def classify(ins, w):
    side = _side(ins, w)
    if ins['color'] == 'W':
        return f'W-{side}'
    tw = 'S' if ins['turn'] == 0 else ('R' if ins['turn'] > 0 else 'L')
    return f'Y{tw}-{side}'


# ==========================================================================
# centerline (adjacent parallel boundary pairs -> ego corridor center)
# ==========================================================================
def _shift(coeffs, dx):
    a, b, c = coeffs
    return (a, b, c + dx)


def _ebottom(coeffs, yb):
    a, b, c = coeffs
    return a * yb * yb + b * yb + c


def _pair_gate(a, b, h, c, lane_w_px=0.0):
    """Can these two fits be the left/right boundary of ONE corridor?

    `lane_w_px` > 0 (BEV only) adds the PHYSICAL gate: the gap must actually equal a
    lane width. In the front view the gap shrinks with y, so no such test exists and
    `pair_gap_min` degenerates into "> 0" — which is why a white line could pair with a
    yellow one across the wrong corridor (§6b). In a metric BEV the gap is constant, so
    "is this really a lane width apart?" is a question we can finally ask.
    """
    # Same COLOUR. Different tapes bound different routes (see Cfg.pair_same_color).
    if c.pair_same_color and a['color'] != b['color']:
        return None
    ylo = max(int(a['ys'].min()), int(b['ys'].min()))
    yhi = min(int(a['ys'].max()), int(b['ys'].max()))
    if yhi - ylo < c.pair_overlap_min * h:
        return None
    yy = np.linspace(ylo, yhi, 7)
    gaps = _ebottom(b['coeffs'], yy) - _ebottom(a['coeffs'], yy)
    if float(gaps.min()) < c.pair_gap_min:
        return None
    if lane_w_px > 0:
        if abs(float(gaps.mean()) - lane_w_px) > c.pair_width_tol * lane_w_px:
            return None                      # not a lane width apart -> not one corridor
    # `spread` = how far from PARALLEL the two boundaries are (max gap - min gap over the
    # overlap). Reported, NOT gated on -- and that distinction is the whole point.
    #
    # A splaying pair is not a bad corridor. It is a FORK. Measured at the yellow branch
    # (frames with 3+ yellow lanes): 21 of 21 frames that produce two corridors produce them
    # as [A,B] and [B,C] -- SHARING a boundary, which is what a fork IS. And the second one
    # always splays (spread p50 = 32.7cm) while the first is parallel (p50 = 3.2cm):
    #
    #     corridor A   spread  3.2cm, width 34cm   <- the lane we are IN
    #     corridor B   spread 32.7cm, width 28cm   <- the route we have NOT taken yet
    #
    # Of course B splays: it is peeling away from us. Gating it out deletes the branch --
    # measured, a 12cm parallel gate took the yellow fork from 52% of its frames to 0%. The
    # judgment layer would have had nothing to judge.
    #
    # So parallelism is not "is this a corridor", it is "am I IN this corridor". `ego_center`
    # uses it for that; `n_corridors` counts them all.
    return ylo, yhi, float(gaps.max() - gaps.min())


def lane_centers(lanes, w, h, c, lane_w_px=0.0, axis=None):
    """Every physically plausible corridor in the frame. Which one is OURS is `ego_center`'s
    job, not this function's.

    Two gates dropped here, both screen-space leftovers:

    `_side(a) == _side(b) -> skip` required one boundary left of the image centre and the
    other right of it. Run wide, or take a curve, and BOTH boundaries of the corridor you are
    actually in land on one side -- so the corridor you are IN got thrown away and perception
    fell back to a single-lane coast while looking straight at both of its boundaries.

    `ego = a.x_bottom < cx <= b.x_bottom` asked the same screen-space question again. Note it
    compared two `x_bottom` values that are not even evaluated at the same row (each is at its
    own instance's lowest observed row), against a centre column that is only the vehicle axis
    by coincidence.

    What is left is physics: `_pair_gate` already demands the pair be a real lane width apart
    (`pair_width_tol`), which is a question only a metric BEV can ask -- and it is a far
    stronger test than "on opposite sides of the screen" ever was.

    ALL PAIRS, not adjacent ones. `zip(ls, ls[1:])` only ever offered `_pair_gate` neighbours
    in x-order, so ONE stray instance between the two real boundaries hid the corridor
    completely -- neither half of it is a lane width, and the real pair was never even a
    candidate. That is not a rare accident, it is the roundabout: the yellow line runs 5-10cm
    inside the white lane edge, and white and yellow are detected by SEPARATE sliding-window
    passes that never merge across colour (nor should they -- measured, 88% of those close
    cross-colour pairs are genuinely different tapes). So the frame reads [white, yellow,
    white] and adjacent pairing offers (white,yellow)=5cm and (yellow,white)=30cm. The real
    (white,white)=35cm corridor is invisible.

    Measured over the 0711 dashcam clips, on the 323 frames that see 3+ lanes:
        adjacent pairs -> 2+ corridors in   0 frames
        all pairs      -> 2+ corridors in 158 frames
    And 2+ corridors is the whole premise of a branch decision: you cannot choose between
    routes you cannot see. `_pair_gate` still has to pass, so this widens the candidate set,
    not the acceptance criteria. With <=6 instances (3 per colour) it is <=15 gate calls.
    """
    cx = w / 2.0 if axis is None else float(axis)
    ls = sorted([x for x in lanes if x['coeffs'] is not None],
                key=lambda x: x['x_bottom'])
    out = []
    for a, b in itertools.combinations(ls, 2):
        ov = _pair_gate(a, b, h, c, lane_w_px)
        if ov is None:
            continue
        ylo, yhi, spread = ov
        coeffs = tuple((p + q) / 2.0 for p, q in zip(a['coeffs'], b['coeffs']))
        x_bottom = float(_ebottom(coeffs, yhi))
        out.append({'coeffs': coeffs, 'x_bottom': x_bottom, 'offset': x_bottom - cx,
                    'ego': False, 'rule': None, 'spread': spread,
                    'a': a, 'b': b, 'y_lo': ylo, 'y_hi': yhi})
    return out


def _support(cs, coeffs, ys, margin):
    """Fraction of the rows of `ys` where the BEV mask actually HAS pixels within `margin`
    of this polynomial. `cs` is the mask's row-wise cumulative sum.

    Asks the only question that can falsify a coast: is there really a lane where we are
    claiming one is?
    """
    h, w = cs.shape
    vs = np.arange(max(0, int(ys.min())), min(h, int(ys.max()) + 1))
    if vs.size == 0:
        return 0.0
    xs = _ebottom(coeffs, vs.astype(float))
    lo = np.clip(np.floor(xs - margin).astype(int), 0, w - 1)
    hi = np.clip(np.ceil(xs + margin).astype(int), 0, w - 1)
    # rows whose window lies entirely outside the BEV carry no evidence either way
    inside = (xs + margin >= 0) & (xs - margin <= w - 1)
    got = (cs[vs, hi] - cs[vs, lo]) > 0
    return float(np.count_nonzero(got & inside) / vs.size)


def coast_side(near, dx, mask, c, margin, cx):
    """Is the boundary we are coasting off really on the side we think it is?

    A coast takes ONE boundary and asserts the corridor lies to its left or its right. The
    assertion is the tracker's identity, and when that identity is wrong the corridor centre
    lands a full lane width from the truth -- with a perfect lane fit, a 0.98 span and a
    1.3cm residual. Nothing about the GEOMETRY of a wrong coast looks wrong (a continuous
    quality score was measured and rejected: corr(quality, error) = +0.246, the WRONG sign);
    the fit is immaculate, it is simply pointed the wrong way.

    So ask the one question geometry cannot: PUT THE PHANTOM BOUNDARY WHERE WE CLAIM IT IS,
    AND LOOK. If the far boundary we are synthesising has no mask under it, but the mirror
    assumption DOES, we chose the wrong side.

    Measured over the 20 coast->pair transitions in the 0711 runs (where the pair that ends
    the coast gives us the coast's true error):

        opposite-side support > 0  in  0 of the 16 coasts that were right (< 15cm error)
        opposite-side support > 0  in  2 of the 4  coasts that were wrong (18cm, 29cm)

    Zero false positives, and it catches half the real failures. The other half are not
    side errors at all (one had a well-supported phantom and was still 15cm out -- a width
    error, a different bug), so this is not a general coast-validator; it is specifically a
    LEFT/RIGHT check, and that is the failure that inverts the steering command.

    Returns `dx`, flipped if the evidence says so.
    """
    if mask is None or near.get('coeffs') is None or c.coast_flip_support <= 0:
        return dx, False
    # Row-wise cumulative sum -> "are there pixels within +-margin of this curve?" becomes
    # one subtraction per row, for any curve. Built HERE, so it costs nothing on the ~50% of
    # frames that see a real pair and never coast at all.
    cs = np.cumsum(mask > 0, axis=1, dtype=np.int32)
    ys = near['ys']
    s_now = _support(cs, _shift(near['coeffs'], 2.0 * dx), ys, margin)
    s_alt = _support(cs, _shift(near['coeffs'], -2.0 * dx), ys, margin)
    # The discriminator is `s_alt`. `s_now == 0` is the NORMAL state of a coast -- measured
    # over the 0711 runs, the chosen phantom has support p50 = 0.000, because the whole reason
    # we are coasting is that the far boundary is not visible (often it is not even inside the
    # BEV: 145617's phantom sat at +43cm against a 29cm half-width). So `s_now <= empty` is a
    # sanity condition, not the signal. The SIGNAL is that the MIRROR has tape under it, and
    # that is rare: mirror support mean = 0.005 across every coast frame in those runs.
    #
    # Both, though, because `s_alt > s_now` alone would flip on 0.35-vs-0.30 -- a coin toss
    # dressed as evidence -- and a false flip is not a bad frame, it is a bad TRACKER
    # (`reseat_coast` persists it until a pair rescues us).
    if not (s_alt >= c.coast_flip_support and s_now <= c.coast_flip_empty):
        return dx, False

    # THE GUARD THAT MAKES THIS SAFE IN A MULTI-LANE SCENE.
    #
    # The mask can tell us "there is a lane over there". It cannot tell us "that lane is
    # MINE". On a two-lane road those coincide. On the four-lane roundabout they do not:
    # sitting in lane 1, coasting off the lane1/lane2 boundary, our own far boundary can be
    # outside the FOV (s_now = 0) while lane 2's far boundary is perfectly visible
    # (s_alt high) -- and the test above would happily flip us one lane over. An unrequested
    # lane change, in the one part of the mission where that is worst.
    #
    # But the car is IN the corridor. So the flip must bring the corridor centre TOWARD the
    # vehicle axis, never away from it. Flipping into the neighbour moves it a full lane
    # width further out; flipping back onto our own lane moves it in. That is a free,
    # unambiguous discriminator, and it costs one subtraction.
    if abs(near['x_bottom'] - dx - cx) >= abs(near['x_bottom'] + dx - cx):
        return dx, False
    return -dx, True


def choose_branch(centers, cx, c, rng):
    """WHICH ROUTE. This is the judgment layer's seat, and the judgment layer is not here yet.

    Everything in this file up to now answers "where are the lanes". This answers "which of
    the available routes do we take", and that is a MISSION question -- yellow shortcut or
    main line, roundabout exit 1 or 2 -- which no amount of geometry can settle. So it does
    not pretend to: it applies a named placeholder and SAYS which one, in `LaneState.ego_rule`.

    `keep` (default) reproduces today's behaviour exactly: the Tracker carries the corridor it
    already had, so the car simply stays in its lane and can NEVER take the shortcut. Measured
    over the 0711 clips: at the 218 branch frames, `tracked` won 98% of the time. The system
    does not choose. It continues.

    Returns None for `keep` (let the Tracker decide), else the chosen corridor.
    """
    if not centers or c.branch_policy == 'keep':
        return None
    if c.branch_policy == 'random':
        return centers[rng.randrange(len(centers))]
    return min(centers, key=lambda x: abs(x['x_bottom'] - cx))          # 'nearest'


def ego_center(centers, lanes, w, width, mL=None, mR=None, axis=None, c=None,
               mask=None, margin=0.0):
    """Ego corridor centreline: a real left/right pair, else COAST off one boundary.

    `mL`/`mR` are the Tracker's picks for this frame. Prefer them EVERYWHERE — for choosing
    which corridor is ours, and for the coast side. The Tracker carries the left/right
    identity ACROSS frames (it matches against the lane it tracked last frame), whereas
    deciding the side afresh from `x_bottom < cx` flips as soon as a curving lane crosses the
    image centre — and a flip inverts the shift, so the centreline jumps to the WRONG SIDE of
    the car. That is what produced the sign reversals: 96 of them, 80% while coasting, and the
    steering command pointed INTO the lane instead of away from it (§6c/§6e, §7.4 f250).
    """
    cx = w / 2.0 if axis is None else float(axis)
    ego_tol = (c.ego_tol if c is not None else 0.75)

    # WHICH CORRIDOR IS OURS. With all-pairs pairing there can now be several -- that is the
    # point (a branch you cannot see is a branch you cannot choose). The rule that picks is
    # recorded in `rule`, and published, because a placeholder that does not say it is a
    # placeholder is just a bug waiting to be inherited. There is no route logic here yet:
    # `nearest` is a STAND-IN for the judgment layer, not a decision.
    #
    # 1. The corridor bounded by the two lanes the TRACKER is following. Unambiguous, and it
    #    needs no geometry at all: identity across frames beats any single frame's layout.
    if mL is not None and mR is not None:
        for cc in centers:
            if cc['a'] is mL and cc['b'] is mR:
                cc['ego'] = True
                cc['rule'] = 'tracked'
                return cc
    # 2. Otherwise the corridor whose centreline is NEAREST THE VEHICLE AXIS, and only if it
    #    is close enough to be one we could plausibly be in. This is the physical form of the
    #    old straddle test, and unlike it, it still works when the car is running wide.
    #    At a branch this is where the route gets chosen -- by proximity, i.e. by accident.
    # You can only BE IN a corridor whose boundaries are parallel. A splaying one is a fork
    # you have not taken (see `_pair_gate`). It stays in `centers` -- the judgment layer needs
    # to see it -- but it is not something the car is currently driving down.
    inlane = [x for x in centers
              if c is None or c.pair_parallel <= 0 or x.get('spread', 0.0) <= c.pair_parallel]
    if inlane and width > 0:
        best = min(inlane, key=lambda x: abs(x['x_bottom'] - cx))
        if abs(best['x_bottom'] - cx) <= ego_tol * width:
            best['ego'] = True
            best['rule'] = 'nearest'
            return best
    if width <= 0:
        return None

    # Side from the Tracker (persistent) — not from this frame's geometry.
    near, dx = None, 0.0
    if mL is not None and mL.get('coeffs') is not None:
        near, dx = mL, +width / 2.0          # left boundary -> corridor lies to its RIGHT
    elif mR is not None and mR.get('coeffs') is not None:
        near, dx = mR, -width / 2.0          # right boundary -> corridor lies to its LEFT
    else:
        cand = [x for x in lanes if x['coeffs'] is not None]
        if not cand:
            return None
        near = min(cand, key=lambda x: abs(x['x_bottom'] - cx))
        if abs(near['x_bottom'] - cx) > width:
            return None                       # too far to be an ego boundary
        dx = width / 2.0 if near['x_bottom'] < cx else -width / 2.0

    # Everything above chose the side from an IDENTITY -- the tracker's, or (last resort) the
    # vehicle axis. Now falsify it against the mask: is there really a lane where we are about
    # to claim one is? See `coast_side`.
    flipped = False
    if c is not None:
        dx, flipped = coast_side(near, dx, mask, c, margin, cx)

    coeffs = _shift(near['coeffs'], dx)
    xb = near['x_bottom'] + dx
    # clamp to the source lane's observed span so heading/curvature/drawing never
    # extrapolate the parabola beyond where the lane was actually detected.
    return {'coeffs': coeffs, 'x_bottom': xb, 'offset': xb - cx, 'ego': True,
            'a': near, 'b': None, 'coast': True, 'flipped': flipped, 'rule': 'coast',
            'y_lo': float(near['ys'].min()), 'y_hi': float(near['ys'].max())}


# ==========================================================================
# E: tracker (ego L/R persistence + lane-width coast + turn)
# ==========================================================================
class Tracker:
    def __init__(self, c, h, w, lane_w_px=0.0):
        self.c = c
        self.h, self.w = h, w
        self.lane_w_px = float(lane_w_px)     # BEV lane width; the physical width gate
        self.L = self.R = self.width = None
        self.lost_s = 0.0                     # TIME with no lane, not a frame count
        self.turn = 1

    def _measure_width(self, mL, mR):
        """Corridor width over the COMMON OBSERVED span, as a median. Never extrapolated.

        The old measurement read both parabolas at y = h-1 -- the very bottom BEV row, 26cm.
        The lanes are almost never detected down there: the near field is outside this lens's
        lateral FOV, which is the entire reason `coast` exists. So it evaluated two 2nd-order
        fits well past the data that constrained them, took their difference, and fed that
        into the width EMA. On a curve the extrapolation error of two fits does not cancel --
        it compounds.
        """
        ylo = max(float(mL['ys'].min()), float(mR['ys'].min()))
        yhi = min(float(mL['ys'].max()), float(mR['ys'].max()))
        if yhi - ylo < 1.0:
            return None                        # no shared span -> no measurement
        vs = np.linspace(ylo, yhi, 7)
        gaps = _ebottom(mR['coeffs'], vs) - _ebottom(mL['coeffs'], vs)
        wdt = float(np.median(gaps))           # median, so one bad end cannot drag it
        return wdt if wdt > 0 else None        # a negative "width" is not a width -- see _assign

    def _width_ok(self, wdt):
        """The corridor width is a PHYSICAL fact, not something to learn from whatever pair
        the tracker happened to grab this frame.

        `lane_centers` already refuses a pair that is not a lane width apart (`pair_width_tol`).
        The tracker did not -- it took ANY mL/mR it matched and wrote their gap into the width
        EMA. A stop-line fragment, or the far boundary of the NEXT corridor, poisons it. And
        `self.width` is precisely what every coast frame shifts the centreline by, so a bad
        width does not surface as LOST, or as low confidence, or in any log. It surfaces as
        the car steering confidently to the wrong place.

        Rejecting a measurement is safe: the fallback is `lane_width_default`, which
        `cfg_to_px` sets to the real, physically measured lane width.

        `track_width_tol <= 0` accepts any measured width.
        """
        if self.c.track_width_tol <= 0:
            return True
        return abs(wdt - self.lane_w_px) <= self.c.track_width_tol * self.lane_w_px

    def _dist(self, ins, tracked):
        """Median |dx| between a candidate and a tracked fit, over the candidate's OWN
        observed rows. The old distance read both fits at y = h-1 -- the bottom BEV row,
        which the near-field FOV means neither of them usually reaches."""
        vs = np.linspace(float(ins['ys'].min()), float(ins['ys'].max()), 7)
        return float(np.median(np.abs(_ebottom(ins['coeffs'], vs) - _ebottom(tracked, vs))))

    def _assign(self, cands):
        """Match this frame's candidates to the tracked left/right — over ALL candidates,
        with NO image-half pre-filter. Greedy, nearest pair first; one candidate per role.

        This is the fix. The old code split the candidates on `x_bottom < w/2` and only THEN
        matched each half against its tracked fit -- so the image centre, a screen-space
        accident, got to overrule the temporal identity the tracker exists to carry. On a
        curve, or with the car merely running wide, the left boundary's x_bottom crosses the
        centre; it vanishes from the left pool, reappears in the right one, and gets matched
        as the RIGHT boundary. The corridor flips, its centre steps by a full lane width
        (35cm / 29cm half-width = 1.2 normalised), and the steering command inverts.

        Session 145617 frame 419 did exactly that: left-coast +0.49 -> right-coast -0.38.
        `outlier_relatch_s` bounds how long the car acts on the inverted value (0.16s); it does
        not stop the flip. This does. The image centre now decides left from right in exactly
        one place -- `_seed`, when there is no track to carry.
        """
        scored = []
        for i, ins in enumerate(cands):
            for role, tracked in (('L', self.L), ('R', self.R)):
                if tracked is None:
                    continue
                d = self._dist(ins, tracked)
                if d <= self.c.jump_max:
                    scored.append((d, i, role))
        scored.sort(key=lambda t: t[0])
        out, used_i, used_role = {}, set(), set()
        for d, i, role in scored:
            if i in used_i or role in used_role:
                continue
            out[role] = cands[i]
            used_i.add(i)
            used_role.add(role)
        mL, mR = out.get('L'), out.get('R')

        # A left boundary is LEFT OF the right one. That is not a heuristic, it is what the
        # words mean -- and dropping the `x_bottom < w/2` pre-split silently dropped the only
        # thing that used to enforce it. Proximity matching alone can hand `L` a lane that
        # sits right of the one it hands `R`, and then `_measure_width` returns a NEGATIVE
        # width, `Tracker.width` goes negative, and `ego_center` bails out on `width <= 0` --
        # every frame, forever. Perception reports LOST at 71% while staring at two good lanes.
        #
        # (Found with `track_width_tol` turned off: the width gate had been rejecting the
        # impossible pairs and hiding this. A guard that only holds while another guard holds
        # is not a guard.)
        #
        # An inverted pair means one of the two matches is wrong. Keep the closer one -- it is
        # the better-evidenced -- and let the other side coast.
        if mL is not None and mR is not None and self._measure_width(mL, mR) is None:
            dL = self._dist(mL, self.L) if self.L is not None else float('inf')
            dR = self._dist(mR, self.R) if self.R is not None else float('inf')
            if dL <= dR:
                mR = None
            else:
                mL = None
        return mL, mR

    def _seed(self, cands):
        """No track yet. The image centre is the only thing we have to say which side is
        which -- and this is the ONLY place it is allowed to."""
        cx = self.w / 2.0
        left = [x for x in cands if x['x_bottom'] < cx]
        right = [x for x in cands if x['x_bottom'] >= cx]
        return (max(left, key=lambda x: x['x_bottom']) if left else None,
                min(right, key=lambda x: x['x_bottom']) if right else None)

    def _ema(self, prev, meas, dt_s):
        if prev is None:
            return meas
        a = _ema_alpha(dt_s, self.c.ema_tau_s)
        return tuple(a * m + (1 - a) * p for m, p in zip(meas, prev))

    def _blend_width(self, wdt, dt_s):
        """The corridor-width EMA. Was a hard-coded `0.6*old + 0.4*new` -- not even a config
        field, so it could not be tuned and it silently changed memory with the frame rate."""
        a = _ema_alpha(dt_s, self.c.ema_tau_s)
        self.width = wdt if self.width is None else (1 - a) * self.width + a * wdt

    def adopt(self, a, b, dt_s):
        """Re-seed left/right from a corridor that `lane_centers` physically validated.

        THE MISSING FEEDBACK LOOP. `Tracker.update` and `lane_centers` ran as strangers: the
        tracker matched candidates against its own remembered fits, `lane_centers` paired the
        boundaries it could actually see, and neither ever told the other it was wrong.

        Which let the tracker be wrong forever. Session 145617, frames 264-280: two lanes on
        the ground at -25cm and +4cm -- a corridor, centre -10cm. The tracker had latched the
        RIGHT boundary (+4) as its `L`, synthesised an `R` 39cm further right at +43cm where
        there was no lane at all, and coasted off that phantom, reporting +0.80. Every few
        frames `lane_centers` paired the two REAL boundaries and correctly reported -0.24.
        Neither corrected the other, so center_error oscillated across a full lane width, 8
        times, and the car steered on it.

        A pair that passed `_pair_gate` is a real lane width apart with a real overlap. That
        beats anything we are merely remembering -- so take it, and take its identity with it.

        The old code stumbled onto this correction by re-deriving left/right from the image
        centre every single frame. That self-heals a bad identity, and destroys a good one the
        instant a lane crosses the centre (which is the flip `_assign` exists to stop). Adopt
        keeps the healing and drops the flip: geometry corrects identity only when geometry
        has actually PROVEN something, not merely because a lane wandered across a column.
        """
        self.L, self.R = a['coeffs'], b['coeffs']
        self.lost_s = 0.0
        wdt = self._measure_width(a, b)
        if wdt is not None and self._width_ok(wdt):
            self._blend_width(wdt, dt_s)

    def reseat_coast(self, near, dx, width):
        """`coast_side` found the mask under the mirror of our phantom: `near` is on the
        OTHER side from what we believed. Move the belief, not just this frame's answer.

        A per-frame correction that leaves the tracker wrong is not a correction, it is an
        oscillation: next frame the (still wrong) identity coasts the wrong way again, the
        mask flips it again, and `center_error` swings a full lane width every frame. That is
        measured, not hypothetical -- flipping only the output took the 0711 runs from 0
        oscillations to 3. Same lesson as `adopt`: correct the IDENTITY or do not correct.
        """
        if width <= 0:
            return
        if dx > 0:                                # near is the LEFT boundary after the flip
            self.L = near['coeffs']
            self.R = _shift(self.L, width)
        else:                                     # near is the RIGHT boundary
            self.R = near['coeffs']
            self.L = _shift(self.R, -width)

    def update(self, instances, dt_s):
        cands = [x for x in instances if x['coeffs'] is not None]
        # Tracking -> identity. Not tracking -> geometry. Never geometry OVER identity.
        if self.L is None and self.R is None:
            mL, mR = self._seed(cands)
        else:
            mL, mR = self._assign(cands)
        gL, gR = mL is not None, mR is not None
        # `is not None` let a non-positive width through; the coast shift then moves the
        # synthesised boundary the WRONG WAY. `_measure_width` cannot produce one any more,
        # but this is the only other place it could enter.
        width = (self.width if (self.width is not None and self.width > 0)
                 else self.c.lane_width_default * self.w)
        if gL and gR:
            self.L = self._ema(self.L, mL['coeffs'], dt_s)
            self.R = self._ema(self.R, mR['coeffs'], dt_s)
            wdt = self._measure_width(mL, mR)
            if wdt is not None and self._width_ok(wdt):
                self._blend_width(wdt, dt_s)
            self.lost_s = 0.0
        elif gL:
            self.L = self._ema(self.L, mL['coeffs'], dt_s)
            self.R = _shift(self.L, width)
            self.lost_s = 0.0
        elif gR:
            self.R = self._ema(self.R, mR['coeffs'], dt_s)
            self.L = _shift(self.R, -width)
            self.lost_s = 0.0
        else:
            self.lost_s += dt_s
        if self.lost_s >= self.c.lost_reset_s:
            self.L = self.R = self.width = None
        if self.L is not None and self.R is not None:
            ca = (self.L[0] + self.R[0]) / 2.0
            self.turn = 0 if abs(ca) < self.c.straight_thresh else (1 if ca > 0 else -1)
        return mL, mR


# ==========================================================================
# F: scalar output stabilizer (center_error EMA + failsafe state name)
# ==========================================================================
class _Stabilizer:
    """center_error EMA + failsafe state name.

    TWO accumulators, not one. `missing_s` accrues time with NO usable measurement (lost
    detection / low confidence); `rejects_s` accrues time spent rejecting an OUTLIER -- a
    measurement that DOES exist but disagrees with the EMA. They answer different
    questions and feed different thresholds (`lost_stop_s` vs `outlier_relatch_s`),
    and sharing one variable silently mixed them:

      5 frames of no detection (lost=5) -> detection returns with a legitimately
      different value -> lost=6 >= relatch threshold -> INSTANT relatch.

    That is the one-frame spike defence deleting itself exactly when it is needed --
    the first frame back after a dropout is the least trustworthy one there is. The
    relatch rule is "the new value held for `outlier_relatch_s` STRAIGHT", and only an
    accumulator that counts rejections can say that.

    Both are SECONDS. As frame counts they were thresholds whose real duration moved with
    the day's FPS -- and the relatch bound is a promise about how long the car may steer on
    a possibly wrong-signed value. That promise has to be in seconds to mean anything.
    """

    def __init__(self, c):
        self.c = c
        self.ema = None
        self.missing_s = 0.0   # consecutive TIME with no usable measurement
        self.rejects_s = 0.0   # consecutive TIME rejecting a usable-but-disagreeing value
        self.hist = deque(maxlen=max(1, c.median_window))

    def update(self, center_error, conf, dt_s):
        c = self.c
        if center_error is None or conf < c.conf_low:
            self.missing_s += dt_s
            # A rejection streak is a claim about DISAGREEING measurements. No
            # measurement is not a disagreement -- it breaks the streak.
            self.rejects_s = 0.0
            return self.ema, ('LOST' if self.missing_s >= c.lost_stop_s else 'HOLD')
        self.missing_s = 0.0
        if self.ema is not None and abs(center_error - self.ema) > c.outlier_jump:
            self.rejects_s += dt_s
            if self.rejects_s < c.outlier_relatch_s:
                return self.ema, 'OUTLIER'
            # The rejection had no way out. `outlier_jump` exists to swallow a ONE-frame
            # spike, but the test is against a frozen EMA, so a SUSTAINED move past it
            # re-rejects itself forever: every later frame is measured against the same
            # stale value and fails the same way. That is a latch, not a filter.
            #
            # It fires on a legitimate event. When the tracker swaps which boundary it
            # can see, the corridor centre steps by a full lane width -- 35cm over a 29cm
            # half-width = 1.2 normalised, way past the 0.5 gate. Session 145617 did this
            # at frame 419 (left-coast +0.49 -> right-coast -0.38) and never recovered:
            # the EMA stayed pinned at +0.428 for the last 44 frames (1.47s) while
            # perception kept correctly reporting -0.38. control_core reads `ema` and
            # ignores `state`, so the car steered on the WRONG SIGN for 1.47s.
            #
            # So: reject a spike, but believe a fact. If the new value holds for
            # `outlier_relatch_s` straight it is not noise -- re-seed onto it.
            self.hist.clear()
            self.ema = None
        self.rejects_s = 0.0
        val = center_error
        if c.use_median:
            self.hist.append(center_error)
            val = float(np.median(self.hist))
        a = _ema_alpha(dt_s, c.ema_tau_s)
        self.ema = val if self.ema is None else a * val + (1 - a) * self.ema
        return self.ema, ('LOW_CONF' if conf < c.conf_low * 1.6 else 'OK')


def _heading_deg(coeffs, y_lo, y_hi):
    """Ego centerline tangent as a signed angle (deg). +: recedes rightward.
    y grows downward, so y_hi is the near (bottom) end, y_lo the far (top) end."""
    x_near, x_far = _ebottom(coeffs, y_hi), _ebottom(coeffs, y_lo)
    denom = (y_hi - y_lo) or 1e-6
    return float(math.degrees(math.atan2(x_far - x_near, denom)))


# ==========================================================================
# Public pipeline (stateful across frames)
# ==========================================================================
class LanePipeline:
    """Lane pipeline. REQUIRES a calibrated `cam` (dracer_core.calib.CameraModel).

    The front-view path is gone. It was not a fallback, it was a second pipeline: lane fits
    in image coords, every geometric threshold a screen fraction, and a coast that shifts by
    a constant pixel offset -- which is only a constant DISTANCE in a BEV. None of the
    physical gates this pipeline now depends on (lane width, pairing, coast side) can even be
    asked in a perspective image, and the cm config that drives them would be meaningless.
    Its last caller (the offline control-prediction tool, since removed) was not using it
    deliberately; it was predicting control commands from a pipeline the car does not run.

    A missing camera.yaml is now a hard failure, and that is the safe behaviour: the car
    does not move (control_node's perception watchdog sees no /lane/state and publishes
    neutral), instead of driving on a pipeline nobody tuned.
    """

    def __init__(self, cfg, cam):
        if cam is None:
            raise ValueError('LanePipeline: CameraModel 이 필요하다 (front-view 경로는 삭제됨).')
        self.cfg = cfg
        self.cam = cam
        self.c = cfg_to_px(cfg, cam)          # cm -> BEV px, once
        self.stab = _Stabilizer(self.c)
        self.trk = None
        self._size = None
        self._in_branch = False               # latch: a route is chosen ONCE per branch
        self._rng = random.Random(self.c.branch_seed)   # seeded: a replay must reproduce

    def reconfigure(self, cfg):
        """Swap the cm config and KEEP the cross-frame state (tracker identity, width, EMAs).

        `ros2 param set` used to rebuild the whole pipeline, which threw away everything the
        Tracker had learned -- left/right identity, the width EMA, the centre EMA -- so tuning
        one gain cost a perception discontinuity, and the README told you to stop the car
        before touching a parameter. That is backwards. The state is a MEASUREMENT (where the
        lanes are, how wide this corridor is); the config is a JUDGEMENT (what counts as a
        lane). Changing the judgement does not invalidate the measurement.

        Everything reads `self.c` per frame and so follows automatically -- EXCEPT two values
        frozen at construction, which is exactly why a rebuild looked like the only option:

          - `_Stabilizer.hist` bakes `median_window` into a deque maxlen.
          - `Tracker.lane_w_px` bakes `lane_width_cm` into BEV px.

        A wildly different `lane_width_cm` can leave the tracked `width` outside the new
        `_width_ok` band. That is safe by construction: the measurement is rejected and the
        tracker falls back to `lane_width_default` (see `_width_ok`).

        BEV GEOMETRY is the one thing this cannot absorb -- if `cam` changes, the tracked
        pixels mean a different place and the state IS void. That path still rebuilds
        (`perception_node._match_camera`), and it must.
        """
        seed_changed = cfg.branch_seed != self.cfg.branch_seed
        self.cfg = cfg
        self.c = cfg_to_px(cfg, self.cam)

        self.stab.c = self.c
        mw = max(1, self.c.median_window)
        if self.stab.hist.maxlen != mw:
            self.stab.hist = deque(self.stab.hist, maxlen=mw)   # carry the samples over

        if self.trk is not None:
            self.trk.c = self.c
            self.trk.lane_w_px = float(self.cam.lane_width_px(cfg.lane_width_cm))

        # Reseed ONLY when the seed itself changed. Restarting the draw sequence on every
        # unrelated `param set` would make a branch choice depend on when you last tuned.
        if seed_changed:
            self._rng = random.Random(self.c.branch_seed)

    def process(self, bgr, dt_s, debug=False, hsv=None):
        """Run one frame. Returns `state`; with debug=True returns (state, dbg), the
        second being intermediates for render_panels (masks, lanes, windows, ec, ...).

        `hsv` is an optional FULL-FRAME HSV of `bgr`. The merged perception node computes
        it once on the frames where mission detection also runs, so the frame is converted
        once instead of twice. Omit it and the pipeline converts its own band, as before.

        `dt_s` is the interval since the previous frame, in SECONDS, and it is REQUIRED --
        every filter in here (lane-coeff EMA, corridor-width EMA, center_error EMA) and every
        failsafe threshold (`lost_reset_s`, `lost_stop_s`, `outlier_relatch_s`) is expressed
        as a physical duration. Perception cannot measure its own dt (it does not own the
        clock or the frame stamps), so the caller must pass it: `perception_node` from the
        ROS clock, the offline tools from the clip's fps.

        Pass 0.0 for the first frame -- no time has passed, so the EMAs seed rather than
        blend, which is exactly right. It is a required positional argument on purpose: the
        old `process(frame, debug=True)` call now raises TypeError instead of quietly
        binding `True` to `dt_s`.
        """
        dt_s = max(0.0, float(dt_s))
        c = self.c
        h, w = bgr.shape[:2]

        # --- BEV seam --------------------------------------------------------
        # Warp the COLOR MASKS, not the frame: colour is a per-pixel test, so thresholding
        # before the resample is both cheaper (2 x 1ch out vs 1 x 3ch) and sharper (no
        # interpolated in-between colours to fool the HSV gates). Everything after the warp
        # is coordinate-agnostic — it just starts getting metric pixels.
        #
        # Rows above the LUT's top feed NO BEV cell, so colour-test only the band it reads
        # (~38% fewer pixels through cvtColor).
        r0, r1 = self.cam.src_rows
        mw = np.zeros((h, w), np.uint8)
        my = np.zeros((h, w), np.uint8)
        mw[r0:r1], my[r0:r1] = color_masks(bgr[r0:r1], c,
                                           None if hsv is None else hsv[r0:r1])
        mw = self.cam.to_bev(mw)
        my = self.cam.to_bev(my)
        # Shape + area judgements now that a pixel IS a length.
        mw, my = morph_gate(mw, my, c)
        w, h = self.cam.bev_size
        lane_w_px = self.cam.lane_width_px(c.lane_width_cm)
        trap, y0 = self.cam.footprint_poly(), r0
        if self._size != (h, w):
            self._size = (h, w)
            self.trk = Tracker(c, h, w, lane_w_px)

        windows = []
        lanes = (sliding_window_lanes(mw, 'W', c, windows) +
                 sliding_window_lanes(my, 'Y', c, windows))
        mL, mR = self.trk.update(lanes, dt_s)

        # The vehicle axis, not the image centre. They coincide only when the camera is
        # mounted dead centre (lateral_offset_cm = 0); `cam.axis_u` is the real one, and it
        # is what `center_error` must be measured from.
        axis = self.cam.axis_u if self.cam is not None else w / 2.0
        # The combined mask, for `coast_side` to falsify a coast against. The expensive part
        # (the cumulative sum) is built inside it, i.e. only on frames that actually coast.
        mask = cv2.bitwise_or(mw, my) if c.coast_flip_support > 0 else None
        centers = lane_centers(lanes, w, h, c, lane_w_px, axis)
        n_corridors = len(centers)     # >1 = a BRANCH. Recorded, so the judgment layer that
                                       # does not exist yet can be designed from real data.

        # --- branch: pick a route, ONCE, and stick to it ----------------------
        # LATCHED. Choosing afresh every frame swings center_error by a full lane width every
        # frame -- that is not a hypothesis, it is what a per-frame `coast_side` flip measured
        # (0 oscillations -> 3, §8+). So: decide on ENTRY to the branch, push the decision into
        # the Tracker (`adopt`), and let the Tracker's identity carry it. The latch clears when
        # the branch does.
        branch_pick = None
        if n_corridors >= 2:
            if not self._in_branch:
                self._in_branch = True
                branch_pick = choose_branch(centers, axis, c, self._rng)
        else:
            self._in_branch = False
        if branch_pick is not None:
            mL, mR = branch_pick['a'], branch_pick['b']     # ego_center picks this up as
            self.trk.adopt(mL, mR, dt_s)                    # 'tracked' -- and STAYS there
        # `if self.trk.width` was the test, and a NEGATIVE width is truthy -- it sailed straight
        # through into `ego_center`, which then returned None on `width <= 0`. Belt and braces:
        # `_measure_width` can no longer produce one, and this can no longer pass one on.
        width = self.trk.width if (self.trk.width and self.trk.width > 0) else c.lane_width_default * w
        ec = ego_center(centers, lanes, w, width, mL, mR, axis, c, mask, c.sw_margin)

        # If the ego corridor is a REAL pair that is not the one the tracker was following,
        # the tracker was following the wrong thing. Take the proven identity (see adopt()).
        if (ec is not None and ec.get('b') is not None
                and (ec['a'] is not mL or ec['b'] is not mR)):
            self.trk.adopt(ec['a'], ec['b'], dt_s)
        # A coast the mask contradicted. Reseat the tracker so the correction STICKS --
        # otherwise it un-corrects itself next frame and oscillates (see reseat_coast).
        elif ec is not None and ec.get('flipped'):
            self.trk.reseat_coast(ec['a'], ec['x_bottom'] - ec['a']['x_bottom'], width)

        if ec is not None:
            center_error = float(ec['offset'] / (w / 2))
            y_lo, y_hi = ec.get('y_lo', 0), ec.get('y_hi', h - 1)
            heading = _heading_deg(ec['coeffs'], y_lo, y_hi)
            curvature = float(ec['coeffs'][0] * 1000.0)
            confidence = 0.5 if ec.get('coast') else 0.9
        else:
            center_error = heading = curvature = None
            confidence = 0.0
        left_conf = 1.0 if mL is not None else 0.0
        right_conf = 1.0 if mR is not None else 0.0
        used_fb = bool(ec is not None and ec.get('coast'))

        ema, fstate = self.stab.update(center_error, confidence, dt_s)

        state = {
            'center_error': center_error, 'ema': ema,
            'heading': heading, 'heading_label': 'lane7',
            'confidence': confidence, 'left_conf': left_conf,
            'right_conf': right_conf, 'curvature': curvature,
            'state': fstate, 'used_fallback': used_fb,
            # Branch evidence, for the judgment layer that does not exist yet. `n_corridors`
            # > 1 means the frame physically supports more than one route and something had
            # to choose; `ego_rule` says WHICH placeholder chose. Neither is acted on.
            'n_corridors': n_corridors,
            'ego_rule': (f'branch_{c.branch_policy}' if (branch_pick is not None and ec is branch_pick)
                         else ((ec.get('rule') or 'none') if ec is not None else 'none')),
        }
        if debug:
            dbg = {'mw': mw, 'my': my, 'lanes': lanes, 'windows': windows, 'ec': ec,
                   'trap': trap, 'y0': y0, 'ema': ema, 'fstate': fstate,
                   'center_error': center_error, 'heading': heading,
                   'confidence': confidence, 'used_fb': used_fb, 'cam': self.cam,
                   'bev_size': (w, h),
                   # EVERY corridor, not just ours, plus the threshold that separates the
                   # one we are IN from the fork we have not taken. `render_bev` draws them
                   # all: n_corridors > 1 IS the branch, and a branch you cannot see is a
                   # branch you cannot debug.
                   'centers': centers, 'n_corridors': n_corridors,
                   'ego_rule': state['ego_rule'], 'pair_parallel': c.pair_parallel}
            return state, dbg
        return state


# ==========================================================================
# rendering (camera view + BEV panels)
# ==========================================================================
def _lane_span(ins):
    return int(ins['ys'].min()), int(ins['ys'].max())


def _draw_fit(img, coeffs, y0, y1, color, thick, cam):
    """Draw a lane fit. The fit lives in BEV ground coords -> unwarp it back onto the
    camera frame, so the overlay a human looks at is always the real view."""
    pts = cam.bev_coeffs_to_image(coeffs, y0, y1)
    h, w = img.shape[:2]
    p = np.array([[int(x), int(y)] for x, y in pts
                  if 0 <= x < w and 0 <= y < h], np.int32)
    if len(p) > 1:
        cv2.polylines(img, [p], False, color, thick)


def _panel(img, title):
    cv2.rectangle(img, (0, 0), (img.shape[1], 14), (0, 0, 0), -1)
    cv2.putText(img, title, (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    return img


def _footer(img, text):
    h = img.shape[0]
    cv2.rectangle(img, (0, h - 14), (img.shape[1], h), (0, 0, 0), -1)
    cv2.putText(img, text, (3, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    return img


def _draw_bev_fit(img, coeffs, y_lo, y_hi, color, thick, dashed=False):
    """Draw a fit in BEV coords. No unwarp: the fits ALREADY live here. (`_draw_fit`
    projects them back to the camera, which is a per-lane cost `render_panels` pays and
    this one does not.)"""
    ys = np.linspace(float(y_lo), float(y_hi), 24)
    xs = _ebottom(coeffs, ys)
    p = np.stack([xs, ys], axis=1).astype(np.int32)
    if not dashed:
        cv2.polylines(img, [p], False, color, thick)
        return
    for i in range(0, len(p) - 1, 2):        # every other segment -> a dash
        cv2.line(img, (int(p[i][0]), int(p[i][1])),
                 (int(p[i + 1][0]), int(p[i + 1][1])), color, thick)


def render_bev(bgr, dbg, cfg, dets=None, confirmed=None):
    """The ONE panel `drive` and `lap` watch: metric BEV | camera.

        [ BEV: masks + labelled lanes + EVERY corridor centreline ] [ camera: mission boxes ]

    Two coordinate systems, side by side, because a bounding box CANNOT live in a BEV. A
    traffic light is above the ground plane, so it projects onto no BEV cell at all; a lane
    corridor is a metric ground fact and is unreadable anywhere else. The BEV is where the
    lane question is asked, the camera view is where the object question is asked, and one
    canvas pretending to answer both would simply be hiding one of them.

    552x240 against `render_panels`' 1280x240 -- 43% of the pixels, and no per-lane unwarp.
    That gap is the whole reason this exists: the 4-panel strip plus its JPEG encode is what
    made perception drop frames, and `drive` is the launch that wants to be watched.

    Corridors are coloured by their BOUNDARIES (see CORRIDOR_COLORS), so the branch TYPE is
    visible at a glance, and a SPLAYING corridor is drawn dashed -- that is not a bad
    corridor, it is the fork we have not taken (`_pair_gate`). The ego centreline (cyan, on
    top) is the only one the car is actually steering on.
    """
    mw, my, lanes = dbg['mw'], dbg['my'], dbg['lanes']
    ec, centers = dbg['ec'], dbg.get('centers') or []
    cam = dbg.get('cam')
    bw, bh = dbg.get('bev_size', (bgr.shape[1], bgr.shape[0]))
    parallel = float(dbg.get('pair_parallel', 0.0))

    bev = np.zeros((bh, bw, 3), np.uint8)
    bev[mw > 0] = (90, 90, 90)
    bev[my > 0] = (0, 110, 140)
    axis_u = int(cam.axis_u) if cam is not None else bw // 2
    cv2.line(bev, (axis_u, 0), (axis_u, bh - 1), (0, 0, 130), 1)   # vehicle axis

    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        _draw_bev_fit(bev, ins['coeffs'], *_lane_span(ins),
                      LABEL_COLORS.get(classify(ins, bw), (0, 255, 0)), 2)

    for cc in centers:
        if cc is ec:
            continue                        # drawn last, on top, in cyan
        splay = parallel > 0 and cc.get('spread', 0.0) > parallel
        _draw_bev_fit(bev, cc['coeffs'], cc['y_lo'], cc['y_hi'],
                      CORRIDOR_COLORS.get(cc['a']['color'], (200, 200, 200)),
                      1, dashed=splay)
    if ec is not None:
        _draw_bev_fit(bev, ec['coeffs'], ec.get('y_lo', 0), ec.get('y_hi', bh - 1),
                      EGO_CENTER_COLOR, 2)

    cam_view = annotate(bgr, dets) if dets else bgr.copy()
    ch = cam_view.shape[0]

    left = np.zeros((ch, bw, 3), np.uint8)
    y0 = max(0, (ch - bh) // 2)
    rows = min(bh, ch - y0)
    left[y0:y0 + rows] = bev[:rows]

    off = ('off=--' if (ec is None or cam is None)
           else f"off={ec['offset'] / cam.px_per_cm:+.1f}cm{'(coast)' if ec.get('coast') else ''}")
    _panel(left, f"BEV  n={dbg.get('n_corridors', 0)}[{dbg.get('ego_rule', 'none')}]")
    _footer(left, f"{off}  {dbg['fstate']}")
    _panel(cam_view, f"camera  {cfg.name}")
    _footer(cam_view, 'mission: ' + (CLASS_NAMES.get(confirmed, '?') if confirmed is not None
                                     else '--'))
    return np.hstack([left, cam_view])


def _inst_color(idx):
    bgr = cv2.cvtColor(np.uint8([[[int((idx * 47) % 180), 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def render_panels(bgr, dbg, cfg):
    """Online debug multi-panel (4 panels):
      [ input+ROI | mask | sliding windows+lanes | label+ego overlay ].
    Built only from `LanePipeline.process(..., debug=True)` intermediates (no
    re-implementation). perception_node publishes this on /lane/debug/compressed.

    Panels 2-3 show the METRIC top-down view (that is where the lanes are actually found),
    letter-boxed to the frame size so the strip stays uniform, while panels 1 and 4 stay in
    the camera view a human can read.
    """
    h, w = bgr.shape[:2]
    mw, my, lanes = dbg['mw'], dbg['my'], dbg['lanes']
    windows, ec, trap = dbg['windows'], dbg['ec'], dbg['trap']
    cam = dbg.get('cam')
    bw, bh = dbg.get('bev_size', (w, h))     # lane coords live in THIS frame

    def _fit_panel(img):
        """BEV canvas -> frame-sized panel (uniform strip, aspect preserved)."""
        if (img.shape[1], img.shape[0]) == (w, h):
            return img
        s = min(w / img.shape[1], h / img.shape[0])
        r = cv2.resize(img, (max(1, int(img.shape[1] * s)), max(1, int(img.shape[0] * s))),
                       interpolation=cv2.INTER_NEAREST)
        out = np.zeros((h, w, 3), np.uint8)
        y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
        out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
        return out

    # P1: input + ROI trapezoid (camera view)
    p1 = bgr.copy()
    cv2.polylines(p1, [trap.astype(np.int32)], True, (0, 200, 255), 1)
    _panel(p1, f'1 input+ROI  {cfg.name}')

    # P2: mask — in BEV mode this is the warped, metric mask
    p2 = np.zeros((bh, bw, 3), np.uint8)
    p2[mw > 0] = (200, 200, 200)
    p2[my > 0] = (0, 220, 255)
    cv2.line(p2, (int(cam.axis_u), 0), (int(cam.axis_u), bh - 1), (0, 0, 120), 1)  # vehicle axis
    p2 = _fit_panel(p2)
    _panel(p2, '2 mask  BEV metric')

    # P3: sliding windows + lane points (same coords as the fits)
    p3 = np.zeros((bh, bw, 3), np.uint8)
    for (xlo, ylo, xhi, yhi) in windows:
        cv2.rectangle(p3, (xlo, ylo), (xhi, yhi), (55, 55, 55), 1)
    for idx, ins in enumerate(lanes):
        col = _inst_color(idx)
        for x, y in zip(ins['xs'][::3], ins['ys'][::3]):
            cv2.circle(p3, (int(x), int(y)), 1, col, -1)
    p3 = _fit_panel(p3)
    _panel(p3, f'3 sliding ({len(lanes)} lane)')

    # P4: labels + ego centreline, unwarped back onto the camera view
    p4 = bgr.copy()
    ty = 26
    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        lab = classify(ins, bw)
        col = LABEL_COLORS.get(lab, (0, 255, 0))
        _draw_fit(p4, ins['coeffs'], *_lane_span(ins), col, 2, cam)
        cv2.putText(p4, lab, (4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)
        ty += 14
    if ec is not None:
        _draw_fit(p4, ec['coeffs'], ec.get('y_lo', 0), ec.get('y_hi', bh - 1),
                  EGO_CENTER_COLOR, 2, cam)
    if ec is None:
        off = 'off=--'
    else:
        off = f"off={ec['offset'] / cam.px_per_cm:+.1f}cm{'(coast)' if ec.get('coast') else ''}"
    _panel(p4, f"4 labels+ego  {off}  {dbg['fstate']}")

    return np.hstack([p1, p2, p3, p4])
