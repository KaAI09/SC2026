"""ROS-independent mission-object detection (shared by the onboard perception node).

Detects the competition mission objects and returns detections as
(cls_id, confidence, bbox=(x, y, w, h)). Class IDs match the team scheme:

    0 GREEN   traffic-light go signal      -> HSV green
    1 RED     traffic-light stop signal    -> HSV red
    2 MARK    dynamic obstacle (ArUco)     -> cv2.aruco 6x6_50, marker ID 3
    3 RIGHT   direction sign (arrow right) -> blue circle + arrow direction
    4 LEFT    direction sign (arrow left)  -> blue circle + arrow direction

Design: each class uses the cheapest reliable method (aruco / color are ~free,
sign is gated). Everything track/camera dependent is a Cfg field so it retunes
to the real camera. Lightweight for real-time D3-G use. Depends only on cv2 +
numpy so it can be unit-tested on images off-board.

    from dracer_core.mission_core import MissionDetector, CLASS_NAMES
    det = MissionDetector()
    dets, confirmed, newly = det.process(bgr)   # confirmed = debounced class id

SHARED BUFFERS. `detect_*` and `MissionDetector.process` take OPTIONAL precomputed
`hsv` / `gray` of the SAME frame. They run in the same node as the lane pipeline, on
the same frame, and each was calling cvtColor again on pixels that had already been
converted. A per-pixel conversion of a slice is bit-identical to a slice of the
conversion, so passing them in changes nothing except how many times the frame is
walked. Omit them and every function converts for itself, exactly as before -- the
offline tools still call `detect_light(bgr, cfg)` and get the same answer.
"""
from collections import Counter, deque
from dataclasses import dataclass

import cv2
import numpy as np

CLASS_NAMES = {0: 'GREEN', 1: 'RED', 2: 'MARK', 3: 'RIGHT', 4: 'LEFT'}
STOP_CLASSES = (1, 2)                    # RED, MARK -- the classes that withhold throttle
CLASS_COLORS = {0: (0, 220, 0), 1: (0, 0, 255), 2: (255, 0, 255),   # BGR: green / red /
                3: (255, 170, 0), 4: (255, 170, 0)}                 # magenta / sign blue


@dataclass
class MissionCfg:
    # --- ArUco (cls 2 MARK) ---
    aruco_dict: int = cv2.aruco.DICT_6X6_50
    aruco_ids: tuple = (3,)              # physical marker ID (detected -> class 2)
    aruco_min_perim: float = 40.0        # reject tiny/far markers (px perimeter)
    # How many wrong bits the dictionary is allowed to correct. OpenCV ships 0.6; 0.8 lifts
    # the hit rate on the marker clip 27% -> 31% for free (2.8ms/frame, unchanged) with ZERO
    # id-3 false positives and no hallucinated ids over 4167 marker-free frames. 0.9+ reaches
    # 36% but starts inventing markers that are not there (id 19/24/31/36), so it stops here.
    # Every other DetectorParameters knob that helped cost 3-20x the CPU for less.
    aruco_error_correction: float = 0.8
    # NOTE: how long the MARK stop is HELD is NOT here. It belongs to MissionGate (below),
    # which lives in control_node -- `mission_mark_hold_s`. MissionCfg is the DETECTOR's
    # config: mission_node turns every field in it into a ROS param, so a knob parked here
    # that the detector never reads would be a knob the venue can turn with no effect.

    # --- traffic light (cls 0 GREEN, 1 RED), HSV ---
    # the light sits high in the frame; restrict search to the top band to avoid
    # the orange lane tape (orange hue overlaps red). Tune with real footage.
    light_roi_top: float = 0.0
    light_roi_bot: float = 1.0            # full frame; circularity rejects the lane. tune on real camera
    # COLOUR + SHAPE. Nothing else.
    #
    # AN LED IS THE PUREST COLOUR IN THE SCENE. It emits near-monochromatic light, so its
    # SATURATION pins to the top of the range. Anything else that is "red" -- paint, a fire
    # extinguisher, a wooden crate, orange lane tape -- reflects broadband light and cannot
    # get there. Measured on the real LED traffic light (data/mission/raw):
    #
    #                   lamp          background of the same hue
    #   RED    sat      232..255      143..170
    #   GREEN  sat      208..218       48..63
    #   both   circ     0.58..0.62    0.20..0.26
    #
    #   sat >= 190 keeps 93% of red and 90% of green lamps, and rejects 99.9% / 100% of the
    #   background. That is the whole detector.
    #
    # Saturation is the right gate because it is a property of the LIGHT SOURCE -- not of the
    # exposure, the background, or the distance. Brightness is NOT: the same LED reads V~59
    # on this camera (auto-exposure meters the bright room, so the small lamp comes out
    # DARKER than the scene), while the same lamp photographed on an iPad screen reads V~190.
    # A brightness gate tuned on one inverts on the other. Saturation does not move.
    green_h: tuple = (50, 85)
    green_s_min: int = 190                # LED is monochromatic; background green is 48-63
    green_v_min: int = 30                 # noise floor only -- brightness does NOT identify a lamp
    green_min_circ: float = 0.50          # lamp 0.57-0.81 vs background 0.20
    red_h_lo: tuple = (0, 10)             # the lit red LED is hue 2-6
    red_h_hi: tuple = (165, 180)          # red straddles the hue wrap; it needs two bands
    # 205, not 190: at 190 a BRIGHT RED BACKGROUND OBJECT (sat 213-217, val 138-140) sitting
    # above the traffic light confirmed RED in two GREEN clips -- the car would have stopped
    # 30s at a green light. The lit LED is sat 227-255. 205 splits them and still confirms RED
    # in all 5 red clips. Do NOT reach for a brightness cap instead: the real lamp is DARK
    # (val 39-64) and the background object is bright, so it looks tempting -- but the same
    # lamp on an iPad screen is val 190, and a venue LED might be too.
    red_s_min: int = 205
    red_v_min: int = 30                   # noise floor only
    red_min_circ: float = 0.50            # lamp 0.54-0.71 vs background 0.26
    light_min_area: int = 35              # the LED is small on a 320-wide frame
    red_min_area: int = 40
    light_max_area_frac: float = 0.05     # reject blobs > 5% of frame (e.g. a red floor)
    # NOT here, and deliberately: a "reject blobs touching the frame edge" rule. It looks
    # principled (a clipped blob is a fragment, so its area and circularity are measured on
    # whatever stayed in view) and it does kill the background false positives. But the REAL
    # lamp reaches the left edge while it is still small -- measured (0,15,10,9), 40-63px, a
    # genuine red light the car has to stop for. The rule deleted it. Saturation is what
    # actually separates them; the edge is a coincidence.
    #
    # There is NO housing/surround filter. It measured the ring of scene brightness around
    # the blob, on the theory that a lamp sits in a dark body. What it actually measured was
    # WHAT IS BEHIND THE TRAFFIC LIGHT: lamp 36-110 vs background 58-85, completely
    # overlapping. It cost a real stop -- in 'red error.mp4' the lamp was in frame for 48
    # frames, round and saturated, and the ring gate rejected every one. The car drove
    # through the red light.

    # --- direction sign (cls 3 RIGHT, 4 LEFT): blue circle + arrow ---
    blue_h: tuple = (100, 130)
    blue_s_min: int = 80
    blue_v_min: int = 50
    sign_min_area: int = 250
    sign_min_circ: float = 0.55
    arrow_min_px: int = 20               # min hole px to read a direction (else skip)
    arrow_margin: float = 0.08           # dead zone for the arrow's lean (upper-half centre
                                         # minus lower-half centre, over width). Real signs
                                         # measure |lean| 0.19-0.28, so 0.08 refuses only
                                         # genuinely ambiguous blobs. LOWER -> decides on
                                         # weaker arrows (risks flips)
    sign_invert: bool = False            # VENUE ESCAPE HATCH: if the arrow style makes
                                         # RIGHT/LEFT come out swapped, set true (yaml) to
                                         # flip 3<->4 -- no arrow-logic tinkering needed

    # --- debounce / confidence (M-of-N: tolerates dropped frames) ---
    confirm_n: int = 5                    # window: the last N PROCESSED frames considered
    confirm_m: int = 3                    # GO (green/sign): confirm at >= M of the last N
    confirm_m_stop: int = 2               # STOP (red/marker): confirm sooner. A false stop
                                          # costs seconds; a missed stop hits the obstacle.
                                          # Also: the car passes the ArUco marker fast, and
                                          # the window is in PROCESSED frames -- at
                                          # frame_skip=2 a 3-of-5 vote needs the marker held
                                          # for ~15 camera frames (0.5s at 30Hz).
    min_conf: float = 0.5


# ==========================================================================
# ArUco (version-robust across OpenCV 4.5 / 4.7+)
# ==========================================================================
def _get_dict(d):
    try:
        return cv2.aruco.getPredefinedDictionary(d)
    except AttributeError:
        return cv2.aruco.Dictionary_get(d)


def _make_aruco(cfg):
    dictionary = _get_dict(cfg.aruco_dict)
    if hasattr(cv2.aruco, 'ArucoDetector'):                     # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        params.errorCorrectionRate = cfg.aruco_error_correction
        return ('new', cv2.aruco.ArucoDetector(dictionary, params))
    params = cv2.aruco.DetectorParameters_create()             # OpenCV 4.5.x
    params.errorCorrectionRate = cfg.aruco_error_correction
    return ('old', (dictionary, params))


def _aruco_detect(aruco, gray):
    mode, obj = aruco
    if mode == 'new':
        corners, ids, _ = obj.detectMarkers(gray)
    else:
        dictionary, params = obj
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids


def detect_aruco(bgr, cfg, aruco, gray=None):
    if gray is None:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids = _aruco_detect(aruco, gray)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        if int(i) not in cfg.aruco_ids:
            continue
        pts = c.reshape(-1, 2).astype(np.float32)
        if cv2.arcLength(pts, True) < cfg.aruco_min_perim:
            continue
        x, y, w, h = cv2.boundingRect(pts.astype(np.int32))
        out.append((2, 1.0, (int(x), int(y), int(w), int(h))))  # aruco = deterministic
    return out


# ==========================================================================
# Color helpers
# ==========================================================================
def _mask(hsv, h_lo, h_hi, s_min, v_min):
    return cv2.inRange(hsv, (h_lo, s_min, v_min), (h_hi, 255, 255))


def _circularity(cnt):
    area = cv2.contourArea(cnt)
    (_, _), r = cv2.minEnclosingCircle(cnt)
    return area / (np.pi * r * r + 1e-6) if r > 0 else 0.0


def _best_blob(mask, min_area, min_circ, max_area=None):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < min_area or (max_area and area > max_area) or _circularity(cnt) < min_circ:
            continue
        if best is None or area > best[0]:
            best = (area, cnt)
    return best


# ==========================================================================
# Traffic light (cls 0 GREEN, 1 RED)
# ==========================================================================
def _light_masks(hsv, cfg):
    """Green + red masks, morph-closed to merge fragmented LEDs.

    Red needs two hue bands: it straddles the wrap at 0/179.

    (A bright-core "bloom fill" used to widen each mask with very-bright pixels
    adjacent to its colour ring. Measured on the 8 real-car raw clips it changed
    nothing at all -- identical detections and identical blob sizes with it on and
    off -- because the auto-exposed lamp reads DARK on this camera (V~55), so there
    are no >235 pixels to fill with. Removed.)
    """
    k = np.ones((3, 3), np.uint8)
    red = cv2.bitwise_or(
        _mask(hsv, cfg.red_h_lo[0], cfg.red_h_lo[1], cfg.red_s_min, cfg.red_v_min),
        _mask(hsv, cfg.red_h_hi[0], cfg.red_h_hi[1], cfg.red_s_min, cfg.red_v_min))
    green = _mask(hsv, cfg.green_h[0], cfg.green_h[1], cfg.green_s_min, cfg.green_v_min)
    green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, k)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k)
    return green, red


def blob_v(vchan, cnt):
    """Median brightness of the blob itself. A LIT lamp emits -- that is what makes it a
    lamp and not red paint -- so on this camera it comes out far brighter than any
    background object of the same hue."""
    m = np.zeros(vchan.shape, np.uint8)
    cv2.drawContours(m, [cnt], -1, 255, -1)
    px = vchan[m > 0]
    return float(np.median(px)) if px.size else 0.0


def _light_best(mask, cfg, min_area, max_area, min_circ, min_v, vchan):
    """Largest blob passing area + circularity + brightness. Returns (area, cnt, circ)."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        circ = _circularity(cnt)
        if circ < min_circ:
            continue
        if blob_v(vchan, cnt) < min_v:
            continue
        if best is None or area > best[0]:
            best = (area, cnt, circ)
    return best


def _light_emit(cls_id, best, cfg, min_area, min_circ, y0):
    area, cnt, circ = best
    x, y, bw, bh = cv2.boundingRect(cnt)
    # a blob passing colour + shape + brightness IS a light; strength from area + roundness
    area_f = min(1.0, area / (min_area * 2.0))
    conf = min(1.0, 0.45 + 0.35 * area_f + 0.20 * min(1.0, circ / max(min_circ, 1e-6)))
    return (cls_id, float(conf), (int(x), int(y0 + y), int(bw), int(bh)))


def detect_light(bgr, cfg, hsv=None):
    """COLOUR + SHAPE only. Red is gated on brightness, green on roundness -- that is what
    the measurements said separates each one from its own background (see MissionCfg).

    `hsv`, if given, is the FULL-FRAME HSV of `bgr`; the ROI is sliced from it.
    """
    h, w = bgr.shape[:2]
    y0, y1 = int(h * cfg.light_roi_top), int(h * cfg.light_roi_bot)
    hsv = cv2.cvtColor(bgr[y0:y1], cv2.COLOR_BGR2HSV) if hsv is None else hsv[y0:y1]
    green, red = _light_masks(hsv, cfg)
    v = hsv[:, :, 2]
    max_area = cfg.light_max_area_frac * h * w
    best_red = _light_best(red, cfg, cfg.red_min_area, max_area,
                           cfg.red_min_circ, cfg.red_v_min, v)
    best_green = _light_best(green, cfg, cfg.light_min_area, max_area,
                             cfg.green_min_circ, cfg.green_v_min, v)
    # A traffic light shows ONE colour at a time. If both somehow survive, take the SAFE
    # one: red. A false stop costs seconds; a false go runs the light.
    if best_red:
        return [_light_emit(1, best_red, cfg, cfg.red_min_area, cfg.red_min_circ, y0)]
    if best_green:
        return [_light_emit(0, best_green, cfg, cfg.light_min_area, cfg.green_min_circ, y0)]
    return []


# ==========================================================================
# Direction sign (cls 3 RIGHT, 4 LEFT): blue circle + arrow direction
# ==========================================================================
def _arrow_direction(blue, cnt, cfg):
    """Return 3 (RIGHT) / 4 (LEFT) / None for the arrow inside a blue circle.

    The arrow is the HOLE in the blue disc, so it needs no threshold of its own:

        arrow = (the blue contour, filled) - (the blue mask)

    Direction is GEOMETRY. A turn arrow is a tail rising from the bottom centre into a
    head that bends to one side, so the arrow's UPPER half sits to the side it points
    at, relative to its LOWER half:

        lean = (mean x of the hole's upper half - mean x of its lower half) / width
            > 0  -> head leans right -> RIGHT
            < 0  ->             left -> LEFT

    Measured on the real-car sign clips: |lean| runs 0.19-0.28, sign always correct,
    0 wrong calls on 386 sign frames.

    Two earlier approaches lost. Counting white pixels left vs right of the midline
    fails by construction -- the tail runs down the middle, so the halves come out
    nearly equal (0-8% imbalance on the RIGHT clip, under any usable margin: 119 of 164
    frames returned "no idea"). Doing the same lean on a brightness-thresholded white
    mask works, but needs an arrow_v_min that the venue's lighting can break; the hole
    needs only the blue mask we already tune.
    """
    x, y, w, h = cv2.boundingRect(cnt)
    filled = np.zeros(blue.shape, np.uint8)
    cv2.drawContours(filled, [cnt], -1, 255, -1)
    hole = cv2.subtract(filled, blue)[y:y + h, x:x + w]
    if int((hole > 0).sum()) < cfg.arrow_min_px:
        return None                                     # no readable arrow
    ys, xs = np.nonzero(hole)
    upper, lower = xs[ys < h / 2], xs[ys >= h / 2]
    if len(upper) < 5 or len(lower) < 5:
        return None
    lean = (upper.mean() - lower.mean()) / w
    if abs(lean) < cfg.arrow_margin:                    # dead centre -> refuse to guess
        return None
    return 3 if lean > 0 else 4


def detect_sign(bgr, cfg, hsv=None):
    if hsv is None:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue = _mask(hsv, cfg.blue_h[0], cfg.blue_h[1], cfg.blue_s_min, cfg.blue_v_min)
    # close the ring before contouring: a broken blue edge leaks the arrow hole out to
    # the background, and the hole IS the direction signal. Also recovers sign frames the
    # circularity gate used to drop (386 blue circles found vs 355 without).
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < cfg.sign_min_area or _circularity(cnt) < cfg.sign_min_circ:
            continue
        direction = _arrow_direction(blue, cnt, cfg)
        if direction is not None:
            if cfg.sign_invert:                          # venue escape hatch: swap R<->L
                direction = 4 if direction == 3 else 3
            out.append((direction, float(min(1.0, _circularity(cnt))),
                        tuple(int(v) for v in cv2.boundingRect(cnt))))
    return out


# ==========================================================================
# Debug: loose red/green candidate scan (for live HSV tuning -> CSV)
# ==========================================================================
def light_candidates(bgr, cfg, top_k=4):
    """LOOSE red/green blob scan so a dim / colour-shifted lamp still shows up even when
    detect_light misses it. Skips noise (<12px) and huge background (> the light max-area).
    Returns the top_k blobs per colour by area, each with the exact quantities the detector
    gates on -- hue/sat/val (colour), area/circ (shape) -- so the venue tuner reads the same
    numbers the car will."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    k = np.ones((3, 3), np.uint8)
    max_area = cfg.light_max_area_frac * h * w
    bands = {
        'RED': cv2.bitwise_or(cv2.inRange(hsv, (0, 40, 40), (18, 255, 255)),
                              cv2.inRange(hsv, (160, 40, 40), (180, 255, 255))),
        'GREEN': cv2.inRange(hsv, (35, 40, 40), (95, 255, 255)),
    }
    out = []
    for color, m in bands.items():
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < 12 or area > max_area:            # skip noise + huge bg (mat/floor)
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            sel = m[y:y + bh, x:x + bw] > 0
            blob = hsv[y:y + bh, x:x + bw]
            blobs.append({
                'color': color,
                'hue': int(np.median(blob[:, :, 0][sel])),
                'sat': int(np.median(blob[:, :, 1][sel])),
                'val': int(np.median(blob[:, :, 2][sel])),   # a LIT lamp is bright: this is
                                                             # what separates red from red paint
                'circ': round(_circularity(cnt), 2),         # and this is what separates green
                'area': int(area),
                'x': int(x), 'y': int(y), 'w': int(bw), 'h': int(bh),
            })
        blobs.sort(key=lambda d: d['area'], reverse=True)
        out.extend(blobs[:top_k])
    return out


def sign_candidates(bgr, cfg, top_k=4):
    """LOOSE blue-sign blob scan for OPTIONAL venue tuning (opt-in via mission_node
    log_signs:=true). A sign is a large solid PRINTED blue disc (a reflector, not an
    emitter), so brightness means nothing here -- this just reports its HSV, size and
    roundness, the same CSV schema as the lamps.

    NOTE: this grounds sign DETECTION (the blue mask) only, NEVER the arrow DIRECTION
    (_arrow_direction). After tuning blue, still dwell a RIGHT/LEFT sign and confirm the
    printed class is correct."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(cv2.inRange(hsv, (90, 40, 40), (135, 255, 255)),
                         cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 40:                                   # skip small blue noise
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        sel = m[y:y + bh, x:x + bw] > 0
        blob = hsv[y:y + bh, x:x + bw]
        blobs.append({
            'color': 'BLUE',
            'hue': int(np.median(blob[:, :, 0][sel])),
            'sat': int(np.median(blob[:, :, 1][sel])),
            'val': int(np.median(blob[:, :, 2][sel])),
            'circ': round(_circularity(cnt), 2),
            'area': int(area),
            'x': int(x), 'y': int(y), 'w': int(bw), 'h': int(bh),
        })
    blobs.sort(key=lambda d: d['area'], reverse=True)
    return blobs[:top_k]


# ==========================================================================
# Detector with temporal debounce
# ==========================================================================
class MissionDetector:
    def __init__(self, cfg=None):
        self.cfg = cfg or MissionCfg()
        self._aruco = _make_aruco(self.cfg)
        self._hist = deque(maxlen=max(1, self.cfg.confirm_n))
        self._confirmed = None

    def process(self, bgr, hsv=None, gray=None):
        """Return (detections_this_frame, confirmed_cls_or_None, newly_confirmed).

        `hsv` / `gray` are optional precomputed conversions of THIS frame (the merged
        perception node already has them). Omit them and each detector converts for
        itself -- same answer, one more pass over the frame.
        """
        dets = []
        dets += detect_aruco(bgr, self.cfg, self._aruco, gray)
        dets += detect_light(bgr, self.cfg, hsv)
        dets += detect_sign(bgr, self.cfg, hsv)

        top = max(dets, key=lambda d: d[1], default=None)
        cls_now = top[0] if (top and top[1] >= self.cfg.min_conf) else None
        self._hist.append(cls_now)

        # M-of-N vote over the last confirm_n PROCESSED frames (tolerates dropped frames,
        # unlike strict N-in-a-row). Sticky until another class wins or the window empties.
        #
        # STOP classes (RED, MARK) confirm at a LOWER threshold than GO, and are checked
        # first, because the two errors are not symmetric: a false stop costs seconds, a
        # missed stop hits the obstacle or runs the light. The asymmetry matters most for
        # the ArUco marker, which the car passes quickly -- at frame_skip=2 a 3-of-5 vote
        # needs the marker held for ~15 camera frames (0.5s at 30Hz), and a car that drives
        # past it faster than that never stops at all.
        counts = Counter(c for c in self._hist if c is not None)
        newly = False
        if counts:
            stop = [(counts[c], c) for c in (STOP_CLASSES) if counts[c] >= self.cfg.confirm_m_stop]
            if stop:
                cls = max(stop)[1]
            else:
                cls, hits = counts.most_common(1)[0]
                cls = cls if hits >= self.cfg.confirm_m else None
            if cls is not None:
                newly = (cls != self._confirmed)
                self._confirmed = cls
        else:                                 # window empty -> nothing in view
            self._confirmed = None
        return dets, self._confirmed, newly


# ==========================================================================
# Stop-and-go gate (pure state machine; ROS-independent)
# ==========================================================================
class MissionGate:
    """Traffic-light stop-and-go, ported from the eun2 lane_follow gate.

    Consumes the confirmed mission class (0 GREEN / 1 RED / 2 MARK / other=none)
    plus a monotonic time, and returns whether throttle is allowed. It ONLY ever
    withholds throttle (returns False) -- it never commands motion. The control
    node applies it as a final throttle gate on top of engage / E-stop / conf.

      GREEN -> go (clears a red stop)
      RED   -> stop; auto-resumes after `resume_s` (or sooner when green returns).
               A false red mid-race must not strand the car (a long stall loses
               the run on the 6-min cap), so the timeout resume stays.
      MARK  -> stop while the barrier is DOWN, released `hold_s` after it was last
               seen. Does not latch the green/red running state.

    Why MARK needs a hold instead of "stop while detected": the marker is a barrier
    that rises and falls, and the car must stay stopped the entire time it is down.
    But ArUco only fires when all four corners are inside the frame, so as the car
    closes in, the marker clips at the border and drops out -- on the one marker clip
    we have it is seen in 29% of frames, with 600ms and 800ms gaps WHILE IT WAS STILL
    DOWN. Releasing on the first gap made the car lurch: stop 0.47s, go 0.37s, stop
    0.57s, go. The hold bridges the gaps, so one sighting buys a clean stop.

    `hold_s` is in SECONDS on purpose. The detector's own debounce counts PROCESSED
    frames, so it stretches and shrinks with frame_skip and with whatever frame rate
    the car actually achieves; a barrier does not care. Raising frame_skip or dropping
    fps at speed cannot shorten this hold.

    Starts stopped (waits for the first green). Race completion is NOT handled
    here -- red is not a permanent finish.

    update() returns (allow_throttle, event) where event is a short string on a
    state transition (for the node to log) or None.
    """
    def __init__(self, resume_s=30.0, start_gated=True, mark_hold_s=1.5):
        self.resume_s = float(resume_s)
        self.mark_hold_s = float(mark_hold_s)
        self.running = not start_gated
        self.stop_t = None
        self.mark_until = None                    # None = barrier is up / never seen

    def update(self, cls, t):
        event = None
        if cls == 0:                              # GREEN -> go (clears a red stop)
            if not self.running:
                event = 'GREEN -> GO'
            self.running = True
            self.stop_t = None
        elif cls == 1:                            # RED -> stop (latched; resumes on green/timeout)
            if self.running:
                event = 'RED -> STOP'
                self.stop_t = t
            self.running = False
        # failsafe: resume without a green once the stop has lasted long enough
        if (not self.running and self.stop_t is not None
                and (t - self.stop_t) >= self.resume_s):
            event = 'RED stop timeout -> GO'
            self.running = True
            self.stop_t = None

        if cls == 2:                              # marker seen -> (re)arm the hold
            if self.mark_until is None:
                event = 'MARK -> STOP'
            self.mark_until = t + self.mark_hold_s
        if self.mark_until is not None:
            if t < self.mark_until:
                return False, event               # barrier down (or within the hold)
            self.mark_until = None                # hold expired -> barrier is really up
            event = event or 'MARK cleared -> GO'
        return self.running, event


def annotate(bgr, dets):
    """Boxes in CAMERA pixels, coloured by class -- so a glance says WHICH object, not just
    that something fired. Font 0.35: the frame is 320x240, and 0.5 buried the box it labels."""
    img = bgr.copy()
    for cls_id, conf, (x, y, w, h) in dets:
        col = CLASS_COLORS.get(cls_id, (0, 255, 0))
        cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
        cv2.putText(img, f'{CLASS_NAMES.get(cls_id, "?")} {conf:.2f}',
                    (x, max(10, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    return img
