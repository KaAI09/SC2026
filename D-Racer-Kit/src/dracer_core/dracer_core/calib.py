"""Camera calibration + BEV. Pure (ROS-independent). Single source of truth.

The ONE thing this module owns: turning raw camera pixels into a metric top-down
(BEV) view of the ground plane, so every stage above it can reason in cm instead
of in a perspective-warped image.

    from dracer_core.calib import CameraModel
    cam = CameraModel.load('src/config/camera.yaml')
    bev_mask = cam.to_bev(mask)              # raw px -> BEV px (undistort + warp, ONE remap)
    pts_img  = cam.bev_to_image(pts_bev)     # BEV px -> raw px (for drawing on the frame)
    x_cm     = cam.bev_x_to_cm(u)            # BEV px -> lateral cm (+ = right of vehicle axis)

WHY THIS IS A SEPARATE MODULE (reuse across tracks):

    K, D  (lens distortion)  depend on the LENS   -> survive re-aiming AND a new track
    H     (homography)       depends on the MOUNT -> re-shoot ONE ground photo if re-aimed
    lane width, colours      depend on the TRACK  -> live in the track profile, NOT here

So `camera.yaml` knows nothing about any track, and a track profile knows nothing
about the camera. At the next venue only the track profile is rewritten; if the
camera was re-aimed, `calibrate.py --ground` refreshes H from a single photo.

WHY K AND D AT ALL (vs. clicking 4 points on a lane image):
  A 4-point homography with no undistortion ABSORBS the lens distortion: it fits
  near those 4 points and skews everywhere else -- worst at the frame edges, which
  is exactly where a lane sits right before it leaves the FOV (i.e. where `coast`
  happens). Chessboard corners are sub-pixel and over-determined; hand clicks are
  +-2..3 px and get amplified by perspective (1 px near the top ~ tens of cm).

COORDINATES
  ground : x = lateral cm, + is RIGHT of the vehicle axis;  y = forward cm from the camera.
  BEV    : u = (x + x_half) * px_per_cm            (u grows rightward)
           v = bev_h - 1 - (y - y_near) * px_per_cm  (v grows DOWNWARD = toward the vehicle)
  So the BEV bottom edge is the NEAREST ground row and the image is metric and
  isotropic: a lane is the same width in px at the top and at the bottom. That is
  what makes `perception_core._shift` (a constant-offset parallel shift) physically
  valid -- in the front view it is not, and that is limitation §6(c) in PERCEPTION.md.

This module NEVER commands the vehicle and never detects a lane; it only changes
coordinates.
"""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np
import yaml


@dataclass
class CameraModel:
    """Calibrated camera + ground homography + BEV grid definition."""

    image_size: tuple            # (w, h) the K/D/H were calibrated AT -- must match runtime
    K: np.ndarray                # 3x3 intrinsics
    D: np.ndarray                # distortion coeffs (k1 k2 p1 p2 k3)
    H: np.ndarray                # 3x3: UNDISTORTED image px -> BEV px
    px_per_cm: float             # BEV scale (isotropic)
    x_half_cm: float             # BEV covers lateral [-x_half, +x_half]
    y_near_cm: float             # BEV bottom row = this far in front of the camera
    y_far_cm: float              # BEV top row
    lateral_offset_cm: float = 0.0   # + shifts the vehicle axis RIGHT (mount not centred)
    rms_px: float = float('nan')     # intrinsics reprojection error (quality record)
    ground_rms_px: float = float('nan')   # homography reprojection error
    square_mm: float = float('nan')       # MEASURED checker size used (not the nominal one)
    _maps: tuple = field(default=None, repr=False, compare=False)

    # ---------------------------------------------------------------- geometry
    @property
    def bev_size(self):
        w = int(round(2 * self.x_half_cm * self.px_per_cm))
        h = int(round((self.y_far_cm - self.y_near_cm) * self.px_per_cm))
        return w, h

    @property
    def axis_u(self):
        """BEV column of the vehicle axis (what center_error is measured against)."""
        return (self.x_half_cm + self.lateral_offset_cm) * self.px_per_cm

    def bev_x_to_cm(self, u):
        """BEV column -> lateral cm from the vehicle axis (+ = right)."""
        return (np.asarray(u, float) - self.axis_u) / self.px_per_cm

    def bev_y_to_cm(self, v):
        """BEV row -> forward cm from the camera."""
        h = self.bev_size[1]
        return self.y_near_cm + (h - 1 - np.asarray(v, float)) / self.px_per_cm

    def cm_to_bev(self, x_cm, y_cm):
        """ground cm -> BEV px."""
        h = self.bev_size[1]
        u = self.axis_u + np.asarray(x_cm, float) * self.px_per_cm
        v = (h - 1) - (np.asarray(y_cm, float) - self.y_near_cm) * self.px_per_cm
        return u, v

    def lane_width_px(self, lane_width_cm):
        """Track lane width (cm, from the TRACK profile) -> BEV px. Constant at every
        row -- the whole point of the BEV. Feeds the pairing gate + coast shift."""
        return lane_width_cm * self.px_per_cm

    # ---------------------------------------------------------------- transforms
    def _build_maps(self):
        """ONE remap: BEV px -> undistorted px -> DISTORTED (raw) px.

        Composing undistort and warp into a single LUT means a frame is resampled
        once, not twice (less blur, ~half the cost). Built lazily, cached.
        """
        bw, bh = self.bev_size
        uu, vv = np.meshgrid(np.arange(bw, dtype=np.float32),
                             np.arange(bh, dtype=np.float32))
        bev_pts = np.stack([uu, vv], -1).reshape(-1, 1, 2)
        # BEV -> undistorted image px
        und = cv2.perspectiveTransform(bev_pts, np.linalg.inv(self.H)).reshape(-1, 2)
        # undistorted px -> normalized camera coords -> re-apply distortion -> raw px
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        xn = (und[:, 0] - cx) / fx
        yn = (und[:, 1] - cy) / fy
        obj = np.stack([xn, yn, np.ones_like(xn)], -1).astype(np.float64)
        raw, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), self.K, self.D)
        raw = raw.reshape(bh, bw, 2).astype(np.float32)
        self._maps = (np.ascontiguousarray(raw[..., 0]), np.ascontiguousarray(raw[..., 1]))
        return self._maps

    def to_bev(self, img, nearest=True):
        """Raw camera image (or a mask) -> BEV. Undistort + warp in one resample.

        Masks MUST use nearest (default) so a binary mask stays binary.
        """
        mx, my = self._maps or self._build_maps()
        return cv2.remap(img, mx, my,
                         cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def bev_to_image(self, pts_bev):
        """BEV px -> RAW (distorted) image px. For drawing BEV results on the frame."""
        pts = np.asarray(pts_bev, np.float32).reshape(-1, 1, 2)
        und = cv2.perspectiveTransform(pts, np.linalg.inv(self.H)).reshape(-1, 2)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        xn = (und[:, 0] - cx) / fx
        yn = (und[:, 1] - cy) / fy
        obj = np.stack([xn, yn, np.ones_like(xn)], -1).astype(np.float64)
        raw, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), self.K, self.D)
        return raw.reshape(-1, 2)

    def bev_coeffs_to_image(self, coeffs, y_lo, y_hi, step=2.0):
        """A BEV lane polynomial x=f(v) -> polyline in RAW image px (for the overlay)."""
        vs = np.arange(float(y_lo), float(y_hi) + 1.0, step)
        us = coeffs[0] * vs * vs + coeffs[1] * vs + coeffs[2]
        return self.bev_to_image(np.stack([us, vs], -1))

    # ---------------------------------------------------------------- validate
    def validate(self, lanes, lane_width_cm, tol=0.20):
        """Is this calibration still good? Run on a STRAIGHT stretch with both lanes.

        The mount gets nudged in normal use, which silently invalidates H (K/D survive).
        A stale H shows up as a wrong steering command that looks like a control bug.
        This turns that into an explicit check: in a metric BEV, a straight stretch's
        two lane boundaries must be (1) vertical, (2) parallel, (3) exactly one lane
        width apart.

        `lanes`: >=2 instances from perception_core.sliding_window_lanes run ON THE BEV.
        Returns a dict; `ok` False means: re-shoot the ground photo (calibrate.py --ground).
        """
        fits = sorted([x for x in lanes if x['coeffs'] is not None],
                      key=lambda x: x['x_bottom'])
        if len(fits) < 2:
            return {'ok': False, 'reason': f'need 2 lanes on a straight, got {len(fits)}'}
        a, b = fits[0], fits[-1]
        v_lo = max(float(a['ys'].min()), float(b['ys'].min()))
        v_hi = min(float(a['ys'].max()), float(b['ys'].max()))
        if v_hi - v_lo < 1.0:
            return {'ok': False, 'reason': 'lanes do not overlap vertically'}
        vs = np.linspace(v_lo, v_hi, 9)

        def ev(c, v):
            return c[0] * v * v + c[1] * v + c[2]

        xa, xb = ev(a['coeffs'], vs), ev(b['coeffs'], vs)
        gaps = xb - xa
        expect = self.lane_width_px(lane_width_cm)
        # (1) vertical: a straight lane must not drift sideways up the BEV
        skew_px = max(abs(xa[0] - xa[-1]), abs(xb[0] - xb[-1]))
        # (2) parallel: the gap must not open or close
        spread_px = float(gaps.max() - gaps.min())
        # (3) metric: the gap must equal the real lane width
        width_err = float(gaps.mean() - expect)
        ok = (abs(width_err) <= tol * expect and spread_px <= tol * expect
              and skew_px <= tol * expect)
        return {
            'ok': bool(ok),
            'width_px': float(gaps.mean()), 'width_expected_px': float(expect),
            'width_err_cm': width_err / self.px_per_cm,
            'parallel_spread_cm': spread_px / self.px_per_cm,
            'vertical_skew_cm': float(skew_px) / self.px_per_cm,
            'reason': '' if ok else 'stale H (camera re-aimed?) or wrong lane_width_cm',
        }

    # ---------------------------------------------------------------- rescale
    def rescale(self, new_size):
        """Move this model to another capture resolution. EXACT, not an approximation.

        WHY THIS EXISTS: at the 320x160 runtime resolution a chessboard laid on the
        GROUND is barely detectable -- perspective squashes it and the 2:1 anamorphic
        scale squashes it again, leaving ~7-11 px per square (a sweep found corners in
        only 2 of 24 tilt/distance combos). So we CALIBRATE at a high resolution, where
        corners are easy and sub-pixel accurate, then rescale to the runtime size.

        This is exact because the GStreamer pipeline is a pure rescale of the same
        sensor frame (no crop, no letterbox): both resolutions come from one native
        frame, so 640x480 -> 320x160 is exactly (sx, sy) = (0.5, 1/3) regardless of
        what the native size is.

            K scales:   fx,cx by sx ; fy,cy by sy
            D is INVARIANT: distortion acts on normalized coords, and
                            (u'-cx')/fx' = (sx*u - sx*cx)/(sx*fx) = (u-cx)/fx
            H composes:  H_new = H_old @ S^-1   (H maps UNDISTORTED px -> BEV px)

        The BEV grid (px_per_cm, extents) is in GROUND cm and does not change.
        """
        sx = float(new_size[0]) / float(self.image_size[0])
        sy = float(new_size[1]) / float(self.image_size[1])
        S = np.diag([sx, sy, 1.0])
        m = CameraModel(
            image_size=(int(new_size[0]), int(new_size[1])),
            K=S @ self.K, D=self.D.copy(), H=self.H @ np.linalg.inv(S),
            px_per_cm=self.px_per_cm, x_half_cm=self.x_half_cm,
            y_near_cm=self.y_near_cm, y_far_cm=self.y_far_cm,
            lateral_offset_cm=self.lateral_offset_cm,
            rms_px=self.rms_px, ground_rms_px=self.ground_rms_px,
            square_mm=self.square_mm)
        return m

    # ---------------------------------------------------------------- io
    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
        cam, bev = d.get('camera', {}), d.get('bev', {})
        m = cls(
            image_size=tuple(cam['image_size']),
            K=np.array(cam['K'], float).reshape(3, 3),
            D=np.array(cam['D'], float).ravel(),
            H=np.array(bev['H'], float).reshape(3, 3),
            px_per_cm=float(bev['px_per_cm']),
            x_half_cm=float(bev['x_half_cm']),
            y_near_cm=float(bev['y_near_cm']),
            y_far_cm=float(bev['y_far_cm']),
            lateral_offset_cm=float(bev.get('lateral_offset_cm', 0.0)),
            rms_px=float(cam.get('rms_px', float('nan'))),
            ground_rms_px=float(bev.get('ground_rms_px', float('nan'))),
            square_mm=float(cam.get('square_mm', float('nan'))),
        )
        return m

    def save(self, path, note=''):
        doc = {
            'note': note or ('Camera calibration. TRACK-INDEPENDENT: reuse at any venue. '
                             'K/D survive re-aiming; re-run `calibrate.py --ground` if the '
                             'camera mount moved. Lane width / colours live in the TRACK profile.'),
            'camera': {
                'image_size': [int(self.image_size[0]), int(self.image_size[1])],
                'K': [float(x) for x in self.K.ravel()],
                'D': [float(x) for x in self.D.ravel()],
                'rms_px': float(self.rms_px),
                'square_mm': float(self.square_mm),
            },
            'bev': {
                'H': [float(x) for x in self.H.ravel()],
                'px_per_cm': float(self.px_per_cm),
                'x_half_cm': float(self.x_half_cm),
                'y_near_cm': float(self.y_near_cm),
                'y_far_cm': float(self.y_far_cm),
                'lateral_offset_cm': float(self.lateral_offset_cm),
                'ground_rms_px': float(self.ground_rms_px),
            },
        }
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)

    def summary(self):
        bw, bh = self.bev_size
        return (f"CameraModel {self.image_size[0]}x{self.image_size[1]} "
                f"rms={self.rms_px:.3f}px ground_rms={self.ground_rms_px:.3f}px | "
                f"BEV {bw}x{bh}px @ {self.px_per_cm:.2f}px/cm  "
                f"x=+-{self.x_half_cm:.0f}cm y={self.y_near_cm:.0f}..{self.y_far_cm:.0f}cm")


# ==========================================================================
# calibration (used by offline/calibrate.py; kept here so the math has ONE home)
# ==========================================================================
def find_corners(gray, pattern):
    """Sub-pixel chessboard corners, or None. `pattern` = INNER corners (cols, rows)."""
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    return cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))


def object_grid(pattern, square_mm):
    """Chessboard corners in board coords (mm), z=0. Row-major, matching OpenCV order."""
    cols, rows = pattern
    g = np.zeros((cols * rows, 3), np.float32)
    g[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)
    return g


def calibrate_intrinsics(images, pattern, square_mm):
    """K, D from N chessboard views. Returns (K, D, rms, n_used, size)."""
    objp = object_grid(pattern, square_mm)
    obj_pts, img_pts, size = [], [], None
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        size = gray.shape[::-1]
        c = find_corners(gray, pattern)
        if c is not None:
            obj_pts.append(objp)
            img_pts.append(c)
    if len(obj_pts) < 5:
        raise RuntimeError(f'need >=5 usable views, got {len(obj_pts)}')
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    return K, D.ravel(), float(rms), len(obj_pts), size


def ground_homography(img, K, D, pattern, square_mm, near_cm, lateral_cm=0.0):
    """H (undistorted px -> BEV px) from ONE photo of the board LYING ON THE GROUND.

    The board defines the ground plane AND the metric scale. Placement contract:
      * board laid flat on the track surface, roughly square to the vehicle,
      * `near_cm`   = forward distance (cm) from the camera to the board's NEAREST
                      corner row  -- measure it with a tape,
      * `lateral_cm`= lateral offset (cm) of the board's centre from the vehicle axis
                      (0 if you centred it; + = board sits to the RIGHT).

    Returns (H_partial, ground_pts_cm, img_pts_und, rms) where H_partial maps
    undistorted px -> GROUND cm. The caller turns cm -> BEV px once the grid is chosen.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners = find_corners(gray, pattern)
    if corners is None:
        raise RuntimeError('no chessboard found in the ground photo')
    und = cv2.undistortPoints(corners, K, D, P=K).reshape(-1, 2)

    cols, rows = pattern
    sq_cm = square_mm / 10.0
    # board grid -> ground cm. Corner order must match cv2.findChessboardCorners:
    # ROW-MAJOR (index k = r*cols + c), same as `object_grid`. Board is centred on
    # `lateral_cm`; its FIRST corner row is the FAR one, its LAST row sits at `near_cm`.
    gx, gy = np.meshgrid(np.arange(cols), np.arange(rows))   # gx[r][c] = c, gy[r][c] = r
    x_cm = (gx.ravel() - (cols - 1) / 2.0) * sq_cm + lateral_cm
    y_cm = near_cm + (rows - 1 - gy.ravel()) * sq_cm
    ground = np.stack([x_cm, y_cm], -1).astype(np.float32)

    Hg, _ = cv2.findHomography(und, ground, method=0)   # exact LS over all corners
    proj = cv2.perspectiveTransform(und.reshape(-1, 1, 2), Hg).reshape(-1, 2)
    rms = float(np.sqrt(((proj - ground) ** 2).sum(axis=1).mean()))
    return Hg, ground, und, rms


def ground_pose(img, K, D, pattern, square_mm):
    """H from ONE photo of the board lying on the ground -- WITHOUT measuring near_cm.

    The board IS the ground plane, so solvePnP recovers where the camera sits relative
    to it, and the "distance to the board" falls out of the math. That removes the one
    measurement that is genuinely hard to take by hand: you cannot see the camera's
    optical centre, nor precisely mark its footprint on the floor, nor tell the first
    INNER corner from the board's edge. Measuring it gave Cy = -3.2 cm / +2.0 cm for the
    two plausible readings -- i.e. both were wrong, since Cy must be 0 by definition.

    Ground frame (the definition every stage above depends on):
        origin = the camera's optical centre projected straight down onto the ground
        +y     = forward  (the optical axis projected onto the ground)
        +x     = right
    Solving for it is not circular: solvePnP fixes the camera-to-board pose from the
    corners alone; we then merely CHOOSE to put the origin under the camera.

    Returns (H_ground, cam_height_cm, near_cm, lateral_cm, rms_cm) where H_ground maps
    UNDISTORTED px -> ground cm, and near_cm/lateral_cm are REPORTED (measured for you),
    not required.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners = find_corners(gray, pattern)
    if corners is None:
        raise RuntimeError('no chessboard found in the ground photo')
    objp = object_grid(pattern, square_mm)                  # board coords (mm), z=0
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError('solvePnP failed on the ground photo')

    R, _ = cv2.Rodrigues(rvec)                             # board -> camera
    C = (-R.T @ tvec).ravel()                              # camera centre, BOARD coords (mm)
    axis = R.T @ np.array([0.0, 0.0, 1.0])                 # optical axis, board coords
    height_mm = abs(float(C[2]))

    fwd = np.array([axis[0], axis[1]], float)              # optical axis on the board plane
    nf = np.linalg.norm(fwd)
    if nf < 1e-6:
        raise RuntimeError('camera looks straight down -- forward direction undefined')
    fwd /= nf
    right = np.array([fwd[1], -fwd[0]])                    # +90 deg from forward
    cam_right = (R.T @ np.array([1.0, 0.0, 0.0]))[:2]      # image-right on the board plane
    if float(right @ cam_right) < 0:
        right = -right

    rel = objp[:, :2] - C[:2]                              # corners relative to the footprint
    ground = np.stack([(rel @ right) / 10.0, (rel @ fwd) / 10.0], -1).astype(np.float32)

    und = cv2.undistortPoints(corners, K, D, P=K).reshape(-1, 2)
    Hg, _ = cv2.findHomography(und, ground, method=0)
    proj = cv2.perspectiveTransform(und.reshape(-1, 1, 2), Hg).reshape(-1, 2)
    rms = float(np.sqrt(((proj - ground) ** 2).sum(axis=1).mean()))
    return (Hg, height_mm / 10.0, float(ground[:, 1].min()),
            float(ground[:, 0].mean()), rms)


def ground_extent(Hg, K, D, image_size, margin=0.02):
    """Where on the ground can this camera actually see? -> (x_half, y_near, y_far) cm.

    Projects the image border into ground cm. Rows near the horizon explode to
    infinity, so we clip to the finite part and trim a margin.
    """
    w, h = image_size
    ys = np.linspace(h * 0.35, h - 1, 40)          # skip the sky/horizon band
    xs = np.linspace(0, w - 1, 40)
    uu, vv = np.meshgrid(xs, ys)
    pts = np.stack([uu.ravel(), vv.ravel()], -1).astype(np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, D, P=K)
    g = cv2.perspectiveTransform(und, Hg).reshape(-1, 2)
    g = g[np.isfinite(g).all(axis=1)]
    g = g[(g[:, 1] > 1.0) & (g[:, 1] < 1000.0)]    # in front of the camera, finite
    if len(g) < 10:
        raise RuntimeError('ground extent degenerate -- bad H?')
    x_half = float(np.percentile(np.abs(g[:, 0]), 95) * (1 - margin))
    y_near = float(np.percentile(g[:, 1], 2))
    y_far = float(np.percentile(g[:, 1], 95) * (1 - margin))
    return x_half, y_near, y_far


def build_model(K, D, Hg, image_size, px_per_cm, x_half, y_near, y_far,
                lateral_offset_cm=0.0, rms=float('nan'), ground_rms=float('nan'),
                square_mm=float('nan')):
    """Compose the ground homography with the chosen BEV grid -> CameraModel."""
    m = CameraModel(image_size=tuple(image_size), K=np.asarray(K, float),
                    D=np.asarray(D, float).ravel(), H=np.eye(3),
                    px_per_cm=float(px_per_cm), x_half_cm=float(x_half),
                    y_near_cm=float(y_near), y_far_cm=float(y_far),
                    lateral_offset_cm=float(lateral_offset_cm),
                    rms_px=float(rms), ground_rms_px=float(ground_rms),
                    square_mm=float(square_mm))
    bh = m.bev_size[1]
    # ground cm -> BEV px  (u = (x + x_half)*s ; v = (bh-1) - (y - y_near)*s)
    s = float(px_per_cm)
    G = np.array([[s, 0.0, (x_half + lateral_offset_cm) * s],
                  [0.0, -s, (bh - 1) + y_near * s],
                  [0.0, 0.0, 1.0]], float)
    m.H = G @ np.asarray(Hg, float)
    m._maps = None
    return m
