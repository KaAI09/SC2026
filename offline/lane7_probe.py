"""7-라벨 차선 인지 오프라인 실험 도구 (배포 아님, 로컬 클립 테스트용).

파이프라인 (제어 없음, 유한 라벨링으로 끝):
  ① 검출   : HSV 흰/노랑 마스크 + morphology + 색 지배 게이트  (그대로 유지)
  ② BEV    : ROI 사다리꼴 → 탑다운 워프 (원근 제거, 차선이 수직·평행)
  ③ 분리   : 하단 base 탐색 → base부터 위로 sliding-window 쌓기.
             연속 sw_max_miss(기본 3)회 빈 창 → 차선 종료.
             세로로 충분히 못 쌓은 궤적(정지선·노이즈·검출실패)은 탈락 → 별도 라벨 없음.
  ④ 분류   : 색분기(흰/노랑) → 좌/우는 위치(base x)와 방향(궤적 추세)을 함께 투표,
             노랑은 곡률로 직진S/좌L/우R → {W-L/R, YS/YL/YR-L/R}
  (감쌈) 기억: ego 좌/우 계수 EMA + per-side coast

sliding-window가 blob이 아니라 "궤적"을 따라가므로:
  - 점선: 창이 간격을 건너뛰며 한 차선으로 이어줌
  - 정지선/가로바: base는 잡혀도 위로 못 쌓음 → 세로 span 미달로 탈락 (라벨 안 붙임)
  - X 교차: 한 가지의 연속만 추종

사용:
  ../.venv/bin/python lane7_probe.py "Dashcam(2025 Track)/070408.mp4" --stages --name t408
"""
import argparse
import os
from dataclasses import dataclass

import cv2
import numpy as np


# ============================ config ============================
@dataclass
class Cfg:
    # 색 밴드 (2025 대시캠 기준 초기값)
    white_s_max: int = 60
    white_v_min: int = 185
    y_h_lo: int = 18
    y_h_hi: int = 40
    y_s_min: int = 55
    y_v_min: int = 90
    # ROI = BEV 소스 사다리꼴
    roi_top: float = 0.20
    trap_top: float = 0.75
    trap_bot: float = 1.0
    # 검출 후처리
    morph_v: int = 5             # 세로 close 커널 (점선 잇기)
    color_gate: float = 0.15     # 소수 색 비율 < 이 값 → 통째 제거
    gate_min_px: int = 80
    # 특징 / 분류
    straight_thresh: float = 0.0006   # |a| < 이 값 → 직진(트래커 곡률용)
    heading_frac: float = 0.06        # |상단-하단 x| ≥ 이 비율×W → 그 방향(heading)으로 L/R
    curv_strong: float = 0.0015       # heading 약해도 |a| ≥ 이 값이면 곡률 부호로 L/R
    # 중앙선 쌍 성립(나란한 좌/우 경계만 짝지음)
    pair_overlap_min: float = 0.30    # 두 차선 y겹침 ≥ 이 비율×H 이어야 쌍 성립
    pair_gap_min: float = 12.0        # 겹침 내 최소 간격(px). 이하로 붕괴 = 교차 → 기각
    #   (원근으로 수렴하는 평행 차선도 간격이 0까지 안 가므로 gap_min 하나로 X교차만 걸림)
    # sliding window (BEV 좌표)
    sw_nwin: int = 12            # 세로 창 개수
    sw_margin: int = 28          # 창 반폭(px)
    sw_minpix: int = 18          # 창 내 최소 픽셀(중심 갱신 기준)
    sw_max_miss: int = 3         # 연속 이만큼 빈 창 → 차선 종료(검출실패/정지선)
    sw_dir_ema: float = 0.6      # 창당 좌우 이동량(방향/곡률) 갱신 EMA (곡선 추종)
    sw_min_hits: int = 2         # 성공 창이 이보다 적으면 차선 아님 → 탈락
    sw_min_span: float = 0.30    # 세로 span < 이 비율×H → 차선 아님(정지선 등) → 탈락
    sw_max_lanes: int = 3        # 색당 최대 base 수
    sw_peak_min: int = 10         # 히스토그램 최소 피크(행 수)
    sw_peak_sep: int = 45        # 피크 최소 간격(px)
    sw_merge_min: int = 8        # 창이 이만큼 이상 겹치면 같은 차선 → 병합
    sw_merge_iou: float = 0.30   # 같은 높이 창끼리 x구간 IOU ≥ 이 값 → "겹침" 1회
    # 추적
    ema: float = 0.4
    jump_max: float = 120.0
    lost_reset: int = 8
    lane_width_default: float = 0.5


LABEL_COLORS = {  # BGR — 색상 대비 극대화(주황/마젠타/초록으로 3방향 명확 구분)
    'W-L': (255, 255, 255), 'W-R': (170, 170, 170),   # 흰/회색
    'YR-L': (0, 140, 255), 'YR-R': (0, 90, 200),      # 주황 (우회전)
    'YL-L': (255, 0, 200), 'YL-R': (200, 0, 150),     # 마젠타 (좌회전)
    'YS-L': (0, 230, 0), 'YS-R': (0, 150, 0),         # 초록 (직진)
}


# ============================ ① 검출 ============================
def detect(frame, c):
    """HSV 흰/노랑 마스크 (전체 프레임, ROI 크롭은 BEV가 담당)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    white = ((S <= c.white_s_max) & (V >= c.white_v_min)).astype(np.uint8) * 255
    yellow = ((H >= c.y_h_lo) & (H <= c.y_h_hi) &
              (S >= c.y_s_min) & (V >= c.y_v_min)).astype(np.uint8) * 255
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


# ============================ ② BEV ============================
def _src_trapezoid(h, w, c):
    y0 = int(h * c.roi_top)
    cx = w / 2.0
    tw, bw = c.trap_top * w, c.trap_bot * w
    return np.float32([[cx - bw / 2, h - 1], [cx - tw / 2, y0],
                       [cx + tw / 2, y0], [cx + bw / 2, h - 1]])


def bev_transform(h, w, c):
    """ROI 사다리꼴 → (w,h) 직사각형. 반환 M, Minv (BEV 크기 = 프레임 크기)."""
    src = _src_trapezoid(h, w, c)
    dst = np.float32([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]])
    return cv2.getPerspectiveTransform(src, dst), cv2.getPerspectiveTransform(dst, src)


# ============================ 특징 빌드 ============================
def _build_instance(ys, xs, color, c, W):
    """분리에서 통과한 궤적 → 2차 피팅으로 base 위치·궤적 추세·방향(turn).
    turn: heading(상단-하단 x변위) 우선, 거의 수직이면 곡률 a 보조. (+1=우/R, -1=좌/L, 0=직진/S)"""
    ins = {'color': color, 'xs': xs, 'ys': ys, 'coeffs': None,
           'x_bottom': float(xs[np.argmax(ys)]),  # base(하단) x
           'x_mean': float(xs.mean()),            # 궤적 추세(방향) x
           'turn': 0}
    if np.unique(ys).size >= 5:
        a, b, cc = np.polyfit(ys.astype(float), xs.astype(float), 2)
        yb, yt = float(ys.max()), float(ys.min())
        ins['coeffs'] = (a, b, cc)
        ins['x_bottom'] = float(a * yb * yb + b * yb + cc)
        hd = (a * yt * yt + b * yt + cc) - ins['x_bottom']  # heading: 먼쪽(상단)-가까운쪽(하단)
        if abs(hd) >= c.heading_frac * W:          # ① heading 우선
            ins['turn'] = 1 if hd > 0 else -1
        elif abs(a) >= c.curv_strong:              # ② 거의 수직이지만 크게 휘면 곡률
            ins['turn'] = 1 if a > 0 else -1
    return ins


# ============================ ③ 분리: sliding window ============================
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


def _win_overlap(wa, wb, iou_min):
    """같은 높이(창 index)에서 두 스택의 x구간 IOU ≥ iou_min 인 창 개수."""
    n = 0
    for i, (alo, ahi) in wa.items():
        b = wb.get(i)
        if b is None:
            continue
        blo, bhi = b
        inter = max(0, min(ahi, bhi) - max(alo, blo))
        union = max(ahi, bhi) - min(alo, blo)
        if union > 0 and inter / union >= iou_min:
            n += 1
    return n


def sliding_window_lanes(bev, color, c, windows_out=None):
    """BEV 마스크: ① 하단 base → ② 곡률로 창 좌우 이동하며 쌓기 → ③ 세로 미달 탈락 → ④ 중복 스택 병합."""
    H, W = bev.shape
    hist = (bev[H // 2:] > 0).sum(axis=0)
    win_h = max(1, H // c.sw_nwin)
    raw = []
    for base in _find_peaks(hist, c):          # ① base
        cur = float(base)
        step = 0.0                             # 창당 좌우 이동량(방향/곡률 추정)
        prev_cx = None
        xs_all, ys_all, miss, hits = [], [], 0, 0
        wins = {}                              # 창 index → (xlo, xhi)  (병합 판정용)
        for i in range(c.sw_nwin):             # ② 곡률 방향으로 창을 옮기며 위로 쌓기
            ci = int(round(cur))
            ylo, yhi = H - (i + 1) * win_h, H - i * win_h
            xlo, xhi = max(0, ci - c.sw_margin), min(W, ci + c.sw_margin)
            if windows_out is not None:
                windows_out.append((xlo, ylo, xhi, yhi))
            wy, wx = np.nonzero(bev[ylo:yhi, xlo:xhi] > 0)
            if wx.size > c.sw_minpix:
                mx = float(wx.mean()) + xlo
                xs_all.append(wx + xlo)
                ys_all.append(wy + ylo)
                wins[i] = (xlo, xhi)
                if prev_cx is not None:         # 관측된 좌우 이동으로 방향 갱신(EMA)
                    obs = max(-c.sw_margin, min(c.sw_margin, mx - prev_cx))
                    step = c.sw_dir_ema * obs + (1 - c.sw_dir_ema) * step
                prev_cx = mx
                cur = mx + step                 # 다음 창은 곡률 방향으로 좌우 예측
                miss = 0
                hits += 1
            else:
                cur += step                     # 빈 창(점선 간격)도 방향 따라 이동
                miss += 1
                if miss >= c.sw_max_miss:       # 연속 N회 실패 → 차선 종료
                    break
            cur = min(max(cur, 0.0), float(W - 1))
        # ③ 차선 성립 판정: 세로로 충분히 쌓였나? (정지선·노이즈·검출실패는 여기서 탈락)
        if not xs_all:
            continue
        xs, ys = np.concatenate(xs_all), np.concatenate(ys_all)
        y_span = float(ys.max() - ys.min())
        if hits < c.sw_min_hits or y_span < c.sw_min_span * H:
            continue                            # 정지선 등 → 라벨 붙이지 않고 버림
        if np.unique(ys).size >= 5 and xs.size >= c.sw_minpix:
            raw.append({'xs': xs, 'ys': ys, 'wins': wins, 'npix': int(xs.size), 'W': W})
    # ④ 중복 스택 병합: 창이 sw_merge_min개 이상 겹치면 같은 차선(같은 밴드를 두 base가 탄 경우).
    #    X자 교차는 겹치는 창이 1~2개뿐 → 임계 미만이라 분리 유지.
    raw.sort(key=lambda r: -r['npix'])         # 픽셀 많은(강한) 스택 우선 보존
    kept = []
    for r in raw:
        if any(_win_overlap(r['wins'], k['wins'], c.sw_merge_iou) >= c.sw_merge_min
               for k in kept):
            continue                            # 이미 보존된 스택과 중복 → 버림
        kept.append(r)
    return [_build_instance(k['ys'], k['xs'], color, c, k['W']) for k in kept]


# ============================ ④ 분류 ============================
def _side(ins, w):
    """좌/우: 위치(base x)와 방향(궤적 추세 x)을 동시에 투표. 불일치 시 base 우선."""
    cx = w / 2.0
    pos = -1 if ins['x_bottom'] < cx else 1     # 위치: 화면 중앙 기준 base
    dirv = -1 if ins['x_mean'] < cx else 1      # 방향: 궤적 전체가 향하는 쪽
    vote = pos + dirv
    if vote != 0:
        return 'L' if vote < 0 else 'R'
    return 'L' if ins['x_bottom'] < cx else 'R'  # 불일치(곡선) → 자차에 가까운 base


def classify(ins, w, corridor_turn=None):
    # corridor_turn(전역 ego 곡률)은 더 이상 쓰지 않음 — 각 차선의 per-instance 방향 사용.
    side = _side(ins, w)
    if ins['color'] == 'W':                      # 색분기: 흰색
        return f'W-{side}'
    tw = 'S' if ins['turn'] == 0 else ('R' if ins['turn'] > 0 else 'L')  # 노랑: heading+곡률
    return f'Y{tw}-{side}'


def _pair_gate(a, b, h, c):
    """두 차선을 나란한 좌/우 경계로 볼 수 있나?
    반환: (ylo, yhi) 겹침구간 or None(기각 — y겹침 부족/교차/비평행)."""
    ylo = max(int(a['ys'].min()), int(b['ys'].min()))
    yhi = min(int(a['ys'].max()), int(b['ys'].max()))
    if yhi - ylo < c.pair_overlap_min * h:        # y로 충분히 겹쳐야 좌/우 경계
        return None
    yy = np.linspace(ylo, yhi, 7)
    gaps = _ebottom(b['coeffs'], yy) - _ebottom(a['coeffs'], yy)  # b가 우측(정렬상)
    if float(gaps.min()) < c.pair_gap_min:        # 간격이 0 근처로 붕괴/역전 = 교차 → 기각
        return None
    return ylo, yhi


def lane_centers(lanes, w, h, c):
    """x_bottom 순 인접 차선 중 '나란한 쌍'만 골라 중앙선 산출(교차/비평행 쌍은 제외).
    각 항목: {'coeffs','x_bottom','offset'(+우/-좌),'ego','a','b','y_lo','y_hi'}.
    중앙선은 두 차선이 실제로 겹치는 y구간에서만 정의(외삽 방지)."""
    cx = w / 2.0
    ls = sorted([x for x in lanes if x['coeffs'] is not None],
                key=lambda x: x['x_bottom'])
    out = []
    for a, b in zip(ls, ls[1:]):                  # 인접 쌍
        if _side(a, w) == _side(b, w):            # side(끝 알파벳) 같으면 좌/우 경계 아님 → 기각
            continue                              # 예: YL-L | YR-L (둘 다 L) → 매칭 안 함
        ov = _pair_gate(a, b, h, c)
        if ov is None:                            # 쌍 안 맞음 → 중앙선 안 만듦
            continue
        ylo, yhi = ov
        coeffs = tuple((p + q) / 2.0 for p, q in zip(a['coeffs'], b['coeffs']))
        x_bottom = float(_ebottom(coeffs, yhi))    # 가장 가까운(아래) 겹침 지점
        ego = a['x_bottom'] < cx <= b['x_bottom']  # 이 쌍이 화면 중앙을 사이에 둠 → 자차 통로
        out.append({'coeffs': coeffs, 'x_bottom': x_bottom, 'offset': x_bottom - cx,
                    'ego': ego, 'a': a, 'b': b, 'y_lo': ylo, 'y_hi': yhi})
    return out


def ego_center(centers, lanes, w, width):
    """자차 통로 중앙선.
    1) 정상: 화면 중앙을 사이에 둔 인접 쌍.
    2) fallback: 쌍이 안 맞으면(한쪽만 있음) 중앙 근처 차선을 차선폭 절반만큼
       중앙 쪽으로 평행이동해 중앙선을 추정(coast). 너무 먼 차선은 제외."""
    for cc in centers:
        if cc['ego']:
            return cc
    cx = w / 2.0
    cand = [x for x in lanes if x['coeffs'] is not None]
    if not cand or width <= 0:
        return None
    near = min(cand, key=lambda x: abs(x['x_bottom'] - cx))
    if abs(near['x_bottom'] - cx) > width:        # ego 경계로 보기 어려울 만큼 멀면 포기
        return None
    dx = width / 2.0 if near['x_bottom'] < cx else -width / 2.0
    coeffs = _shift(near['coeffs'], dx)
    xb = near['x_bottom'] + dx
    return {'coeffs': coeffs, 'x_bottom': xb, 'offset': xb - cx, 'ego': True,
            'a': near, 'b': None, 'coast': True}


EGO_CENTER_COLOR = (255, 255, 0)   # BGR 시안 — 자차 통로 중앙선(제어값)


# ============================ 기억(추적) ============================
def _shift(coeffs, dx):
    a, b, c = coeffs
    return (a, b, c + dx)


def _ebottom(coeffs, yb):
    a, b, c = coeffs
    return a * yb * yb + b * yb + c


class Tracker:
    def __init__(self, c, h, w):
        self.c = c
        self.h, self.w = h, w
        self.L = self.R = self.width = None
        self.lost = 0
        self.turn = 1

    def _pick(self, cands, tracked, want_side):
        cands = [x for x in cands if x['coeffs'] is not None]
        if not cands:
            return None
        yb = self.h - 1
        if tracked is not None:
            best = min(cands, key=lambda x: abs(_ebottom(x['coeffs'], yb) - _ebottom(tracked, yb)))
            return best if abs(_ebottom(best['coeffs'], yb) - _ebottom(tracked, yb)) <= self.c.jump_max else None
        cx = self.w / 2
        pool = [x for x in cands if (x['x_bottom'] < cx) == (want_side == 'L')]
        if not pool:
            return None
        return (max if want_side == 'L' else min)(pool, key=lambda x: x['x_bottom'])

    def _ema(self, prev, meas):
        if prev is None:
            return meas
        a = self.c.ema
        return tuple(a * m + (1 - a) * p for m, p in zip(meas, prev))

    def update(self, instances):
        drive = list(instances)   # 정지선 등은 이미 분리 단계에서 탈락 → 전부 주행 차선
        Lc = [x for x in drive if x['x_bottom'] < self.w / 2]
        Rc = [x for x in drive if x['x_bottom'] >= self.w / 2]
        mL, mR = self._pick(Lc, self.L, 'L'), self._pick(Rc, self.R, 'R')
        yb = self.h - 1
        gL, gR = mL is not None, mR is not None
        width = self.width if self.width is not None else self.c.lane_width_default * self.w
        if gL and gR:
            self.L = self._ema(self.L, mL['coeffs'])
            self.R = self._ema(self.R, mR['coeffs'])
            wdt = _ebottom(mR['coeffs'], yb) - _ebottom(mL['coeffs'], yb)
            self.width = wdt if self.width is None else 0.6 * self.width + 0.4 * wdt
            self.lost = 0
        elif gL:
            self.L = self._ema(self.L, mL['coeffs'])
            self.R = _shift(self.L, width)
            self.lost = 0
        elif gR:
            self.R = self._ema(self.R, mR['coeffs'])
            self.L = _shift(self.R, -width)
            self.lost = 0
        else:
            self.lost += 1
        if self.lost >= self.c.lost_reset:
            self.L = self.R = self.width = None
        if self.L is not None and self.R is not None:
            ca = (self.L[0] + self.R[0]) / 2.0
            self.turn = 0 if abs(ca) < self.c.straight_thresh else (1 if ca > 0 else -1)
        return mL, mR


# ============================ 렌더 ============================
STAGE_SC = 2


def _inst_color(idx):
    bgr = cv2.cvtColor(np.uint8([[[int((idx * 47) % 180), 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def _panel(img, title):
    cv2.rectangle(img, (0, 0), (img.shape[1] - 1, 12), (0, 0, 0), -1)
    cv2.putText(img, title, (2, 9), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
    return img


def _draw_bev_curve(img, coeffs, color, thick, y0=None, y1=None):
    h, w = img.shape[:2]
    ys = np.arange(0 if y0 is None else int(y0), h if y1 is None else int(y1) + 1)
    xs = coeffs[0] * ys * ys + coeffs[1] * ys + coeffs[2]
    pts = np.array([[int(x), int(y)] for x, y in zip(xs, ys) if 0 <= x < w], np.int32)
    if len(pts) > 1:
        cv2.polylines(img, [pts], False, color, thick)


def _warp_back(coeffs, Minv, h, w, y0=None, y1=None):
    ys = np.arange(0 if y0 is None else int(y0), h if y1 is None else int(y1) + 1, 2).astype(np.float32)
    xs = coeffs[0] * ys * ys + coeffs[1] * ys + coeffs[2]
    pts = np.stack([xs, ys], 1).reshape(-1, 1, 2).astype(np.float32)
    return cv2.perspectiveTransform(pts, Minv).reshape(-1, 2)


def _draw_pts(img, pts, color, thick):
    h, w = img.shape[:2]
    p = np.array([[int(x), int(y)] for x, y in pts if 0 <= x < w and 0 <= y < h], np.int32)
    if len(p) > 1:
        cv2.polylines(img, [p], False, color, thick)


def _centers_panel(h, w, centers, ec, bev_w, bev_y, cturn):
    """인접 차선 쌍의 중앙선만 모아 BEV로 표시. ego 통로 중앙선은 시안·굵게.
    ec가 coast(쌍 결손 → 차선폭 fallback)면 별도 시안 파선으로 추가 표시."""
    img = np.zeros((h, w, 3), np.uint8)
    img[bev_w > 0] = (40, 40, 40)                 # 흐린 차선 배경(맥락용)
    img[bev_y > 0] = (40, 45, 50)
    cv2.line(img, (w // 2, 14), (w // 2, h - 1), (60, 60, 60), 1)  # 화면 중앙 기준선
    ty = 24
    for idx, cc in enumerate(centers):
        col = EGO_CENTER_COLOR if cc['ego'] else _inst_color(idx + 3)
        _draw_bev_curve(img, cc['coeffs'], col, 3 if cc['ego'] else 2, cc['y_lo'], cc['y_hi'])
        tag = (f"{'*' if cc['ego'] else ' '}{classify(cc['a'], w, cturn)}|"
               f"{classify(cc['b'], w, cturn)} off={cc['offset']:+.0f}")
        cv2.putText(img, tag, (4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.30, col, 1)
        ty += 15
    if ec is not None and ec.get('coast'):        # 차선폭 fallback ego 중앙선
        _draw_bev_curve(img, ec['coeffs'], EGO_CENTER_COLOR, 3)
        cv2.putText(img, f"*coast {classify(ec['a'], w, cturn)}+w/2 off={ec['offset']:+.0f}",
                    (4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.30, EGO_CENTER_COLOR, 1)
    _panel(img, f"6 centers ({len(centers)})  *=ego")
    return img


def render_stages(frame, wm, ym, bev_w, bev_y, lanes, trk, Minv, windows):
    h, w = frame.shape[:2]
    cturn = trk.turn if (trk.L is not None and trk.R is not None) else None
    p1 = frame.copy()
    cv2.polylines(p1, [_src_trapezoid(h, w, trk.c).astype(np.int32)], True, (0, 200, 255), 1)
    _panel(p1, "1 input + ROI")
    p2 = np.zeros((h, w, 3), np.uint8)
    p2[bev_w > 0] = (200, 200, 200)
    p2[bev_y > 0] = (0, 220, 255)
    _panel(p2, "2 BEV mask")
    p3 = np.zeros((h, w, 3), np.uint8)
    for (xlo, ylo, xhi, yhi) in windows:
        cv2.rectangle(p3, (xlo, ylo), (xhi, yhi), (55, 55, 55), 1)
    for idx, ins in enumerate(lanes):
        col = _inst_color(idx)
        for x, y in zip(ins['xs'][::3], ins['ys'][::3]):
            cv2.circle(p3, (int(x), int(y)), 1, col, -1)
    _panel(p3, f"3 BEV sliding ({len(lanes)} lane)")
    p4 = np.zeros((h, w, 3), np.uint8)
    p4[bev_w > 0] = (45, 45, 45)
    p4[bev_y > 0] = (45, 50, 55)
    p5 = frame.copy()
    for ins in lanes:
        col = LABEL_COLORS.get(classify(ins, w, cturn), (0, 255, 0))
        if ins['coeffs'] is not None:
            _draw_bev_curve(p4, ins['coeffs'], col, 2)
            _draw_pts(p5, _warp_back(ins['coeffs'], Minv, h, w), col, 2)  # 원본에 라벨 오버레이
        else:
            for x, y in zip(ins['xs'][::3], ins['ys'][::3]):
                cv2.circle(p4, (int(x), int(y)), 1, col, -1)
    centers = lane_centers(lanes, w, h, trk.c)
    width = trk.width if trk.width else trk.c.lane_width_default * w
    ec = ego_center(centers, lanes, w, width)
    if ec is not None:                           # 자차 통로 중앙선(제어값): BEV + 원본에 시안으로
        y0, y1 = ec.get('y_lo'), ec.get('y_hi')  # 정상 쌍은 겹침구간만, coast는 전체
        _draw_bev_curve(p4, ec['coeffs'], EGO_CENTER_COLOR, 2, y0, y1)
        _draw_pts(p5, _warp_back(ec['coeffs'], Minv, h, w, y0, y1), EGO_CENTER_COLOR, 2)
    _panel(p4, "4 BEV classify")
    off = (f"off={ec['offset']:+.0f}px{'(coast)' if ec.get('coast') else ''}"
           if ec is not None else "off=--")
    _panel(p5, f"5 label overlay (orig)  {len(lanes)}L  {off}")
    p6 = _centers_panel(h, w, centers, ec, bev_w, bev_y, cturn)
    grid = np.vstack([np.hstack([p1, p2, p3]), np.hstack([p4, p5, p6])])
    return cv2.resize(grid, (w * 3 * STAGE_SC, h * 2 * STAGE_SC), interpolation=cv2.INTER_NEAREST)


def render_single(frame, lanes, cturn, Minv, ec):
    img = frame.copy()
    h, w = frame.shape[:2]
    for ins in lanes:
        if ins['coeffs'] is None:
            continue
        col = LABEL_COLORS.get(classify(ins, w, cturn), (0, 255, 0))
        _draw_pts(img, _warp_back(ins['coeffs'], Minv, h, w), col, 2)
    if ec is not None:
        _draw_pts(img, _warp_back(ec['coeffs'], Minv, h, w, ec.get('y_lo'), ec.get('y_hi')),
                  EGO_CENTER_COLOR, 2)
    return img


# ============================ 메인 ============================
def run(path, name, dump, c, stages=False):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    M, Minv = bev_transform(h, w, c)
    outw, outh = (w * 3 * STAGE_SC, h * 2 * STAGE_SC) if stages else (w, h)
    runid = name or os.path.splitext(os.path.basename(path))[0]
    outdir = os.path.join("rslt", runid)
    os.makedirs(outdir, exist_ok=True)
    video = os.path.join(outdir, "stages.mp4" if stages else "overlay.mp4")
    writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*'mp4v'), fps, (outw, outh))
    trk = Tracker(c, h, w)
    counts = {}
    n_frames = n_ego = 0
    ego_turn = {'L': 0, 'S': 0, 'R': 0}
    n_ecenter = n_ecoast = 0
    off_sum = 0.0
    fi = 0
    dump = set(dump or [])
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        wm, ym = detect(frame, c)
        bev_w = cv2.warpPerspective(wm, M, (w, h), flags=cv2.INTER_NEAREST)
        bev_y = cv2.warpPerspective(ym, M, (w, h), flags=cv2.INTER_NEAREST)
        windows = []
        lanes = (sliding_window_lanes(bev_w, 'W', c, windows) +
                 sliding_window_lanes(bev_y, 'Y', c, windows))
        mL, mR = trk.update(lanes)
        has_ego = trk.L is not None and trk.R is not None
        cturn = trk.turn if has_ego else None
        for ins in lanes:
            lab = classify(ins, w, cturn)
            counts[lab] = counts.get(lab, 0) + 1
        if has_ego:
            n_ego += 1
            ego_turn['S' if trk.turn == 0 else ('R' if trk.turn > 0 else 'L')] += 1
        width = trk.width if trk.width else c.lane_width_default * w
        ec = ego_center(lane_centers(lanes, w, h, c), lanes, w, width)
        if ec is not None:
            n_ecenter += 1
            off_sum += abs(ec['offset'])
            if ec.get('coast'):
                n_ecoast += 1
        n_frames += 1
        vis = (render_stages(frame, wm, ym, bev_w, bev_y, lanes, trk, Minv, windows)
               if stages else render_single(frame, lanes, cturn, Minv, ec))
        writer.write(vis)
        if fi in dump:
            p = os.path.join(outdir, f"f{fi}.png")
            sc = 1 if stages else 3
            cv2.imwrite(p, cv2.resize(vis, (vis.shape[1] * sc, vis.shape[0] * sc),
                                      interpolation=cv2.INTER_NEAREST))
            print(f"  dump {p}")
        fi += 1
    cap.release()
    writer.release()
    print(f"\n[{os.path.basename(path)}] {n_frames}프레임")
    print(f"  ego 양측 확보 : {n_ego}/{n_frames} ({100 * n_ego / max(1, n_frames):.0f}%)")
    print(f"  ego turn      : L(좌)={ego_turn['L']} S(직진)={ego_turn['S']} R(우)={ego_turn['R']}")
    print(f"  라벨 분포     : " + " ".join(f"{k}={v}" for k, v in
                                         sorted(counts.items(), key=lambda x: -x[1])))
    print(f"  ego 중앙선 확보: {n_ecenter}/{n_frames} ({100 * n_ecenter / max(1, n_frames):.0f}%)"
          f"  (coast {n_ecoast})  평균|offset|={off_sum / max(1, n_ecenter):.0f}px")
    print(f"  출력 폴더     : {outdir}/  (video: {os.path.basename(video)})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--name", default="")
    ap.add_argument("--dump", default="")
    ap.add_argument("--stages", action="store_true")
    ap.add_argument("--sw-margin", type=int)
    ap.add_argument("--sw-nwin", type=int)
    ap.add_argument("--roi-top", type=float)
    a = ap.parse_args()
    cfg = Cfg()
    if a.sw_margin is not None:
        cfg.sw_margin = a.sw_margin
    if a.sw_nwin is not None:
        cfg.sw_nwin = a.sw_nwin
    if a.roi_top is not None:
        cfg.roi_top = a.roi_top
    dump = [int(x) for x in a.dump.split(",") if x.strip()] if a.dump else []
    run(a.input, a.name, dump, cfg, stages=a.stages)
