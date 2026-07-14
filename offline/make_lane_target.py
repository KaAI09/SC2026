"""직선 차선 검증 타깃 생성 — A4 타일 PDF (트랙 없이 `calibrate.py --check` 를 돌리려고).

`--check` 는 **차선 두 개가 보이는 직선 구간**을 요구한다 (`CameraModel.validate`: 두 경계가
(1) 수직이고 (2) 평행하고 (3) 정확히 한 차선폭 떨어져 있는가). 트랙이 없으면 캘리브를 검증할
수 없고, 검증 안 된 캘리브는 **조용히** 틀린다 — 스티어링 버그처럼 보이는 종류로.

그래서 트랙 한 조각을 인쇄한다. 검정 바탕에 흰 차선 2줄, 실제와 **같은 35cm 폭**으로.

  왜 A4 한 장이 안 되나: 차선폭 35cm + 테이프 3cm = **38cm** 가 필요한데 A4 는 21cm 다.
  그리고 BEV 는 전방 26~78cm(깊이 52cm)를 보는데 A4 세로는 29.7cm 다. 2x2 로 타일링한다.

  프린터 여백: 무여백 인쇄가 되는 프린터는 드물다. 그래서 타일을 A4(210x297)가 아니라
  **200x287mm** 로 잡고 페이지 중앙에 앉힌다(여백 5mm). 2x2 = 400x574mm = 40 x 57.4cm.
  38cm 차선이 들어가고 52cm 깊이도 덮는다.

  ⚠ **인쇄 배율이 곧 검증 정확도다.** 프린터가 "용지에 맞춤" 으로 축소하면 차선폭이 35cm 가
  아니게 되고, 그러면 `--check` 는 캘리브가 아니라 프린터를 측정하게 된다. 반드시
  **배율 100% / 실제 크기 / 페이지 크기 조정 안 함** 으로 인쇄하고, 마지막 페이지의
  스케일바를 **자로 재서 100mm 인지 확인**하라.

  회색(V=100)으로 그린 것들(재단선·라벨·스케일바)은 흰색 검출(`white_v_min=185`)에 걸리지
  않는다 — 검출되는 것은 오직 차선뿐이다.

    .venv/bin/python offline/make_lane_target.py --out /tmp/lane_target.pdf
    .venv/bin/python offline/make_lane_target.py --lane-width-cm 35 --tape-width-cm 3

조립 → 촬영 → 검증 절차는 생성된 PDF 의 마지막 페이지에 인쇄된다.
"""
import argparse

MM = 72.0 / 25.4          # mm -> PDF pt (PDF 단위는 1/72 inch)
A4 = (210.0, 297.0)       # mm


def _esc(s):
    return s.replace('\\', r'\\').replace('(', r'\(').replace(')', r'\)')


class Page:
    """한 페이지의 content stream. 좌표는 mm, 원점은 좌하단."""

    def __init__(self):
        self.ops = []

    def rect(self, x, y, w, h, rgb):
        r, g, b = rgb
        self.ops.append(f'{r:.3f} {g:.3f} {b:.3f} rg '
                        f'{x * MM:.3f} {y * MM:.3f} {w * MM:.3f} {h * MM:.3f} re f')

    def text(self, x, y, s, size=9, rgb=(0.4, 0.4, 0.4)):
        r, g, b = rgb
        self.ops.append(f'BT {r:.3f} {g:.3f} {b:.3f} rg /F1 {size} Tf '
                        f'{x * MM:.3f} {y * MM:.3f} Td ({_esc(s)}) Tj ET')

    def stream(self):
        return '\n'.join(self.ops).encode('latin-1')


def write_pdf(pages, path):
    """의존성 없이 PDF 를 쓴다. 벡터라 인쇄 배율만 지키면 치수가 정확하다."""
    objs = []                                    # 1-indexed body objects
    n_pages = len(pages)
    font_id = 3 + 2 * n_pages                    # after catalog(1), pages(2), page/content pairs

    kids = ' '.join(f'{3 + 2 * i} 0 R' for i in range(n_pages))
    objs.append(b'<< /Type /Catalog /Pages 2 0 R >>')
    objs.append(f'<< /Type /Pages /Kids [{kids}] /Count {n_pages} >>'.encode())
    for i, pg in enumerate(pages):
        content_id = 4 + 2 * i
        objs.append((
            f'<< /Type /Page /Parent 2 0 R '
            f'/MediaBox [0 0 {A4[0] * MM:.3f} {A4[1] * MM:.3f}] '
            f'/Contents {content_id} 0 R '
            f'/Resources << /Font << /F1 {font_id} 0 R >> >> >>').encode())
        data = pg.stream()
        objs.append(b'<< /Length ' + str(len(data)).encode() + b' >>\nstream\n'
                    + data + b'\nendstream')
    objs.append(b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>')

    out = bytearray(b'%PDF-1.4\n')
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += f'{i} 0 obj\n'.encode() + body + b'\nendobj\n'
    xref = len(out)
    out += f'xref\n0 {len(objs) + 1}\n'.encode()
    out += b'0000000000 65535 f \n'
    for off in offsets:
        out += f'{off:010d} 00000 n \n'.encode()
    out += (f'trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n').encode()
    with open(path, 'wb') as f:
        f.write(bytes(out))


GRAY = (0.39, 0.39, 0.39)      # V ~= 100 < white_v_min(185) -> 흰색 검출에 안 걸린다
BLACK = (0.0, 0.0, 0.0)
WHITE = (1.0, 1.0, 1.0)


def build(a):
    tw, th = a.tile_mm                       # 타일(= 인쇄 영역) mm
    cols, rows = a.tiles
    W, H = tw * cols, th * rows              # 전체 캔버스 mm
    ox, oy = (A4[0] - tw) / 2, (A4[1] - th) / 2   # 페이지 안에서 타일의 여백

    lane_mm = a.lane_width_cm * 10.0
    tape_mm = a.tape_width_cm * 10.0
    # 차선 중심선은 캔버스 중앙 기준 +-lane/2. 흰 띠는 그 중심에서 +-tape/2.
    centres = [W / 2 - lane_mm / 2, W / 2 + lane_mm / 2]
    bands = [(c - tape_mm / 2, c + tape_mm / 2) for c in centres]

    if bands[0][0] < 0 or bands[1][1] > W:
        raise SystemExit(
            f'차선({a.lane_width_cm}cm + 테이프 {a.tape_width_cm}cm = '
            f'{(lane_mm + tape_mm) / 10:.1f}cm)이 캔버스 폭 {W / 10:.1f}cm 를 넘는다. '
            f'--tiles 를 늘려라.')

    pages = []
    for r in range(rows - 1, -1, -1):            # 위 행부터 인쇄 (1번이 좌상단)
        for c in range(cols):
            pg = Page()
            pg.rect(ox, oy, tw, th, BLACK)       # 타일 전체 = 검정 노면

            # 이 타일이 담당하는 캔버스 구간 [x0, x0+tw) x [y0, y0+th)
            x0, y0 = c * tw, r * th
            for b0, b1 in bands:                 # 흰 차선 (타일 경계에서 클리핑)
                s, e = max(b0, x0), min(b1, x0 + tw)
                if e > s:
                    pg.rect(ox + (s - x0), oy, e - s, th, WHITE)

            # 재단선: 타일 경계 모서리. 여기까지 자르고 맞대어 붙인다.
            L = 6.0
            for cx, cy in ((ox, oy), (ox + tw, oy), (ox, oy + th), (ox + tw, oy + th)):
                pg.rect(cx - L / 2, cy - 0.15, L, 0.3, GRAY)
                pg.rect(cx - 0.15, cy - L / 2, 0.3, L, GRAY)

            idx = (rows - 1 - r) * cols + c + 1
            pos = f'{"상" if r == rows - 1 else "하"}{"좌" if c == 0 else "우"}'
            pg.text(ox + 3, oy + th - 6, f'{idx}  (row {rows - r}, col {c + 1})',
                    size=8, rgb=GRAY)
            pg.text(ox + 3, oy - 6,
                    f'SC2026 lane target  {a.lane_width_cm}cm lane / '
                    f'{a.tape_width_cm}cm tape  -  print at 100% (no fit-to-page)',
                    size=7, rgb=GRAY)
            pages.append((pg, idx, pos))

    pages.sort(key=lambda t: t[1])
    out = [p for p, _, _ in pages]

    # ---- 마지막 페이지: 스케일바 + 조립/촬영/검증 안내 (흰 바탕) ------------
    g = Page()
    g.rect(0, 0, *A4, WHITE)
    y = 275
    g.text(20, y, 'SC2026 - lane target: assemble / shoot / check', 13, BLACK); y -= 9
    g.text(20, y, f'canvas {W / 10:.1f} x {H / 10:.1f} cm  from {cols}x{rows} A4 tiles '
                  f'({tw:.0f}x{th:.0f} mm each)', 9, GRAY); y -= 14

    g.text(20, y, '0. VERIFY THE PRINT SCALE FIRST', 11, BLACK); y -= 8
    g.text(20, y, 'Measure the bar below with a ruler. It must be exactly 100 mm.', 9, BLACK)
    y -= 12
    g.rect(20, y - 4, 100, 1.2, BLACK)                    # 100mm 스케일바
    for i in range(11):                                    # 10mm 눈금
        g.rect(20 + i * 10 - 0.3, y - 8, 0.6, 4, BLACK)
    g.text(20, y - 14, '0', 8, BLACK)
    g.text(117, y - 14, '100 mm', 8, BLACK)
    y -= 24
    g.text(20, y, 'If it is not 100 mm, the printer rescaled: reprint with scale 100% /', 9, BLACK)
    y -= 7
    g.text(20, y, 'actual size / no page scaling. A wrong scale makes --check measure the', 9, BLACK)
    y -= 7
    g.text(20, y, 'PRINTER, not the calibration.', 9, BLACK); y -= 14

    for title, lines in (
        ('1. ASSEMBLE', [
            'Cut each tile along the corner crop marks, then butt the edges together',
            '(do not overlap). Tape from the BACK. A white seam on the front would be',
            'detected as lane pixels; a black seam just looks like a dashed lane, which',
            'the sliding window bridges on purpose.',
            'Lay it dead flat - a curled sheet breaks the ground-plane assumption that H',
            'is built on. Tape it down.']),
        ('2. SHOOT', [
            'Park the car on the target, wheels straight, so the two lanes run straight',
            'away from the camera and BOTH are visible. The car must be in its normal',
            'driving state (battery in, tyres inflated) - camera height and pitch ARE H.',
            'ros2 launch dracer_bringup collect.launch.py   # START -> 2-3 s -> STOP']),
        ('3. CHECK (local)', [
            'scp the mp4, grab one frame, then:',
            '  .venv/bin/python offline/calibrate.py --check \\',
            '      D-Racer-Kit/src/config/camera.yaml \\',
            f'      --straight offline/calib/straight.png --lane-width-cm {a.lane_width_cm:g}',
            'Pass = "OK - calibration valid". Tolerance is 20% of the lane width (7cm at',
            '35cm), but that is the PASS LINE, not the target: a good calibration lands',
            'within 1-2 cm of width error and under 1 cm of parallel spread.']),
    ):
        g.text(20, y, title, 11, BLACK); y -= 8
        for ln in lines:
            g.text(20, y, ln, 9, GRAY); y -= 6.5
        y -= 6

    g.text(20, y, 'NOTE  the target only validates the BEV geometry (metric / parallel /', 8, GRAY)
    y -= 6
    g.text(20, y, 'vertical). Lane COLOUR thresholds still belong to the real track profile.', 8, GRAY)
    out.append(g)
    return out, (W, H)


def _tiles(s):
    c, r = s.lower().split('x')
    return int(c), int(r)


def _mm(s):
    w, h = s.lower().split('x')
    return float(w), float(h)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='lane_target.pdf')
    ap.add_argument('--lane-width-cm', type=float, default=35.0,
                    help='차선 중심선 간 거리 (트랙 실측값과 같게)')
    ap.add_argument('--tape-width-cm', type=float, default=3.0, help='테이프 폭')
    ap.add_argument('--tiles', type=_tiles, default=(2, 2), help='A4 타일 배치 ColsxRows')
    ap.add_argument('--tile-mm', type=_mm, default=(200.0, 287.0),
                    help='타일 1장의 인쇄 영역 mm (A4 210x297 에서 여백 5mm)')
    a = ap.parse_args()

    pages, (W, H) = build(a)
    write_pdf(pages, a.out)
    cols, rows = a.tiles
    print(f'{a.out}  —  A4 {len(pages)}쪽 (타일 {cols}x{rows} + 안내 1쪽)')
    print(f'  캔버스   : {W / 10:.1f} x {H / 10:.1f} cm')
    print(f'  차선     : 중심선 간 {a.lane_width_cm}cm, 테이프 폭 {a.tape_width_cm}cm')
    print(f'  BEV 커버 : 전방 26~78cm(깊이 52cm) 필요 → 캔버스 세로 {H / 10:.1f}cm '
          f'{"OK" if H / 10 >= 52 else "✗ 부족 — --tiles 세로를 늘려라"}')
    print('  ⚠ 배율 100% (용지 맞춤 끄기) 로 인쇄하고, 마지막 쪽 스케일바를 자로 재라.')


if __name__ == '__main__':
    main()
