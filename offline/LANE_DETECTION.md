# 차선검출 오프라인 실험 설계 문서 (perception_preview.py / perception_select.py)

> **위치/성격**: `offline/`. 로컬 실험 도구이며 차량/배포 코드가 아님. 코드·문서는 저장소에 공유(추적)하되, 실행 결과물(`*.mp4`/`*.png`/`*.db3`)은 git-ignore.
> **목적**: 녹화 클립으로 여러 검출 로직을 **모드 선택 + 파라미터 튜닝**만으로 비교하고, 대회 당일 실제 트랙에서 최적 조합을 고른다.
> **비목표**: 이 문서/코드는 조향·throttle 등 차량 제어를 직접 수행하지 않는다(검출·상태추정까지만, 제어는 분리).
> **파일 역할**: `perception_preview.py`(검출 적용 + 3패널 시각화) → `perception_select.py`(조합 비교 + 지각지표 랭킹 + 선택 export). 검출 로직 자체는 공유 코어 `driving_core.lane_core`가 실행(온라인 노드와 동일).

관련 문서: 전체 오프라인 파이프라인·핸드오프 구조는 [PIPELINE.md](PIPELINE.md), 제어는 [CONTROL_DESIGN.md](CONTROL_DESIGN.md). 대회 규정 요약은 `Notice/미션 및 규정 설명 OT.pdf`, 카메라/토픽은 [D-Racer-Kit/docs/](../docs/) 참고.

---

## 1. 설계 원칙

1. **config-driven 합성 파이프라인** — 각 축(A~F)을 조합 가능한 단계로 두고, "모드"는 `Cfg` 프리셋일 뿐이다. 로직 중복·경우의 수 폭발을 막는다.
2. **한 축에서 다기능 조합 허용** — segmentation을 `HSV OR LAB`, `(HSV OR LAB) AND edge`처럼 결합한다.
3. **BEV/IPM은 optional·저우선** — 4점 호모그래피 캘리브레이션 부담 + 카메라 각도 미확정. 이미지 공간에서 안정화 후 필요 시에만.
4. **ROI 형태/점유도를 적극 실험** — BEV 대신 하단 크롭 + 사다리꼴 ROI를 다양하게 스윕.
5. **검출 ↔ 제어 분리** — 이 도구의 출력은 `center_error / heading_error / confidence / state`까지. 제어 매핑은 별도.

### 파이프라인 흐름
```
프레임(BGR)
  └─ A. segmentation ─ B. ROI ─ C. extraction ─ D. state ─┬─ E. temporal ─ F. failsafe(주석)
                                                          └─ 시각화(side-by-side 패널)
```

---

## 2. 조합(fusion) 방식 분류

| 방식 | 의미 | 용도 | 코드상 |
|---|---|---|---|
| OR fusion | 하나라도 검출되면 후보 | recall 확보 | `fuse='or'` (HSV\|LAB) |
| AND fusion | 둘 다 만족 | FP 제거 | `fuse='and'` |
| cascade | 앞 결과를 뒤가 정제 | FP 제거(과소검출 방지) | `edge_validate=True` (edge dilate 후 AND) |
| fallback | 1순위 실패 시 2순위 | 실패 대응 | ✅ `seg_fallback_adaptive` (M6) HSV→Adaptive |
| weighted | 가중 합 | 미사용 | — |

> **주의(구현 보정)**: 채운 흰마스크와 얇은 Canny를 raw 픽셀 AND하면 과도하게 얇아진다. 그래서 **에지를 dilate 후 AND**하는 cascade로 구현(`edge_dilate`).

---

## 3. 축별 설계

각 파라미터는 `Cfg` 데이터클래스 필드명 / 기본값 기준.

### 축 A — 차선 픽셀 분리 (segmentation)

**색상 필드 재설계**: 두 트랙은 **흰색 + 노랑**만 사용(orange 개념 폐기). `use_hsv/use_lab/use_orange` → **`colors` 집합**으로 통합.
```python
colors = ('white',) | ('yellow',) | ('white','yellow')   # G5 = 흰+노랑 융합
white_use_lab: bool          # 밝기 보조(LAB L-channel OR)
seg_fallback_adaptive: bool  # G6: 실패 시 adaptive threshold fallback
```

| 후보 | 구성 | 목적 | 상태 |
|---|---|---|---|
| A1 | HSV white | 기본 흰색 분리 | ✅ `colors=('white',)` |
| A2 | + LAB L-channel OR | 밝기 보조 | ✅ `white_use_lab` |
| A3 | Yellow hue | 노랑 로터리/지름길 | ✅ `colors=('yellow',)` |
| A4 | White OR Yellow 융합 | 혼재/분기 | ✅ `colors=('white','yellow')` (G5) |
| A5 | (color) AND edge cascade | FP↓ | ✅ `edge_validate` |
| A6 | Adaptive fallback | 조명 편차·실패 대응 | ✅ `seg_fallback_adaptive` (G6) |
| A7 | + morphology | 점선 틈 메움 | ✅ `morph_kernel` (G4=5) |

**실측 파라미터(보존)**: `white_s_max=60`, `white_v_min=185`; `yellow_h_lo=18, yellow_h_hi=36, yellow_s_min=65, yellow_v_min=100`; `lab_l_min=170`, `canny_lo=50`, `canny_hi=150`, `edge_dilate=2`, `morph_kernel=3`(G4는 5). (§4 실측 근거)

### 축 B — ROI / 기하
| 후보 | 구성 | 상태 |
|---|---|---|
| B1 | 고정 하단 ROI | ✅ `roi_top_frac` |
| B2 | 사다리꼴 ROI | ✅ `trap_top_w`/`trap_bot_w` |
| B3 | horizon cut | ✅ (roi_top_frac로 상단 컷) |
| B4 | 이전 프레임 기반 동적 ROI | ✅ `dynamic_roi` (`--dynamic-roi`) |
| B5 | 하단 + 사다리꼴 (기본값) | ✅ 기본 |
| B7 | BEV/IPM | ⬜ optional, 저우선 |

**실측 파라미터(보존, 2025 대시캠 히트맵)**: `roi_top_frac`=흰/안전망 **0.35**·노랑/혼재 **0.30**, `trap_top_w=0.80`, `trap_bot_w=1.0`. 기존 0.55/0.55는 곡선·로터리에서 차선을 잘라먹어 폐기. **차량 카메라에서 재보정 필요**(FOV 상이). CLI `--roi-top`으로 스윕.

### 축 C — 차선 포인트/모델 추출
| 후보 | 구성 | 상태 |
|---|---|---|
| C1 | row centroid | ✅ 행별 좌/우 흰선 → 중심 |
| C2 | scanline 좌/우 분리 | ✅ (중심 기준 좌/우 split) |
| C3 | contour filtering | ✅ `min_contour_area` |
| C6 | polynomial fit(2차) | ✅ `do_polyfit` (`--polyfit`) |
| C5 | HoughLinesP heading | ✅ `heading_method='hough'` (M4) |
| C4 | connected components | ⬜ (contour로 대체) |

| C7 | split 기준 선택 | ✅ `split_ref`: center / prev_frame / prev_row (`--split`) |
| C8 | 종횡비/길이 필터 | ✅ `min_aspect`/`min_length` (`--aspect`/`--length`, 기본 off) |
| C9 | lane_width 이상치 제거 | ✅ `lane_width_tol` (`--lane-width-tol`, off) |

**로직**: 각 행에서 **split 기준(cx / 이전프레임 EMA / 이전행 전파)** 기준 좌측 최댓값·우측 최솟값을 차선 경계로, 중심=중점. 한쪽만 보이면 `lane_width_default=0.6·W`(또는 직전 추정폭 `lane_width_est`)로 보정(D4 fallback 내장).

### 축 D — LaneState 추정
| 출력 | 정의 | 상태 |
|---|---|---|
| center_error | 하단부 중심 x의 평균을 [-1,1] 정규화 | ✅ |
| heading_error | 진행방향 각도 | ✅ `heading_method`: slope/two_point/norm_slope/hough (`--heading`). slope는 과장 → two_point/hough 권장 |
| confidence | 유효 행 수 / 전체 스캔 행 수 | ✅ |
| curvature | 2차 피팅 곡률 | ✅ `curvature` (M5) |
| lane_width_est | 직전 유효 차선폭 | ✅ (fallback용) |
| left/right conf | 좌/우 개별 검출률 | ✅ `per_lane_conf` (`--per-lane-conf`) |
- 추천 기본 출력 = **D7: center + heading + confidence**.

### 축 E — 시간적 안정화
| 후보 | 상태 | 파라미터 |
|---|---|---|
| E1 EMA | ✅ | `ema_alpha=0.4` |
| E4 outlier rejection | ✅ | `outlier_jump=0.5` (|Δcenter_error| 초과 시 기각) |
| E3 previous valid hold | ✅ | 손실 시 직전 EMA 유지 |
| E2 median | ✅ `use_median`/`median_window` (`--median`) |
| E8 Kalman | ✖ 과함 |
- 추천 기본 = **E6: outlier reject + EMA** (+ prev-hold).

### 축 F — fallback / fail-safe (오프라인은 주석만)
| state 표기 | 조건 | 차량 의미(추후) |
|---|---|---|
| `OK` | conf 충분 | 정상 주행 |
| `LOW_CONF(slow)` | conf < `conf_low`*1.6 | 감속 |
| `OUTLIER(reject)` | center 급변 | 직전값 유지 |
| `HOLD(prev)` | 검출 실패, lost < N | 직전 조향 유지 |
| `LOST(stop)` | lost ≥ `lost_stop_frames` | 정지 |
- 파라미터: `conf_low=0.25`, `lost_stop_frames=8`.
- 추천 기본 = **F7: confidence 감속 + prev hold + N프레임 lost stop**.

---

## 4. 실험군(조건 기반 6군) — 2025 대시캠 실측 근거

기존 M(흰)/O(노랑) **알고리즘 누적** 프리셋을 폐기하고, 실제 두 트랙에서 관찰된 **조건(color × 기하 × 연속성)** 기준 6군으로 재정의한다(→ [PIPELINE.md](PIPELINE.md) §0). 각 군은 프리셋 1개가 아니라 그 조건에서 스윕할 파라미터 세트. **밴드·ROI 값은 2025 대시캠 실측으로 보존**(§아래), 2026 차량 카메라에서 재보정.

| # | group | colors/seg | ROI (top/topW) | extraction | temporal | 근거 클립 |
|---|---|---|---|---|---|---|
| **G1** `white_line` | white (S≤60,V≥185) | 0.35 / 0.80 | split=center | EMA | 401,403 |
| **G2** `white_curve` | white+lab (V≥175) | 0.35 / 0.80 | split=prev_row, polyfit+curv, 단일선 fallback | median+EMA | 411,413 |
| **G3** `yellow_solid` | yellow (H18-36,S≥65,V≥100) | 0.30 / 0.80 | split=prev_row, polyfit+curv | median+EMA | 408,409,410 |
| **G4** `yellow_dashed` | yellow | 0.30 / 0.80 | morph_kernel=5, aspect/length OFF | split=prev_row, EMA | 404,405 |
| **G5** `white_yellow` | white+yellow 융합 | 0.30 / 0.80 | split=prev_row | EMA | 404,406 |
| **G6** `robust_lowlight` | white+lab + adaptive fallback (느슨 S≤70,V≥160) | 0.35 / 0.80 | outlier reject | prev-hold+EMA | 407·전반 |

- 두 트랙 매핑: G1/G2 = 흰 엣지(2025 얇은선·2026 넓은도로), G3/G4/G5 = 노랑 로터리·지름길, G6 = 안전망. → [[track-2026-spec]].
- **1단계 preview**: 위 그룹 하나를 **적용만** 해서 3패널 렌더(비교·랭킹 없음). 그룹 비교는 **2단계** [perception_select](PIPELINE.md).

### 실측 밴드/ROI (보존값, 2025 대시캠)
- **흰색**: `white_s_max=60`, `white_v_min=185` (실측 S p95≈16, V p5≈207 → 기존 80/160보다 타이트, 노면 FP↓).
- **노랑**: `yellow_h_lo=18, yellow_h_hi=36, yellow_s_min=65, yellow_v_min=100` (실측 H 22-32/S≥66/V≥112, 기존 O밴드 검증됨).
- **ROI**: 흰/안전망 `roi_top_frac=0.35`, 노랑/혼재 `0.30`, 공통 `trap_bot_w=1.0, trap_top_w=0.80`. (히트맵: 하단 코너 강신호·중앙 하단 빈 삼각형·상단 30% 노이즈 → 세로 65~70%+넓은 상단 유리. **대시캠 FOV 기준, 차량 카메라 재보정 필요**.)

---

## 5. 실험 절차

```
Step 1  M1로 전체 클립 debug 결과 확인 (베이스라인)
Step 2  M2 vs M3 비교 — M2가 더 잘 잡는가 / M3가 오검출을 줄이는가
Step 3  M4로 heading_error 안정성 확인
Step 4  M5로 곡선 구간 center_error jitter 감소 확인
Step 5  M6 fallback이 실제로 필요한지 판단
Step 6  이미지 공간이 한계일 때만 M7(BEV) 실험
```

**ROI 스윕**: 각 모드에서 `--roi-top`을 0.5 / 0.55 / 0.6 / 0.65로 바꿔 배경 오검출 vs 원거리 차선 확보의 균형을 본다.

---

## 6. 사용법

```bash
# 로컬 venv 사용 (레포 루트의 .venv). 최초 1회 공유 코어 설치:
#   ../.venv/bin/pip install -e ../D-Racer-Kit/src/driving_core
# (perception_preview/perception_select 는 온라인 노드와 동일한 driving_core 를 import 한다)
# ※ 레포가 iCloud(~/Documents) 아래면 .pth가 hidden 처리되어 editable import가 깨질 수 있다.
#   해결: site-packages 에 소스 심볼릭 링크
#   ln -sfn "$(pwd)/D-Racer-Kit/src/driving_core/driving_core" .venv/lib/python3.13/site-packages/driving_core
cd offline
CLIP="Dashcam(2025 Track)/070401.mp4"

# ① 단일 그룹 미리보기 (3패널)
../.venv/bin/python perception_preview.py "$CLIP" --group G1

# ① 그룹 일괄 미리보기
for g in G1 G2 G5; do ../.venv/bin/python perception_preview.py "$CLIP" --group $g; done

# ① ROI 스윕 + 곡선 피팅 오버레이
../.venv/bin/python perception_preview.py "$CLIP" --group G2 --roi-top 0.30 --polyfit

# ② 그룹 비교(매트릭스+격자) + (선택) export
../.venv/bin/python perception_select.py "Dashcam(2025 Track)"/0704*.mp4 --frames 3
../.venv/bin/python perception_select.py "Dashcam(2025 Track)"/0704*.mp4 --export G5 \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml   # profile [perception] write
```

**perception_preview CLI**: `input`, `--group {G1..G6}`, `--colors white,yellow`, `--roi-top`, `--trap-top`, `--split {center,prev_frame,prev_row}`, `--heading {slope,two_point,norm_slope,hough}`, `--white-s-max/--white-v-min`, `--yellow-h LO HI`, `--yellow-s-min/--yellow-v-min`, `--morph`, `--polyfit`, `--curvature`, `--median`, `--dynamic-roi`, `--per-lane-conf`, `--lab`, `--output`. (그룹 프리셋을 CLI로 덮어써 스윕)
**출력**: `rslt/<클립명>__<그룹>.mp4`(preview), `rslt/perception_matrix.png`·`perception_grid.png`(select). 모두 git-ignore.

### 패널 해석
```
[ 원본 + ROI(노란 사다리꼴) + 중앙선 | 마스크(분리&ROI) | 검출: 초록점=행중심, 자홍선=2차피팅, 빨강선=EMA중심, 좌상단 상태텍스트 ]
```
- `center_err`: 횡오차([-1,1], +우측). `ema`: 평활값. `heading`: 진행각(방법에 따라 deg 또는 무차원). `conf`: 유효행 비율. `L/R conf`: 좌/우 개별. `curv`: 상대곡률. `state`: F축. 파랑선=split 기준.
- split-ref 파랑선(원본 패널)이 곡선/분기에서 어떻게 따라가는지 보면 split 기준 선택 효과를 확인할 수 있다.

---

## 7. 지각 지표 & 선택 export (perception_select.py)

여러 모드를 같은 프레임에서 비교(시각)하는 데 더해, **클립 전체에서 검출 품질 지표**를 집계해 정량 랭킹한다. 이 지표들은 **컨트롤러와 무관**한 순수 지각 품질이다(제어 지표는 [CONTROL_DESIGN.md](CONTROL_DESIGN.md) §5).

| 지표 | 정의 | 좋음 |
|---|---|---|
| **coverage(검출율)** | `conf ≥ τ` 인 프레임 비율 | 높을수록 |
| **center bias** | `center_error` 평균 | 0에 가까울수록(대칭) |
| **center jitter** | `center_error` std / 프레임간 |Δ| | 낮을수록(안정) |
| **heading jitter** | heading 프레임간 변동 | 낮을수록 |
| **L/R 균형** | `per_lane_conf` 좌/우 검출률 차 | 작을수록 |
| **일관성** | 인접 프레임 center 급변(outlier) 빈도 | 낮을수록 |

- **선택 → export**: 최적 mode + 파라미터를 profile YAML의 `perception:` 섹션에 in-place로 write(`--export <track>.yaml`). 나머지 섹션(`control:` 등)은 보존. 이 파일이 온라인 `perception_node`가 로드하는 계약(→ [PIPELINE.md](PIPELINE.md) §3).
- 지표는 어디까지나 후보를 좁히는 1차 필터이며, **최종은 시각(3패널) 확인과 병행**한다.

## 8. 설계 결정 / 가정

- **가정(확인 필요)**: 실제 대회 트랙에는 **중앙 점선이 없다**(현장 확인 전 가정). 테스트 클립에는 점선이 있음.
  → 따라서 테스트 주행에서 **점선 검출 여부는 무관**하며, 종횡비/길이 필터가 점선을 제거해도 문제 없음(오히려 유익).
- **종횡비 필터 on/off 두 버전 모두 보존**한다. 실제 트랙에 점선이 없다면 필터 on(뭉툭한 잡물 제거)이 유리하나, 만약 점선을 살려야 하는 트랙이면 off 버전을 쓴다. 프리셋/파라미터로 전환 가능하게 유지.

## 9. TODO (구현 대기/예정)

- [x] contour **종횡비/길이 필터** (`min_aspect`/`min_length`, 기본 off) — 물체 블롭 제거 확인. §7대로 on/off 보존.
- [x] **heading_error 옵션화** — slope(참고용, 과장) / two_point(근·원 2점) / norm_slope(무차원) / hough 4종 선택. 부호 규약: **+ = 진행방향 우측**. (robust fit은 추후)
- [x] M4(Hough heading) / M5(polyfit+median+curvature) / M6(Adaptive fallback) 구현.
- [x] 동적 ROI: 이전 프레임 EMA 중심으로 ROI 재중심(`--dynamic-roi`).
- [x] (옵션) 좌/우 **개별 신뢰도**(`--per-lane-conf`) 및 lane_width **이상치 처리**(`--lane-width-tol`).
- [ ] (선택) M7 BEV: 카메라 각도 확정 후 4점 캘리브레이션.
- [ ] 검출 확정 후 → 제어 매핑(P/PID)·시간적 확인은 **별도 노드/문서**로.

## 10. 대회장 운용 메모

- 이 구조의 목표: 현장에서 **알고리즘을 새로 짜지 않고**, `--mode` 선택 + threshold/ROI/필터 파라미터 튜닝만으로 대응.
- 실제 트랙은 조명·차선색·곡률·배경·카메라 각도가 이 클립과 다르므로, 위 실험 절차로 후보를 좁혀 두고 당일 최종 선택.
- 확정 로직만 온보드 `opencv_node`(또는 신규 `lane_detect` 노드)로 이식하여 D3-G에서 지연/FPS 검증.
