# 차선 인지 (Perception)

확정된 단일 인지 파이프라인. front-view 카메라 한 장에서 차선을 검출·분류하고
자차 통로 중앙선을 추정해 `/lane/state`(center_error·heading·confidence·…)로
발행한다. 실험용 프리셋(구 G1~G6)·모드는 없다 — 파이프라인은 하나다.

- 코어(순수, ROS 무관): [dracer_core/perception_core.py](D-Racer-Kit/src/dracer_core/dracer_core/perception_core.py)
- 온라인 노드: [perception/perception_node.py](D-Racer-Kit/src/perception/perception/perception_node.py) — `/camera/image/compressed` → `/lane/state` (+ `/lane/debug/compressed` 디버그 오버레이)
- 오프라인 시각화/실험: [offline/perception_probe.py](offline/perception_probe.py) (BEV 실험 포함)

---

## 1. 흐름

```
frame ─► ① detect ─► ② ROI 크롭 ─► ③ sliding-window 다차선
                                        │
        ⑦ state ◄─ ⑥ Tracker ◄─ ⑤ ego 중앙선 ◄─ ④ 7-label 분류
```

1. **detect** — HSV 흰/노랑 마스크 + morphology + 색 지배 게이트
2. **ROI 크롭** — 사다리꼴 ROI로 마스크 크롭(하늘·차체 제거)
3. **sliding-window** — 하단 히스토그램 base → 곡률(방향 EMA) 따라 창을 위로 쌓아 차선 궤적 추출
4. **분류** — 색(흰/노랑) + 위치(base x) + 방향(heading·곡률)으로 7라벨: `W-L/R`, `YS/YL/YR-L/R`
5. **ego 중앙선** — 인접한 좌/우 경계 쌍의 중앙선; 쌍이 없으면 단일 차선에서 차선폭 절반만큼 coast
6. **Tracker** — ego 좌/우 계수 EMA + 차선폭 유지 + turn
7. **state** — center_error·ema·heading·confidence·curvature·state·used_fallback

## 2. 기법

| 단계 | 기법 | 요점 |
|---|---|---|
| detect | HSV 색 마스크 + 색 게이트 | 소수 색 비율 < `color_gate`면 그 색 통째 제거(노이즈) |
| ROI | 사다리꼴 크롭 | `roi_top_frac`·`trap_top_w`·`trap_bot_w` |
| sliding-window | base 히스토그램 → 방향 EMA 곡선추종 | 점선도 창이 간격 건너뛰며 이어줌. 세로 span 미달(정지선 등)은 탈락 |
| 병합 | **폴리라인 근접(MAX \|Δx\|)** | 두 스택이 전 구간 일치(max<`merge_dx`)면 같은 차선. **한쪽 끝이라도 벌어지면 병합 거부** → Y분기·원근수렴쌍 보존 |
| 분류 | heading 우선 + 곡률 보조 | 노랑 곡률로 직진S/좌L/우R |
| 쌍매칭 | y겹침 + 최소간격 게이트 | 교차(간격 붕괴)·비평행 쌍 기각 |
| coast | 단일 차선 ± 차선폭/2 평행이동 | **관측 y-span으로 클램프**(외삽 발산 차단). 고정폭이라 정확도 한계 → BEV로 근본 개선 예정 |
| 안정화 | center_error EMA + 상태 명명 | OK/LOW_CONF/OUTLIER/HOLD/LOST |

## 3. 좌표계 — front-view (BEV 보류)

원래 오프라인 실험 도구는 ROI 사다리꼴→직사각형 **원근 워프(근사 BEV)** 위에서
동작했으나, 온라인은 **워프를 제외하고 front-view 원본 마스크**에서 돈다(coeffs가
이미 이미지 좌표라 그리기도 워프백 불필요). 카메라 캘리브레이션 기반의 **제대로 된
BEV는 후속 단계**(perception_core에 워프 지점 seam 주석). front-view에서는 차선이
소실점으로 수렴하므로 특히 **단일 차선 coast의 정확도가 낮다**(알려진 한계).

## 4. 파라미터 · 실시간 튜닝

`perception_core.Cfg`의 모든 필드가 perception_node의 **live ROS 파라미터**다.
profile `[perception]` 섹션이 seed하고, 실행 중 다음처럼 즉시 반영된다(파이프라인
재생성):

```bash
ros2 param set /perception_node roi_top_frac 0.25
ros2 param set /perception_node merge_dx 40.0
ros2 param set /perception_node colors "['white']"
```

주요 튜닝 대상: `roi_top_frac`·`trap_top_w`·yellow 밴드(`yellow_h_lo/hi`,
`yellow_s_min`, `yellow_v_min`)·`sw_margin`·`merge_dx`·`pair_gap_min`·
`lane_width_default`. `drive`/`perceive` 런치에서 monitor(:5000)가
`/lane/debug/compressed` 4패널을 저지연 스트림하므로 조정 결과를 바로 본다.

## 5. state 계약 (LaneState)

| 필드 | 의미 |
|---|---|
| `center_error` | 정규화 횡오차 [-1,1], + = 우측 (없으면 NaN, `valid`로 게이트) |
| `ema` | center_error EMA |
| `heading` / `heading_label` | ego 중앙선 접선각(deg) / `'ego'` |
| `confidence`·`left_conf`·`right_conf` | 검출 신뢰도 |
| `curvature`·`has_curvature` | ego 중앙선 곡률 |
| `state` | OK/LOW_CONF/OUTLIER/HOLD/LOST |
| `used_fallback` | coast(단일 차선) 사용 여부 |

제어(`control_core`)가 이 state를 받아 조향·스로틀로 매핑한다 — 인지는 차량을
조작하지 않는다.

## 6. 알려진 한계

- **단일 차선 coast**: 고정 차선폭 평행이동이라 front-view 원근에서 부정확(전체
  ~40% 프레임에서 발생). 클램프로 발산은 막았으나 근본 해결은 캘리브레이션 BEV.
  실차에선 coast 구간을 저속/hold로 다루는 게 안전.
- **분기(Y)**: MAX 병합이 분기를 분리 유지하고 방향 라벨(YL/YR)로 구분하나, 어느
  갈래로 갈지는 상위 미션 로직의 몫.

## 7. 검증 이력

Dashcam+Mission 31클립 27,438프레임 전수: 크래시 0·NaN 0·분기 오병합 0(진짜 분기
41프레임 전부 분리 유지)·총 상실률 1.2%. 상세는 [REFACTORING.md](REFACTORING.md).
