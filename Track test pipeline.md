# Track Test Pipeline (트랙 테스트 진행 파이프라인)

> 대회 두 트랙([[track-2025-test]]·[[track-2026-spec]], 범위는 이 둘만 — [[two-track-design-scope]])에서
> **수동 → 오프라인 분석/선정 → 온라인 자율**로 이어지는 표준 진행 절차. 앞으로 모든 트랙 테스트는
> 이 파이프라인을 따른다. 각 단계는 특정 **launch(차량)** / **offline 도구** / **online 코드**에 매핑된다.
> 설계 배경: 오프라인 4단계는 [offline/PIPELINE.md](offline/PIPELINE.md), 지각 [offline/LANE_DETECTION.md](offline/LANE_DETECTION.md),
> 제어 [offline/CONTROL_DESIGN.md](offline/CONTROL_DESIGN.md). 리팩토링 이력은 [REFACTORING.md](REFACTORING.md).

**상태 범례**: ✅ 구현됨 · 🔧 일부 존재(보완 필요) · ⬜ 신규 필요

---

## 개요 — 3차 주행 구조

| 주행 | 목적 | 저장물 | Launch |
|---|---|---|---|
| **1차 (수동)** | 카메라 각도·높이 세팅 + steering_trim·accel_ratio 튜닝/저장 | vehicle_config.yaml | **Launch 1** |
| **2차 (수동)** | 고정 카메라로 주행영상 확보 → 오프라인 분석 | drive.mp4 | **Launch 2** |
| **3차 (수동+지각)** | 지각 예측 + 수동 로그 + 주행영상 동시 저장 → 제어 분석 | mp4+csv(+예측) | **Launch 3** |
| **4차 (자율)** | 자율주행 + 기록 → 파라미터 보정 | 제어로그+검출 | online_auto |

오프라인은 그 사이사이 지각(1·2단계)·제어(3·4단계)를 분석해 **profile YAML** 하나로 수렴시킨다.

---

## 12단계 상세

### 🚗 1차 주행 — 세팅 & 캘리브레이션

**Step 1** — monitor + camera 노드 + 수동주행 노드(control, joystick) + battery 실행
- 위치: 차량(D3-G) · Launch 1
- 상태: ✅ 노드 존재 (camera/control/joystick/monitor/battery)
- monitor는 경량화됨 — 실시간 카메라 + 배터리 + 저장공간 3종만 표시(제어/녹화/그래프/디버그 패널 제거로 지연 최소화). 배터리 패널을 위해 Launch 1에 `battery_node` 포함.

**Step 2** — 수동 1차 주행 + 실시간 모니터링으로 **카메라 각도/높이 조절** + **steering_trim·accel_ratio 조정**
- 위치: 차량 · **Launch 1**
- 조작: joystick `calibration_mode` Y/B(trim), L1/R1(accel_ratio), START(세션), X(E-stop)
- 저장: trim·accel_ratio → `config/vehicle_config.yaml` `STEER_TRIM`·`ACCEL_RATIO` ✅ (조정 시 즉시 저장, 이후 모든 launch가 로드)
- → **Launch 1** ✅ `control/launch/calibrate.launch.py` (`camera + control + joystick[calibration_mode] + monitor + battery`). D3-G 검증 대기.

### 🚗 2차 주행 — 주행영상 확보

**Step 3** — 고정 카메라 조건에서 수동 주행 + START 버튼으로 **카메라 영상 저장**(지각 없음)
- 위치: 차량 · **Launch 2**
- 저장: `recorder_node`가 START→STOP마다 원본 `raw_<ts>.mp4`(+csv) ✅ (image_topic=`/camera/image/compressed` 원본)
- → **Launch 2** ✅ `control/launch/record_manual.launch.py` (`camera + control + joystick + recorder`, perception 미포함). D3-G 검증 대기.

### 💻 오프라인 — 지각(확정: 7-label BEV)

**Step 4-6** — 차선 검출·인지·지각 **7-label BEV 방식 확정** (2026-07-08)
- 위치: 오프라인(로컬)
- → ✅ **`offline/lane7_probe.py`**: BEV(원근제거) + sliding-window(방향 EMA 곡선추종) + 2차 polyfit, 7라벨(W-L/R·YS/YL/YR-L/R), 창 IOU 중복병합, heading+곡률 turn, pair-gate 중앙선, coast fallback, 6패널 시각화(독립 실행).
- 기존 front-view 탐색 도구 `track_analyze.py`·`perception_preview.py`(G1~G6)·`perception_select.py`는 **제거됨**.
- **온라인 BEV 통합은 실차 테스트 완료 후로 연기**: 향후 별도 BEV 코어(`driving_core/bev_lane.py` 등) + 카메라 캘리브레이션 코드 신설 예정. 그전까지 온라인 인지는 front-view `driving_core/lane_core.py` 유지, profile `[perception]`은 front-view baseline(자동 export 없음).

### 🚗 3차 주행 — 지각 예측 + 수동 로그 동시 확보

**Step 7** — 지각 검출(예측값+결과영상) + 수동주행(주행로그) + 주행영상 저장하며 2번째 수동 주행
- 위치: 차량 · **Launch 3**
- 기본 세팅: 주행 파라미터 초기값 분석(steering/throttle min·max·scale·bias)
- 저장: `recorder_node` → `drive_<ts>.mp4`(**다패널 디버그 영상** 입력+ROI\|mask\|검출, `/lane/debug/compressed`) + `.csv`(LaneState + 수동 command 동기) ✅. 현재 front-view 3패널, BEV 6패널은 실차 후 통합 시.
- → **Launch 3** ≈ 기존 `online_manual.launch.py` (`camera + control + joystick + perception + recorder`) 🔧

### 💻 오프라인 — 제어 선정

**Step 8** — (오프라인 3단계) 3차 주행 기록 기반 **제어값 계산 로직 테스트**
- → ✅ **`offline/control_predict.py`** (영상+수동CSV+perception profile → 컨트롤러 C1~C5 명령 open-loop 예측 → 예측 CSV). 실제 주행 CSV 검증은 3차 주행 후.

**Step 9** — (오프라인 4단계) 계산 로직 비교·분석 → **최적 제어 로직 선정 + online 전달 파라미터/로직 output 생성**
- → ✅ **`offline/control_select.py`** (open-loop 제어지표[smoothness·oscillation·response·saturation·human참조] 복합점수 랭킹, profile `control:` export)
- 산출: `config/profiles/<track>.yaml` [control] — 완성된 profile

### 🖥 온라인 — 자율주행 & 보정

**Step 10** — yaml(profile) 기반 online 코드 적용
- → ✅ `perception_node`/`driving_node`가 `profile` 파라미터로 로드([offline/PIPELINE.md](offline/PIPELINE.md) §3). 🔧 (필드 계약 최신화 완료)

**Step 11** — 차선검출 + **자율주행(수동개입 없음)** + START 기록(제어로그+검출) 하며 3번째 주행(자율)
- 위치: 차량 · 기존 `online_auto.launch.py` (driving_node engage 게이트) ✅
- 안전: 바퀴 들고 방향확인 → 저속 트랙, X E-stop 상시, watchdog

**Step 12** — 3차(자율) 기록 기반 **파라미터 보정**
- → **⬜ 보류(TODO)**: 자율 기록(제어로그+검출)에서 setpoint·게인 편차를 재추정해 profile 갱신(오프라인 재피팅 or 온라인 보정 노드).
- **진행 시점**: 전체 파이프라인 점검 + **실주행 테스트(D3-G) 완료 후**에 설계·구현. 실주행 데이터가 있어야 보정이 의미 있음.

---

## 파일 매핑 요약

| 종류 | 파일 | 단계 | 상태 |
|---|---|---|---|
| launch | **Launch 1** `calibrate.launch.py` | 1-2 | ✅ (camera+control+joystick+monitor[경량]+battery, accel_ratio 저장 포함, D3-G 검증 대기) |
| launch | **Launch 2** `record_manual.launch.py` | 3 | ✅ (recorder 원본영상, perception 없음) |
| launch | **Launch 3** `online_manual.launch.py` | 7 | ✅ 기존 재사용 (camera+control+joystick+perception+recorder) |
| launch | **최종 자율** `online_auto.launch.py` | 11 | ✅ (perception+driving[engage]+recorder) |
| offline | `lane7_probe.py` (7-label BEV, 지각 확정) | 4-6 | ✅ (온라인 BEV 통합은 실차 후) |
| offline | `control_predict.py` | 8 | ✅ (실주행 CSV 검증 대기) |
| offline | `control_select.py` | 9 | ✅ (실주행 CSV 검증 대기) |
| online | profile 로더(perception/driving_node) | 10 | ✅ |
| online | 보정 코드/노드 | 12 | ⬜ |

## 진행 순서 (권장)
1. **Launch 1~3 리팩토링** + accel_ratio 저장 기능 (1차·2차·3차 주행 준비)
2. `lane7_probe.py`(Step 4-6) — 7-label BEV 지각 확정 (온라인 BEV 통합은 실차 후)
3. `control_predict`/`control_select`(Step 8-9) — 제어 오프라인 4단계
4. Step 12 보정 코드 — 자율 기록 기반 재피팅

각 launch/코드는 **차량 안전 규칙**(바퀴 들고 검증 → 저속, E-stop 상시, 무단 액추에이션 금지)을 준수한다.
