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

**Step 1** — monitor + camera 노드 + 수동주행 노드(control, joystick) 실행
- 위치: 차량(D3-G) · Launch 1
- 상태: ✅ 노드 존재 (camera/control/joystick/monitor)

**Step 2** — 수동 1차 주행 + 실시간 모니터링으로 **카메라 각도/높이 조절** + **steering_trim·accel_ratio 조정**
- 위치: 차량 · **Launch 1**
- 조작: joystick `calibration_mode` Y/B(trim), L1/R1(accel_ratio), START(세션), X(E-stop)
- 저장: trim·accel_ratio → `config/vehicle_config.yaml` `STEER_TRIM`·`ACCEL_RATIO` ✅ (조정 시 즉시 저장, 이후 모든 launch가 로드)
- → **Launch 1** ✅ `control/launch/calibrate.launch.py` (`camera + control + joystick[calibration_mode] + monitor`). D3-G 검증 대기.

### 🚗 2차 주행 — 주행영상 확보

**Step 3** — 고정 카메라 조건에서 수동 주행 + START 버튼으로 **카메라 영상 저장**(지각 없음)
- 위치: 차량 · **Launch 2**
- 저장: `recorder_node`가 START→STOP마다 원본 `raw_<ts>.mp4`(+csv) ✅ (image_topic=`/camera/image/compressed` 원본)
- → **Launch 2** ✅ `control/launch/record_manual.launch.py` (`camera + control + joystick + recorder`, perception 미포함). D3-G 검증 대기.

### 💻 오프라인 — 트랙 컨디션 분석 & 지각 선정

**Step 4** — 수동 영상 기반 **트랙 컨디션 파라미터 분석**(밴드값·색상필드·ROI·차선폭)
- 위치: 오프라인(로컬)
- → **⬜ `offline/track_analyze.py`** (신규): 클립에서 흰/노랑 HSV 분포, ROI 히트맵, 차선폭 측정 → G1~G6 밴드/ROI 기본값 산출.
  (2025 대시캠에 대해 이미 프로토타이핑: white S60/V185, yellow H18-36/S65/V100, roi 0.30-0.35/T0.80 — [offline/LANE_DETECTION.md](offline/LANE_DETECTION.md) §4)

**Step 5** — (오프라인 1단계) 도출 파라미터로 **G1~G6 적용 테스트**
- → ✅ **`offline/perception_preview.py`** (`--group G#`, 3패널)

**Step 6** — (오프라인 2단계) 결과 비교·분석 → **최적 검출 로직 선정**
- → ✅ **`offline/perception_select.py`** (group×clip 복합점수 매트릭스 + 검출 격자, 사람이 profile `perception:` export)
- 산출: `config/profiles/<track>.yaml` [perception]

### 🚗 3차 주행 — 지각 예측 + 수동 로그 동시 확보

**Step 7** — 지각 검출(예측값+결과영상) + 수동주행(주행로그) + 주행영상 저장하며 2번째 수동 주행
- 위치: 차량 · **Launch 3**
- 기본 세팅: 주행 파라미터 초기값 분석(steering/throttle min·max·scale·bias)
- 저장: `recorder_node` → `drive_<ts>.mp4 + .csv`(LaneState + 수동 command 동기) ✅
- → **Launch 3** ≈ 기존 `online_manual.launch.py` (`camera + control + joystick + perception + recorder`) 🔧

### 💻 오프라인 — 제어 선정

**Step 8** — (오프라인 3단계) 3차 주행 기록 기반 **제어값 계산 로직 테스트**
- → **⬜ `offline/control_predict.py`** (영상+수동CSV+perception profile → 컨트롤러별 명령 open-loop 예측 → 예측 CSV)

**Step 9** — (오프라인 4단계) 계산 로직 비교·분석 → **최적 제어 로직 선정 + online 전달 파라미터/로직 output 생성**
- → **⬜ `offline/control_select.py`** (open-loop 제어지표 랭킹, profile `control:` export)
- 산출: `config/profiles/<track>.yaml` [control] — 완성된 profile

### 🖥 온라인 — 자율주행 & 보정

**Step 10** — yaml(profile) 기반 online 코드 적용
- → ✅ `perception_node`/`driving_node`가 `profile` 파라미터로 로드([offline/PIPELINE.md](offline/PIPELINE.md) §3). 🔧 (필드 계약 최신화 완료)

**Step 11** — 차선검출 + **자율주행(수동개입 없음)** + START 기록(제어로그+검출) 하며 3번째 주행(자율)
- 위치: 차량 · 기존 `online_auto.launch.py` (driving_node engage 게이트) ✅
- 안전: 바퀴 들고 방향확인 → 저속 트랙, X E-stop 상시, watchdog

**Step 12** — 3차(자율) 기록 기반 **파라미터 보정**
- → **⬜ online 보정 코드**: 자율 기록(제어로그+검출)에서 setpoint·게인 편차를 재추정해 profile 갱신(오프라인 재피팅 or 온라인 보정 노드). 설계 논의 대상.

---

## 파일 매핑 요약

| 종류 | 파일 | 단계 | 상태 |
|---|---|---|---|
| launch | **Launch 1** `calibrate.launch.py` | 1-2 | ✅ (accel_ratio 저장 포함, D3-G 검증 대기) |
| launch | **Launch 2** `record_manual.launch.py` | 3 | ✅ (recorder 원본영상, perception 없음) |
| launch | **Launch 3** `online_manual.launch.py` | 7 | ✅ 기존 재사용 (camera+control+joystick+perception+recorder) |
| launch | **최종 자율** `online_auto.launch.py` | 11 | ✅ (perception+driving[engage]+recorder) |
| offline | `track_analyze.py` | 4 | ⬜ |
| offline | `perception_preview.py` | 5 | ✅ |
| offline | `perception_select.py` | 6 | ✅ |
| offline | `control_predict.py` | 8 | ⬜ |
| offline | `control_select.py` | 9 | ⬜ |
| online | profile 로더(perception/driving_node) | 10 | ✅ |
| online | 보정 코드/노드 | 12 | ⬜ |

## 진행 순서 (권장)
1. **Launch 1~3 리팩토링** + accel_ratio 저장 기능 (1차·2차·3차 주행 준비)
2. `track_analyze.py`(Step 4) — 컨디션 파라미터 자동 산출
3. `control_predict`/`control_select`(Step 8-9) — 제어 오프라인 4단계
4. Step 12 보정 코드 — 자율 기록 기반 재피팅

각 launch/코드는 **차량 안전 규칙**(바퀴 들고 검증 → 저속, E-stop 상시, 무단 액추에이션 금지)을 준수한다.
