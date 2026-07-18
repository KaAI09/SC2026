# KaAI SC2026

SEA:ME Hackathon 2026 — TOPST D3-G 기반 자율주행 스케일카.

카메라 한 장을 **캘리브레이션된 metric BEV**로 펴서 차선을 검출하고, 자차 통로(코리도어)의
중앙선을 추정해 조향·스로틀로 매핑한다. 인지 파이프라인은 **하나**다 — 실험용 모드·프리셋은
없다.

| 문서 | 내용 |
|---|---|
| **이 문서** | 시스템 구조 · 인지 · 제어 · 최종 결과 |
| [Task command.md](Task%20command.md) | 런치별 운영 개요 (calibrate / collect / drive / lap) |
| [offline/README.md](offline/README.md) | 오프라인 도구 (panel_replay · calibrate) |
| [CLAUDE.md](CLAUDE.md) | Claude Code 작업 규칙 |
| `D-Racer-Kit/docs/`, `Notice/` | 공식 하드웨어·규정 문서 (**1차 기술 참조 — 수정 금지**) |

---

## 1. 저장소 구조

```
D-Racer-Kit/src/     ROS2 패키지 (아래 §2)
D-Racer-Kit/scripts/ 하드웨어 보조 스크립트 (actuation_test · pwm_calibrate · camera_diagnostics …)
D-Racer-Kit/docs/    공식 하드웨어·SW 가이드
offline/             ROS 없이 도는 분석·튜닝 도구 (macOS, 레포 .venv)
Notice/              대회 미션·규정 자료
Env/                 팀 아키텍처·워크플로 문서
```

## 2. 시스템 구조

### 패키지 (11개)

| 패키지 | 역할 |
|---|---|
| `camera` | GStreamer → `/camera/image/compressed` |
| `perception` | 카메라 → **차선 파이프라인 + 객체 검출** → `/lane/state` + `/mission/state` |
| `control` | LaneState → `/control` + engage/E-stop 안전 게이팅 |
| `actuator` | PCA9685 조향/스로틀 구동. **E-STOP 이 여기서 걸린다** |
| `joystick` | 패드 입력 → 조향/스로틀 + 캘리브레이션 |
| `recorder` | 동기 mp4 + csv 기록 (**rosbag 아님**) |
| `monitor` | Flask 웹 대시보드 (:5000) |
| `battery` | INA219 배터리 상태 |
| `dracer_core` | **ROS 비의존** 인지·제어·객체검출 코어 (numpy/opencv만). 노드 없음 — 온라인·오프라인이 이걸 공유한다 |
| `dracer_msgs` | 메시지 정의 (Battery · Control · Joystick · LaneState · MissionState) |
| `dracer_bringup` | 런치 전용 |
| `topst_utils` | HW 드라이버 (d3racer · gamepads · ina219 · pca9685) |

> 실행 파일 이름은 전부 `*_node` 다 (`perception_node`, `control_node`, …).
> `dracer_core` · `dracer_msgs` · `dracer_bringup` · `topst_utils` 에는 노드가 없다.

### 노드 · 토픽

```
camera_node ──/camera/image/compressed──┬─► perception_node ──/lane/state─────┬─► control_node ──/control──┐
                                        │      ├──/mission/state ─────────────┤                            │
                                        │      └──/lane/debug/compressed      │                            │
                                        └─► monitor(:5000) · recorder         └─► recorder                 │
joystick_node ──/joystick─────────────────────────────────────────────────────► control_node               │
                                        └─────────────────────────────────────────────────────────────────► actuator_node
battery_node ──/battery_status──► monitor
```

- **`perception_node` 는 차선과 객체를 한 노드에서 돌린다.** 같은 프레임, 같은 stamp 로
  imdecode 1회 · HSV 1회 · GRAY 1회 · 디버그 인코드 1회. "표지판을 언제 봤고 분기가 언제
  떴나" 를 한 프레임 위에서 물을 수 있다.
- `/lane/debug/compressed` (디버그 패널)는 **구독자가 있을 때만 생성**된다 — 렌더링이 두 검출기를
  합친 것보다 비싸다. 그래서 **모니터를 어디에 붙이느냐가 곧 비용의 스위치**다: raw 카메라에
  붙이면 0, 디버그 토픽에 붙이면 그 비용을 자기가 요청한 것이다. `lap` 은 모니터가 아예 없다.
- `actuator_node` 와 `monitor_node` 는 **구독만** 한다.

### 런치 (`dracer_bringup`)

**core** = camera · actuator · joystick. 어떤 런치에서도 빠지지 않는다 — 조이스틱이
**E-STOP(X) · engage(A) · 녹화(START)** 경로다. `monitor`·`battery` 는 런치가 골라 붙인다.

| 런치 | 주행 | 인지 | 제어 | 모니터 | 녹화 | 용도 |
|---|---|---|---|---|---|---|
| `calibrate` | 수동 | ✗ | ✗ | 원본 | ✗ | 카메라 각도 + 서보 중립(`SERVO_CENTER_US`)/`ACCEL_RATIO` **저장** |
| `collect` | 수동 | ✓ | ✗ | 원본 | 원본 + csv | **데이터 수집** (패널은 오프라인에서) |
| `drive` | 자동 | ✓ | ✓ | 선택 | 원본 + csv | 자율주행 **튜닝·검증** (`engage`) |
| `lap` | 자동 | ✓ | ✓ | **✗** | **✗** | **랩타임 측정** (최경량) |

- **녹화는 언제나 원본 + csv 뿐이다.** 4패널 mp4 는 만들지 않는다 — `offline/panel_replay.py`
  가 raw+csv 에서 정확히 되살린다. 차가 자기도 안 보는 영상을 렌더링하느라 프레임을 흘릴
  이유가 없다.
- **`lap` = `drive` − 모니터 − recorder.** 웹으로 JPEG 를 스트리밍하고 mp4 를 인코딩하는 차는
  타임드 랩을 달릴 차가 아니다. 켜둔 채 잰 랩타임은 실제보다 느리다.

운영 명령은 [Task command.md](Task%20command.md).

### 설정 파일 (`D-Racer-Kit/src/config/`)

| 파일 | 성격 |
|---|---|
| `vehicle_config.yaml` | **D3-G 로컬 캘리브레이션** — `SERVO_*`(조향 서보 실측), `ACCEL_RATIO`, 카메라 장치·해상도. `git reset --hard` 가 덮어쓴다 |
| `camera.yaml` | 렌즈 K/D + 지면 호모그래피 H + BEV 범위. `offline/calibrate.py` 산출 |
| `profiles/track.yaml` | 인지·제어 파라미터 = **오프라인↔온라인 유일 계약**. 손으로 편집한다 |

**카메라 마운트를 움직이면 `camera.yaml` 의 H 가 무효가 된다.** 지면 사진 한 장으로 다시
뽑아야 한다 (K/D 는 살아남는다).

---

## 3. 인지 (`dracer_core/perception_core.py`)

### 파이프라인

```
frame ─► ① 행 크롭 ─► ② 색 마스크 ─► ③ BEV 워프 ─► ④ morph·색게이트 ─► ⑤ 슬라이딩 윈도우
         (BEV가 읽는 행만)  (HSV, 워프 전)   (remap LUT)      (워프 후)          (흰/노랑 별도)
                                                                                    │
      ⑨ state ◄─ ⑧ Tracker ◄─ ⑦ ego 코리도어 ◄─ ⑥ 폴리핏(2차) + 중복 병합 ◄────────┘
```

**① 행 크롭 · ② 색 마스크 — 워프 *전에* 임계한다.** BEV LUT 가 실제로 샘플링하는 raw 행
밴드만 HSV 변환한다. 색은 per-pixel 판정이라 리샘플 전에 임계하는 게 더 싸고 더 선명하다 —
보간된 중간색이 HSV 게이트를 속이지 않는다. 3채널 프레임이 아니라 **1채널 마스크 2장**을
워프한다.

**③ BEV 워프.** `cv2.remap` LUT 한 번에 undistort + warp 가 합성돼 있다 (BEV px → `inv(H)` →
왜곡 재적용 → raw px). 마스크는 `INTER_NEAREST`. **LUT 자체가 캘리브레이션된 사다리꼴
크롭이다** — 그 위에 손튜닝 ROI 사다리꼴을 또 얹을 이유가 없다(크롭 위의 크롭이 된다).

**④ morphology·색 게이트 — 워프 *후*.** BEV 에서는 1픽셀이 곧 1거리라 커널·면적 임계가
26cm 에서도 78cm 에서도 같은 뜻을 갖는다. 소수 색 비율이 `color_gate` 미만이면 그 색을 통째로
버린다.

**⑤ 슬라이딩 윈도우.** 하단 절반 히스토그램에서 base peak → 방향 EMA 로 창을 위로 쌓는다.
흰/노랑이 **따로** 돈다. 점선도 창이 간격을 건너뛰며 잇는다. `miss` 는 **첫 hit 이후에만**
센다 (선행 빈 창을 미스로 세면 짧은 차선이 통째로 탈락한다).

**⑥ 폴리핏 + 병합.** `x = a·y² + b·y + c`. 두 스택의 공유 y 구간을 7점 샘플해 **MAX** |Δx| 가
`merge_dx` 미만이면 같은 테이프로 보고 하나를 버린다. **한쪽 끝이라도 벌어지면 병합하지
않는다** → Y 분기·원근수렴쌍이 보존된다.

**⑦ ego 코리도어** — §3.2.
**⑧ Tracker** — 프레임 간 좌/우 정체성 + 차선폭 유지.
**⑨ state** — §3.4.

### 3.1 좌표계 — metric BEV

`camera.yaml` 이 **필수다.** 없으면 `LanePipeline` 이 `ValueError` 를 던진다 — 차선폭·쌍매칭·
coast 같은 물리적 게이트는 원근 이미지에서 아예 물을 수 없기 때문이다. 1픽셀 = 1/`px_per_cm` cm
이므로 차선폭·쌍매칭·coast·heading 이 전부 **실제 거리**다.

**런타임 해상도는 자유 변수다.** `camera.yaml` 은 캘리브한 해상도로 저장하고, perception 이
카메라가 실제로 보내는 크기에 `CameraModel.match()` 로 정확히 rescale 한다. 둘이 어긋나면
`to_bev()` 가 **예외를 던진다.**

**차선폭 35cm 의 근거**: 테이프 실측은 내부 32cm / 외부 38cm (폭 3cm) 인데, 검출 다항식은
테이프의 **중심선**을 따라간다 → 코드가 쓰는 폭은 **중심선 간 35cm** 다.

### 3.2 코리도어 — 색이 경로다

두 차선을 짝지어 코리도어를 만든다. 후보는 **모든 조합**(`itertools.combinations`)이다. 인접
쌍만 보면 로터리에서 `[흰, 노랑, 흰]` 이 잡힐 때 진짜 코리도어인 (흰,흰)=35cm 가 후보에 오르지도
못한다 — 보이지 않는 갈림길은 선택할 수 없다.

`_pair_gate` 통과 조건 (전부 AND):

1. **`pair_same_color`** — 흰-노랑 쌍 금지. **색 조합이 곧 경로 정체성이다**: 본선은 흰-흰,
   노란 지름길은 노랑-노랑. 흰-노랑은 물리적으로 한 차선의 두 경계일 수 없다 — 두 경로의
   경계가 교차점에서 우연히 35cm 떨어진 것이다. 폭 게이트로는 못 잡는다(둘 다 진짜로 35cm).
2. y 겹침 ≥ `pair_overlap_min`
3. 최소 간격 ≥ `pair_gap_min_cm` (교차·붕괴 기각)
4. 폭이 `lane_width_cm` ± `pair_width_tol`

> **평행도(`spread`)는 게이트가 아니다 — 보고만 한다.** 경계를 공유하는 진짜 갈림길에서는
> 두 번째 코리도어가 벌어지는 게 당연하다 — 그게 갈림길이니까. 평행도는 "이것이 코리도어인가"
> 가 아니라 **"내가 이 안에 있는가"** 다. 그래서 `pair_parallel_cm` 은 `ego_center` 의
> `nearest` 규칙에서만 쓴다.

**ego 선택 우선순위**: **`fork_L`/`fork_R`** (use_fork + 섬 감지 + 표지판 hint — 섬 바깥
한쪽으로 회피) → `tracked` (Tracker 가 따르던 코리도어) → `nearest` (차량축에 가장 가까운
in-lane 코리도어) → `coast` (경계 하나만 보고 차선폭 절반 평행이동).

**분기(`n_corridors >= 2`)는 진입 시 한 번만 래치한다.** 프레임마다 새로 고르면 차선폭만큼
진동한다. 선택된 코리도어는 `Tracker.adopt` 로 정체성에 밀어넣는다.

### 3.3 coast — 마스크에 좌/우를 되묻는다

coast 는 경계 **하나**를 보고 "코리도어는 이것의 왼쪽/오른쪽에 있다" 고 **주장**한다. 그 주장이
틀리면 중심이 **차선폭만큼** 어긋난다 — 완벽한 차선 피팅을 달고서. **기하로는 절대 못 잡는다.**
그래서 기하가 못 묻는 것을 묻는다: 주장하는 자리에 유령 경계를 놓고 **거기 마스크가 있는지
본다.** 거울상 쪽에만 근거가 있으면 방향을 잘못 골랐다.

세 개의 가드가 전부 필요하다:

1. `coast_flip_support` — 거울상에 실제 근거가 있어야 한다.
2. `coast_flip_empty` — 우리 유령 쪽엔 근거가 없어야 한다. 없으면 동전던지기에서 뒤집는다.
3. **축 가드 — flip 은 코리도어를 차량축 쪽으로 당겨야 한다.** 마스크는 "저기 차선이 있다"
   는 말할 수 있어도 **"저게 내 차선이다"** 는 못 한다. flip 이 중심을 축에서 멀어지게 하면
   그건 옆 차선이다.

그리고 **교정은 트래커를 고쳐야 한다** (`reseat_coast`). 출력만 뒤집으면 다음 프레임에 여전히
틀린 정체성이 또 coast 하고 마스크가 또 뒤집는다 → 매 프레임 진동.

### 3.4 상태 (`LaneState.state`)

| 값 | 조건 |
|---|---|
| `OK` | 측정 채택, conf 충분 |
| `LOW_CONF` | 측정 채택, conf 낮음 |
| `OUTLIER` | `\|center_error − ema\| > outlier_jump` 로 거부. **거부가 `outlier_relatch_s` 동안 이어지면 사실로 받아들이고 재시드** |
| `HOLD` | 측정 없음, `lost_stop_s` 미만 |
| `LOST` | 측정 없음이 `lost_stop_s` 이상 |

`OUTLIER` 는 스로틀을 끊는다 (`throttle_outlier: 0.0`). "지금 내가 주는 `ema` 를 믿지 마라"
는 뜻이고, 코리도어 전환 직후엔 부호까지 반대다.

### 3.5 `/lane/state` 계약 (`dracer_msgs/LaneState`)

| 필드 | 의미 | 누가 쓰나 |
|---|---|---|
| `center_error` | 정규화 횡오차 [-1,1], **+ = 코리도어가 오른쪽** (invalid 시 NaN) | control |
| `center_error_cm` | **cm 단위 횡오차 (calib 불변)** — metric 분석·PP 는 이걸 쓴다 | 기록·분석 |
| `ema` | `center_error` EMA — **제어가 실제로 읽는 값** (`use_ema=True`) | control |
| `state` | `OK`/`LOW_CONF`/`OUTLIER`/`HOLD`/`LOST` | control (throttle) |
| `confidence` | **이산값**: 0.9(pair) / 0.5(coast) / 0.0(없음) | control (`conf_gate`) |
| `used_fallback` | coast 사용 여부 | 기록 |
| `n_corridors` | 유효 코리도어 수. **> 1 = 분기** | 판단 계층 |
| `ego_rule` | 무엇이 골랐나: `tracked`/`nearest`/`coast`/`fork_L`/`fork_R`/`branch_*`/`none` | 판단 계층 |
| `fork_type` / `n_islands` | 갈림길 감지: `''`/`island`/`branch` · island 코리도어 수 | 회피(`use_fork`) |
| `sign_hint` / `sign_hint_age` | 래치된 표지판 방향 `''`/`L`/`R` · 확정 후 프레임 수 | 회피·진단(기록) |
| `heading` / `curvature` | ego 중앙선 접선각(deg) / 곡률 | 미사용 |
| `left_conf` / `right_conf` | 0 또는 1 | 미사용 |

**판단 계층은 갈림길에 대해 존재한다.** `use_fork`(기본 true)가 켜지면 **갈림길(W-W 섬)에서
표지판 `sign_hint` 방향으로 한쪽 corridor 를 고른다**(`ego_rule` = `fork_L`/`fork_R`). 그 외의
일반 분기에서는 `tracked`(이어가기)/`nearest`(자리표시자) 규칙을 쓴다.

**라벨 분류(`W-L`, `YS-R`, …)는 계약에 없다** — 디버그 오버레이 전용이고 중앙선 도출에 쓰이지
않는다 (실제 라벨은 `W×2 + Y{S,L,R}×2` = 8개).

### 3.6 파라미터 — 진짜 노브는 cm 다

`Cfg` 의 필드가 `perception_node` 의 live ROS 파라미터로 선언된다. profile `[perception]` 이
seed 하고, `ros2 param set` 이 즉시 반영된다.

| cm 파라미터 | 뜻 |
|---|---|
| `lane_width_cm` 35.0 | 트랙 차선폭 (중심선 간) |
| `sw_margin_cm` 5.0 | 슬라이딩 창 반폭 |
| `merge_dx_cm` 5.0 | 같은 테이프로 병합할 근접도 |
| `jump_max_cm` 15.0 | 프레임 간 최대 차선 점프 |
| `pair_gap_min_cm` 8.0 | 쌍 붕괴(교차) 기각 |
| `pair_width_tol` 0.25 | 코리도어 폭 허용 오차 |
| `pair_parallel_cm` 8.0 | in-lane 판정(=ego 선택에만) |
| `morph_cm` · `sw_minpix_cm2` · `gate_min_cm2` · `sw_peak_min_cm` · `heading_cm` | 커널·면적·길이 |

> **`sw_margin`·`jump_max`·`merge_dx`·`morph_v`·`sw_minpix`·`sw_peak_min`·`sw_peak_sep`·
> `pair_gap_min`·`pair_parallel`·`gate_min_px`·`lane_width_default`·`heading_frac` 는
> 파라미터가 아니다.** `cfg_to_px` 가 위 cm 값에서 계산한다(`DERIVED_PX`) — ROS 가 `param set`
> 을 **거부한다.** 픽셀 *개수*는 면적이라 `px_per_cm` 의 제곱으로 변하므로, 면적(`cm²`)·길이
> (`cm`)로 표현해야 BEV 스케일 변경에 면역이다.

**시간 문턱은 초다 — 프레임이 아니다.** `ema_tau_s`·`outlier_relatch_s`·`lost_reset_s`·
`lost_stop_s` 는 전부 초 단위다. 프레임 수로 두면 그 값의 실제 의미가 그날의 FPS 가 된다 —
`outlier_relatch` 는 특히 "코리도어가 뒤집혀 **부호까지 반대인** 값으로 차가 조향해도 되는
시간" 의 상한이고, 그런 약속은 초로 말해야 뜻이 있다.

| 시간 파라미터 | 값 | = 30Hz 에서 |
|---|---|---|
| `ema_tau_s` | 0.065 | `alpha 0.4` (차선 계수 · 코리도어 폭 · center_error EMA 공용) |
| `outlier_relatch_s` | 0.16 | 5프레임 |
| `lost_reset_s` · `lost_stop_s` | 0.26 | 8프레임 |

`process(bgr, dt_s)` 가 dt 를 **요구한다** — 인지는 클록도 스탬프도 소유하지 않으므로 호출자가
준다(노드는 ROS 클록, 오프라인은 클립 fps). 프레임당 가중치는 매 프레임
`alpha = 1 − exp(−dt/tau)` 로 파생된다. `median_window` 만 프레임(샘플 수)으로 남았다 — 중앙값은
샘플 개수를 필요로 하니까.

**`ros2 param set` 은 상태를 보존한다.** 상태는 **측정값**(차선이 어디 있나, 이 코리도어가
얼마나 넓나)이고 설정은 **판정 기준**이다 — 기준을 바꾼다고 측정이 무효가 되지 않는다.
`LanePipeline.reconfigure` 가 `cfg` 만 갈아끼운다(같은 cfg 로 호출하면 결과가 비트 단위로
동일). 단 **BEV 기하(`cam`)가 바뀌면** 추적 중인 픽셀이 다른 장소를 뜻하므로 그 경로
(`_match_camera`)는 파이프라인을 재생성한다.

---

## 4. 제어 (`dracer_core/control_core.py`)

인지 레이트(30Hz, LaneState 마다 1회)로 도는 루프:

```
① dt      dt_s = clamp(dt, 0, dt_max)          ← 드롭아웃 직후 dt 는 갭 전체다
② 오차    e = center − center_target           (center = ema)
③ 제어기  u = f(e, ė, ∫e, heading, speed)      ← 내부값이다
④ 후처리  clamp(±steer_max) → slew_rate_per_sec·dt_s → out_ema
          → u · steer_sign                     ← 부호는 여기서 딱 한 번
⑤ 스로틀  throttle_base − curv_slow·|u|  (하한 throttle_min)
          state == OUTLIER → min(thr, throttle_outlier)   ← 하한을 우회한다
⑥ 안전    conf < conf_gate → 직전값 유지 · (노드단) 인지/조이스틱 워치독 · E-STOP
```

| 제어기 | 식 | 상태 |
|---|---|---|
| **PD** | `−(kp·e + kd·ė)` | 기본 |
| **PID** | `−(kp·e + kd·ė + ki·∫e)` | 선택 |

`controller: PD | PID`. **모르는 이름은 예외를 던진다** — 조용한 폴백은 프로파일 오타 하나로
적어둔 것과 **다른 제어기로 차를 달리게** 한다.

**PD 는 일정 곡률 코너에서 `e_ss = u_필요 / kp` 만큼의 정상상태 오차를 반드시 남긴다.** 적분항
없는 P 제어의 구조적 성질이다 — 곡선 횡오차는 서보 트림이 아니라 여기서 온다.

**부호 규약.** `center_error` + = 코리도어가 **오른쪽**. 법칙이 `u = −kp·e` 이므로 코리도어가
**왼쪽**(e<0)이면 `u > 0`. 이 차량(`steer_sign = +1.0`)에서 **`u > 0` = 좌조향**이다 — 차가
코리도어 쪽으로 돈다. 배선이 반대면 `steer_sign = -1`.

**`slew_rate_per_sec` 는 초당이다.** "프레임당" 이면 차의 최대 회전 속도가 그날의 인지 FPS 가
된다. 물리량으로 못박은 값이 4.5/s(@30Hz)다.

**조향 스케일은 서보 실측이 정한다.** `조향각(u) = 25·u` 도 — `u = ±1.0` 이 정확히 `±25도`이고
그 전 구간이 선형이다. 트림은 서보 중립(`SERVO_CENTER_US = 1650us`)에 있으므로 `u = 0` 이 곧
직진이고, `steer_max` 는 **1.0** 이다. `kp`·`slew_rate_per_sec` 는 u 단위라 서보 스케일에 묶여
있다.

**A(engage) 와 X(E-STOP) 는 다른 층이다.** A 는 `control_node` 에게 "그만 보내라" 고 **부탁**
한다. X 는 `actuator` 에게 "무시해라" 고 **명령**한다. `control_node` 가 고장나면 A 는 아무것도
못 한다.

**워치독**: 인지 dead-man `state_timeout: 0.25s`, 조이스틱 dead-man `joystick_timeout: 0.3s`.
둘 중 하나라도 끊기면 `/control` 이 `(0, 0)` 으로 고정된다.

---

## 5. 오프라인 도구

로컬 macOS 에서 ROS 없이 돈다. `dracer_core` 가 순수 파이썬이라 **차와 같은 알고리즘**을 그대로
돌린다. 상세는 [offline/README.md](offline/README.md).

- **`panel_replay.py`** — 주행 raw + csv → 4패널 재구성 + 실차 LaneState 대조 + 파라미터 A/B.
  인지 파라미터는 여기서 A/B 하고 profile 에 반영한다.
- `calibrate.py` — 체커보드·지면 사진 → `camera.yaml` (K/D + 호모그래피 H) + 검증(`--check`).

**제어기 튜닝은 실차 폐루프로만 한다.** 녹화 영상은 **사람이 지난 경로의 뷰**만 담으므로, 다르게
조향한 컨트롤러가 봤을 프레임은 존재하지 않는다(covariate shift). 오프라인으로는 명령 품질밖에
못 보고, 최종 판정은 실차다. profile 의 `[control]` 은 손으로 편집한다.

데이터: `offline/rslt/<세션>/` (raw + csv, 미추적), `offline/calib/` (캘리브 사진).

---

## 6. 최종 결과

### 자율주행

| | |
|---|---|
| 인지 레이트 | **30~31 Hz** (렌더링 없이 주행 레이트와 같다) |
| `state` (engage 중) | **OK** — 실차 자율주행에서 `LOST`·`OUTLIER` 0회 |
| 실차 ↔ 오프라인 재생 | **재현 일치** (`\|Δcenter_error\|` 중앙값 0.012) |
| 조향 포화 | 직선·완만 구간 0% / 급코너 12~26% (병목) |
| 직선 횡오차 | **+0.2cm** (정상상태 조향 ≈ 0 — 트림이 서보 중립에 있다) |
| 곡선 횡오차 | 코리도어 반폭(17.5cm) 안 — PD 의 `e_ss` 8cm 대 |

**직선은 완벽하고, 곡선 오차는 PD 의 구조적 `e_ss` 다.** 일정 곡률 코너는 일정한 외란이고,
적분항 없는 P 제어는 `e_ss = u_필요/kp` 를 반드시 남긴다. 급코너 조향 포화가 병목이라 `kp` 를
더 올리는 대신 `curv_slow`(급코너 감속)가 레버다.

### 인지 · 캘리브레이션

| | 값 |
|---|---|
| `valid` / `state OK` | 100% (직선) · 93~100% (12세션 대회장 수동주행) |
| 쌍검출 / coast | 직선 96~100% · 급커브에서는 coast (곡선 바깥 경계가 FOV 밖 — 설계된 동작) |
| 렌즈 캘리브 (K·D) | 체커보드 19장, 재투영 RMS **0.533px** |
| 카메라 높이 / 가시 지면 | 23.3cm / y 31~78cm · x ±29cm (BEV 232x189px @ 4px/cm) |
| 차선폭 (실측) | **34.8cm** (독립 캘리브 2회에서 34.78 / 34.76cm 로 재현) |

> **쌍검출률은 구간마다 다르다.** 직선 100%, 급커브 8%. 낮은 구간은 실패가 아니라 **곡선 바깥
> 경계가 FOV 에 없는 것**이고, 그때 `coast` 가 정확히 그 일을 한다 — 그 클립들도 `state` 는 OK 다.

### 미션

- **갈림길 회피(`use_fork`, 기본 true).** W-W 섬을 `(L,R)` 반대곡률 + spread/gap 게이트로
  감지하고, 표지판 `sign_hint` 방향으로 섬 바깥 한쪽 corridor 를 고른다. 일반구간 오검출 0%.
- **우회전 표지판 좌/우 구분.** RIGHT 세션은 `ego_rule=fork_R`+우조향, LEFT 세션은 `fork_L`+
  좌조향, 검출→확정→회피규칙→물리 조향까지 좌/우 정확(교차 오류 0).
- **신호등·ArUco.** `mission_gate`(기본 on)가 RED/MARK 정지·GREEN 출발을 담당한다.
