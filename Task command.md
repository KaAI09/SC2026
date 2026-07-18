# 운영 개요 — 런치 · 조이스틱 · 토픽

실차 트랙 테스트 운영 레퍼런스. 시스템 구조·인지·제어·최종 결과는 [README.md](README.md).

- 🖥 **로컬(offline)** — macOS, ROS 불필요, 레포 `.venv`
- 🚗 **D3-G(online)** — 차량, ROS2 Humble, `colcon`

**안전(액추에이션 공통)**: 바퀴 지면에서 띄우고(wheels-off) 먼저 → 조이스틱 **X = E-STOP**
상시 → 저속 → D3-G 에서 코드 수정·commit 금지.

> **설정 요약** ([`track.yaml`](D-Racer-Kit/src/config/profiles/track.yaml) · [`camera.yaml`](D-Racer-Kit/src/config/camera.yaml)):
> `x_half_cm` 29 · `kp`/`kd` = 1.55/0.145 · `throttle_base`/`min` = 0.23/0.22 · `colors [white]` ·
> `use_fork` true · `sign_invert` false · `mission_gate` on(RED/MARK 정지·GREEN 출발).

---

## 런치 (🚗 D3-G)

`ros2 launch dracer_bringup <name>.launch.py`. `profile`·`camera` 생략 시 런치가 기본값
(`src/config/profiles/track.yaml`, `src/config/camera.yaml`)을 찾는다.

| 런치 | 주행 | 인지 | 제어 | 모니터 | 녹화 | 용도 |
|---|---|---|---|---|---|---|
| `calibrate` | 수동 | ✗ | ✗ | 원본 | ✗ | 카메라 각도 + 서보 중립/ACCEL_RATIO **저장** |
| `collect` | 수동 | ✓ | ✗ | 원본 | 원본 + csv | **데이터 수집** — 패널은 오프라인에서 |
| `drive` | 자동 | ✓ | ✓ | 원본(기본) | 원본 + csv | 자율주행 **튜닝·검증** (`engage`) |
| `lap` | 자동 | ✓ | ✓ | ✗ | ✗ | **랩타임 측정** (최경량) |

- **core** = camera · actuator · joystick (조이스틱이 E-STOP·engage·녹화 경로라 어떤 런치에서도
  빠지지 않는다). `monitor`·`battery` 는 런치가 골라 붙인다.
- **렌더링은 구독이 켠다.** 인지는 `/lane/debug/compressed` 에 구독자가 있을 때만 패널을
  합성·JPEG 인코딩한다. `drive` 모니터 기본값은 **RAW 카메라**(비용 0)이고, 패널은
  `monitor_topic:=/lane/debug/compressed` 로 명시해야 뜬다.
- **녹화는 항상 원본 + csv 뿐이다.** 4패널은 `offline/panel_replay.py` 가 raw+csv 에서 되살린다.
- **`drive`/`lap` 는 액추에이션이다.** `engage` 는 기본 false 로 시작하고, 바퀴를 띄운 상태에서
  조이스틱 A(또는 `param set /control_node engage true`)로 켠다. `mission_gate` 가 기본 on 이라
  engage 해도 **GREEN 을 볼 때까지 정지로 시작**한다(`mission_gate:=false` 로 순수 주행 확인).
- **랩타임은 `lap` 으로 잰다.** `drive` 에서 모니터·recorder 를 뺀 것 — 웹 스트리밍·mp4 인코딩은
  같은 보드의 CPU 를 먹어 랩타임을 실제보다 느리게 만든다.

기동이 정상이면 인지 30Hz, `control_node` 워치독(`state_timeout` 0.25s / `joystick_timeout`
0.3s), `actuator` 서보(center 1650us)가 로그에 뜬다. **Hz 가 30 이 아니면 멈춰라 — 게인은 30Hz
에서 튜닝됐다.**

---

## 오프라인 (🖥 로컬)

- `panel_replay.py` — 주행 raw + csv 를 4패널로 재구성, 실차 LaneState 와 프레임 단위 대조,
  파라미터 A/B. 인지 튜닝은 여기서 하고 profile 에 반영한다.
- `calibrate.py` — 카메라 마운트를 움직였으면 지면 사진으로 `H` 를 다시 푼다. `--check` 로 BEV
  가 metric 한지 검증(직선 구간 프레임 필요).

상세·주의는 [offline/README.md](offline/README.md). **제어기 튜닝은 실차 폐루프에서만**
(`ros2 param set /control_node …`) 한다 — 녹화 영상으로는 다르게 조향한 결과를 재현할 수 없다
(covariate shift).

---

## 조이스틱

| 버튼 | 기능 |
|---|---|
| Y / B | 서보 중립(`SERVO_CENTER_US`) ∓10us. 트림은 명령이 아니라 서보 중립에 있다 |
| L1 / R1 | accel_ratio −/+ (**조이스틱 주행에만 적용**) |
| START | 녹화 시작/정지 (mp4 + csv) |
| **A** | **engage 토글 (자율 구동)** — control_node 에서만 동작 |
| **X** | **E-STOP** — **actuator 에서** 모든 명령을 무시한다 (한 층 아래). 되돌리려면 노드 재시작 |

> A 와 X 는 **다른 층이다.** A 는 control_node 에게 "그만 보내라" 고 부탁한다. X 는 actuator
> 에게 "무시해라" 고 명령한다. **control_node 가 고장나면 A 는 아무것도 못 한다.**

---

## 토픽

| 토픽 | 용도 |
|---|---|
| `/camera/image/compressed` | 원본 카메라 (BEST_EFFORT / depth 1) |
| `/lane/state` | 인지 상태 (아래) |
| `/mission/state` | 객체 검출 — 신호등·ArUco·방향표지판 (아래) |
| `/lane/debug/compressed` | 디버그 패널 (**구독자가 있을 때만 생성**) |
| `/control` | 제어 명령 (steering / throttle) |
| `/joystick` | 조이스틱 (control_msg · e_stop_en · engage · is_recording) |
| `/battery_status` | 배터리 |

**`/lane/state` (dracer_msgs/LaneState)**

| 필드 | 의미 |
|---|---|
| `center_error` / `ema` | 정규화 횡오차 [-1,1], + = 우측 (`valid` 로 게이트) |
| `center_error_cm` | cm 단위 횡오차 (calib 불변) |
| `heading` / `curvature` | ego 중앙선 접선각(deg) / 곡률 — 제어 미사용 |
| `confidence` | 이산값: 0.9(pair) / 0.5(coast) / 0.0(없음) |
| `state` | `OK` / `LOW_CONF` / `OUTLIER` / `HOLD` / `LOST` |
| `used_fallback` | coast(단일 차선) 사용 여부 |
| `n_corridors` | 물리적으로 유효한 코리도어 수. > 1 = 분기 |
| `ego_rule` | 무엇이 골랐나: `tracked` / `nearest` / `coast` / `fork_L` / `fork_R` / `branch_*` / `none` |
| `fork_type` / `n_islands` | 갈림길 감지: `''`/`island`/`branch` · island 코리도어 수 |
| `sign_hint` / `sign_hint_age` | 래치된 표지판 방향 `''`/`L`/`R` · 확정 후 프레임 수 |

**`/mission/state` (dracer_msgs/MissionState)** — `perception_node` 가 `/lane/state` 와 같은
프레임·같은 stamp 로 낸다.

| 필드 | 의미 |
|---|---|
| `cls` | **확정** 클래스 (M-of-N 디바운스). `-1` 없음 · `0 GREEN` · `1 RED` · `2 MARK`(ArUco id 3) · `3 RIGHT` · `4 LEFT` |
| `newly_confirmed` | 이번 프레임에 `cls` 가 바뀌었다 (엣지) |
| `det_cls` / `det_conf` | 이번 프레임의 원시 최고 검출 (확정과 다를 수 있다) |
| `det_x/y/w/h` | 그 bbox — **카메라 픽셀** (신호등은 지면 위라 BEV 에 투영되지 않는다) |

> `cls` 는 **끈적하고**(sticky) `det_*` 는 **순간적이다.** STOP 클래스(RED·MARK)는 GO 보다
> 낮은 문턱으로 확정된다 — 헛정지는 몇 초를 잃고, 놓친 정지는 장애물을 받는다.

---

## 산출물 이동 (D3-G ↔ 로컬)

- 녹화(mp4/csv)는 git 미추적 → **scp** 로 로컬 `offline/rslt/` 로 가져온다.
- profile YAML 은 git 추적 → **git** 으로 D3-G 에 전달. **D3-G 에서 코드를 고치거나 커밋하지
  마라.**
- `git reset`/`clean` 은 보드 고유 캘리브(`vehicle_config.yaml` 의 SERVO_*/ACCEL_RATIO,
  `camera.yaml`)를 커밋값으로 덮어쓰고 녹화 데이터도 지운다 — **백업 먼저.**
