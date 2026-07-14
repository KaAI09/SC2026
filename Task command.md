# Track Test — 운영 명령 가이드

실차 트랙 테스트의 전 과정. 실행 위치를 구분한다.
- 🖥 **로컬(offline)** — macOS, ROS 불필요, 레포 `.venv`
- 🚗 **D3-G(online)** — 차량, ROS2 Humble, `colcon`

**안전(액추에이션 공통)**: 바퀴 지면에서 띄우고(wheels-off) 먼저 → 조이스틱 **X = E-STOP**
상시 → 저속 → D3-G 에서 코드 수정·commit 금지.

> 시스템 구조 · 인지 파이프라인 · 남은 작업은 [README.md](README.md). 이 문서는 상시
> 운영 레퍼런스다.

---

## 파이프라인 개요

```
🚗 calibrate ─► 🚗 collect ─► 🖥 panel_replay (오프라인 재생 · 파라미터 A/B)
                                     │
                                     └──► profile 갱신 ──► 🚗 drive (자율, 감시·튜닝)
                                                                  │
                                                                  └──► 🚗 lap (랩타임)
```

| 런치 | 주행 | 인지 | 제어 | 모니터 | 녹화 | 용도 |
|---|---|---|---|---|---|---|
| `calibrate` | 수동 | ✗ | ✗ | 원본 | ✗ | 카메라 각도 + 서보 중립/ACCEL_RATIO **저장** |
| `collect` | 수동 | ✓ (패널 OFF) | ✗ | **원본** | 원본 + csv | **데이터 수집** — 패널은 오프라인에서 |
| `drive` | 자동 | ✓ | ✓ | **원본**(기본) | 원본 + csv | 자율주행 **튜닝·검증** |
| `lap` | 자동 | ✓ (패널 OFF) | ✓ | **✗** | **✗** | **랩타임 측정** (최경량) |

> ⚠ **`drive` 의 모니터 기본값은 RAW 카메라다** (0714 부터). 디버그 패널을 보려면 명시하라:
> `monitor_topic:=/lane/debug/compressed`. 패널은 CPU 가 아니라 **대역폭**에서 먼저 죽는다 —
> §B-3.

> **core** = camera · actuator · joystick. 어떤 런치에서도 빠지지 않는다 — 조이스틱이
> **E-STOP(X) · engage(A) · 녹화(START)** 경로다. `monitor`(웹 :5000)와 `battery` 는 런치가
> 골라서 붙인다.
>
> **렌더링은 구독이 켠다.** 인지는 `/lane/debug/compressed` 에 구독자가 있을 때만 패널을
> 합성·JPEG 인코딩한다. 그래서 모니터를 **raw 카메라**에 붙이면 그 비용이 0 이고, 디버그
> 토픽에 붙이면 그 비용을 **자기가 요청한 것**이다. `lap` 은 모니터 자체가 없다.
>
> **녹화는 항상 원본 + csv 뿐이다.** 4패널은 `offline/panel_replay.py` 가 raw+csv 에서
> 정확히 되살린다 — 차는 자기가 보지도 않는 영상을 렌더링하느라 프레임을 흘리지 않는다.

---

## A. 환경 준비

### A-1. 🖥 로컬 venv (최초 1회)
```bash
cd ~/workspace/SC2026
.venv/bin/pip install -e D-Racer-Kit/src/dracer_core
.venv/bin/python -c "from dracer_core.perception_core import Cfg; print(Cfg().name)"
```

### A-2. 🚗 D3-G — 동기화 + 빌드

> **차가 새 코드를 돌고 있는지 확인하는 법**: 녹화 파일명이 `collect_`/`drive_` 이고 csv 의
> `state` 컬럼이 **채워져 있으면** 새 코드다. `raw_<stamp>` 이고 csv 가 **비어 있으면** 구
> `record.launch` 이고, 그것은 **인지를 안 돌린 프레임을 남긴다.**

```bash
export WS=~/SC2026/D-Racer-Kit
cd "$WS"

# ⚠ vehicle_config.yaml 은 D3-G 로컬 캘리브레이션이다. reset --hard 가 덮어쓴다.
grep -E "SERVO_|ACCEL_RATIO" src/config/vehicle_config.yaml       # 먼저 적어둬라

git fetch origin && git checkout main
git reset --hard origin/main
git clean -fdx          # ⚠ build/install/recorder(mp4·csv) 삭제 — 필요분 먼저 scp (↓E)

source /opt/ros/humble/setup.bash
colcon build --symlink-install        # ⚠ 전체 빌드. MissionState.msg 가 새로 생겼다.
source install/setup.bash
```
> ⚠ **`--packages-select` 를 쓰지 마라.** `dracer_msgs` 에 `MissionState.msg` 가 추가됐고,
> 그것을 모르는 채 빌드된 노드는 **조용히 옛 인터페이스로** 돈다.
>
> 새 터미널마다: `cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash`

**기동이 맞는지 한 줄로 확인:**
```bash
ros2 launch dracer_bringup collect.launch.py    # 이 런치가 없다면 = 옛 코드다
```

---

## B. 런치별 명령 (🚗 D3-G)

`profile` · `camera` 를 생략하면 런치가 기본값을 자동으로 찾는다
(`src/config/profiles/track.yaml`, `src/config/camera.yaml`).

### B-1. calibrate — 카메라 세팅 + trim/accel 저장
```bash
ros2 launch dracer_bringup calibrate.launch.py
```
- 웹 `http://<D3-G_IP>:5000` → 카메라 실시간 보며 각도/높이 조절.
- 조이스틱: **Y/B** = 서보 중립 ∓10us, **L1/R1** = accel_ratio −/+ (즉시 `vehicle_config.yaml`
  저장), **X** = E-STOP.
```bash
grep -E "SERVO_|ACCEL_RATIO" "$WS"/src/config/vehicle_config.yaml
```

> ⚠ **카메라 마운트를 움직였다면 `camera.yaml` 의 H(호모그래피)가 무효가 된다.**
> `offline/calibrate.py --ground` 로 지면 사진 한 장에서 다시 뽑아야 한다. K/D(렌즈)는 살아남는다.

> ⚠ **`ACCEL_RATIO` 는 자율주행과 무관하다.** `joystick_node` 가 조이스틱 축에만 곱한다.
> 자율주행 스로틀은 프로파일의 `throttle_base` 다.

### B-2. collect — 데이터 수집 (수동 주행 + 인지 + 원본 녹화)
```bash
ros2 launch dracer_bringup collect.launch.py
ros2 launch dracer_bringup collect.launch.py record_dir:=$HOME/recorder
```
확인(새 터미널):
```bash
ros2 topic echo /lane/state --once
ros2 topic echo /mission/state --once     # cls · det_cls · det_conf · bbox
ros2 topic hz /lane/state          # 30Hz — 렌더링을 안 하므로 주행 레이트와 같다
# 웹 :5000 → raw 카메라 (사람은 "차가 어디 있나" 만 보면 된다)
```
- 조이스틱 **START** 로 녹화 시작/정지. 한 세션이 같은 basename 으로 2개 파일:
  `recorder/raw/collect_<stamp>.mp4` (원본, **무각인**) +
  `recorder/csv/collect_<stamp>.csv`.
- **패널은 여기서 안 만든다.** `offline/panel_replay.py` 가 raw+csv 로 되살린다 (§C-1).

**csv 컬럼** — 차선과 객체가 **한 프레임에서 나온다**:
```
frame_time,
valid, center_error, ema, heading_valid, heading, confidence,
left_conf, right_conf, state, used_fallback,
n_corridors, ego_rule,                      ← 분기 증거
mission_cls,                                ← 확정 클래스 (-1=없음)
mission_det_cls, mission_det_conf,          ← 이번 프레임 원시 최고 검출
mission_det_x, mission_det_y, mission_det_w, mission_det_h,   ← 카메라 픽셀 bbox
ctrl_steering, ctrl_throttle, manual_steering, manual_throttle, e_stop
```
> 미션은 `mission_frame_skip`(기본 2) 마다 돈다 = 매 3프레임. 그래서 `mission_*` 이 세 행
> 연속 같은 값이면 **세 영상 프레임이 한 검출을 공유한 것**이고, 그게 실제로 일어난 일이다.
> 리샘플하지 않는다.

**대회장 미션 튜닝** — 조명이 바뀌면 검출 임계를 다시 잡는다:
```bash
ros2 launch dracer_bringup collect.launch.py mission_config:=<venue.yaml>
```
> `MissionGate` 는 아직 `control_node` 에 배선되어 있지 않다 — 미션은 **검출·발행만** 하고
> 차를 멈추지 않는다. `/mission/state` 와 csv 의 `mission_*` 컬럼으로 **무엇을 봤는지** 먼저
> 확인하고 배선하라.

CPU 를 아끼고 싶으면 미션 자체를 끈다:
```bash
ros2 launch dracer_bringup collect.launch.py use_mission:=false
```

> ✅ **구 `record` + `perceive` 를 대체한다.** 둘은 원래 한 작업이 쪼개져 있던 것이고,
> 패널이 오프라인으로 간 순간 쪼갤 이유가 사라졌다 — `record` 는 인지를 안 돌린 프레임을
> 남겼고, `perceive` 는 노트북이 나중에 공짜로 그려줄 그림을 **핫패스에서** 그리느라
> ~15Hz 로 떨어졌다. `collect` 는 렌더링을 하지 않으므로 **30Hz, 즉 주행과 같은 레이트**로
> 돈다. 여기서 튜닝한 레이트가 곧 주행 레이트다.

**live 튜닝 — 진짜 노브는 cm 단위다:**
```bash
ros2 param set /perception_node merge_dx_cm     6.0     # 같은 테이프로 볼 거리
ros2 param set /perception_node jump_max_cm    20.0     # 프레임 간 최대 차선 점프
ros2 param set /perception_node sw_margin_cm    6.0     # 슬라이딩 윈도우 반폭
ros2 param set /perception_node yellow_v_min   90       # 색 문턱값
ros2 param set /perception_node lane_width_cm  34.8     # 트랙 차선폭 (중심-중심, 실측)
```

> ⚠ **`merge_dx` · `jump_max` · `sw_margin` · `morph_v` · `sw_minpix` · `sw_peak_min` ·
> `sw_peak_sep` · `pair_gap_min` · `gate_min_px` · `lane_width_default` · `heading_frac`
> 는 파라미터가 아니다.** `cfg_to_px` 가 위의 cm 값에서 계산한다(`DERIVED_PX`). ROS 가
> `param set` 을 **거부한다** — 예전에는 "성공" 이라고 답하고 아무것도 하지 않았다.

> ⚠ **ROI 사다리꼴 파라미터(`roi_top_frac`·`trap_*`)는 없다.** BEV LUT 자체가 캘리브레이션된
> 사다리꼴 크롭이라 손튜닝 ROI 는 크롭 위의 크롭이 된다.

> ✅ **`ros2 param set` 은 이제 상태를 보존한다** (B3). 로그에 `(Tracker/EMA 상태 유지)` 가
> 뜬다. 예전엔 파이프라인을 재생성해서, 파라미터를 **하나도 안 바꾸고** 재설정만 해도
> `|Δema|` 가 0.0377 튀었다 — 튜닝하려던 값의 효과(0.0014)보다 리셋 노이즈가 27배 컸다.
> (BEV 기하가 바뀌는 경우만 여전히 재생성한다 — 거기선 추적 중인 픽셀이 다른 장소를 뜻한다.)

### B-3. drive — 자율주행 (⚠ 액추에이션)
```bash
ros2 launch dracer_bringup drive.launch.py         # engage=false 로 시작
```

**기동 로그가 이 값이어야 한다:**
```
perception_node : [lane] 30.0Hz state=OK ... corridors=1[tracked]
control_node    : state_timeout=0.25s joystick_timeout=0.3s throttle_outlier=0.0 steer_max=1.0
actuator_node   : servo: center=1650us span=300us range=1250~2050us  command_hz=30.0
```
**Hz 가 30 이 아니면 멈춰라.** 게인은 30Hz 에서 튜닝됐다.
**`kp` 1.0 / `kd` 0.13 이 프로파일에서 온다** (옛 0.75 는 곡선에서 부족했다 — §B-7).
**그 게인으로 달린 주행은 아직 녹화된 적이 없다 — START 로 한 세션 남겨라.**

안전 절차(새 터미널):
```bash
ros2 topic echo /control                            # engage 전엔 중립
# ↓ 바퀴 띄운 상태 확인 후에만
ros2 param set /control_node engage true            # 또는 조이스틱 A
ros2 param set /control_node engage false           # 정지 (또는 조이스틱 X = E-STOP)
```

**제어 live 튜닝:**
```bash
ros2 param set /control_node kp                0.75   # A1 이후 스케일. 진동하면 낮춘다
ros2 param set /control_node slew_rate_per_sec 7.5    # 초당! (프레임당 아니다)
ros2 param set /control_node throttle_base     0.23
ros2 param set /control_node throttle_outlier  0.0    # OUTLIER 시 스로틀 (1.0 = 컷 안 함)
ros2 param set /control_node steer_sign       -1.0    # 조향 방향이 반대일 때
ros2 param set /control_node controller        PID    # PD | PID
ros2 param set /control_node ki                0.05   # PID 만
ros2 param set /control_node i_clamp           0.5    # PID anti-windup 한계
```

> ⚠ **`controller` 는 `PD` / `PID` 뿐이다.** 모르는 이름을 넣으면 **예외를 던진다** (예전에는
> 조용히 P 로 폴백해서, 오타 하나가 적어둔 것과 다른 제어기로 차를 달리게 했다).
> Pure Pursuit 는 아직 없다 — 구현하려면 인지↔제어 계약 변경이 필요하다
> ([README.md](README.md) 남은 작업 B1).

**런치 인자 (실행 중 못 바꾼다 — 타이머를 생성 시점에 만든다):**
```bash
ros2 launch dracer_bringup drive.launch.py command_hz:=30.0 publish_rate:=30.0
```

**모니터 패널 (웹 :5000) — ⚠ 기본은 RAW 카메라다. 패널은 명시적으로 켠다:**

```bash
ros2 launch dracer_bringup drive.launch.py monitor_topic:=/lane/debug/compressed
```

> **패널은 CPU 가 아니라 대역폭에서 먼저 죽는다.** 렌더는 프레임을 먹고, 스트림은
> **지연**을 먹는데, MJPEG 스트림은 링크가 못 나르는 프레임을 **버리지 않고 큐에 쌓는다** —
> 그래서 지연이 평평해지지 않고 **계속 자란다.** 혼잡한 대회장 Wi-Fi 에서는 `ros2 topic hz
> /lane/state` 가 30 을 찍는데 화면만 몇 초 뒤처진다. **토픽을 보라, 그림을 보지 말고.**
> (0714: `debug_scale` 기본값이 2.0 이라 552x240 패널을 **1104x480 으로 확대해** 인코딩하고
> 있었다 = 원본 카메라의 **픽셀 7배**를 30Hz 로 Wi-Fi 에 밀어넣었다. 지금은 1.0 이다.)

```bash
ros2 param set /perception_node debug_scale 1.0    # 라이브로 줄인다 (0714 부터 실제로 먹는다)
ros2 param set /perception_node debug_view off     # 렌더 자체를 끈다
```

```
┌──────────────┬──────────────────┐
│  BEV metric  │  camera 320x240  │   552x240 (4패널 1280x240 의 43%)
│              │                  │
│  ─ 흰 차선   │   ┌────┐         │   BEV  : 7-라벨 색상 차선
│  ─ 노랑 차선 │   │RED │ ← bbox  │          + 모든 코리도어 중심선
│   ═ ego(cyan)│   └────┘         │            · 흰-흰 = 회색
│   ┄ fork     │      ┌──┐        │            · 노랑-노랑 = 주황
│   │ 차량축   │      │3 │ ArUco  │            · 벌어지는 것(fork) = 점선
│              │      └──┘        │            · ego(제어값) = cyan 굵게
└──────────────┴──────────────────┘   camera: 객체 bbox (클래스별 색)
  off=+2.1cm OK   n=2[tracked]  RED
```

**바운딩 박스는 BEV 에 못 그린다.** 신호등은 지면 위에 있어서 BEV 셀에 아예 투영되지 않는다.
차선 질문은 BEV 에서, 객체 질문은 카메라 뷰에서 묻는 것이고, 한 캔버스가 둘 다 답하는 척하면
둘 중 하나를 숨기게 된다. 그래서 나란히 붙인다 — 토픽 1개, JPEG 1장.

```bash
ros2 launch dracer_bringup drive.launch.py monitor_topic:=/lane/debug/compressed debug_view:=panels
#   옛 4패널 (슬라이딩 윈도우 확인용). 1280x240 = BEV 뷰의 2.3배 픽셀.
ros2 launch dracer_bringup drive.launch.py    # 기본 = RAW 카메라 = 렌더 비용 0
```
- 조이스틱 **START** 로 녹화. `recorder/raw/drive_<stamp>.mp4` + `recorder/csv/drive_<stamp>.csv`
  (컬럼은 §B-2 와 동일 — 자율 command 가 채워진다는 것만 다르다).
  **패널 mp4 는 안 만든다** — `offline/panel_replay.py` 가 raw+csv 로 되살린다 (§C-1).

### B-4. lap — 랩타임 측정 (⚠ 액추에이션, 최경량)
```bash
ros2 launch dracer_bringup lap.launch.py           # engage=false 로 시작
ros2 param set /control_node engage true           # 바퀴 띄운 확인 후에만 (또는 조이스틱 A)
```
`drive` 에서 **모니터와 recorder 를 뺀 것**, 그게 전부이고 그게 요점이다. 웹 대시보드로
JPEG 를 스트리밍하고 mp4 를 인코딩하는 차는 **타임드 랩을 달릴 차가 아니다** — 둘 다 인지가
도는 것과 같은 보드에서 프레임당 CPU 를 먹으므로, 켜둔 채 잰 랩타임은 실제보다 느리다.

**빠지지 않는 것**: `joystick`(E-STOP·engage), `actuator`(서보로 가는 유일한 길),
`battery`(공짜에 가깝고, 느린 랩이 "느린 차" 때문인지 "주저앉은 배터리" 때문인지 구분해준다).

> 두 경로(본선 S자 / 로터리) 랩타임 비교는 이 런치로 **각각 돌려서** 잰다.

### B-5. 워치독 검증 (⚠ 바퀴 지면에서 떼고)
```bash
ros2 param set /control_node engage true
pkill -f perception_node    # → "PERCEPTION STALE ..." + /control 이 (0,0)
pkill -f joystick_node      # → "JOYSTICK STALE ... Forcing engage OFF"
```
> ⚠ **패드를 뽑는 것으로는 joystick 워치독이 발동하지 않는다.** `joystick_node` 가 마지막
> 입력을 50Hz 로 계속 재발행한다 — **알려진 미수정 구멍이다** (`control_node` docstring 의
> CAVEAT). 워치독은 `joystick_node` 가 **죽을** 때만 발동한다.

### B-6. 분기(로터리) 확인
```bash
ros2 topic echo /lane/state --field n_corridors     # 로터리에서 2 이상이 뜨는가
ros2 topic echo /lane/state --field ego_rule        # tracked / nearest / coast / branch_*
```
```bash
# ⚠ 랜덤 경로 선택 실험. 차가 노란 지름길로 갈 수 있다. 저속·감시 하에서만.
ros2 param set /perception_node branch_policy random
ros2 param set /perception_node branch_policy keep      # 원복 (기본값 = 현재 동작)
```

---

### B-7. 🚗 제어 — `kp` 1.0 적용됨. **다음 주행을 녹화해서 닫아라**

**`kp` 1.0 / `kd` 0.13 은 이미 [`track.yaml`](D-Racer-Kit/src/config/profiles/track.yaml)
에 들어가 있고 실차로 달려봤다.** 다만 **그 주행이 녹화되지 않아 수치가 없다.**
개선은 눈으로 봤지만 **측정된 적이 없다** — **START 로 한 세션만 녹화하면 닫힌다.**

**판정 기준** (옛 게인 `kp` 0.75 로 달린 [`offline/rslt/07142315`](offline/rslt/07142315),
engage 1,903프레임 대비):

| 확인 | 기대 | 옛 값 |
|---|---|---|
| 곡선 `\|ema\|` p95 | **< 17.5cm** (= 코리도어 안) | 23cm |
| 조향 포화 | `\|steer\|` 가 1.0 에 닿지 않는다 | 포화 0% (p90 0.65) |
| 진동 | 조향 부호반전이 늘지 않는다 | 1.0~1.2회/초 |
| 직선 | 그대로 완벽하다 | 횡오차 0.2cm |

**왜 올렸나** — 옛 게인에서 직선은 완벽했는데(횡오차 0.2cm, 정상상태 조향 −0.004) 곡선에서
벗어났다(p95 **23cm**, 코리도어 반폭 17.5cm). **서보가 아니라 PD 의 구조다**: 일정 곡률의
코너는 **일정한 외란**이고, 적분항 없는 P 제어는 `e_ss = u_필요/kp` 를 **반드시** 남긴다.
그리고 **조향에 여유가 있었다** (`|steer|` p90 0.65, 포화 0%).

| `kp` | 곡선 `\|ema\|` 예상 | 그때 `\|steer\|` p90 |
|---|---|---|
| 0.75 (옛) | 12.8cm | 0.65 |
| 0.90 | 10.7cm | 0.78 |
| **1.00** | **9.6cm** | **0.86** |
| 1.20 | 8.0cm | 1.04 ← 포화 시작 |

```bash
ros2 launch dracer_bringup drive.launch.py         # engage=false 로 시작. kp 1.0 은 프로파일에서 온다
# 바퀴 띄운 상태로 engage(A) 확인 -> 내리고 저속 -> START 로 녹화

# 여기서 더 움직여 볼 때만:
ros2 param set /control_node kp 1.2                # 곡선에서 여전히 벗어나면 (포화 주의)
ros2 param set /control_node kp 0.9                # 진동하면 낮춘다
ros2 param set /control_node throttle_base 0.23    # ⚠ 기준선 주행은 0.25 로 달렸다
```

> ⚠ **속도부터 되돌려라.** 기준선 주행은 `throttle_base` 0.25 로 달렸는데 프로파일 값은
> **0.23** 이다. 코너 이탈에는 속도도 기여한다 — 0.23 으로 내리고 `kp` 를 시험하는 것이 순서다.

> **`kp 1.0` · `kd 0.13` 은 이미 `track.yaml` 에 들어가 있다.** 위 `param set` 은 그 값에서
> **더 움직여 볼 때** 쓴다.

**웹 모니터가 밀리면 — 노브는 `jpeg_quality` 하나다:**
```bash
ros2 launch dracer_bringup drive.launch.py     # 기본이 RAW 카메라 (렌더 비용 0)
```
실측 (320x240 주행 프레임): `q90` = 16.4 KB/frame = **3.8 Mbps** @30Hz · `q70` = 9.3 KB =
**2.2 Mbps** (기본값이 70 이다). MJPEG 스트림은 링크가 못 나르는 프레임을 **버리지 않고 큐에
쌓으므로** 지연이 평평해지지 않고 **자란다.**
> ⚠ `vehicle_config` 의 `IMAGE_DISPLAY_*` 는 **전송량과 무관하다** — 카메라 미연결 시
> placeholder SVG 크기일 뿐이다.

**확인할 것:**
```bash
ros2 topic echo /control --field steering          # 최대 |steering| 이 1.0 에 닿는가 (포화 = kp 과다)
```
| 증상 | 뜻 | 조치 |
|---|---|---|
| 좌우로 진동 | `kp` 과다 | `kp` 낮춘다 (기준선의 조향 부호반전은 1.0~1.2회/초였다) |
| 곡선에서 여전히 벗어난다 | `kp` 부족 | 올린다. 단 `|steer|` 가 1.0 에 닿기 시작하면 거기가 한계다 |
| **한쪽 코너만** 언더스티어 | 서보 중립이 틀어졌다 | Y/B 로 재조정 (기준선에서는 **없던 증상**이다) |

**성공 판정**: 곡선 `|ema|` p95 가 **17.5cm 아래**로 (= 코리도어 안에 머문다).
```bash
# START 로 녹화 -> raw+csv 를 로컬로 (§E) -> panel_replay --csv 로 확인
```

> **`kp` 상향은 증상 완화다.** `center_error` 를 전방 31cm **한 점**에서 재는 순수 횡오차
> 레귤레이터인 한 코너 진입 반응은 구조적으로 늦다. 근본은 Pure Pursuit (README §7 B1) 이고,
> 전제(최대 조향각 25도 · 휠베이스 17.5cm)는 **이미 다 실측됐다.**

## C. 오프라인 (🖥 로컬)

### C-1. panel_replay — 주행 raw 를 4패널로 되살리기 (⭐ 주력 도구)
`drive.launch` 는 패널을 녹화하지 않는다 (렌더링이 검출의 4배를 먹는다). 대신 raw + csv 를
남기고 **여기서 사후에** 같은 `dracer_core` 파이프라인으로 정확히 되살린다.

```bash
cd ~/workspace/SC2026
.venv/bin/python offline/panel_replay.py offline/rslt/recorder/raw/drive_<stamp>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track.yaml \
    --csv offline/rslt/recorder/csv/drive_<stamp>.csv

# 파라미터 A/B (원본은 안 건드린다)
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera ... --profile ... \
    --set lane_width_cm=35 --set branch_policy=random --no-video
```
`--csv` 를 주면 실차가 그때 발행한 LaneState 와 프레임 단위로 대조한다 (= 오프라인 튜닝을
믿어도 되는지 검증).

### C-2. calibrate — 카메라 마운트를 움직였다면 (⭐ 지면 사진 **5장**)

**마운트를 움직이면 `H`(자세)만 무효가 된다. `K·D`(렌즈)는 살아남는다** — 그러니 체커보드
20장은 다시 안 찍는다. `--from-camera` 가 기존 `camera.yaml` 에서 `K·D`(+ `px_per_cm`·축
오프셋·런타임 해상도)를 물려받고 `H` 만 다시 푼다.

> ⚠ **`--intrinsics` 는 렌즈/카메라 자체를 바꿨을 때만.** 그냥 마운트를 조정한 것이라면
> `--from-camera` 다.
>
> ⚠⚠ **`--intrinsics` 를 쓸 때는 `--runtime-size 320x240` 을 반드시 같이 줘라.**
> `--from-camera` 는 런타임 해상도를 기존 `camera.yaml` 에서 물려받지만, `--intrinsics` 는
> 물려받을 파일이 없다. 안 주면 기본값이 들어가고 — 0714 에서 실제로 `320x160` 이 들어가
> **960x720 → 320x160 이라는 비등방 리스케일**(`fx ×0.333` / `fy ×0.222`)이 저장됐다.
> `CameraModel.match()` 가 런타임에 되돌려주긴 하지만, **저장된 파일이 거짓말을 하고 있는
> 것**이고 320x160 프레임이 실제로 오면 조용히 틀린다. 로그의 `런타임:` 줄에서
> **`fx·cx ×0.3333, fy·cy ×0.3333`** 처럼 두 배율이 **같은지** 확인하라.

> ⭐ **한 장으로는 부족하다 — 0714 에서 배웠다.** 5장을 찍어 **각각** 풀고 서로 대조하라.
> 그중 2장은 **혼자서는 완벽해 보였다** (카메라 높이 23.0cm 정합, RMS 0.087cm "양호") 지만
> **차선폭을 35 → 20cm 로 깨뜨렸다** — 보드가 멀리 있어(near 48~52cm) 근거리 `H` 가
> 무너진 것이다. 나머지 3장이 **서로 독립적으로** 같은 답에 모이는 것, 그것이 유일한 보증이다.
>
> **고르는 기준은 차선폭의 평균이 아니라 변동이다.** "폭이 35cm 다" 는 2025 트랙 실측이고
> 지금 트랙의 폭은 잰 적이 없다. 그 가정에 기대는 지표(평균)와 달리 **"폭이 거리와 무관하게
> 일정하다"** 는 트랙 폭을 몰라도 성립한다. (0714 채택본: 다섯 지점 전부 34.8cm, 변동 0.03cm.)

> ⚠ **보드가 지면에서 떠 있으면 `--board-offset-cm` 으로 알려줘라.** `ground_pose` 는
> "보드 = 지면" 을 전제로 `solvePnP` 를 푼다. 안 알려주면 `H` 가 지면이 아니라 **그 높이의
> 유령 평면**으로의 사영이 되고, **숫자는 전부 멀쩡해 보인다** (RMS 는 오히려 좋다 — 보드
> 평면에 대한 fit 은 훌륭하니까). 교차검증: 산출된 **카메라 높이가 실측과 맞는지** 보라
> (`--cam-height-cm <실측>` 이 대조해 준다).

**① 🚗 D3-G — 지면 사진 5장 촬영** (보드 위치·거리를 조금씩 바꿔가며)
```bash
cd ~/SC2026/D-Racer-Kit
cp src/config/vehicle_config.yaml /tmp/vehicle_config.bak     # ★ 백업

# 촬영 해상도를 올린다 (지면 보드는 원근에 눌려 코너가 안 잡힌다)
sed -i 's/^IMAGE_WIDTH:.*/IMAGE_WIDTH: 960/; s/^IMAGE_HEIGHT:.*/IMAGE_HEIGHT: 720/' \
    src/config/vehicle_config.yaml

ros2 launch dracer_bringup calibrate.launch.py image_topic:=/calib/preview/compressed
python3 scripts/capture_camera_calib.py --out ~/calib --name ground --count 5
#   보드를 노면에 평평하게 눕히고, 화면에 다 들어오게. 웹 :5000 에서 코너 확인.
#   ⭐ 5장의 보드를 **가깝게** 두라 (near <= 40cm). 멀면 근거리 H 가 무너진다 (위 ⭐ 참조).
#   ⭐ 보드가 지면에서 뜬 높이를 **재서 적어둬라** — ②의 --board-offset-cm 에 넣는다.
#   min gap >= 14px 면 좋다. 10px 미만이면 해상도를 더 올릴 것.

cp /tmp/vehicle_config.bak src/config/vehicle_config.yaml     # ★★ 원복 (필수)
grep -E "IMAGE_(WIDTH|HEIGHT)" src/config/vehicle_config.yaml #    320 / 240 확인
```

**①-b 🚗 D3-G — 검증용 직선 구간 프레임** (해상도 원복 **후**, 런타임 320x240 으로)

`--check` 는 **새 마운트로 찍은** 직선 구간 프레임이 필요하다. 기존 `rslt/0712` 는 **옛
마운트** 영상이라 새 캘리브 검증에 쓸 수 없다.

```bash
ros2 launch dracer_bringup collect.launch.py   # 직선 구간에 세워두고 START → 2~3초 → STOP
```

> **트랙이 없으면 인쇄 타깃을 쓴다.** `offline/calib/lane_target.pdf` (A4 4장 = 40×57.4cm,
> 검정 노면 + 실제와 같은 35cm 차선). **배율 100%** 로 인쇄 → 재단선까지 잘라 맞대어 붙이고
> (**뒷면에서** 테이프) → 평평하게 깔고 → 그 위에 차를 정상 주행 상태로 세워 촬영.
> 상세·주의는 [offline/README.md](offline/README.md) 와 PDF 마지막 쪽.

**② 🖥 로컬 — 5장을 각각 풀고, 서로 대조해서 고른다**
```bash
cd ~/workspace/SC2026
scp topst@<D3-G_IP>:'~/calib/ground_*.png' offline/calib/<세션>/
cp D-Racer-Kit/src/config/camera.yaml /tmp/camera.yaml.bak    # ★ 백업 (아래 ⚠ 참조)

# 먼저 5장을 전부 풀어서 카메라 높이가 서로/실측과 맞는지 본다 (--out 없이 = 저장 안 함)
cd offline
for i in 0 1 2 3 4; do
  echo "--- ground0$i"
  ../.venv/bin/python calibrate.py --from-camera ../D-Racer-Kit/src/config/camera.yaml \
      --ground calib/<세션>/ground0$i.png --square-mm 25.0 \
      --board-offset-cm <보드가 뜬 높이> --cam-height-cm <카메라 높이 실측> --lane-width-cm 35
done
```
| 확인 | 기대 |
|---|---|
| 카메라 높이 | 5장이 **서로** 맞고, **실측과도** 맞는다 (0714: 23.0~23.9 vs 실측 23.2) |
| 재투영 RMS | < 1cm |
| `near` | **작을수록 좋다.** 48cm 넘게 멀면 근거리 `H` 를 의심하라 |
| 보드 횡위치 | 보드를 차량축에 뒀다면 ≈ 0. **차가 차선 중앙이 아니었다면 0 이 아닌 게 정상이다** |

**그 다음, 진짜 판정 기준으로 고른다 — 각 `H` 가 차선폭을 거리와 무관하게 일정히 재는가.**
5장으로 각각 `camera.yaml` 을 만들어 `--check`(③) 를 돌리고, **변동(간격 변동)이 가장 작은
것**을 채택한다. 채택본만 `--out` 으로 덮어쓴다:
```bash
../.venv/bin/python calibrate.py --from-camera ../D-Racer-Kit/src/config/camera.yaml \
    --ground calib/<세션>/ground0<채택>.png --square-mm 25.0 \
    --board-offset-cm <보드 높이> --cam-height-cm <실측> \
    --x-half 29 --y-far 78 --lane-width-cm 35 \
    --out ../D-Racer-Kit/src/config/camera.yaml
```
> `--x-half 29 --y-far 78` 은 **인지 비용 예산**이다. 안 주면 가시범위 전체(0714 기준
> x±50cm, y 34~184cm)를 BEV 로 펴서 픽셀이 6배가 된다. 자동 범위는 "카메라가 볼 수 있는
> 곳" 이지 "차선이 있는 곳" 이 아니다.

**③ 🖥 로컬 — 검증 (건너뛰지 말 것)**
```bash
cd ~/workspace/SC2026
scp topst@<D3-G_IP>:'~/recorder/raw/*.mp4' /tmp/           # recorder 기본 경로 = $HOME/recorder

# 그 영상에서 프레임 1장 뽑는다 (런타임 320x240 그대로)
.venv/bin/python -c "
import cv2; cap = cv2.VideoCapture('/tmp/<직선구간>.mp4')
cap.set(cv2.CAP_PROP_POS_FRAMES, 30); ok, f = cap.read()
assert ok; cv2.imwrite('offline/calib/straight.png', f); print(f.shape)"

# BEV 가 실제로 metric 한지 — 차선폭 오차 / 평행성 / 수직성
cd offline
../.venv/bin/python calibrate.py --check ../D-Racer-Kit/src/config/camera.yaml \
    --straight calib/straight.png --lane-width-cm 35
```
`→ OK — 캘리브레이션 유효` 여야 한다. 판정 기준은 `CameraModel.validate(tol=0.20)` — 차선폭
오차·평행성·수직성이 **각각 차선폭의 20%(35cm 기준 7cm) 이내**. 다만 그건 **합격선**이지
목표가 아니다. 잘 된 캘리브는 폭 오차 1~2cm, 평행성 1cm 미만이다. 결과 시각화는
`offline/rslt/calib_check.png` (좌: 원본, 우: BEV 마스크 + 차량축 빨강).

> ⚠ **반드시 진짜 직선 구간이어야 한다.** 곡선 프레임을 넣으면 평행성·수직성이 그냥
> 커브 때문에 커지고, 캘리브가 멀쩡한데 FAIL 이 뜬다 (반대로 폭 오차는 우연히 작을 수도
> 있다). 두 차선이 다 보이는 곧은 구간에 세워두고 찍어라.

> ⚠ **옛 주행 raw(`rslt/0712`)로 검증하지 마라.** 그건 옛 마운트 영상이라 `center_error` 가
> 달라지는 게 **정상**이고, 실차 csv 대조는 의미가 없다. 새 캘리브는 **새 프레임으로만**
> 검증된다.

**④ 🖥 로컬 — 인지 재생으로 최종 확인** (`--check` 는 프레임 1장, 이건 클립 전체다)
```bash
.venv/bin/python offline/panel_replay.py <직선구간>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track.yaml --no-video
#   → 직선 구간이면 valid 100% / 쌍검출 100% / OK 100% 가 나와야 한다 (0714 실측)
```

**⑤ 🚗 D3-G — 실차 저속 확인.** B-2(`collect`)로 차선 검출을 확인하고, 그 다음 B-3(`drive`).

> ⚠ **`camera.yaml` 을 덮어쓰기 전에 백업하라.** 되돌릴 수 있어야 한다
> (`git checkout -- D-Racer-Kit/src/config/camera.yaml`).

> ⚠ **`panel_replay` 실차 대조(`--csv`)는 캘리브 오류에 둔감하다.** `center_error` 가 BEV
> 폭으로 정규화된 값이라 **BEV 스케일이 통째로 틀어져도 상대 위치는 비슷하게 나온다.**
> 0714 에서 **차선폭을 16~21cm 로 재던 틀린 캘리브도 "재현 일치" 를 통과했다.**
> `--check` 를 건너뛰지 마라 — 그것만이 BEV 가 **metric** 한지 묻는다.

> **제어기 튜닝은 실차 폐루프에서만 한다.** 다른 조향을 했으면 다른 프레임을 봤을 텐데 녹화
> 영상으로는 그걸 재현할 수 없다(covariate shift). 위 B-4 의 `ros2 param set /control_node …`
> 를 쓰라.

### C-3. lane_color_probe — 대회장 조명에서 색 임계를 **측정**한다

**⚠ 캘리브(C-2)를 먼저 확정하라.** 이 도구는 BEV 위에서 재고, BEV 가 틀리면 답도 틀린다.
그리고 **틀린 H 는 색 문제처럼 보인다** — 차선폭을 16cm 로 재는 BEV 에서는 색이 완벽해도
쌍 코리도어가 안 만들어진다 (README A2). 색부터 만지면 기하 오류를 색으로 덮게 된다.

```bash
cd ~/workspace/SC2026
.venv/bin/python offline/lane_color_probe.py offline/rslt/<세션>/raw/collect_<stamp>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track.yaml --stride 5
```
사진 1장으로도 돈다 (조명만 빠르게 볼 때). 하지만 **확정은 트랙 한 바퀴로** — 사진 한 장은
그 순간의 조명이지 트랙의 조명이 아니다.

제안값은 **가설이다.** 프로파일에 넣기 전에 같은 클립으로 A/B 하라:
```bash
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera ... --profile ... \
    --set yellow_h_lo=13 --set color_gate=0.0 --no-video
```
D3-G 에서는 라이브로도 시험한다 (B3 덕에 Tracker 상태가 보존된다):
```bash
ros2 param set /perception_node yellow_h_lo 13
ros2 param set /perception_node color_gate  0.0
ros2 param set /perception_node white_v_min 165
```

---

## D. 조이스틱 · 토픽 참조

| 버튼 | 기능 |
|---|---|
| Y / B | 서보 중립(`SERVO_CENTER_US`) ∓10us (calibration_mode). 트림은 명령이 아니라 서보 중립에 있다 |
| L1 / R1 | accel_ratio −/+ (**조이스틱 주행에만 적용**) |
| START | 녹화 시작/정지 (mp4 + csv) |
| **A** | **engage 토글 (자율 구동)** — control_node 에서만 동작 |
| **X** | **E-STOP** — **actuator 에서** 모든 명령을 무시한다 (한 층 아래). 되돌리려면 노드 재시작 |

> A 와 X 는 **다른 층이다.** A 는 control_node 에게 "그만 보내라" 고 부탁한다. X 는 actuator
> 에게 "무시해라" 고 명령한다. **control_node 가 고장나면 A 는 아무것도 못 한다.**

| 토픽 | 용도 |
|---|---|
| `/camera/image/compressed` | 원본 카메라 (BEST_EFFORT / depth 1 = 최신 프레임만) |
| `/lane/state` | 인지 상태 — 아래 참조 |
| `/mission/state` | 객체 검출 (신호등 · ArUco · 방향표지판) — 아래 참조 |
| `/lane/debug/compressed` | 디버그 패널 (**구독자가 있을 때만 생성**) |
| `/control` | 제어 명령 (steering / throttle) |
| `/joystick` | 조이스틱 (control_msg · e_stop_en · engage · is_recording) |
| `/battery_status` | 배터리 |

**`/mission/state` (dracer_msgs/MissionState)** — `perception_node` 가 **같은 프레임·같은
stamp** 로 `/lane/state` 와 함께 낸다.

| 필드 | 의미 |
|---|---|
| `cls` | **확정** 클래스 (M-of-N 디바운스 통과). `-1` = 없음 |
| | `0 GREEN` · `1 RED` · `2 MARK`(ArUco id 3) · `3 RIGHT` · `4 LEFT` |
| `newly_confirmed` | 이번 프레임에 `cls` 가 바뀌었다 (엣지) |
| `det_cls` / `det_conf` | **이번 프레임의 원시 최고 검출** (확정과 다를 수 있다) |
| `det_x/y/w/h` | 그 bbox — **카메라 픽셀**. BEV 가 아니다: 신호등은 지면 위에 있어서 BEV 에 투영되지 않는다 |

> `cls` 는 **끈적하고**(sticky) `det_*` 는 **순간적이다.** 둘 다 발행하는 이유: `cls` 만
> 있는 기록으로는 "아무것도 없었다" 와 "뭔가 떴는데 투표에서 떨어졌다" 를 구분할 수 없다.
> STOP 클래스(RED·MARK)는 GO 보다 **낮은 문턱**으로 확정된다 — 헛정지는 몇 초를 잃고,
> 놓친 정지는 장애물을 받는다.

**`/lane/state` (dracer_msgs/LaneState)**

| 필드 | 의미 |
|---|---|
| `center_error` / `ema` | 정규화 횡오차 [-1,1], + = 우측 (`valid` 로 게이트) |
| `heading` / `curvature` | ego 중앙선 접선각(deg) / 곡률 — **제어에 미사용** |
| `confidence` | **이산값**: 0.9(pair) / 0.5(coast) / 0.0(없음) |
| `state` | `OK` / `LOW_CONF` / `OUTLIER` / `HOLD` / `LOST` |
| `used_fallback` | coast(단일 차선) 사용 여부 |
| **`n_corridors`** | **물리적으로 유효한 코리도어 수. > 1 = 분기** |
| **`ego_rule`** | **무엇이 골랐나: `tracked` / `nearest` / `coast` / `branch_random` / `none`** |

메시지·노드는 `dracer_msgs` · `dracer_core`. 인지 상세는 [README.md](README.md).

---

## E. 산출물 이동 (D3-G ↔ 로컬)

```bash
# D3-G 녹화 → 로컬 (오프라인 분석)   [로컬에서 실행]
scp -r topst@<D3-G_IP>:~/recorder/{panel,raw,csv}  ./offline/rslt/recorder/

# 완성 profile 로컬 → D3-G   [git 경유]
#   로컬: git add ... && git commit && git push
#   D3-G: git fetch && git reset --hard origin/main
```
> profile YAML 은 git 추적 → git 으로. 녹화(mp4/csv)는 미추적 → scp.
> **D3-G 에서 코드를 고치거나 커밋하지 마라.**
