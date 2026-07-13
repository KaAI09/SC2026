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
🚗 calibrate ─► 🚗 record ─► 🖥 panel_replay (오프라인 재생 · 파라미터 A/B)
                                     │
🚗 perceive (인지검증 + live 튜닝) ───┴──► profile 갱신 ──► 🚗 drive (자율, engage)
```

| 런치 | 구성 | 용도 |
|---|---|---|
| `calibrate` | base | 카메라 각도 + STEER_TRIM/ACCEL_RATIO 저장 |
| `record` | base + recorder(raw) | 오프라인용 원본 영상 수집 |
| `perceive` | base + perception + recorder | 인지 검증 + 데이터 수집 + **live 튜닝** |
| `drive` | base + perception + control + recorder | 자율주행(engage) |

> **base** = camera · actuator · joystick · **monitor(웹 :5000)** · **battery**. 전 런치 공통.
> `drive` 는 렌더링을 핫패스에서 빼기 위해 monitor·recorder 를 **raw 카메라**에 붙인다
> (`/lane/debug/compressed` 구독자가 0 이면 인지가 패널을 아예 만들지 않는다).

---

## A. 환경 준비

### A-1. 🖥 로컬 venv (최초 1회)
```bash
cd ~/workspace/SC2026
.venv/bin/pip install -e D-Racer-Kit/src/dracer_core
.venv/bin/python -c "from dracer_core.perception_core import Cfg; print(Cfg().name)"
```

### A-2. 🚗 D3-G — 동기화 + 빌드
```bash
export WS=~/SC2026/D-Racer-Kit
cd "$WS"

# ⚠ vehicle_config.yaml 은 D3-G 로컬 캘리브레이션이다. reset --hard 가 덮어쓴다.
grep -E "STEER_TRIM|ACCEL_RATIO" src/config/vehicle_config.yaml   # 먼저 적어둬라

git fetch origin && git checkout main
git reset --hard origin/main
git clean -fdx          # ⚠ build/install/recorder(mp4·csv) 삭제 — 필요분 먼저 scp (↓E)

source /opt/ros/humble/setup.bash
colcon build --packages-select dracer_msgs --symlink-install   # 메시지 먼저
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```
> 새 터미널마다: `cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash`

---

## B. 런치별 명령 (🚗 D3-G)

`profile` · `camera` 를 생략하면 런치가 기본값을 자동으로 찾는다
(`src/config/profiles/track2025.yaml`, `src/config/camera.yaml`).

### B-1. calibrate — 카메라 세팅 + trim/accel 저장
```bash
ros2 launch dracer_bringup calibrate.launch.py
```
- 웹 `http://<D3-G_IP>:5000` → 카메라 실시간 보며 각도/높이 조절.
- 조이스틱: **Y/B** = steering_trim −/+, **L1/R1** = accel_ratio −/+ (즉시 `vehicle_config.yaml`
  저장), **X** = E-STOP.
```bash
grep -E "STEER_TRIM|ACCEL_RATIO" "$WS"/src/config/vehicle_config.yaml
```

> ⚠ **카메라 마운트를 움직였다면 `camera.yaml` 의 H(호모그래피)가 무효가 된다.**
> `offline/calibrate.py --ground` 로 지면 사진 한 장에서 다시 뽑아야 한다. K/D(렌즈)는 살아남는다.

> ⚠ **`ACCEL_RATIO` 는 자율주행과 무관하다.** `joystick_node` 가 조이스틱 축에만 곱한다.
> 자율주행 스로틀은 프로파일의 `throttle_base` 다.

### B-2. record — 원본 영상 수집
```bash
ros2 launch dracer_bringup record.launch.py record_dir:=$HOME/recorder
```
- 조이스틱 **START** 로 녹화 시작/정지. 인지가 없는 런치라 raw 가 주 스트림
  → `recorder/raw/raw_<stamp>.mp4` + `recorder/csv/raw_<stamp>.csv` (panel 없음).

### B-3. perceive — 인지 검증 + 데이터 수집 + live 튜닝
```bash
ros2 launch dracer_bringup perceive.launch.py
```
확인(새 터미널):
```bash
ros2 topic echo /lane/state --once
# 웹 :5000 → /lane/debug/compressed 4패널 저지연 스트림
```

**live 튜닝 — 진짜 노브는 cm 단위다:**
```bash
ros2 param set /perception_node merge_dx_cm     6.0     # 같은 테이프로 볼 거리
ros2 param set /perception_node jump_max_cm    20.0     # 프레임 간 최대 차선 점프
ros2 param set /perception_node sw_margin_cm    6.0     # 슬라이딩 윈도우 반폭
ros2 param set /perception_node yellow_v_min   90       # 색 문턱값
ros2 param set /perception_node lane_width_cm  35.0     # 트랙 차선폭 (중심-중심)
```

> ⚠ **`merge_dx` · `jump_max` · `sw_margin` · `morph_v` · `sw_minpix` · `sw_peak_min` ·
> `sw_peak_sep` · `pair_gap_min` · `gate_min_px` · `lane_width_default` · `heading_frac`
> 는 파라미터가 아니다.** `cfg_to_px` 가 위의 cm 값에서 계산한다(`DERIVED_PX`). ROS 가
> `param set` 을 **거부한다** — 예전에는 "성공" 이라고 답하고 아무것도 하지 않았다.

> ⚠ **ROI 사다리꼴 파라미터(`roi_top_frac`·`trap_*`)는 없다.** BEV LUT 자체가 캘리브레이션된
> 사다리꼴 크롭이라 손튜닝 ROI 는 크롭 위의 크롭이 된다.

> ⚠ **`ros2 param set /perception_node <field>` 는 파이프라인을 재생성한다** =
> Tracker/EMA 상태가 리셋된다. **주행 중이 아니라 정지 상태에서** 바꿔라.

- **START** 녹화 → 한 세션이 같은 basename 으로 3개 파일:
  `recorder/panel/drive_<stamp>.mp4` (4패널 디버그, 우상단 `f<idx> t<sec>` 각인) +
  `recorder/raw/drive_<stamp>.mp4` (원본, **무각인** — 오프라인 재실행·캘리브레이션용) +
  `recorder/csv/drive_<stamp>.csv` (LaneState + 자율/수동 command).
  패널 각인의 `f<idx>` 는 csv 데이터 행 번호와 1:1.

### B-4. drive — 자율주행 (⚠ 액추에이션)
```bash
ros2 launch dracer_bringup drive.launch.py         # engage=false 로 시작
```

**기동 로그가 이 값이어야 한다:**
```
perception_node : [lane] 30.0Hz state=OK ... corridors=1[tracked]
control_node    : state_timeout=0.25s joystick_timeout=0.3s throttle_outlier=0.0 steer_max=0.7
actuator_node   : command_hz=30.0
```
**Hz 가 30 이 아니면 멈춰라.** 게인(`kp 0.45`)은 30Hz 에서 튜닝됐다.

안전 절차(새 터미널):
```bash
ros2 topic echo /control                            # engage 전엔 중립
# ↓ 바퀴 띄운 상태 확인 후에만
ros2 param set /control_node engage true            # 또는 조이스틱 A
ros2 param set /control_node engage false           # 정지 (또는 조이스틱 X = E-STOP)
```

**제어 live 튜닝:**
```bash
ros2 param set /control_node kp                0.5
ros2 param set /control_node slew_rate_per_sec 4.5    # 초당! (프레임당 아니다)
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

### B-5. 워치독 검증 (⚠ 바퀴 지면에서 떼고)
```bash
ros2 param set /control_node engage true
pkill -f perception_node    # → "PERCEPTION STALE ..." + /control 이 (0,0)
pkill -f joystick_node      # → "JOYSTICK STALE ... Forcing engage OFF"
```
> ⚠ **패드를 뽑는 것으로는 joystick 워치독이 발동하지 않는다.** `joystick_node` 가 마지막
> 입력을 50Hz 로 계속 재발행한다 — 알려진 미수정 구멍 ([README.md](README.md) 남은 작업 A1).

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

## C. 오프라인 (🖥 로컬)

### C-1. panel_replay — 주행 raw 를 4패널로 되살리기 (⭐ 주력 도구)
`drive.launch` 는 패널을 녹화하지 않는다 (렌더링이 검출의 4배를 먹는다). 대신 raw + csv 를
남기고 **여기서 사후에** 같은 `dracer_core` 파이프라인으로 정확히 되살린다.

```bash
cd ~/workspace/SC2026
.venv/bin/python offline/panel_replay.py offline/rslt/recorder/raw/drive_<stamp>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml \
    --csv offline/rslt/recorder/csv/drive_<stamp>.csv

# 파라미터 A/B (원본은 안 건드린다)
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera ... --profile ... \
    --set lane_width_cm=35 --set branch_policy=random --no-video
```
`--csv` 를 주면 실차가 그때 발행한 LaneState 와 프레임 단위로 대조한다 (= 오프라인 튜닝을
믿어도 되는지 검증).

### C-2. calibrate — camera.yaml 재생성 (카메라 마운트를 움직였다면)
```bash
cd offline
../.venv/bin/python calibrate.py --intrinsics shots/intr --ground shots/ground.png \
    --square-mm 25.0 --lane-width-cm 35 --px-per-cm 4.0 \
    --out ../D-Racer-Kit/src/config/camera.yaml
```
> 마운트를 움직이면 **H(호모그래피)만** 무효가 된다. K/D(렌즈)는 살아남으므로 `--ground` 만
> 다시 찍으면 된다. 상세는 [offline/README.md](offline/README.md).

> **제어기 튜닝은 실차 폐루프에서만 한다.** 다른 조향을 했으면 다른 프레임을 봤을 텐데 녹화
> 영상으로는 그걸 재현할 수 없다(covariate shift). 위 B-4 의 `ros2 param set /control_node …`
> 를 쓰라.

---

## D. 조이스틱 · 토픽 참조

| 버튼 | 기능 |
|---|---|
| Y / B | steering_trim −/+ (calibration_mode) |
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
| `/lane/debug/compressed` | 4패널 디버그 (**구독자가 있을 때만 생성**) |
| `/control` | 제어 명령 (steering / throttle) |
| `/joystick` | 조이스틱 (control_msg · e_stop_en · engage · is_recording) |
| `/battery_status` | 배터리 |

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
