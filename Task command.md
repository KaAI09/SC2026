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
| `calibrate` | base | 카메라 각도 + 서보 중립(SERVO_CENTER_US)/ACCEL_RATIO 저장 |
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
grep -E "SERVO_|ACCEL_RATIO" src/config/vehicle_config.yaml       # 먼저 적어둬라

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
- 조이스틱: **Y/B** = 서보 중립 ∓10us, **L1/R1** = accel_ratio −/+ (즉시 `vehicle_config.yaml`
  저장), **X** = E-STOP.
```bash
grep -E "SERVO_|ACCEL_RATIO" "$WS"/src/config/vehicle_config.yaml
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
ros2 topic hz /lane/state
# 웹 :5000 → /lane/debug/compressed 4패널 저지연 스트림
```

> ⚠ **`perceive` 는 ~15Hz 로 돈다 (`drive` 는 30Hz).** monitor 가
> `/lane/debug/compressed` 를 구독하므로 perception 이 핫패스에서 4패널 합성 + JPEG
> 인코딩을 하기 때문이다 — **정상이다.** 다만 **여기서 튜닝한 레이트가 주행 레이트가
> 아니라는 뜻**이다. 시간 문턱값(EMA·`outlier_relatch_s`·`lost_stop_s`)은 B2 로 레이트에
> 면역이지만, 프레임 간격이 2배면 차가 프레임 사이에 2배 움직이므로 `jump_max_cm` 과
> Tracker 매칭 조건은 달라진다. 검출이 아슬아슬한 파라미터는 **`drive` 에서 재확인하라.**

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

> ✅ **`ros2 param set` 은 이제 상태를 보존한다** (B3). 로그에 `(Tracker/EMA 상태 유지)` 가
> 뜬다. 예전엔 파이프라인을 재생성해서, 파라미터를 **하나도 안 바꾸고** 재설정만 해도
> `|Δema|` 가 0.0377 튀었다 — 튜닝하려던 값의 효과(0.0014)보다 리셋 노이즈가 27배 컸다.
> (BEV 기하가 바뀌는 경우만 여전히 재생성한다 — 거기선 추적 중인 픽셀이 다른 장소를 뜻한다.)

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
control_node    : state_timeout=0.25s joystick_timeout=0.3s throttle_outlier=0.0 steer_max=1.0
actuator_node   : servo: center=1650us span=300us range=1250~2050us  command_hz=30.0
```
**Hz 가 30 이 아니면 멈춰라.** 게인은 30Hz 에서 튜닝됐다.
**⚠ `kp 0.75` / `slew 7.5` 는 서보 실측(A1) 이후 계산으로 잡은 값이고 트랙 미검증이다** — §B-7.

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

### B-7. 🚗 트랙 검증 — A1 · B2 · B3 (**미완. 이걸 통과해야 기준선이 갱신된다**)

셋 다 코드는 끝났고 오프라인/기동 검증도 통과했지만 **트랙에서 달려본 적이 없다.**
**A1 은 조향 스케일 자체를 바꿨다** (`kp` 0.45 → 0.75, `slew` 4.5 → 7.5, `steer_max` 0.7 → 1.0)
— 이 검증 전까지 **어떤 주행 결과도 기준선이 아니다.**

```bash
# ⚠ Joystick.msg 가 바뀌었다 -> --packages-select 쓰지 말 것
cd ~/SC2026 && git pull && cd D-Racer-Kit
colcon build && source install/setup.bash
```

**① 서보 (A1) — ⚠ 바퀴 지면에서 떼고**
```bash
ros2 launch dracer_bringup calibrate.launch.py
```
| 확인 | 기대 |
|---|---|
| 기동 로그 | `servo: center=1650us span=300us range=1250~2050us` |
| 조향 중립 | 바퀴가 **직진**. 아니면 Y/B 로 조정 (10us 스텝) |
| Y/B 조정 시 로그 | `servo centre -> ...us  (±25.0도, 좌우 대칭)` |
| 조이스틱 좌/우 최대 | **좌우 조향각이 눈으로 봐도 대칭** (각 25도) |
| 옛 saturation 경고 | **안 떠야 한다** (원인이 사라졌다) |

**② 인지 (B2 · B3) — 액추에이션 없음**
```bash
ros2 launch dracer_bringup perceive.launch.py
ros2 topic hz /lane/state          # perceive 는 ~15Hz 가 정상 (디버그 렌더링). drive 에서 30Hz.
ros2 topic echo /lane/state --once # state=OK, valid=true

# B3: 라이브 튜닝이 Tracker 를 리셋하지 않는가
ros2 param set /perception_node yellow_s_min 70
#   -> 로그에 "perception live-update: [...] (Tracker/EMA 상태 유지)"
#   -> center_error 가 튀지 않아야 한다
ros2 param set /perception_node yellow_s_min 65    # 원복
```

**③ 저속 자율 주행 (A1 의 게인) — ⚠ 여기가 진짜 시험이다**
```bash
ros2 launch dracer_bringup drive.launch.py
ros2 topic hz /lane/state                          # 30Hz 유지되는가
# 바퀴 띄운 상태로 engage(A) 확인 -> 내리고 저속
```
`kp 0.75` / `slew 7.5` 는 **계산상 등가일 뿐 실차 미검증**이다. 저속·감시 하에서:

```bash
ros2 param set /control_node kp 0.6      # 진동하면 낮춘다
ros2 param set /control_node kp 0.9      # 굼뜨면 올린다
ros2 topic echo /control --field steering   # 최대 |steering| 이 1.0 에 닿는가
```
| 증상 | 뜻 | 조치 |
|---|---|---|
| 좌우로 진동 | `kp` 과다 | `kp` 낮춘다 |
| 코너에서 늦게 반응 | `kp` 부족 / `slew` 제한 | `kp` 올린다, `slew` 확인 |
| **한쪽 코너만 언더스티어** | 서보 중립이 틀어졌다 | **Y/B 로 중립 재조정** (A1 이 없앤 증상이다 — 다시 나오면 안 된다) |

**④ 성공 시 — 기준선 갱신**
```bash
# START 로 녹화하고 raw+csv 를 로컬로 (§E)
# panel_replay 로 재구성 -> README §6 의 수치를 새 주행으로 갈아끼운다
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

### C-2. calibrate — 카메라 마운트를 움직였다면 (⭐ 지면 사진 1장)

**마운트를 움직이면 `H`(자세)만 무효가 된다. `K·D`(렌즈)는 살아남는다** — 그러니 체커보드
20장은 다시 안 찍는다. **지면 사진 1장**이면 된다. `--from-camera` 가 기존 `camera.yaml` 에서
`K·D`(+ `px_per_cm`·축 오프셋·런타임 해상도)를 물려받고 `H` 만 다시 푼다.

> ⚠ **`--intrinsics` 는 렌즈/카메라 자체를 바꿨을 때만.** 그냥 마운트를 조정한 것이라면
> `--from-camera` 다. (검증: 같은 지면 사진에 대해 두 경로가 같은 `K·D`·`H` 를 낸다.)

**① 🚗 D3-G — 지면 사진 1장 촬영**
```bash
cd ~/SC2026/D-Racer-Kit
cp src/config/vehicle_config.yaml /tmp/vehicle_config.bak     # ★ 백업

# 촬영 해상도를 올린다 (지면 보드는 원근에 눌려 코너가 안 잡힌다)
sed -i 's/^IMAGE_WIDTH:.*/IMAGE_WIDTH: 960/; s/^IMAGE_HEIGHT:.*/IMAGE_HEIGHT: 720/' \
    src/config/vehicle_config.yaml

ros2 launch dracer_bringup calibrate.launch.py image_topic:=/calib/preview/compressed
python3 scripts/capture_camera_calib.py --out ~/calib --name ground --count 1
#   보드를 노면에 평평하게 눕히고, 좌우 중앙, 화면에 다 들어오게. 웹 :5000 에서 코너 확인.
#   min gap >= 14px 면 좋다. 10px 미만이면 해상도를 더 올릴 것.

cp /tmp/vehicle_config.bak src/config/vehicle_config.yaml     # ★★ 원복 (필수)
grep -E "IMAGE_(WIDTH|HEIGHT)" src/config/vehicle_config.yaml #    320 / 240 확인
```

**①-b 🚗 D3-G — 검증용 직선 구간 프레임** (해상도 원복 **후**, 런타임 320x240 으로)

`--check` 는 **새 마운트로 찍은** 직선 구간 프레임이 필요하다. 기존 `rslt/0712` 는 **옛
마운트** 영상이라 새 캘리브 검증에 쓸 수 없다.

```bash
ros2 launch dracer_bringup record.launch.py     # 직선 구간에 세워두고 START → 2~3초 → STOP
```

> **트랙이 없으면 인쇄 타깃을 쓴다.** `offline/calib/lane_target.pdf` (A4 4장 = 40×57.4cm,
> 검정 노면 + 실제와 같은 35cm 차선). **배율 100%** 로 인쇄 → 재단선까지 잘라 맞대어 붙이고
> (**뒷면에서** 테이프) → 평평하게 깔고 → 그 위에 차를 정상 주행 상태로 세워 촬영.
> 상세·주의는 [offline/README.md](offline/README.md) 와 PDF 마지막 쪽.

**② 🖥 로컬 — H 만 다시 푼다**
```bash
cd ~/workspace/SC2026
scp topst@<D3-G_IP>:~/calib/ground_00.png offline/calib/ground_01.png
cp D-Racer-Kit/src/config/camera.yaml /tmp/camera.yaml.bak    # ★ 백업 (아래 ⚠ 참조)

cd offline
../.venv/bin/python calibrate.py \
    --from-camera ../D-Racer-Kit/src/config/camera.yaml \
    --ground calib/ground_01.png --square-mm 25.0 --lane-width-cm 35 \
    --out ../D-Racer-Kit/src/config/camera.yaml
```
확인할 값: **재투영 RMS < 1cm**, 보드 횡위치 ≈ 0, 카메라 높이가 실측과 ±3cm 이내
(`--cam-height-cm <실측>` 을 주면 교차검증해 준다).

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

**④ 🚗 D3-G — 실차 저속 확인.** B-3(`perceive`)로 차선 검출을 눈으로 보고, 그 다음 B-4.

> ⚠ **`camera.yaml` 을 덮어쓰기 전에 반드시 백업하라.** 현재 커밋된 `camera.yaml` 은
> **지금의 `calib.py` 로 재현되지 않는다** (`H[2,2]` 가 0.539 인데 현재 `build_model` 은
> 구조상 1.0 만 낸다 — 커밋 전 다른 버전으로 생성된 파일이다). 실차 주행 성공(0712)의
> 기준선이므로, 새 캘리브가 실차에서 확인되기 전까지 **되돌릴 수 있어야 한다.**
> (참고: 두 파일 모두 0712 재생에서 "재현 일치" 를 통과한다 — 재생성본이 쌍검출 73%→84%
> 로 오히려 높다. 그래도 실차 확인 전에는 기준선을 지운다.)

> **제어기 튜닝은 실차 폐루프에서만 한다.** 다른 조향을 했으면 다른 프레임을 봤을 텐데 녹화
> 영상으로는 그걸 재현할 수 없다(covariate shift). 위 B-4 의 `ros2 param set /control_node …`
> 를 쓰라.

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
