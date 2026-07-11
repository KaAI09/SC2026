# ROLLBACK & TODO — 0712 변경분

**기준선:** 커밋 `2d329ee` = 2026-07-11 14:55 **완주** (drive.launch 3세션, 1,505프레임,
트랙 이탈 0, 인지 30.0Hz, LOST 0%).

**현재:** 커밋 10개. **오프라인 회귀만 통과했다. D3-G 실차 재주행 미완** (트랙 사용 종료).

| | |
|---|---|
| 회귀 테스트 | 68/68 |
| 0711 raw 재생 (1,505프레임) | `valid 100%` · `LOST 0%` · 145515·145537 `OUTLIER 0%` |
| 인지 비용 | 0.72 ms/frame (예산 33ms @30Hz) |

이 문서는 ① 트랙에서 무언가 나빠졌을 때 **코드를 건드리지 않고** 되돌리는 법, ② **전체 명령
시퀀스**, ③ **남은 작업(TODO)** 이다.

---
---

# 1. 전체 명령 시퀀스 — git 부터 테스트까지

## 1-0. 🖥 로컬 (macOS) — 오프라인 검증

```bash
cd ~/workspace/SC2026

# 최초 1회
.venv/bin/pip install -e D-Racer-Kit/src/dracer_core

# 회귀 (ROS 불필요)
.venv/bin/python offline/panel_replay.py offline/rslt/recorder/raw/drive_20260711_145515.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml --no-video
#   기대: valid=100%  쌍검출=65%  coast=35%  OUTLIER=0%  LOST=0%
```

## 1-1. 🚗 D3-G — 동기화 + 빌드

```bash
export WS=~/SC2026/D-Racer-Kit
cd "$WS"

# ⚠ vehicle_config.yaml 은 D3-G 로컬 캘리브레이션(STEER_TRIM/ACCEL_RATIO)이다.
#    reset --hard 는 그것을 덮어쓴다. 먼저 값을 적어두거나 백업하라.
grep -E "STEER_TRIM|ACCEL_RATIO" src/config/vehicle_config.yaml

git fetch origin && git checkout kos/track-test2
git reset --hard origin/kos/track-test2

source /opt/ros/humble/setup.bash

# ⚠ dracer_msgs 인터페이스가 바뀌었다 (LaneState 에 n_corridors / ego_rule 추가).
#    메시지 패키지를 먼저 빌드하지 않으면 나머지가 컴파일되지 않는다.
colcon build --packages-select dracer_msgs --symlink-install
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 새 터미널마다: `cd "$WS" && source /opt/ros/humble/setup.bash && source install/setup.bash`

## 1-2. 🚗 정지 상태 검증 — **바퀴를 지면에서 떼고**

### ① 기동 로그가 이 값이어야 한다

```bash
ros2 launch dracer_bringup drive.launch.py     # engage=false 로 시작
```

```
perception_node : BEV 232x207px @ 4.00px/cm  x=+-29cm y=26..78cm
                  [lane] 30.0Hz state=OK ... corridors=1[tracked]
control_node    : state_timeout=0.25s joystick_timeout=0.3s
                  throttle_outlier=0.0 steer_max=0.7
actuator_node   : command_hz=30.0
```

**`[lane]` 앞의 Hz 가 30 이 아니면 여기서 멈춰라.** 게인(`kp 0.45`)은 30Hz 에서 튜닝됐다.
24Hz 미만이면 인지가 스스로 경고를 찍는다 (`rate_floor_hz`).

### ② perception 워치독 (새 터미널)

```bash
ros2 param set /control_node engage true        # 바퀴 떴는지 확인 후에만
ros2 topic echo /control                        # 명령이 나오는지 확인
# 다른 터미널에서:
pkill -f perception_node
```
→ `PERCEPTION STALE: no /lane/state for >0.25s` 가 뜨고 `/control` 이 `(0, 0)` 으로 고정.
→ 인지를 다시 띄우면 `lane state recovered — controller reset`.

**이게 안 되면 인지가 죽어도 차가 계속 달린다. 반드시 확인하라.**

### ③ joystick 워치독

```bash
pkill -f joystick_node
```
→ `JOYSTICK STALE ... Forcing engage OFF`.

> ⚠ **패드를 뽑는 것으로는 발동하지 않는다.** `joystick_node` 가 read 실패를 삼키고 마지막
> 입력을 50Hz 로 계속 재발행하므로 `/joystick` 이 신선하다. **반드시 노드를 죽여서** 시험하라.
> (이건 알려진 미수정 구멍 — TODO C1)

### ④ 조향 방향

```bash
ros2 topic echo /control --once
```
차선이 **왼쪽**에 있으면 `steering > 0` (좌조향) 이어야 한다. 반대면:
```bash
ros2 param set /control_node steer_sign -1.0
```

## 1-3. 🚗 저속 트랙 주행

```bash
ros2 launch dracer_bringup drive.launch.py
# 조이스틱 A = engage 토글 / X = E-STOP / START = 녹화
```

**여기서 볼 것은 단 하나다: OUTLIER 스로틀 컷의 주행감.**

0711 데이터 기준 **랩당 3~4회, 매번 최대 5프레임(0.17초)** 타력주행이 예상된다 (전체 주행
시간의 0.5초 남짓). 차가 울컥거리거나 멈칫하면:

```bash
ros2 param set /control_node throttle_outlier 1.0    # 컷 끄기 (구 동작)
```

`throttle_base 0.23` / `throttle_min 0.22` 라 스로틀 여유폭이 0.01 뿐이다. 감속 정책이
사실상 "끊거나 말거나" 뿐이라는 뜻이다.

**나머지 변경은 전부 실패 경로에서만 동작하므로 정상 주행에 영향이 없다.**

## 1-4. 🚗 로터리 분기 확인 (신규)

```bash
# 주행 후, csv 를 로컬로
scp -r topst@<D3-G_IP>:~/recorder/{panel,raw,csv} ./offline/rslt/
```

```bash
# 🖥 로컬: 분기가 실제로 보이는가
python3 -c "
import csv,collections
r=list(csv.DictReader(open('offline/rslt/csv/drive_<stamp>.csv')))
nc=collections.Counter(x['n_corridors'] for x in r)
rule=collections.Counter(x['ego_rule'] for x in r if x['n_corridors'] not in ('','0','1'))
print('코리도어 수:', dict(nc))
print('분기에서 선택한 규칙:', dict(rule))"
```

기대: `n_corridors >= 2` 가 **3% 안팎**, 분기에서 `ego_rule` 이 대부분 `tracked`
(= 시스템이 선택하지 않고 이어간다 — 판단 계층이 없으니 정상).

```bash
# 랜덤 경로 선택 실험 (⚠ 차가 노란 지름길로 갈 수 있다. 저속·감시 하에서만)
ros2 param set /perception_node branch_policy random
ros2 param set /perception_node branch_policy keep     # 원복
```

---
---

# 2. 되돌리는 법

## 2-1. 즉시 전체 복구 (리빌드 없음)

```bash
ros2 launch dracer_bringup drive.launch.py \
    profile:=$HOME/SC2026/D-Racer-Kit/src/config/profiles/track2025_0711.yaml \
    command_hz:=10.0 publish_rate:=20.0
```

`command_hz` / `publish_rate` 는 **반드시 명령줄에 같이** 넣어야 한다. 둘 다 노드 생성 시점에
타이머를 만들기 때문에 프로파일이나 `ros2 param set` 으로는 되돌릴 수 없다.

## 2-2. 변경 항목별 복구값

| 항목 | 0711 (검증됨) | 현재 | 복구 | 정상 주행 중 발동? |
|---|---|---|---|---|
| **`throttle_outlier`** | 없음 | `0.0` | **[P]** `1.0` | 🔴 **예 — 랩당 3~4회, 최대 0.17s** |
| `outlier_relatch` | `6` | `5` | **[P]** `6` | 예 (측정상 개선) |
| `steer_max` | `0.8` | `0.7` | **[P]** `0.8` | 아니오 (실측 max \|u\| = 0.528) |
| `track_width_tol` | 없음 | `0.25` | **[P]** `0.0` | 아니오 (1,505프레임 중 0회 발동) |
| `coast_flip_support` | 없음 | `0.15` | **[P]** `0.0` | 예 (1,505프레임 중 **1회** 발동) |
| `pair_same_color` | 없음 | `true` | **[P]** `false` | 예 (흰-노랑 ego 35 → 0) |
| `pair_parallel_cm` | 없음 | `8.0` | **[P]** `0.0` | 예 (ego 선택에만 사용) |
| `branch_policy` | 없음 | `keep` | — | **아니오 — 기본값이 구 동작이다** |
| `slew_rate` → `slew_rate_per_sec` | `0.15`/step | `4.5`/sec | **[P]** `4.5` | 아니오 (30Hz 에서 동일) |
| `dt_max` | 없음 | `0.1` | **[P]** `10.0` | 아니오 (30Hz 에서 dt=0.033) |
| `command_hz` | `10.0` | `30.0` | **[L]** `10.0` | 예 (지연 감소) |
| `publish_rate` | `20.0` | `30.0` | **[L]** `20.0` | 예 (지연 감소) |
| `state_timeout` | 없음 | `0.25` | **[P]** `0.0` = 비활성 | 아니오 (고장 시만) |
| `joystick_timeout` | 없음 | `0.3` | **[P]** `0.0` = 비활성 | 아니오 (고장 시만) |
| `rate_floor_hz` | 없음 | `24.0` | **[P]** `0.0` = 비활성 | 로그만 |

**[P]** = 프로파일 / `ros2 param set` · **[L]** = 런치 인자로만

### 트랙에서 한 항목씩 끄기 (engage 해제 후)

```bash
ros2 param set /control_node    throttle_outlier   1.0    # ← 먼저 이것부터
ros2 param set /control_node    steer_max          0.8
ros2 param set /perception_node outlier_relatch    6
ros2 param set /perception_node track_width_tol    0.0
ros2 param set /perception_node coast_flip_support 0.0
ros2 param set /perception_node pair_same_color    false
ros2 param set /perception_node pair_parallel_cm   0.0
ros2 param set /control_node    state_timeout      0.0    # 워치독 끄기 (권장하지 않음)
ros2 param set /control_node    joystick_timeout   0.0
```

> ⚠ `ros2 param set /perception_node <field>` 는 파이프라인을 재생성한다 = **Tracker/EMA
> 상태가 리셋된다.** 주행 중이 아니라 정지 상태에서 바꿔라. (TODO C4)
>
> ⚠ `sw_margin` · `jump_max` · `merge_dx` 등 **11개는 이제 파라미터가 아니다** (`DERIVED_PX`).
> `cfg_to_px` 가 cm 값에서 계산하므로 ROS 가 `param set` 을 **거부한다.** 진짜 노브는
> `sw_margin_cm` · `jump_max_cm` · `merge_dx_cm` 다. 예전에는 성공했다고 답하고 아무 일도
> 하지 않았다.

## 2-3. 파라미터로 되돌아가지 않는 것 (리빌드 필요)

| 변경 | 왜 안전한가 (측정) |
|---|---|
| `_Stabilizer` 카운터 분리 | 구/신을 1,505프레임에 나란히 통과 → **state·ema 비트 단위 동일** |
| `steer_sign` 을 `_emit()` 한 곳에서만 | `steer_sign=1.0` 에서 산술적으로 동일. 문제의 hold 경로는 현재 도달 불가능 |
| Tracker 폭 median | 외삽 제거. 최악 오차 29.0 → 32.4cm (실제 35cm) |
| **P2: 중앙 하드컷 제거 + `adopt`** | **진동 7 → 0**, OUTLIER 2.4 → 1.1%. ⚠ 가장 큰 인지 변경 |
| **모든 쌍 페어링** | 쌍검출 +3%p, 분기 가시화 0% → 3% |
| front-view 삭제 | 실차는 이미 BEV 전용이었다 |

```bash
# 인지만 통째로 되돌리기 (위 6개가 전부 함께 돌아간다)
git checkout 2d329ee -- D-Racer-Kit/src/dracer_core/dracer_core/perception_core.py
colcon build --packages-select dracer_core --symlink-install
```

## 2-4. 완전 복구 (핵폭탄)

```bash
git checkout 2d329ee -- D-Racer-Kit/ offline/
colcon build --symlink-install
```

---
---

# 3. TODO — 남은 작업

> **§1 의 실차 검증이 전부 통과한 뒤에 착수한다.** 10개 커밋이 미검증 상태로 쌓여 있고,
> 그 위에 무엇을 올리면 문제가 생겼을 때 원인 분리가 불가능하다.

## 🔴 A. 실차 검증 — 다른 모든 것을 막고 있다

- [x] **§1-1 빌드** (`dracer_msgs` 먼저) — 2026-07-11 19:30 완료
- [x] **§1-2 기동 로그** — 2026-07-11 19:30 **전부 기대값**
      ```
      actuator   : command_hz=30.0
      control    : controller=C2 throttle_base=0.23 steer_max=0.7
                   state_timeout=0.25s joystick_timeout=0.3s throttle_outlier=0.0
      perception : BEV 232x207px @ 4.00px/cm  x=±29cm y=26..78cm  ground_rms=0.114px
      [lane]     : 30.0Hz  state=OK  LOST 0  corridors=1[tracked] / 0[coast]
      ```
- [x] **§1-3 저속 주행** — 2026-07-11 **주행 정상**. OUTLIER 스로틀 컷은 이 세션에서 미발생
      (`state=OK` 내내). 랩 전체를 돌려 랩당 3~4회 예상치를 확인해야 한다.
- [ ] **perception / joystick 워치독 실증** — 아직 안 함 (`pkill -f perception_node` 등)
- [ ] **조향 방향** — `/control` echo 로 확인 안 함
- [ ] **§1-4 분기 확인** — `n_corridors >= 2` 가 로터리에서 실제로 뜨는가 ⬅ **다음**

### A-x. 0711 19:30 기동 로그에서 발견 (미수정, 합의 하에 보류)

- [ ] **A1. engage 를 인지가 뜨기 전에 눌러도 아무 로그가 없다.**
      ```
      19:30:06.396  ENGAGE ON (joystick A)
      19:30:07.279  perception: loaded profile      <- 0.9초 뒤
      19:30:07.926  [lane] 첫 LaneState             <- 1.5초 뒤
      ```
      perception 워치독이 `/control` 을 `(0,0)` 으로 잡고 있었다 — **의도대로 동작했다.**
      그러나 `state_stale` 초기값이 `True` (기동 스팸 방지) 라서 **로그가 하나도 없다.**
      운전자는 "A 를 눌렀는데 왜 안 가지?" 를 알 방법이 없다. 한 줄 로그면 된다.
- [ ] **A2. `control_node` 만 종료 시 다른 예외로 죽는다.**
      ```
      RuntimeError: Unable to convert call argument to Python object
        in _take_subscription -> sub.handle.take_message(...)
      ```
      Ctrl-C 로 컨텍스트가 무효화된 상태의 in-flight `take_message` 실패 (rclpy 종료 레이스).
      `main()` 이 `except KeyboardInterrupt` 만 잡아서 이 `RuntimeError` 가 빠져나간다.
      → `destroy_node()` 의 **중립 `(0,0)` 발행**을 건너뛸 수 있다. actuator 의
      `control_timeout: 0.5s` 가 백업이라 차는 결국 서지만 최대 0.5초 늦는다.
      (나머지 7개 노드의 "rcl_shutdown already called" 는 기존부터 있던 이중 shutdown
       패턴이고 무해하다 — 내 변경과 무관.)
- [ ] **A3. 로그 스팸이 CPU 를 먹고 `[lane]` 로그를 묻는다.**
      `camera_node` "Published frame" @30Hz (초당 30줄) + `joystick_node` DBG @5Hz (초당 45줄)
      = **초당 75줄**. D3-G 에서 무시할 수 없고, 실제로 봐야 할 `[lane]` 이 안 보인다.
      `drive.launch` 에서 `debug_log:=false` / `debug_log_enable:=false` 로 끄면 된다.

## 🔴 B. 안전 — 알려진 미수정 구멍

- [ ] **C1. `joystick_node` 가 패드 언플러그를 삼킨다.**
      `gamepad_read_loop` 의 `except` 가 에러를 로그만 찍고 넘어가서, 타이머가 **마지막 입력을
      50Hz 로 계속 재발행**한다 — `engage=True` 플래그까지. `/joystick` 이 완벽하게 신선하므로
      **control_node 의 조이스틱 워치독이 못 잡는다.** 그리고 패드가 죽으면 A 도 X 도 누를 수
      없다. `except` 에서 `engage_latched = False` 로 내리는 한 줄이면 된다.
- [ ] **C2. `STEER_TRIM` 을 `ServoCalib.center_us` 로 옮기기.**
      `steer_max: 0.7` 은 임시 방편이다. actuator 가 `clamp(u + 0.3, -1, 1)` 로 트림을 더하므로
      u 와 트림이 같은 서보 예산을 나눠 쓴다. 트림을 서보 중립 자체로 옮기면 ±1.0 전체를
      **대칭으로** 쓸 수 있다. 수동 주행 경로까지 건드리므로 별도 작업.

## 🟠 C. 원래 계획에서 미완

- [ ] **B1. 인지 EMA 시간 단위화.** `slew_rate` 만 고쳤다. `_Stabilizer.ema_alpha`,
      `Tracker._ema`, 폭 EMA(`0.6/0.4` 하드코딩)가 **아직 프레임 단위**다. 20Hz 에서 시정수가
      65 → 98ms 로 늘어난다. `LanePipeline.process()` 에 `dt` 를 넘겨야 해서 시그니처가 바뀐다.
      (저FPS 에서 "더 스무딩되는" 안전한 방향이라 후순위)
- [ ] **B2. C4/C5 파라미터를 `_CTRL_FLOATS` 에 노출.** `lookahead`, `pp_gain`, `stanley_k`,
      `stanley_soft`, `heading_gain`, `use_ema`, `i_clamp` 가 **ROS 파라미터가 아니다.**
      지금 `controller: C4` 로 바꿔도 **튜닝할 방법이 없다.** Pure Pursuit 의 전제조건.
- [ ] **C3. coast 폭 오류.** `coast_side` 가 못 잡는 15cm 오차 1건 = **방향이 아니라 폭** 버그다.
- [ ] **C4. `ros2 param set` 이 Tracker 를 리셋한다.** `LanePipeline` 을 통째로 재생성하므로
      주행 중 라이브 튜닝이 매번 인지 불연속을 만든다.
- [ ] **C5. `color_gate: 0.15` 가 미션용 노란선을 지울 수 있다.** 소수색을 **통째로** 0 으로
      만든다. 분기 진입 판단에 치명적일 수 있다 — 판단 계층을 붙일 때 필수.
- [ ] **C6.** PID 적분이 low-conf 에서도 누적되고 anti-windup 이 없다 (C3 쓸 때만).
- [ ] **C7.** `lost_reset` / `lost_stop_frames` 가 프레임 카운트 (저위험 — `HOLD` 부터 이미
      스로틀이 끊긴다).
- [ ] **C8.** 7-label 분류가 제어/판단에 **미사용** (디버그 오버레이 전용).
- [ ] **C9.** 36cm coast 오차 1건 — 양쪽 다 근거가 없어 정보가 없다.

## 🟡 D. 미션 (합의된 순서)

```
객체 검출  →  판단 설계  →  Pure Pursuit
```

- [ ] **D1. 객체 검출** — 신호등 / 방향표지(ArUco) / 동적 장애물.
      *블로커: 현재 녹화(2025 트랙)에 이것들이 찍혀 있는지 미확인.*
- [ ] **D2. 판단 계층** — **분기에서 `tracked` 규칙을 오버라이드하는 것.**
      측정 완료: 분기 **218프레임(3.0%), 28회, p50 6프레임, max 20 (0.7초)**.
      현재 분기에서 `tracked` 가 98% 를 관통한다 = **시스템은 선택하지 않는다. 이어갈 뿐이다.**
      → **차가 노란 지름길을 절대 못 탄다.**
      배선은 끝났다: `n_corridors` · `ego_rule` · `branch_policy`(래치 + `adopt`) · 코리도어의
      색 조합(흰-흰 = 본선, 노랑-노랑 = 지름길). `choose_branch()` 안을 채우면 된다.
      메시지를 `LaneObservation` / `PathTarget` 으로 쪼개는 것도 여기서.
- [ ] **D3. metric Pure Pursuit + 곡률 감속.** *B2 가 선행.*
      현재 `center_error` 는 **전방 26~30cm** 에서 측정된다 — preview 가 거의 없는 순수 횡오차
      레귤레이터다. 저속에선 잘 돌지만 속도를 올리면 반드시 진동한다. 그런데 **78cm 앞까지의
      중앙선 다항식이 이미 손에 있다.** `curvature` 는 발행만 되고 아무도 안 쓴다.

## ⚪ E. 하지 마라 (측정으로 기각됨 — `PERCEPTION.md` §8-, §8--, §8---)

- **연속 confidence** — `corr(quality, 오차) = +0.246`. **방향이 반대다.** 틀린 coast 는
  기하학적으로 완벽하다 (span 0.98, 잔차 1.3cm).
- **쌍검출률 개선 (2차선 프레임)** — 렌즈 FOV 한계다. 히스토그램 창·게이트 전부 효과 0.
  *(3차선 이상은 모든 쌍 페어링으로 해결됨 — §8++)*
- **핫패스 최적화** — `cv2.inRange` **−19%**, 2채널 remap **−15%**. **둘 다 느려진다.**
  파이프라인은 0.72ms / 33ms 예산이다. 최적화할 필요가 없다.
- **평행도를 페어링 게이트로** — 노란 갈림길을 **52% → 0%** 로 지운다. 평행도는
  "이것이 코리도어인가" 가 아니라 **"내가 이 안에 있는가"** 다.

---

## 문서 지도

| 문서 | 내용 |
|---|---|
| [PERCEPTION.md](PERCEPTION.md) | 인지 파이프라인 · 한계 · **채택/기각된 접근과 그 측정치** |
| [Task command.md](Task%20command.md) | 런치별 운영 명령 (calibrate / record / perceive / drive) |
| [offline/PIPELINE.md](offline/PIPELINE.md) | 오프라인 도구 (panel_replay / control_predict / control_select) |
| [offline/CONTROL_DESIGN.md](offline/CONTROL_DESIGN.md) | 제어기 C1~C5 설계 · open-loop 평가의 한계 |
| [REFACTORING.md](REFACTORING.md) | (이력) 0709 이전 리팩토링 기록 |
