# KaAI SC2026

SEA:ME Hackathon 2026 — TOPST D3-G 기반 자율주행 스케일카.

카메라 한 장을 **캘리브레이션된 metric BEV**로 펴서 차선을 검출하고, 자차 통로(코리도어)의
중앙선을 추정해 조향·스로틀로 매핑한다. 인지 파이프라인은 **하나**다 — 실험용 모드·프리셋은
없다.

| 문서 | 내용 |
|---|---|
| **이 문서** | 시스템 구조 · 인지 · 제어 · 현재 상태 · 남은 작업 |
| [Task command.md](Task%20command.md) | 런치별 운영 명령 (calibrate / record / perceive / drive) |
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
| `perception` | 카메라 → `dracer_core` 파이프라인 → `/lane/state` |
| `control` | LaneState → `/control` + engage/E-stop 안전 게이팅 |
| `actuator` | PCA9685 조향/스로틀 구동. **E-STOP 이 여기서 걸린다** |
| `joystick` | 패드 입력 → 조향/스로틀 + 캘리브레이션 |
| `recorder` | 동기 mp4 + csv 기록 (**rosbag 아님**) |
| `monitor` | Flask 웹 대시보드 (:5000) |
| `battery` | INA219 배터리 상태 |
| `dracer_core` | **ROS 비의존** 인지·제어 코어 (numpy/opencv만). 노드 없음 — 온라인·오프라인이 이걸 공유한다 |
| `dracer_msgs` | 메시지 정의 (Battery · Control · Joystick · LaneState) |
| `dracer_bringup` | 런치 전용 |
| `topst_utils` | HW 드라이버 (d3racer · gamepads · ina219 · pca9685) |

> 실행 파일 이름은 전부 `*_node` 다 (`perception_node`, `control_node`, …).
> `dracer_core` · `dracer_msgs` · `dracer_bringup` · `topst_utils` 에는 노드가 없다.

### 노드 · 토픽

```
camera_node ──/camera/image/compressed──┬─► perception_node ──/lane/state──┬─► control_node ──/control──┐
                                        │        └──/lane/debug/compressed │                            │
                                        └─► monitor(:5000) · recorder      └─► recorder                 │
joystick_node ──/joystick──────────────────────────────────────────────────► control_node               │
                                        └──────────────────────────────────────────────────────────────► actuator_node
battery_node ──/battery_status──► monitor
```

- `/lane/debug/compressed` (4패널)는 **구독자가 있을 때만 생성**된다. `drive` 런치는 아무도
  구독하지 않으므로 인지가 패널을 아예 만들지 않는다 — 렌더링이 검출의 4배를 먹는다.
- `actuator_node` 와 `monitor_node` 는 **구독만** 한다.

### 런치 (`dracer_bringup`)

**base** = camera · actuator · joystick · monitor · battery (전 런치 공통).

| 런치 | 구성 | 용도 |
|---|---|---|
| `calibrate` | base | 카메라 각도 + `STEER_TRIM`/`ACCEL_RATIO` 저장 |
| `record` | base + recorder | 오프라인용 원본 영상 수집 |
| `perceive` | base + perception + recorder | 인지 검증 + 데이터 수집 + live 튜닝 |
| `drive` | base + perception + control + recorder | 자율주행 (`engage`) |

명령은 [Task command.md](Task%20command.md).

### 설정 파일 (`D-Racer-Kit/src/config/`)

| 파일 | 성격 |
|---|---|
| `vehicle_config.yaml` | **D3-G 로컬 캘리브레이션** — `STEER_TRIM`, `ACCEL_RATIO`, 카메라 장치·해상도. `git reset --hard` 가 덮어쓴다 |
| `camera.yaml` | 렌즈 K/D + 지면 호모그래피 H + BEV 범위. `offline/calibrate.py` 산출 |
| `profiles/track2025.yaml` | 인지·제어 파라미터 = **오프라인↔온라인 유일 계약**. 손으로 편집한다 |

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
`to_bev()` 가 **예외를 던진다** — 예전에는 조용히 검게 채웠다.

**차선폭 35cm 의 근거**: 테이프 실측은 내부 32cm / 외부 38cm (폭 3cm) 인데, 검출 다항식은
테이프의 **중심선**을 따라간다 → 코드가 쓰는 폭은 **중심선 간 35cm** 다.

### 3.2 코리도어 — 색이 경로다

두 차선을 짝지어 코리도어를 만든다. 후보는 **모든 조합**(`itertools.combinations`)이다.
인접 쌍만 보던 시절엔 로터리에서 `[흰, 노랑, 흰]` 이 잡히면 (흰,노랑)=5cm 거부 /
(노랑,흰)=30cm 거부 / **진짜 코리도어인 (흰,흰)=35cm 는 후보에 오르지도 못했다.** 차선이
3~4개 보이는 프레임이 6% 인데 **코리도어가 2개 이상인 프레임은 0이었다** — 보이지 않는
갈림길은 선택할 수 없다.

`_pair_gate` 통과 조건 (전부 AND):

1. **`pair_same_color`** — 흰-노랑 쌍 금지. **색 조합이 곧 경로 정체성이다**: 본선은 흰-흰,
   노란 지름길은 노랑-노랑. 흰-노랑은 물리적으로 한 차선의 두 경계일 수 없다 — 두 경로의
   경계가 교차점에서 우연히 35cm 떨어진 것이다. 폭 게이트로는 못 잡는다(둘 다 진짜로 35cm).
   평상시엔 무해하지만(ego 의 0.9%) **분기에서는 코리도어의 43%** 다.
2. y 겹침 ≥ `pair_overlap_min`
3. 최소 간격 ≥ `pair_gap_min_cm` (교차·붕괴 기각)
4. 폭이 `lane_width_cm` ± `pair_width_tol`

> ⚠ **평행도(`spread`)는 게이트가 아니다 — 보고만 한다.** 평행 게이트를 걸었더니 노란
> 갈림길이 **52% → 0%** 로 사라졌다. 파고들어 보니 21/21 전부 `[A,B]`+`[B,C]` 로 **경계를
> 공유하는 진짜 갈림길**이었고, **두 번째 코리도어가 벌어지는 게 당연했다 — 그게 갈림길이니까.**
> 평행도는 "이것이 코리도어인가" 가 아니라 **"내가 이 안에 있는가"** 다. 그래서
> `pair_parallel_cm` 은 `ego_center` 의 `nearest` 규칙에서만 쓴다.

**ego 선택 우선순위**: `tracked` (Tracker 가 따르던 코리도어) → `nearest` (차량축에 가장 가까운
in-lane 코리도어) → `coast` (경계 하나만 보고 차선폭 절반 평행이동).

**분기(`n_corridors >= 2`)는 진입 시 한 번만 래치한다.** 프레임마다 새로 고르면 차선폭만큼
진동한다 — 가설이 아니라 측정이다. 선택된 코리도어는 `Tracker.adopt` 로 정체성에 밀어넣는다.

### 3.3 coast — 마스크에 좌/우를 되묻는다

coast 는 경계 **하나**를 보고 "코리도어는 이것의 왼쪽/오른쪽에 있다" 고 **주장**한다. 그 주장이
틀리면 중심이 **차선폭만큼** 어긋난다 — 완벽한 차선 피팅(span 0.98, 잔차 1.3cm)을 달고서.
**기하로는 절대 못 잡는다.** 그래서 기하가 못 묻는 것을 묻는다: 주장하는 자리에 유령 경계를
놓고 **거기 마스크가 있는지 본다.** 거울상 쪽에만 근거가 있으면 방향을 잘못 골랐다.

세 개의 가드가 전부 필요하다:

1. `coast_flip_support` — 거울상에 실제 근거가 있어야 한다 (전체 coast 프레임 평균 0.005 —
   이 신호는 원래 희귀하다).
2. `coast_flip_empty` — 우리 유령 쪽엔 근거가 없어야 한다. 없으면 0.35 대 0.30 같은
   동전던지기에서 뒤집는다.
3. **축 가드 — flip 은 코리도어를 차량축 쪽으로 당겨야 한다.** 마스크는 "저기 차선이 있다"
   는 말할 수 있어도 **"저게 내 차선이다"** 는 못 한다. 4차선 로터리에서 1차선 coast 중이면
   내 쪽 경계는 FOV 밖이고 2차선 바깥 경계는 잘 보인다 → **옆 차선으로 flip.** 그런데 차는
   그 코리도어 **안에** 있다. flip 이 중심을 축에서 멀어지게 하면 그건 옆 차선이다.

그리고 **교정은 트래커를 고쳐야 한다** (`reseat_coast`). 출력만 뒤집으면 다음 프레임에 여전히
틀린 정체성이 또 coast 하고 마스크가 또 뒤집는다 → 매 프레임 진동(실측: 0 → 3).

### 3.4 상태 (`LaneState.state`)

| 값 | 조건 |
|---|---|
| `OK` | 측정 채택, conf 충분 |
| `LOW_CONF` | 측정 채택, conf 낮음 |
| `OUTLIER` | `\|center_error − ema\| > outlier_jump` 로 거부. **연속 거부가 `outlier_relatch` 에 도달하면 사실로 받아들이고 재시드** |
| `HOLD` | 측정 없음, `lost_stop_frames` 미만 |
| `LOST` | 측정 없음이 `lost_stop_frames` 이상 |

`OUTLIER` 는 스로틀을 끊는다 (`throttle_outlier: 0.0`). "지금 내가 주는 `ema` 를 믿지 마라"
는 뜻이고, 코리도어 전환 직후엔 부호까지 반대다.

### 3.5 `/lane/state` 계약 (`dracer_msgs/LaneState`)

| 필드 | 의미 | 누가 쓰나 |
|---|---|---|
| `center_error` | 정규화 횡오차 [-1,1], **+ = 코리도어가 오른쪽** (invalid 시 NaN) | control |
| `ema` | `center_error` EMA — **제어가 실제로 읽는 값** (`use_ema=True`) | control |
| `state` | `OK`/`LOW_CONF`/`OUTLIER`/`HOLD`/`LOST` | control (throttle) |
| `confidence` | **이산값**: 0.9(pair) / 0.5(coast) / 0.0(없음) | control (`conf_gate`) |
| `used_fallback` | coast 사용 여부 | 기록 |
| `n_corridors` | 유효 코리도어 수. **> 1 = 분기** | **(없음 — 판단 계층용)** |
| `ego_rule` | 무엇이 골랐나: `tracked`/`nearest`/`coast`/`branch_*`/`none` | **(없음 — 판단 계층용)** |
| `heading` / `curvature` | ego 중앙선 접선각(deg) / 곡률 | **아무도 안 씀** (metric PP 가 쓸 것) |
| `left_conf` / `right_conf` | 0 또는 1 | **아무도 안 씀** |

**`n_corridors` / `ego_rule` 은 아직 아무것도 하지 않는다 — 기록만 한다.** 판단 계층은 측정
위에 설계한다. `ego_rule` 이 `branch_random` 이면 그것은 **결정이 아니라 자리표시자이고, 스스로
그렇게 말하는 것**이다.

**라벨 분류(`W-L`, `YS-R`, …)는 계약에 없다** — 디버그 오버레이 전용이고 중앙선 도출에 쓰이지
않는다. (코드 주석은 "7-label" 이라 하지만 실제 라벨은 `W×2 + Y{S,L,R}×2` = **8개**다.)

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

> ⚠ **`sw_margin`·`jump_max`·`merge_dx`·`morph_v`·`sw_minpix`·`sw_peak_min`·`sw_peak_sep`·
> `pair_gap_min`·`pair_parallel`·`gate_min_px`·`lane_width_default`·`heading_frac` 는
> 파라미터가 아니다.** `cfg_to_px` 가 위 cm 값에서 계산한다(`DERIVED_PX`) — ROS 가 `param set`
> 을 **거부한다.**

**픽셀 *개수*는 면적이라 `px_per_cm` 의 제곱으로 변한다.** BEV 스케일을 절반으로 낮추면 모든
카운트가 1/4 이 되는데 임계값은 그대로라 검출이 로그 한 줄 없이 죽는다. 면적(`cm²`)·길이(`cm`)
로 표현하면 면역이다.

**프레임 수 기준 파라미터는 Hz 에 묶여 있다.** `lost_reset`·`lost_stop_frames`·
`outlier_relatch`·`median_window`·`ema_alpha` 는 시간이 아니라 **프레임 수**다. 인지가
10.7 → 30Hz 가 되면서 같은 값의 실제 지속시간이 **1/3 로 줄었다.** Hz 를 바꾸면 이들도 같이 봐야
한다.

> ⚠ `ros2 param set /perception_node <field>` 는 **파이프라인을 재생성한다** = Tracker/EMA
> 상태가 리셋된다. 주행 중이 아니라 정지 상태에서 바꿔라.

---

## 4. 제어 (`dracer_core/control_core.py`)

인지 레이트(30Hz, LaneState 마다 1회)로 도는 루프:

```
① dt      dt_s = clamp(dt, 0, dt_max)          ← 드롭아웃 직후 dt 는 갭 전체다
② 오차    e = center − center_target           (center = ema)
③ 제어기  u = f(e, ė, ∫e, heading, speed)      ← C1~C5. 내부값이다
④ 후처리  clamp(±steer_max) → slew_rate_per_sec·dt_s → out_ema
          → u · steer_sign                     ← 부호는 여기서 딱 한 번
⑤ 스로틀  throttle_base − curv_slow·|u|  (하한 throttle_min)
          state == OUTLIER → min(thr, throttle_outlier)   ← 하한을 우회한다
⑥ 안전    conf < conf_gate → 직전값 유지 · (노드단) 인지/조이스틱 워치독 · E-STOP
```

| 제어기 | 식 | 상태 |
|---|---|---|
| **PD** | `−(kp·e + kd·ė)` | ✅ **기본** (`kp 0.45`, `kd 0.1` — 성공 주행 값) |
| **PID** | `−(kp·e + kd·ė + ki·∫e)` | ✅ (anti-windup 없음 — 남은 작업 B5) |

`controller: PD | PID`. **모르는 이름은 예외를 던진다** — 조용한 폴백은 프로파일 오타 하나로
적어둔 것과 **다른 제어기로 차를 달리게** 한다.

> **Pure Pursuit 는 아직 없다.** 진짜 PP 는 전방주시 지점의 **실제 횡오차(cm)** 를 알아야
> 하는데, 그건 LaneState 가 아직 싣지 않는다 — 인지↔제어 계약 변경이 필요하다 (남은 작업 B1).

**부호 규약.** `center_error` + = 코리도어가 **오른쪽**. 법칙이 `u = −kp·e` 이므로 코리도어가
**왼쪽**(e<0)이면 `u > 0`. 이 차량(`steer_sign = +1.0`, 실차 주행으로 확인)에서 **`u > 0` =
좌조향**이다 — 차가 코리도어 쪽으로 돈다. 배선이 반대면 `steer_sign = -1`.

**`slew_rate_per_sec` 는 초당이다.** "프레임당" 이면 차의 최대 회전 속도가 그날의 인지 FPS 가
된다 (10.7Hz → 1.6/s, 30Hz → 4.5/s — 2.8배가 아무 말 없이 바뀐다). 성공 주행이 30Hz 에서 실제로
낸 값이 **4.5/s** 이고, 그걸 물리량으로 못박았다.

> ⚠ **`steer_max` 는 `1.0 − |STEER_TRIM|` 이어야 한다.** actuator 가 `clamp(u + trim, -1, 1)`
> 로 트림을 더하므로 u 와 트림이 같은 서보 예산을 나눠 쓴다. 넘으면 **한쪽 조향 권한만** 조용히
> 깎인다 (서보는 멈춰 있는데 제어기는 계속 더 달라고 한다 → 한쪽 코너만 언더스티어).
> 현재 `STEER_TRIM = 0.3` → `steer_max: 0.7`.

**A(engage) 와 X(E-STOP) 는 다른 층이다.** A 는 `control_node` 에게 "그만 보내라" 고 **부탁**
한다. X 는 `actuator` 에게 "무시해라" 고 **명령**한다. `control_node` 가 고장나면 A 는 아무것도
못 한다.

**워치독**: 인지 dead-man `state_timeout: 0.25s`, 조이스틱 dead-man `joystick_timeout: 0.3s`.
둘 중 하나라도 끊기면 `/control` 이 `(0, 0)` 으로 고정된다.

---

## 5. 오프라인 도구

로컬 macOS 에서 ROS 없이 돈다. `dracer_core` 가 순수 파이썬이라 **차와 같은 알고리즘**을 그대로
돌린다. 상세는 [offline/README.md](offline/README.md).

- **`panel_replay.py`** ⭐ 주력. 주행 raw + csv → 4패널 재구성 + 실차 LaneState 대조 + 파라미터 A/B
- `calibrate.py` — 체커보드·지면 사진 → `camera.yaml`

**제어기 튜닝은 실차 폐루프로만 한다.** 녹화 영상은 **사람이 지난 경로의 뷰**만 담으므로, 다르게
조향한 컨트롤러가 봤을 프레임은 존재하지 않는다(covariate shift). 오프라인으로는 명령 품질밖에
못 보고, 최종 판정은 어차피 실차다. profile 의 `[control]` 은 손으로 편집한다.

데이터: `offline/rslt/<세션>/` (raw + csv, 미추적 — scp 로 가져온다), `offline/calib/` (캘리브 사진).

---

## 6. 현재 상태 — 성공 주행 기준선

**D3-G 자율주행 성공** (`drive.launch`, 기록 = [`offline/rslt/0712`](offline/rslt/0712)).
아래는 그 3세션 **1,294프레임**을 그대로 집계한 값이다.

| | |
|---|---|
| 인지 레이트 | **30.0 Hz** (카메라 풀레이트) |
| `valid` | **100%** |
| `state` | **OK 100%** — OUTLIER · HOLD · LOST **0회** |
| 쌍검출 / coast | 36.4% / 65.5% |
| 분기 (`n_corridors ≥ 2`) | 21프레임 (**1.6%**), 최대 2개 |
| 자율 구동 (engage) | 853프레임 (65.9%) |
| 최대 \|steering\| | **0.399** (`steer_max 0.7` 초과 0회) |
| 인지 비용 | **0.72 ms/frame** (예산 33ms @30Hz) |

검증 전 항목 통과: 기동 로그 · 저속 주행 · perception/joystick 워치독 · 조향 방향 · 분기 확인.

> coast 가 65.5% 로 높다 — 이 트랙의 상당 구간에서 한쪽 경계만 FOV 에 들어온다는 뜻이다.
> 그래도 `state` 가 100% OK 인 것은 coast 의 좌/우를 마스크로 반증하기 때문이다 (§3.3).
> 쌍검출률 자체를 소프트웨어로 끌어올리는 건 렌즈 FOV 한계라 **측정으로 기각됐다** (아래).

---

## 7. 남은 작업

### 🔴 A. 안전

- [ ] **A1. `STEER_TRIM` 을 `ServoCalib.center_us` 로 옮기기.** `steer_max: 0.7` 은 임시
      방편이다 (§4). 트림을 서보 중립 자체로 옮기면 ±1.0 전체를 **대칭으로** 쓸 수 있다.

### 🟠 B. 기능 — 전제조건이 있는 것

- [ ] **B1. metric Pure Pursuit + 곡률 감속.** 현재 `center_error` 는 **전방 26~30cm** 에서
      측정된다 — preview 가 거의 없는 순수 횡오차 레귤레이터다. 저속에선 잘 돌지만 속도를
      올리면 반드시 진동한다. 그런데 **78cm 앞까지의 중앙선 다항식이 이미 손에 있다.**
      ```
      Ld   = 50cm (또는 speed 적응)
      v    = cam.cm_to_bev(0, Ld)[1]                      # BEV 행 (y_lo 로 클램프)
      x_cm = cam.bev_x_to_cm(_ebottom(ec['coeffs'], v))   # 그 지점의 실제 횡오차
      κ    = 2·x_cm / Ld²                                 # Pure Pursuit 곡률
      δ    = atan(WHEELBASE_cm · κ)                       # Ackermann
      u    = δ / MAX_STEER_RAD
      ```
      파라미터가 게인이 아니라 **측정값**(휠베이스·최대 조향각)이고, `κ` 가 공짜로 나오니
      **곡률 기반 감속**(`curv_slow·|κ|`)이 함께 붙는다. 지금 `curv_slow` 는 곡률이 아니라
      `|steering|` 에 곱해져 **결과에 반응**한다 — 위상이 늦다.
      **전제: LaneState 에 lookahead 횡오차 필드 추가 = 인지↔제어 계약 변경(리빌드).**
      그리고 휠베이스·최대 조향각 **실측**이 필요하다.
- [ ] **B2. 인지 EMA 시간 단위화.** `slew_rate` 만 고쳤다. `_Stabilizer.ema_alpha`,
      `Tracker._ema`, 폭 EMA 가 **아직 프레임 단위**다.
- [ ] **B3. `ros2 param set` 이 Tracker 를 리셋한다** — 주행 중 라이브 튜닝이 매번 인지
      불연속을 만든다.
- [ ] **B4. `color_gate: 0.15` 가 미션용 노란선을 지울 수 있다.** 소수색을 **통째로** 0 으로
      만든다. 분기 진입 판단에 치명적일 수 있다.
- [ ] **B5. PID anti-windup 없음.** 적분이 low-conf 에서도 누적되고 출력 포화와 연동되지
      않는다. `lost_reset`/`lost_stop_frames` 는 프레임 카운트다.

### 🟡 C. 미션 (합의된 순서)

```
객체 검출  →  판단 설계  →  Pure Pursuit
```

- [ ] **C1. 객체 검출** — 신호등 / 방향표지(ArUco) / 동적 장애물.
      *블로커: 현재 녹화(2025 트랙)에 이것들이 찍혀 있는지 미확인.*
- [ ] **C2. 판단 계층** — **분기에서 `tracked` 규칙을 오버라이드하는 것.** 측정 완료: 분기
      **218프레임(3.0%), 28회, p50 6프레임, max 20 (0.7초)**. 현재 분기에서 `tracked` 가 98% 를
      관통한다 = **시스템은 선택하지 않는다. 이어갈 뿐이다** → **차가 노란 지름길을 절대 못
      탄다.** 배선은 끝났다 (`n_corridors`·`ego_rule`·`branch_policy` 래치 + `adopt` + 코리도어
      색 조합). `choose_branch()` 안을 채우면 된다. 그리고 **0.7초 안에** 해야 한다.
- [ ] **C3. Pure Pursuit** — B1.

### ⚪ 하지 마라 — 측정으로 기각됨

- **연속 confidence** — `corr(quality, 오차) = +0.246`. **방향이 반대다.** 틀린 coast 는
  기하학적으로 완벽하다 (span 0.98, 잔차 1.3cm).
- **쌍검출률을 소프트웨어로 올리기 (2차선 프레임)** — 렌즈 FOV 한계다. 히스토그램 창·게이트
  전부 효과 0. *(3차선 이상은 모든 쌍 페어링으로 해결됨 — §3.2)*
- **핫패스 최적화** — `cv2.inRange` **−19%**, 2채널 remap **−15%**. **둘 다 느려진다.**
  파이프라인은 0.72ms / 33ms 예산이다. 최적화할 필요가 없다.
- **평행도를 페어링 게이트로** — 노란 갈림길을 **52% → 0%** 로 지운다 (§3.2).
</content>
