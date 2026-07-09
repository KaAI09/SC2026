# SC2026 리팩토링 로그 — `kos/track-test`

> 이 문서는 `kos/track-test` 브랜치 **전용** 진행 기록이다. `main`에 병합하지 않는다.
> 워크트리: `SC2026(refactoring)/` (베이스: `main` @ e964069).

## 0. 목적 / 원칙

기존 테스트 코드(현재 `kos/hw-cam-track-test`에 산재)를 **온라인(실차)** 과 **오프라인(영상 평가)** 두 환경으로 명확히 분리하고, 모놀리식 노드를 **인지 / 제어 / 액추에이터 / 기록** 노드로 쪼개 경량화한다.

- **오프라인**: 주행 영상으로 모든 주행 로직·알고리즘·파라미터 조합을 평가 → 트랙 특화 **최적 프로파일** 도출.
- **온라인**: 오프라인이 고른 **단일 최적 프로파일**로만 D3-G 실차 주행 + 로그/영상 저장.
- **단일 진실원(single source of truth)**: 인지/제어 코어 로직은 한 곳에만 두고 온·오프라인이 공유(현재 `src/opencv/`와 `local_scripts/`에 중복 복사되어 있어 드리프트 위험 → 제거 대상).

## 1. 현재 상태 스냅샷 (리팩토링 시작 시점)

`main` 베이스 + `kos/hw-cam-track-test`에 구현된 기능들을 이 브랜치로 삽입 예정.

| 구분 | 위치 | 성격 |
|---|---|---|
| 인지 코어 | `src/opencv/opencv/lane_core.py` **및** `local_scripts/lane_preview.py`(복붙) | 순수 파이썬 파이프라인(A~F 축, 프리셋 M/O) |
| 제어 코어 | `src/opencv/opencv/control_core.py` **및** `local_scripts/control_core.py`(복붙) | 순수 파이썬 컨트롤러(C1~C5) |
| 인지 노드 | `src/opencv/opencv/lane_detect_node.py` | 인지 전용 + 오버레이 mp4/CSV 녹화(액추에이션 없음) |
| 폐루프 노드 | `src/opencv/opencv/lane_follow_node.py` | **인지+제어+액추에이션+녹화 모놀리식** ← 분리 대상 |
| 액추에이터 | `src/control/` (`control_node`) | PCA9685/D3Racer PWM 드라이버 + 데드맨 워치독 (베이스, 유지) |
| 오프라인 도구 | `local_scripts/` (`lane_preview.py`, `lane_compare.py`, `control_eval.py`) | 조합 비교/평가 |
| 메시지 | `control_msgs/Control`, `joystick_msgs/Joystick` | LaneState 메시지는 **없음**(신규 필요) |
| 베이스 노드 | `camera`, `joystick`, `monitor`, `battery`, `topst_utils` | 유지/검토 |

## 2. 목표 아키텍처

```
[오프라인 환경]  로컬 PC, venv, ROS 불필요
  주행영상(mp4/bag) ─▶ 인지코어 + 제어코어 조합 평가 ─▶ 최적 프로파일 YAML
                       (lane_preview / lane_compare / control_eval)          │
                                                                             ▼
[온라인 환경]  D3-G, ROS2 Humble                                    profiles/<track>.yaml
  camera ─▶ [인지 노드] ─(LaneState)─▶ [제어 노드] ─(/control)─▶ [액추에이터 노드] ─▶ 차량
                  │                         ▲                          (control, 베이스)
                  │                    joystick(engage/E-stop)
                  └───────────▶ [기록 노드] ◀── camera/LaneState/control
                               (mp4 + CSV + rosbag)
```

- **인지 노드**(신규 `perception` pkg, `perception_node`): camera → `driving_core` 인지 → `LaneState` 발행.
- **제어 노드**(신규 `driving` pkg, `driving_node`): `LaneState` → `driving_core` 제어 → `/control` 발행. 안전 게이트(engage/E-stop/conf) 포함.
- **액추에이터 노드**(기존 `control` pkg 유지): `/control` → PWM. 워치독 유지.
- **기록 노드**(신규 `recorder` pkg): 주행 로그 CSV + 영상 mp4 + rosbag 저장. joystick START/STOP 트리거.
- **오프라인**: `LaneState`/`Control` 코어를 ROS 없이 import해 영상에 대해 조합 평가.

### LaneState 메시지 (인지↔제어 계약, 신규 `lane_msgs`)
```
std_msgs/Header header
float32 center_error
float32 ema
float32 heading
string  heading_label
float32 confidence
float32 left_conf
float32 right_conf
float32 curvature
string  state          # OK / LOW_CONF / OUTLIER / HOLD / LOST
bool    used_fallback
bool    valid          # center_error 존재 여부
```

### 최적 프로파일 YAML (오프라인→온라인 유일 계약)
```yaml
# profiles/2025track.yaml  (오프라인 평가 산출물)
perception: {mode: O1, roi_top_frac: 0.45, orange_h_lo: 15, orange_h_hi: 38, ...}
control:    {controller: C2, kp: 0.5, kd: 0.1, steer_max: 0.8, throttle_base: 0.13, ...}
```
인지/제어 노드는 수십 개 ROS 파라미터 대신 이 프로파일 하나를 로드(점 3·4·11 구현).

## 3. 패키지 재편 계획

| 패키지 | 처리 | 비고 |
|---|---|---|
| `camera`, `joystick`, `topst_utils`, `config`, `control_msgs`, `joystick_msgs` | **유지** | 베이스, 사용 중 |
| `control` (액추에이터) | **유지** | PWM 드라이버·워치독. 개념상 "actuator" |
| `driving_core` | **신규** | 인지/제어 **공유 코어**(순수 파이썬, 단일 진실원). 온·오프라인 공용 |
| `perception` | **신규** | 인지 노드(`perception_node`), `driving_core` 인지 import |
| `driving` | **신규** | 제어 노드(`driving_node`), `driving_core` 제어 import → `/control` |
| `lane_msgs` | **신규** | LaneState 메시지 |
| `recorder` | **신규** | 통합 기록 노드(mp4+CSV+bag) |
| `opencv`(`opencv_node`) | **제거** | 단순 재발행 노드, `perception`이 대체 |
| `monitor` | **유지·경량화** | Launch 1 카메라 세팅 모니터링. 카메라+배터리+저장소 3종만 구독(제어/녹화/ROS그래프/OpenCV디버그 패널·구독 제거로 지연 최소화) |
| `battery` | **유지** | monitor 배터리 패널용 → `calibrate.launch`(Launch 1)에서 실행 |
| `data_acquisition.sh` | **유지 or 흡수** | START 버튼 bag 녹화용. 기록 노드로 흡수 가능 |
| `image_raw.jpg` | **제거 후보** | 샘플 이미지 |

## 4. 단계별 마이그레이션 계획

- [x] **P0** 워크트리·브랜치·본 문서 생성
- [x] **P1** `main` 위에 기존 구현 기능 삽입(베이스라인 확보)
- [x] **P2** 인지/제어 코어 **중복 제거** → 단일 공유 모듈(온·오프라인 공용)
- [x] **P3** `LaneState` 메시지 정의 + 인지/제어 노드 **분리**
- [x] **P4** 기록 노드 추출(mp4 + CSV; rosbag은 joystick_node 소유 유지)
- [x] **P5** 프로파일 YAML 배선(오프라인 산출 → 온라인 로드)
- [x] **P6** 미사용 패키지/노드 정리·경량화
- [x] **P7** launch 계층화(offline / online-manual / online-auto) + 문서화
- [ ] **P8** `offline/` 도구 재구성: 지각 2 + 제어 2 + 공용 1 대칭 구조로 분리, open-loop 지표·profile 핸드오프 배선 (설계 [offline/PIPELINE.md](offline/PIPELINE.md))
- 각 단계: macOS 정적검사(py_compile/flake8/코어 유닛테스트) → D3-G 빌드·실차검증 → 본 문서 로그 기록.

## 5. 결정 사항 (LOCKED, 2026-07-05)

1. **제어 로직 패키지 = `driving`**, 노드 = **`driving_node`** (`control`=액추에이터와 구분).
2. **공유 코어 = 단일 모듈** — `src/driving_core/`(ament_python 순수 파이썬 패키지)에 인지/제어 코어를 한 벌만 두고, 온라인 노드는 ROS import, 오프라인은 venv에 `pip install -e`(또는 PYTHONPATH)로 동일 코어 import. 복붙 2벌 제거.
3. **제거 = `opencv_node`만.** `monitor`는 초반 카메라 각도 세팅·모니터링용으로 **유지**. `data_acquisition.sh`는 주행 테스트 중 START 버튼 bag 녹화에 필요하면 **유지 또는 기록 노드로 흡수**.
4. **오프라인 도구 = 신규 최상위 `offline/`** 로 이동(온라인 `src/`와 물리 분리).
5. **monitor 경량화(트랙 테스트 지연 제거).** `monitor_node`는 카메라 이미지 + 배터리 + 저장공간 3종만 유지. 제어(control) 구독·녹화(joystick) 구독·ROS 그래프(`/api/graph`, `graph_utils.py`)·OpenCV 디버그 영상 3종(grayscale/blur/edge) 구독과 대응 웹 패널을 전부 제거. dead 설정키 `CONTROL_TOPIC`·`JOYSTICK_TOPIC`·`OPENCV_DEBUG_MODE`도 `vehicle_config.yaml`에서 삭제. 배터리 패널이 값을 받도록 `battery_node`를 `calibrate.launch.py`(monitor를 띄우는 유일한 런치)에 추가.
6. **legacy launch 정리.** 파이프라인 4개(calibrate/record_manual/online_manual/online_auto) 기준으로 `auto_driving.launch.py`(존재하지 않는 `inference` 패키지 참조·실행 불가) + `record_driving.launch.py`(rosbag `-a`, recorder_node mp4/csv 파이프라인과 무관·중복) 삭제. `manual_driving.launch.py`(벤더 docs가 최소 수동 런치로 참조)·`actuation_test.launch.py`(wheels-off 액추에이션 진단·전용 스크립트 동반)는 유지. 노드명은 이미 전부 일관(entry-point=모듈=내부명)이라 리네임 불필요.
7. **(2026-07-08) perception = 7-label BEV(`offline/lane7_probe.py`) 확정.** 온라인 BEV 통합은 실차 테스트 후로 연기(별도 BEV 코어 + 카메라 캘리브레이션 신설 예정, 그전까지 온라인은 front-view `lane_core` 유지). perception 탐색 도구 `perception_preview.py`/`perception_select.py`/`track_analyze.py` 제거. 제어 도구·`lane_core`·`control_core`·`_common`은 유지(§6 로그 2026-07-08).

## 7. 운용 가이드 (리팩토링 후)

### 오프라인 (로컬 PC, ROS 불필요)
```bash
# 최초 1회: 공유 코어 설치
.venv/bin/pip install -e D-Racer-Kit/src/driving_core
# 주행 영상으로 조합 비교 → 최적 (mode+params, controller+gains) 선정 (P8 이후 도구명)
cd offline
../.venv/bin/python perception_preview.py CLIP.mp4 --mode O1 --roi-top 0.45   # ① 3패널 확인
../.venv/bin/python perception_select.py CLIP.mp4 --modes M1,M2,O1,O2 \
    --export ../D-Racer-Kit/src/config/profiles/track2025.yaml                # ② [perception] export
../.venv/bin/python control_predict.py rslt/drive_XXXX.mp4 --csv rslt/drive_XXXX.csv \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml --controllers C1,C2,C4  # ③ 예측
../.venv/bin/python control_select.py rslt/pred_XXXX.csv \
    --export ../D-Racer-Kit/src/config/profiles/track2025.yaml                # ④ [control] export
# 최종 산출물 = 채워진 프로파일 D-Racer-Kit/src/config/profiles/<track>.yaml
```

### 온라인 (D3-G, ROS2)
```bash
cd D-Racer-Kit && colcon build && source install/setup.bash
# 수동 주행 + 인지 관찰 + 기록 (액추에이션 없음)
ros2 launch control online_manual.launch.py profile:=$PWD/src/config/profiles/track2025.yaml
# 자율 주행 (바퀴 들고 방향 확인 후 engage)
ros2 launch control online_auto.launch.py
ros2 param set /driving_node engage true     # wheels-off 확인 후에만
# 정지: joystick X(E-stop) / 기록: joystick START(mp4+csv+bag)
```

### 데이터 흐름
`camera → perception_node(/lane/state) → driving_node(/control, engage 게이트) → control_node(PWM)`.
`recorder_node`가 START마다 `drive_<ts>.mp4`+`.csv` 저장, bag은 joystick_node가 담당. 인지/제어 코어는 `driving_core` 한 벌을 온·오프라인이 공유.

## 6. 진행 로그

| 날짜 | 단계 | 내용 |
|---|---|---|
| 2026-07-09 | 리팩토링 P5d: 런치 재설계 | **공유 base + 리네이밍 + battery 전 런치 + calibration_mode 표준화.** `dracer_bringup`을 파이썬 모듈화(`launch_common.py`): `base_nodes(camera·actuator·joystick·monitor·battery)` + `vehicle_config_path`/`default_profile_path`/`default_record_dir`. 4개 런치가 base 호출 + 모드별 노드만 추가 → **base 4중 중복 제거**. **리네이밍**: `record_manual`→`record`, `online_manual`→`perceive`, `online_auto`→`drive`(calibrate 유지). **battery_node를 base에 넣어 전 런치 포함**(이전엔 calibrate에만 있어 online_* 웹 배터리 패널이 빈값이던 버그 해소). **calibration_mode 표준화**: 튜닝 런치(calibrate·perceive)만 True. 구성 — calibrate=base, record=+recorder(raw), perceive=+perception+recorder(debug뷰), drive=+perception+control+recorder(engage). 도크스트링 run 명령도 `dracer_bringup`·실경로로. 검증: launch_common+4런치 py_compile. ※ D3-G에서 `ros2 launch dracer_bringup perceive.launch.py` 재검증 필요(모듈 import·base 구성). |
| 2026-07-09 | 리팩토링 P5b: monitor 저지연 스트림 | **웹 영상 지연 해소** — 기존 JS 폴링(`/api/frame` 150ms)을 **MJPEG 푸시 스트림**(`/api/stream`, multipart/x-mixed-replace)으로 교체. flask_app_factory에 스트림 라우트(최신 프레임만, 변화 없으면 skip, ~50Hz cap) 추가, app.js가 `<img>`를 스트림에 직접 연결(실패 시 placeholder fallback). 새 의존성 0(순수 Flask). 검증: py_compile. ※ 실지연·브라우저 렌더는 D3-G+브라우저 확인 필요. 튜닝 슬라이더(P5c)는 후속. |
| 2026-07-09 | 리팩토링 P5a: 인지 live 파라미터 | **perception_node에 live 튜닝 추가**(P2 제어와 동일 패턴). `perception_core.Cfg`의 전 필드(name 제외)를 ROS param으로 선언(profile [perception]로 seed, 타입은 Cfg 기본값에 맞춰 coerce), `add_on_set_parameters_callback`로 `ros2 param set` 시 **Cfg+LanePipeline 재생성**(재기동 없이 즉시 반영). `_build_cfg`/`_on_set_params` 추가. 검증(macOS): py_compile + 전 필드 라운드트립(param dict→cfg_from_profile==Cfg()) + override 반영 확인. monitor 슬라이더(P5c)가 이 param을 set. |
| 2026-07-09 | 리팩토링 P4: 패키지·파일 리네이밍 + bringup | **역할에 맞춘 전면 리네이밍.** (P4a) `driving_core`→`dracer_core`, `lane_core.py`→`perception_core.py`. (P4b) `driving`(제어로직)→`control`(ControlNode·`control_node`), `control`(액추에이터)→`actuator`(ActuatorNode·`actuator_node`) — 'control' 이름 스왑이라 actuator 먼저 이름 비운 뒤 driving→control. (P4c) 런치 4개(calibrate·record_manual·online_manual·online_auto)를 **`dracer_bringup`** 패키지로 이동+`package=`/`executable=` 참조 rewrite(actuator/control 매핑), `manual_driving`·`actuation_test` 삭제, `offline/lane7_probe.py`→`perception_probe.py`. setup.py entry_points·setup.cfg·package.xml `<name>`·의존(`driving_core`→`dracer_core`)·docstring 일괄 갱신. 최종 12 패키지(actuator·battery·camera·control·dracer_bringup·dracer_core·dracer_msgs·joystick·monitor·perception·recorder·topst_utils). 검증(macOS): 전 노드+런치 py_compile 통과, 새 위치 실제 import 동작, 구 이름 잔존 소거, config `_find` 경로 불변. ※ colcon 빌드·노드 실행은 **D3-G clean 재빌드 필수**. |
| 2026-07-09 | 리팩토링 P3: msg 통합 | **커스텀 msg 4개 → `dracer_msgs` 1개 통합.** `lane_msgs`·`control_msgs`·`joystick_msgs`·`battery_msgs`를 `dracer_msgs`(LaneState·Control·Joystick·Battery)로 합침. Joystick.msg의 `control_msgs/Control`→same-package `Control`. 전 노드 import 13곳(`from X_msgs.msg`→`from dracer_msgs.msg`)·package.xml 의존(중복 제거)·docstring 토픽 타입 표기 갱신. 구 4개 패키지 삭제. LaneState.msg 주석도 현행화(heading_label 'ego', coast fallback). 검증(macOS): 전 노드 py_compile 통과 + 구 msg 참조 완전 소거. ※ rosidl 생성·노드 실행은 **D3-G clean 재빌드 필수**(msg 인터페이스 재생성). |
| 2026-07-09 | 리팩토링 P2: 제어 live 튜닝 + 트림 매핑 | **(1) 제어 파라미터 live 튜닝** — `driving_node`: profile [control]을 ROS param으로 push(단일 소스화) → `add_on_set_parameters_callback`로 `ros2 param set` 시 **컨트롤러·게인 재기동 없이 즉시 재빌드**(C1~C5 교체·kp/kd/steer_sign 등). `_apply_profile_params`/`_build_controller`/`_on_set_params` 추가, `_CTRL_FLOATS`/`_CTRL_PARAMS` 정의. **(2) 액추에이터 트림 매핑 수정** — `control_node.control_callback`: 자율 `/control`에 `+STEER_TRIM` 적용(clamp [-1,1]) → 명령 0=기계 직진(수동과 대칭). 이전엔 자율만 트림 미적용이라 **상시 STEER_TRIM(0.3)만큼 편향**되던 원인 해소. 검증(macOS): py_compile. ※ param 즉시반영·트림 실효는 rclpy 필요 → **D3-G wheels-off 검증 필수**(engage 후 /control 값·조향 방향). |
| 2026-07-09 | 리팩토링 P1: 인지 단일화 | **인지 실험 스캐폴딩 제거(인지=확정 파이프라인 하나).** `lane_core.py`: `PRESETS(G1~G6)`·`make_cfg` 모드 시스템·legacy-compat 죽은 필드(do_polyfit·curvature·per_lane_conf) 삭제 → 단일 `Cfg` + `cfg_from_profile(section)`(미지 키 무시, colors list→tuple). `perception_node.py`: `mode`+축별 오버라이드 param 20여 개 삭제 → profile [perception] 로드만. `_common.py`: 죽은 렌더/지표 함수 5개(three_panel·detection_tile·_draw_detection·perception_metrics·quality_score) 삭제 → IO 헬퍼만. `control_predict.py`: `make_cfg`→`cfg_from_profile`. 검증(macOS): py_compile 전부 통과 + Dashcam 070503 1029프레임 재실행 결과 **P1 이전과 동일**(OK 990/OUTLIER 39) → 동작 보존. 파일 리네임(perception_core.py 등)·live 튜닝은 후속 Phase. 설계 기준: [REFACTOR_DESIGN.md](REFACTOR_DESIGN.md). |
| 2026-07-09 | 인지 병합 개선 + engage 버튼 | **(1) 차선 병합을 창 IOU → 폴리라인 근접(MAX \|Δx\|<merge_dx=30)으로 교체** — 두꺼운/사선 한 선이 병렬 스택으로 쪼개지던 유령 분할 제거(같은-side 중복 71%→7%, 3-lane 604→43프레임). **MAX 기준**이라 Y분기(위 발산)·원근수렴쌍(아래 발산)처럼 한쪽 끝이라도 벌어지면 병합 거부 → 분기 보존. **(2) coast 외삽 클램프** — 단일차선 ego 중앙선에 소스 차선 y-span 부여, 관측 밖 포물선 발산 차단(heading/curvature/그리기 유효구간 한정). **(3) engage A버튼 토글** — `Joystick.msg`에 `bool engage` 추가, joystick_node가 A 눌림 edge로 토글·발행(X E-stop 시 강제 False), driving_node가 `engage = param OR joystick` OR결합(E-stop 항상 우선). 검증: Dashcam+Mission 31클립·27,438프레임 전수 — 크래시 0/NaN 0/극단값 0, **진짜 분기 41프레임 전부 오병합 0**(YL/YR 방향 라벨 구분 육안 확인), 총 상실률 1.2%. ※ 고정폭 coast 중앙값(평균 ~39% 프레임)은 근본해결을 카메라 캘리브레이션 BEV로 보류 — 실차에선 coast 구간 hold/저속 권장. engage는 msg 변경이라 D3-G 재빌드+wheels-off 검증 필요. |
| 2026-07-09 | 온라인 인지 lane7 이식 | **`offline/lane7_probe.py`의 7-label 로직을 온라인 `driving_core/lane_core.py`로 이식(기존 row-scan `LanePipeline` 교체).** detect(HSV 흰/노랑+morph+색게이트) → ROI 크롭 → sliding-window 다차선(방향 EMA 곡선추종) → 2차 polyfit → 7라벨(W-L/R·YS/YL/YR-L/R) → 창 IOU 중복병합 → heading+곡률 turn → pair-gate ego 통로 중앙선 → Tracker(L/R EMA·width coast). **BEV(사다리꼴→직사각형 워프)만 제외** — front-view 원본 마스크에서 동작(coeffs가 이미 이미지 좌표, 워프백 불필요). 카메라 캘리브레이션 BEV는 후속(`process()`에 seam 주석). 공개 API·`state` 계약 유지 → perception_node·컨트롤러·`control_predict`·`perception_metrics` 무손상. `make_cfg`는 미지 키 자동 필터(legacy 파라미터명 방어). `render_panels` front-view 4패널(입력+ROI\|W/Y mask\|sliding\|라벨+ego)로 교체(3패널 문제 해소). track2025 profile 주석 최신화. macOS 검증: py_compile + 합성 프레임 스모크(state 키·컨트롤러 step·검출 경로 center_error 부호). ※ 실차 검출 품질·front-view 파라미터(sw_margin/pair_gap/peak_sep) 재튜닝은 D3-G 빌드 후 확정 필요. |
| 2026-07-09 | 녹화 단일화 | **START 녹화를 recorder_node(mp4+csv) 단일 경로로 통일.** joystick_node의 START가 `data_acquisition.sh`(`ros2 bag record -a`)를 서브프로세스로 띄우던 로직 제거 — 이제 START는 `is_recording` 토글만 발행하고 recorder_node가 이를 미러링해 mp4+csv 기록. 배경: (1) bag 저장 불필요(팀 결정), (2) 2중 녹화가 빈 bag(카메라 미발행 시)·혼란 유발, (3) `start_new_session=True`로 분리 실행되던 bag 프로세스가 joystick 비정상 종료 시 **좀비로 잔존해 `/dev/video1`·토픽을 물고 다음 실행을 방해**할 위험 제거. joystick_node에서 `subprocess`/`signal` import·`data_acquisition_script` 파라미터·`start/stop/sync_recording` 메서드 삭제. recorder docstring·package.xml·online_manual docstring 정합. `data_acquisition.sh`는 파일만 남김(수동 실행 옵션, 미배선). 전체 py_compile 통과. ※ 벤더 문서 `docs/[8] Joystick & Control Package.md`의 START→bag 설명은 kit 원형 기준이라 미수정(현 팀 구현과 상이). |
| 2026-07-08 | perception 확정 | 차선 검출·인지·지각을 **7-label BEV 방식 `offline/lane7_probe.py`** 로 확정(BEV 원근제거 + sliding-window[방향 EMA 곡선추종] + 2차 polyfit, 7라벨 W-L/R·YS/YL/YR-L/R, 창 IOU 중복병합, heading+곡률 turn, pair-gate 중앙선, coast fallback, 6패널). **온라인 BEV 통합은 실차 테스트 완료 후로 연기** — 향후 별도 BEV 코어(`driving_core/bev_lane.py` 등) + 카메라 캘리브레이션 코드 신설 예정. 그 전까지 온라인 인지는 front-view `driving_core/lane_core.py` 유지(프로파일 perception 섹션도 front-view). perception 탐색 도구 3종(`perception_preview.py`·`perception_select.py`·`track_analyze.py`) **제거**. 제어 도구(`control_predict`·`control_select`)·`control_core`·`lane_core`(control_predict가 상태 생성에 사용)·`_common`·`driving`/`control` 노드는 **유지**(여러 컨트롤러 테스트 예정). |
| 2026-07-05 | P8(설계) | `offline/` 재구성 설계 확정(코드 전 문서화). 5파일 대칭 구조(`perception_preview`/`perception_select`/`control_predict`/`control_select`/`_common`), 데이터 흐름·profile 단일 in-place 핸드오프, open-loop 평가(covariate shift로 폐루프 궤적지표 불가 → 지각지표는 File2·명령품질지표는 File4로 분리). 신규 [offline/PIPELINE.md](offline/PIPELINE.md), `LANE_DETECTION.md`/`CONTROL_DESIGN.md` 갱신, profile/REFACTORING 문서 정합. 구현은 P8 후속. |
| 2026-07-05 | P0 | 워크트리 `SC2026(refactoring)` + 브랜치 `kos/track-test`(main 기준) 생성. 현재 구조 분석·목표 아키텍처·단계 계획 수립. 결정 4건 확정(§5). |
| 2026-07-05 | P1 | `kos/hw-cam-track-test`의 구현 기능 전체를 main 위에 삽입(베이스라인). 노랑 밴드 튜닝(15/38/70/90)·`lane_compare.py` 보존. 전체 py_compile 통과. (커밋 e409b51) |
| 2026-07-05 | P7 | launch 계층화: `online_manual.launch.py`(camera+control[joystick]+joystick+perception+recorder, 액추에이션 없음)와 `online_auto.launch.py`(+driving_node, engage 게이트·`ParameterValue(bool)`). 둘 다 `profile` 인자로 오프라인 프로파일 주입, `record_dir`·`bagfile` 리졸버. 운용 가이드(§7) 작성. 두 launch py_compile 통과. 리팩토링 P0~P7 완료. |
| 2026-07-05 | P6 | 경량화: `opencv` 패키지 전체 제거(`opencv_node`는 미사용, `lane_detect_node`/`lane_follow_node`는 P3/P4의 perception/driving/recorder로 대체됨). 이를 참조하던 `lane_detect_manual`/`lane_follow` launch도 제거(P7에서 신규 대체). 코어 docstring의 옛 `local_scripts`/`opencv` 참조 정리. offline 도구 import·실행 정상 확인. |
| 2026-07-05 | P5 | 프로파일 계약 배선: `driving_core/profile.py`(YAML 로더), 샘플 `config/profiles/track2025.yaml`(오프라인 선정 O1 튜닝밴드 + C2). `perception_node`/`driving_node`에 `profile` 파라미터 추가 — 설정 시 mode/params·controller/gains를 프로파일이 authoritative로 대체(점 4·11). end-to-end 검증: 프로파일→`make_cfg`/`make_ctrl` 정상 매핑(스키마 키 유효성 포함). |
| 2026-07-05 | P4 | 기록 노드 `recorder`(`recorder_node`) 신설: joystick START(`is_recording`) 미러링으로 START→STOP마다 `drive_<ts>.mp4` + `.csv`(LaneState + 자율 `/control` + 수동 joystick 명령 동기 기록). rosbag은 joystick_node가 `data_acquisition.sh`로 이미 소유 → 중복 방지 위해 recorder는 mp4+csv만 담당(그래서 `data_acquisition.sh` 유지 필요 확인). py_compile 통과. |
| 2026-07-05 | P3 | `lane_msgs/LaneState.msg`(인지↔제어 계약) 신설. 인지 노드 `perception`(`perception_node`: camera→`driving_core`→`/lane/state`+debug, 녹화 없음)과 제어 노드 `driving`(`driving_node`: `/lane/state`→컨트롤러→`/control`, engage/E-stop/conf 게이트·워치독 친화 발행)로 분리. 기존 `lane_detect_node`/`lane_follow_node`는 P6까지 병존. 두 노드 py_compile 통과(ROS 빌드는 D3-G). |
| 2026-07-05 | P2 | 공유 코어 패키지 `driving_core` 신설(`lane_core`+`control_core` git mv). `LanePipeline.process(debug=True)` 추가로 오프라인 패널을 단일 코어에서 렌더. `opencv` 노드 import→`driving_core`, package.xml 의존성 추가. 오프라인 도구를 최상위 `offline/`로 이동하며 인라인 중복 파이프라인 제거(`lane_preview`/`lane_compare` 재작성, 중복 `control_core` 삭제). `local_scripts/` 제거. 루트 `.gitignore`에 누락됐던 아티팩트 무시 규칙(`*.mp4`/`.venv`/`bagfile`/`offline/rslt`) 보강. venv에 `pip install -e driving_core` 후 오프라인 lane_compare end-to-end 검증(검출·렌더 정상). |
