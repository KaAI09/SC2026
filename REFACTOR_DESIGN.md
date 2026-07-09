# SC2026 리팩토링 설계 (트랙 테스트 최적화)

> 상태: **설계 확정 대기** (구현 전 리뷰용). 확정 후 이 문서를 실행 기준으로 삼는다.
> 전제: **인지 = lane7 확정/고정**, **제어 = 다요소 비교 실험 유지**, **주행 노드(joystick·monitor·camera·actuator·battery) 형태 보존**, **ROS 적극 단순화**, **파라미터 저지연 실시간 튜닝**.

---

## 0. 목표와 원칙

1. 인지는 lane7 하나로 고정 — G1~G6 실험 조합·축별 오버라이드 전부 제거.
2. 제어는 C1~C5 비교와 게인 튜닝을 계속 지원 — live 교체 가능.
3. 주행에 필요한 노드는 인터페이스·동작 보존.
4. 패키지·토픽·런치를 역할에 맞게 재편(이름 혼란 해소, msg 통합).
5. 트랙에서 **재빌드·재기동 없이** 인지/제어 파라미터를 조정하고 **저지연으로 결과를 눈으로** 본다.

---

## 1. 목표 아키텍처

### 1.0 확정 네이밍표 (전면 리네임 — "lane7"/역할혼란 제거)

**패키지**
| 현재 | 최종 |
|---|---|
| `driving` (제어 로직) | **`control`** |
| `control` (액추에이터) | **`actuator`** |
| `driving_core` | **`dracer_core`** |
| `lane_msgs`·`control_msgs`·`joystick_msgs`·`battery_msgs` | **`dracer_msgs`** (통합) |
| (런치 모음) | **`dracer_bringup`** |
| `monitor` | **`monitor`** (확장: 튜너 흡수 — 상태+라이브뷰+튜닝 통합 웹 노드) |
| camera·perception·joystick·recorder·battery·topst_utils | 유지 |

**노드**
| 현재 | 최종 |
|---|---|
| `driving_node` | **`control_node`** (control 패키지) |
| `control_node` (액추에이터) | **`actuator_node`** (actuator 패키지) |
| `monitor_node` | `monitor_node` (확장: 튜닝 흡수) |
| perception/joystick/recorder/camera/battery_node | 유지 |

**파일**
| 현재 | 최종 |
|---|---|
| `driving_core/lane_core.py` | **`dracer_core/perception_core.py`** (control_core와 대칭) |
| `driving_core/control_core.py` | `dracer_core/control_core.py` |
| `driving_core/profile.py` | `dracer_core/profile.py` |
| `offline/lane7_probe.py` | **`offline/perception_probe.py`** |
| `offline/control_predict.py`·`control_select.py`·`_common.py` | 유지 |
| `data_acquisition.sh` | **삭제** |

**"lane7" 문자열**
| 위치 | 최종 |
|---|---|
| LaneState `heading_label` 값 `'lane7'` | **`'ego'`** (ego 중앙선 접선 기반) |
| 주석·docstring의 고유명 "lane7" | **"sliding-window 7-label lane perception"** (사실 서술) |
| profile 주석 "lane7 front-view" | "sliding-window 7-label (front-view)" |
| REFACTORING.md 과거 로그 | 역사 기록 유지, 신규 항목부터 새 네이밍 |

**유지(churn 회피)**: 토픽 이름 전부(`/camera/image/compressed`·`/lane/state`·`/lane/debug/compressed`·`/control`·`/joystick`·`/battery_status`), `topst_utils`, `config`.

### 1.1 패키지 재편 결과 (14 → 12, 역할 명확화)

msg 4개 통합 + driving↔control 리네이밍 + bringup/tuner 신설. `data_acquisition.sh` 삭제.

### 1.2 노드·토픽 그래프 (변경 후)

```
camera_node ── /camera/image/compressed ──┬─► perception_node(lane7) ─► /lane/state ─────┐
                                          │                         └► /lane/debug/compressed ─┐
                                          └─► recorder_node (record 시)                        │
joystick_node ─ /joystick ─┬─► control_node(제어로직) ─ /control ─► actuator_node ─► 서보      │
                           └─► actuator_node(수동/estop/engage)                                │
battery_node ─ /battery_status ──────────────────────────────────────────────┐               │
monitor_node (병합 웹 노드, :5000) ◄── /lane/debug/compressed ────────────────┴───────────────┘
   ├─ 배터리·저장공간·라이브뷰 표시 (WebSocket 저지연, 최신 프레임만)
   └─ 웹 슬라이더 ─(param client)─► perception_node / control_node 로 live 파라미터 set + profile 저장
```

토픽 이름은 **전부 유지**(LaneState/Control/Joystick/Battery의 소속 패키지만 `dracer_msgs`로 바뀜).

---

## 2. 인지 = lane7 고정

### 2.1 `dracer_core/perception_core.py` (구 `lane_core.py`)
- **삭제**: `PRESETS(G1~G6)`, `mode` 개념, `make_cfg`의 모드 분기, legacy-compat 죽은 필드(`do_polyfit`·`curvature`(필드)·`per_lane_conf`).
- **유지/정리**: 단일 `Cfg`(sliding-window 7-label 파라미터) + `cfg_from_profile(section: dict) -> Cfg`(defaults에 override 적용, 미지 키 무시). detect→ROI→sliding-window(MAX 병합)→7-label→ego center→Tracker→state 파이프라인 그대로.
- 공개 API: `LanePipeline`, `Cfg`, `cfg_from_profile`, `render_panels`. (`PRESETS`/`make_cfg` 제거)

### 2.2 `perception/perception_node.py`
- **삭제**: `mode` + 축별 오버라이드 param 20여 개(`split_ref`·`heading_method`·`min_aspect`·`min_length`·`dynamic_roi`·`min_contour_area`·`morph_kernel` 등).
- **유지**: subscribe/state/debug 토픽, `profile`, `debug_scale`, `jpeg_quality`, `log_hz`, `publish_debug`.
- **추가(live)**: lane7 튜닝 파라미터를 ROS param으로 선언 + `add_on_set_parameters_callback` → 변경 시 `Cfg` 재구성·`LanePipeline` 재생성(§5).
  - 튜닝 대상: `roi_top_frac`, `trap_top_w`, `trap_bot_w`, `white_s_max`, `white_v_min`, `yellow_h_lo/hi`, `yellow_s_min`, `yellow_v_min`, `morph_v`, `color_gate`, `sw_nwin`, `sw_margin`, `sw_minpix`, `sw_max_miss`, `sw_min_span`, `merge_dx`, `pair_gap_min`, `pair_overlap_min`, `lane_width_default`, `ema_alpha`, `conf_low`, `lost_reset`.
- 기동 시 profile [perception] 로드 → 위 param 기본값 세팅.

### 2.3 오프라인 도구(`offline/`)
- `lane7_probe.py` → **`perception_probe.py`**: BEV 실험/튜닝 시각화 1개만 유지(향후 캘리브레이션 BEV 개발 기준).
- `_common.py`: 죽은 함수 5개(`three_panel`·`detection_tile`·`_draw_detection`·`perception_metrics`·`quality_score`) 삭제 → IO 헬퍼만.
- `control_predict.py`/`control_select.py`: **유지**(제어 알고리즘 open-loop 비교의 핵심). 코어 API 변경(make_cfg 제거, `dracer_core.perception_core`)에 맞춰 `cfg_from_profile` 사용으로 수정.

---

## 3. 제어 = 유연 + live 튜닝

### 3.1 `driving_core/control_core.py`
- **유지**: C1(P)·C2(PD)·C3(PID)·C4(PurePursuit)·C5(Stanley) + `CtrlCfg`. 비교 실험의 대상이므로 손대지 않음.

### 3.2 `control/control_node.py` (구 `driving_node.py`)
- **유지**: engage(param OR joystick A) + E-stop + watchdog 안전 계층.
- **추가(live)**: `CtrlCfg` 전 파라미터(`controller`, `kp`, `kd`, `ki`, `center_target`, `steer_max`, `steer_sign`, `slew_rate`, `out_ema`, `throttle_base`, `throttle_min`, `curv_slow`, `conf_gate`)를 ROS param + `add_on_set_parameters_callback` → **컨트롤러 live 교체(C1↔C5)·게인 즉시 반영**.

### 3.3 액추에이터 트림 매핑 (별도 확정 이슈)
- 자율 `/control`에도 `+STEER_TRIM` 적용(수동과 대칭) → 명령 0 = 기계 직진. (지난 분석의 상시 편향 원인) — 이번 리팩토링에 포함.

---

## 4. ROS 구조 재편 (적극)

### 4.1 msg 통합 → `dracer_msgs`
- `LaneState`·`Control`·`Joystick`·`BatteryStatus`를 `dracer_msgs/msg/`로 이동.
- 전 노드 import 변경: `from lane_msgs.msg import LaneState` → `from dracer_msgs.msg import LaneState` (약 8개 노드 + driving_core 없음).
- `package.xml`/`CMakeLists.txt` depends 갱신. **전체 clean 재빌드 필요.**

### 4.2 패키지 리네이밍
- `driving` → `control` (제어 로직), `control` → `actuator` (서보), `driving_core` → `dracer_core`.
- 영향: 각 `setup.py`/`package.xml` entry_points·이름, 런치의 `package=`/`executable=`, 전 노드 `from dracer_core...` import, 문서. **§1.0 매핑표로 일괄 치환.**

### 4.3 런치 → `dracer_bringup`
- 확정 파이프라인만 유지: `calibrate` · `record` · `perceive` · `drive`.
- `manual_driving`·`actuation_test` → **삭제**.
- 각 런치에 `tuner:=true` 옵션(선택 기동).

---

## 5. 실시간 튜닝 (병합 monitor — 저지연 웹, 원격 접근)

### 5.1 제약과 판단
- 운용은 **D3-G에 SSH 접속** → GUI는 **원격에서 접근 가능**해야 함.
- native `cv2.imshow`는 D3-G에서 `ssh -X`(X11) 필요 → 영상 지연 더 큼, 또는 로컬 ROS2 실행(별 프로세스·"병합" 위배).
- **웹은 브라우저만으로 원격 접근**(D3-G_IP:5000 또는 SSH 터널) → SSH 운용에 최적.
- 현 웹 지연의 원인은 **MJPEG multipart 버퍼링**이지 웹 자체가 아님 → 전송 방식 교체로 해결.
- **결론: 웹을 유지·저지연화하고 monitor에 튜닝을 병합.** (별도 `tuner` 패키지 폐기)

### 5.2 병합 `monitor_node` (상태 + 라이브뷰 + 튜닝 통합 웹)
- **저지연 영상**: `/lane/debug/compressed` 구독 → **WebSocket 바이너리 푸시**, 서버 큐 없이 **항상 최신 프레임만**(stale 드롭), 해상도/JPEG 품질/fps 조절 파라미터. (현 MJPEG multipart 대체)
- **상태**: 배터리(`/battery_status`) + 저장공간(poll) — 기존 기능 보존.
- **튜닝 슬라이더**: 웹 UI의 range 입력 → monitor가 보유한 **ROS2 async param client**로 `perception_node`/`control_node`에 param set → §2.2·§3.2 콜백 즉시 반영. 컨트롤러 C1~C5 셀렉트 포함.
- **profile 저장 버튼**: `save_profile`(std_srvs/Trigger, §5.3) 호출.
- 한 페이지에서 라이브뷰 + 상태 + 슬라이더 → 조정 결과 실시간 확인.

### 5.3 profile ↔ live 왕복(영속화)
- `perception_node`·`control_node`에 `save_profile`(std_srvs/Trigger) 서비스 → 현재 param을 profile YAML 해당 섹션에 기록(D3-G 로컬).
- monitor 웹의 저장 버튼이 두 서비스를 호출 → 트랙에서 맞춘 값을 그대로 저장.

### 5.4 대안(선택)
- 절대 최저지연이 필요하고 로컬에 디스플레이+ROS2가 있으면 `rqt_image_view`+`rqt_reconfigure`(DDS 직수신) 병행 가능. 기본 경로는 병합 웹.

---

## 6. 트랙 테스트 파이프라인 재구축

`dracer_bringup` 런치(확정 형태 반영: lane7·A-engage·recorder mp4+csv·tuner):

| 런치 | 구성 노드 | 용도 |
|---|---|---|
| `calibrate` | camera·actuator·joystick(calib)·monitor | 카메라 각도 + STEER_TRIM/ACCEL_RATIO |
| `record` | camera·actuator·joystick·recorder(raw) | 오프라인용 원본 영상 |
| `perceive` | camera·perception·actuator(joystick)·joystick·recorder·(tuner) | 인지 검증 + 데이터 수집 + live 튜닝 |
| `drive` | camera·perception·control·actuator·joystick·recorder·(tuner) | 자율주행(engage A) + live 튜닝 |

- 조작: START=녹화, X=E-stop, A=engage, tuner 슬라이더=인지/제어 튜닝, `s`=profile 저장.
- `Track test command.md`를 이 형태로 전면 갱신, 낡은 문서(§7) 정리.

---

## 7. 문서 정리 (동반)

- **신설 `PERCEPTION.md`**(repo 루트, 추적됨) — 차선 인지 파트 하나를 서술: **흐름**(detect→ROI→sliding-window→분류→ego center→tracker→state), **구성**(perception_core 모듈·파라미터), **기법**(HSV 색 마스크·MAX 폴리라인 병합·7라벨 분류·coast·클램프). "7-label/lane7" 같은 브랜딩 없이 "인지 파트"로 기술. 스택 stale한 `offline/LANE_DETECTION.md`는 이걸로 대체(삭제).
- **신설/갱신 제어 문서** — C1~C5 비교·open-loop 평가는 `offline/CONTROL_DESIGN.md` 유지·갱신(제어는 계속 비교 실험 대상).
- **갱신**: `Track test command.md`(파이프라인·A-engage·병합 monitor·tuner), `offline/PIPELINE.md`(현행 인지+제어 비교 흐름).
- **삭제**: orphan(`D3G_VERIFY.md`, `Env/전체 구조.md`), 대체된 `offline/LANE_DETECTION.md`. (git 이력엔 남음)
- **보존**: vendor `D-Racer-Kit/docs/[1]~[9]`. `[8]`의 START→bag 불일치는 주석 1줄만.
- `REFACTORING.md`: 진행 로그 유지.

---

## 8. 실행 계획 (내부 순서 · 검증 · 리스크)

> 설계 확정 후 구현. 리팩토링 규모가 커서 **한 브랜치에서 순서대로** 진행하고 각 묶음마다 macOS 정적/오프라인 검증, 최종 D3-G clean 재빌드.

| 순서 | 작업 | macOS 검증 | D3-G |
|---|---|---|---|
| 1 | 인지 lane7 고정(§2) + `_common` 정리 | py_compile + 합성/오프라인 스모크 | — |
| 2 | 제어 live param(§3) + 트림 매핑 | py_compile | 재빌드·param set 즉시반영 |
| 3 | msg 통합 `dracer_msgs`(§4.1) | py_compile(import 경로) | **clean 재빌드 필수** |
| 4 | 패키지 리네이밍(§4.2) + `dracer_bringup`(§4.3) | 런치 파싱 | clean 재빌드 |
| 5 | monitor 병합 확장(§5): 저지연 웹 + 슬라이더 + save_profile | 웹 로컬 확인 | D3-G:5000 원격 접근·live set |
| 6 | 파이프라인·문서(§6·§7) | 문서 검토 | 실차 검증 |

**리스크**
- msg 통합·패키지 리네이밍은 **전 노드 import/빌드 파일**을 건드려 clean 재빌드 필수 → 한 번에 일괄, 매핑 표로 누락 방지.
- macOS는 ROS 빌드 불가 → 3·4·5는 **D3-G 검증이 실제 확인 지점**.
- tuner는 디스플레이·네트워크(DDS 디스커버리) 필요 → Control PC 환경 확인.

**롤백**: 각 순서 독립 커밋 → 문제 시 해당 커밋만 되돌림.

---

## 9. 확정된 결정 (구현 착수 준비 완료)

- [x] 패키지/노드 전면 리네임: `driving`→`control`, `control`→`actuator`, `driving_core`→`dracer_core`, msg 4→`dracer_msgs`
- [x] 파일: `lane_core.py`→`perception_core.py`, `lane7_probe.py`→`perception_probe.py`
- [x] "lane7" 문자열 제거: `heading_label` `'lane7'`→`'ego'`, 주석은 서술형으로
- [x] `manual_driving`·`actuation_test` 런치 → **삭제**
- [x] orphan 문서(`D3G_VERIFY.md`·`Env/전체 구조.md`) → **삭제**
- [x] 토픽 이름·`topst_utils`·`config` 유지
- [x] 튜너 → **monitor 노드에 병합**(별도 tuner 패키지 폐기), 저지연 **WebSocket 웹**으로 원격(SSH) 접근
- [x] 인지 브랜딩 "7-label/lane7" 제거 → "인지 파트", 기법은 `PERCEPTION.md`에 서술
- [!] `Env/`는 `.gitignore`의 `env/`(대소문자 무시)로 추적 안 됨 → 설계/인지 문서는 **repo 루트**에 둔다(추적). 팀 아키텍처 문서를 버전관리하려면 `.gitignore` 조정 별도 결정 필요
- [ ] monitor 웹 슬라이더 노출 파라미터 최종 목록 — 구현 시 §2.2·§3.2에서 선별(1차 제안: 인지 roi_top_frac·yellow bands·sw_margin·merge_dx·pair_gap_min / 제어 controller·kp·kd·steer_sign·steer_max·throttle_base·conf_gate)

**→ 설계 확정. 구현 착수 가능.**

---

## 부록 A. 최종 패키지 (14 → 12)

| # | 패키지 | 종류 | 노드/내용 |
|---|---|---|---|
| 1 | `camera` | py | camera_node |
| 2 | `perception` | py | perception_node (인지, dracer_core.perception_core 사용) |
| 3 | `control` | py | control_node (제어 로직, 구 driving) |
| 4 | `actuator` | py | actuator_node (서보 드라이버, 구 control) |
| 5 | `joystick` | py | joystick_node |
| 6 | `monitor` | py | monitor_node (상태+라이브뷰+튜닝 웹, tuner 흡수) |
| 7 | `recorder` | py | recorder_node (mp4+csv) |
| 8 | `battery` | py | battery_node |
| 9 | `dracer_core` | py | 공유 로직: perception_core·control_core·profile |
| 10 | `dracer_msgs` | cmake | LaneState·Control·Joystick·Battery (4개 msg 통합) |
| 11 | `dracer_bringup` | py | 런치 4개 (calibrate·record·perceive·drive) |
| 12 | `topst_utils` | py | vendor 게임패드 라이브러리 |

- `config/`(vehicle_config·profiles)는 `D-Racer-Kit/src/config/` 그대로 유지(노드 `_find` 경로 불변).
- 삭제: `data_acquisition.sh`, `manual_driving`/`actuation_test` 런치.

## 부록 B. 노드 · 통신 구조

```
                        /camera/image/compressed (CompressedImage)
 camera_node ───────────┬──────────────────────────► perception_node
                        ├───────────────────────────► monitor_node (라이브뷰 대체 가능)
                        └───────────────────────────► recorder_node

 perception_node ─ /lane/state (LaneState) ─────────┬► control_node
                  ─ /lane/debug/compressed ─────────┼► monitor_node (저지연 웹뷰)
                                                     └► recorder_node

 joystick_node ─ /joystick (Joystick: control_msg·e_stop_en·engage·is_recording)
        ├──► control_node   (engage·estop)
        ├──► actuator_node  (수동 조향·estop)
        └──► recorder_node  (is_recording)

 control_node ─ /control (Control) ──┬► actuator_node ──► 서보/스로틀
                                     └► recorder_node

 battery_node ─ /battery_status (Battery) ──► monitor_node
```

**서비스 · 파라미터 (튜닝)**
```
 monitor_node (웹 슬라이더 / 저장 버튼)
   ├─(param client set)─► perception_node : 인지 파라미터 live 반영 (on_set 콜백)
   ├─(param client set)─► control_node    : 제어 파라미터·컨트롤러 live 교체
   ├─(Trigger)─────────► perception_node/save_profile : 현재값 → profile[perception]
   └─(Trigger)─────────► control_node/save_profile    : 현재값 → profile[control]
```

**토픽 요약**
| 토픽 | 타입(dracer_msgs 외 표기) | pub | sub |
|---|---|---|---|
| `/camera/image/compressed` | sensor_msgs/CompressedImage | camera | perception·monitor·recorder |
| `/lane/state` | LaneState | perception | control·recorder |
| `/lane/debug/compressed` | sensor_msgs/CompressedImage | perception | monitor·recorder |
| `/control` | Control | control | actuator·recorder |
| `/joystick` | Joystick | joystick | control·actuator·recorder |
| `/battery_status` | Battery | battery | monitor |

## 부록 C. 디렉토리 · 파일 구조

```
SC2026/
├─ D-Racer-Kit/
│  ├─ src/
│  │  ├─ dracer_msgs/                 [cmake]
│  │  │  ├─ msg/{LaneState,Control,Joystick,Battery}.msg
│  │  │  └─ CMakeLists.txt · package.xml
│  │  ├─ dracer_core/                 [py] 공유 로직
│  │  │  ├─ dracer_core/{perception_core,control_core,profile,__init__}.py
│  │  │  └─ setup.py · package.xml
│  │  ├─ dracer_bringup/              [py] 런치
│  │  │  ├─ launch/{calibrate,record,perceive,drive}.launch.py
│  │  │  └─ setup.py · package.xml
│  │  ├─ config/                      (유지)
│  │  │  ├─ vehicle_config.yaml
│  │  │  └─ profiles/track2025.yaml
│  │  ├─ camera/      camera/camera_node.py
│  │  ├─ perception/  perception/perception_node.py
│  │  ├─ control/     control/control_node.py          (구 driving)
│  │  ├─ actuator/    actuator/actuator_node.py         (구 control)
│  │  ├─ joystick/    joystick/joystick_node.py
│  │  ├─ monitor/     monitor/{monitor_node,flask_app_factory,image_utils,monitor_state}.py + web/
│  │  ├─ recorder/    recorder/recorder_node.py
│  │  ├─ battery/     battery/battery_node.py
│  │  └─ topst_utils/ topst_utils/gamepads.py           (vendor)
│  ├─ docs/           [1]~[9] vendor 레퍼런스 (보존)
│  └─ README.md
├─ offline/                           오프라인 도구
│  ├─ perception_probe.py             (구 lane7_probe — BEV 실험/시각화)
│  ├─ control_predict.py · control_select.py
│  ├─ _common.py                      (IO 헬퍼만, 죽은 함수 제거)
│  ├─ PIPELINE.md · CONTROL_DESIGN.md
│  └─ Dashcam(2025 Track)/ · rslt/
├─ REFACTOR_DESIGN.md                 (본 문서, 루트=추적)
├─ PERCEPTION.md                      (신설: 인지 파트 흐름·구성·기법)
├─ Track test command.md
├─ REFACTORING.md · CLAUDE.md · README.md
└─ Env/                               (⚠ .gitignore로 미추적 — 이미지 등 로컬 전용)
```
(삭제: `D3G_VERIFY.md`, `offline/LANE_DETECTION.md`, `data_acquisition.sh`. `Env/전체 구조.md`는 미추적이라 로컬 rm)
