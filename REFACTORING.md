> **이력 문서 (2026-07-09 이전).** 현재 상태를 반영하지 않는다 — 그 뒤로 metric BEV 도입,
> front-view 경로 삭제, 인지 정체성 추적(P2), 안전 워치독이 들어왔다.
> **현재 상태는 [ROLLBACK.md](ROLLBACK.md) 와 [PERCEPTION.md](PERCEPTION.md) 를 보라.**
> 이 파일은 "왜 그렇게 됐는가" 의 기록으로 남긴다.

# SC2026 리팩토링 — 설계 · 진행 기록

> 이 문서 = **현행 설계**(§0~부록 C) + **진행 로그**(P1~P6) + **초기 계획**(부록, superseded).
> 브랜치 `kos/track-test2`. P1~P6 구현·D3-G 검증 완료(남은 것: P5c 웹 슬라이더) — 상세는 아래 진행 로그.
> 전제: **인지 = 확정 단일 파이프라인**, **제어 = C1~C5 비교 유지**, 주행 노드 보존, ROS 적극 단순화, 파라미터 저지연 실시간 튜닝.

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
- `Task command.md`를 이 형태로 전면 갱신, 낡은 문서(§7) 정리.

---

## 7. 문서 정리 (동반)

- **신설 `PERCEPTION.md`**(repo 루트, 추적됨) — 차선 인지 파트 하나를 서술: **흐름**(detect→ROI→sliding-window→분류→ego center→tracker→state), **구성**(perception_core 모듈·파라미터), **기법**(HSV 색 마스크·MAX 폴리라인 병합·7라벨 분류·coast·클램프). "7-label/lane7" 같은 브랜딩 없이 "인지 파트"로 기술. 스택 stale한 `offline/LANE_DETECTION.md`는 이걸로 대체(삭제).
- **신설/갱신 제어 문서** — C1~C5 비교·open-loop 평가는 `offline/CONTROL_DESIGN.md` 유지·갱신(제어는 계속 비교 실험 대상).
- **갱신**: `Task command.md`(파이프라인·A-engage·병합 monitor·tuner), `offline/PIPELINE.md`(현행 인지+제어 비교 흐름).
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
├─ 본 문서 설계                 (본 문서, 루트=추적)
├─ PERCEPTION.md                      (신설: 인지 파트 흐름·구성·기법)
├─ Task command.md
├─ REFACTORING.md · CLAUDE.md · README.md
└─ Env/                               (⚠ .gitignore로 미추적 — 이미지 등 로컬 전용)
```
(삭제: `D3G_VERIFY.md`, `offline/LANE_DETECTION.md`, `data_acquisition.sh`. `Env/전체 구조.md`는 미추적이라 로컬 rm)

---

## 6. 진행 로그

| 날짜 | 단계 | 내용 |
|---|---|---|
| 2026-07-09 | 리팩토링 P6: 문서 정리 | **현행화 + orphan 제거.** 신설 [PERCEPTION.md](PERCEPTION.md)(인지 파트 흐름·구성·기법·한계, "lane7/7-label" 브랜딩 없이). [Task command.md](Task%20command.md) 전면 재작성(dracer_bringup·calibrate/record/perceive/drive·A-engage·live 튜닝·monitor). offline/PIPELINE.md 현행 재작성(perception_probe·control_predict/select·profile 계약), CONTROL_DESIGN.md 낡은 "BEV 확정/실차 후" 문구 정리. D-Racer-Kit/README.md 패키지표·트리를 12패키지로 갱신(구 msg 3개·opencv 제거, 신규 반영). **삭제**: `D3G_VERIFY.md`·`offline/LANE_DETECTION.md`·`Track test pipeline.md`(command.md로 통합)·`Env/전체 구조.md`(gitignore). 본 문서 설계 런치명·pipeline.md 참조 갱신. vendor `docs/[1]~[9]`는 보존. |
| 2026-07-09 | 리팩토링 P5d: 런치 재설계 | **공유 base + 리네이밍 + battery 전 런치 + calibration_mode 표준화.** `dracer_bringup`을 파이썬 모듈화(`launch_common.py`): `base_nodes(camera·actuator·joystick·monitor·battery)` + `vehicle_config_path`/`default_profile_path`/`default_record_dir`. 4개 런치가 base 호출 + 모드별 노드만 추가 → **base 4중 중복 제거**. **리네이밍**: `record_manual`→`record`, `online_manual`→`perceive`, `online_auto`→`drive`(calibrate 유지). **battery_node를 base에 넣어 전 런치 포함**(이전엔 calibrate에만 있어 online_* 웹 배터리 패널이 빈값이던 버그 해소). **calibration_mode 표준화**: 튜닝 런치(calibrate·perceive)만 True. 구성 — calibrate=base, record=+recorder(raw), perceive=+perception+recorder(debug뷰), drive=+perception+control+recorder(engage). 도크스트링 run 명령도 `dracer_bringup`·실경로로. 검증: launch_common+4런치 py_compile. ※ D3-G에서 `ros2 launch dracer_bringup perceive.launch.py` 재검증 필요(모듈 import·base 구성). |
| 2026-07-09 | 리팩토링 P5b: monitor 저지연 스트림 | **웹 영상 지연 해소** — 기존 JS 폴링(`/api/frame` 150ms)을 **MJPEG 푸시 스트림**(`/api/stream`, multipart/x-mixed-replace)으로 교체. flask_app_factory에 스트림 라우트(최신 프레임만, 변화 없으면 skip, ~50Hz cap) 추가, app.js가 `<img>`를 스트림에 직접 연결(실패 시 placeholder fallback). 새 의존성 0(순수 Flask). 검증: py_compile. ※ 실지연·브라우저 렌더는 D3-G+브라우저 확인 필요. 튜닝 슬라이더(P5c)는 후속. |
| 2026-07-09 | 리팩토링 P5a: 인지 live 파라미터 | **perception_node에 live 튜닝 추가**(P2 제어와 동일 패턴). `perception_core.Cfg`의 전 필드(name 제외)를 ROS param으로 선언(profile [perception]로 seed, 타입은 Cfg 기본값에 맞춰 coerce), `add_on_set_parameters_callback`로 `ros2 param set` 시 **Cfg+LanePipeline 재생성**(재기동 없이 즉시 반영). `_build_cfg`/`_on_set_params` 추가. 검증(macOS): py_compile + 전 필드 라운드트립(param dict→cfg_from_profile==Cfg()) + override 반영 확인. monitor 슬라이더(P5c)가 이 param을 set. |
| 2026-07-09 | 리팩토링 P4: 패키지·파일 리네이밍 + bringup | **역할에 맞춘 전면 리네이밍.** (P4a) `driving_core`→`dracer_core`, `lane_core.py`→`perception_core.py`. (P4b) `driving`(제어로직)→`control`(ControlNode·`control_node`), `control`(액추에이터)→`actuator`(ActuatorNode·`actuator_node`) — 'control' 이름 스왑이라 actuator 먼저 이름 비운 뒤 driving→control. (P4c) 런치 4개(calibrate·record_manual·online_manual·online_auto)를 **`dracer_bringup`** 패키지로 이동+`package=`/`executable=` 참조 rewrite(actuator/control 매핑), `manual_driving`·`actuation_test` 삭제, `offline/lane7_probe.py`→`perception_probe.py`. setup.py entry_points·setup.cfg·package.xml `<name>`·의존(`driving_core`→`dracer_core`)·docstring 일괄 갱신. 최종 12 패키지(actuator·battery·camera·control·dracer_bringup·dracer_core·dracer_msgs·joystick·monitor·perception·recorder·topst_utils). 검증(macOS): 전 노드+런치 py_compile 통과, 새 위치 실제 import 동작, 구 이름 잔존 소거, config `_find` 경로 불변. ※ colcon 빌드·노드 실행은 **D3-G clean 재빌드 필수**. |
| 2026-07-09 | 리팩토링 P3: msg 통합 | **커스텀 msg 4개 → `dracer_msgs` 1개 통합.** `lane_msgs`·`control_msgs`·`joystick_msgs`·`battery_msgs`를 `dracer_msgs`(LaneState·Control·Joystick·Battery)로 합침. Joystick.msg의 `control_msgs/Control`→same-package `Control`. 전 노드 import 13곳(`from X_msgs.msg`→`from dracer_msgs.msg`)·package.xml 의존(중복 제거)·docstring 토픽 타입 표기 갱신. 구 4개 패키지 삭제. LaneState.msg 주석도 현행화(heading_label 'ego', coast fallback). 검증(macOS): 전 노드 py_compile 통과 + 구 msg 참조 완전 소거. ※ rosidl 생성·노드 실행은 **D3-G clean 재빌드 필수**(msg 인터페이스 재생성). |
| 2026-07-09 | 리팩토링 P2: 제어 live 튜닝 + 트림 매핑 | **(1) 제어 파라미터 live 튜닝** — `driving_node`: profile [control]을 ROS param으로 push(단일 소스화) → `add_on_set_parameters_callback`로 `ros2 param set` 시 **컨트롤러·게인 재기동 없이 즉시 재빌드**(C1~C5 교체·kp/kd/steer_sign 등). `_apply_profile_params`/`_build_controller`/`_on_set_params` 추가, `_CTRL_FLOATS`/`_CTRL_PARAMS` 정의. **(2) 액추에이터 트림 매핑 수정** — `control_node.control_callback`: 자율 `/control`에 `+STEER_TRIM` 적용(clamp [-1,1]) → 명령 0=기계 직진(수동과 대칭). 이전엔 자율만 트림 미적용이라 **상시 STEER_TRIM(0.3)만큼 편향**되던 원인 해소. 검증(macOS): py_compile. ※ param 즉시반영·트림 실효는 rclpy 필요 → **D3-G wheels-off 검증 필수**(engage 후 /control 값·조향 방향). |
| 2026-07-09 | 리팩토링 P1: 인지 단일화 | **인지 실험 스캐폴딩 제거(인지=확정 파이프라인 하나).** `lane_core.py`: `PRESETS(G1~G6)`·`make_cfg` 모드 시스템·legacy-compat 죽은 필드(do_polyfit·curvature·per_lane_conf) 삭제 → 단일 `Cfg` + `cfg_from_profile(section)`(미지 키 무시, colors list→tuple). `perception_node.py`: `mode`+축별 오버라이드 param 20여 개 삭제 → profile [perception] 로드만. `_common.py`: 죽은 렌더/지표 함수 5개(three_panel·detection_tile·_draw_detection·perception_metrics·quality_score) 삭제 → IO 헬퍼만. `control_predict.py`: `make_cfg`→`cfg_from_profile`. 검증(macOS): py_compile 전부 통과 + Dashcam 070503 1029프레임 재실행 결과 **P1 이전과 동일**(OK 990/OUTLIER 39) → 동작 보존. 파일 리네임(perception_core.py 등)·live 튜닝은 후속 Phase. 설계 기준: 본 문서(설계부). |
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

---

## 부록: 초기 리팩토링 계획 (superseded, 참고용)

> 아래는 리팩토링 초기(2026-07-05, `kos/track-test`)의 계획으로, 위 설계로 대체되었다. 기록 보존용.

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

