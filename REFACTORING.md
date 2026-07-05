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
| `monitor` | **유지** | 초반 카메라 각도 세팅·모니터링용 |
| `battery` | **유지** | `auto_driving.launch`에서 사용 |
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
| 2026-07-05 | P8(설계) | `offline/` 재구성 설계 확정(코드 전 문서화). 5파일 대칭 구조(`perception_preview`/`perception_select`/`control_predict`/`control_select`/`_common`), 데이터 흐름·profile 단일 in-place 핸드오프, open-loop 평가(covariate shift로 폐루프 궤적지표 불가 → 지각지표는 File2·명령품질지표는 File4로 분리). 신규 [offline/PIPELINE.md](offline/PIPELINE.md), `LANE_DETECTION.md`/`CONTROL_DESIGN.md` 갱신, profile/REFACTORING 문서 정합. 구현은 P8 후속. |
| 2026-07-05 | P0 | 워크트리 `SC2026(refactoring)` + 브랜치 `kos/track-test`(main 기준) 생성. 현재 구조 분석·목표 아키텍처·단계 계획 수립. 결정 4건 확정(§5). |
| 2026-07-05 | P1 | `kos/hw-cam-track-test`의 구현 기능 전체를 main 위에 삽입(베이스라인). 노랑 밴드 튜닝(15/38/70/90)·`lane_compare.py` 보존. 전체 py_compile 통과. (커밋 e409b51) |
| 2026-07-05 | P7 | launch 계층화: `online_manual.launch.py`(camera+control[joystick]+joystick+perception+recorder, 액추에이션 없음)와 `online_auto.launch.py`(+driving_node, engage 게이트·`ParameterValue(bool)`). 둘 다 `profile` 인자로 오프라인 프로파일 주입, `record_dir`·`bagfile` 리졸버. 운용 가이드(§7) 작성. 두 launch py_compile 통과. 리팩토링 P0~P7 완료. |
| 2026-07-05 | P6 | 경량화: `opencv` 패키지 전체 제거(`opencv_node`는 미사용, `lane_detect_node`/`lane_follow_node`는 P3/P4의 perception/driving/recorder로 대체됨). 이를 참조하던 `lane_detect_manual`/`lane_follow` launch도 제거(P7에서 신규 대체). 코어 docstring의 옛 `local_scripts`/`opencv` 참조 정리. offline 도구 import·실행 정상 확인. |
| 2026-07-05 | P5 | 프로파일 계약 배선: `driving_core/profile.py`(YAML 로더), 샘플 `config/profiles/track2025.yaml`(오프라인 선정 O1 튜닝밴드 + C2). `perception_node`/`driving_node`에 `profile` 파라미터 추가 — 설정 시 mode/params·controller/gains를 프로파일이 authoritative로 대체(점 4·11). end-to-end 검증: 프로파일→`make_cfg`/`make_ctrl` 정상 매핑(스키마 키 유효성 포함). |
| 2026-07-05 | P4 | 기록 노드 `recorder`(`recorder_node`) 신설: joystick START(`is_recording`) 미러링으로 START→STOP마다 `drive_<ts>.mp4` + `.csv`(LaneState + 자율 `/control` + 수동 joystick 명령 동기 기록). rosbag은 joystick_node가 `data_acquisition.sh`로 이미 소유 → 중복 방지 위해 recorder는 mp4+csv만 담당(그래서 `data_acquisition.sh` 유지 필요 확인). py_compile 통과. |
| 2026-07-05 | P3 | `lane_msgs/LaneState.msg`(인지↔제어 계약) 신설. 인지 노드 `perception`(`perception_node`: camera→`driving_core`→`/lane/state`+debug, 녹화 없음)과 제어 노드 `driving`(`driving_node`: `/lane/state`→컨트롤러→`/control`, engage/E-stop/conf 게이트·워치독 친화 발행)로 분리. 기존 `lane_detect_node`/`lane_follow_node`는 P6까지 병존. 두 노드 py_compile 통과(ROS 빌드는 D3-G). |
| 2026-07-05 | P2 | 공유 코어 패키지 `driving_core` 신설(`lane_core`+`control_core` git mv). `LanePipeline.process(debug=True)` 추가로 오프라인 패널을 단일 코어에서 렌더. `opencv` 노드 import→`driving_core`, package.xml 의존성 추가. 오프라인 도구를 최상위 `offline/`로 이동하며 인라인 중복 파이프라인 제거(`lane_preview`/`lane_compare` 재작성, 중복 `control_core` 삭제). `local_scripts/` 제거. 루트 `.gitignore`에 누락됐던 아티팩트 무시 규칙(`*.mp4`/`.venv`/`bagfile`/`offline/rslt`) 보강. venv에 `pip install -e driving_core` 후 오프라인 lane_compare end-to-end 검증(검출·렌더 정상). |
