# Track Test Command — 전과정 명령어 (온/오프라인 구분)

> [Track test pipeline.md](Track%20test%20pipeline.md)의 12단계를 실제 명령으로. 각 단계의 **실행 위치**를 구분한다.
> - 🖥 **로컬(offline)** — macOS, ROS 불필요, 레포 `.venv` 사용
> - 🚗 **D3-G(online)** — 차량, ROS2 Humble, `colcon`
>
> **안전(액추에이션 단계 공통)**: 바퀴 지면에서 띄우고(wheels-off) 먼저 → 조이스틱 **X = E-stop** 상시 → 저속 → D3-G에서 코드 수정·commit 금지.

---

## 0. 실차 테스트 준비 체크리스트 (pre-flight)

**동기화·빌드**
- [ ] D3-G가 **`kos/track-test2`** 로 pull + `colcon build --symlink-install` + `source install/setup.bash` (A-2, A-3)
- [ ] `git log --oneline -1` 이 원격 tip(`0c03eac` 이상)과 일치
- [ ] profile 경로 확인: `ls $WS/src/config/profiles/track2025.yaml`

**하드웨어·안전**
- [ ] 배터리 충전 상태 확인(웹 모니터 `:5000` 또는 `ros2 topic echo /battery_status`)
- [ ] 조이스틱 페어링 + **X(E-stop) 즉시 정지 동작 확인** (수동 주행으로 먼저 검증)
- [ ] 즉시 정지 가능한 위치·여유 공간 확보, 차량 붙잡을 사람 대기

**단계 순서(무부하 우선)**
- [ ] Step 1-2 `calibrate` — 카메라 각도/높이 + STEER_TRIM/ACCEL_RATIO 저장
- [ ] Step 7 `online_manual` — **바퀴 들고** `/lane/state` 정상(center_error/confidence) 확인, 오버레이 육안 확인
- [ ] Step 11 `online_auto` — `engage=false`로 기동 → `/control` 방향 확인 → **바퀴 들고 `engage true`로 조향 방향 검증** → 반대면 `profile control.steer_sign=-1` → 저속 트랙

**주의**
- online_auto는 `engage:=true` 일 때만 `/control` 발행. 노드 죽으면 액추에이터 watchdog(0.5s)이 중립 정지.
- 저신뢰/차선 상실(state LOST/HOLD·conf<gate) 시 driving_node가 스로틀 0.

---

## A. 환경 준비

### A-1. 🖥 로컬 venv (최초 1회)
```bash
cd <레포 루트>                      # 예: ~/workspace/SC2026
.venv/bin/pip install -e D-Racer-Kit/src/driving_core
# iCloud(~/Documents) 아래면 .pth가 hidden 처리돼 editable import가 깨질 수 있음 → 심볼릭 링크:
ln -sfn "$(pwd)/D-Racer-Kit/src/driving_core/driving_core" \
        .venv/lib/python3.13/site-packages/driving_core
.venv/bin/python -c "from driving_core.lane_core import PRESETS; print(list(PRESETS))"  # G1..G6
```

### A-2. 🚗 D3-G — 로컬 전부 무시 + 미추적 삭제 + 원격으로 동기화
> 실차 테스트 활성 브랜치는 **`kos/track-test2`** (perception 7-label BEV 확정 + 최신 문서). `kos/track-test`는 보존용 이전 스냅샷.
```bash
export WS=~/SC2026/D-Racer-Kit          # 실제 경로로 조정
cd "$WS"
git fetch origin
git checkout kos/track-test2
git reset --hard origin/kos/track-test2  # ① 추적 파일의 로컬 수정 전부 폐기
git clean -fdx                           # ② 미추적 파일/폴더 전부 삭제 (build/install/bagfile 포함)
git log --oneline -3                      # 원격 HEAD(0c03eac 이상)와 일치 확인
```
> ⚠ **되돌릴 수 없음**:
> - `reset --hard` → `vehicle_config.yaml`이 repo 버전으로 복원 → **STEER_TRIM/ACCEL_RATIO 초기화** → **Launch 1 재캘리브레이션 필요**.
> - `clean -fdx` → `bagfile/`의 녹화(mp4/csv)·`build/`·`install/` 삭제. **필요한 녹화는 먼저 로컬로 백업(scp, ↓D절)**.
> - 미추적만 지우고 build/install은 남기려면 `-x` 빼고 `git clean -fd`.

### A-3. 🚗 D3-G 빌드
```bash
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
> 새 터미널마다: `cd "$WS" && source install/setup.bash`

---

## B. 파이프라인 단계별 명령

### Step 1-2 · 🚗 calibrate — 카메라 세팅 + trim/accel 저장 (Launch 1)
```bash
ros2 launch control calibrate.launch.py
```
- 웹 모니터 `http://<D3-G_IP>:5000` → 카메라 실시간 보며 각도/높이 조절. (경량 모니터: 카메라+배터리+저장공간만 표시, battery_node 포함)
- 조이스틱: **Y/B**=steering_trim −/+, **L1/R1**=accel_ratio −/+ (조정 즉시 `vehicle_config.yaml` 저장), **X**=E-stop.
- 저장/로드 확인:
  ```bash
  grep -E "STEER_TRIM|ACCEL_RATIO" "$WS"/src/config/vehicle_config.yaml   # 즉시 반영 확인
  # 런치 재실행 시 joystick 로그: loaded ACCEL_RATIO=... / steering_trim 복원
  ```

### Step 3 · 🚗 record_manual — 원본 카메라 영상 저장 (Launch 2)
```bash
ros2 launch control record_manual.launch.py record_dir:=$HOME/bagfile
# record_dir 미지정 시 기본값은 repo의 D-Racer-Kit/bagfile/ 로 잡힐 수 있어 명시 권장
```
- 조이스틱 **START**로 녹화 시작/정지 (원본 `/camera/image/compressed`).
```bash
ls -lt $HOME/bagfile/raw_*.mp4 $HOME/bagfile/raw_*.csv | head
```

### Step 4-6 · 🖥 perception — 7-label BEV 확정 (`lane7_probe.py`)
> **확정 (2026-07-08)**: 차선 검출·인지·지각은 **7-label BEV 방식 `offline/lane7_probe.py`** 로 확정.
> 기존 front-view 탐색 도구 `track_analyze.py`·`perception_preview.py`·`perception_select.py`는 **제거됨**.
> **온라인 BEV 통합은 실차 테스트 후로 연기**(별도 BEV 코어 + 카메라 캘리브레이션 신설 예정) → 지금 profile `[perception]`은 front-view baseline 유지, 자동 export 없음.
```bash
cd offline
../.venv/bin/python lane7_probe.py <영상>.mp4     # 7-label BEV 검출·인지 + 6패널 시각화(독립)
```

### Step 7 · 🚗 online_manual — 지각 + 수동 + 기록 (Launch 3)
```bash
ros2 launch control online_manual.launch.py \
    profile:=$WS/src/config/profiles/track2025.yaml record_dir:=$HOME/bagfile
```
확인(새 터미널):
```bash
ros2 topic hz   /lane/state
ros2 topic echo /lane/state --once          # center_error/ema/heading/confidence
# 오버레이 라이브 뷰(선택): ros2 run monitor monitor_node --ros-args -p image_topic:=/lane/debug/compressed
```
- **START** 녹화 → `drive_*.mp4`(**다패널 디버그 영상**: 입력+ROI | mask | 검출+상태) + `.csv`(LaneState + 수동 command 동기). recorder가 `/lane/debug/compressed`(perception 다패널)를 저장 — 원본 카메라가 아님.
  > 현재는 **front-view 3패널**. BEV 6패널은 실차 후 BEV 통합 시 적용.
  ```bash
  ls -lt $HOME/bagfile/drive_*.mp4 $HOME/bagfile/drive_*.csv | head
  ```

### Step 8 · 🖥 control_predict — 컨트롤러 명령 open-loop 예측
```bash
cd offline
../.venv/bin/python control_predict.py <drive>.mp4 --csv <drive>.csv \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml \
    --controllers C1,C2,C3,C4,C5
# 출력: rslt/pred_<drive>.csv
```

### Step 9 · 🖥 control_select — 제어 지표 랭킹 + (선택)export
```bash
cd offline
../.venv/bin/python control_select.py rslt/pred_<drive>.csv
../.venv/bin/python control_select.py rslt/pred_<drive>.csv --export C2 \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml
```
- 완성된 profile(`[perception]`+`[control]`)을 커밋·푸시 → D3-G에서 pull.

### Step 10 · 🚗 profile 적용 — 온라인 노드가 로드
```bash
cd "$WS" && git pull origin kos/track-test2 && colcon build --symlink-install && source install/setup.bash
# perception_node/driving_node가 profile:= 인자로 로드 (Step 7/11 런치에서 지정)
```

### Step 11 · 🚗 online_auto — 차선검출 + 자율주행 + 기록 (⚠ 액추에이션)
```bash
ros2 launch control online_auto.launch.py \
    profile:=$WS/src/config/profiles/track2025.yaml record_dir:=$HOME/bagfile   # engage=false로 시작
```
안전 절차(새 터미널):
```bash
ros2 topic hz /lane/state                     # 지각 정상?
ros2 topic echo /control                       # 방향 확인 (engage 전엔 미발행/중립)
# ↓ 바퀴 띄운 상태 확인 후에만
ros2 param set /driving_node engage true       # 구동 시작
ros2 param set /driving_node engage false      # 정지 (또는 조이스틱 X)
```
- 방향 반대면 profile `control.steer_sign`을 -1로. **START**로 자율 기록 저장(Step 12 데이터).

### Step 12 · 🖥/🚗 파라미터 보정 — **보류(TODO)**
- 자율 기록(제어로그+검출) 기반 setpoint·게인 재피팅. **실주행 데이터 확보 후 설계·구현.**

---

## C. 조이스틱 · 토픽 참조

| 버튼 | 기능 |
|---|---|
| Y / B | steering_trim −/+ (calibration_mode) |
| L1 / R1 | accel_ratio −/+ |
| START | 녹화 시작/정지 (mp4+csv) |
| X | E-stop (구동 즉시 정지) |

| 토픽 | 용도 |
|---|---|
| `/camera/image/compressed` | 원본 카메라 |
| `/lane/state` | 지각 상태(center_error/ema/heading/confidence) |
| `/lane/debug/compressed` | 다패널 디버그(입력+ROI\|mask\|검출) — recorder가 저장 |
| `/control` | 제어 명령(throttle/steering) |
| `/joystick` | 조이스틱 입력 |
| `/battery_status` | 배터리 |

---

## D. 산출물 이동 (D3-G ↔ 로컬)

```bash
# D3-G 녹화 → 로컬 (오프라인 분석용)   [로컬에서 실행]
scp topst@<D3-G_IP>:~/bagfile/'drive_*.{mp4,csv}' ./offline/rslt/
scp topst@<D3-G_IP>:~/bagfile/'raw_*.mp4'          ./offline/rslt/

# 완성 profile 로컬 → D3-G                [git 경유]
#   로컬: git add config/profiles/<track>.yaml && git commit && git push
#   D3-G: git pull origin kos/track-test2
```
> profile YAML은 git 추적 대상 → git으로 전달. 녹화(mp4/csv)는 미추적 → scp로 전달.

---

## 요약 흐름
```
🚗 calibrate(1-2) → 🚗 record_manual(3)
   → 🖥 lane7_probe(4-6, 7-label BEV 확정; profile[perception]은 front-view baseline 유지, BEV 통합은 실차 후)
🚗 online_manual(7) → 🖥 control_predict(8) → 🖥 control_select(9) ═▶ profile[control]
🚗 (pull+build,10) → 🚗 online_auto engage(11) → 🖥/🚗 보정(12, 보류)
```
