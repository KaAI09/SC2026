# Track Test — 운영 명령 가이드

실차 트랙 테스트의 전 과정. 실행 위치를 구분한다.
- 🖥 **로컬(offline)** — macOS, ROS 불필요, 레포 `.venv`
- 🚗 **D3-G(online)** — 차량, ROS2 Humble, `colcon`

**안전(액추에이션 공통)**: 바퀴 지면에서 띄우고(wheels-off) 먼저 → 조이스틱 **X = E-stop** 상시 → 저속 → D3-G에서 코드 수정·commit 금지.

---

## 파이프라인 개요

```
🚗 calibrate ─► 🚗 record ─► 🖥 perception_probe(오프라인 실험)
                    │
🚗 perceive(인지검증+튜닝) ─► 🖥 control_predict/select(제어 비교) ═▶ profile[control]
🚗 drive(자율, engage) ◄─────────────────────────────────────────┘
```

| 런치 | 구성 | 용도 |
|---|---|---|
| `calibrate` | base | 카메라 각도 + STEER_TRIM/ACCEL_RATIO 저장 |
| `record` | base + recorder(raw) | 오프라인용 원본 영상 수집 |
| `perceive` | base + perception + recorder | 인지 검증 + 데이터 수집 + **live 튜닝** |
| `drive` | base + perception + control + recorder | 자율주행(engage) |

> **base** = camera · actuator · joystick · **monitor(웹 :5000)** · **battery**. 전 런치 공통.
> 온라인 튜닝: 실행 중 `ros2 param set /perception_node …` / `/control_node …` → 즉시 반영, monitor에서 결과 확인.

---

## A. 환경 준비

### A-1. 🖥 로컬 venv (최초 1회)
```bash
cd ~/workspace/SC2026
.venv/bin/pip install -e D-Racer-Kit/src/dracer_core
.venv/bin/python -c "from dracer_core.perception_core import Cfg; print(Cfg().name)"
```

### A-2. 🚗 D3-G — 원격 동기화 + 빌드
```bash
export WS=~/SC2026/D-Racer-Kit
cd "$WS"
git fetch origin && git checkout kos/track-test2
git reset --hard origin/kos/track-test2    # ⚠ vehicle_config 복원 → 재캘리브레이션 필요
git clean -fdx                              # ⚠ build/install/bagfile(mp4·csv) 삭제 — 필요분 먼저 scp(↓D)
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```
> 새 터미널마다: `cd "$WS" && source install/setup.bash`

---

## B. 런치별 명령 (🚗 D3-G)

공통: profile 인자를 생략하면 런치가 기본값(`src/config/profiles/track2025.yaml`)을 자동으로 찾는다.

### B-1. calibrate — 카메라 세팅 + trim/accel 저장
```bash
ros2 launch dracer_bringup calibrate.launch.py
```
- 웹 `http://<D3-G_IP>:5000` → 카메라 실시간 보며 각도/높이 조절.
- 조이스틱: **Y/B**=steering_trim −/+, **L1/R1**=accel_ratio −/+ (즉시 `vehicle_config.yaml` 저장), **X**=E-stop.
```bash
grep -E "STEER_TRIM|ACCEL_RATIO" "$WS"/src/config/vehicle_config.yaml   # 반영 확인
```

### B-2. record — 원본 영상 수집
```bash
ros2 launch dracer_bringup record.launch.py record_dir:=$HOME/bagfile
```
- 조이스틱 **START**로 녹화 시작/정지 → 원본 `raw_*.mp4` (`/camera/image/compressed`).
```bash
ls -lt $HOME/bagfile/raw_*.mp4 | head
```

### B-3. perceive — 인지 검증 + 데이터 수집 + live 튜닝
```bash
ros2 launch dracer_bringup perceive.launch.py     # profile 생략 시 기본값
```
확인·튜닝(새 터미널):
```bash
ros2 topic echo /lane/state --once                # center_error/heading/confidence
# 웹 :5000 → /lane/debug/compressed 4패널 저지연 스트림
ros2 param set /perception_node merge_dx 40.0      # 즉시 반영, 뷰에서 확인
ros2 param set /perception_node roi_top_frac 0.25
```
- **START** 녹화 → `drive_*.mp4`(4패널 디버그) + `.csv`(LaneState + 수동 command 동기).

### B-4. drive — 자율주행 (⚠ 액추에이션)
```bash
ros2 launch dracer_bringup drive.launch.py         # engage=false로 시작
```
안전 절차(새 터미널):
```bash
ros2 topic echo /control                            # 방향 확인 (engage 전엔 중립)
# ↓ 바퀴 띄운 상태 확인 후에만
ros2 param set /control_node engage true            # 또는 조이스틱 A 버튼
ros2 param set /control_node engage false           # 정지 (또는 조이스틱 X = E-stop)
# 제어 live 튜닝
ros2 param set /control_node kp 0.7
ros2 param set /control_node controller C3          # C1~C5 교체
```
- **engage** = `ros2 param set … engage` **또는 조이스틱 A 버튼**(토글). **X(E-stop)**이 항상 우선.
- 조향 방향 반대면 `ros2 param set /control_node steer_sign -1.0`.

---

## C. 오프라인 (🖥 로컬)

### C-1. perception_probe — BEV 실험/시각화
```bash
cd offline
../.venv/bin/python perception_probe.py <raw>.mp4 --stages --name t1
```

### C-2. control_predict / control_select — 제어 알고리즘 비교
```bash
cd offline
../.venv/bin/python control_predict.py <drive>.mp4 --csv <drive>.csv \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml --controllers C1,C2,C3,C4,C5
../.venv/bin/python control_select.py rslt/pred_<drive>.csv --export C2 \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml
```
- 완성된 profile(`[perception]`+`[control]`)을 커밋·푸시 → D3-G에서 pull.

---

## D. 조이스틱 · 토픽 참조

| 버튼 | 기능 |
|---|---|
| Y / B | steering_trim −/+ (calibration_mode) |
| L1 / R1 | accel_ratio −/+ |
| START | 녹화 시작/정지 (mp4+csv) |
| **A** | **engage 토글 (자율 구동)** |
| X | E-stop (구동 즉시 정지, engage 강제 해제) |

| 토픽 | 용도 |
|---|---|
| `/camera/image/compressed` | 원본 카메라 |
| `/lane/state` | 지각 상태(center_error/ema/heading/confidence) |
| `/lane/debug/compressed` | 다패널 디버그 — monitor가 저지연 스트림 |
| `/control` | 제어 명령(steering/throttle) |
| `/joystick` | 조이스틱(control_msg·e_stop_en·engage·is_recording) |
| `/battery_status` | 배터리 |

메시지·노드는 `dracer_msgs`·`dracer_core`. 인지 상세는 [PERCEPTION.md](PERCEPTION.md).

---

## E. 산출물 이동 (D3-G ↔ 로컬)

```bash
# D3-G 녹화 → 로컬 (오프라인 분석)   [로컬에서 실행]
scp topst@<D3-G_IP>:~/bagfile/'raw_*.mp4'           ./offline/rslt/
scp topst@<D3-G_IP>:~/bagfile/'drive_*.{mp4,csv}'   ./offline/rslt/

# 완성 profile 로컬 → D3-G   [git 경유]
#   로컬: git add ... && git commit && git push
#   D3-G: git pull origin kos/track-test2
```
> profile YAML은 git 추적 → git으로. 녹화(mp4/csv)는 미추적 → scp.
