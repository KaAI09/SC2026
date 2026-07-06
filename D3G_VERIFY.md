# D3-G 검증 명령 세트 (Track Test Pipeline 1~11단계)

> 로컬(macOS)에서 정적 검증(컴파일·profile·import·심볼)은 통과. 여기서는 **D3-G 실차**에서
> 각 런치를 빌드·실행하고 확인하는 절차. [Track test pipeline.md](Track%20test%20pipeline.md) 매핑을 따른다.
>
> **안전 원칙(모든 단계 공통)**
> - 액추에이션 단계는 **바퀴를 지면에서 띄운 상태(wheels-off)** 로 먼저.
> - 조이스틱 **X = E-stop** 상시 대기. 즉시 정지 가능 위치 확보.
> - 스로틀 0·보수적 조향으로 시작. 트랙 주행은 무부하 검증 통과 후.
> - D3-G에서는 **코드 수정·커밋 금지**(pull·build·run만).

경로 예시는 `~/SC2026/D-Racer-Kit` 기준. 실제 경로에 맞게 `WS`만 조정.

---

## 0. 브랜치 pull + 빌드 + source

```bash
export WS=~/SC2026/D-Racer-Kit           # 실제 경로로 조정
cd "$WS"
git fetch origin && git checkout kos/track-test && git pull origin kos/track-test
git log --oneline -3                     # 9ccbf9b(monitor 경량화·legacy launch 정리) 이상인지 확인

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
빌드 실패 시 정리 후 재빌드:
```bash
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```

## 1. 스모크 체크 (노드·토픽 살아있는지)

새 터미널마다 먼저: `cd "$WS" && source install/setup.bash`
```bash
ros2 node list                           # 실행한 런치의 노드가 보이는지
ros2 topic list                          # /camera/image/compressed, /control, /joystick 등
ros2 topic hz /camera/image/compressed   # 카메라 프레임레이트(~30Hz 기대)
ros2 topic echo /joystick --once         # 조이스틱 입력 들어오는지
ros2 topic echo /battery_status --once    # 배터리 상태
```

---

## 2. Launch 1 — calibrate (Step 1-2, 세팅·캘리브레이션)

**목적**: 카메라 각도/높이 조절 + steering_trim·accel_ratio 튜닝·저장. **액추에이션은 수동 조이스틱만.**
```bash
ros2 launch control calibrate.launch.py
```
확인:
- `ros2 node list` → **camera / control / joystick / monitor / battery** 5개 노드 (battery_node 신규 포함).
- 웹 모니터 접속(monitor_node, `http://<D3-G_IP>:5000`): 브라우저에서 카메라 실시간 피드 → **카메라 각도/높이 물리 조절**.
  - **경량 모니터 회귀 확인**: 웹에 **카메라 + 배터리 + 저장공간 3패널만** 표시(제어/녹화/ROS그래프/OpenCV디버그 패널 제거됨). 지연이 줄었는지 체감 확인.
  - **배터리 패널 동작**: battery_node 포함으로 배터리 값이 WAITING이 아닌 실제 값 → `ros2 topic echo /battery_status --once`.
- 조이스틱 버튼 동작 (control_node 로그 관찰):
  - **Y / B** → `steering_trim` 감소/증가 (로그 `steering_trim updated to ...`)
  - **L1 / R1** → `accel_ratio` 감소/증가 (로그 `accel_ratio decreased/increased to ...`)
- **저장 검증 (핵심 신규 기능)**: 조정 후 vehicle_config.yaml에 즉시 반영되는지
  ```bash
  grep -E "STEER_TRIM|ACCEL_RATIO" "$WS/src/config/vehicle_config.yaml"
  ```
- **로드 검증**: 런치 종료 후 재실행 → joystick_node 로그에 `loaded ACCEL_RATIO=... from vehicle config`,
  steering_trim이 저장값으로 복원되는지.

⚠ 바퀴는 wheels-off로 두고 조향 방향·트림 반응만 확인 권장.

---

## 3. Launch 2 — record_manual (Step 3, 원본 영상 확보)

**목적**: 고정된 카메라로 수동 주행하며 **원본 카메라 영상** 저장(지각 없음 → 오프라인 분석용).
```bash
ros2 launch control record_manual.launch.py
# 저장 경로 지정 시: ros2 launch control record_manual.launch.py record_dir:=$HOME/bagfile
```
확인:
- `ros2 node list` → camera/control/joystick/recorder (perception 없음 확인).
- **START 버튼**으로 녹화 시작 → 잠깐 주행 → **START** 다시 눌러 정지.
- 파일 생성 확인(원본 프레임):
  ```bash
  ls -lt $HOME/bagfile/raw_*.mp4 $HOME/bagfile/raw_*.csv | head
  ```
- recorder 로그: `image=/camera/image/compressed`(원본) 인지 확인.

---

## 4. Launch 3 — online_manual (Step 7, 지각+수동+기록)

**목적**: 지각 검출(예측+오버레이) + 수동 주행 + 로그 동시 저장. **자율 액추에이션 없음.**
```bash
ros2 launch control online_manual.launch.py \
    profile:=$WS/src/config/profiles/track2025.yaml
```
확인:
- `ros2 topic hz /lane/state` → 지각 상태 발행(카메라 레이트 근처).
- `ros2 topic echo /lane/state --once` → center_error/ema/heading/confidence 값 유효한지.
- perception_node 로그: `loaded profile ...track2025.yaml`, mode **G5**(colors white+yellow) 적용.
- 웹 모니터/`/lane/debug/compressed`로 검출 오버레이 육안 확인.
- **START** 녹화 → `drive_<ts>.mp4`(+csv, LaneState+수동명령 동기) 생성:
  ```bash
  ls -lt $HOME/bagfile/drive_*.mp4 $HOME/bagfile/drive_*.csv | head
  head -2 $HOME/bagfile/drive_*.csv        # 헤더 + 첫 행(center_error, manual_steering ...)
  ```
→ 이 csv가 **control_predict**(오프라인 Step 8) 입력.

---

## 5. Launch 자율 — online_auto (Step 11, 차선검출+자율주행+기록)

⚠ **액추에이션 단계. 반드시 wheels-off → 방향확인 → engage → 저속 트랙 순서.**
```bash
ros2 launch control online_auto.launch.py \
    profile:=$WS/src/config/profiles/track2025.yaml
# 시작 시 engage=false (구동 안 함)
```
**절차 (안전 게이트)**:
1. **바퀴를 지면에서 띄운 상태**로 시작.
2. 지각 정상 확인: `ros2 topic hz /lane/state`, 오버레이 육안.
3. 조향 방향 확인: `ros2 topic echo /control` — 차선 오차 부호와 조향 방향이 맞는지(반대면 profile `steer_sign` 조정).
4. **engage 켜기 (구동 시작)** — wheels-off 확인 후에만:
   ```bash
   ros2 param set /driving_node engage true
   ```
   바퀴 회전·조향이 의도대로면 → 바닥 내려 **저속 트랙 주행**.
5. **정지**: 조이스틱 **X (E-stop)** / 또는 `ros2 param set /driving_node engage false`.
6. **START**로 자율 주행 기록(제어로그+검출) 저장 → Step 12(보정) 데이터.

확인:
```bash
ros2 topic echo /control --once          # engage 전엔 중립/미발행, 후엔 명령
ros2 param get /driving_node engage
ls -lt $HOME/bagfile/drive_*.mp4 | head
```

---

## 6. 단계별 산출물 → 오프라인 연결 요약

| D3-G 산출물 | 다음(오프라인, 로컬) |
|---|---|
| `raw_*.mp4` (Launch 2) | `track_analyze.py` → 밴드/ROI/차선폭 (Step 4) |
| track 영상 | `perception_preview/select` → 검출 선정 → profile [perception] (Step 5-6) |
| `drive_*.mp4 + .csv` (Launch 3) | `control_predict/select` → 제어 선정 → profile [control] (Step 8-9) |
| 자율 `drive_*` (Launch auto) | **Step 12 보정(보류)** |

## 7. 문제 시 체크리스트
- 노드 안 뜸 → `ros2 node list`, 런치 콘솔 에러(패키지 빌드 여부, `source install/setup.bash`).
- 카메라 없음 → `ros2 topic hz /camera/image/compressed`, `vehicle_config.yaml`의 `USB_CAM_DEVICE`.
- 지각 값 이상 → profile 밴드/ROI가 이 카메라와 안 맞음 → `track_analyze.py`로 재측정(2026은 필수).
- 구동 안 함(auto) → `engage` 파라미터, E-stop 래치 상태(X 눌렀는지), conf_gate 미달.
- 트림/가속 저장 안 됨 → vehicle_config.yaml 쓰기 권한, `calibration_mode:=true` 여부.
