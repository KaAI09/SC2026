# Lap Command — lap 런치 운영 가이드

랩타임 측정(`lap.launch.py`) 중심의 명령 모음. 브랜치 `kos/out-course`.

- ⚠ **lap 은 액추에이션이다.** `/control` 은 engage 시에만 나간다(조이스틱 A 또는 `engage:=true`).
  E-STOP = 조이스틱 X. **처음엔 바퀴를 지면에서 떼고** 확인한다.
- lap 은 `debug_view` 를 `off` 로 강제한다(최경량, 렌더 없음). 확인은 **토픽 echo** 로 한다.
- 현재 캘리브: **camera.yaml 29cm**(넓은 42cm 는 center_error 를 0.69배로 축소해 조향이 약해진다 — 쓰지 않는다).
- 제어: **kp 1.4 / kd 0.14**(교차로 실차 성공값). 노랑 검출 **임시 OFF**(colors [white]).

---

## A. D3-G 원격과 완전 동기화 (⚠ 로컬 변경·빌드 전부 삭제)

> ⚠ `reset`/`clean` 은 **보드 고유 캘리브**(`vehicle_config.yaml` 의 SERVO_*/ACCEL_RATIO,
> `camera.yaml`)를 커밋값으로 덮어쓰고, 녹화 데이터(`offline/rslt/`, `recorder/`)도 지운다.
> **백업 먼저.**

```bash
cd ~/SC2026     # D3-G 레포 경로 (다르면 맞춰서)

# 1) 보드 고유 캘리브 백업
cp D-Racer-Kit/src/config/vehicle_config.yaml ~/vehicle_config.yaml.bak
cp D-Racer-Kit/src/config/camera.yaml        ~/camera.yaml.bak

# 2) 원격과 완전 동기화 + 빌드 삭제
git fetch origin
git checkout -f -B kos/out-course origin/kos/out-course   # 로컬 수정 폐기 + 브랜치 전환
git clean -fdx                                            # build/install/log + 모든 untracked·ignored 삭제

# 3) 보드 고유 캘리브 복원 (실차 정확성에 필수)
cp ~/vehicle_config.yaml.bak D-Racer-Kit/src/config/vehicle_config.yaml
cp ~/camera.yaml.bak        D-Racer-Kit/src/config/camera.yaml

# 4) 재빌드
cd D-Racer-Kit && colcon build && source install/setup.bash
```

> 캘리브까지 완전 초기화를 원하면 3)을 생략한다. 그 경우 **실차 전 vehicle_config(SERVO/ACCEL)와
> camera 캘리브를 다시 맞춰야** 한다.

---

## B. lap 실행

```bash
# B-1. 검출만 (미션·회피 행동 없음, 토픽 발행만) — 가장 안전
ros2 launch dracer_bringup lap.launch.py
#   engage 기본 false → 안 움직임. /mission/state·/lane/state 로 검출 확인.

# B-2. 미션 스로틀 게이트 (RED/MARK 정지, GREEN 출발)
ros2 launch dracer_bringup lap.launch.py mission_gate:=true
#   ⚠ ON = GREEN 을 볼 때까지 안 움직인다(정지로 시작). engage 해도 마찬가지.

# B-3. 랩 (제어 파라미터는 profile=track.yaml 에서: kp 1.4/kd 0.14)
ros2 launch dracer_bringup lap.launch.py mission_gate:=true
#   그 뒤 회피까지 켜려면 ↓
ros2 param set /perception_node use_fork true
#   (lap 엔 use_fork launch 인자가 없다. param 반영이 안 되면 런치를 내렸다 다시 올린다.)
```

주요 launch 인자: `engage`(기본 false), `mission_gate`(기본 false), `publish_rate`(30.0), `profile`, `mission_config`.

---

## C. 토픽 확인 (별도 터미널)

```bash
# 미션 검출 — 확정 클래스
ros2 topic echo /mission/state --field cls
#   -1 없음 · 0 GREEN · 1 RED · 2 MARK · 3 RIGHT · 4 LEFT

# 차선 + 갈림길
ros2 topic echo /lane/state
#   fork_type: ''|island|branch · n_islands · n_corridors · ego_rule(fork_L/R 이면 회피 중)
ros2 topic echo /lane/state --field fork_type      # 섬 감지만
ros2 topic echo /lane/state --field ego_rule       # 회피 규칙만

# 실제 기동값 (스로틀/조향이 나가는지)
ros2 topic echo /control
```

| 확인할 것 | 토픽·필드 | 기대 |
|---|---|---|
| 섬 검출 | `/lane/state` `fork_type` | 섬 구간 `island` |
| 회피 기동 | `/lane/state` `ego_rule` | 표지판+섬이면 `fork_L`/`fork_R` |
| 미션 검출 | `/mission/state` `cls` | 3/4 표지판, 0/1 신호등, 2 ArUco |
| 실제 출력 | `/control` `steering`/`throttle` | 회피 시 steering, GREEN 후 throttle |

> 회피는 **표지판을 본 뒤 12프레임(약 0.4초) 안에 섬이 나타날 때만** 작동한다(sign_live_hold).
> `/mission/state` cls 가 3/4 로 뜬 직후 `/lane/state` ego_rule 을 같이 본다.

### C-2. 로그를 txt 로 저장 (rslt 폴더, 사후 분석용)

`tee` 는 화면에 보여주며 파일에도 줄 단위로 바로 쓴다(`>` 만 쓰면 버퍼링돼 늦게 써진다). Ctrl+C 로 종료.

```bash
# 미션 전체(header 타임스탬프 + cls + det_cls) — 표지판 검출 분석
ros2 topic echo /mission/state | tee ~/SC2026/offline/rslt/mission_$(date +%H%M%S).txt

# det_cls(순간 검출) + 실시간 시각 — 표지판 바꾼 시점 대조
ros2 topic echo /mission/state --field det_cls \
  | while read l; do echo "$(date +%T) $l"; done \
  | tee ~/SC2026/offline/rslt/detcls_$(date +%H%M%S).txt

# 조향 규칙 + 시각 — 검출→조향 대조
ros2 topic echo /lane/state --field ego_rule \
  | while read l; do echo "$(date +%T) $l"; done \
  | tee ~/SC2026/offline/rslt/ego_$(date +%H%M%S).txt
```

> det_cls 분포는 `rg '^det_cls:' <파일> | sort | uniq -c` 로 센다.
> (실측: invert off 는 RIGHT 를 전부 4 로 오독, invert on 은 3/4 정상 — §E sign_invert 참고.)

---

## D. engage / 안전

```bash
# 바퀴 띄운 것 확인 후 마지막에
ros2 param set /control_node engage true      # 또는 조이스틱 A
# 정지
ros2 param set /control_node engage false     # 또는 조이스틱 X (E-STOP)
```

순서: **바퀴 off → mission_gate 만으로 RED/GREEN 에 /control throttle 0/복귀 확인 → use_fork → 저속 → 지면.**

---

## E. 현재 설정 요약

| 항목 | 값 | 비고 |
|---|---|---|
| camera.yaml | 29cm | 42cm 는 center_error 0.69배 축소 → 조향 약화. 쓰지 않는다 |
| kp / kd | 1.4 / 0.14 | 교차로 실차 성공값 |
| colors | [white] | ⚠ 노랑 OFF(임시) — 노란 인코스 오판 방지. 되돌리려면 [white, yellow] |
| sign_invert | **false (기본)** | 이 대회장은 원본이 정확(좌회전→4/우회전→3). 뒤집히는 대회장서만 `param set /perception_node sign_invert true` |
| use_fork | off (기본) | param 으로 켠다 |
| mission_gate | off (기본) | launch 인자로 켠다 |
