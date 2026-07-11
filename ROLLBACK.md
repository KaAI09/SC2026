# ROLLBACK — 0712 안전/강건성 변경분

**기준선:** 커밋 `2d329ee` = 2026-07-11 14:55 완주 (drive.launch 3세션, 1,505프레임,
트랙 이탈 0, 인지 30.0Hz, LOST 0%). 이 커밋 이후의 변경은 **아직 D3-G 실차 검증이 없다**
(트랙 사용 종료). 오프라인 회귀만 통과했다.

이 문서는 트랙에서 무언가 나빠졌을 때 **코드를 건드리지 않고** 검증된 상태로 되돌리는 법이다.

---

## 1. 즉시 전체 복구 (리빌드 없음)

```bash
ros2 launch dracer_bringup drive.launch.py \
    profile:=$HOME/SC2026/D-Racer-Kit/src/config/profiles/track2025_0711.yaml \
    command_hz:=10.0 publish_rate:=20.0
```

`command_hz` / `publish_rate` 는 **반드시 명령줄에 같이** 넣어야 한다. 두 값 모두 노드 생성
시점에 타이머를 만들기 때문에 프로파일이나 `ros2 param set` 으로는 되돌릴 수 없다.

이걸로 아래 표의 **[P]** 항목이 전부 0711 값으로 돌아간다.

---

## 2. 변경 항목별 복구값

| 항목 | 0711 (검증됨) | 0712 (현재) | 복구 | 정상 주행 중 발동? |
|---|---|---|---|---|
| **`throttle_outlier`** | 없음 | `0.0` | **[P]** `1.0` | 🔴 **예 — 랩당 3~4회, 최대 0.17s** |
| `outlier_relatch` | `6` | `5` | **[P]** `6` | 예 (측정상 개선) |
| `steer_max` | `0.8` | `0.7` | **[P]** `0.8` | 아니오 (실측 max \|u\|=0.528) |
| `track_width_tol` | 없음(=무제한) | `0.25` | **[P]** `0.0` | 아니오 (1,505프레임 중 0회 발동) |
| `coast_flip_support` | 없음 | `0.15` | **[P]** `0.0` | 예 (1,505프레임 중 **1회** 발동) |
| `slew_rate` → `slew_rate_per_sec` | `0.15`/step | `4.5`/sec | **[P]** `4.5` | 아니오 (30Hz에서 동일) |
| `dt_max` | 없음 | `0.1` | **[P]** `10.0` | 아니오 (30Hz에서 dt=0.033) |
| `command_hz` | `10.0` | `30.0` | **[L]** `10.0` | 예 (지연 감소) |
| `publish_rate` | `20.0` | `30.0` | **[L]** `20.0` | 예 (지연 감소) |
| `state_timeout` (perception 워치독) | 없음 | `0.25` | **[P]** `0.0` = 비활성 | 아니오 (고장 시만) |
| `joystick_timeout` (조이스틱 워치독) | 없음 | `0.3` | **[P]** `0.0` = 비활성 | 아니오 (고장 시만) |
| `rate_floor_hz` (인지 Hz 경고) | 없음 | `24.0` | **[P]** `0.0` = 비활성 | 로그만 |

**[P]** = 프로파일 / `ros2 param set` 으로 복구 · **[L]** = 런치 인자로만 복구

### 트랙에서 한 항목씩 끄기 (엔게이지 해제 후)

```bash
ros2 param set /control_node    throttle_outlier   1.0  # 스로틀 컷 끄기 ← 먼저 이것부터
ros2 param set /control_node    steer_max          0.8
ros2 param set /perception_node outlier_relatch    6
ros2 param set /perception_node track_width_tol    0.0
ros2 param set /perception_node coast_flip_support 0.0  # coast 좌/우 마스크 반증 끄기
ros2 param set /control_node    state_timeout    0.0    # 워치독 끄기 (권장하지 않음)
ros2 param set /control_node    joystick_timeout 0.0
```

> ⚠ `ros2 param set /perception_node <field>` 는 파이프라인을 재생성한다 = **Tracker/EMA
> 상태가 리셋된다.** 주행 중이 아니라 정지 상태에서 바꿔라.

---

## 3. 파라미터로 되돌아가지 않는 것 (리빌드 필요)

| 변경 | 파일 | 측정 결과 |
|---|---|---|
| `_Stabilizer` 카운터 분리 (`missing`/`rejects`) | `perception_core.py` | 구/신을 1,505프레임에 나란히 통과시킨 결과 **state·ema 모두 비트 단위 동일**. 이 세션들엔 미검출 프레임이 0개라 구코드의 `lost` 가 outlier 로만 증가했다. |
| `steer_sign` 을 `_emit()` 한 곳에서만 적용 | `control_core.py` | `steer_sign=1.0` 에서 산술적으로 동일. 게다가 문제의 `low_conf_hold` 경로는 conf ∈ {0.9, 0.5, 0.0} vs `conf_gate=0.4` 라 **현재 도달 불가능**. |
| Tracker 폭: `y=h-1` 외삽 → 공통 관측구간 median | `perception_core.py` | 관측 범위 밖 외삽 제거. 폭 추정 최악 오차 29.0 → 32.4cm (실제 35cm). 게이트는 `track_width_tol` 로 끌 수 있으나 **median 계산 자체는 파라미터가 없다.** |
| **P2: 이미지 중앙 하드컷 제거 + `Tracker.adopt`** | `perception_core.py` | ⚠ **가장 큰 인지 변경.** 아래 참조. |

### P2 상세 (`_assign` / `_seed` / `adopt` / `lane_centers` / `ego_center`)

`w/2` 를 좌우 구분자로 쓰던 세 곳을 제거하고, 트래커의 시간적 정체성을 우선하도록 바꿨다.
그리고 물리적으로 검증된 페어가 트래커 정체성을 교정하도록 되먹임(`adopt`)을 추가했다.

0711 3세션 A/B (구 = 2d329ee):

| | 구 | 신 |
|---|---|---|
| **진동** (뒤집혔다 되튀김) | **7회** | **0회** |
| 부호 반전 (총) | 17회 | 3회 (전부 일회성 교정, 되돌아오지 않음) |
| OUTLIER | 2.4% | 1.1% |
| max \|Δcenter_error\| | 1.316 | 1.248 |
| 쌍검출 / coast | 65/56/33%, coast 50% | 동일 |

되돌리려면 **파라미터가 없다.** `perception_core.py` 전체를 되돌려야 하고, 그러면 위의 세
항목(카운터 분리·폭 median)도 같이 돌아간다. 인지가 이상하면 여기부터 의심하라.

셋 다 의심스러우면:

```bash
git diff 2d329ee -- D-Racer-Kit/src/dracer_core/dracer_core/perception_core.py
git checkout 2d329ee -- D-Racer-Kit/src/dracer_core/dracer_core/perception_core.py
colcon build --packages-select dracer_core --symlink-install
```

---

## 4. 완전 복구 (핵폭탄)

```bash
git checkout 2d329ee -- D-Racer-Kit/ offline/
colcon build --symlink-install
```

---

## 5. 트랙 복귀 시 검증 순서

**바퀴를 지면에서 뗀 상태로 1~3번을 먼저 끝내라.**

1. **기동 로그 확인**
   ```
   control_node    : state_timeout=0.25s joystick_timeout=0.3s throttle_outlier=0.0 steer_max=0.7
   actuator_node   : command_hz=30.0
   perception_node : [lane] 30.0Hz state=OK ...
   ```
   `[lane]` 앞의 Hz 가 30 이 아니면 **여기서 멈춰라.** 게인은 30Hz 에서 튜닝됐다.

2. **perception 워치독** — engage 후 `perception_node` 를 죽인다 →
   `PERCEPTION STALE` 이 뜨고 `/control` 이 `(0, 0)` 으로 고정돼야 한다.

3. **joystick 워치독** — engage 후 `joystick_node` 를 죽인다 →
   `JOYSTICK STALE ... Forcing engage OFF`.
   (⚠ **패드를 뽑는 걸로는 발동하지 않는다** — `joystick_node` 가 마지막 입력을 50Hz 로
   계속 재발행한다. 반드시 노드를 죽여서 시험하라.)

4. **저속 주행** — 여기서 볼 것은 딱 하나다:
   **OUTLIER 스로틀 컷의 주행감.** 0711 데이터 기준 랩당 3~4회, 매번 최대 0.17초 타력주행이
   예상된다(전체 주행 시간의 0.5초 남짓). 차가 울컥거리거나 멈칫하면 **가장 먼저**
   `throttle_outlier` 를 1.0 으로 되돌려라. `throttle_base 0.23` / `throttle_min 0.22` 라
   스로틀 여유폭이 0.01 밖에 없어서, 감속 정책이 사실상 "끊거나 말거나" 뿐이다.

5. 4번이 문제없으면 나머지는 전부 실패 경로에서만 동작하므로 정상 주행에 영향이 없다.
