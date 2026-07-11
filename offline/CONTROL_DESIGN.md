# 라인팔로잉 제어 설계 문서 (control_core.py / control_predict.py / control_select.py)

> **성격**: perception(`perception_core`, 확정 단일 파이프라인)과 같은 패턴 — **원리 고정 · 조건 파라미터화**의 config-driven 제어기. 코드+문서로 기록하고, 여러 제어기(C1~C5)를 모드로 실험한다.
> **비목표**: 이 모듈은 **차량을 구동하지 않는다**. 지각 상태 → `(steering, throttle)` 명령만 계산. 실제 발행/구동은 별도 노드 + 차량안전 계층 + 명시적 승인.
> **파일 역할**: `control_predict.py`(영상+수동CSV+perception profile → 컨트롤러별 명령 **예측 계산** → 예측 CSV) → `control_select.py`(예측 CSV를 **open-loop 제어지표**로 랭킹 + 선택 export). 제어 로직 자체는 공유 코어 `dracer_core.control_core.Controller`가 실행(온라인 노드와 동일).
> **관련**: 전체 흐름은 [PIPELINE.md](PIPELINE.md). 인지 출력 계약은 [../PERCEPTION.md](../PERCEPTION.md) §5. 데이터는 recorder가 START 녹화 시 남기는 동기 `drive_<ts>.mp4` + `drive_<ts>.csv`(프레임 1:1 정렬). 남은 작업은 [../ROLLBACK.md](../ROLLBACK.md) §3.

---

## 1. 제어 루프 골격 (모든 제어기 공통)

인지 레이트(**30Hz**, LaneState 마다 1회)로 도는 5단계:
```
① dt       dt_s = clamp(dt, 0, dt_max)            ← 드롭아웃 직후 dt 는 갭 전체다
② 오차     e = center − center_target             (center = ema, use_ema=True)
③ 제어기   u = f(e, ė, ∫e, heading, speed)         ← "알고리즘"(C1..C5). 내부값이다.
④ 후처리   clamp(±steer_max) · slew_rate_per_sec·dt_s · out_ema
           → _emit(u) = u · steer_sign             ← 부호는 여기서 딱 한 번
⑤ throttle throttle_base − curv_slow·|u|  (하한 throttle_min)
           state == OUTLIER → min(thr, throttle_outlier)   ← floor 를 우회한다
⑥ 안전     conf<conf_gate → 직전값 유지, (노드단) 인지·조이스틱 watchdog · E-STOP
```
- 입력은 **인지 출력**, 출력은 **명령**. 인지↔제어 분리.

**부호 규약 (예전 문서가 틀렸다).** `center_error` 는 코리도어 중심의 차량축 오프셋이고
`+` = 코리도어가 **오른쪽**. 법칙이 `u = −Kp·e` 이므로 코리도어가 **왼쪽**(e < 0)이면
`u > 0`. 이 차량(`steer_sign = +1.0`, 0711 트랙 주행으로 확인)에서 **`u > 0` = 좌조향**이다 —
차가 코리도어 쪽으로 돈다. 배선이 반대인 차량은 `steer_sign = -1`.

**`slew_rate` 는 이제 `slew_rate_per_sec` 다.** 예전엔 "프레임당" 이라 차의 최대 회전 속도가
그날의 인지 FPS 였다 (10.7Hz → 1.6/s, 30Hz → 4.5/s — 2.8배가 아무 말 없이). 0711 완주가
30Hz 에서 실제로 낸 값이 **4.5/s** 이고, 그걸 물리량으로 못박았다.

## 2. 제어기 프리셋 (원리 고정)

| 모드 | 제어기 | 식(원리) | 강점 | 조건/약점 | 상태 |
|---|---|---|---|---|---|
| **C1 P** | 비례 | `−Kp·e` | 최단순 | 곡선 정상상태 오차 | ✅ |
| **C2 PD** | 비례+미분 | `−(Kp·e + Kd·ė)` | 진동 감쇠, RC 기본 | Kd 노이즈 민감 | ✅ |
| **C3 PID** | +적분 | `−(Kp·e + Kd·ė + Ki·∫e)` | 정상오차 제거 | windup(→ `i_clamp`) | ✅ |
| **C4 PurePursuit** | 기하(전방주시) | 곡률 `~2·x_L/Ld²` (lookahead 점) | 곡선 강건, 직관적 | lookahead 튜닝, 먼 점 필요 | ✅(근사) |
| **C5 Stanley** | 횡오차+헤딩 | `−(heading + atan2(k·e, soft+v))` | 경로추종 정석 | **heading 신뢰 필요**(현재 노이즈) | ✅(근사) |
| ~~C6 Bang-bang~~ | — | — | — | **의도적 제외** | ✖ |
| C7 학습/BC | 회귀·모방 | `steering=f(특징)` | 데이터 기반 | 분포 과적합, **현장 학습** | ⬜ optional |

> C4/C5는 현재 CSV에 **lookahead 점·안정된 heading이 없어 근사**(`~`)로 평가된다. 온보드 로깅에 곡률·lookahead 횡오차를 추가하면 정식 평가 가능.

## 3. 파라미터 (조건별 그룹) — `CtrlCfg`

**원리는 고정, 아래 조건들만 튜닝**한다. 차량/트랙/해상도가 바뀌면 값만 바꾼다.

| 그룹 | 파라미터 | 무엇에 의존 |
|---|---|---|
| 오차원/기준 | `use_ema`, `center_target` | **검출·트랙**(직진일 때의 center 오프셋) |
| 게인 | `kp`, `kd`, `ki`, `i_clamp` | **속도·트랙 곡률** |
| Pure Pursuit | `lookahead`, `pp_gain` | **해상도·시야깊이·속도** |
| Stanley | `stanley_k`, `stanley_soft`, `heading_gain` | **속도·heading 품질** |
| 차량/출력 | `steer_max`, `steer_sign`, **`slew_rate_per_sec`**, `out_ema`, **`dt_max`** | **차량 서보·배선** |
| throttle | `throttle_base`, `throttle_min`, `curv_slow`, **`throttle_outlier`** | **차량·트랙·규정 속도** |
| 안전 | `conf_gate` | **검출 신뢰도 분포** |

> ⚠ **`steer_max` 는 `1.0 − |STEER_TRIM|` 이어야 한다.** actuator 가 `clamp(u + trim, -1, 1)`
> 로 트림을 더하므로 u 와 트림이 같은 `[-1,1]` 서보 예산을 나눠 쓴다. 넘으면 **한쪽 조향
> 권한만** 조용히 깎인다 (서보는 멈춰 있는데 제어기는 계속 더 달라고 한다 → 한쪽 코너만
> 언더스티어). 현재 `STEER_TRIM = 0.3` → `steer_max: 0.7`. actuator 가 포화 시 경고를 찍는다.

> ⚠ **C4/C5 의 파라미터(`lookahead`·`pp_gain`·`stanley_k`·`stanley_soft`·`heading_gain`·
> `use_ema`·`i_clamp`)는 `control_node` 의 ROS 파라미터가 아니다.** `controller: C4` 로 바꿔도
> 기본값으로만 돈다 — **튜닝할 방법이 없다.** Pure Pursuit 의 전제조건이다
> ([../ROLLBACK.md](../ROLLBACK.md) TODO B2).

프리셋을 `make_ctrl('C2', kp=0.7, center_target=-0.15, steer_sign=-1)`처럼 덮어써 조합.

## 4. 부호 & setpoint 캘리브레이션 (필수 선행)
- **steer_sign**: 바퀴 들고 소량 조향 명령을 줘 실제 방향과 부호가 맞는지 **1회 확인**.
  차선이 **왼쪽**이면 `steering > 0` 이어야 한다.
- **center_target**: 직진 구간을 주행해 정상상태 `center_error` 를 읽어 넣는다. 다만 metric
  BEV 로 옮긴 뒤로는 편향이 크게 줄었다 (0711 완주는 `center_target: 0.0`).
- ⚠ 예전 문서가 권하던 `trap_top_w`(ROI 사다리꼴) 튜닝은 **불가능하다 — 삭제됐다.**
  BEV LUT 자체가 캘리브레이션된 사다리꼴 크롭이다. 그리고 `lane_width_default` 는 이제
  **파생 필드**다(`cfg_to_px` 가 `lane_width_cm` 에서 계산). 진짜 노브는 `lane_width_cm` 다.

## 5. 오프라인 예측 & 평가 (`control_predict.py` → `control_select.py`)

### 5.0 핵심 제약 — offline은 open-loop다 (covariate shift)
녹화 영상은 **사람이 지난 경로의 카메라 뷰**만 담는다. 컨트롤러가 다르게 조향했다면 차량 pose·뷰·차선 상태가 달라지지만 그 뷰는 존재하지 않는다. 따라서 **"컨트롤러가 실제로 몰았을 때의 차선중심-차량중심 오차"는 offline으로 측정 불가**하고, 녹화의 `center_error`는 전부 사람 궤적 값이라 컨트롤러 랭킹에 못 쓴다. → 평가는 **사람 모방 정확도가 아니라, 명령 자체의 품질(open-loop)** 로 한다. (사람 주행은 정답이 아니라 참조.)

### 5.1 control_predict.py — 명령 예측 계산
입력 = **영상 + 수동 CSV + profile의 perception 섹션**(`perception_core` — 온라인과 동일 인지). 프레임마다:
1. profile의 perception으로 `LanePipeline`을 **영상에 재실행** → lane state(record-time 검출값에 의존 X, 새 지각+제어 조합을 재녹화 없이 평가).
2. 각 컨트롤러 후보(`--controllers C1,C2,C4` + override)를 `Controller.step(state, dt)`로 돌려 조향/스로틀 **예측**.
3. 같은 행의 `manual_steering/throttle`과 함께 **예측 CSV**로 저장(프레임 1:1 정렬).
```
../.venv/bin/python control_predict.py ../rslt/drive_YYYYMMDD.mp4 \
    --csv ../rslt/drive_YYYYMMDD.csv \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml \
    --controllers C1,C2,C3,C4,C5            # 예측 CSV -> rslt/pred_*.csv
```
예측 CSV(wide): `frame_time, center_error, ema, heading, confidence, manual_steering, manual_throttle, pred_steer_C2, pred_thr_C2, gated_C2, ...`

### 5.2 control_select.py — open-loop 제어지표 랭킹 + export
예측 CSV의 각 후보 명령 시퀀스에서 지표를 집계해 랭킹한다. **폐루프 궤적 지표(진동/발산)는 track 지도 시뮬이 필요 → 후순위**([PIPELINE.md](PIPELINE.md) §4).

| 지표 | 정의 | 좋음 |
|---|---|---|
| **부드러움** | mean&#124;Δu&#124;, RMS jerk | 낮을수록 |
| **흔들림(oscillation)** | 조향 부호변경률 / wobble 진폭 | 낮을수록 |
| **응답 정합성** | u 와 −e(·heading) 부호일치율/상관 — 오차를 줄이는 방향인가 | 높을수록 |
| **포화율** | &#124;u&#124; ≥ `steer_max` 비율 | 낮을수록 |
| **게이팅** | low-conf hold 발생 횟수 | 낮을수록 |
| **사람 참조** | manual_steering 상관/MAE — **정답 아님, 참조만** | 참고 |

- **선택 → export**: 최적 controller + gains를 profile YAML의 `control:` 섹션에 in-place로 write(`--export <track>.yaml`). `perception:` 등 나머지는 보존. 이 파일이 온라인 `control_node`가 로드하는 계약.

## 6. 실험 방식 (2단계, 안전)
1. **오프라인 예측·평가** — 위 predict→select로 차 없이 제어기·게인 후보를 open-loop 지표로 랭킹.(1차 필터)
2. **폐루프 실차** — 통과분만. `control_node`에 export한 profile 적용, `/control` 발행. **바퀴 들고→저속 트랙** 순서, 조이스틱 X E-stop 상시, `control_timeout` watchdog. **명시적 승인 후에만.**

### 참고: 과거 모방-피팅 결과 (한계 예시)
초기 `control_eval.py`는 사람 steering에 각 제어기를 최소자승 피팅(R²/MAE)했으나, 한 방향 곡선 루프(조향 57% 포화) 데이터에선 C1/C2의 `R²(no bias)`가 음수 → **상수 우조향 오프셋이 데이터를 지배**해 게인을 신뢰성 있게 못 뽑았다. 이는 §5.0의 open-loop·데이터 편향 문제를 보여준다. **양방향 + 직진 + 복귀 주행 데이터 보강**이 선행되어야 한다.

## 7. TODO

전체 목록은 [../ROLLBACK.md](../ROLLBACK.md) §3. 제어 관련만:

- [ ] **C4/C5 파라미터를 `_CTRL_FLOATS` 에 노출** (TODO B2). **다른 모든 제어 작업의 전제조건.**
- [ ] **metric Pure Pursuit** (TODO D3). 현재 `center_error` 는 **전방 26~30cm** 에서 측정된다 —
      preview 가 거의 없는 순수 횡오차 레귤레이터다. 저속에선 잘 돌지만 속도를 올리면 반드시
      진동한다. 그런데 **78cm 앞까지의 중앙선 다항식이 이미 손에 있다.**
      ```
      Ld   = 50cm (또는 speed 적응)
      v    = cam.cm_to_bev(0, Ld)[1]                      # BEV 행 (ec['y_lo'] 로 클램프)
      x_cm = cam.bev_x_to_cm(_ebottom(ec['coeffs'], v))   # 그 지점의 실제 횡오차
      κ    = 2·x_cm / Ld²                                 # Pure Pursuit 곡률
      δ    = atan(WHEELBASE_cm · κ)                       # Ackermann
      u    = δ / MAX_STEER_RAD
      ```
      파라미터가 게인이 아니라 **측정값**(휠베이스, 최대 조향각)이고, `κ` 가 공짜로 나오니
      **곡률 기반 감속**(`curv_slow · |κ|`)도 함께 붙는다. 지금 `curv_slow` 는 곡률이 아니라
      `|steering|` 에 곱해져서 **결과에 반응**한다 — 위상이 늦다. 그리고 프로파일에서 0 이다.
- [ ] **인지 EMA 시간 단위화** (TODO B1). `slew_rate` 만 고쳤다.
- [ ] **PID anti-windup** (TODO C6). 적분이 low-conf 에서도 누적되고, 출력 포화와 연동되지 않는다.
- [ ] (optional) C7 학습형: 균형 데이터 확보 후 현장 학습.

## 8. 대회장 운용 메모
- 목표: 현장에서 **제어기 새로 짜지 않고** `controller` 선택 + 게인/`center_target`/`throttle_base` 튜닝만.
- 트랙마다 곡률·속도·차선색·해상도가 다르므로 **원리 고정 + 파라미터 튜닝** 구조가 핵심.
- 확정 제어기만 온보드 노드로 이식, D3-G에서 지연/안정성 검증.
