# 라인팔로잉 제어 설계 문서 (control_core.py / control_eval.py)

> **성격**: perception(`lane_core`/`lane_preview`)과 같은 패턴 — **원리 고정 · 조건 파라미터화**의 config-driven 제어기. 코드+문서로 기록하고, 여러 제어기를 모드로 실험한다.
> **비목표**: 이 모듈은 **차량을 구동하지 않는다**. 지각 상태 → `(steering, throttle)` 명령만 계산. 실제 발행/구동은 별도 노드 + 차량안전 계층 + 명시적 승인.
> **관련**: 지각 출력은 [LANE_DETECTION.md](LANE_DETECTION.md)의 `center_error / ema / heading / confidence`. 데이터는 `lane_detect_node`가 START 녹화 시 남기는 `lane_*.csv`.

---

## 1. 제어 루프 골격 (모든 제어기 공통)

카메라 레이트(~25Hz)로 도는 5단계:
```
① 오차     e = center − center_target            (center = ema 또는 center_error)
② 제어기   u = f(e, ė, ∫e, heading, speed)        ← "알고리즘"(C1..C5)
③ 후처리   steer_sign · clamp(±steer_max) · slew_rate · out_ema
④ throttle throttle_base − curv_slow·|u|  (하한 throttle_min)
⑤ 안전     conf<conf_gate → 직전값 유지/감속, (노드단) watchdog·E-stop
```
- 입력은 **지각 출력**, 출력은 **명령**. 지각↔제어 분리.
- **부호 규약**: 수집 데이터 기준 `center_error < 0 → 우조향(+)`. 코드 기본이 이를 따르며, 배선 반대면 `steer_sign=-1`.

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
| 차량/출력 | `steer_max`, `steer_sign`, `slew_rate`, `out_ema` | **차량 서보·배선** |
| throttle | `throttle_base`, `throttle_min`, `curv_slow` | **차량·트랙·규정 속도** |
| 안전 | `conf_gate` | **검출 신뢰도 분포** |

프리셋을 `make_ctrl('C2', kp=0.7, center_target=-0.15, steer_sign=-1)`처럼 덮어써 조합.

## 4. 부호 & setpoint 캘리브레이션 (필수 선행)
- **center_target**: 검출 `center_error`는 "직진=0"이 아니다(수집 평균 -0.16 편향; 단일선 fallback·ROI·장착 탓). **직진 구간을 주행해 정상상태 center 값을 읽어** `center_target`으로 넣는다.
- **steer_sign**: 바퀴 들고 소량 조향 명령을 줘 실제 방향과 부호가 맞는지 1회 확인.
- 검출 편향이 크면 제어가 흔들리므로 **먼저 2선 ROI(`trap_top_w`)·`lane_width_default` 튜닝**으로 center를 대칭·안정화(→ [LANE_DETECTION.md]).

## 5. 오프라인 모방평가 (`control_eval.py`)
차 없이 각 제어기를 **사람 조작과 비교**해 1차 랭킹. 각 제어기를 상태의 *특징변환*으로 보고 사람 steering에 **최적 스케일+bias**를 피팅 → `R²`, `MAE`, 추정게인 리포트. `bias`는 한방향 루프의 상수 feedforward를 흡수하므로 **`R²(no bias)`로 "상수오프셋 의존도"**를 함께 본다.
```
../.venv/bin/python control_eval.py ../rslt/lane_O2_20260703_160323.csv
```

### 첫 결과 (lane_O2_20260703_160323.csv, 757프레임/29.7s)
```
controller        R2(+bias)  R2(no bias)   MAE   fitted
C1 P                  0.223     -0.972    0.344  Kp≈0.59, bias +0.53
C2 PD                 0.239     -0.965    0.338
C3 PID                0.258     +0.206    0.332
C4 PurePursuit~       0.373     +0.198    0.285  (best)
C5 Stanley~           0.352     +0.141    0.288
```
**해석(중요)**: 모든 R²가 낮다. 이 클립은 **한 방향 곡선 루프 + 조향 57% 포화**라, C1/C2는 `R²(no bias)`가 **음수** → 순수 center-following 신호는 거의 없고 **상수 우조향 오프셋이 데이터를 지배**한다. 즉 **이 데이터만으로는 제어기·게인을 신뢰성 있게 못 뽑는다.** → **양방향 + 직진 + 복귀 주행으로 데이터 보강**이 선행되어야 한다.

## 6. 실험 방식 (2단계, 안전)
1. **오프라인 모방평가** — 위 하버스로 차 없이 제어기·게인 후보 랭킹.(1차 필터)
2. **폐루프 실차** — 통과분만. `lane_follow_node`에 `controller:=C2`·게인 파라미터, `/control` 발행. **바퀴 들고→저속 트랙** 순서, 조이스틱 X E-stop 상시, `control_timeout` watchdog. **명시적 승인 후에만.**

## 7. TODO
- [ ] 지각 로깅에 **곡률 + lookahead 횡오차** 추가 → C4/C5 정식 평가.
- [ ] **양방향·직진·복귀 데이터 수집** 후 setpoint·게인 재피팅.
- [ ] `control_eval`에 **폐루프 시뮬레이션**(간이 차량모델)로 진동/발산 예측 추가.
- [ ] `lane_follow_node`(온보드): `control_core` 재사용 + 안전계층 + 파라미터.
- [ ] (optional) C7 학습형: 균형 데이터 확보 후 현장 학습.

## 8. 대회장 운용 메모
- 목표: 현장에서 **제어기 새로 짜지 않고** `controller` 선택 + 게인/`center_target`/`throttle_base` 튜닝만.
- 트랙마다 곡률·속도·차선색·해상도가 다르므로 **원리 고정 + 파라미터 튜닝** 구조가 핵심.
- 확정 제어기만 온보드 노드로 이식, D3-G에서 지연/안정성 검증.
