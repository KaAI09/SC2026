# 오프라인 파이프라인 구조 (offline/)

> **성격**: 로컬 실험 도구. 차량/배포 코드가 아니며, 실제 주행·구동을 하지 않는다.
> 모든 검출·제어 **로직은 공유 코어 `driving_core`**(온라인 노드와 동일)를 그대로 실행하고,
> offline은 **영상/CSV 입출력 · 시각화 · 성능 지표 · 조합 선택**만 담당한다(single-source).
> **최종 산출물**: 한 트랙의 검출·제어 조합을 담은 **driving-profile YAML**
> (`config/profiles/<track>.yaml`) — 온라인 `perception_node`/`driving_node`가 로드하는
> P5 계약. offline의 목표는 "무엇을 고를지"를 이 한 파일로 확정하는 것.

관련 심화 문서: 지각 [LANE_DETECTION.md](LANE_DETECTION.md), 제어 [CONTROL_DESIGN.md](CONTROL_DESIGN.md).

---

## 0. 대상 트랙 (설계 범위 = 딱 2개)

모든 색상·ROI·차선폭·크기 조건은 **아래 두 트랙에서만** 도출한다(범용화하지 않음). 근거 자료도 이 둘로 한정.

| | 2025 트랙 (테스트) | 2026 트랙 (본선) |
|---|---|---|
| 자료 | `offline/Dashcam(2025 Track)/*.mp4`(13개, 320×160) + `2025Track Info.png` | `Notice/2026 Track(1~4)`, `2026 SEAME 규정집.pdf`, `미션 및 규정 설명 OT.pdf` |
| 주행 경계 | **얇은 흰색 실선 2줄**(양쪽) | **넓은 흑색 도로(≈350mm) + 양쪽 흰색 엣지 라인(≈30mm)** |
| 중앙 특수 | 노랑/주황 **테이프** 지름길 + 회전교차로 | 노랑 **실선+점선 유도선** 회전교차로 |
| 크기 | (도면만, 대시캠 튜닝) | ≈ **5172 × 4227 mm**, 곡률 R449~R1502, S자 슬라럼 |
| 특수 요소 | 빨간 스쿨존, 횡단보도/정지선 | 빨간 구간, 체커 시작/피니시, 상단 island, ArUco(동적장애물) |

- **공통 색상 모델**: 흰색 = 주행 경계(**M 프리셋**), 노랑/주황 = 로터리·지름길(**O 프리셋**). driving_core의 M/O 구조가 두 트랙 모두 커버.
- **2026 주행영상(dashcam)은 대회 당일 업로드 예정** → 2026 지각 HSV/ROI 최종값은 그때 확정. 그 전 프로파일 값은 도면+2025 대시캠 기반 **provisional**.
- 핸드오프 프로파일도 두 트랙 전용 2벌: `config/profiles/track2025.yaml`, `config/profiles/track2026.yaml`.

---

## 1. 5개 파일과 역할 (지각 2 + 제어 2 + 공용 1)

| 파일 | 도메인 | 역할 | 입력 | 출력 |
|---|---|---|---|---|
| **perception_preview.py** | 지각 | 검출 적용 + 3패널 시각화 | 영상 | `rslt/*__<mode>.mp4` |
| **perception_select.py** | 지각 | 조합 비교 + **지각 지표 랭킹** + 선택 export | 영상 | 비교 PNG + profile `perception:` |
| **control_predict.py** | 제어 | 실제 컨트롤러로 명령 **예측 계산**(open-loop) | 영상 + 수동 CSV + perception profile | 예측 CSV |
| **control_select.py** | 제어 | **제어 지표 랭킹** + 선택 export | 예측 CSV (+수동 CSV, 영상) | 리포트 + profile `control:` |
| **_common.py** | 공용 | 영상 IO · 패널 렌더 · CSV 로드/정렬 · profile r/w · 지표 헬퍼 | — | — |

각 도메인은 **"적용+시각화 → 비교+핸드오프"** 로 대칭이다.

## 2. 데이터 흐름 (선형 의존)

```
영상 ──▶ ① perception_preview   (mode/param을 눈으로 튜닝)
영상 ──▶ ② perception_select ──▶ profiles/<track>.yaml [perception]  ═▶ D3-G
                                          │
영상 + 수동CSV ──▶ ③ control_predict (②의 perception을 영상에 재실행)──▶ 예측 CSV
                                          │
예측 CSV ──▶ ④ control_select ──▶ profiles/<track>.yaml [control]     ═▶ D3-G
```

- ②가 perception 섹션을 확정 → ③이 그 profile로 영상에 `LanePipeline`을 **재실행**해 lane state를
  새로 생성(녹화 CSV의 record-time 검출값에 의존하지 않음 → 새 지각+제어 조합을 재녹화 없이 평가).
- ③은 각 컨트롤러 후보를 `control_core.Controller.step()`으로 돌려 조향/스로틀을 예측만 한다(구동 X).
- ④는 예측을 지표로 랭킹해 control 섹션을 확정. 최종 = 채워진 track profile 하나.

## 3. 핸드오프 계약 (profile YAML)

`config/profiles/<track>.yaml` — `driving_core.profile`가 로드. 키는 `Cfg`/`CtrlCfg` 필드와 1:1.
```yaml
name: <track>
perception: { mode: <str>, <lane_core.Cfg field>: <value>, ... }   # ②가 write
control:    { controller: <str>, <control_core.CtrlCfg field>: <value>, ... }  # ④가 write
```
- **단일 파일 in-place 업데이트**: ②는 `perception:` 섹션만, ④는 `control:` 섹션만 교체하고 나머지는 보존.
- 온라인 소비: `perception_node`/`driving_node`가 `profile` 파라미터로 이 파일을 읽어
  `make_cfg(mode, **perception)` / `make_ctrl(controller, **control)`에 그대로 넘긴다.

## 4. 성능 지표 (어디서 무엇을)

**핵심 제약 — offline은 open-loop다.** 녹화 영상은 *사람이 지난 경로*의 뷰만 담는다. 컨트롤러가
다르게 조향했다면 차량 pose·카메라 뷰·차선 상태가 달라지지만 그 뷰는 존재하지 않는다(covariate
shift). 따라서 **"컨트롤러가 실제로 몰았을 때의 차선중심-차량중심 오차"는 offline으로 측정 불가**.
녹화 데이터의 `center_error`는 전부 사람 궤적의 값이라 컨트롤러 랭킹에 쓸 수 없다. 그래서 지표를 둘로 나눈다.

- **File 2 (지각 지표, 컨트롤러 무관)** — 자세히는 [LANE_DETECTION.md](LANE_DETECTION.md)
  - 검출율(conf ≥ τ 프레임 비율), `center_error` std·bias(대칭성), heading jitter,
    좌/우 균형(`per_lane_conf`), 프레임간 일관성
- **File 4 (제어 지표, open-loop, 후보별)** — 자세히는 [CONTROL_DESIGN.md](CONTROL_DESIGN.md)
  - 부드러움(mean|Δu|·RMS jerk), 흔들림(조향 부호변경률), 응답 정합성(u vs −e 부호일치/상관),
    포화율(|u| ≥ steer_max), 게이팅 횟수, 사람 조향 상관/MAE(**정답이 아닌 참조 지표**)
- 궤적 기반 폐루프 지표(진동/발산)는 track 지도 기반 시뮬이 필요 → 별도/후순위.

## 5b. perception_select (2단계) 상세 설계

6군이 **조건 전문가**라, "전체 1등"이 아니라 **조건별 승자 + 단일 설정이 트랙 전체를 커버하는지**를 본다(= "흰+노랑 통합 vs 구간전환" 결정 실험).

- **조건 라벨**: 클립→조건 **수동 매핑**(작은 라벨 파일). 확정 매핑: `401,403=white_line`, `411,413=white_curve`, `408,409,410=yellow_solid`, `404,405=yellow_dashed`, `406=white_yellow`, `407=robust`.
- **처리**: 각 (그룹 × 클립) LanePipeline 전 프레임 실행 → 지각지표(coverage·center bias/jitter·heading jitter·L/R balance·outlier율) 집계 → **group×clip 매트릭스**.
- **산출(사람이 판단)**: 자동 export 안 함. ① 지표 **매트릭스 리포트**(히트맵/표) + ② **조건별 검출 격자 PNG**(기존 compare식) 둘 다 출력 → 사람이 단일/전환 결정 후 profile `perception:` write.
- 분석 관점: 조건별 최적 그룹 / 강건성 랭킹(전 클립 최악값) / 커버리지 갭(단일 그룹으로 모든 조건 합격 가능?).

## 5. 산출물 & git

- 코드(`*.py`)·문서(`*.md`)는 저장소 공유(추적).
- 실행 결과물(`rslt/*.mp4`, `rslt/*.png`, 예측 `*.csv`)은 git-ignore.
- profile YAML(`config/profiles/*.yaml`)은 **추적**(온라인 노드가 로드하는 계약이므로).
