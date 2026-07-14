# 오프라인 도구

로컬(macOS, ROS 불필요, 레포 `.venv`)에서 도는 분석·튜닝 도구. 코어(`dracer_core`)가 순수
파이썬이라 **차가 돌리는 것과 같은 알고리즘**을 그대로 돌린다.

```bash
cd ~/workspace/SC2026
.venv/bin/pip install -e D-Racer-Kit/src/dracer_core        # 최초 1회
```

| 파일 | 역할 |
|---|---|
| **`panel_replay.py`** ⭐ | 주행 raw → 4패널 재구성 + 실차 csv 대조 + 파라미터 A/B |
| `calibrate.py` | 체커보드·지면 사진 → `camera.yaml` (K/D + 호모그래피 H) |
| `lane_color_probe.py` | 대회장 조명에서 흰/노랑이 **HSV 어디 있는지 측정** → 임계값 제안 |
| `color_gate_probe.py` | `color_gate` 가 진짜 차선을 지우는지 센다 (남은 작업 B4) |
| `make_lane_target.py` | 직선 차선 검증 타깃 → A4 타일 PDF (트랙 없이 `--check`) |
| `_common.py` | 영상/CSV/profile IO 헬퍼 |

데이터 (git 미추적 — scp 로 가져온다):

| 경로 | 내용 |
|---|---|
| `rslt/0712/` | **성공 주행 기록** (raw mp4 + csv, 3세션 1,294프레임) |
| `calib/` | 캘리브레이션 사진 |

---

## panel_replay — 주력 도구

`drive.launch` 는 패널을 녹화하지 않는다: 패널 합성 + JPEG 인코딩이 검출의 **4배**를 먹어서
그게 프레임 드랍의 진범이었다. 대신 **raw 카메라 + LaneState csv** 를 남기고, 그 둘로 패널을
**여기서, 사후에** 되살린다. 차는 주행 중 렌더링 비용을 한 푼도 내지 않는다.

같은 `dracer_core` 파이프라인을 그대로 돌리므로 재구성 결과는 보드가 봤을 화면과 **같다.**

```bash
.venv/bin/python offline/panel_replay.py offline/rslt/recorder/raw/drive_<stamp>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml \
    --csv offline/rslt/recorder/csv/drive_<stamp>.csv

# 파라미터 A/B (원본 profile 은 안 건드린다)
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera ... --profile ... \
    --set lane_width_cm=35 --set branch_policy=random --no-video
```

출력: `valid%` · 쌍검출% · coast% · `OK/HOLD/OUTLIER/LOST` 비율.

`--csv` 를 주면 실차가 그때 발행한 LaneState 와 **프레임 단위로 대조**한다 (`|Δcenter_error|`
중앙값·p90·최대). **중앙값 < 0.02 면 재현 일치** — 오프라인 튜닝을 믿어도 되는지에 대한
검증이다.

## lane_color_probe — 색 임계를 재는 도구

색 임계를 눈으로 맞추면 **순환에 빠진다**: 임계가 틀려서 테이프를 못 잡으면, 그 테이프의
HSV 분포도 볼 수 없다. 그래서 느슨한 **탐색 임계**로 후보를 먼저 건지고, 거기서 나온
**분위수**로 운영 임계를 제안한다.

측정은 전부 **BEV 위에서** 한다. 원본 프레임에는 관중·천장·옆 트랙이 같이 찍히고, 그것들의
HSV 를 섞은 히스토그램은 노면에 대해 아무 말도 하지 않는다. BEV 는 캘리브레이션된 지면
크롭이니 거기 있는 픽셀은 정의상 노면이다. **그러니 `camera.yaml` 이 먼저 맞아야 한다.**

```bash
.venv/bin/python offline/lane_color_probe.py offline/rslt/<세션>/raw/collect_*.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml --stride 5
```

0714 대회장 지면 사진 결과: 노란 테이프의 H 피크가 **17~19** 인데 `yellow_h_lo` 가 **18** —
임계 경계에 걸터앉아 난색 픽셀의 **31%** 가 잘려나간다. 그리고 `y_frac = 0.074` 로
`color_gate` (0.15)의 **절반**이라 노란색이 **상시 버려진다**. 이 트랙의 노란 차선은 실선
지름길이 아니라 **점선 중앙선**이고, 점선은 면적 비율 게이트를 원리적으로 통과할 수 없다.

## color_gate_probe — B4 를 세는 도구

```bash
.venv/bin/python offline/color_gate_probe.py offline/rslt/<세션>/raw/*.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml
```

`color_gate` 를 켠 패스와 끈 패스를 **프레임 단위로 대조**한다. 핵심 숫자는 하나 —
**"게이트가 분기를 숨긴 프레임"** (B 에선 코리도어 2개, A 에선 1개 이하). 0 이면 게이트는
그 데이터에서 무해하다.

0712 성공 주행: kill 14프레임(1.1%), **숨긴 분기 0**, 게이트 on/off 시 `n_corridors` 완전
동일. 정작 죽는 건 흰색(57.7%)이었다 — 이 트랙은 두 색이 거의 배타적이라(`y_frac` 중앙값
0.983) 게이트가 "다수색 구간의 잔재 청소부" 로 일한다. **노란 분기 진입 구간을 담은 주행이
있어야 B4 를 판정할 수 있다.**

## make_lane_target — 트랙 없이 캘리브를 검증하는 인쇄 타깃

`--check` 는 **차선 두 개가 보이는 직선 구간**을 요구한다. 트랙이 없으면 캘리브를 검증할 수
없고, **검증 안 된 캘리브는 조용히 틀린다** — 스티어링 버그처럼 보이는 종류로. 그래서 트랙
한 조각(검정 노면 + 흰 차선 2줄, 실제와 같은 35cm 폭)을 인쇄한다.

```bash
.venv/bin/python offline/make_lane_target.py --out offline/calib/lane_target.pdf
# 이미 생성돼 있다: offline/calib/lane_target.pdf  (A4 5쪽 = 2x2 타일 + 안내)
```

- **A4 4장(2×2) = 40 × 57.4cm.** 한 장으로는 안 된다 — 차선폭 35cm + 테이프 3cm = **38cm** 가
  필요한데 A4 는 21cm 이고, BEV 는 전방 26~78cm(**깊이 52cm**)를 보는데 A4 세로는 29.7cm 다.
- 타일을 A4(210×297) 가 아니라 **200×287mm** 로 잡아 페이지 중앙에 앉힌다 — 무여백 인쇄가 되는
  프린터는 드물기 때문이다. 재단선까지 자르고 **맞대어** 붙인다(겹치지 말 것).
- **뒷면에서** 테이프로 붙여라. 앞면의 흰 이음새는 차선 픽셀로 검출된다. 검은 이음새는 점선처럼
  보일 뿐이고 슬라이딩 윈도우가 원래 그런 간격을 건너뛴다.
- 재단선·라벨·스케일바는 **회색(V≈100)** 이라 흰색 검출(`white_v_min=185`)에 안 걸린다.

> ⚠ **인쇄 배율이 곧 검증 정확도다.** 프린터가 "용지에 맞춤" 으로 축소하면 차선폭이 35cm 가
> 아니게 되고, `--check` 는 캘리브가 아니라 **프린터를 측정**하게 된다 (체커보드 `square_mm`
> 과 똑같은 함정). **배율 100% / 실제 크기** 로 인쇄하고, 마지막 쪽 **스케일바를 자로 재서
> 100mm 인지 확인**하라. 그리고 **평평하게** 깔아라 — 우글거린 종이는 `H` 가 딛고 선 지면 평면
> 가정을 깬다.

검증: 이 타깃의 기하를 BEV 에 그대로 놓고 `validate` 를 돌리면 폭 오차 **0.00cm**, 평행성
**0.00cm**, 수직성 **0.00cm** 로 통과한다 — 즉 실제 측정에서 나오는 오차는 전부 캘리브(또는
인쇄·조립)의 것이다.

## calibrate — camera.yaml 생성

**K·D 는 렌즈 고유**(재조준·트랙 변경에도 살아남는다), **H 는 마운트 자세**다. 그래서 카메라를
움직였을 때 다시 필요한 건 **지면 사진 1장**뿐이다.

```bash
cd offline

# ⭐ 마운트만 움직였다 (거의 항상 이 경우) — K·D 재사용, H 만 다시 푼다
../.venv/bin/python calibrate.py \
    --from-camera ../D-Racer-Kit/src/config/camera.yaml \
    --ground calib/ground_01.png --square-mm 25.0 --lane-width-cm 35 \
    --out ../D-Racer-Kit/src/config/camera.yaml

# 렌즈/카메라 자체가 바뀌었다 — 체커보드부터
../.venv/bin/python calibrate.py \
    --intrinsics calib/intr --ground calib/ground_00.png \
    --square-mm 25.0 --lane-width-cm 35 --px-per-cm 4.0 --runtime-size 320x240 \
    --out ../D-Racer-Kit/src/config/camera.yaml

# 검증 (직선 구간 프레임으로 폭·평행성·수직성 3가지)
../.venv/bin/python calibrate.py --check ../D-Racer-Kit/src/config/camera.yaml \
    --straight <frame>.png --lane-width-cm 35
```

- `--from-camera` = 기존 `camera.yaml` 에서 **K·D + BEV 격자(`px_per_cm`·축 오프셋·런타임
  해상도)** 를 물려받는다. 저장된 K·D 는 런타임 해상도(320x240) 기준이고 지면 사진은 고해상도
  (960x720)로 찍으므로 **촬영 해상도로 정확히 rescale** 한다(오차 0px). 명시 인자가 있으면 그것이
  이긴다. **검증: 같은 지면 사진에 대해 `--intrinsics` 경로와 같은 K·D·H 를 낸다.**
- `--intrinsics` = 체커보드 사진 폴더 → K·D. 코너 검출 5장 미만이면 종료한다. **`--from-camera`
  와 같이 쓸 수 없다** (K·D 출처는 하나여야 한다).
- `--ground` = 지면에 눕힌 보드 사진 **1장** → 호모그래피 H (`solvePnP`). 카메라 높이 실측은
  **불필요하다** — 산출된다 (`--cam-height-cm` 을 주면 교차검증만 한다).
- 촬영 절차(해상도 올리고 → 찍고 → **원복**)는 [Task command](../Task%20command.md) C-2.

> ⚠ 현재 커밋된 `camera.yaml` 은 **지금의 `calib.py` 로 재현되지 않는다** (`H[2,2]` 가 0.539 인데
> 현재 `build_model` 은 구조상 1.0 만 낸다 — 커밋 전 다른 버전으로 만들어진 파일이다). 실차
> 성공 주행(0712)의 기준선이므로 **덮어쓰기 전에 백업하라.** 참고로 두 파일 모두 0712 재생에서
> "재현 일치" 를 통과하고, 재생성본이 쌍검출 73% → 84% 로 오히려 높다.

> ⚠ `--lateral-cm` 은 파싱만 되고 **어디서도 쓰이지 않는다** (dead flag). 축 오프셋은
> `--axis-offset-cm`.

## 제어기 튜닝은 왜 여기 없나 — open-loop 의 한계

녹화 영상은 **사람이 지난 경로의 카메라 뷰**만 담는다. 컨트롤러가 다르게 조향했다면 차량
pose·뷰·차선 상태가 달라지지만 **그 뷰는 존재하지 않는다**(covariate shift). 따라서 "컨트롤러가
실제로 몰았을 때의 횡오차" 는 오프라인으로 **측정 불가**하고, 녹화의 `center_error` 는 전부 사람
궤적 값이라 랭킹에 못 쓴다. 오프라인으로 볼 수 있는 건 명령 자체의 품질(부드러움·진동·오차를
줄이는 방향인가)뿐이고, **최종 판정은 어차피 실차 폐루프**다.

**제어 튜닝은 실차에서 `ros2 param set /control_node …` 로 한다**
([../Task command.md](../Task%20command.md) B-4).

인지는 다르다 — `dracer_core` 가 같은 코드라 `panel_replay.py --set` 이 차와 **동일한** 결과를
낸다. 인지 파라미터는 여기서 A/B 하고 profile 에 반영하면 된다.

## profile — 오프라인↔온라인 계약

`config/profiles/track2025.yaml` 한 파일.

```yaml
name: track2025
perception: { <perception_core.Cfg 필드>: <값>, ... }   # 실차 live 튜닝값 유지
control:    { controller: PD, kp: 0.45, ... }           # PD | PID
```

**손으로 편집한다.** 자동 writer 는 없다 — PyYAML 이 주석을 보존하지 못하는데, 이 파일의
주석(kp 가 왜 0.45 인지, steer_max 가 왜 0.7 인지)이 자동 재작성보다 가치가 크다.

profile YAML 은 git 추적 → **git 으로** D3-G 에 전달. 녹화(mp4/csv)는 미추적 → **scp** 로.
상세 흐름은 [../Task command.md](../Task%20command.md).
</content>
