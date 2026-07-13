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

## calibrate — camera.yaml 생성

```bash
cd offline
../.venv/bin/python calibrate.py \
    --intrinsics shots/intr --ground shots/ground.png \
    --square-mm 25.0 --lane-width-cm 35 --px-per-cm 4.0 \
    --out ../D-Racer-Kit/src/config/camera.yaml

# 검증만 (직선 구간 프레임으로 수직성·평행성·폭 3가지 확인)
../.venv/bin/python calibrate.py --check ../D-Racer-Kit/src/config/camera.yaml \
    --straight <frame>.png --lane-width-cm 35
```

- `--intrinsics` = 체커보드 사진 폴더 → K/D. 코너 검출 5장 미만이면 종료한다.
- `--ground` = 지면에 눕힌 보드 사진 **1장** → 호모그래피 H (`solvePnP`). 카메라 높이 실측은
  **불필요하다** — 산출된다.
- **카메라 마운트를 움직이면 H 가 무효**가 된다. K/D 는 살아남으므로 `--ground` 만 다시 찍으면
  된다.

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
