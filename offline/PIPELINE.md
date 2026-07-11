# 오프라인 도구 파이프라인

로컬(macOS, ROS 불필요, 레포 `.venv`)에서 도는 도구들. 코어(`dracer_core`)가
순수 파이썬이라 온라인 노드와 **같은 알고리즘**을 오프라인에서 그대로 돌린다.

## 구성

| 파일 | 역할 | 입력 | 출력 |
|---|---|---|---|
| **`panel_replay.py`** ⭐ | **주행 raw → 4패널 재구성 + 실차 csv 대조 + 파라미터 A/B** | raw 영상 + camera + profile | 패널 mp4 + 집계 지표 |
| `calibrate.py` | 카메라 K/D + 지면 호모그래피 H → `camera.yaml` | 체스보드 사진 | `camera.yaml` |
| `perception_probe.py` | 인지 실험/시각화 | 영상 | 다패널 시각화 |
| `control_predict.py` | 제어기 open-loop 예측 | drive 영상 + 수동 CSV + **camera** + profile | `rslt/pred_*.csv` |
| `control_select.py` | 제어 지표 랭킹 + profile `[control]` export | pred CSV | profile 갱신 |
| `_common.py` | 영상/CSV/profile IO 헬퍼 | — | — |

## panel_replay — 주력 도구

`drive.launch` 는 패널을 녹화하지 않는다: 패널 합성 + JPEG 인코딩이 검출의 4배를 먹어서 그게
프레임 드랍의 진범이었다(§7.8). 대신 **raw 카메라 + LaneState csv** 를 남기고, 그 둘로 패널을
**여기서, 사후에** 되살린다. 차는 주행 중 렌더링 비용을 한 푼도 내지 않는다.

같은 `dracer_core` 파이프라인을 그대로 돌리므로 재구성 결과는 보드가 봤을 화면과 **같다**.

```bash
cd ~/workspace/SC2026
.venv/bin/python offline/panel_replay.py offline/rslt/recorder/raw/drive_<stamp>.mp4 \
    --camera D-Racer-Kit/src/config/camera.yaml \
    --profile D-Racer-Kit/src/config/profiles/track2025.yaml \
    --csv offline/rslt/recorder/csv/drive_<stamp>.csv

# 파라미터 A/B (원본 profile 은 안 건드린다)
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera ... --profile ... \
    --set lane_width_cm=35 --set branch_policy=random --no-video
```

`--csv` 를 주면 실차가 그때 발행한 LaneState 와 **프레임 단위로 대조**한다 — 오프라인 튜닝을
믿어도 되는지에 대한 검증이다.

## 인지 (perception)

온라인 인지는 확정된 단일 파이프라인(`dracer_core.perception_core`)이고, **live
파라미터 튜닝은 실차에서 `ros2 param set /perception_node …` + monitor(:5000)**로
한다([../PERCEPTION.md](../PERCEPTION.md)). `perception_probe.py`는 그와 별개로
**BEV 실험·시각화용 오프라인 도구**다(향후 캘리브레이션 BEV 개발 기준). 자동
profile export는 없다 — `[perception]` 섹션은 손으로/실차 튜닝값으로 유지한다.

```bash
cd offline
../.venv/bin/python perception_probe.py <raw>.mp4 --stages --name t1
```

## 제어 (control) — 여러 컨트롤러 비교

open-loop(폐루프 궤적지표는 covariate shift로 불가): 영상에 인지를 재실행해 프레임별
lane state를 만들고, 각 후보 컨트롤러(C1~C5)를 스텝해 명령을 예측 → 지표로 랭킹 →
이긴 컨트롤러의 게인을 profile `[control]`에 기록.

```bash
cd offline
../.venv/bin/python control_predict.py <drive>.mp4 --csv <drive>.csv \
    --camera ../D-Racer-Kit/src/config/camera.yaml \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml --controllers C1,C2,C3,C4,C5
../.venv/bin/python control_select.py rslt/pred_<drive>.csv --export C2 \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml
```

> ⚠ **`--camera` 는 필수다.** 예전에는 cam 없이 `LanePipeline(cfg)` 를 불러서 **차가 쓰지 않는
> front-view 파이프라인**으로 제어를 예측했고, `control_select` 가 그 예측으로 제어기를 골랐다.
> 모드가 아니라 버그였다. front-view 경로는 삭제됐고 `CameraModel` 이 필수다.

> ⚠ **C4/C5 는 지금 공정하게 비교할 수 없다.** `lookahead`·`pp_gain`·`stanley_k` 등이
> `control_node` 의 ROS 파라미터로 노출돼 있지 않아 실차에서 튜닝할 방법이 없다
> ([../ROLLBACK.md](../ROLLBACK.md) TODO B2).

## profile — 오프라인↔온라인 계약

`config/profiles/track2025.yaml` 한 파일. `control_select`가 `[control]` 섹션만
in-place 교체(나머지 보존). 노드는 로드 시:
`cfg_from_profile(perception)` / `make_ctrl(controller, **control)`.

```yaml
perception: { <perception_core.Cfg field>: <value>, ... }   # 실차 live 튜닝값 유지
control:     { controller: C2, kp: 0.5, ... }               # control_select가 기록
```

profile YAML은 git 추적 → git으로 D3-G에 전달. 녹화(mp4/csv)는 미추적 → scp.
상세 흐름은 [../Task command.md](../Task%20command.md).
