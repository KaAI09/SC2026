# 오프라인 도구 파이프라인

로컬(macOS, ROS 불필요, 레포 `.venv`)에서 도는 도구들. 코어(`dracer_core`)가
순수 파이썬이라 온라인 노드와 **같은 알고리즘**을 오프라인에서 그대로 돌린다.

## 구성

| 파일 | 역할 | 입력 | 출력 |
|---|---|---|---|
| `perception_probe.py` | 인지 실험/시각화 (BEV 실험 포함) | 영상 | 6패널 시각화 |
| `control_predict.py` | 제어기 open-loop 예측 | drive 영상 + 수동 CSV + profile | `rslt/pred_*.csv` |
| `control_select.py` | 제어 지표 랭킹 + profile `[control]` export | pred CSV | profile 갱신 |
| `_common.py` | 영상/CSV/profile IO 헬퍼 | — | — |

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
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml --controllers C1,C2,C3,C4,C5
../.venv/bin/python control_select.py rslt/pred_<drive>.csv --export C2 \
    --profile ../D-Racer-Kit/src/config/profiles/track2025.yaml
```

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
