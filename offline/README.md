# 오프라인 도구

로컬(macOS, ROS 불필요, 레포 `.venv`)에서 도는 분석·튜닝 도구. 코어(`dracer_core`)가 순수
파이썬이라 **차가 돌리는 것과 같은 알고리즘**을 그대로 돌린다.

| 파일 | 역할 |
|---|---|
| **`panel_replay.py`** | 주행 raw → 4패널 재구성 + 실차 csv 대조 + 파라미터 A/B |
| `calibrate.py` | 체커보드·지면 사진 → `camera.yaml` (K/D + 호모그래피 H) |
| `_common.py` | 영상/CSV/profile IO 헬퍼 (`panel_replay`·`calibrate` 공용) |

주행/캘리브 세션 데이터(mp4 + csv + png)는 git 미추적이라 scp 로 옮긴다 (`rslt/`, `calib/`).

최초 1회 설치: `.venv/bin/pip install -e D-Racer-Kit/src/dracer_core`

---

## panel_replay — 주행 raw 를 4패널로 되살린다

`drive.launch` 는 패널을 녹화하지 않는다: 패널 합성 + JPEG 인코딩이 검출의 **4배**를 먹어서
그게 프레임 드랍의 진범이었다. 대신 **raw 카메라 + LaneState csv** 를 남기고, 그 둘로 패널을
여기서 사후에 되살린다. 같은 `dracer_core` 파이프라인을 그대로 돌리므로 재구성 결과는 보드가
봤을 화면과 같다.

```bash
.venv/bin/python offline/panel_replay.py <raw>.mp4 --camera <camera.yaml> \
    --profile <track.yaml> --csv <drive>.csv
```

`--csv` 를 주면 실차가 그때 발행한 LaneState 와 **프레임 단위로 대조**한다 (`|Δcenter_error|`
중앙값·p90·최대). **중앙값 < 0.02 면 재현 일치** — 오프라인 튜닝을 믿어도 되는지에 대한
검증이다. `--set key=value` 로 프로파일을 안 건드리고 파라미터를 A/B 한다.

인지 파라미터는 `dracer_core` 가 차와 같은 코드라 여기서 A/B 한 결과가 차와 동일하다. 반면
**제어기 튜닝은 오프라인으로 판정할 수 없다** — 녹화 영상은 사람이 지난 경로의 뷰만 담으므로
다르게 조향한 컨트롤러가 봤을 프레임은 존재하지 않는다(covariate shift). 오프라인으로 보이는
건 명령 자체의 품질뿐이고, 제어 최종 판정은 실차 폐루프다.

## calibrate — camera.yaml 생성

**K·D 는 렌즈 고유**(재조준·트랙 변경에도 살아남는다), **H 는 마운트 자세**다. 그래서 카메라를
움직였을 때 다시 필요한 건 **지면 사진 1장**뿐이다.

```bash
# 마운트만 움직였다 — K·D 재사용, H 만 다시 푼다
.venv/bin/python offline/calibrate.py --from-camera <camera.yaml> \
    --ground <ground.png> --square-mm 25.0 --lane-width-cm 35 --out <camera.yaml>

# 검증 — 직선 구간 프레임으로 폭·평행성·수직성
.venv/bin/python offline/calibrate.py --check <camera.yaml> \
    --straight <frame>.png --lane-width-cm 35
```

- `--from-camera` 는 기존 `camera.yaml` 에서 K·D + BEV 격자(`px_per_cm`·축 오프셋·런타임
  해상도)를 물려받는다. 저장된 K·D 는 런타임 해상도(320x240) 기준이고 지면 사진은 고해상도로
  찍으므로 촬영 해상도로 정확히 rescale 한다.
- `--intrinsics <폴더>` 는 렌즈/카메라 자체가 바뀌었을 때만 쓴다 (체커보드 사진 → K·D).
  이때는 `--runtime-size 320x240` 을 반드시 같이 줘라 — 안 주면 비등방 리스케일이 저장된다.
- `--ground` 지면 사진은 5장을 찍어 각각 풀고 서로 대조하라. 보드가 멀면(near > 48cm) 근거리
  `H` 가 무너지는데 그 사진도 혼자 보면 "카메라 높이 정합, RMS 양호" 로 통과한다 — 다섯 지점의
  차선폭이 거리와 무관하게 **일정한지(변동)** 가 유일한 판정 기준이다.
- 보드가 지면에서 떴으면 `--board-offset-cm` 으로 알려줘라. 안 알려주면 `H` 가 그 높이의 유령
  평면으로의 사영이 되고 숫자는 전부 멀쩡해 보인다. 교차검증은 산출된 카메라 높이 vs 실측.

`--check` 판정 기준은 `CameraModel.validate(tol=0.20)` — 차선폭 오차·평행성·수직성이 각각
차선폭의 20%(7cm) 이내. 다만 그건 합격선이지 목표가 아니다. 잘 된 캘리브는 폭 오차 1~2cm,
평행성 1cm 미만이다.

## profile — 오프라인↔온라인 계약

`config/profiles/track.yaml` 한 파일. `perception:`(perception_core.Cfg 필드)와
`control:`(PD/PID 게인)로 나뉜다. **손으로 편집한다** — PyYAML 이 주석을 보존하지 못하는데, 이
파일의 주석(어떤 근거가 그 값을 정했는지)이 자동 재작성보다 가치가 크다. profile YAML 은 git
추적 → git 으로 D3-G 에 전달, 녹화(mp4/csv)는 미추적 → scp.
