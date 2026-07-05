# Monitor Package 가이드

<br>

## 1) Monitor 패키지가 하는 일
`monitor` 패키지는 핵심 상태만 웹 대시보드로 보여주는 **경량 모니터**입니다.
트랙 테스트 중 지연을 줄이기 위해 구독 토픽을 최소화했습니다.

핵심 기능(3종만):
- 실시간 메인 카메라 스트림 표시
- 배터리 잔량 표시
- 저장장치 사용량 표시

> 제어값/녹화 상태/ROS 노드·토픽 그래프/OpenCV 디버그 영상 패널은
> 지연 최소화를 위해 **제거**되었습니다. (구독 토픽 2개 + 저장공간 폴링 타이머만 사용)

<br>


## 2) 동작 구조 한눈에 보기
`monitor_node`는 다음 순서로 동작합니다.

1. ROS 파라미터와 `vehicle_config.yaml`을 읽음
2. 카메라(`image_topic`)·배터리(`battery_topic`) 토픽만 subscribe, 저장공간은 타이머로 폴링
3. 수신 데이터를 `MonitorState`에 저장
4. 내장 Flask 서버가 `/api/status`, `/api/frame`, `/api/frame/placeholder` API로 상태/이미지를 제공
5. 웹 페이지(`index.html` + `app.js`)가 API를 주기적으로 호출해 화면 갱신

<br>

## 3) monitor_node 구동 방법
### 3-1. 빌드
```bash
cd /home/topst/SC2026/D-Racer-Kit
colcon build --packages-select monitor
source install/setup.bash
```

### 3-2. 기본 실행
```bash
ros2 run monitor monitor_node
```

> 파이프라인에서는 `calibrate.launch.py`(Launch 1)가 `monitor_node`와
> `battery_node`를 함께 띄웁니다. 배터리 패널이 값을 보이려면 `battery_node`가
> `/battery_status`를 publish 해야 하므로, 웹 모니터를 쓰는 유일한 런치인 Launch 1에
> battery_node가 포함되어 있습니다.

### 3-3. 호스트/포트 변경 실행 예시
```bash
ros2 run monitor monitor_node --ros-args \
  -p web_host:=0.0.0.0 \
  -p web_port:=5000
```
혹은, `config/vehicle_config.yaml` 파일에서 수정 후 적용해도 동일합니다.
브라우저 접속 주소는 로그의 `web=http://...` 값을 확인하면 됩니다.(사용자의 IP)

<br>

## 4) 주요 ROS 파라미터 설명
아래 파라미터는 `monitor_node`에서 직접 사용됩니다.

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `vehicle_config_file` | string | 자동 탐색 경로 | YAML 설정 파일 경로 |
| `battery_topic` | string | `battery_status` | 배터리 토픽 |
| `image_topic` | string | `''`→YAML/`/camera/image/compressed` | 메인 카메라 이미지 토픽 (`sensor_msgs/CompressedImage`). launch override 우선 |
| `storage_path` | string | `/` | 저장공간 사용량 계산 대상 경로 |
| `storage_poll_interval_sec` | float | `1.0` | 저장공간 갱신 주기(초) |
| `web_host` | string | `0.0.0.0` | Flask 바인딩 호스트 |
| `web_port` | int | `5000` | Flask 포트 |
| `page_title` | string | `D-Racer Monitor` | 대시보드 페이지 제목 |
| `refresh_interval_ms` | int | `1000` | 상태 API polling 주기(ms) |
| `image_refresh_interval_ms` | int | `100` | 이미지 갱신 주기(ms) |
| `stale_timeout_sec` | float | `3.0` | 데이터 stale 판단 기준 시간(초) |
| `image_source_width` | int | `160` | 원본 이미지 폭 fallback |
| `image_source_height` | int | `120` | 원본 이미지 높이 fallback |
| `image_display_width` | int | `160` | placeholder 표시 폭 |
| `image_display_height` | int | `120` | placeholder 표시 높이 |
| `debug_log` | bool | `false` | 배터리/이미지 디버깅 로그 출력 여부 |

참고:
- 일부 값은 `vehicle_config.yaml`의 키(`BATTERY_TOPIC`, `IMAGE_TOPIC`, `STORAGE_PATH`, `WEB_HOST`, `WEB_PORT`, `IMAGE_DISPLAY_WIDTH/HEIGHT`)로 덮어쓸 수 있습니다.

<br>


## 5) 입력 토픽 요약
`monitor_node`가 subscribe하는 토픽(2개):
- 배터리: `battery_topic` (`/battery_status`)
- 메인 카메라: `image_topic` (`/camera/image/compressed`)

저장공간은 토픽이 아니라 `storage_poll_interval_sec` 주기의 로컬 디스크 폴링으로 갱신합니다.

<br>


## 6) 문제 해결 체크리스트
- 카메라 화면이 placeholder로만 보일 때: `image_topic`이 실제 publish 중인지 (`ros2 topic hz <topic>`) 확인.
- 배터리가 계속 `WAITING`일 때: `battery_node`가 실행 중이고 `/battery_status`를 publish 하는지 확인
  (파이프라인에서는 `calibrate.launch.py`에 포함).
- 웹 접속이 안 될 때: `monitor_node` 실행 로그의 `web=http://...` 주소/포트가 기대값과 일치하는지 확인.

유용한 확인 명령:
```bash
ros2 topic hz /camera/image/compressed
ros2 topic echo /battery_status --once
ros2 param get /monitor_node web_port
```
