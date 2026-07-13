std_msgs/Header header
Control control_msg
float32 accel_ratio
# 조향 서보의 중립 펄스. joystick_node 가 Y/B 로 조정하고 여기 실어 보내면 actuator 가
# 서보에 즉시 반영한다 (accel_ratio 와 같은 패턴 -- 캘리브값이 조이스틱에서 나온다).
#
# 이것이 STEER_TRIM 을 대체한다. 예전에는 트림을 조향 '명령' 에 더했고(joystick 은 수동
# 경로에서, actuator 는 자율 경로에서 각자), 그래서 명령 u 와 트림이 같은 [-1,1] 서보
# 예산을 나눠 써서 한쪽 조향 권한이 조용히 깎였다. 트림은 서보의 중립이지 명령의 일부가
# 아니다 -- 중립으로 옮기면 u 는 +-1.0 전체를 대칭으로 쓴다.
float32 steer_center_us
bool e_stop_en
bool is_recording
bool engage
