# 현재 활성 제어 파라미터 (latched). recorder 가 세션 사이드카로 스냅샷한다.
# 사이드카 .meta.json 스키마와 1:1.
string  controller
float32 kp
float32 kd
float32 throttle_base
float32 throttle_min
float32 slew_rate_per_sec
float32 steer_max
float32 conf_gate
float32 curv_slow
float32 x_half_cm      # center_error 정규화 스케일 태그 (calib 의존)
