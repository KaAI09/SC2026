# Lane perception state published by perception_node, consumed by driving_node.
# Mirrors the driving_core.lane_core pipeline output. Floats that can be absent
# are set to NaN and gated by `valid` (center_error) / heading_valid.
std_msgs/Header header

bool    valid            # center_error is available this frame
float32 center_error     # lateral error, normalized [-1, 1], + = right (NaN if !valid)
float32 ema              # temporally smoothed center_error (NaN if none yet)

bool    heading_valid    # heading is available this frame
float32 heading          # heading estimate (deg or unitless per heading_label)
string  heading_label    # slope / two_point / norm_slope / hough

float32 confidence       # valid scan rows / total rows, [0, 1]
float32 left_conf         # left-line detection rate, [0, 1]
float32 right_conf        # right-line detection rate, [0, 1]

bool    has_curvature    # curvature computed this frame
float32 curvature        # relative curvature (NaN if !has_curvature)

string  state            # OK / LOW_CONF / OUTLIER / HOLD / LOST
bool    used_fallback    # adaptive-threshold fallback was used
