# Lane perception state published by perception_node, consumed by control_node.
# Mirrors the dracer_core.perception_core pipeline output. Floats that can be absent
# are set to NaN and gated by `valid` (center_error) / heading_valid.
std_msgs/Header header

bool    valid            # center_error is available this frame
float32 center_error     # lateral error, normalized [-1, 1], + = right (NaN if !valid)
float32 ema              # temporally smoothed center_error (NaN if none yet)

bool    heading_valid    # heading is available this frame
float32 heading          # heading estimate (deg or unitless per heading_label)
string  heading_label    # perception method label (e.g. 'ego')

float32 confidence       # detection confidence, [0, 1]
float32 left_conf         # left-line detection rate, [0, 1]
float32 right_conf        # right-line detection rate, [0, 1]

bool    has_curvature    # curvature computed this frame
float32 curvature        # relative curvature (NaN if !has_curvature)

string  state            # OK / LOW_CONF / OUTLIER / HOLD / LOST
bool    used_fallback    # coast (single-lane) fallback was used

# --- branch evidence (recorded, NOT acted on) -------------------------------
# There is no judgment layer yet. These two say what the frame SAW and which placeholder
# rule picked a route, so the layer can be designed from measurements instead of guesses.
uint8   n_corridors      # physically valid corridors this frame (35cm +-25%, real overlap).
                         # > 1 = a BRANCH: more than one route is geometrically available.
string  ego_rule         # which rule chose the ego corridor:
                         #   tracked - the pair the Tracker was already following (identity)
                         #   nearest - closest corridor to the vehicle axis  <- PLACEHOLDER.
                         #             at a branch this picks the route BY PROXIMITY, i.e.
                         #             by accident. This is where route logic goes.
                         #   coast   - one boundary + an assumed lane width
                         #   none    - no centreline this frame

# --- fork evidence (12-label judgment) --------------------------------------
string  fork_type        # '' | island(W-W (L,R)) | branch(그 외 분기). 감지, 회피는 use_fork
uint8   n_islands        # 이 프레임 island 로 감지된 corridor 수
