# Mission-object detection, published by perception_node (dracer_core.mission_core).
#
# ONE class per frame. The detector takes the highest-confidence detection of the
# frame and votes it through an M-of-N debounce; `cls` is the CONFIRMED result, which
# is what a consumer should act on. `det_*` is the raw top detection of THIS frame --
# it is what you draw a box around and what tells you WHY cls is (or is not) what it
# is. They disagree on purpose: cls is sticky, det_* is instantaneous.
std_msgs/Header header

int8    cls              # confirmed class, or -1 = nothing confirmed
                         #   0 GREEN  traffic light, go
                         #   1 RED    traffic light, stop
                         #   2 MARK   dynamic obstacle (ArUco id 3)
                         #   3 RIGHT  direction sign, arrow right
                         #   4 LEFT   direction sign, arrow left
bool    newly_confirmed  # cls changed on THIS frame (edge, for logging/latching)

int8    det_cls          # top raw detection this frame, or -1 = none
float32 det_conf         # its confidence, [0, 1]  (ArUco is deterministic -> 1.0)
uint16  det_x            # its bbox in CAMERA pixels (not BEV: a traffic light is
uint16  det_y            # above the ground plane and projects nowhere on it)
uint16  det_w
uint16  det_h
