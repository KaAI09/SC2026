"""Launch 2/4 — collect: manual drive + perception + RAW recording. DATA COLLECTION.

Replaces the old `record` (manual + raw video) and `perceive` (manual + live 4-panel).
They were one job split in two, and the split stopped making sense the moment the
panel moved offline: `record` produced frames nobody had run perception on, and
`perceive` paid the live-render cost to show a human something a laptop could render
later, for free, at any resolution.

So `collect` DOES NOT RENDER. Perception runs (that is the point -- the csv carries a
real LaneState per frame), but its debug overlay is off and the monitor shows the RAW
camera, which is all a human needs while walking the car around a track: where am I.
The 4-panel view is reconstructed offline from raw/ + csv/ (`offline/panel_replay.py`)
on a machine with cycles to spare.

Recording is the joystick START toggle, as everywhere. One START->STOP cycle writes
one session, sharing one basename:

    <record_dir>/raw/collect_<stamp>.mp4    RAW camera, never annotated
    <record_dir>/csv/collect_<stamp>.csv    per-frame LaneState + manual command

`calibration_mode` is OFF on purpose. Trim belongs to `calibrate`: if it moves during
a recording, the steering column of that csv changes meaning halfway through the file.

    ros2 launch dracer_bringup collect.launch.py
    ros2 launch dracer_bringup collect.launch.py record_dir:=$HOME/recorder
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dracer_bringup.launch_common import (battery_node, core_nodes, default_camera_path,
                                          default_profile_path, default_record_dir,
                                          monitor_node, vehicle_config_path)


def generate_launch_description():
    vehicle_config = vehicle_config_path()
    profile = LaunchConfiguration('profile')
    camera = LaunchConfiguration('camera')
    record_dir = LaunchConfiguration('record_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value=default_camera_path(),
            description='camera calibration YAML -> metric BEV. REQUIRED: the pipeline '
                        'raises ValueError without it (front-view path was removed)'),
        DeclareLaunchArgument('profile', default_value=default_profile_path(),
                              description='driving profile YAML ([perception] applied)'),
        DeclareLaunchArgument('record_dir', default_value=default_record_dir(),
                              description='directory for recorder mp4/csv output'),
        DeclareLaunchArgument('mission_config', default_value='',
                              description='venue mission YAML from scripts/mission_tune.py '
                                          '(traffic-light HSV retuned under the real LEDs)'),
        *core_nodes(
            vehicle_config,
            calibration_mode=False,     # trim/accel edits belong to calibrate.launch
            use_joystick_control=True,  # the human drives
        ),
        # RAW camera. The operator needs to see the road, not the pipeline.
        monitor_node(vehicle_config, '/camera/image/compressed'),
        battery_node(),
        Node(
            package='perception', executable='perception_node', name='perception_node',
            output='screen',
            # publish_debug false: belt and braces. Perception already renders only when
            # something subscribes to the debug topic, and here nothing does -- but saying
            # so in the launch is what makes this launch's identity legible.
            #
            # Mission detection IS on. It costs ~3ms every third frame and it is the only
            # reason the csv can answer the question this launch exists to answer: what did
            # the car SEE, and where was the branch when it saw it. Both now carry the same
            # frame stamp, because they are the same frame.
            parameters=[{'profile': profile, 'camera': camera,
                         'publish_debug': False,
                         'use_mission': True,
                         'mission_config': LaunchConfiguration('mission_config')}],
        ),
        Node(
            package='recorder', executable='recorder_node', name='recorder_node',
            output='screen',
            # image_topic = the raw camera and raw_topic empty -> recorder treats raw as the
            # MAIN stream and writes raw/ + csv/ only. Raw frames are never stamped, so the
            # pixels offline perception and camera calibration read stay untouched.
            parameters=[{'record_dir': record_dir,
                         'image_topic': '/camera/image/compressed',
                         'raw_topic': '',
                         'name_prefix': 'collect'}],
        ),
    ])
