#!/usr/bin/env python3
"""Export a CompressedImage topic from a rosbag2 (sqlite3 .db3) to MP4/PNG.

The .db3 is CDR-serialized binary, so it cannot be viewed directly. This decodes
the JPEG frames and writes an MP4 (and, optionally, individual PNG frames) that
you can open in any video player.

Two backends are supported and auto-selected:
  * rosbag2_py  - on the D3-G / any ROS2 Humble machine (no extra install).
  * rosbags     - pure Python, NO ROS2 needed (works on macOS/Windows).
                  install:  pip install rosbags opencv-python numpy

Usage:
    # D3-G:
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 scripts/bag_to_video.py bagfile/bag_20260701_085726

    # Local laptop (no ROS2), after pip install rosbags opencv-python numpy:
    python3 scripts/bag_to_video.py D-Racer-Kit/bagfile/bag_20260701_135527

    # options:
    python3 scripts/bag_to_video.py <bag_dir> --topic /camera/image/compressed \
        --output out.mp4 --frames-dir frames --fps auto --backend auto

Notes
    * Playback FPS defaults to the real recorded rate (from bag timestamps);
      pass --fps <N> to force a fixed rate.
    * Pass either the bag DIRECTORY (containing metadata.yaml) or a .db3 file.
"""
import argparse
import os
import sys

import cv2
import numpy as np


def resolve_bag_dir(path):
    # rosbag2 wants the directory that holds metadata.yaml.
    if path.endswith('.db3'):
        return os.path.dirname(os.path.abspath(path))
    return os.path.abspath(path)


def read_with_rosbag2_py(bag_dir, topic):
    """Yield (jpeg_bytes, t_ns) using the ROS2 rosbag2_py backend."""
    import rclpy.serialization
    import rosbag2_py
    from sensor_msgs.msg import CompressedImage

    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage, conv)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    _check_topic(topic, types)
    while reader.has_next():
        t_topic, data, t_ns = reader.read_next()
        if t_topic != topic:
            continue
        msg = rclpy.serialization.deserialize_message(data, CompressedImage)
        yield bytes(msg.data), t_ns


def read_with_rosbags(bag_dir, topic):
    """Yield (jpeg_bytes, t_ns) using the pure-Python rosbags backend."""
    from pathlib import Path
    from rosbags.highlevel import AnyReader

    # rosbag2 bags don't embed type definitions, so supply a ROS2 Humble
    # typestore (covers sensor_msgs/CompressedImage and other standard types).
    kwargs = {}
    try:
        from rosbags.typesys import Stores, get_typestore
        kwargs['default_typestore'] = get_typestore(Stores.ROS2_HUMBLE)
    except Exception:  # noqa: BLE001  (older rosbags: ships types by default)
        pass

    with AnyReader([Path(bag_dir)], **kwargs) as reader:
        types = {c.topic: c.msgtype for c in reader.connections}
        _check_topic(topic, types)
        conns = [c for c in reader.connections if c.topic == topic]
        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            yield bytes(bytearray(msg.data)), t_ns


def _check_topic(topic, types):
    if topic not in types:
        sys.exit(f'topic {topic} not in bag. Available:\n  ' + '\n  '.join(sorted(types)))
    if 'CompressedImage' not in types[topic]:
        print(f'WARNING: {topic} is {types[topic]}, expected CompressedImage.', file=sys.stderr)


def pick_backend(name):
    if name == 'rosbag2_py':
        return read_with_rosbag2_py
    if name == 'rosbags':
        return read_with_rosbags
    # auto: prefer ROS2 if present, else pure-python.
    try:
        import rosbag2_py  # noqa: F401
        return read_with_rosbag2_py
    except ImportError:
        pass
    try:
        import rosbags  # noqa: F401
        return read_with_rosbags
    except ImportError:
        sys.exit('No backend available. Install one:\n'
                 '  ROS2:  source /opt/ros/humble/setup.bash\n'
                 '  local: pip install rosbags opencv-python numpy')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='bag directory or .db3 file')
    parser.add_argument('--topic', default='/camera/image/compressed')
    parser.add_argument('--output', default=None, help='output mp4 path (default: <bag>_<topic>.mp4)')
    parser.add_argument('--frames-dir', default=None, help='also dump PNG frames here')
    parser.add_argument('--fps', default='auto', help='"auto" (from timestamps) or a number')
    parser.add_argument('--backend', default='auto', choices=['auto', 'rosbag2_py', 'rosbags'])
    args = parser.parse_args()

    bag_dir = resolve_bag_dir(args.bag)
    if not os.path.isdir(bag_dir):
        sys.exit(f'bag directory not found: {bag_dir}')

    read = pick_backend(args.backend)

    out_path = args.output or f'{bag_dir.rstrip("/")}_{args.topic.strip("/").replace("/", "_")}.mp4'
    if args.frames_dir:
        os.makedirs(args.frames_dir, exist_ok=True)

    frames = []       # decoded BGR frames
    stamps_ns = []    # bag receive timestamps
    for jpeg, t_ns in read(bag_dir, args.topic):
        buf = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            continue
        frames.append(frame)
        stamps_ns.append(t_ns)

    if not frames:
        sys.exit(f'No decodable frames on {args.topic}.')

    # Determine FPS.
    if args.fps == 'auto':
        if len(stamps_ns) >= 2:
            span_s = (stamps_ns[-1] - stamps_ns[0]) / 1e9
            fps = (len(stamps_ns) - 1) / span_s if span_s > 0 else 30.0
        else:
            fps = 30.0
    else:
        fps = float(args.fps)
    fps = max(1.0, round(fps, 2))

    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    if not writer.isOpened():
        sys.exit(f'Failed to open VideoWriter for {out_path} (codec/permission issue).')

    for i, frame in enumerate(frames):
        if frame.shape[:2] != (h, w):
            frame = cv2.resize(frame, (w, h))
        writer.write(frame)
        if args.frames_dir:
            cv2.imwrite(os.path.join(args.frames_dir, f'frame_{i:05d}.png'), frame)
    writer.release()

    print(f'backend   : {read.__name__}')
    print(f'topic     : {args.topic}')
    print(f'frames    : {len(frames)}  ({w}x{h})')
    print(f'fps       : {fps}')
    print(f'mp4 saved : {out_path}')
    if args.frames_dir:
        print(f'frames dir: {args.frames_dir}')


if __name__ == '__main__':
    main()
