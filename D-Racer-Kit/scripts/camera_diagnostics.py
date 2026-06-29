#!/usr/bin/env python3
"""Measure camera stream characteristics on the D3-G and save a report.

Subscribes to the compressed image topic and measures, over a fixed window:
  - actual decoded resolution (width x height x channels)
  - effective publish rate (FPS) and inter-frame interval jitter
  - end-to-end-ish latency: receive_time - header.stamp (same machine clock)
  - per-frame JPEG size and estimated bandwidth

The summary is written to <output_dir>/camera_diag_<YYYYmmdd_HHMMSS>.txt.
Optionally records `v4l2-ctl --list-formats-ext` for the camera device.

Run on the D3-G AFTER the camera node is up, e.g.:
    ros2 run camera camera_node            # terminal A
    python3 scripts/camera_diagnostics.py  # terminal B
    # or, with options:
    python3 scripts/camera_diagnostics.py --duration 15 --device /dev/video1
"""
import argparse
import statistics
import subprocess
import sys
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class CameraDiagnostics(Node):
    def __init__(self, topic, duration):
        super().__init__('camera_diagnostics')
        self.duration = float(duration)
        # Match the camera_node publisher QoS (RELIABLE / VOLATILE / KEEP_LAST 10).
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.topic = topic

        self.recv_times = []       # seconds (monotonic-ish ros clock)
        self.latencies_ms = []
        self.sizes_bytes = []
        self.resolution = None     # (w, h, ch)
        self.start_time = None
        self.get_logger().info(
            f'Listening on {topic} for {self.duration:.1f}s. Make sure the camera node is running.'
        )

    def on_image(self, msg: CompressedImage):
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now_s

        self.recv_times.append(now_s)
        self.sizes_bytes.append(len(msg.data))

        stamp = msg.header.stamp
        stamp_s = stamp.sec + stamp.nanosec / 1e9
        if stamp_s > 0:
            self.latencies_ms.append((now_s - stamp_s) * 1e3)

        if self.resolution is None:
            try:
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                if frame is not None:
                    if frame.ndim == 2:
                        h, w = frame.shape
                        ch = 1
                    else:
                        h, w, ch = frame.shape
                    self.resolution = (w, h, ch)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'Failed to decode frame for resolution: {exc}')

        if now_s - self.start_time >= self.duration:
            raise KeyboardInterrupt


def summarize(node):
    lines = []
    n = len(node.recv_times)
    if n < 2:
        lines.append('NOT ENOUGH FRAMES RECEIVED. Is the camera node running and publishing?')
        lines.append(f'frames_received={n}')
        return '\n'.join(lines)

    elapsed = node.recv_times[-1] - node.recv_times[0]
    fps = (n - 1) / elapsed if elapsed > 0 else float('nan')
    intervals_ms = [
        (node.recv_times[i] - node.recv_times[i - 1]) * 1e3 for i in range(1, n)
    ]

    def stat_block(name, values, unit):
        if not values:
            return f'{name}: (no data)'
        s = sorted(values)
        p95 = s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]
        return (
            f'{name} [{unit}]: '
            f'mean={statistics.mean(values):.2f} '
            f'median={statistics.median(values):.2f} '
            f'min={min(values):.2f} max={max(values):.2f} '
            f'p95={p95:.2f}'
        )

    res = node.resolution
    res_str = f'{res[0]}x{res[1]} (channels={res[2]})' if res else 'unknown'
    mean_size = statistics.mean(node.sizes_bytes)
    bandwidth_mbps = mean_size * 8 * fps / 1e6

    lines.append(f'topic                 : {node.topic}')
    lines.append(f'window_seconds        : {elapsed:.2f}')
    lines.append(f'frames_received       : {n}')
    lines.append(f'decoded_resolution    : {res_str}')
    lines.append(f'effective_fps         : {fps:.2f}')
    lines.append(stat_block('inter_frame_interval', intervals_ms, 'ms'))
    lines.append(stat_block('latency(recv-stamp)', node.latencies_ms, 'ms'))
    lines.append(stat_block('jpeg_size', node.sizes_bytes, 'bytes'))
    lines.append(f'mean_jpeg_size_bytes  : {mean_size:.0f}')
    lines.append(f'estimated_bandwidth   : {bandwidth_mbps:.2f} Mbps')
    if not node.latencies_ms:
        lines.append('NOTE: header.stamp was empty/zero; latency not measured.')
    return '\n'.join(lines)


def v4l2_formats(device):
    try:
        out = subprocess.run(
            ['v4l2-ctl', '--list-formats-ext', '-d', device],
            capture_output=True, text=True, timeout=10,
        )
        if out.returncode == 0:
            return out.stdout
        return f'(v4l2-ctl returned {out.returncode})\n{out.stderr}'
    except FileNotFoundError:
        return '(v4l2-ctl not installed)'
    except Exception as exc:  # noqa: BLE001
        return f'(v4l2-ctl failed: {exc})'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera/image/compressed')
    parser.add_argument('--duration', type=float, default=10.0, help='measurement window seconds')
    parser.add_argument('--device', default=None, help='camera device for v4l2-ctl, e.g. /dev/video1')
    parser.add_argument('--output-dir', default='.', help='directory for the report file')
    args = parser.parse_args()

    rclpy.init()
    node = CameraDiagnostics(args.topic, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    report = summarize(node)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    header = f'# Camera diagnostics report\n# generated: {stamp}\n'
    if args.device:
        report += '\n\n# v4l2-ctl --list-formats-ext\n' + v4l2_formats(args.device)

    out_path = f'{args.output_dir.rstrip("/")}/camera_diag_{stamp}.txt'
    with open(out_path, 'w', encoding='utf-8') as fp:
        fp.write(header + '\n' + report + '\n')

    print(report)
    print(f'\nReport saved to: {out_path}', file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
