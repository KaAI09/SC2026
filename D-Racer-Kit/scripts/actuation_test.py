#!/usr/bin/env python3
"""Interactive steering/throttle limit finder for the D-Racer.

Publishes raw control_msgs/Control to /control while you watch the vehicle and
type feedback. Use it to discover the usable steering range and the practical
forward/reverse throttle limits, then `mark` the values you confirm by eye.

REQUIREMENTS
    Run control_node in DIRECT mode and joystick_node as an E-STOP backup:
        ros2 launch control actuation_test.launch.py
    Then, in another terminal:
        python3 scripts/actuation_test.py

SAFETY
    * START WITH THE WHEELS OFF THE GROUND.
    * Joystick X button = hardware E-STOP (latched in control_node).
    * Ctrl+C or `q` here -> commands return to neutral (0) immediately.
    * This script republishes the current setpoint at --rate Hz so the
      control_node watchdog (control_timeout) stays satisfied; if this script
      dies, the watchdog stops the vehicle within control_timeout seconds.
    * Throttle is capped at --throttle-cap to prevent fat-finger inputs.

COMMANDS (type at the prompt)
    s <v>        set steering   (-1.0 .. 1.0; negative=left or right per wiring)
    t <v>        set throttle   (-cap .. cap; positive=forward, negative=reverse)
    c            steering -> 0 (center)
    0            throttle -> 0 (neutral)
    stop         steering 0 AND throttle 0
    mark <label> log current (steering, throttle) as a confirmed limit
    note <text>  log a free-form observation
    cap <v>      raise/lower the throttle cap this session
    p            print current setpoint
    q | quit     neutral, save log, exit

Every command and your marks/notes are appended to
    <output_dir>/actuation_test_<YYYYmmdd_HHMMSS>.csv
"""
import argparse
import csv
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node

from control_msgs.msg import Control


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class ActuationTester(Node):
    def __init__(self, topic, rate_hz):
        super().__init__('actuation_test')
        self.pub = self.create_publisher(Control, topic, 10)
        self.topic = topic
        self.steering = 0.0
        self.throttle = 0.0
        self.lock = threading.Lock()
        self.running = True
        period = 1.0 / float(rate_hz)
        self.thread = threading.Thread(target=self._publish_loop, args=(period,), daemon=True)
        self.thread.start()

    def _publish_loop(self, period):
        # Continuously republish the current setpoint so the control_node
        # watchdog stays fresh. rclpy timers need spin; a plain thread does not.
        import time
        while self.running and rclpy.ok():
            with self.lock:
                steering, throttle = self.steering, self.throttle
            msg = Control()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'actuation_test'
            msg.steering = float(steering)
            msg.throttle = float(throttle)
            self.pub.publish(msg)
            time.sleep(period)

    def set_setpoint(self, steering=None, throttle=None):
        with self.lock:
            if steering is not None:
                self.steering = steering
            if throttle is not None:
                self.throttle = throttle
            return self.steering, self.throttle

    def neutral(self):
        return self.set_setpoint(steering=0.0, throttle=0.0)

    def stop(self):
        self.running = False
        self.neutral()
        # Push a few explicit neutral frames before shutdown.
        for _ in range(5):
            msg = Control()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'actuation_test'
            msg.steering = 0.0
            msg.throttle = 0.0
            self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default='/control')
    parser.add_argument('--rate', type=float, default=20.0, help='republish rate Hz')
    parser.add_argument('--throttle-cap', type=float, default=0.30,
                        help='max |throttle| accepted (safety)')
    parser.add_argument('--output-dir', default='.')
    args = parser.parse_args()

    throttle_cap = abs(args.throttle_cap)

    rclpy.init()
    node = ActuationTester(args.topic, args.rate)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = f'{args.output_dir.rstrip("/")}/actuation_test_{stamp}.csv'
    log_file = open(log_path, 'w', newline='', encoding='utf-8')
    writer = csv.writer(log_file)
    writer.writerow(['time', 'action', 'steering', 'throttle', 'label_or_note'])

    def log(action, label=''):
        writer.writerow([
            datetime.now().strftime('%H:%M:%S'),
            action, f'{node.steering:.3f}', f'{node.throttle:.3f}', label,
        ])
        log_file.flush()

    print(__doc__)
    print(f'>>> publishing to {args.topic} at {args.rate:.0f} Hz, throttle cap = +/-{throttle_cap:.2f}')
    print('>>> WHEELS OFF THE GROUND. Joystick X = E-STOP. Logging to:', log_path)

    try:
        while True:
            try:
                raw = input('act> ').strip()
            except EOFError:
                break
            if not raw:
                continue
            parts = raw.split(maxsplit=1)
            cmd = parts[0].lower()
            arg = parts[1].strip() if len(parts) > 1 else ''

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd == 's':
                try:
                    v = clamp(float(arg), -1.0, 1.0)
                except ValueError:
                    print('  usage: s <value -1.0..1.0>')
                    continue
                node.set_setpoint(steering=v)
                log('steering', '')
                print(f'  steering = {v:.3f}')
            elif cmd == 't':
                try:
                    v = float(arg)
                except ValueError:
                    print('  usage: t <value>')
                    continue
                if abs(v) > throttle_cap:
                    print(f'  REFUSED: |{v:.3f}| exceeds cap {throttle_cap:.2f}. '
                          f'Use "cap <v>" to raise it deliberately.')
                    continue
                node.set_setpoint(throttle=v)
                log('throttle', '')
                print(f'  throttle = {v:.3f}')
            elif cmd == 'c':
                node.set_setpoint(steering=0.0)
                log('steering_center', '')
                print('  steering = 0.000')
            elif cmd == '0':
                node.set_setpoint(throttle=0.0)
                log('throttle_zero', '')
                print('  throttle = 0.000')
            elif cmd == 'stop':
                node.neutral()
                log('stop', '')
                print('  steering = 0.000, throttle = 0.000')
            elif cmd == 'mark':
                log('MARK', arg or '(unlabeled)')
                print(f'  marked: steering={node.steering:.3f} throttle={node.throttle:.3f} '
                      f'label="{arg}"')
            elif cmd == 'note':
                log('NOTE', arg)
                print('  noted.')
            elif cmd == 'cap':
                try:
                    throttle_cap = abs(float(arg))
                    print(f'  throttle cap = +/-{throttle_cap:.2f}')
                except ValueError:
                    print('  usage: cap <value>')
            elif cmd == 'p':
                print(f'  steering={node.steering:.3f} throttle={node.throttle:.3f} '
                      f'cap=+/-{throttle_cap:.2f}')
            else:
                print('  unknown command. s/t/c/0/stop/mark/note/cap/p/q')
    except KeyboardInterrupt:
        print('\n  KeyboardInterrupt -> neutral')
    finally:
        node.stop()
        log('exit_neutral', '')
        log_file.close()
        print(f'\nNeutral sent. Log saved to: {log_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
