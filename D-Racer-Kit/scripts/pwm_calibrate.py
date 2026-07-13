#!/usr/bin/env python3
"""Low-level RAW PWM calibration for the D-Racer steering servo and ESC.

This bypasses control_node and the D3Racer -1..1 normalization/clip and drives
the PCA9685 channels directly in MICROSECONDS, so you can find the TRUE usable
steering range and probe ESC forward/reverse behavior with NO software clip
(only the PCA9685 12-bit hardware limit remains).

WHEN TO USE
    * Finding real steering min/center/max microseconds (mechanical limits).
    * Probing why reverse does not engage (ESC arming / deadband).

DO NOT RUN control_node AT THE SAME TIME
    Both would write the same PCA9685 channels and fight each other. Stop any
    running control/launch first:  (Ctrl+C the launch, or `pkill -f control_node`)

SAFETY (read before running)
    * WHEELS OFF THE GROUND for every throttle command.
    * There is NO ROS e-stop/watchdog here. Your ultimate stop is the battery
      disconnect. Keep it within reach.
    * Steering: raw microseconds beyond the mechanical stop will STALL the servo
      (buzzing, heat, current draw, gear damage). Increase in small steps and
      STOP as soon as the wheel no longer turns further.
    * Throttle commands auto-return to neutral after --hold seconds.

COMMANDS
    s <us> [force]   steering pulse (microseconds). Outside limits needs `force`.
    t <us> [dur]     throttle pulse for `dur` s (default --hold), then neutral.
    sc               steering -> center/neutral (--neutral-us)
    tn               throttle -> neutral (cancels the auto-neutral timer)
    stop             steering AND throttle -> neutral
    mark <label>     log current pulses as a confirmed limit
    note <text>      log a free-form observation
    p                print current pulses and limits
    q | quit         neutral both, save log, exit

TYPICAL FLOWS
    Steering range:  sc -> s 1600 -> s 1700 -> ... watch the wheel, `mark max`
                     then sc -> s 1400 -> s 1300 -> ... `mark min`
    Reverse probe :  tn -> t 1000 1 (brake) -> tn -> t 1000 1 (reverse?) ...
                     or step 1450,1400,...,1100 to find the reverse threshold.
"""
import argparse
import csv
import threading
from datetime import datetime

from topst_utils.pca9685 import PCA9685


class PwmCalibrator:
    def __init__(self, args):
        self.neutral_us = float(args.neutral_us)
        self.steer_min = float(args.min_us)
        self.steer_max = float(args.max_us)
        self.thr_min = float(args.throttle_min_us)
        self.thr_max = float(args.throttle_max_us)
        self.hold = float(args.hold)
        self.steer_ch = int(args.steer_ch)
        self.throttle_ch = int(args.throttle_ch)

        self.pwm = PCA9685(bus=int(args.i2c_bus), address=int(args.addr, 0)
                           if isinstance(args.addr, str) else int(args.addr),
                           freq_hz=float(args.freq))
        self.steer_us = self.neutral_us
        self.throttle_us = self.neutral_us
        self._timer = None
        self._lock = threading.Lock()

        # Start from a known safe neutral on both channels.
        self.pwm.set_pulse_us(self.steer_ch, self.neutral_us)
        self.pwm.set_pulse_us(self.throttle_ch, self.neutral_us)

    def set_steer(self, us):
        self.steer_us = us
        self.pwm.set_pulse_us(self.steer_ch, us)

    def _apply_throttle(self, us):
        self.throttle_us = us
        self.pwm.set_pulse_us(self.throttle_ch, us)

    def set_throttle(self, us, dur):
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            self._apply_throttle(us)
            if abs(us - self.neutral_us) > 1e-6 and dur > 0:
                self._timer = threading.Timer(dur, self._auto_neutral)
                self._timer.daemon = True
                self._timer.start()

    def _auto_neutral(self):
        with self._lock:
            self._apply_throttle(self.neutral_us)
            self._timer = None

    def throttle_neutral(self):
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            self._apply_throttle(self.neutral_us)

    def stop(self):
        self.throttle_neutral()
        self.set_steer(self.neutral_us)

    def close(self):
        try:
            self.stop()
        finally:
            self.pwm.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--i2c-bus', default=3)
    parser.add_argument('--addr', default='0x40')
    parser.add_argument('--freq', type=float, default=50.0)
    parser.add_argument('--steer-ch', type=int, default=0)
    parser.add_argument('--throttle-ch', type=int, default=1)
    parser.add_argument('--neutral-us', type=float, default=1500.0)
    parser.add_argument('--min-us', type=float, default=900.0, help='steering lower guard')
    parser.add_argument('--max-us', type=float, default=2100.0, help='steering upper guard')
    parser.add_argument('--throttle-min-us', type=float, default=1100.0)
    parser.add_argument('--throttle-max-us', type=float, default=1900.0)
    parser.add_argument('--hold', type=float, default=2.0, help='throttle auto-neutral seconds')
    parser.add_argument('--output-dir', default='.')
    args = parser.parse_args()

    cal = PwmCalibrator(args)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = f'{args.output_dir.rstrip("/")}/pwm_calibrate_{stamp}.csv'
    log_file = open(log_path, 'w', newline='', encoding='utf-8')
    writer = csv.writer(log_file)
    writer.writerow(['time', 'action', 'steer_us', 'throttle_us', 'label_or_note'])

    def log(action, label=''):
        writer.writerow([
            datetime.now().strftime('%H:%M:%S'),
            action, f'{cal.steer_us:.0f}', f'{cal.throttle_us:.0f}', label,
        ])
        log_file.flush()

    print(__doc__)
    print(f'>>> PCA9685 bus={args.i2c_bus} addr={args.addr} freq={args.freq:.0f}Hz '
          f'steer_ch={args.steer_ch} throttle_ch={args.throttle_ch}')
    print(f'>>> guards: steering [{cal.steer_min:.0f},{cal.steer_max:.0f}]us '
          f'throttle [{cal.thr_min:.0f},{cal.thr_max:.0f}]us  hold={cal.hold:.1f}s')
    print('>>> WHEELS OFF THE GROUND. Battery disconnect = your e-stop. Log:', log_path)

    try:
        while True:
            try:
                raw = input('pwm> ').strip()
            except EOFError:
                break
            if not raw:
                continue
            parts = raw.split()
            cmd = parts[0].lower()

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd == 's':
                try:
                    us = float(parts[1])
                except (IndexError, ValueError):
                    print('  usage: s <microseconds> [force]')
                    continue
                forced = len(parts) > 2 and parts[2].lower() == 'force'
                if not forced and not (cal.steer_min <= us <= cal.steer_max):
                    print(f'  REFUSED: {us:.0f}us outside [{cal.steer_min:.0f},'
                          f'{cal.steer_max:.0f}]. Append "force" to override (servo may stall).')
                    continue
                cal.set_steer(us)
                log('steer')
                print(f'  steer_us = {us:.0f}')
            elif cmd == 't':
                try:
                    us = float(parts[1])
                except (IndexError, ValueError):
                    print('  usage: t <microseconds> [seconds]')
                    continue
                dur = cal.hold
                forced = False
                for extra in parts[2:]:
                    if extra.lower() == 'force':
                        forced = True
                    else:
                        try:
                            dur = float(extra)
                        except ValueError:
                            pass
                if not forced and not (cal.thr_min <= us <= cal.thr_max):
                    print(f'  REFUSED: {us:.0f}us outside [{cal.thr_min:.0f},'
                          f'{cal.thr_max:.0f}]. Append "force" to override.')
                    continue
                cal.set_throttle(us, dur)
                log('throttle', f'dur={dur:.1f}s')
                print(f'  throttle_us = {us:.0f} for {dur:.1f}s -> auto neutral')
            elif cmd == 'sc':
                cal.set_steer(cal.neutral_us)
                log('steer_center')
                print(f'  steer_us = {cal.neutral_us:.0f}')
            elif cmd == 'tn':
                cal.throttle_neutral()
                log('throttle_neutral')
                print(f'  throttle_us = {cal.neutral_us:.0f}')
            elif cmd == 'stop':
                cal.stop()
                log('stop')
                print('  both -> neutral')
            elif cmd == 'mark':
                label = ' '.join(parts[1:]) or '(unlabeled)'
                log('MARK', label)
                print(f'  marked: steer={cal.steer_us:.0f} throttle={cal.throttle_us:.0f} "{label}"')
            elif cmd == 'note':
                log('NOTE', ' '.join(parts[1:]))
                print('  noted.')
            elif cmd == 'p':
                print(f'  steer_us={cal.steer_us:.0f} throttle_us={cal.throttle_us:.0f} '
                      f'neutral={cal.neutral_us:.0f}')
            else:
                print('  unknown command. s/t/sc/tn/stop/mark/note/p/q')
    except KeyboardInterrupt:
        print('\n  KeyboardInterrupt -> neutral')
    finally:
        cal.close()
        log('exit_neutral')
        log_file.close()
        print(f'\nNeutral sent, PCA9685 closed. Log saved to: {log_path}')


if __name__ == '__main__':
    main()
