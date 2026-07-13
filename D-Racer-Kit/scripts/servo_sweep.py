#!/usr/bin/env python3
"""조향 서보 실측 — center / 좌우 끝단 (남은 작업 A1). **D3-G에서 실행.**

`ServoCalib` 의 `center_us=1500 / span_us=500 / min_us=1000 / max_us=2000` 은 RC 서보의
**관례 숫자이지 이 차의 서보를 측정한 값이 아니다** (`topst_utils/d3racer.py` dataclass 기본값).
그래서 지금은 서보 혼이 틀어진 것을 `STEER_TRIM=0.3` 으로 **명령에 더해서** 보정하는데,
그러면 조향 명령 u 와 트림이 같은 [-1,1] 예산을 나눠 쓴다:

    actuator: clamp(u + 0.3, -1, 1)      u=+0.7 -> +1.0 (여기가 끝)
                                         u=-0.7 -> -0.4 (아직 0.6 여유)

좌우가 비대칭이고, `steer_max: 0.7` 은 "양쪽 다 0.7까지만 쓰자" 는 **임시 방편**이다.
넘으면 한쪽 조향 권한만 조용히 깎인다(서보는 멈춰 있는데 제어기는 계속 더 달라고 한다
-> 한쪽 코너만 언더스티어).

진짜 해법은 트림을 **서보 중립 자체**로 옮기는 것이고, 그러려면 세 값을 재야 한다:

    center_us   바퀴가 정확히 직진하는 펄스
    hi_us       한쪽으로 **바퀴가 더 안 도는** 지점 (서보가 아니라 바퀴를 봐라)
    lo_us       반대쪽도 마찬가지
    => span_us = min(hi - center, center - lo)      좌우 대칭 예산

    ★ 기준은 **바퀴**다. 서보는 그 밖에서도 돌지만 스티어링 랙이 끝단에 닿으면 바퀴는
      더 안 돈다. 그 구간을 span 에 넣으면 u 의 끝부분이 조향각을 바꾸지 못하는 데드존이
      되고, 그것은 이 작업이 없애려는 바로 그 버그다 — 제어기는 더 달라는데 바퀴는 멈춰
      있다. 2026-07 실측: center 1600us, 1200~2000us = 좌우 25도.

    좁은 쪽에 맞추는 것이 핵심이다. 비대칭 조향 권한은 제어기가 다룰 수 없는 종류의
    거짓말이고, 넓은 쪽의 남는 여유는 버리는 게 맞다.

⚠ 안전
    - **바퀴를 지면에서 뗄 것.** 조향이 돌아도 차가 움직이지 않고, 타이어가 바닥을 물어
      서보에 무리한 부하가 걸리지 않는다.
    - **actuator_node / joystick_node 를 꺼둘 것.** 이 스크립트는 PCA9685 에 직접 펄스를
      넣는다. actuator 가 동시에 돌면 둘이 같은 채널을 다투며 서보가 떨린다.
    - **스로틀은 건드리지 않는다** (ESC 채널에 접근조차 하지 않는다).
    - 서보가 **신음소리(스톨)** 를 내면 즉시 반대 방향으로 되돌리거나 전원을 끊어라.
      박아둔 채로 두면 기어가 나간다.

    python3 scripts/servo_sweep.py                 # 기본: bus 3, addr 0x40, ch 0
    python3 scripts/servo_sweep.py --step 5        # 미세하게

조작
    a / d   -step / +step us        q / e   -1 / +1 us  (미세)
    s       이 값을 현재 단계의 측정값으로 확정하고 다음 단계
    0       중립(시작 펄스)으로 즉시 복귀
    x       중립 복귀 후 종료 (Ctrl-C 도 같다)
"""
import argparse
import os
import sys
import termios
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'topst_utils'))
from topst_utils.pca9685 import PCA9685      # noqa: E402

# D3Racer 가 아니라 PCA9685 를 직접 쓴다: D3Racer.set_steering_percent 는 min_us/max_us 로
# clip 하는데, 지금 재려는 것이 바로 그 한계값이다. clip 이 걸리면 측정이 거짓말이 된다.


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


STEPS = [
    ('CENTER', '바퀴가 **정확히 직진**할 때까지 맞춘다. 자·직선을 대고 좌우 대칭인지 봐라.'),
    ('HI (한쪽 끝)', '한쪽으로 천천히. **바퀴가 더 안 도는 지점**에서 멈춘다 (서보 말고 바퀴를 봐라).'),
    ('LO (반대쪽)', '반대쪽도 바퀴가 더 안 돌 때까지.'),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bus', type=int, default=3)
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x40)
    ap.add_argument('--channel', type=int, default=0, help='조향 채널 (스로틀은 1 — 안 건드린다)')
    ap.add_argument('--freq', type=float, default=50.0)
    ap.add_argument('--start-us', type=float, default=1500.0, help='시작 펄스')
    ap.add_argument('--step', type=float, default=10.0, help='a/d 스텝 (us)')
    ap.add_argument('--limit', type=_range, default=(900.0, 2100.0), metavar='LO:HI',
                    help='하드 안전 범위 us (기본 900:2100). 이 밖으로는 나가지 않는다')
    a = ap.parse_args()

    lo_lim, hi_lim = a.limit
    pwm = PCA9685(bus=a.bus, address=a.addr, freq_hz=a.freq)
    us = float(a.start_us)
    got = {}

    def apply(v):
        v = max(lo_lim, min(hi_lim, v))
        pwm.set_pulse_us(a.channel, v)
        return v

    print(__doc__.split('조작')[0])
    print(f'bus={a.bus} addr=0x{a.addr:02X} ch={a.channel} freq={a.freq}Hz  '
          f'안전범위 {lo_lim:.0f}~{hi_lim:.0f}us  step={a.step:.0f}us')
    print('⚠ 바퀴가 지면에서 떨어져 있는지, actuator/joystick 노드가 꺼져 있는지 확인했나?')
    print('  Enter 로 시작 (Ctrl-C 로 취소)', end='')
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print('\n취소.')
        return

    try:
        us = apply(us)
        for name, hint in STEPS:
            print(f'\n── {name} ─────────────────────────────────────────')
            print(f'   {hint}')
            print('   a/d = ∓%g us   q/e = ∓1 us   s = 확정   0 = 중립   x = 종료'
                  % a.step)
            while True:
                print(f'\r   pulse = {us:7.1f} us   ', end='', flush=True)
                k = getch()
                if k in ('\x03', 'x', 'X'):          # Ctrl-C / x
                    raise KeyboardInterrupt
                elif k in ('a', 'A'):
                    us = apply(us - a.step)
                elif k in ('d', 'D'):
                    us = apply(us + a.step)
                elif k in ('q', 'Q'):
                    us = apply(us - 1.0)
                elif k in ('e', 'E'):
                    us = apply(us + 1.0)
                elif k == '0':
                    us = apply(a.start_us)
                elif k in ('s', 'S'):
                    got[name] = us
                    print(f'\r   pulse = {us:7.1f} us   ✔ {name} = {us:.0f} us')
                    break
        report(got, a)
    except KeyboardInterrupt:
        print('\n\n중단 — 중립으로 복귀한다.')
    finally:
        pwm.set_pulse_us(a.channel, a.start_us)      # 항상 중립으로 되돌린다
        pwm.close()
        print(f'서보를 {a.start_us:.0f}us 로 되돌렸다.')


def report(got, a):
    c = got.get('CENTER')
    hi = got.get('HI (한쪽 끝)')
    lo = got.get('LO (반대쪽)')
    if None in (c, hi, lo):
        print('\n측정이 완결되지 않았다.')
        return
    if hi < lo:                       # 어느 쪽을 먼저 쟀든 상관없게
        hi, lo = lo, hi
    up, dn = hi - c, c - lo
    span = min(up, dn)

    print('\n' + '═' * 62)
    print('실측 결과')
    print('═' * 62)
    print(f'  center_us : {c:7.0f}     (RC 관례 기본값은 1500 — 실측이 다르면 그게 정상이다)')
    print(f'  hi_us     : {hi:7.0f}     center 로부터 +{up:.0f}  (바퀴가 더 안 도는 지점)')
    print(f'  lo_us     : {lo:7.0f}     center 로부터 -{dn:.0f}')
    print()
    print(f'  → span_us : {span:7.0f}     좁은 쪽({"hi" if up < dn else "lo"})에 맞춘 대칭 예산')
    print(f'    min_us  : {c - span:7.0f}')
    print(f'    max_us  : {c + span:7.0f}')
    print(f'    버리는 여유: {abs(up - dn):.0f}us ({"hi" if up > dn else "lo"} 쪽)')
    print()
    print('  대칭이 핵심이다. 같은 오차에 좌회전과 우회전이 다른 각도로 돌면 제어기는')
    print('  한쪽에만 맞는 게인으로 도는 셈이 된다 — 넓은 쪽의 여유는 버리는 게 맞다.')
    print()
    print('─' * 62)
    print('이 값을 로컬로 가져가서 알려달라 (아직 아무것도 저장하지 않았다).')
    print('코드 변경(ServoCalib 를 vehicle_config 에서 읽기, actuator 의 trim 덧셈 제거,')
    print('steer_max 0.7 -> 1.0, joystick 트림 조정 경로)은 그 다음이다.')
    print('─' * 62)
    print(f'SERVO_CENTER_US: {c:.0f}\nSERVO_SPAN_US: {span:.0f}\n'
          f'SERVO_MIN_US: {c - span:.0f}\nSERVO_MAX_US: {c + span:.0f}')
    print('  (vehicle_config.yaml 에 이대로 넣는다)')


def _range(s):
    lo, hi = s.split(':')
    return float(lo), float(hi)


if __name__ == '__main__':
    main()
