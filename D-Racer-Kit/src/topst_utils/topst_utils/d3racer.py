
from __future__ import annotations
from dataclasses import dataclass
from topst_utils.pca9685 import PCA9685

@dataclass
class ServoCalib:
    """조향 서보 캘리브레이션. **실측값이다** — 관례 숫자가 아니다.

    아래 기본값(1500/500/1000/2000)은 RC 서보의 관례이지 어떤 차의 측정값도 아니다.
    실측 (D3-G, **실차 직진으로 확인** — 바퀴에 조향이 전달되는 범위):

        center 1650us = 0도,   1350us = 우 25도,   1950us = 좌 25도   (300us = 25도)
        서보 자체는 1250~2050us 까지 무리 없이 돈다 (그러나 바퀴는 더 안 돈다)

    옛 설정(center 1500 / span 500 / actuator 가 clamp(u + 0.3))이 실제로 내던 조향각:

        u =  0.0  -> 1650us ->   0.00도    직진이었다 — STEER_TRIM 0.3 은 맞았다
        u = +0.6  -> 1950us -> +25.00도    여기서 이미 최대
        u = +0.7  -> 2000us -> +25.00도    포화 (조향각이 더 안 늘어난다)

    좌우는 대칭이었다. 문제는 **포화**다: steer_max 0.7 인데 u=±0.6 에서 바퀴가 끝까지 돌아,
    명령 범위의 14% 가 조향각을 전혀 바꾸지 못했다. 제어기는 더 달라는데 바퀴는 멈춰 있고,
    제어기는 그것을 알 길이 없다.

    `center_us` 는 **서보의 중립**이다. 조향 명령에 더하는 트림이 아니다 (예전의 STEER_TRIM).
    명령에 더하면 u 와 트림이 같은 [-1,1] 예산을 나눠 쓰고, 그래서 steer_max 를 1-|trim| 로
    깎아야 했다 — 그것이 0.7 의 정체다. 중립으로 옮기면 u 는 ±1.0 전체를 쓰고 그 전 구간이
    선형이다 (±25도).

    `span_us` 는 **조향이 실제로 일어나는 반경**이다 (실측 300us). min/max 까지의 여유가
    아니다 — 넓히면 u 의 끝부분이 조향각을 바꾸지 못하는 데드존이 되고, 그것이 위에서 본
    바로 그 버그다. `effective_span()` 을 볼 것.
    """
    center_us: int = 1500       # 바퀴가 직진하는 펄스 (실측!)
    span_us: int = 500          # 조향이 실제로 일어나는 반경 (실측). p=±1.0 이 여기까지
    min_us: int = 1000          # 서보 하드 clip (조향 반경보다 넓어도 된다 = 트림 조정 여유)
    max_us: int = 2000

    def symmetric_span(self) -> float:
        """center 에서 좌우로 **똑같이** 갈 수 있는 최대 거리 (min/max 안에서).

        center 를 옮기면(트림 조정) 한쪽 여유가 줄어든다. 좁은 쪽에 맞추지 않으면 넓은 쪽만
        더 돌고 좁은 쪽은 min/max 에서 조용히 clip 된다 — 같은 오차에 좌우가 다른 각도로 도는
        것이고, 제어기는 한쪽에만 맞는 게인으로 도는 셈이 된다.
        """
        return min(self.max_us - self.center_us, self.center_us - self.min_us)

    def effective_span(self) -> float:
        """실제로 쓰는 span. `p = +-1.0` 이 여기까지 간다.

        두 개의 상한을 **동시에** 지켜야 한다:

          span_us          조향이 실제로 일어나는 범위 (실측). 넘겨봐야 바퀴가 더 안 돈다 —
                           u 의 끝부분이 조향각을 못 바꾸는 데드존이 되고, 그것이 이 작업이
                           없애려는 바로 그 버그다.
          symmetric_span() min/max 안에서 좌우가 같으려면 여기까지.

        그래서 **둘 중 작은 쪽**이다. span_us 를 늘리는 방향으로는 절대 가지 않는다: 서보가
        더 돌 수 있다는 것과 바퀴가 더 돈다는 것은 다른 얘기다.
        """
        return min(self.span_us, self.symmetric_span())


@dataclass
class EscCalib:
    neutral_us: int = 1500
    fwd_us: int = 2000          # +1.0
    rev_us: int = 1000          # -1.0
    min_us: int = 1000
    max_us: int = 2000


class D3Racer:
    """
    PiRacerPro와 유사한 API:
      - set_steering_percent(x): -1.0 ~ +1.0 (좌/우)
      - set_throttle_percent(x): -1.0 ~ +1.0 (후/전), 0=중립
    """
    def __init__(
        self,
        i2c_bus: int = 3,
        pca9685_addr: int = 0x40,
        freq_hz: float = 50.0,
        steering_channel: int = 0,
        throttle_channel: int = 1,
        steering: ServoCalib = ServoCalib(),
        esc: EscCalib = EscCalib(),
    ):
        self.pwm = PCA9685(bus=i2c_bus, address=pca9685_addr, freq_hz=freq_hz)
        self.st_ch = steering_channel
        self.th_ch = throttle_channel
        self.st = steering
        self.esc = esc

        # 안전: 초기 중립
        self.set_steering_percent(0.0)
        self.set_throttle_percent(0.0)

    @staticmethod
    def clip(x: float, lo: float, hi: float) -> float:
        return lo if x < lo else hi if x > hi else x

    def set_steering_center(self, center_us: float):
        """서보 중립을 런타임에 옮긴다 — 조이스틱 Y/B 트림 조정이 여기로 들어온다.

        조정하며 바퀴가 즉시 움직여야 트림을 맞출 수 있다. 재시작해야 반영되는 캘리브는
        캘리브가 아니다.

        `span_us`(실측 조향 범위)는 **건드리지 않는다.** 중립이 치우쳐 좌우 여유가 달라지면
        `effective_span()` 이 알아서 좁은 쪽에 맞춘다. 여기서 span 을 symmetric_span 으로
        덮어썼다면 중립이 가운데일 때 span 이 **늘어나** 조향이 안 되는 구간까지 명령을
        보내게 된다 — 없애려던 데드존을 되살리는 짓이다.

        반환값: 새 effective span (호출자가 로그로 알릴 수 있게 — 조용히 줄이지 않는다).
        """
        self.st.center_us = float(center_us)
        return self.st.effective_span()

    def set_steering_percent(self, p: float):
        p = float(p)
        p = self.clip(p, -1.0, 1.0)

        pulse = self.st.center_us + p * self.st.effective_span()
        pulse = self.clip(pulse, self.st.min_us, self.st.max_us)
        self.pwm.set_pulse_us(self.st_ch, pulse)

    def set_throttle_percent(self, p: float):
        p = float(p)
        p = self.clip(p, -1.0, 1.0)

        if p > 0:
            pulse = self.esc.neutral_us + p * (self.esc.fwd_us - self.esc.neutral_us)
        elif p < 0:
            pulse = self.esc.neutral_us + p * (self.esc.neutral_us - self.esc.rev_us)
        else:
            pulse = self.esc.neutral_us

        pulse = self.clip(pulse, self.esc.min_us, self.esc.max_us)
        self.pwm.set_pulse_us(self.th_ch, pulse)

    def stop(self):
        self.set_throttle_percent(0.0)

    def close(self):
        self.stop()
        self.pwm.close()
