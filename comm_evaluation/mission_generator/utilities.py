import math
from datetime import timedelta


class Utility:
    def func(self, m, t, s):
        raise NotImplementedError()

    def integrate(self, m, t_start, t_end, states=None):
        raise NotImplementedError()

    def __call__(self, m, t, s):
        return self.func(m, t, s)


class UtilityBattery(Utility):
    def __init__(self):
        self.MAX_BATT_DEPLETION_RATE = 0.05

    def func(self, m, t, s):
        length = (t-m.t_sent)
        if isinstance(length, timedelta):
            length = length.total_seconds()

        return 0 if t < m.t_rcv else max(
            1 - (length *
                 self.MAX_BATT_DEPLETION_RATE)/m.data['battery_level'],
            0
        )

    def integrate(self, m, t_start, t_end):
        # all times in the system are datetime and timedelta objects,
        # but here in order to make the computation possible
        # we define a time reference point and then treat all times
        # as a difference between them and a reference point, in seconds

        reference = t_start

        def normalize(t):
            t = t - reference
            if isinstance(t, timedelta):
                return t.total_seconds()
            else:
                return t

        t_start = 0
        t_rcv = normalize(m.t_rcv)
        t_end = normalize(t_end)
        t_sent = normalize(m.t_sent)

        a = max(t_start, t_rcv)
        b = min(
            t_end,
            m.data['battery_level']/self.MAX_BATT_DEPLETION_RATE + t_sent
        )

        if a > b:
            result = 0
        else:
            result = b - a + (
                (self.MAX_BATT_DEPLETION_RATE/m.data['battery_level']) *
                (t_sent*(b-a) - ((b-a)*(b+a) / 2))
            )
        return result


class UtilityPosition(Utility):
    def __init__(self):
        self.VEL_MAX = 1
        self.AREA = 100

    def func(self, m, t, s):
        if t < m.t_rcv:
            return 0
        else:
            return max(
                1 - ((t-m.t_sent)**2 * math.pi*self.VEL_MAX**2)/self.AREA,
                0
            )

    def integrate(self, m, t_start, t_end):
        return 1
