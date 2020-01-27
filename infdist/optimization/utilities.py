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
    def func(self, m, t, s):
        length = (t-m.t_gen)
        if isinstance(length, timedelta):
            length = length.total_seconds()

        return 0 if t < m.t_rcv else max(
            1 - (
                length *
                m.data.max_depl_rate
            )/m.data.battery_level,
            0
        )

    def integrate(self, m, t_start, t_end):
        t_rcv = m.t_rcv
        t_gen = m.t_gen

        max_depl_rate = m.data.max_depl_rate
        batt_level = m.data.battery_level

        a = t_start if t_start > t_rcv else t_rcv

        t_batt_zero = batt_level/max_depl_rate + t_gen
        b = t_end if t_end < t_batt_zero else t_batt_zero

        if a > b:
            result = 0
        else:
            result = b - a + (
                (max_depl_rate/batt_level) *
                (t_gen*(b-a) - ((b-a)*(b+a) / 2))
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
                1 - ((t-m.t_gen)**2 * math.pi*self.VEL_MAX**2)/self.AREA,
                0
            )

    def integrate(self, m, t_start, t_end):
        return 1
