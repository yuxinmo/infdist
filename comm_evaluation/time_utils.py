import math
import time
from datetime import datetime

from builtin_interfaces.msg import Time


def get_current_time_msg():
    modf = math.modf(time.time())
    nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
    return Time(sec=sec, nanosec=nanosec)


def datetime_from_time_msg(time):
    return datetime.fromtimestamp(
        time.sec + time.nanosec * 1e-9
    )
