from datetime import datetime


def datetime_from_time_msg(time):
    return datetime.fromtimestamp(
        time.sec + time.nanosec * 1e-9
    )
