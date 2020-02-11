from collections import deque


def send_immidietly(msgs):
    for m in msgs.all():
        if m.t_sent is None:
            m.t_sent = m.t_gen


def latency(msgs, latency):
    send_immidietly(msgs)
    for m in msgs.all():
        m.t_rcv = m.t_sent + latency


def create_msgnum_constraint_violations(msgnum=9, timeslot=2.7):
    def msgnum_constraint_violations(msgs):
        """
        Returns the number of times the msgnum constraint is violated.
        """
        window = deque()
        result = 0
        for m in msgs.all():
            window.append(m)
            while m.t_sent - window[0].t_sent > timeslot:
                window.popleft()
            if len(window) > msgnum:
                result += 1
        return result
    return msgnum_constraint_violations


def create_throughput_constraint_violations(throughput, timeslot,
                                            message_size):
    """
    :param throughput: throughput in Mbps
    :param message_size: message size in bytes
    """

    def throughput_constraint_violations(msgs):
        """
        Returns the number of times the throughput constraint is violated.
        """
        window = deque()
        result = 0
        throughput_in_bytes = (timeslot*throughput/8)*10**6
        for m in msgs.all():
            window.append(m)
            while m.t_sent - window[0].t_sent > timeslot:
                window.popleft()
            if len(window)*message_size > throughput_in_bytes:
                result += 1
        return result
    return throughput_constraint_violations
