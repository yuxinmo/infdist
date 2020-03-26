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
    def msgnum_constraint_violations(messageset, incremental=False):
        """
        Returns the number of times the msgnum constraint is violated.
        """
        window = deque()
        result = 0
        if incremental:
            msgs = messageset.msgs_gen_after(
                messageset.message.t_gen - timeslot
            )
        else:
            msgs = messageset.all()
        for m in msgs:
            window.append(m)
            while m.t_sent - window[0].t_sent > timeslot:
                window.popleft()
            if len(window) > msgnum:
                result += 1
        return result

    def compute_value(messageset, t):
        return len([
            m
            for m in messageset.gen_after(t-timeslot)
            if m.t_gen <= t
        ])

    msgnum_constraint_violations.compute_value = compute_value

    return msgnum_constraint_violations


def create_throughput_constraint_violations(throughput, timeslot):
    """
    :param throughput: throughput in Mbps
    :param message_size: message size in bytes
    """
    throughput_in_bytes = (timeslot*throughput/8)*10**6

    def throughput_constraint_violations(messageset, incremental=False):
        """
        Returns the number of times the throughput constraint is violated.
        """
        window = deque()
        result = 0

        if incremental:
            msgs = messageset.msgs_gen_after(
                messageset.message.t_gen - timeslot
            )
        else:
            msgs = messageset.all()

        for m in msgs:
            window.append(m)
            while m.t_sent - window[0].t_sent > timeslot:
                window.popleft()
            if sum([m.size for m in window]) > throughput_in_bytes:
                result += 1
        return result

    def compute_value(messageset, t):
        return sum([
            m.size
            for m in messageset.gen_after(t-timeslot)
            if m.t_gen <= t
        ])

    throughput_constraint_violations.compute_value = compute_value

    return throughput_constraint_violations
