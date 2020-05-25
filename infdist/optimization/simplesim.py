from collections import deque
from sklearn.linear_model import SGDRegressor


def sum_generator(timeslot=2.5):
    def _average_generator():
        window = deque()
        total = 0
        while True:
            t, d = yield total

            window.append((t, d))
            total += d
            while t - window[0][0] > timeslot:
                removed_t, removed_d = window.popleft()
                total -= removed_d
    g = _average_generator()
    next(g)  # start generating
    return g


def average_generator2(timeslot=2.5):
    def _average_generator():
        window = deque()
        avg = 0
        while True:
            (t, d) = yield avg

            window.append((t, d))
            while t - window[0][0] > timeslot:
                window.popleft()

            n = len(window)
            avg = sum(
                (d)*(i+1)
                for i, m in enumerate(window)
            )/((n+1) * n/2)
    g = _average_generator()
    next(g)  # start generating
    return g


def send_immidietly(msgs):
    for m in msgs.all():
        if m.t_sent is None:
            m.t_sent = m.t_gen


def apply_latency(msgs, latency):
    send_immidietly(msgs)
    for m in msgs.all():
        m.t_rcv = m.t_sent + latency


def weighted_average_latency_generator(timeslot=2.5):
    def _average_latency_generator():
        m = yield 0
        result = m.t_rcv - m.t_gen
        prev_gen = m.t_gen
        while True:
            m = yield result
            latency = m.t_rcv - m.t_gen
            time_since_last_message = (m.t_gen - prev_gen)
            assert time_since_last_message >= 0
            d0 = .02
            k = .5
            p = d0+time_since_last_message*k
            if p > 1:
                p = 1

            result = latency*p + result*(1-p)
            prev_gen = m.t_gen

    g = _average_latency_generator()
    next(g)  # start generating
    return g


def average_latency_generator2(timeslot=2.5):
    def _average_latency_generator():
        window = deque()
        avg = 0
        while True:
            m = yield avg

            window.append(m)
            while m.t_gen - window[0].t_gen > timeslot:
                window.popleft()

            n = len(window)
            avg = sum(
                (m.t_rcv - m.t_gen)*(i+1)
                for i, m in enumerate(window)
            )/((n+1) * n/2)
    g = _average_latency_generator()
    next(g)  # start generating
    return g


def average_latency_generator(timeslot=2.5):
    def _average_latency_generator():
        window = deque()
        avg = 0
        total = 0
        while True:
            m = yield avg

            window.append(m)
            total += m.t_rcv - m.t_gen
            while m.t_gen - window[0].t_gen > timeslot:
                removed_m = window.popleft()
                total -= removed_m.t_rcv - removed_m.t_gen

            avg = total/len(window)
    g = _average_latency_generator()
    next(g)  # start generating
    return g


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


def create_throughput_constraint_violations(throughput, timeslot_length):
    """
    :param throughput: throughput in Mbps
    :param message_size: message size in bytes
    """
    throughput_in_bytes = (timeslot_length*throughput/8)*10**6

    def throughput_constraint_violations(messageset, incremental=False):
        """
        Returns the number of times the throughput constraint is violated.
        """
        window = deque()
        result = 0

        if incremental:
            msgs = messageset.msgs_gen_after(
                messageset.message.t_gen - timeslot_length
            )
        else:
            msgs = messageset.all()

        for m in msgs:
            window.append(m)
            while m.t_sent - window[0].t_sent > timeslot_length:
                window.popleft()
            if sum([m.size for m in window]) > throughput_in_bytes:
                result += 1
        return result

    def compute_value(messageset, t):
        return sum([
            m.size*8/timeslot_length/10**6
            for m in messageset.gen_after(t-timeslot_length)
            if m.t_gen <= t
        ])

    throughput_constraint_violations.compute_value = compute_value
    throughput_constraint_violations.update_model = lambda a, b: None

    return throughput_constraint_violations


def create_rate_constraint_violations(
    timeslot_length,
    alpha=0.25,
    delta=0.01756,
    eta0=0.00005,
    coef_init=None,
    intercept_init=None,
):
    if coef_init is None:
        coef_init = [[0]]

    if intercept_init is None:
        intercept_init = [0]

    scale = 12
    # eta0 = 0.0002
    # eta0 = 0.0002

    rate_model = SGDRegressor(
        # loss='hinge',
        loss='squared_epsilon_insensitive',
        alpha=0,
        # shuffle=True,
        verbose=0,
        epsilon=0.1,
        learning_rate='constant',
        eta0=eta0,
        power_t=0.3,
        # fit_intercept=False,
    )
    train_data = []
    params_history = []

    rate_model.eta0 = 1
    rate_model.fit(
        coef_init, intercept_init,
        coef_init=coef_init,
        intercept_init=intercept_init,
    )
    rate_model.eta0 = eta0
    # print(
    #     f'initial coef: {rate_model.coef_}, {rate_model.intercept_}'
    # )

    def update_model(all_messages, t):
        window = messageset_to_window(all_messages, t-0.5)
        without_sent = [
                m
                for m in window
                if m.t_rcv is not None
        ]

        if len(without_sent) == 0:
            return

        throughput = average_throughput(window)
        rate = average_rate(without_sent)
        X = [[throughput*scale]]
        Y = [rate*scale]
        train_data.append((throughput, rate))
        # print(
        #     f'Observing ({X}, {Y}) -> '
        #     f'new coef: {rate_model.coef_}, {rate_model.intercept_}'
        # )
        rate_model.partial_fit(X, Y)
        params_history.append(
            (t, rate_model.coef_[0], rate_model.intercept_[0])
        )

    def messageset_to_window(messageset, t):
        msgs = messageset.gen_after(t-timeslot_length)
        return [
            m
            for m in msgs
            if m.t_gen <= t
        ]

    def modeled_value(messageset, t):
        return modeled_window_value(messageset_to_window(messageset, t))

    def average_throughput(window):
        assert window[-1].t_gen - window[0].t_gen <= timeslot_length, window
        return sum([
            m.size*8/timeslot_length/10**6
            for m in window
        ])

    def modeled_window_value(window):
        throughput = average_throughput(window)
        result = rate_model.predict([[throughput*scale]])[0]
        return result/scale

    def _rate_constraint_violated(window):
        if modeled_window_value(window) <= alpha:
            return 0
        else:
            return 1

    def rate_constraint_violations(messageset, incremental=False):
        """
        Returns the number of times the rate constraint is violated.
        """
        if incremental:
            msgs = messageset.msgs_gen_after(
                messageset.message.t_gen - timeslot_length
            )
            return _rate_constraint_violated(tuple(msgs))

        else:
            msgs = messageset.all()
            result = 0
            window = deque()

            for m in msgs:
                window.append(m)
                while m.t_sent - window[0].t_sent > timeslot_length:
                    window.popleft()

                if _rate_constraint_violated(window):
                    result += 1

            return result

    def compute_value(messageset, t):
        return average_rate(messageset_to_window(messageset, t))

    def average_rate(window):
        # rates = [
        #     (m.size*8/10**6)/(m.t_rcv - m.t_gen)
        #     for m in window
        # ]

        # return sum(rates)/len(rates)
        avg_data_inflight = sum([
            m.size * (m.t_rcv - m.t_gen) * 8 / 10**6
            for m in window
        ])/timeslot_length
        avg_latency = sum([
            (m.t_rcv - m.t_gen)
            for m in window
        ])/len(window)
        return abs(avg_data_inflight/avg_latency - avg_data_inflight/delta)
        # return avg_data_inflight/0.01756

    rate_constraint_violations.compute_value = compute_value
    rate_constraint_violations.modeled_value = modeled_value
    rate_constraint_violations.update_model = update_model
    rate_constraint_violations.train_data = train_data
    rate_constraint_violations.model_params_history = params_history

    return rate_constraint_violations
