class Message:
    def __init__(self, sender, receivers, t_sent, data_type, data=None,
                 t_rcv=None):
        self.sender = sender
        self.receivers = receivers
        self.t_sent = t_sent
        self.t_rcv = t_rcv
        self.data_type = data_type
        self.data = data

    def __repr__(self):
        return (
            'Message<sender={}, receivers={}, t_sent={}, t_rcv={}>'.format(
                self.sender,
                self.receivers,
                self.t_sent,
                self.t_rcv,
            )
        )


class MessageSet:
    def __init__(self, t_end, messages):
        self.t_end = t_end
        self._messages = messages
        self._sort()

    def all(self):
        return self._messages

    def _sort(self):
        self._messages.sort(key=lambda m: m.t_sent)

    def sent_before(self, t):
        msgs = self.all()
        lft = 0
        r = len(msgs)

        while lft != r:
            i = (lft + r) // 2
            if msgs[i].t_sent < t:
                lft = i+1
            else:
                r = i

        return self.all()[0:r]

    def __repr__(self):
        return (
            'MessageSet<messages_num={}, t_end={}>'.format(
                len(self.all()),
                self.t_end
            )
        )

    def __str__(self, param_names=['sender', 'receivers', 't_sent']):
        return '\n'.join([
            self.__repr__(),
        ] + [
            '\tMessage<{}>'.format(
                ', '.join([
                    '{}={}'.format(
                        param_name, str(getattr(m, param_name))
                    )
                    for param_name in param_names
                ])
            )
            for m in self.all()
        ])

    def __add__(self, other):
        return MessageSet(
            max(self.t_end, other.t_end),
            self._messages + other._messages
        )


def __add__(self, other):
    return MessageSet(
        max(self.t_end, other.t_end),
        self._messages + other._messages
    )


class Utility:
    @staticmethod
    def func(m, t, s):
        raise NotImplementedError()

    @staticmethod
    def integrate(m, t_start, t_end, states=None):
        raise NotImplementedError()

    def __call__(self, m, t, s):
        return self.func(m, t, s)


class Aggregation:
    def __init__(self, utility_func, messages):
        self.utility_func = utility_func
        self.messages = messages

    def func(self, t, s={}):
        raise NotImplementedError()

    def evaluate(self, ts, states=None):
        if states is None:
            return [self.func(t) for t in ts]
        else:
            return [self.func(t, s) for t, s in zip(ts, states)]

    def integrate(self, states=None):
        raise NotImplementedError()
