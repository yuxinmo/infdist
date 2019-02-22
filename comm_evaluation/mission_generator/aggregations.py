
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


class AggregationMax(Aggregation):
    def func(self, t, s):
        msgs = self.messages.all()
        if not msgs:
            return 0
        else:
            return max(
                self.utility_func(m, t, s)
                for m in msgs
            )

    def integrate(self):
        # Assumes that max is always the most recent message
        result = 0
        msgs = self.messages.all()
        if not msgs:
            return 0

        t_end = self.messages.t_end

        for m, next_m in zip(msgs[:-1], msgs[1:]):
            result += self.utility_func.integrate(
                m, m.t_rcv, min(next_m.t_rcv, t_end)
            )
        result += self.utility_func.integrate(msgs[-1], msgs[-1].t_rcv, t_end)
        return result
