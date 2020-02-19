
class Aggregation:
    def __init__(self, utility_func):
        self.utility_func = utility_func

    def func(self, t, s={}):
        raise NotImplementedError()

    def evaluate(self, ts, states=None):
        if states is None:
            return [self.func(t) for t in ts]
        else:
            return [self.func(t, s) for t, s in zip(ts, states)]

    def integrate(self, states=None):
        raise NotImplementedError()

    def dynamic_integrate(self, states=None):
        raise NotImplementedError()


class AggregationMostRecent(Aggregation):
    """
    Aggregation integrates always only the most recent message.
    The most recent message should also provide the most utility.
    """
    def func(self, messages, t, s):
        msgs = messages.all()
        if not msgs:
            return 0
        else:
            return max(
                self.utility_func(m, t, s)
                for m in msgs
            )

    def _integration_step(self, m, next_m, t_end):
        return self.utility_func.integrate(
            m, m.t_rcv, min(next_m.t_rcv, t_end)
        )

    def integrate(self, messages, debug_weight=1):
        """
        TODO: weight si only used for debugging, remove it
        """
        result = 0
        msgs = messages.all()
        if not msgs:
            return 0

        t_end = messages.t_end

        for m, next_m in zip(msgs[:-1], msgs[1:]):
            gain = self._integration_step(m, next_m, t_end)
            m.gained_utility(
                gain*debug_weight/(min(next_m.t_rcv, t_end)-m.t_rcv)
            )
            result += gain
        result += self.utility_func.integrate(msgs[-1], msgs[-1].t_rcv, t_end)
        return result

    def dynamic_integrate_to_next_message(self, messages, message):
        if messages.message is None:
            return 0
        m = messages.message
        next_m = message
        return self._integration_step(m, next_m, next_m.t_rcv)

    def dynamic_integrate_to_end(self, messages, t_end):
        m = messages.message
        if m is None:
            return 0
        return self.utility_func.integrate(m, m.t_rcv, t_end)
