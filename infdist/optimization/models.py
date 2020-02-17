from types import SimpleNamespace


class Message:
    __slots__ = (
        'sender', 'receivers', 't_gen', 't_sent', 't_rcv', 'data_type_name',
        'data',
    )

    def __init__(self, sender, receivers, t_gen,
                 data_type_name, data=None,
                 t_sent=None, t_rcv=None):
        self.sender = sender
        self.receivers = receivers
        self.t_gen = t_gen
        self.t_sent = t_sent
        self.t_rcv = t_rcv
        self.data_type_name = data_type_name
        self.data = SimpleNamespace(**data)

    def __hash__(self):
        return hash((
            self.sender,
            self.t_sent,
            self.data_type_name,
        ))

    def __repr__(self):
        return (
            'Message<sender={}, receivers={},'
            't_sent={}, t_rcv={}, data_type_name={}>'.format(
                self.sender,
                self.receivers,
                self.t_sent,
                self.t_rcv,
                self.data_type_name,
            )
        )

    def __str__(self, param_names=['sender', 't_gen']):
        return 'Message<{}>'.format(
            ', '.join([
                '{}={}'.format(
                    param_name, str(getattr(self, param_name))
                )
                for param_name in param_names
            ])
        )

    def __eq__(self, other):
        if other is None:
            return False
        eps = 0.01
        return (
            abs(self.t_gen - other.t_gen) < eps and
            (self.sender == other.sender)
        )


class MessageSet:
    __slots__ = (
        't_end', 't_start', '_messages'
    )

    def __init__(self, t_end, messages=None, t_start=0):
        if messages is None:
            messages = []
        self.t_end = t_end
        self.t_start = t_start
        self._messages = messages
        self._sort()

    def convert_to_relative_time(self):
        def relative_time(t):
            if t is None:
                return None
            return (t-self.t_start).total_seconds()
        for msg in self.all():
            msg.t_sent = relative_time(msg.t_sent)
            msg.t_gen = relative_time(msg.t_gen)
            msg.t_rcv = relative_time(msg.t_rcv)
        self.t_end = relative_time(self.t_end)
        self.t_start = relative_time(self.t_start)

    def all(self):
        return self._messages

    def _sort(self):
        if not self._messages:
            return

        def key(m):
            return m.t_gen

        self._messages.sort(key=key)

    def received_after(self, t):
        return self.all()[self.supremum_idx(t, 't_rcv'):]

    def gen_after(self, t):
        return self.all()[self.supremum_idx(t, 't_gen'):]

    def supremum_idx(self, t, attribute_name='t_gen'):
        """ Implemented using binary search """
        msgs = self.all()
        lft = 0
        r = len(msgs)

        while lft != r:
            i = (lft + r) // 2
            if getattr(msgs[i], attribute_name) < t:
                lft = i+1
            else:
                r = i

        return r

    def __repr__(self):
        return (
            'MessageSet<messages_num={}, t_end={}>'.format(
                len(self.all()),
                self.t_end
            )
        )

    def __str__(self, param_names=['sender', 'receivers', 't_gen']):
        return '\n'.join([
            self.__repr__(),
        ] + [
            '\t{}'.format(
                m.__str__(param_names)
            )
            for m in self.all()
        ])

    def __len__(self):
        return len(self._messages)

    def __add__(self, other):
        t_start = min(self.t_start, other.t_start)
        return MessageSet(
            max(self.t_end, other.t_end),
            self._messages + other._messages,
            t_start,
        )

    def filter(self, **kwargs):
        msgs = self.all()
        if 'sender' in kwargs:
            msgs = [
                msg for msg in msgs
                if msg.sender == kwargs['sender']
            ]
        if 'receiver' in kwargs:
            msgs = [
                msg for msg in msgs
                if kwargs['receiver'] in msg.receivers
            ]
        if 'data_type' in kwargs:
            msgs = [
                msg for msg in msgs
                if kwargs['data_type'] == msg.data_type_name
            ]
        if 'received_after' in kwargs:
            msgs = self.received_after(kwargs['received_after'])
        if 'gen_after' in kwargs:
            msgs = self.gen_after(kwargs['gen_after'])
        return MessageSet(self.t_end, msgs)

    def append(self, message, keep_order=True):
        self._messages.append(message)
        if keep_order:
            self._sort()

    def remove(self, message):
        self._messages = [
            m
            for m in self.all()
            if m != message
        ]

    def senders(self):
        return set([
            m.sender
            for m in self.all()
        ])

    def receivers(self):
        receivers = set()
        for m in self.all():
            receivers.update(m.receivers)
        return receivers


class InformationType:
    __slots__ = (
        'data_type_name', 'utility_cls', 'aggregation', 'weight',
        'message_forecast_cls', 'message_forecast_kwargs',
    )

    def __init__(
        self,
        data_type_name,
        utility_cls,
        aggregation_cls,
        message_forecast_cls,
        message_forecast_kwargs={},
        weight=1
    ):
        self.data_type_name = data_type_name
        self.utility_cls = utility_cls
        self.aggregation = aggregation_cls(utility_cls())
        self.message_forecast_cls = message_forecast_cls
        self.message_forecast_kwargs = message_forecast_kwargs
        self.weight = weight

    def __repr__(self):
        return "<InformationType({})>".format(self.data_type_name)


class TotalUtility:
    __slots__ = (
        'utility_dict',
    )

    def __init__(self, utility_dict):
        self.utility_dict = utility_dict

    def value(self):
        return sum([
            sum(type_utility.values())
            for type_utility in self.utility_dict.values()
        ])


class MissionContext:
    __slots__ = (
        'message_types',
    )

    def __init__(self, message_types):
        self.message_types = message_types

    def utility_type(self, messages, msg_type):
        return msg_type.weight * msg_type.aggregation.integrate(
            messages.filter(data_type=msg_type.data_type_name)
        )

    def utility_by_type(self, messages):
        return {
            msg_type: self.utility_type(messages, msg_type)
            for msg_type in self.message_types
        }

    def utility_by_receiver(self, messages, receiver):
        return self.utility_by_type(messages.filter(receiver=receiver))

    def _utility_dict(self, messages):
        return {
            receiver: self.utility_by_receiver(messages, receiver)
            for receiver in messages.receivers()
        }

    def utility(self, messages):
        return TotalUtility(
            self._utility_dict(messages)
        )

    def __add__(self, other):
        return MissionContext(
            self.message_types.union(other.message_types)
        )
