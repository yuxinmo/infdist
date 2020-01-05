class Message:
    def __init__(self, sender, receivers, planned_t_sent,
                 data_type_name, data=None,
                 t_sent=None, t_rcv=None):
        self.sender = sender
        self.receivers = receivers
        self.planned_t_sent = planned_t_sent
        self.t_sent = t_sent
        self.t_rcv = t_rcv
        self.data_type_name = data_type_name
        self.data = data

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


class MessageSet:
    def __init__(self, t_end, messages, t_start=None):
        self.t_end = t_end
        self.t_start = t_start
        self._messages = messages
        self._sort()

    def convert_to_relative_time(self):
        def relative_time(t):
            return (t-self.t_start).total_seconds()
        for msg in self._messages:
            msg.t_sent = relative_time(msg.t_sent)
            msg.t_rcv = relative_time(msg.t_rcv)
        self.t_end = relative_time(self.t_end)
        self.t_start = relative_time(self.t_start)

    def all(self):
        return self._messages

    def _sort(self):
        if not self._messages:
            return

        if self._messages[0].t_sent is None:
            def key(m):
                return m.planned_t_sent
        else:
            def key(m):
                return m.t_sent

        self._messages.sort(key=key)

    def sent_before(self, t):
        """ Implemented using binary search """
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
            self._messages + other._messages,
            min(self.t_start, other.t_start),
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
        return MessageSet(self.t_end, msgs)

    def append(self, message):
        self._messages.append(message)
        self._sort()

    def senders(self):
        return set([
            m.sender
            for m in self.all()
        ])


class InformationType:
    def __init__(self, data_type_name, utility_cls, aggregation_cls):
        self.data_type_name = data_type_name
        self.utility_cls = utility_cls
        self.aggregation_cls = aggregation_cls

    def __repr__(self):
        return "<InformationType({})>".format(self.data_type_name)


class MissionContext:
    def __init__(self, message_types):
        self.message_types = message_types

    def utility_type(self, messages, msg_type):
        aggregation = msg_type.aggregation_cls(
            msg_type.utility_cls(),
            messages.filter(data_type_name=msg_type.data_type_name)
        )
        return aggregation.integrate()

    def utility_by_type(self, messages):
        return {
            msg_type: self.utility_type(messages, msg_type)
            for msg_type in self.message_types
        }

    def utility_by_sender(self, messages, sender):
        return self.utility_by_type(messages.filter(sender=sender))

    def utility_dict(self, messages):
        return {
            sender: self.utility_by_sender(messages, sender)
            for sender in messages.senders()
        }

    def utility(self, messages):
        return sum([
            sum(by_type.values())
            for by_type in self.utility_dict(messages).values()
        ])
