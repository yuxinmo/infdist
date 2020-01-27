from .models import MessageSet


class DynamicMessageSet:
    __slots__ = (
        '_tail', 'message', 'reverse',
    )

    def __init__(self, reverse=False):
        self._tail = None
        self.message = None
        self.reverse = reverse

    def all(self):
        """ O(n) """
        if self.message is None:
            return []
        msgs = self._tail.all()
        msgs.append(self.message)
        return msgs

    def __repr__(self):
        return (
            'DynamicMessageSet<messages_num={}>'.format(
                len(self.all()),
            )
        )

    def __str__(self, param_names=['sender', 'receivers', 't_gen']):
        return '\n'.join([
            self.__repr__(),
        ] + [
            '\t{} {} '.format(
                m.__str__(param_names),
                '<-' if self.reverse else '->',
            )
            for m in self.all()
        ])

    def plus(self, message, keep_order=True):
        if self.message is not None and keep_order:
            if (
                (not self.reverse and message.t_gen < self.message.t_gen) or
                (self.reverse and message.t_gen > self.message.t_gen)
            ):
                self._tail = self._tail.plus(message)
                return self

            if not self.reverse:
                assert message.t_gen >= self.message.t_gen, (
                    "{} < {}".format(message, self.message)
                )
            else:
                assert message.t_gen <= self.message.t_gen

        d = DynamicMessageSet(self.reverse)
        d.message = message
        d._tail = self
        return d

    def __len__(self):
        return len(self.all())

    def tricky_remove(self, message, depth=0):
        """
        WARNING: it removes message from this DynamicTree and some other
        trees that are based on the deleted message, but NOT all of them.
        Make sure you know what you are doing before using this method.
        """
        if depth > 1000:
            return self
        if self.message == message:
            return self._tail
        if self._tail is None:
            return self

        self._tail = self._tail.tricky_remove(message, depth+1)
        return self

    def add_messageset(self, messageset):
        if self.reverse:
            messages = messageset.all()[::-1]
        else:
            messages = messageset.all()

        head = self
        for m in messages:
            head = head.plus(m)
        return head

    def received_after(self, t):
        assert self.reverse, "received_after only available for reverse sets"
        if self.message is None or self.message.t_rcv >= t:
            return self
        return self._tail.received_after(t)

    def to_messageset(self, t_end):
        return MessageSet(t_end, self.all())


class DynamicAggregatedUtility:
    """
    Stores aggregated utility of a dynamic set of a specific message type.
    """

    __slots__ = (
        'message', 'information_type', 'tail', 'utility_so_far',
    )

    def __init__(self, information_type):
        self.message = None
        self.information_type = information_type
        self.tail = None
        self.utility_so_far = 0

    def plus(self, message):
        d = DynamicAggregatedUtility(self.information_type)
        d.tail = self
        d.message = message
        if self.message:
            aggregation = self.information_type.aggregation
            d.utility_so_far = (
                self.utility_so_far +
                aggregation.dynamic_integrate_to_next_message(
                    self, message
                )
            )
        return d

    def value(self, t_end):
        return (
            self.utility_so_far +
            self.information_type.aggregation.dynamic_integrate_to_end(
                self, t_end
            )
        )


class DynamicTotalUtility:
    """
    Stores total (divided between agents and message types) utility
    of a dynamic message set.
    """

    __slots__ = (
        'utility_dict', 'mission_context', 't_end',
    )

    def __init__(self, t_end, mission_context,
                 utility_dict=None):
        if utility_dict is None:
            utility_dict = {}
        self.utility_dict = utility_dict
        self.mission_context = mission_context
        self.t_end = t_end

    def plus(self, message):
        # we copy the directories, but leave the same DynamicAggregatedUtility
        # objects

        new_utility_dict = {
            receiver: {
                data_type_name: type_utility
                for data_type_name, type_utility in type_utilities.items()
            }
            for receiver, type_utilities in self.utility_dict.items()
        }

        for receiver in message.receivers:
            if receiver not in new_utility_dict:
                new_utility_dict[receiver] = {
                    information_type.data_type_name: DynamicAggregatedUtility(
                        information_type
                    )
                    for information_type in self.mission_context.message_types
                }
            new_utility_dict[receiver][message.data_type_name] = (
                new_utility_dict[receiver][message.data_type_name].plus(
                    message
                )
            )
        return DynamicTotalUtility(
            self.t_end,
            self.mission_context,
            new_utility_dict,
        )

    def value(self):
        return sum([
            sum([
                type_utility.value(self.t_end)
                for type_utility in type_utility.values()
            ])
            for type_utility in self.utility_dict.values()
        ])

    def add_messageset(self, messageset):
        head = self
        for m in messageset.all():
            head = head.plus(m)
        return head
