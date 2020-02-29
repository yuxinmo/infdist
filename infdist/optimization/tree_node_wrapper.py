from copy import copy
from montecarlo.node import Node


class TreeNodeWrapper:
    def __init__(self, node):
        self.state = node.state
        self.node = node

    @staticmethod
    def create(
        sent_messages, future_messages, dynamic_utility, constraints
    ):
        return Node(
            (sent_messages, future_messages, dynamic_utility, constraints)
        )

    @property
    def sent_messages(self):
        return self.state[0]

    @property
    def future_messages(self):
        return self.state[1]

    @property
    def dynamic_utility(self):
        return self.state[2]

    @property
    def constraints(self):
        return self.state[3]

    @property
    def message(self):
        """ Message on which we are deciding """
        return self.future_messages.message

    def create_positive_child(self, new_future):
        message = copy(self.message)
        # message.t_sent = message.t_gen
        # message.t_rcv = message.t_gen
        new_sent = self.sent_messages.plus(message, False)
        for constraint in self.constraints.values():
            if constraint(new_sent, True) > 0:
                return

        positive_child = TreeNodeWrapper.create(
            new_sent,
            new_future,
            self.dynamic_utility.plus(self.message),
            self.constraints
        )
        self.node.add_child(positive_child)

    def create_negative_child(self, new_future):
        negative_child = TreeNodeWrapper.create(
            self.sent_messages,
            new_future,
            self.dynamic_utility,
            self.constraints,
        )
        self.node.add_child(negative_child)

    def __str__(self):
        return (
            "{} msg sender: {} t_sent: {:.3f}\n"
            "score: {:.5f}\n"
            "mean val: {:.5f} ({} visits)\n"
            "messages: {} sent, {} to go"
        ).format(
            getattr(self.message, 'data_type_name', 'unk'),
            getattr(self.message, 'sender', 'unk'),
            getattr(self.message, 't_sent', -1),
            getattr(self, 'score', -1),
            self.win_value / (self.visits or 1),
            self.visits,
            len(self.sent_messages),
            len(self.future_messages),
        )
