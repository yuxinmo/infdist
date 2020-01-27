from copy import copy

from montecarlo.node import Node
from montecarlo.montecarlo import MonteCarlo

from .graph_visualizer import show_graph  # NOQA
from .models import MessageSet
from optimization import simplesim


class TreeNode(Node):
    def __init__(self, sent_messages, future_messages):
        super().__init__((sent_messages, future_messages))

    @property
    def sent_messages(self):
        return self.state[0]

    @property
    def future_messages(self):
        return self.state[1]

    @property
    def message(self):
        """ Message on which we are deciding """
        future_messages = self.future_messages.all()
        if future_messages:
            return future_messages[0]
        else:
            return None

    def create_positive_child(self, new_future):
        new_sent = MessageSet(
            self.sent_messages.t_end,
            self.sent_messages.all() + [self.message],
        )
        if simplesim.msgnum_constraint_violations(new_sent) > 0:
            return
        positive_child = TreeNode(
            new_sent,
            new_future,
        )
        positive_child.update_policy_value(
            1  # probability that win value in this state is correct
            # we could probably relate it somehow to network utilization
        )
        self.add_child(positive_child)

    def create_negative_child(self, new_future):
        negative_child = TreeNode(
            self.sent_messages,
            new_future,
        )
        negative_child.update_policy_value(
            1  # probability that win value in this state is correct
        )
        self.add_child(negative_child)

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


class CopyingMessageTree:
    def __init__(self, t_end, messages_context, max_utility=None):
        self.past_messages = MessageSet(0)
        self.future_messages = MessageSet(0)
        self.latency = 0.1
        self.messages_context = messages_context
        self.max_utility = None

    def reinit_mcts(self):
        def _child_finder(node, mc):
            new_future = MessageSet(
                node.future_messages.t_end,
                node.future_messages.all()[1:],
            )

            if node.future_messages:
                node.create_negative_child(new_future)
                node.create_positive_child(new_future)

            sent_messages = simplesim.apply_msgnum_constraint(
                node.sent_messages
            )
            node.update_win_value(
                self.messages_context.utility(
                    sent_messages
                ).value()/self.max_utility
            )

        future_messages = copy(self.future_messages)
        self.montecarlo = MonteCarlo(TreeNode(
            copy(self.past_messages),
            future_messages
        ))
        self.montecarlo.child_finder = _child_finder

    def update_future(self, future_messages):
        self.past_messages.t_end = future_messages.t_end
        self.future_messages = copy(future_messages)
        simplesim.latency(
            self.future_messages,
            self.latency,
        )
        if self.max_utility is None:
            self.max_utility = self.messages_context.utility(
                self.future_messages
            ).value()

    def progress_time(self, t):
        self.future_messages = self.future_messages.filter(
            received_after=t-self.latency
        )

    def register_message(self, message):
        if message.t_rcv is None:
            message.t_rcv = message.t_gen + self.latency
        self.future_messages.remove(message)
        self.past_messages.append(message)

    def decide(self, message):
        self.reinit_mcts()
        message.t_rcv = message.t_gen + self.latency
        self.montecarlo.simulate(1000)

        # self.debug_once_agents()
        if message.t_sent > 0:
            self.debug_once()

        while self.montecarlo.root_node.message != message:
            self.montecarlo.root_node = self.montecarlo.make_choice()
        choice = self.montecarlo.make_choice()
        if message in choice.sent_messages.all():
            return True
        return False

    def debug_once(self):
        if not getattr(CopyingMessageTree, "DEBUGGED_ONCE", False):
            CopyingMessageTree.DEBUGGED_ONCE = True
            self.show_graph()

    def debug_once_agents(self):
        self.show_graph()
        self.debug_once_agents = lambda: None
        import time
        time.sleep(1)

    def show_graph(self):
        show_graph(self.montecarlo.root_node)

    def __str__(self):
        return (
            "####\n"
            "Past:\n {}\n"
            "Future:\n {}\n"
            "----\n"
        ).format(
            self.past_messages,
            self.future_messages,
        )
