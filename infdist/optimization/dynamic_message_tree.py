from copy import deepcopy, copy
from datetime import datetime


from montecarlo.node import Node
from montecarlo.montecarlo import MonteCarlo

from .graph_visualizer import show_graph  # NOQA
# from .models import MessageSet
from .dynamic_models import DynamicMessageSet
from .dynamic_models import DynamicTotalUtility
from . import simplesim


class TreeNode(Node):
    def __init__(
        self, sent_messages, future_messages, dynamic_utility, constraints
    ):
        super().__init__((sent_messages, future_messages, dynamic_utility))
        self.constraints = constraints

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
    def message(self):
        """ Message on which we are deciding """
        return self.future_messages.message

    def create_positive_child(self, new_future):
        message = copy(self.message)
        # message.t_sent = message.t_gen
        # message.t_rcv = message.t_gen
        new_sent = self.sent_messages.plus(message, False)
        for constraint in self.constraints.values():
            if constraint(new_sent) > 0:
                return

        positive_child = TreeNode(
            new_sent,
            new_future,
            self.dynamic_utility.plus(self.message),
            self.constraints
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
            self.dynamic_utility,
            self.constraints,
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


class DynamicMessageTree:
    def __init__(self, t_end, messages_context, constraints):
        self.past_messages = DynamicMessageSet()
        self.future_messages = DynamicMessageSet(reverse=True)
        self.pessymistic_latency = 0.2
        self.optimistic_latency = 0.01
        self.messages_context = messages_context
        self.max_utility = None
        self.t_end = t_end
        self.debug = False
        self.constraints = constraints

    @property
    def latency(self):
        return (self.pessymistic_latency + self.optimistic_latency)/2

    @latency.setter
    def latency(self, latency):
        self.pessymistic_latency = latency
        self.optimistic_latency = latency / 2

    def reinit_mcts(self, message):
        def _child_finder(node, mc):
            new_future = node.future_messages._tail

            if node.future_messages.message:
                node.create_negative_child(new_future)
                node.create_positive_child(new_future)

            if node.message is not None:
                current_t = node.message.t_gen
            else:
                current_t = node.parent.message.t_gen

            node.update_win_value(
                node.dynamic_utility.value()/self.max_utility
                * self.t_end / current_t
            )
        future_messages = self.future_messages
        self.montecarlo = MonteCarlo(TreeNode(
            deepcopy(self.past_messages),
            future_messages,
            DynamicTotalUtility(
                self.t_end,
                self.messages_context,
            ).add_messageset(self.past_messages),
            self.constraints,
        ))
        self.montecarlo.child_finder = _child_finder

    def update_future(self, future_messages):
        simplesim.latency(
            future_messages,
            self.optimistic_latency,
        )
        self.future_messages = DynamicMessageSet(reverse=True)
        self.future_messages = self.future_messages.add_messageset(
            future_messages
        )

        if self.max_utility is None:
            self.max_utility = self.messages_context.utility(
                future_messages
            ).value()

    def update_max_utility(self, max_utility):
        self.max_utility = max_utility

    def progress_time(self, t):
        self.future_messages = self.future_messages.received_after(
            t-self.pessymistic_latency
        )

    def register_message(self, message):
        if self.debug:
            print(message)
            print(self.past_messages)

        if message.t_rcv is None:
            message.t_rcv = message.t_gen + self.optimistic_latency
        if message.t_sent is None:
            message.t_sent = message.t_gen
        self.future_messages = self.future_messages.tricky_remove(message)

        self.past_messages = self.past_messages.plus(
            message, False
        )
        if self.debug:
            print(self.past_messages)
            print("-----")

    def decide(self, message):
        start_time = datetime.now()
        self.reinit_mcts(message)
        mcts_init_time = datetime.now()
        message.t_rcv = message.t_gen + self.optimistic_latency
        self.montecarlo.simulate(200)
        simulate_time = datetime.now()

        # if len(self.montecarlo.root_node.future_messages) < 8:
        #     print("Debugging!")
        #     self.debug_once()
        #     print("Your turn to debug!")

        while self.montecarlo.root_node.message != message:
            self.montecarlo.root_node = self.montecarlo.make_choice()
        try:
            choice = self.montecarlo.make_choice()
        except IndexError:
            print(
                "WARNING: the message to decide on is not in the tree! "
                "Not enough simulations? Dropping message."
            )
            return False

        end_time = datetime.now()
        if self.debug:
            print("MCTS init: {}, simulate: {}, all: {}".format(
                mcts_init_time - start_time,
                simulate_time - mcts_init_time,
                end_time - start_time,
            ))

        if message in choice.sent_messages.all():
            return True
        return False

    def debug_once(self):
        if not getattr(DynamicMessageTree, "DEBUGGED_ONCE", False):
            DynamicMessageTree.DEBUGGED_ONCE = True
            self.show_graph()
            print(self.verbose_repr())

    def debug_once_agents(self):
        self.show_graph()
        self.debug_once_agents = lambda: None
        import time
        time.sleep(1)

    def show_graph_once(self, node=None):
        self.show_graph_once = lambda _: None
        show_graph(node)

    def show_graph(self, node=None):
        if node is None:
            node = self.montecarlo.root_node
        show_graph(node)

    def verbose_repr(self):
        return (
            "####\n"
            "Past:\n {}\n"
            "Future:\n {}\n"
            "----\n"
        ).format(
            self.past_messages.__str__(['t_sent', 't_rcv']),
            self.future_messages.__str__(['t_sent', 't_rcv']),
        )

    def __str__(self):
        return (
            "####\n"
            "Past:\n {}\n"
            "Future:\n {}\n"
            "----\n"
        ).format(
            repr(self.past_messages),
            repr(self.future_messages),
        )
