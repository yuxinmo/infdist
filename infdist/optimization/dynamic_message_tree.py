from datetime import datetime


from montecarlo.montecarlo import MonteCarlo

from .graph_visualizer import show_graph  # NOQA
from .models import MessageSet
from .dynamic_models import DynamicMessageSet
from .dynamic_models import DynamicTotalUtility
from .tree_node_wrapper import TreeNodeWrapper
from . import simplesim


class DynamicMessageTree:
    def __init__(self, t_end, messages_context, constraints):
        self.past_messages = MessageSet(t_end)
        self.future_messages = DynamicMessageSet(reverse=True)
        self.pessymistic_latency = 0.2
        self.optimistic_latency = 0.01
        self.messages_context = messages_context
        self.max_utility = None
        self.t_end = t_end
        self.debug = False
        self.suppress_warnings = False
        self.constraints = constraints
        self.limit_history = 100

    @property
    def latency(self):
        return (self.pessymistic_latency + self.optimistic_latency)/2

    @latency.setter
    def latency(self, latency):
        self.pessymistic_latency = latency*2
        self.optimistic_latency = latency

    def reinit_mcts(self, message):
        def _child_finder(real_node, mc):
            node = TreeNodeWrapper(real_node)
            new_future = node.future_messages._tail

            if node.future_messages.message:
                node.create_negative_child(new_future)
                node.create_positive_child(new_future)

            if node.message is not None:
                current_t = node.message.t_gen
            else:
                current_t = TreeNodeWrapper(real_node.parent).message.t_gen

            if current_t < 0.01:
                current_t = 0.01

            real_node.update_win_value(
                node.dynamic_utility.value()/self.max_utility
                * self.t_end / current_t
            )
        if self.limit_history:
            self.past_messages._messages = (
                self.past_messages._messages[-self.limit_history:]
            )

        future_messages = self.future_messages
        past_messages = DynamicMessageSet()
        past_messages = past_messages.add_messageset(self.past_messages)
        self.montecarlo = MonteCarlo(TreeNodeWrapper.create(
            past_messages,
            future_messages,
            DynamicTotalUtility(
                self.t_end,
                self.messages_context,
            ).add_messageset(self.past_messages),
            self.constraints,
        ))
        self.montecarlo.child_finder = _child_finder

    def update_future(self, future_messages):
        self.t_end = future_messages.t_end
        self.past_messages.t_end = self.t_end
        simplesim.apply_latency(
            future_messages,
            self.optimistic_latency,
        )
        self.future_messages = DynamicMessageSet(reverse=True)
        self.future_messages = self.future_messages.add_messageset(
            future_messages
        )

        self.max_utility = self.messages_context.utility(
            self.past_messages + future_messages
        ).value()

    def progress_time(self, t):
        self.future_messages = self.future_messages.received_after(
            t-self.pessymistic_latency
        )

    def register_message(self, message):
        if message.t_rcv is None:
            message.t_rcv = message.t_gen + self.optimistic_latency
        if message.t_sent is None:
            message.t_sent = message.t_gen
        # self.future_messages = self.future_messages.tricky_remove(message)

        self.past_messages.append(
            message, False
        )

    def generate_tree_visualization(self, message, simulations_num=100):
        print("Generating tree visualization started")
        for i in range(1, simulations_num):
            print(f'{i}/{simulations_num}')
            self.montecarlo.simulate(1)
            self.show_graph(
                None,
                message,
                f=f'/tmp/mcts/{i:03d}',
                sim_num=i
            )
        self.montecarlo.simulate(200)

        DynamicMessageTree.DEBUGGED_ONCE = True
        print("Generating tree visualization finished")

    def decide(self, message, simulations_num=1500):
        start_time = datetime.now()
        self.reinit_mcts(message)
        mcts_init_time = datetime.now()
        message.t_rcv = message.t_gen + self.optimistic_latency
        self.montecarlo.simulate(simulations_num)
        simulate_time = datetime.now()

        # print('################')
        # print(message)
        # print(self.verbose_repr())
        # if len(TreeNodeWrapper(
        #        self.montecarlo.root_node).future_messages) < 6:
        #     self.debug_once(message)

        try:
            while (
                TreeNodeWrapper(self.montecarlo.root_node).message != message
            ):
                self.montecarlo.root_node = self.montecarlo.make_choice()
            choice = self.montecarlo.make_choice()
        except IndexError:
            if not self.suppress_warnings:
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

        if message in TreeNodeWrapper(choice).sent_messages.all():
            return True
        return False

    def debug_once(self, emphasized_message):
        if not getattr(DynamicMessageTree, "DEBUGGED_ONCE", False):
            print("Debugging!")
            DynamicMessageTree.DEBUGGED_ONCE = True
            self.show_graph(None, emphasized_message)
            print(self.verbose_repr())
            print("Your turn to debug!")

    def debug_once_agents(self):
        self.show_graph()
        self.debug_once_agents = lambda: None
        import time
        time.sleep(1)

    def show_graph_once(self, node=None, emphasized_message=None):
        self.show_graph_once = lambda _: None
        show_graph(node, emphasized_message)

    def show_graph(
        self,
        node=None,
        emphasized_message=None,
        f="/tmp/graph.png",
        sim_num=1500,
    ):
        if node is None:
            node = self.montecarlo.root_node
        show_graph(node, emphasized_message, f=f, sim_num=sim_num)

    def verbose_repr(self):
        fields = ['sender', 't_sent', 't_rcv', 'data_type_name', 'size']
        return (
            "####\n"
            "Past:\n {}\n"
            "Future:\n {}\n"
            "----\n"
        ).format(
            self.past_messages.__str__(fields),
            self.future_messages.__str__(fields),
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
