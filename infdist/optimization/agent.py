from copy import copy, deepcopy
from itertools import islice
import random

from .dynamic_message_tree import DynamicMessageTree
from .dynamic_models import DynamicMessageSet
from .models import MessageSet
from .message_forecast import MessageForecast


class BaseAgent:
    ACT_DROP = 0
    ACT_SEND = 1
    ACT_NO_DECISION = -1

    def __init__(self, ident, net, messages_context, now_func):
        self.ident = ident
        self.received_messages = MessageSet(0)
        self.sent_messages = MessageSet(0)
        self.generated_messages = MessageSet(0)
        self.net = net
        self.messages_context = messages_context
        self.now_func = now_func

    def gen_message_received_callback(self):
        def message_received(message):
            self.received(message)
        return message_received

    def send(self, native_message, message):
        assert message.sender == self.ident
        self.net.send(native_message)
        message.t_sent = self.now_func()
        self.register_sent(message)

    def generated(self, native_message):
        message = self.net.deserialize(native_message)
        self.register_generated(message)
        result = self.process_message(message)
        if result == self.ACT_SEND:
            self.send(native_message, message)
        elif result != self.ACT_DROP:
            raise Exception(f"Unknown action {result}")

    def process_message(self, m):
        raise NotImplementedError()

    def received(self, native_message):
        message = self.net.deserialize(native_message)
        self.register_received(message)

    def register_generated(self, message):
        self.generated_messages.append(message)

    def register_sent(self, message):
        self.sent_messages.append(message)

    def register_received(self, message):
        self.received_messages.append(message)

    def gen_generate_message_callback(self, m):
        def _generate_message():
            self.generated(m)
        return _generate_message

    def finish_mission(self, t):
        self.received_messages.t_end = t


class FullCommAgent(BaseAgent):
    """
    This agent sends all messages.
    """
    def process_message(self, m):
        return self.ACT_SEND


class FixedRatioAgent(BaseAgent):
    """
    Agent that randomly drops messages with a predefined probability.
    """
    def __init__(self, *args, **kwargs):
        self.drop_ratio = kwargs.pop('drop_ratio', 0.5)
        super().__init__(*args, **kwargs)

    def process_message(self, m):
        if random.random() > self.drop_ratio:
            return self.ACT_SEND
        return self.ACT_DROP


class ConstrainedAgent(BaseAgent):
    """
    A base class for all agents implementing constraints.
    """
    def __init__(self, *args, **kwargs):
        self.constraints = kwargs.pop('constraints')
        super().__init__(*args, **kwargs)

    def process_message(self, message):
        for constraint in self.constraints.values():
            constraint.update_model(
                self.received_messages + self.sent_messages, self.now_func()
            )
        return self.ACT_NO_DECISION


class GreedyConstrainedAgent(ConstrainedAgent):
    """
    A greedy agent that tries to maintain constraint.

    Keep in mind that no predictions are being made about what other agents
    are sending, so it is very likely that the constraint is not maintained.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def process_message(self, message):
        super().process_message(message)
        est_sent_message = copy(message)
        est_sent_message.t_sent = self.now_func()

        all_messages = self.received_messages + self.sent_messages
        all_messages.append(est_sent_message)
        messages = DynamicMessageSet()
        messages = messages.add_messageset(all_messages)

        for constraint in self.constraints.values():
            if constraint(
                messages, True
            ):
                return self.ACT_DROP
        return self.ACT_SEND


class BaseTreeAgent(ConstrainedAgent):
    """
    Base class for agents implementing a decision tree.
    """
    DEFAULT_T_END = 10

    def __init__(self, *args, **kwargs):
        self.agents = kwargs.pop('agents')
        self.simulations_num = kwargs.pop('simulations_num', 1500)
        limit_history = kwargs.pop('limit_history', 0)
        super().__init__(*args, **kwargs)

        self.tree = DynamicMessageTree(
            self.DEFAULT_T_END,
            self.messages_context,
            self.constraints,
        )
        self.tree.limit_history = limit_history

        self.active = True

    def process_message(self, message):
        super().process_message(message)
        if not self.active:
            return self.ACT_DROP
        self.tree.progress_time(message.t_gen)
        est_sent_message = copy(message)
        est_sent_message.t_sent = self.now_func()
        if self.tree.decide(est_sent_message, self.simulations_num):
            return self.ACT_SEND
        # print("DROPPING")
        return self.ACT_DROP

    def register_sent(self, message):
        super().register_sent(message)
        message_copy = copy(message)
        self.tree.register_message(message_copy)

    def register_received(self, message):
        super().register_received(message)
        message_copy = copy(message)
        message_copy.receivers = (
            set(self.agents.keys()) - set([message_copy.sender])
        )
        self.tree.register_message(message_copy)
        latency = self.now_func() - message.t_gen
        self.tree.latency = latency


class FullKnowledgeAgent(BaseTreeAgent):
    """
    A decision tree-based agent knowing all messages that are going to be
    exchanged in the system.
    """
    def __init__(self, *args, **kwargs):
        all_messages = kwargs.pop('all_messages')
        super().__init__(*args, **kwargs)

        self.all_messages = all_messages

    def process_message(self, message):
        self.tree.update_future(deepcopy(self.all_messages))
        return super().process_message(message)


class EstimatingAgent(BaseTreeAgent):
    """
    A decision tree-based agent that estimates the messages to be exchanged
    by other agents.

    The estimation is made by MessageForecast class based on the mission
    context.
    """
    def __init__(self, *args, **kwargs):
        self.window_size = kwargs.pop('window_size')
        self.future_messages_num = kwargs.pop('future_messages_num', 30)
        super().__init__(*args, **kwargs)
        self.forecast = MessageForecast(self.messages_context)

    def register_sent(self, message):
        super().register_sent(message)
        self.forecast.register(message)

    def register_received(self, message):
        super().register_received(message)
        if message.t_rcv is None:
            message.t_rcv = self.now_func()
        self.forecast.register(message)

    def process_message(self, message):
        future_generator = self.forecast.message_generator(
            message.t_gen-min(
                self.tree.pessymistic_latency,
                self.window_size
            ),
            [message],
        )

        self.tree.update_future(
            MessageSet(
                t_end=self.forecast.estimate_t_end(),
                messages=list(
                    islice(future_generator, self.future_messages_num)
                )
            )
        )
        return super().process_message(message)
