from copy import copy, deepcopy
from itertools import islice
import random

from .dynamic_message_tree import DynamicMessageTree
from .models import MessageSet
from .message_forecast import MessageForecast


class BaseAgent:
    ACT_DROP = 0
    ACT_SEND = 1

    def __init__(self, ident, net, messages_context, now_func):
        self.ident = ident
        self.received_messages = MessageSet(0)
        self.sent_messages = MessageSet(0)
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
    def process_message(self, m):
        return self.ACT_SEND


class FixedRatioAgent(BaseAgent):
    def __init__(self, *args, **kwargs):
        self.drop_ratio = kwargs.pop('drop_ratio', 0.5)
        super().__init__(*args, **kwargs)

    def process_message(self, m):
        if random.random() > self.drop_ratio:
            return self.ACT_SEND
        return self.ACT_DROP


class BaseTreeAgent(BaseAgent):
    DEFAULT_T_END = 60

    def __init__(self, *args, **kwargs):
        self.agents = kwargs.pop('agents')
        self.constraints = kwargs.pop('constraints')
        super().__init__(*args, **kwargs)

        self.tree = DynamicMessageTree(
            self.DEFAULT_T_END,
            self.messages_context,
            self.constraints,
        )

        self.active = True

    def process_message(self, message):
        if not self.active:
            return self.ACT_DROP
        self.tree.progress_time(message.t_gen)
        est_sent_message = copy(message)
        est_sent_message.t_sent = self.now_func()
        if self.tree.decide(est_sent_message):
            return self.ACT_SEND
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
    def __init__(self, *args, **kwargs):
        all_messages = kwargs.pop('all_messages')
        super().__init__(*args, **kwargs)

        self.all_messages = all_messages

    def process_message(self, message):
        self.tree.update_future(deepcopy(self.all_messages))
        return super().process_message(message)


class EstimatingAgent(BaseTreeAgent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.forecast = MessageForecast(self.messages_context)

    def register_sent(self, message):
        super().register_sent(message)
        self.forecast.register(message)

    def register_received(self, message):
        super().register_received(message)
        self.forecast.register(message)

    def process_message(self, message):
        future_generator = self.forecast.message_generator(
            message.t_gen-self.tree.pessymistic_latency,
            [message],
        )

        # TODO: set max_utility reasonably
        self.tree.update_max_utility(9404)
        self.tree.update_future(
            MessageSet(
                t_end=self.forecast.estimate_t_end(),
                messages=list(islice(future_generator, 2000000))
            )
        )
        return super().process_message(message)
