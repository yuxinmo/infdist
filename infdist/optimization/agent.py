from copy import copy, deepcopy
import random

from .dynamic_message_tree import DynamicMessageTree
from .copying_message_tree import CopyingMessageTree
from .models import MessageSet


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
        self.sent_messages.append(message)

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


class FullKnowledgeAgent(BaseAgent):
    def __init__(self, *args, **kwargs):
        all_messages = kwargs.pop('all_messages')
        self.agents = kwargs.pop('agents')
        self.constraints = kwargs.pop('constraints')
        super().__init__(*args, **kwargs)

        if True:
            self.tree = DynamicMessageTree(
                all_messages.t_end,
                self.messages_context,
                self.constraints,
            )
        else:
            self.tree = CopyingMessageTree(
                all_messages.t_end,
                self.messages_context,
            )
        self.tree.update_future(deepcopy(all_messages))

        self.active = True

    def send(self, native_message, message):
        super().send(native_message, message)
        message_copy = copy(message)
        self.tree.register_message(message_copy)

    def process_message(self, message):
        if not self.active:
            return self.ACT_DROP
        self.tree.progress_time(message.t_gen)
        est_sent_message = copy(message)
        est_sent_message.t_sent = self.now_func()
        if self.tree.decide(est_sent_message):
            return self.ACT_SEND
        return self.ACT_DROP

    def received(self, message):
        super().received(message)
        message_copy = copy(message)
        message_copy.receivers = (
            set(self.agents.keys()) - set([message_copy.sender])
        )
        self.tree.register_message(message_copy)
        latency = self.now_func() - message.t_gen
        self.tree.latency = latency
