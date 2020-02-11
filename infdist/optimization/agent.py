from copy import copy, deepcopy
import random

from .dynamic_message_tree import DynamicMessageTree
from .copying_message_tree import CopyingMessageTree
from .models import MessageSet


class BaseAgent:
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

    def send(self, m):
        assert m.sender == self.ident
        self.net.send(m)
        m.t_sent = self.now_func()
        self.sent_messages.append(m)

    def generated(self, m):
        raise NotImplementedError()

    def received(self, message):
        self.received_messages.append(message)

    def gen_generate_message_callback(self, m):
        def _generate_message():
            self.generated(m)
        return _generate_message

    def finish_mission(self, t):
        self.received_messages.t_end = t


class FullCommAgent(BaseAgent):
    def generated(self, m):
        self.send(m)


class FixedRatioAgent(BaseAgent):
    def __init__(self, *args, **kwargs):
        self.drop_ratio = kwargs.pop('drop_ratio', 0.5)
        super().__init__(*args, **kwargs)

    def generated(self, m):
        if random.random() > self.drop_ratio:
            self.send(m)


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

    def send(self, message):
        super().send(message)
        message_copy = copy(message)
        self.tree.register_message(message_copy)

    def generated(self, message):
        if not self.active:
            return

        self.tree.progress_time(message.t_gen)

        est_sent_message = copy(message)
        est_sent_message.t_sent = self.now_func()
        if self.tree.decide(est_sent_message):
            self.send(message)
        else:
            pass

    def received(self, message):
        super().received(message)
        message_copy = copy(message)
        message_copy.receivers = (
            set(self.agents.keys()) - set([message_copy.sender])
        )
        self.tree.register_message(message_copy)
        latency = self.now_func() - message.t_gen
        self.tree.latency = latency
