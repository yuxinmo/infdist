from copy import copy, deepcopy
import random

from .dynamic_message_tree import DynamicMessageTree
from .models import MessageSet


class BaseAgent:
    def __init__(self, ident, net):
        self.ident = ident
        self.received_messages = MessageSet(0)
        self.net = net

    def gen_message_received_callback(self):
        def message_received(message):
            self.received(message)
        return message_received

    def send(self, m):
        assert m.sender == self.ident
        self.net.send(m)

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
        """ now func is used to set t_sent for generated messages """
        self.now_func = kwargs.pop('now_func')
        super().__init__(*args, **kwargs)

        self.tree = DynamicMessageTree()
        self.tree.update_future(deepcopy(all_messages))

        self.flag = False

    def send(self, message):
        t_sent = self.now_func()
        super().send(message)
        message_copy = copy(message)
        message_copy.t_sent = t_sent
        self.tree.register_message(message_copy)

    def generated(self, message):
        self.tree.progress_time(message.t_gen)
        if self.flag:
            if self.tree.decide(message):
                print("sending", message)
                self.send(message)
            else:
                print("dropping", message)

    def received(self, message):
        super().received(message)
        message_copy = copy(message)
        message_copy.receivers = (
            set(self.agents.keys()) - set([message_copy.sender])
        )
        self.tree.register_message(message_copy)
