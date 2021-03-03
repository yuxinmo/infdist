from copy import deepcopy

from optimization.agent import (  # NOQA
    EstimatingAgent,
    FixedRatioAgent,
    FullCommAgent,
    FullKnowledgeAgent,
    GreedyConstrainedAgent,
)

from simulator.network import NS3Network
from simulator import simulator
from optimization import missions, simplesim
from optimization.models import MessageSet


class Trial:
    def __init__(self, nodes_num, t_end, msgset):
        self.nodes_num = nodes_num
        self.t_end = t_end
        self.msgset = msgset
        self.agent_cls = FullCommAgent
        self.agent_kwargs = {}
        self.messages = None
        self.ctx = None
        self.net = None
        self.agents = None
        self.constraints = {}
        self.now_func = simulator.now_float
        self.network_data_rate = 5.5

    def create_agent(self, i):
        return self.agent_cls(i, self.net, self.ctx, self.now_func,
                              **self.agent_kwargs)

    def agent_stats(self):
        return {
            agent: (
                len(agent.received_messages),
                self.ctx.utility(agent.received_messages).value(),
            )
            for agent in self.agents
        }

    def stats(self):
        total_utility = self.ctx.utility(self.all_received_messages()).value()
        no_duplicates = MessageSet(
            self.all_received_messages().t_end,
            list(set(self.all_received_messages().all())),
        )

        latencies = [
            m.t_rcv - m.t_gen
            for m in self.all_received_messages().all()
        ]
        avg_latency = sum(latencies)/(len(latencies) or 1)

        constraints = {}
        for name, constraint in self.constraints.items():
            constraints[name] = constraint(no_duplicates)

        all_messages = deepcopy(self.messages)
        simplesim.apply_latency(all_messages, 0)
        return {
            'all_messages': all_messages,
            't_end': self.t_end,
            'no_duplicates': no_duplicates,
            'all_received_messages': self.all_received_messages(),
            'received_num': sum(
                [len(agent.received_messages) for agent in self.agents]
            ),
            'sent_num': sum(
                [len(agent.sent_messages) for agent in self.agents]
            ),
            'sent_received_num': len(no_duplicates),
            'total_utility': total_utility,
            'normalized_utility': total_utility/len(self.agents)/self.t_end,
            'total_messages': len(self.messages),
            'constraints': constraints,
            'max_utility': self.ctx.utility(all_messages).value(),
            'avg_latency': avg_latency,
            'agents_num': len(self.agents),
        }

    def all_generated_messages(self):
        result = MessageSet(0, [])
        for agent in self.agents:
            result += agent.generated_messages
        return result

    def all_received_messages(self):
        result = MessageSet(0, [])
        for agent in self.agents:
            result += agent.received_messages
        return result

    @staticmethod
    def print_stats(stats):
        print(
            (
                "Received # {}, sent: {}, "
                "total utility: {}, "
                "normalized utility: {}"
            ).format(
                stats['received_num'],
                stats['sent_num'],
                stats['total_utility'],
                stats['normalized_utility'],
            )
        )

        print((
            "Received {:.0f}% of all messages, "
            "{:.0f}% of sent messages.").format(
            stats['sent_received_num']/stats['total_messages']*100,
            stats['sent_received_num']/stats['sent_num']*100,
        ))
        print("AVG data rate: {:.3f} Mbps with avg latency of {}".format(
            sum([m.size for m in stats['no_duplicates'].all()]) * 8 / 10**6
            / stats['t_end'],
            stats['avg_latency'],
        ))

        print("Max utility: {}".format(
            stats['max_utility']
        ))

        for name, constraint_violations in stats['constraints'].items():
            if constraint_violations > 0:
                print("!!! {} constraint NOT met ({} times)".format(
                    name, constraint_violations
                ))

    def finish_mission(self):
        real_t_end = simulator.now_float()
        for a in self.agents:
            a.finish_mission(real_t_end)

    @staticmethod
    def generate_messages_from_msgset(msgset, t_end, nodes_num):
        msgset_type = msgset.get('type', '3D_reconstruction')
        seed = msgset.get('seed', 0)
        if msgset_type == '3D_reconstruction':
            messages, ctx = \
                missions.generate_simple_3D_reconstruction(
                    t_end,
                    msgset=msgset,
                    senders=set(range(nodes_num)),
                    seed=seed,
                )
        elif msgset_type == 'serialized':
            messages = msgset['messages']
            ctx = msgset['ctx']
        return messages, ctx

    def prepare_messages(self):
        if self.messages is not None:
            assert self.ctx is not None
            return  # already prepared
        self.messages, self.ctx = self.generate_messages_from_msgset(
            self.msgset, self.t_end, self.nodes_num,
        )

    def prepare_agents(self):
        assert self.net is not None, "Network has to be prepared before agents"

        if self.agents is not None:
            return  # already prepared

        self.agents = [
            self.create_agent(i)
            for i in range(self.nodes_num)
        ]

    def prepare_network(self):
        if self.net is not None:
            return  # already prepared
        self.net = NS3Network(self.nodes_num, self.network_data_rate)

    def print_progress(self):
        print(
            f"  {self.now_func():.02f}s "
            f"({self.now_func()/self.t_end*100:.02f}%)",
            end="\r"
        )

    def run(self):
        self.prepare_messages()
        self.prepare_network()

        self.prepare_agents()  # this has to be done after network

        for i in range(self.nodes_num):
            self.net.add_message_received_callback(
                self.agents[i].gen_message_received_callback(),
                i
            )

        for m in self.messages.all():
            # print("Scheduling sending at {} by {}".format(m.t_gen, m.sender))
            native_message = self.net.serialize(m)
            agent = self.agents[m.sender]
            simulator.schedule(m.t_gen, agent.gen_generate_message_callback(
                native_message
            ))

            simulator.schedule(
                m.t_gen,
                self.print_progress
            )

        simulator.schedule(self.t_end, self.finish_mission)
        simulator.stop(self.t_end+1)
        simulator.run()


class FixedRatioTrial(Trial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.agent_cls = FixedRatioAgent

    def set_drop_rate(self, drop_rate):
        self.agent_kwargs = {'drop_ratio': drop_rate}


class GreedyTrial(Trial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.agent_cls = GreedyConstrainedAgent
        self.agent_kwargs = {
            'constraints': {},
        }

    @property
    def constraints(self):
        return self.agent_kwargs['constraints']

    @constraints.setter
    def constraints(self, value):
        self.agent_kwargs['constraints'] = value


class TreeTrial(Trial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.prepare_messages()
        self.agent_cls = EstimatingAgent
        self.agent_kwargs = {
            'agents': {
                ident: lambda t: set()
                for ident in range(self.nodes_num)
            },
            'constraints': {},
            'window_size': 1000,
        }
        self.drop_rate_set = False
        self.throughput_set = False

    @property
    def constraints(self):
        return self.agent_kwargs['constraints']

    @constraints.setter
    def constraints(self, value):
        self.agent_kwargs['constraints'] = value

    def add_msgnum_constraint(self, messages_num, timeslot_length):
        # self.agent_kwargs['window_size'] = timeslot_length
        self.constraints = {
            'MSGNUM': simplesim.create_msgnum_constraint_violations(
                messages_num, timeslot_length
            ),
        }

    def add_throughput_constraint(self, throughput, timeslot_length):
        # self.agent_kwargs['window_size'] = timeslot_length
        self.constraints = {
            'TPUT': simplesim.create_throughput_constraint_violations(
                throughput, timeslot_length,
            ),
        }
        return self.constraints

    def set_throughput(self, throughput):
        return self.add_throughput_constraint(throughput, 2.5)

    def set_drop_rate(self, drop_rate):
        assert not self.drop_rate_set
        timeslot_length = 2.5
        avg_msgs_per_second = 1.5*len(self.messages)/self.t_end
        self.add_msgnum_constraint(
                (1-drop_rate)*(timeslot_length)*avg_msgs_per_second,
                timeslot_length
        )
