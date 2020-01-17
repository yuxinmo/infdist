from optimization.agent import FixedRatioAgent

from simulator.network import Network
from simulator import simulator
from optimization import missions


class Experiment:
    def __init__(self, nodes_num, t_end):
        self.nodes_num = nodes_num
        self.t_end = t_end
        self.agent_cls = FixedRatioAgent
        self.agent_kwargs = {}
        self.messages = None
        self.ctx = None
        self.net = None
        self.agents = None

    def create_agent(self, i):
        return self.agent_cls(i, self.net, **self.agent_kwargs)

    def agent_stats(self):
        return {
            agent: (
                len(agent.received_messages),
                self.ctx.utility(agent.received_messages),
            )
            for agent in self.agents
        }

    def total_stats(self):
        total_utility = sum([
            self.ctx.utility(agent.received_messages)
            for agent in self.agents
        ])
        return (
            sum([len(agent.received_messages) for agent in self.agents]),
            total_utility,
            total_utility/len(self.agents)/self.t_end,
        )

    def print_stats(self):
        # for agent, utility in self.agent_stats().items():
        #     print("{}: received {} msg, utility {}".format(
        #         agent.ident,
        #         len(agent.received_messages),
        #         utility,
        #     ))

        total_rcvd_msgs, total_utility, normalized_utility = self.total_stats()
        print(
            "#: {}, total utility: {}, normalized utility: {}".format(
                total_rcvd_msgs,
                total_utility,
                normalized_utility,
            )
        )

    def finish_mission(self):
        real_t_end = simulator.now_float()
        for a in self.agents:
            a.finish_mission(real_t_end)

    def prepare_messages(self):
        if self.messages is not None:
            assert self.ctx is not None
            return  # already prepared
        self.messages, self.ctx = missions.generate_simple_3D_reconstruction(
            self.t_end, senders=set(range(self.nodes_num)),
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
        self.net = Network(self.nodes_num)

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
            agent = self.agents[m.sender]
            simulator.schedule(m.t_gen, agent.gen_generate_message_callback(m))

        simulator.schedule(self.t_end, self.finish_mission)
        simulator.run()
