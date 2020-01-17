from simulator.experiment import Experiment
from simulator import simulator
from optimization.agent import FullKnowledgeAgent


def drop_rate_experimnets():
    for drop_ratio in [0, 0.25, 0.5, 0.75, 1]:
        print(" ----- {} ---- ".format(drop_ratio))
        e = Experiment(50, 20)
        e.agent_kwargs = {'drop_ratio': drop_ratio}
        e.run()
        e.print_stats()


def tree_experiments():
    e = Experiment(3, t_end=4)
    e.prepare_messages()
    e.agent_cls = FullKnowledgeAgent
    e.agent_kwargs = {
        'all_messages': e.messages,
        'agents': {
            ident: lambda t: set()
            for ident in range(e.nodes_num)
        },
        'now_func': simulator.now_float
    }
    e.prepare_network()
    e.prepare_agents()
    e.agents[0].flag = True
    e.run()
    e.print_stats()


def main():
    tree_experiments()


if __name__ == '__main__':
    main()
