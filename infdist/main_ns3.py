import sys

from simulator.experiment.trial import Trial, TreeTrial
from simulator.experiment.base_experiment import (
    BaseExperiment
)
from simulator.experiment.drop_rate_vs_utility import (
    DropRateVsUtilityExperiment
)
from simulator import simulator
from optimization.agent import FullKnowledgeAgent
from visualization.messageset_plotly import write_messageset_plots

from optimization import simplesim

AGENTS_NUM = 6
T_END = 20


def base_experiment():
    experiment = BaseExperiment(AGENTS_NUM, T_END)
    experiment.trial_cls = TreeTrial
    experiment.run(debug=True)
    experiment.print_result()


def drop_rate_experimnets():
    experiment1 = DropRateVsUtilityExperiment(AGENTS_NUM, T_END)
    experiment1.run(debug=True)
    experiment1.print_result()
    experiment1.graph_results()

    experiment2 = DropRateVsUtilityExperiment(AGENTS_NUM, T_END)
    experiment2.trial_cls = TreeTrial
    experiment2.run(debug=True)
    experiment2.print_result()
    experiment2.graph_results("/tmp/tree_experiments.pdf")

    g1 = experiment1.get_graph('random')
    g2 = experiment2.get_graph('tree')
    experiment2.graph_results('/tmp/cumulative.pdf', g1 + g2)


def tree_experiments():
    constraints = {
    }

    e = Trial(AGENTS_NUM, t_end=T_END)
    e.prepare_messages()
    e.agent_cls = FullKnowledgeAgent
    e.agent_kwargs = {
        'all_messages': e.messages,
        'agents': {
            ident: lambda t: set()
            for ident in range(e.nodes_num)
        },
        'now_func': simulator.now_float,
        'constraints': constraints
    }
    e.constraints = constraints
    e.prepare_network()
    e.prepare_agents()
    e.prepare_messages()
    # e.agents[0].active = True

    # e.run()
    # e.print_stats()

    simplesim.latency(e.messages, 0)
    write_messageset_plots(e.messages, e.ctx, '/tmp/plots')


def main():
    base_experiment()
    # tree_experiments()
    # drop_rate_experimnets()


if __name__ == '__main__':
    if 'profile' in sys.argv:
        import cProfile
        cProfile.run('main()', '/tmp/profiler_stats')
    else:
        main()
