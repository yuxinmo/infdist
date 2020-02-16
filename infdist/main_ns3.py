import sys

from simulator.experiment import (  # NOQA
    BaseExperiment,
    DropRateVsUtilityExperiment,
    LimitedThroughputExperiment,
    GraphMessagesExperiment,
    BytesInWindowGraph,
    WindowLengthExperiment,
    MessageSizeExperiment,
    TEndExperiment,
    PlaygroundExperiment,
)

# AGENTS_NUM = 10
#AGENTS_NUM = 2
#T_END = 3
AGENTS_NUM = 10
T_END = 30


def run_experiment(experiment_cls):
    experiment = experiment_cls(AGENTS_NUM, T_END)
    experiment.run(debug=2)
    experiment.print_result()
    experiment.save_graphs()


def main():
    experiment_clss = [
        DropRateVsUtilityExperiment,
        LimitedThroughputExperiment,
        GraphMessagesExperiment,
        WindowLengthExperiment
    ]
    experiment_clss
    # run_experiment(LimitedThroughputExperiment)
    # run_experiment(DropRateVsUtilityExperiment)
    # run_experiment(GraphMessagesExperiment)
    # run_experiment(MessageSizeExperiment)
    # run_experiment(BytesInWindowGraph)
    # run_experiment(WindowLengthExperiment)
    # run_experiment(TEndExperiment)
    run_experiment(PlaygroundExperiment)


if __name__ == '__main__':
    if 'profile' in sys.argv:
        import cProfile
        cProfile.run('main()', '/tmp/profiler_stats')
    else:
        main()
