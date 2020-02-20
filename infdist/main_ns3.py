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

AGENTS_NUM = 4
T_END = 10
MSGSETS = range(3)
MSGSETS = range(1)


def get_results_filename(msgset):
    return f"/tmp/results_{AGENTS_NUM}_{T_END}_{msgset}.pickle"


def run_experiment(experiment_cls):
    for msgset in MSGSETS:
        experiment = experiment_cls(AGENTS_NUM, T_END, msgset)
        experiment.run(debug=2)
        experiment.save_graphs(
            f'/tmp/graphs/{AGENTS_NUM}_{T_END}_{msgset}' + '_{}.pdf'
        )
        experiment.print_result()
        experiment.save_results(get_results_filename(msgset))


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
