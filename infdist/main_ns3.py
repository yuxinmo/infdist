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
    VaryingBackgroundTrafficExperiment,
    GoodputVsRateExperiment,
)

from optimization.missions import presets as msgset_presets

# EXPERIMENTS_TYPE = 'goodput_vs_rate'
# EXPERIMENTS_TYPE = 'varying background traffic'
EXPERIMENTS_TYPE = 'small'

if EXPERIMENTS_TYPE == 'very_small':
    AGENTS_NUM = 6
    T_END = 10
    MSGSETS = [msgset_presets[0]]
elif EXPERIMENTS_TYPE == 'small':
    AGENTS_NUM = 10
    T_END = 10
    MSGSETS = [msgset_presets[i] for i in range(3)]
elif EXPERIMENTS_TYPE == 'goodput_vs_rate':
    AGENTS_NUM = 8
    T_END = 10
    MSGSETS = [msgset_presets[5]]
elif EXPERIMENTS_TYPE == 'varying background traffic small':
    AGENTS_NUM = 8
    T_END = 30
    MSGSETS = [msgset_presets[6]]
elif EXPERIMENTS_TYPE == 'varying background traffic':
    AGENTS_NUM = 8
    T_END = 100
    MSGSETS = [msgset_presets[4]]
elif EXPERIMENTS_TYPE == 'big':
    AGENTS_NUM = 10
    T_END = 100
    MSGSETS = [msgset_presets[i] for i in range(3)]
else:
    raise Exception("Unknown experiments type")


def get_results_filename(msgset, experiment_cls):
    experiment_name = experiment_cls.__name__
    msgset_id = msgset['ident']
    return (
        f"/tmp/results_"
        f"{AGENTS_NUM}_{T_END}_{msgset_id}_{experiment_name}.pickle"
    )


PUBLICATION_DATA_DIR = \
    "/home/zeroos/uni/papers/2020-MBarcis-infdist_adaptation/data"


def run_experiment(experiment_cls):
    for msgset in MSGSETS:
        experiment = experiment_cls(AGENTS_NUM, T_END, msgset)
        experiment.run(debug=2)
        experiment.save_results(get_results_filename(msgset, experiment_cls))
        experiment.print_result()
        experiment.save_graphs(
            f'/tmp/graphs/{AGENTS_NUM}_{T_END}_{msgset["ident"]}' + '_{}.pdf'
        )
        # experiment.save_publication_graphs(
        #     PUBLICATION_DATA_DIR
        # )


EXPERIMENTS_TO_RUN = [
    DropRateVsUtilityExperiment,
    # GraphMessagesExperiment,
    # VaryingBackgroundTrafficExperiment,
    # LimitedThroughputExperiment,
    # GoodputVsRateExperiment,
]


def main():
    # run_experiment(LimitedThroughputExperiment)
    # run_experiment(DropRateVsUtilityExperiment)
    # run_experiment(GraphMessagesExperiment)
    # run_experiment(MessageSizeExperiment)
    # run_experiment(BytesInWindowGraph)
    # run_experiment(WindowLengthExperiment)
    # run_experiment(TEndExperiment)
    # run_experiment(PlaygroundExperiment)
    for experiment in EXPERIMENTS_TO_RUN:
        run_experiment(experiment)


if __name__ == '__main__':
    if 'profile' in sys.argv:
        import cProfile
        cProfile.run('main()', '/tmp/profiler_stats')
    else:
        main()
