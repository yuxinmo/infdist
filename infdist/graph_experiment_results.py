import pickle

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


from main_ns3 import (
    AGENTS_NUM, T_END, MSGSETS, get_results_filename,
    EXPERIMENTS_TO_RUN, PUBLICATION_DATA_DIR,
)


def main():
    for msgset in MSGSETS:
        for experiment_cls in EXPERIMENTS_TO_RUN:
            experiment = experiment_cls(AGENTS_NUM, T_END, msgset)
            experiment.result = pickle.load(
                open(get_results_filename(msgset, experiment_cls), 'rb')
            )
            experiment.save_graphs(
                f'/tmp/graphs/'
                f'{AGENTS_NUM}_{T_END}_{msgset["ident"]}' + '_{}.pdf'
            )
            experiment.save_publication_graphs(
                PUBLICATION_DATA_DIR
            )


if __name__ == '__main__':
    main()
