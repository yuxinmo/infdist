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

from main_ns3 import AGENTS_NUM, T_END, MSGSETS, get_results_filename


def main():
    for msgset in MSGSETS:
        experiment = DropRateVsUtilityExperiment(AGENTS_NUM, T_END, msgset)
        experiment.result = pickle.load(
            open(get_results_filename(msgset), 'rb')
        )
        experiment.save_graphs(
            f'/tmp/graphs/{AGENTS_NUM}_{T_END}_{msgset}' + '_{}.pdf'
        )


if __name__ == '__main__':
    main()
