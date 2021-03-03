#! /usr/bin/env python
# -*- coding: utf-8 -*-


# from .legacy import (  # NOQA
#     BaseExperiment,
#     DropRateVsUtilityExperiment,
#     LimitedThroughputExperiment,
#     GraphMessagesExperiment,
#     BytesInWindowGraph,
#     WindowLengthExperiment,
#     MessageSizeExperiment,
#     TEndExperiment,
#     PlaygroundExperiment,
#     VaryingBackgroundTrafficExperiment,
#     GoodputVsRateExperiment,
# )

from optimization.missions import presets as msgset_presets

# Possible experiments types, e.g.,
# 'goodput_vs_rate'
# 'varying background traffic'
# 'very_small'
# 'small'


def get_results_filename(
    msgset, experiment_cls, agents_num, t_end, results_dir
):
    experiment_name = experiment_cls.__name__
    msgset_id = msgset['ident']
    return (
        f"{results_dir}/results_"
        f"{agents_num}_{t_end}_{msgset_id}_{experiment_name}.pickle"
    )


def run_legacy_experiment(
    experiment_cls,
    experiments_type,
    results_dir="/tmp",
    pub_results_dir=None,
):
    if pub_results_dir is None:
        pub_results_dir = results_dir + '/pub'
    if experiments_type == 'very_small':
        agents_num = 6
        t_end = 10
        MSGSETS = [msgset_presets[0]]
    elif experiments_type == 'small':
        agents_num = 10
        t_end = 100
        MSGSETS = [msgset_presets[i] for i in range(3)]
    elif experiments_type == 'goodput_vs_rate':
        agents_num = 8
        t_end = 10
        MSGSETS = [msgset_presets[5]]
    elif experiments_type == 'varying background traffic small':
        # the name is misleading
        agents_num = 16
        t_end = 100
        MSGSETS = [msgset_presets[6]]
    elif experiments_type == 'varying background traffic':
        agents_num = 8
        t_end = 100
        MSGSETS = [msgset_presets[4]]
    elif experiments_type == 'big':
        agents_num = 10
        t_end = 100
        MSGSETS = [msgset_presets[i] for i in range(3)]
    else:
        raise Exception("Unknown experiments type")

    for msgset in MSGSETS:
        experiment = experiment_cls(agents_num, t_end, msgset)
        experiment.run(debug=2)
        experiment.save_results(
            get_results_filename(
                msgset, experiment_cls, agents_num, t_end, results_dir
            )
        )
        experiment.print_result()
        experiment.save_graphs(
            f'{results_dir}/graphs/'
            f'{agents_num}_{t_end}_{msgset["ident"]}' + '_{}.pdf'
        )
        experiment.save_publication_graphs(
            f'{pub_results_dir}'
        )
