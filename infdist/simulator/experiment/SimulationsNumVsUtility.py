#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import pickle
import plotly.express as px
import plotly.io as pio

from .legacy.trial import TreeTrial
from optimization.missions import presets as msgset_presets


DESCRIPTION = """
    The experiment shows the relationship between the number of simulations
    and utility.
"""

PUBLICATION = None

TRIAL_CLS = TreeTrial
AGENTS_NUM = 30
T_END = 10
# MSGSETS = [msgset_presets[i] for i in range(3)]
MSGSETS = [
    {
        'ident': 'X',
        'max_depl_rate_mi': lambda: np.random.normal(0.2, 0.2),
        'max_depl_rate': lambda mi: np.random.normal(mi, 0.1),
        't_gen': lambda t: abs(np.random.normal(t, 0.1)),
        'topic_weight': lambda i: 2**(i+1),
        'message_size': lambda _: 2048,
    },
]


RESULTS_FILE = '/tmp/SimNumVsUtility.pickle'
GRAPH_FILE = '/tmp/SimNumVsUtility_{}.pdf'

EXPERIMENT_SETUPS = [
    {
        'id': f'{msgset["ident"]}@{sim_num}',
        'simulations_num': sim_num,
        'msgset': msgset,
    }
    # for sim_num in range(0, 1001, 100)
    for sim_num in range(0, 301, 20)
    for msgset in MSGSETS
]


def prepare_trial(simulations_num, msgset):
    trial = TRIAL_CLS(AGENTS_NUM, T_END, msgset)
    trial.set_drop_rate(0.9)
    trial.set_simulations_num(simulations_num)
    trial.set_suppress_warnings(True)

    return trial


def graph_results(results, filename):
    xs = [k for k in results.keys()]
    ys = [r['total_utility']/r['max_utility'] for r in results.values()]

    print(xs, ys)
    fig = px.scatter(
        x=xs,
        y=ys,
    )
    pio.write_image(fig, filename)


def run():
    results = {}
    experiments_num = len(EXPERIMENT_SETUPS)
    for i, setup in enumerate(EXPERIMENT_SETUPS):
        msgset_ident = setup['msgset']['ident']
        if msgset_ident not in results:
            results[msgset_ident] = {}
        print(
            ' ------------- ',
            setup['simulations_num'],
            f'{i}/{experiments_num}'
        )
        t = prepare_trial(
            setup['simulations_num'],
            setup['msgset'],
        )
        t.run()
        results[msgset_ident][setup['id']] = t.stats()
        print('Result:', t.stats()['total_utility']/t.stats()['max_utility'])
        t.print_stats(t.stats())

    with open(RESULTS_FILE, 'wb') as f:
        pickle.dump(results, f)

    for msgset_ident, result_1 in results.items():
        for ident, result in result_1.items():
            print(
                msgset_ident,
                ident,
                'utility', result['total_utility']/result['max_utility'],
                'achieved', 1-(result['sent_received_num']/result['total_messages']),
                'utility/msg', result['total_utility']/result['total_messages'],
            )
        graph_results(result_1, GRAPH_FILE.format(msgset_ident))
