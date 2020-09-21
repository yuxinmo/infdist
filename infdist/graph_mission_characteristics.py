#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import pickle

from simulator.experiment import (
    DropRateVsUtilityExperiment,
)

from infdist.optimization.models import MessageSet
from infdist.optimization import mission


def my_mission(t_end, msgset, agents_num):
    assert t_end == 95
    assert msgset == 4
    assert agents_num == 8
    return "A"


mission.generate_simple_3D_reconstruction = my_mission

RESULTS_DIR = "infdist/results/balboa_experiments/fetched"


def main():
    results = {}
    robots = os.listdir(RESULTS_DIR)

    all_messages = MessageSet(0)
    for robot in robots:
        results[robot] = pickle.load(open(os.path.join(
            RESULTS_DIR,
            robot,
            'log', 'test_results.pickle'
        ), 'br'))
        all_messages += results[robot]['generated_msgs']
    print(all_messages)

    experiment = DropRateVsUtilityExperiment(
        agents_num=8, t_end=95, msgset=4
    )
    experiment.run(debug=2)
    experiment.save_graphs(
        '/tmp/graph.pdf'
    )


if __name__ == '__main__':
    main()
