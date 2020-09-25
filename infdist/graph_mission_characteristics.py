#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script was created in order to graph characteristics from
mission conducted on Balboas.
"""


import os
import pickle


from simulator.experiment import (
    DropRateVsUtilityExperiment,
)

from infdist.optimization.models import MessageSet

from camera_streaming.mission import generate_mission_context

RESULTS_DIR = "/home/zeroos/programming/nav/infdist_aws/generic/ros2_overlay_ws/src/infdist/infdist/results/balboa_experiments/fetched"  # NOQA


HOSTS = ['1', '2', '3', '4', '5', '6', '8', '9']
HOSTS_TO_INT_MAPPING = {
    v: i
    for i, v in enumerate(HOSTS)
}
T_END = 95


def get_ctx():
    return generate_mission_context(
        agents=HOSTS,
        data_types=['v'],
        t_end=T_END,
        T=0.5,
    )


def adjust_messages_for_simulation(messages):
    """ Trial class requires that agent ids are integers, this requirement
    is not needed for optimization, though and the robotic experiment
    used string ids.

    Additionally, in the simulations size bigger than MTU is not supported.
    """
    for message in messages.all():
        message.sender = HOSTS_TO_INT_MAPPING[message.sender]
        message.receivers = {
            10
            # int_ident
            # for hostname, int_ident in message.receivers.items()
            # if int_ident != message.sender
        }
        message.size = 2048
        message.data_type_name = f'v{message.sender}'


def main():
    results = {}
    robots = os.listdir(RESULTS_DIR)

    all_messages = MessageSet(T_END)
    for robot in robots:
        results[robot] = pickle.load(open(os.path.join(
            RESULTS_DIR,
            robot,
            'log', 'test_results.pickle'
        ), 'br'))
        all_messages += results[robot]['generated_msgs']
    adjust_messages_for_simulation(all_messages)

    experiment = DropRateVsUtilityExperiment(
        agents_num=8, t_end=T_END, msgset={
            'type': 'serialized',
            'messages': all_messages,
            'ctx': get_ctx(),
        }
    )
    experiment.run(debug=2)
    experiment.save_graphs(
        '/tmp/graph-{}.pdf'
    )


if __name__ == '__main__':
    main()
