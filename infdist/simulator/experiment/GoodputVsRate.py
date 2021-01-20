#! /usr/bin/env python
# -*- coding: utf-8 -*-

from .legacy.goodput_vs_rate import GoodputVsRateExperiment
from .utils.legacy import run_legacy_experiment

DESCRIPTION = """
    The experiment plots relationships between publishing rate,
    amount of data inflight and various measurable factors like the value used
    in variable goodput constraint (called 'TCP Vegas' on graphs), latency
    and reception rate. It is helpful to understand how the network behaves.
"""

PUBLICATION = """
    Information Distribution in Multi-Robot Systems:
    Adapting to Varying Communication Conditions
"""


def run():
    run_legacy_experiment(
        GoodputVsRateExperiment,
        "very_small"
    )
