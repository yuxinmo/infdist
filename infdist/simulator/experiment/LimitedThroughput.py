#! /usr/bin/env python
# -*- coding: utf-8 -*-

from .utils.legacy import run_legacy_experiment
from .legacy import LimitedThroughputExperiment

DESCRIPTION = """
    The experiment is designed to show how the adaptive goodput constraint
    performs under limited throughput conditions.
"""

PUBLICATION = """
    Information Distribution in Multi-Robot Systems:
    Adapting to Varying Communication Conditions
"""


def run():
    run_legacy_experiment(
        LimitedThroughputExperiment,
        "varying background traffic"
    )
