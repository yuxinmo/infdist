#! /usr/bin/env python
# -*- coding: utf-8 -*-

from .legacy.varying_background_traffic_experiment import (
    VaryingBackgroundTrafficExperiment
)
from .utils.legacy import run_legacy_experiment

DESCRIPTION = """
    The experiment shows how the adaptive goodput constraint performs under
    varying background traffic.
"""

PUBLICATION = """
    Information Distribution in Multi-Robot Systems:
    Adapting to Varying Communication Conditions
"""


def run():
    run_legacy_experiment(
        VaryingBackgroundTrafficExperiment,
        "varying background traffic small",
        pub_results_dir="/home/zeroos/uni/papers/2020-MBarcis-infdist_adaptation/data/"
    )
