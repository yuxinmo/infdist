#! /usr/bin/env python
# -*- coding: utf-8 -*-

from .utils.legacy import run_legacy_experiment
from .legacy import DropRateVsUtilityExperiment

DESCRIPTION = """
    The experiment designed to show the relationship between the amount of
    messages that can be transmitted and utility. It also plots mission
    characteristics as a way to quantify problem difficulty.
"""

PUBLICATION = """
    Information Distribution in Multi-Robot Systems:
    Generic, Utility-Aware Optimization Middleware
"""


def run():
    run_legacy_experiment(
        DropRateVsUtilityExperiment,
        "small"
    )
