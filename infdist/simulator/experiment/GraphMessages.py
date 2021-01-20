#! /usr/bin/env python
# -*- coding: utf-8 -*-

from .legacy.groph_messages import GraphMessagesExperiment
from .utils.legacy import run_legacy_experiment

DESCRIPTION = """
    This simple experiment only graphs the messages and their utilities on
    a time scale.
"""

PUBLICATION = """
    Information Distribution in Multi-Robot Systems:
    Utility-Based Evaluation Model
"""


def run():
    run_legacy_experiment(
        GraphMessagesExperiment,
        "very_small"
    )
