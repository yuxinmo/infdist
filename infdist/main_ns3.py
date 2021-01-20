#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import pkgutil
import sys
from importlib import import_module

from simulator import experiment


pkgpath = os.path.dirname(experiment.__file__)

available_experiments = [
    name
    for _, name, ispkg in pkgutil.iter_modules([pkgpath])
    if not ispkg
]


parser = argparse.ArgumentParser(description='Run experiment')
parser.add_argument('experiment_name',
                    help='name of the experiment to run',
                    default=None,
                    nargs="?",
                    )

args = parser.parse_args()


if args.experiment_name is None:
    print("No experiment specified! Please choose at least one and "
          "provide its name as a first argument.")
    print("Available experiments:")
    for experiment_name in available_experiments:
        experiment_module = import_module(
            "." + experiment_name,
            "simulator.experiment"
        )
        description = getattr(
            experiment_module, "DESCRIPTION", "no description"
        )
        publication = getattr(
            experiment_module, "PUBLICATION", None
        )
        print(f"=== {experiment_name}")
        if publication is not None:
            print(f"  Published in: {publication}")
        print(f"  Description: {description}")
    sys.exit(2)


experiment_module = import_module(
    "." + args.experiment_name,
    "simulator.experiment"
)
run = getattr(experiment_module, "run", None)
if run is None:
    print("ERROR: experiment must define a 'run' function")
    sys.exit(1)
run()
