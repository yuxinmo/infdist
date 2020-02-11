from collections import namedtuple
from datetime import datetime
import itertools

from . import BaseExperiment
from .trial import TreeTrial

ExperimentConfiguration = namedtuple(
    'ExperimentConfiguration',
    ' '.join([
        'agents_num',
        't_end',
        'throughput_limit',
        'window_length',
        'network_throughput',
        'msg_size',
    ])
)


def generate_configuration_permutations(*args):
    return [
        ExperimentConfiguration(
            *conf
        )
        for conf in itertools.product(*args)
    ]


class BaseTreeExperiment(BaseExperiment):
    def __init__(self):
        pass

    @property
    def configurations(self):
        raise NotImplementedError()

    def filter_results(self, **kwargs):
        zipped_results = zip(self.configurations, self.results)
        return [
            r for r in zipped_results
            if all(
                getattr(r[0], key) == value
                for key, value in kwargs.items()
            )
        ]

    def prepare_trial(self, config):
        t = TreeTrial(config.agents_num, config.t_end)
        t.network_data_rate = config.network_throughput

        t.prepare_network()

        if config.msg_size == 'small':
            t.net.use_small_packets()
        elif config.msg_size == 'big':
            t.net.use_big_packets()
        else:
            t.packet_size = config.msg_size

        t.add_throughput_constraint(
            config.throughput_limit, config.window_length)

        return t

    def run(self, debug=False):
        start_time = datetime.now()
        results = []
        for i, config in enumerate(self.configurations):
            self.progress(100*i/len(self.configurations), debug, start_time)
            t = self.prepare_trial(config)
            t.run()
            results.append(t)
        self.results = results

    def print_result(self):
        pass
        # print(self.results)
