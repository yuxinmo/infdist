import numpy as np
import plotly.graph_objs as go

from . import BaseTreeExperiment
from .base_tree_experiment import generate_configuration_permutations


class MessageSizeExperiment(BaseTreeExperiment):
    def __init__(self, agents_num, t_end):
        self.agents_num = agents_num
        self.t_end = t_end
        self.network_throughputs = [1, 2, 5.5]
        self.message_sizes = list(np.linspace(2048, 60480, 15))

    @property
    def configurations(self):
        return generate_configuration_permutations(
            [self.agents_num],
            [self.t_end],
            [2],  # network throughput
            [1],  # window length
            self.network_throughputs,
            self.message_sizes,
        )

    def get_graphs(self):
        r = {}
        for throughput in self.network_throughputs:
            xs = self.message_sizes
            ys = [
                r[1].stats()['total_utility']
                for r in self.filter_results(network_throughput=throughput)
            ]
            r[throughput] = go.Scatter(
                name='network throughput={}'.format(throughput),
                x=xs,
                y=ys,
                line={
                    'width': 1,
                },
                mode='lines',
            )

        return {
            'message_sizes': list(r.values())
        }
