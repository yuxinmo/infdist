import numpy as np
import plotly.graph_objs as go

from . import BaseTreeExperiment
from .base_tree_experiment import generate_configuration_permutations


class WindowLengthExperiment(BaseTreeExperiment):
    def __init__(self, agents_num, t_end):
        self.agents_num = agents_num
        self.t_end = t_end
        self.window_lengths = list(np.linspace(0.1, 20, 20))
        self.throughputs = list(np.linspace(0.2, 2, 4))

    @property
    def configurations(self):
        return generate_configuration_permutations(
            [self.agents_num],
            [self.t_end],
            list(self.throughputs),
            list(self.window_lengths),
            [1],
            ['big'],
        )

    def get_graphs(self):
        r = {}
        for throughput in self.throughputs:
            xs = self.window_lengths
            ys = [
                r[1].stats()['total_utility']
                for r in self.filter_results(throughput_limit=throughput)
            ]
            r[throughput] = go.Scatter(
                name='throughput={}'.format(throughput),
                x=xs,
                y=ys,
                line={
                    'width': 1,
                },
                mode='lines',
            )

        return {
            'window_lengths': list(r.values())
        }

    def get_graph_xaxis_title(self, graph_name):
        return "window length [s]"

    def get_graph_yaxis_title(self, graph_name):
        return "utility"
