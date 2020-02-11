import plotly.graph_objs as go

from . import BaseTreeExperiment
from .base_tree_experiment import generate_configuration_permutations


class TEndExperiment(BaseTreeExperiment):
    def __init__(self, agents_num, t_end):
        self.agents_nums = [3, 5, 10]
        self.t_ends = [100, 30, 10]

    @property
    def configurations(self):
        return generate_configuration_permutations(
            self.agents_nums,
            self.t_ends,
            [1],
            [0.5],
            [1],
            ['big'],
        )

    def get_graphs(self):
        r = {}
        for agents_num in self.agents_nums:
            xs = self.t_ends
            ys = [
                r[1].stats()['total_utility']/t_end
                for t_end, r in
                zip(self.t_ends, self.filter_results(agents_num=agents_num))
            ]
            r[agents_num] = go.Scatter(
                name='agents_num={}'.format(agents_num),
                x=xs,
                y=ys,
                line={
                    'width': 1,
                },
                mode='lines',
            )

        return {
            't_ends': list(r.values())
        }

    def get_graph_xaxis_title(self, graph_name):
        return "t_end [s]"

    def get_graph_yaxis_title(self, graph_name):
        return "utility"
