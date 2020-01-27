import plotly.graph_objs as go
import plotly.io as pio

from .base_experiment import BaseExperiment
from .trial import FixedRatioTrial


class DropRateVsUtilityExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drop_rates = [
            i/10
            for i in range(11)
        ]
        self.trial_cls = FixedRatioTrial

    def prepare_trial(self, drop_rate):
        t = super().prepare_trial()
        t.set_drop_rate(drop_rate)
        return t

    def run(self, debug=False):
        result = {}
        for drop_rate in self.drop_rates:
            if debug:
                print(' --- {} ---'.format(drop_rate))
            t = self.prepare_trial(drop_rate)
            t.run()
            result[drop_rate] = t.stats()
        self.result = result

    def print_result(self):
        print(self.result)

    def get_graph(self, name='unk'):
        xs = list(self.result.keys())
        ys = [
            stat['total_utility']
            for stat in self.result.values()
        ]
        d1 = go.Scatter(
            name=name + ' set',
            x=xs,
            y=ys,
            line={
                'width': 1,
            },
            mode='lines',
        )

        xs = [
            1-(stat['sent_received_num']/stat['total_messages'])
            for stat in self.result.values()
        ]

        ys = [
            stat['total_utility']
            for stat in self.result.values()
        ]
        d2 = go.Scatter(
            name=name + ' achieved',
            x=xs,
            y=ys,
            line={
                'width': 1,
            },
            mode='lines',
        )

        return [d1, d2]

    def graph_results(self, f='/tmp/drop_rate_results.pdf', graph=None):
        if graph is None:
            graph = self.get_graph()
        fig = go.Figure({'data': graph})
        pio.write_image(fig, f)
