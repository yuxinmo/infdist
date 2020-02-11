import plotly.graph_objs as go

from .base_experiment import BaseExperiment
from .trial import FixedRatioTrial, TreeTrial


def avg(l):
    return sum(l)/(len(l) or 1)


class DropRateVsUtilityExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        drop_rates_num = 20
        self.drop_rates = [
            i/drop_rates_num
            for i in range(drop_rates_num+1)
        ]
        self.repeats = 3
        self.trial_cls = None
        self.trial_clss = [FixedRatioTrial, TreeTrial]

    def prepare_trial(self, trial_cls, drop_rate=0):
        t = super().prepare_trial(trial_cls)
        t.set_drop_rate(drop_rate)
        return t

    def run(self, debug=False):
        result = {}
        for trial_i, trial_cls in enumerate(self.trial_clss):
            trial_name = trial_cls.__name__
            result[trial_name] = {}
            if debug == self.DEBUG_ALL:
                print("Running experiments for {}".format(trial_name))

            for drop_rate_i, drop_rate in enumerate(self.drop_rates):
                if debug == self.DEBUG_ALL:
                    print(' --- drop rate: {} ---'.format(drop_rate))

                self.progress(
                    100*(trial_i*len(self.drop_rates)+drop_rate_i) /
                    (len(self.drop_rates)*len(self.trial_clss)),
                    debug
                )

                result[trial_name][drop_rate] = []
                for i in range(self.repeats):
                    t = self.prepare_trial(trial_cls, drop_rate)
                    t.run()
                    result[trial_name][drop_rate].append(t.stats())
                    result[trial_name][drop_rate][-1]['set_drop_rate'] = \
                        drop_rate
        self.result = result

    def print_result(self):
        print(self.result)

    def _get_single_result_graph(self, results, name, xs_func, ys_func):
        xs = [
            avg([
                xs_func(stat)
                for stat in stat_list])
            for stat_list in results.values()
        ]
        ys = [
            avg([
                ys_func(stat)
                for stat in stat_list
            ])
            for stat_list in results.values()
        ]

        return go.Scatter(
            name=name,
            x=xs,
            y=ys,
            line={
                'width': 1,
            },
            mode='lines',
        )

    def get_graph_name(self):
        return 'cumulative'

    def get_graphs(self):
        g = {}
        for trial_name in [
            trial_cls.__name__ for trial_cls in self.trial_clss
        ]:
            g[trial_name] = {}
            g[trial_name]['set'] = self._get_single_result_graph(
                self.result[trial_name],
                trial_name + ' set',
                lambda stat: stat['set_drop_rate'],
                lambda stat: stat['total_utility'],
            )
            g[trial_name]['achieved'] = self._get_single_result_graph(
                self.result[trial_name],
                trial_name + ' achieved',
                lambda stat: 1-(
                    stat['sent_received_num']/stat['total_messages']),
                lambda stat: stat['total_utility'],
            )

        return {
            self.get_graph_name(): sum([
                [
                    g[trial_name]['set'],
                    g[trial_name]['achieved'],
                ]
                for trial_name in [
                    trial_cls.__name__ for trial_cls in self.trial_clss
                ]
            ], [])
        }
