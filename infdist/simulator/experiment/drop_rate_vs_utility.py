from copy import deepcopy
import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import plotly.express as px
from statsmodels.distributions.empirical_distribution import ECDF

from .base_experiment import BaseExperiment
from .trial import FixedRatioTrial, TreeTrial


_PLOTLY_COLORS = [
    '#1f77b4',  # muted blue
    '#ff7f0e',  # safety orange
    '#2ca02c',  # cooked asparagus green
    '#d62728',  # brick red
    '#9467bd',  # muted purple
    '#8c564b',  # chestnut brown
    '#e377c2',  # raspberry yogurt pink
    '#7f7f7f',  # middle gray
    '#bcbd22',  # curry yellow-green
    '#17becf',   # blue-teal
]


def avg(l):
    return sum(l)/(len(l) or 1)


class DropRateVsUtilityExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        drop_rates_num = 30
        self.drop_rates = [
            i/drop_rates_num
            for i in range(drop_rates_num+1)
        ]
        self.repeats = [1, 1]
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
                for i in range(self.repeats[trial_i]):
                    t = self.prepare_trial(trial_cls, drop_rate)
                    t.run()
                    result[trial_name][drop_rate].append(t.stats())
                    result[trial_name][drop_rate][-1]['set_drop_rate'] = \
                        drop_rate
        self.result = result

    def print_result(self):
        everything_fine = True
        for trial_cls in self.trial_clss:
            trial_name = trial_cls.__name__
            for drop_rate in self.drop_rates:
                for stats in self.result[trial_name][drop_rate]:
                    for name, constraint_violations in (
                        stats['constraints'].items()
                    ):
                        if constraint_violations > 0:
                            print(
                                f"!!! {trial_name}/{drop_rate}: "
                                f"{name} constraint violated "
                                f"{constraint_violations} times!!!"
                            )
                            everything_fine = False
        if everything_fine:
            print("All constraints met.")

    def _get_single_result_graph(
        self, results, name, xs_func, ys_func, style,
    ):
        symbol = 3 if style == 0 else 4
        color = _PLOTLY_COLORS[style+2]
        dash = 'dash' if style == 0 else 'dot'
        xs = [
            xs_func(stat)
            for stat in sum(results.values(), [])
        ]
        ys = [
            ys_func(stat)
            for stat in sum(results.values(), [])
        ]

        fig = px.scatter(
            x=xs,
            y=ys,
            # trendline='lowess',
        )
        for d in fig.data:
            d['marker'] = {
                'color': color,
                'symbol': symbol,
            }
            d['line'] = {
                'dash': dash,
            }
        return fig
        # return go.Scatter(
        #     name=name,
        #     x=xs,
        #     y=ys,
        #     mode='markers',
        #     marker_symbol=symbol,
        #     marker_size=4,
        # )

    def get_graph_name(self):
        return 'cumulative'

    def get_cumulative_graph(self):
        g = {}
        for i, trial_name in enumerate([
            trial_cls.__name__ for trial_cls in self.trial_clss
        ]):
            g[trial_name] = {}
            g[trial_name]['set'] = self._get_single_result_graph(
                self.result[trial_name],
                trial_name + ' set',
                lambda stat: stat['set_drop_rate'],
                lambda stat: stat['total_utility'],
                style=i,
            )
            g[trial_name]['achieved'] = self._get_single_result_graph(
                self.result[trial_name],
                trial_name + ' achieved',
                lambda stat: 1-(
                    stat['sent_received_num']/stat['total_messages']),
                lambda stat: stat['total_utility']/stat['max_utility'],
                style=i,
            )

        return [
            g[trial_name]['achieved']
            for trial_name in [
                trial_cls.__name__ for trial_cls in self.trial_clss
            ]
        ]

    def get_graphs(self):
        graphs = {
            'histogram_fixed': self.get_message_utility_histogram(),
            'histogram_tree': self.get_message_utility_histogram('TreeTrial'),
            'cdf_fixed': self.get_message_utility_cdf(),
            'cdf_tree': self.get_message_utility_cdf('TreeTrial'),
        }
        graphs['drop_rate_experiment'] = self.make_final_figure(
            self.get_cumulative_graph(),
            graphs['histogram_fixed'][0],
            graphs['cdf_fixed'][0],  # TODO: instead of fixed we should have sth like random  # NOQA
        )
        graphs['legend'] = self.extract_legend(
            self.get_cumulative_graph(),
            graphs['histogram_fixed'][0],
            graphs['cdf_fixed'][0],  # TODO: instead of fixed we should have sth like random  # NOQA
        )
        return graphs

    def extract_legend(self, cumulative, histogram, cdf):
        fig = go.Figure()

        names = [
            "Information Distribution Middleware",
            "random method",
        ]
        for cumulative_fig in cumulative:
            for d in cumulative_fig.data:
                d['name'] = names.pop()
                d['showlegend'] = True
                fig.add_trace(
                    d,
                )
        fig.add_trace(histogram)
        fig.add_trace(cdf)

        fig.update_layout(
            legend=dict(
                x=0, y=1,
                bgcolor="white",
                orientation="h",
            ),
            margin=go.layout.Margin(
                l=0,  # NOQA
                r=0,
                b=0,
                t=0,
                pad=0
            ),
            autosize=False,
            width=660,
            height=22,
        )
        fig.update_yaxes(
            range=[100, 100.0001],
            showticklabels=False,
        )
        fig['layout']['showlegend'] = True
        return fig

    def make_final_figure(
        self, cumulative, histogram, cdf
    ):
        fig = make_subplots(rows=2, cols=1)
        for cumulative_fig in cumulative:
            for d in cumulative_fig.data:
                fig.append_trace(
                    d,
                    row=1, col=1
                )
        x_title_standoff = 5
        y_title_standoff = 5
        fig.update_xaxes(
            title_text="drop rate",
            title_standoff=x_title_standoff,
            range=[0, 1],
            row=1, col=1,
        )
        fig.update_yaxes(
            title_text="% of utility",
            title_standoff=y_title_standoff,
            range=[0, 1],
            row=1, col=1,
        )

        fig.append_trace(
            histogram,
            row=2, col=1
        )
        fig.append_trace(
            cdf,
            row=2, col=1
        )
        fig.update_xaxes(
            title_text="message utility",
            title_standoff=x_title_standoff,
            range=[0, 1],
            ticktext=["low", "medium", "high"],
            tickvals=[0, 0.5, 1],
            row=2, col=1,
        )
        fig.update_yaxes(
            title_text="% of messages",
            title_standoff=y_title_standoff,
            range=[0, 1],
            row=2, col=1,
        )

        fig.update_layout({
            "margin": go.layout.Margin(
                    l=40,  # NOQA
                    r=0,
                    b=0,
                    t=0,
                    pad=0
                ),
            "width": 300,
            "height": 370,
            "showlegend": False,
        })
        return fig

    def _prepare_histogram_data(
        self, stat,
    ):
        ctx = stat['context']
        utility = ctx.utility(stat['all_received_messages']).value()
        utility
        gained_values = [
            getattr(m, 'gained_value')
            for m in stat['all_received_messages'].all()
            if hasattr(m, 'gained_value')
        ]
        normalized_data = [
            v / max(gained_values)
            for v in gained_values
        ]
        return normalized_data

    def _get_single_gained_value_histogram(
        self, trial_name, drop_rate, try_num
    ):
        stat = self.result[trial_name][drop_rate][try_num]
        normalized_data = self._prepare_histogram_data(stat)
        achieved_drop_rate = (
            1-(stat['sent_received_num']/stat['total_messages'])
        )
        return go.Histogram(
            nbinsx=7,
            x=normalized_data,
            histnorm='probability',
            name=f"{trial_name} {achieved_drop_rate:.02f}",
        )

    def get_message_utility_histogram(self, trial_name='FixedRatioTrial'):
        # return self._multiple_histograms_on_one(trial_name)
        return self._single_histogram(trial_name)

    def get_message_utility_cdf(self, trial_name='FixedRatioTrial'):
        return self._single_cdf(trial_name)

    def _single_cdf(self, trial_name):
        data = []
        for drop_rate in self.result[trial_name].keys():
            for stat in self.result[trial_name][drop_rate]:
                data += self._prepare_histogram_data(stat)
        data = [
            v  # / max(data)
            for v in data
        ]
        ecdf = ECDF(data)
        return [go.Scatter(
            name="empirical distribution function",
            x=np.unique(data),
            y=ecdf(np.unique(data)),
            line_shape='hv',
            line_color='royalblue',
        )]

    def _single_histogram(self, trial_name):
        data = []
        for drop_rate in self.result[trial_name].keys():
            stat = self.result[trial_name][drop_rate][0]
            normalized_data = self._prepare_histogram_data(stat)
            data = data + normalized_data
        data = [
            v  # / max(data)
            for v in data
        ]
        return [go.Histogram(
            xbins={
                'start': 0,
                'end': 1,
                'size': 0.2,
            },
            x=data,
            histnorm='probability',
            name=f"histogram",
            marker_color='lightblue',
        )]

    def _multiple_histograms_on_one(self, trial_name):
        graphs = [
            self._get_single_gained_value_histogram(
                trial_name, drop_rate, 0
            )
            for drop_rate in self.result[trial_name].keys()
        ]
        fig = go.Figure()
        for g in graphs:
            fig.add_trace(g)
        fig.update_layout(barmode='overlay')
        fig.update_traces(opacity=0.3)

        return fig
