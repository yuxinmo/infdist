from copy import deepcopy
from functools import lru_cache
import numpy as np

from .trial import GreedyTrial, Trial # NOQA
from . import BaseExperiment
from optimization import simplesim
from simulator.background_traffic import BackgroundTrafficPattern

import plotly.graph_objs as go


def avg(l):
    return sum(l)/(len(l) or 1)


class GoodputVsRateExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.trial_cls = Trial
        self.freqs = list(np.linspace(
            1, 8, 200
        ))[1:]

        self.sigmas = [0.001, 0.01, 0.1]
        self.background_traffic_rates = [0, 0.25, 0.5]

        self.constraints = {
            'TPUT': simplesim.create_throughput_constraint_violations(
                throughput=1.05,
                timeslot_length=2.5,
            ),
            'RATE': simplesim.create_rate_constraint_violations(
                timeslot_length=2.5,
            ),
        }

    def prepare_trial(self, msgset, background_traffic_rate):
        trial = super().prepare_trial(msgset=msgset)
        trial.constraints = self.constraints
        trial.network_data_rate = 1
        trial.prepare_network()

        background_traffic_pattern = BackgroundTrafficPattern(
            [
                i*self.t_end/10
                for i in range(10+1)
            ],
            [
                background_traffic_rate
                for i in range(10)
            ],
        )

        trial.net.add_background_traffic_pattern(
            background_traffic_pattern,
            'udp',
        )
        return trial

    def run(self, debug=False):
        self.result = {}
        i = 0
        for sigma in self.sigmas:
            for background_traffic in self.background_traffic_rates:
                key = (sigma, background_traffic)
                self.result[key] = {}
                for freq in self.freqs:
                    self.progress(
                        100*i/(
                            len(self.freqs) *
                            len(self.sigmas) *
                            len(self.background_traffic_rates)
                        ), debug
                    )
                    i += 1
                    msgset = deepcopy(self.msgset)
                    msgset['f'] = freq
                    msgset['t_start'] = lambda sender: sender/(8*freq)
                    msgset['t_gen'] = lambda t: abs(np.random.normal(t, sigma))
                    trial = self.prepare_trial(msgset, background_traffic)
                    trial.run()
                    self.result[key][freq] = \
                        trial.stats()

    @lru_cache
    def processed_results(self):
        tput_constraint = self.constraints['TPUT']
        rate_constraint = self.constraints['RATE']
        final_result = {}
        for key in self.result.keys():
            result = {}
            for f, stats in self.result[key].items():
                result[f] = {}
                messages = stats['no_duplicates']
                result[f]['tput'] = avg([
                    tput_constraint.compute_value(messages, m.t_gen)
                    for m in messages.gen_after(2.5)
                ])
                result[f]['rate'] = avg([
                    rate_constraint.compute_value(messages, m.t_gen)
                    for m in messages.gen_after(2.5)
                ])
                result[f]['expected_tput'] = (
                    sum([m.size for m in stats['all_messages'].all()]) /
                    stats['t_end'] * 8 / 10**6
                )
                result[f]['inflight'] = (
                    sum([
                        m.size * (m.t_rcv - m.t_gen)
                        for m in stats['no_duplicates'].all()
                    ]) / stats['t_end']
                )
                result[f]['latency'] = (
                    avg([
                        m.t_rcv-m.t_gen
                        for m in stats['no_duplicates'].all()
                    ])  # TODO change to all_messages
                )
            final_result[key] = result
        return final_result

    def print_result(self):
        for key in self.result.keys():
            for f, result in self.processed_results()[key].items():
                print(f" === {key} {f} === ")
                print(result)

    def compute_max_inflight_idx(self, processed_results):
        i = 0

        single_results = list(
            list(processed_results.values())[0].values()
        )
        while (
            i < len(single_results) and
            single_results[i]['inflight'] < 3000
        ):
            i += 1
        return i

    def get_graphs(self):
        results = self.processed_results()
        graph_variants = list(zip(
            [
                (sigma, bg)
                for sigma in self.sigmas
                for bg in self.background_traffic_rates
            ],
            ['solid', 'dash', 'dot']*3,
        ))
        legend = {
            'orientation': 'h',
            'x': 0,
            'y': 1.1,
        }
        max_inflight_index = self.compute_max_inflight_idx(results)

        graphs = {}

        for sth, xaxis_title, max_index, xaxis_range in [
            ('expected_tput', 'publishing rate [Mbps]', -1, [0.17, 1.05]),
            ('inflight', 'amount of data inflight [bytes]',
             max_inflight_index, [250, 3000]),
        ]:
            graphs[f'{sth}_vs_rate'] = go.Figure([
                    self.get_sth_vs_rate_graph(
                        results, sth, max_index,
                        *graph_args
                    )
                    for graph_args in graph_variants
                ],
                layout={
                    'xaxis': {
                        'title': xaxis_title,
                        'range': xaxis_range,
                    },
                    'yaxis': {
                        'title': 'TCP Vegas [Mbps]',
                    },
                    'legend': legend,
                }
            )
            graphs[f'{sth}_vs_expected_goodput'] = go.Figure([
                    self.get_sth_vs_expected_graph(
                        results, sth, max_index,
                        *graph_args
                    )
                    for graph_args in graph_variants
                ],
                layout={
                    'xaxis': {
                        'title': xaxis_title,
                        'range': xaxis_range,
                    },
                    'yaxis': {
                        'title': 'reception rate [Mbps]',
                    },
                    'legend': legend,
                }
            )
            graphs[f'{sth}_vs_latency'] = go.Figure([
                    self.get_sth_vs_latency_graph(
                        results, sth, max_index,
                        *graph_args
                    )
                    for graph_args in graph_variants
                ],
                layout={
                    'xaxis': {
                        'title': xaxis_title,
                        'range': xaxis_range,
                    },
                    'yaxis': {
                        'title': 'latency [s]',
                    },
                    'legend': legend,
                }
            )
        return graphs

    def get_publication_graphs(self):
        results = self.processed_results()
        legend = {
            'orientation': 'h',
            'x': 0,
            'y': 1.1,
        }
        return {
            'sigma_and_goodput': go.Figure([
                    self.get_sth_vs_expected_graph(
                        results,
                        'expected_tput',
                        -1,
                        (sigma, 0),
                        dash,
                        name=f'$\\sigma = {sigma}$',
                    )
                    for sigma, dash in zip(
                        self.sigmas,
                        ['solid', 'dash', 'dot']*3
                    )
                ],
                layout={
                    'xaxis': {
                        'title': 'publishing rate [Mbps]',
                    },
                    'yaxis': {
                        'title': 'reception rate [Mbps]',
                    },
                    'legend': legend,
                }
            ),
            'sigma_and_latency': go.Figure([
                    self.get_sth_vs_latency_graph(
                        results,
                        'expected_tput',
                        -1,
                        (sigma, 0),
                        dash,
                        name=f'$\\sigma = {sigma}$',
                    )
                    for sigma, dash in zip(
                        self.sigmas,
                        ['solid', 'dash', 'dot']*3
                    )
                ],
                layout={
                    'xaxis': {
                        'title': 'publishing rate [Mbps]',
                    },
                    'yaxis': {
                        'title': 'latency [s]',
                    },
                    'legend': legend,
                }
            ),
            'background_and_diff': go.Figure([
                    self.get_sth_vs_rate_graph(
                        results,
                        'expected_tput',
                        -1,
                        (0.01, S),
                        dash,
                        name=f'$S = {S}' + r'\mathrm{\,Mbps}$',
                    )
                    for S, dash in zip(
                        self.background_traffic_rates,
                        ['solid', 'dash', 'dot']*3
                    )
                ],
                layout={
                    'xaxis': {
                        'title': 'publishing rate [Mbps]',
                    },
                    'yaxis': {
                        'title': r'$\overline{|{W}|_\Delta} \mathrm{\,[Mbps]}$'
                    },
                    'legend': legend,
                }
            ),
        }

    def get_sth_vs_rate_graph(
        self, processed_results, sth, max_index, key, dash,
        name=None
    ):
        results = processed_results[key]
        sigma, bg = key
        if name is None:
            name = f'$\\sigma = {sigma}, S = {bg}$'
        return go.Scatter(
            name=name,
            x=[r[sth] for r in results.values()][:max_index],
            y=[r['rate'] for r in results.values()][:max_index],
            line={
                'dash': dash,
            },
        )

    def get_sth_vs_expected_graph(
        self, processed_results, sth, max_index, key, dash,
        name=None
    ):
        results = processed_results[key]
        sigma, bg = key
        if name is None:
            name = f'$\\sigma = {sigma}, S = {bg}$'

        return go.Scatter(
            name=name,
            x=[r[sth] for r in results.values()][:max_index],
            y=[r['tput'] for r in results.values()][:max_index],
            line={
                'dash': dash,
            },
        )

    def get_sth_vs_latency_graph(
        self, processed_results, sth, max_index, key, dash,
        name=None,
    ):
        results = processed_results[key]
        sigma, bg = key

        if name is None:
            name = f'$\\sigma = {sigma}, S = {bg}$'

        return go.Scatter(
            name=name,
            x=[r[sth] for r in results.values()][:max_index],
            y=[r['latency'] for r in results.values()][:max_index],
            line={
                'dash': dash,
            },
        )
