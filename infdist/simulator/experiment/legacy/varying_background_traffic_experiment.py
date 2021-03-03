import plotly.graph_objs as go

from simulator.background_traffic import BackgroundTrafficPattern

from optimization import simplesim
from visualization import rate_vs_throughput_from_rate_constraint
from . import BaseExperiment
from .trial import GreedyTrial, Trial # NOQA


class VaryingBackgroundTrafficExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.background_traffic_pattern = BackgroundTrafficPattern(
            [
                i*self.t_end/20
                for i in range(20+1)
            ],
            [
                # 0.4 if i < 0 or i > 1 else 0.05
                # 0.5 if i > 1 else 0
                0.1 if i < 10 else 0.25
                for i in range(20)
            ],
        )

        self.constraints = {
            'rate': {
                'TPUT': simplesim.create_throughput_constraint_violations(
                    throughput=1.05,
                    timeslot_length=2.,
                ),
                'RATE': simplesim.create_rate_constraint_violations(
                    timeslot_length=2.,
                    alpha=0.1,
                ),
            },
            'aimd': {
                'TPUT': simplesim.create_throughput_constraint_violations(
                    throughput=1.05,
                    timeslot_length=2.,
                ),
                'AIMD': simplesim.create_aimd_constraint_violations(
                    timeslot_length=1.0,
                    alpha=0.1,
                    a=2048,
                    b=0.998,
                    additive_throughput_diff=0.05,
                    initial_value=0.7*10**6,
                ),
            },
            'aimd_aggressive': {
                'TPUT': simplesim.create_throughput_constraint_violations(
                    throughput=1.05,
                    timeslot_length=2.,
                ),
                'AIMD': simplesim.create_aimd_constraint_violations(
                    timeslot_length=1.0,
                    alpha=0.1,
                    a=2048,
                    b=0.8,
                    additive_throughput_diff=0.05,
                    initial_value=0.7*10**6,
                ),
            },
        }

        self.trial_cls = GreedyTrial
        # self.trial_cls = Trial

    def prepare_trial(self, constraints, *args, **kwargs):
        trial = super().prepare_trial()
        trial.constraints = constraints
        trial.network_data_rate = 1
        trial.prepare_network()
        trial.net.add_background_traffic_pattern(
            self.background_traffic_pattern,
            'udp',
        )
        return trial

    def run(self, *args, **kwargs):
        self.result = {}
        self.resulting_trial = {}
        for trial_name, constraints in self.constraints.items():
            print(f"Running {trial_name} trial...")
            t = self.prepare_trial(constraints)
            t.run()
            self.result[trial_name] = self.get_result(t, constraints)
            self.resulting_trial[trial_name] = t

    def print_result(self):
        for trial_name, constraints in self.constraints.items():
            self.resulting_trial[trial_name].print_stats(
                self.resulting_trial[trial_name].stats()
            )

    def get_result(self, trial, constraints):
        tested_constraint = constraints.get('RATE', constraints.get('AIMD'))
        return {
            'all_received_messages': trial.all_received_messages(),
            'background_noise_history': trial.net.background_noise_history,
            'no_duplicates': trial.stats()['no_duplicates'],
            'train_data': tested_constraint.train_data,
            'model_params_history': tested_constraint.model_params_history,
        }

    def get_latency_graph(self, result):
        messages = result['all_received_messages']
        result = go.Scatter(
            name='latency',
            x=[m.t_rcv for m in messages.all()],
            y=[m.t_rcv - m.t_gen for m in messages.all()],
        )
        return result

    def get_smoothed_latency_graph(self, result):
        messages = result['all_received_messages']
        gen = simplesim.average_latency_generator2(1)
        result = go.Scatter(
            name='smooth latency',
            x=[m.t_gen for m in messages.all()],
            y=[
                gen.send(m)
                for m in messages.all()
            ],
        )
        return result

    def get_constraints_graphs(self, result, constraints):
        messages = result['no_duplicates']
        tested_constraint = constraints.get('RATE', constraints.get('AIMD'))
        return [
            go.Scatter(
                name=f'{constraint_name}',
                x=[m.t_rcv for m in messages.all()],
                y=[constraint.compute_value(messages, m.t_gen)
                   for m in messages.all()],
                yaxis='y2',
                line={
                    'dash': 'dash',
                },
            )
            for constraint_name, constraint in constraints.items()
        ] + [
            go.Scatter(
                name='modeled rate',
                x=[m.t_rcv for m in messages.all()],
                y=[tested_constraint.modeled_value(messages, m.t_gen)
                   for m in messages.all()],
                yaxis='y2',
                line={
                    'dash': 'dash',
                },
            )
        ]

    def get_background_traffic_graphs(self, result, include_set=True):
        graphs = []
        if include_set:
            graphs.append(go.Scatter(
                name='background traffic set',
                x=self.background_traffic_pattern.ts,
                y=self.background_traffic_pattern.throughputs + [
                    self.background_traffic_pattern.throughputs[-1]
                ],
                line_shape='hv',
                yaxis='y2',
            ))
        window_size = 1
        gen = simplesim.sum_generator(window_size)
        graphs.append(go.Scatter(
            name=('background traffic achieved'
                  if include_set else 'background traffic'),
            x=[
                t
                for t, bs in result['background_noise_history']
            ],
            y=[
                gen.send(item)*8/10**6 / window_size
                for item in result['background_noise_history']
            ],
            line_shape='hv',
            yaxis='y2',
            line={
                'dash': 'dot',
                'color': '#AB63FA',
            },
        ))
        return graphs

    def get_model_development_graphs(self, result, include_params=True):
        model_params_history = result['model_params_history']
        alpha = 0.1
        scale = 12
        graphs = [
            go.Scatter(
                name='adaptive constraint',
                x=[
                    t
                    for t, coef, intercept in model_params_history
                ],
                y=[
                    min(
                        1,
                        ((alpha*scale-intercept) / (coef*scale))
                        if coef != 0 else 0
                    )
                    for t, coef, intercept in model_params_history
                ],
                line={
                    'color': 'darkgray',
                },
                yaxis='y2',
            ),
        ]
        if include_params:
            graphs += [
                go.Scatter(
                    name='model coef',
                    x=[
                        t
                        for t, coef, intercept in model_params_history
                    ],
                    y=[
                        coef
                        for t, coef, intercept in model_params_history
                    ],
                    yaxis='y3',
                ),
                go.Scatter(
                    name='model intercept',
                    x=[
                        t
                        for t, coef, intercept in model_params_history
                    ],
                    y=[
                        intercept
                        for t, coef, intercept in model_params_history
                    ],
                    yaxis='y4',
                ),
            ]
        return graphs

    def get_rate_vs_throughput_graphs(self, result, constraints):
        rate_constraint = constraints['RATE']
        tput_constraint = constraints['TPUT']

        messages = result['no_duplicates']

        initial_rate_constraint = simplesim.create_rate_constraint_violations(
            timeslot_length=2.5,
        )

        return rate_vs_throughput_from_rate_constraint.graph(
            tput_constraint,
            rate_constraint,
            messages,
            result['train_data'],
            initial_rate_constraint,
            scale=12,
        )

    def get_varying_background_traffic_graph(
        self,
        result,
        constraints,
        include_model_development_graphs=False,
    ):
        graphs = (
            [
                self.get_latency_graph(result),
                self.get_smoothed_latency_graph(result),
            ]
            + self.get_background_traffic_graphs(result)
            + self.get_constraints_graphs(result, constraints)
        )
        if include_model_development_graphs:
            graphs += self.get_model_development_graphs(result)

        return go.Figure(
            graphs,
            layout={
                'xaxis': {
                    'title': 'time [s]',
                    'domain': [0, 0.9],
                },
                'yaxis': {
                    'title': 'latency [s]',
                },
                'yaxis2': {
                    'title': 'constraint [Mbps]',
                    'side': 'right',
                    'overlaying': 'y',
                },
                'yaxis3': {
                    'title': 'coef',
                    'side': 'right',
                    'overlaying': 'y',
                    'position': 1,
                },
                'yaxis4': {
                    'title': 'coef',
                    'side': 'right',
                    'overlaying': 'y',
                    'position': 1,
                },
                'legend': {
                    'orientation': 'h',
                },
            }
        )

    def get_varying_background_traffic_publication_graph(
        self, result, constraints,
        include_model_development_graphs=False,
        result_aggressive=None,
        constraints_aggressive=None,
    ):
        messages = result['no_duplicates']
        graphs = (
            self.get_background_traffic_graphs(result, include_set=False)
        )
        graphs += [
            go.Scatter(
                name='application goodput',
                x=[m.t_rcv for m in messages.all()],
                y=[constraints['TPUT'].compute_value(messages, m.t_gen)
                   for m in messages.all()],
                yaxis='y2',
                line={
                    'dash': 'dash',
                    'color': '#00CC96',
                },
            )
        ]
        if include_model_development_graphs:
            graphs += self.get_model_development_graphs(
                result,
                include_params=False,
            )
        else:
            messages_aggressive = result_aggressive['no_duplicates']
            graphs += [
                go.Scatter(
                    name='AIMD-based constraint',
                    x=[m.t_rcv for m in messages.all()],
                    y=[constraints['AIMD'].modeled_value(messages, m.t_gen)
                       for m in messages.all()],
                    yaxis='y2',
                    line={
                        'color': 'darkgray',
                    },
                ),
                go.Scatter(
                    name='aggressive AIMD-based constraint',
                    x=[m.t_rcv for m in messages.all()],
                    y=[constraints_aggressive['AIMD'].modeled_value(
                        messages_aggressive, m.t_gen)
                       for m in messages_aggressive.all()],
                    yaxis='y2',
                    line={
                        'color': 'gray',
                    },
                ),
            ]

        return go.Figure(
            graphs,
            layout={
                'xaxis': {
                    'title': 'time [s]',
                    # 'domain': [0, 0.9],
                    'range': [0, 100],
                },
                'yaxis': {
                    'title': 'latency [s]',
                },
                'yaxis2': {
                    'title': 'goodput [Mbps]',
                },
                'yaxis3': {
                    'title': 'coef',
                    'side': 'right',
                    'overlaying': 'y',
                    'position': 1,
                },
                'yaxis4': {
                    'title': 'coef',
                    'side': 'right',
                    'overlaying': 'y',
                    'position': 1,
                },
                'legend': {
                    'orientation': 'h',
                    'yanchor': 'bottom',
                    'y': 1.0,
                    'xanchor': 'left',
                    'x': 0,
                },
            }
        )

    def get_graphs(self):
        graphs = {}

        if 'rate' in self.result:
            graphs['varying_background_traffic_rate'] = \
                self.get_varying_background_traffic_graph(
                    self.result['rate'],
                    self.constraints['rate'],
                    include_model_development_graphs=True,
                )
            graphs['rate_vs_throughput'] = go.Figure(
                self.get_rate_vs_throughput_graphs(
                    self.result['rate'], self.constraints['rate']
                ),
                layout={
                    'xaxis': {
                        'title': 'goodput [Mbps]',
                    },
                    'yaxis': {
                        'title': 'rate [Mbps]',
                    },
                    'legend': {
                        'orientation': 'h',
                    },
                }
            )
        if 'aimd' in self.result:
            graphs['varying_background_traffic_aimd'] = \
                self.get_varying_background_traffic_graph(
                    self.result['aimd'],
                    self.constraints['aimd'],
                )
        return graphs

    def get_publication_graphs(self):
        graphs = {}
        if 'rate' in self.result:
            graphs['varying_background_traffic_rate'] = \
                self.get_varying_background_traffic_publication_graph(
                    self.result['rate'],
                    self.constraints['rate'],
                    include_model_development_graphs=True,
                )
        if 'aimd' in self.result:
            graphs['varying_background_traffic_aimd'] = \
                self.get_varying_background_traffic_publication_graph(
                    self.result['aimd'],
                    self.constraints['aimd'],
                    result_aggressive=self.result['aimd_aggressive'],
                    constraints_aggressive=self.constraints['aimd_aggressive'],
                )
        return graphs
