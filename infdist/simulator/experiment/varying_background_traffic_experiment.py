
import plotly.graph_objs as go

from simulator.background_traffic import BackgroundTrafficPattern

from optimization import simplesim
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
                0.5 if i > 1 else 0
                for i in range(20)
            ],
        )

        self.constraints = {
            'TPUT': simplesim.create_throughput_constraint_violations(
                throughput=1.05,
                timeslot_length=2.,
            ),
            'RATE': simplesim.create_rate_constraint_violations(
                timeslot_length=2.,
            ),

        }
        # self.trial_cls = GreedyTrial
        # self.trial_cls = Trial

    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.constraints = self.constraints
        trial.network_data_rate = 1
        trial.prepare_network()
        trial.net.add_background_traffic_pattern(
            self.background_traffic_pattern,
            'udp',
        )
        return trial

    def get_result(self, trial):
        rate_constraint = self.constraints['RATE']
        return {
            'all_received_messages': trial.all_received_messages(),
            'background_noise_history': trial.net.background_noise_history,
            'no_duplicates': trial.stats()['no_duplicates'],
            'train_data': rate_constraint.train_data,
            'model_params_history': rate_constraint.model_params_history,
        }

    def get_latency_graph(self):
        messages = self.result['all_received_messages']
        result = go.Scatter(
            name='latency',
            x=[m.t_rcv for m in messages.all()],
            y=[m.t_rcv - m.t_gen for m in messages.all()],
        )
        return result

    def get_smoothed_latency_graph(self):
        messages = self.result['all_received_messages']
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

    def get_constraints_graphs(self):
        messages = self.result['no_duplicates']
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
            for constraint_name, constraint in self.constraints.items()
        ] + [
            go.Scatter(
                name='modeled rate',
                x=[m.t_rcv for m in messages.all()],
                y=[self.constraints['RATE'].modeled_value(messages, m.t_gen)
                   for m in messages.all()],
                yaxis='y2',
                line={
                    'dash': 'dash',
                },
            )
        ]

    def get_background_traffic_graphs(self):
        set_values = go.Scatter(
            name='background traffic set',
            x=self.background_traffic_pattern.ts,
            y=self.background_traffic_pattern.throughputs + [
                self.background_traffic_pattern.throughputs[-1]
            ],
            line_shape='hv',
            yaxis='y2',
        )
        window_size = 1
        gen = simplesim.sum_generator(window_size)
        received = go.Scatter(
            name='background traffic achieved',
            x=[
                t
                for t, bs in self.result['background_noise_history']
            ],
            y=[
                gen.send(item)*8/10**6 / window_size
                for item in self.result['background_noise_history']
            ],
            line_shape='hv',
            yaxis='y2',
        )
        return [set_values, received]

    def get_model_development_graphs(self):
        model_params_history = self.result['model_params_history']
        return [
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

    def get_rate_vs_throughput_graphs(self):
        rate_constraint = self.constraints['RATE']
        tput_constraint = self.constraints['TPUT']

        messages = self.result['no_duplicates']

        initial_rate_constraint = simplesim.create_rate_constraint_violations(
            timeslot_length=2.5,
        )

        return [
            go.Scatter(
                name='data',
                x=[
                    tput_constraint.compute_value(messages, m.t_gen)
                    for m in messages.all()
                ],
                y=[
                    rate_constraint.compute_value(messages, m.t_gen)
                    for m in messages.all()
                ],
                mode='markers',
            ),
            go.Scatter(
                name='train data',
                x=[
                    x
                    for x, y in self.result['train_data']
                ],
                y=[
                    y
                    for x, y in self.result['train_data']
                ],
                mode='markers',
            ),
            go.Scatter(
                name='final model',
                x=[
                    tput_constraint.compute_value(messages, m.t_gen)
                    for m in messages.all()
                ],
                y=[
                    rate_constraint.modeled_value(messages, m.t_gen)
                    for m in messages.all()
                ],
            ),
            go.Scatter(
                name='initial model',
                x=[
                    tput_constraint.compute_value(messages, m.t_gen)
                    for m in messages.all()
                ],
                y=[
                    initial_rate_constraint.modeled_value(messages, m.t_gen)
                    for m in messages.all()
                ],
            ),
        ]

    def get_graphs(self):
        return {
            'varying_background_traffic': go.Figure([
                    self.get_latency_graph(),
                    self.get_smoothed_latency_graph(),
                ] + self.get_background_traffic_graphs()
                  + self.get_constraints_graphs()
                  + self.get_model_development_graphs(),
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
            ),
            'rate_vs_throughput': go.Figure(
                self.get_rate_vs_throughput_graphs(),
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
        }
