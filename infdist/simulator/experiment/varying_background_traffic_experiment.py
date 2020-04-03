
import plotly.graph_objs as go

from simulator.background_traffic import BackgroundTrafficPattern

from optimization import simplesim
from . import BaseExperiment


class VaryingBackgroundTrafficExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.background_traffic_pattern = BackgroundTrafficPattern(
        #     [
        #         i*self.t_end/10
        #         for i in range(10+1)
        #     ],
        #     [
        #         i*0.1
        #         for i in range(10)
        #     ],
        # )
        self.background_traffic_pattern = BackgroundTrafficPattern(
            [
                i*self.t_end/10
                for i in range(10+1)
            ],
            [
                0.6
                for i in range(10)
            ],
        )

        self.constraints = {
            'TPUT': simplesim.create_throughput_constraint_violations(
                throughput=1.05,
                timeslot_length=2.5,
            ),
            'RATE': simplesim.create_rate_constraint_violations(
                timeslot_length=2.5,
            ),

        }

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
        return {
            'all_received_messages': trial.all_received_messages(),
            'background_noise_history': trial.net.background_noise_history,
            'no_duplicates': trial.stats()['no_duplicates'],
        }

    def get_latency_graph(self):
        messages = self.result['all_received_messages']
        result = go.Scatter(
            name=f'latency',
            x=[m.t_rcv for m in messages.all()],
            y=[m.t_rcv - m.t_gen for m in messages.all()],
        )
        return result

    def get_smoothed_latency_graph(self):
        messages = self.result['all_received_messages']
        gen = simplesim.average_latency_generator2(1)
        result = go.Scatter(
            name=f'smooth latency',
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
                name=f'modeled rate',
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
            name=f'background traffic set',
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
            name=f'background traffic achieved',
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

    def get_graphs(self):
        return {
            'varying_background_traffic': go.Figure([
                    self.get_latency_graph(),
                    self.get_smoothed_latency_graph(),
                ] + self.get_constraints_graphs()
                  + self.get_background_traffic_graphs(),
                layout={
                    'yaxis': {
                        'title': 'latency',
                    },
                    'yaxis2': {
                        'title': 'constraint',
                        'side': 'right',
                        'overlaying': 'y',
                    },
                    'legend': {
                        'orientation': 'h',
                    },
                }
            )
        }
