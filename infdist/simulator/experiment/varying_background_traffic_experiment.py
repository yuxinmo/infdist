
import plotly.graph_objs as go

from simulator.background_traffic import BackgroundTrafficPattern

from . import BaseExperiment


class VaryingBackgroundTrafficExperiment(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.background_traffic_pattern = BackgroundTrafficPattern(
            [0, 10, 20, self.t_end],
            [0, 1, 3],
        )

    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.set_throughput(1)
        trial.prepare_network()
        trial.network_data_rate = 1
        trial.net.add_background_traffic_pattern(
            self.background_traffic_pattern
        )
        return trial

    def save_results(self, f):
        print("Saving results not implemented yet.")

    def get_latency_graph(self):
        messages = self.result.all_received_messages()
        result = go.Scatter(
            name=f'latency',
            x=[m.t_rcv for m in messages.all()],
            y=[m.t_rcv - m.t_gen for m in messages.all()],
        )
        return result

    def get_constraints_graphs(self):
        messages = self.result.all_received_messages()
        return [
            go.Scatter(
                name=f'{constraint_name}',
                x=[m.t_rcv for m in messages.all()],
                y=[constraint.compute_value(messages, m.t_gen)/10**6
                   for m in messages.all()],
                yaxis='y2',
                line={
                    'dash': 'dash',
                },
            )
            for constraint_name, constraint in self.result.constraints.items()
        ]

    def get_background_traffic_graph(self):
        result = go.Scatter(
            name=f'background traffic',
            x=self.background_traffic_pattern.ts,
            y=self.background_traffic_pattern.throughputs + [
                self.background_traffic_pattern.throughputs[-1]
            ],
            line_shape='hv',
            yaxis='y2',
        )
        return result

    def get_graphs(self):
        return {
            'varying_background_traffic': go.Figure([
                    self.get_latency_graph(),
                    self.get_background_traffic_graph(),
                ] + self.get_constraints_graphs(),
                layout={
                    'yaxis': {
                        'title': 'latency',
                    },
                    'yaxis2': {
                        'title': 'constraint',
                        'side': 'right',
                        'overlaying': 'y',
                    },
                }

            )

        }
