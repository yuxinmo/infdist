from statistics import variance, mean, StatisticsError
import plotly.graph_objs as go
import plotly.express as px

from . import DropRateVsUtilityExperiment, BaseExperiment
from .trial import FixedRatioTrial, GreedyTrial, TreeTrial  # NOQA
from simulator.background_traffic import BackgroundTrafficPattern

from optimization import simplesim

GRAPH_NAME_MAP = {
    'infdist10': r'$\alpha = 0.1$',
    'infdist15': r'$\alpha = 0.15$',
    'infdist20': r'$\alpha = 0.2$',
    'infdist25': r'$\alpha = 0.25$',
    'infdist30': r'$\alpha = 0.3$',
    'FixedRatioTrial': 'simulated network',
}
LATENCY_LABEL_POSITION_MAP = {
    'infdist10': 'bottom left',
    'infdist15': 'bottom center',
    'infdist20': 'bottom right',
    'infdist25': 'middle right',
    'infdist30': 'middle right',
    'FixedRatioTrial': 'bottom right',
}
DROP_LABEL_POSITION_MAP = {
    'infdist10': 'bottom right',
    'infdist15': 'bottom right',
    'infdist20': 'bottom right',
    'infdist25': 'middle right',
    'infdist30': 'bottom right',
    'FixedRatioTrial': 'bottom right',
}

colormap = px.colors.qualitative.Plotly
COLOR_MAP = {
    'infdist10': colormap[1],
    'infdist15': colormap[2],
    'infdist20': colormap[3],
    'infdist25': colormap[5],
    'infdist30': colormap[7],
    'FixedRatioTrial': colormap[6],
}


class LimitedThroughputExperiment(DropRateVsUtilityExperiment):
    GRAPH_WIDTH = 1000
    GRAPH_HEIGHT = 100
    NORMALIZE_UTILITY = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.trial_clss = [FixedRatioTrial]
        self.NETWORK_DATA_RATE = 1
        self.methods_to_compare = {
            'infdist10': (TreeTrial, 0.1),
            'infdist15': (TreeTrial, 0.15),
            'infdist20': (TreeTrial, 0.2),
            'infdist25': (TreeTrial, 0.25),
            # 'infdist30': (TreeTrial, 0.3),
        }

    def create_constraints(self, alpha):
        return {
            # 'TPUT': simplesim.create_throughput_constraint_violations(
            #     throughput=1.05,
            #     timeslot_length=2.5,
            # ),
            'RATE': simplesim.create_rate_constraint_violations(
                timeslot_length=2.5, alpha=alpha
            ),
        }

    def run(self, debug=False):
        if debug == self.DEBUG_ALL:
            print("Running drop rate experiments")
        super().run(debug)

        for method_name, (trial_cls, alpha) in self.methods_to_compare.items():
            if debug == self.DEBUG_ALL:
                print(f"Running '{method_name}' experiments")
            trial = BaseExperiment.prepare_trial(
                self, trial_cls
            )

            trial.constraints = self.create_constraints(alpha)
            self._setup_network(trial)
            trial.run()
            self.result[method_name] = {
                'default': [trial.stats()],
            }

    def print_result(self):
        for method_name, (trial_cls, alpha) in self.methods_to_compare.items():
            print(f' ===== {method_name} =====')
            trial_cls.print_stats(self.result[method_name]['default'][0])

    def _setup_network(self, trial):
        trial.network_data_rate = self.NETWORK_DATA_RATE
        trial.prepare_network()
        # trial.net.use_big_packets()

        background_traffic_pattern = BackgroundTrafficPattern(
            [
                i*self.t_end/10
                for i in range(10+1)
            ],
            [
                0.5
                for i in range(10)
            ],
        )
        trial.net.add_background_traffic_pattern(
            background_traffic_pattern,
            'udp',
        )

    def prepare_trial(self, *args, **kwargs):
        t = super().prepare_trial(*args, **kwargs)
        self._setup_network(t)
        return t

    def save_results(self, filename):
        return super().save_results(filename)

    def _get_single_result_graph2(
        self, results, name, xs_func, ys_func,
        scatter_kwargs={}, ys_multiplier=100,
        error_y_func=None,
    ):
        xs = [
            xs_func(stat)
            for stat in sum(results.values(), [])
        ]
        ys = [
            ys_func(stat) * ys_multiplier
            for stat in sum(results.values(), [])
        ]
        if error_y_func is not None:
            error_y = [
                error_y_func(stat) * ys_multiplier
                for stat in sum(results.values(), [])
            ]

        return go.Scatter(
            name=name,
            x=xs,
            y=ys,
            mode=(
                'lines' if name == GRAPH_NAME_MAP['FixedRatioTrial']
                else 'markers+text'
            ),
            error_y=dict(
                type='data',  # value of error bar given in data coordinates
                array=error_y,
                visible=True
            ) if error_y_func is not None else dict(),
            **scatter_kwargs,
        )

    def get_avg_latency_graph(self, trial_name):
        def stat_latency_variance(stat):
            try:
                return variance([
                    m.t_rcv - m.t_gen
                    for m in stat['all_received_messages'].all()
                ])
            except StatisticsError:
                return 0

        def avg_latency_in_the_second_half(stat):
            try:
                return mean([
                    m.t_rcv - m.t_gen
                    for m in stat['all_received_messages'].all()
                    if m.t_gen > 50
                ])
            except StatisticsError:
                return 0

        MSG_SIZE = 2048
        return self._get_single_result_graph2(
            self.result[trial_name],
            f'{trial_name} - latency',
            # lambda stat: stat['sent_num']/stat['total_messages']*100,
            lambda stat: stat['sent_num']*MSG_SIZE*8/10**6/100,
            avg_latency_in_the_second_half,
            scatter_kwargs={
                'showlegend': False,
                'line': {
                    'color': colormap[4],
                },
            } if trial_name == 'FixedRatioTrial' else {
                'text': GRAPH_NAME_MAP[trial_name],
                'textposition': LATENCY_LABEL_POSITION_MAP[trial_name],
                'marker_color': COLOR_MAP[trial_name],
                'showlegend': False,
                'marker_symbol': 'x',
                'marker_size': 15,
            },
            ys_multiplier=1,
            error_y_func=stat_latency_variance,
        )

    def get_loss_graph(self, trial_name):
        return self._get_single_result_graph2(
            self.result[trial_name],
            f'{trial_name} - loss',
            lambda stat: stat['sent_num']/stat['total_messages']*100,
            lambda stat: 0 if stat['sent_num'] == 0 else (
                1 - stat['received_num']/(
                    stat['sent_num']*(stat['agents_num']-1)
                )
            ),
        )

    def get_drop_rate_graph(self, trial_name):
        MSG_SIZE = 2048
        return self._get_single_result_graph2(
            self.result[trial_name],
            GRAPH_NAME_MAP.get(trial_name, f'{trial_name} - drop_rate'),
            lambda stat: stat['sent_num']*MSG_SIZE*8/10**6/100,
            lambda stat: stat['received_num']*MSG_SIZE*8/(
                stat['agents_num']-1
            )/10**6/100,
            # lambda stat: stat['received_num']*MSG_SIZE*8/(
            #     stat['agents_num']-1
            # )/10**6/100,
            # lambda stat: stat['sent_num']/stat['total_messages']*100,
            # lambda stat: stat['received_num']/(
            #     stat['total_messages']*(stat['agents_num']-1)
            # ),
            scatter_kwargs={
                'fill': 'tozeroy',
                'line': {
                    'color': 'black',
                    'width': 1,
                },
            } if trial_name == 'FixedRatioTrial' else {
                'text': GRAPH_NAME_MAP[trial_name],
                'textposition': DROP_LABEL_POSITION_MAP[trial_name],
                'marker_color': COLOR_MAP[trial_name],
                'textfont': {
                    # 'family': "serif",
                    # 'size': 18,
                    'color': "White",
                },
                'marker_symbol': 'x',
                'marker_size': 15,
                'showlegend': False
            },
            ys_multiplier=1,
        )

    def get_linear_graph(self):
        return go.Scatter(
            name="unconstrained communication",
            x=[0, 1],
            y=[0, 1],
            mode='lines',
            line={
                'color': 'gray',
                'width': 1,
                'dash': 'dot',
            },
            fill='tonexty',
        )

    def get_limited_througphut_graph(self):
        fig = go.Figure(
            layout={
                'xaxis': {
                    'title': 'publishing rate [Mbps]',
                    'range': [0, 0.7],
                },
                'yaxis': {
                    'title': 'reception rate [Mbps]',
                    'range': [0, 0.7],
                },
                'yaxis2': {
                    'title': 'latency [s]',
                    'side': 'right',
                    'overlaying': 'y',
                },
                'legend': {
                    'orientation': 'h',
                    'x': -0.05,
                    'y': 1.1,
                },
            }
        )
        fig.add_trace(self.get_linear_graph())
        for i, trial_name in enumerate(
            self.get_trial_names() + list(self.methods_to_compare.keys())
        ):
            gs = [
                self.get_drop_rate_graph(trial_name),
                # self.get_avg_latency_graph(trial_name),
                # self.get_loss_graph(trial_name),
            ]
            for g in gs:
                fig.add_trace(g)

        return fig

    def get_limited_throughput_latency_graph(self):
        fig = go.Figure(
            layout={
                'xaxis': {
                    'title': 'publishing rate [Mbps]',
                    'range': [0, 0.7],
                },
                'yaxis': {
                    'title': 'latency [s]',
                },
                'legend': {
                    'orientation': 'h',
                },
            }
        )
        for i, trial_name in enumerate(
            self.get_trial_names() + list(self.methods_to_compare.keys())
        ):
            gs = [
                self.get_avg_latency_graph(trial_name),
            ]
            for g in gs:
                fig.add_trace(g)

        return fig

    def get_graphs(self):
        return {
            'limited_throughput': self.get_limited_througphut_graph(),
        }

    def get_publication_graphs(self):
        return {
            'drop_rate_comparison': self.get_limited_througphut_graph(),
            'latency_comparison': self.get_limited_throughput_latency_graph(),
        }

    def get_graph_name(self):
        return 'limited_throughput'
