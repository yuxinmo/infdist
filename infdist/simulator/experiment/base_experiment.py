import os
import pickle
import plotly.graph_objs as go
import plotly.io as pio
from datetime import datetime

from .trial import TreeTrial


class BaseExperiment:
    DEBUG_NONE = 0
    DEBUG_PROGRESS = 1
    DEBUG_ALL = 2

    def __init__(self, agents_num, t_end, msgset):
        self.agents_num = agents_num
        self.t_end = t_end
        self.msgset = msgset
        self.trial_cls = TreeTrial
        self._example_trial = None

    def prepare_trial(self, trial_cls=None, msgset=None):
        if trial_cls is None:
            trial_cls = self.trial_cls

        if msgset is None:
            msgset = self.msgset

        t = trial_cls(self.agents_num, self.t_end, msgset)
        return t

    def progress(self, percent, debug, start_time=None):
        if debug >= self.DEBUG_PROGRESS:
            p = percent/100
            if start_time and percent != 0:
                duration = (datetime.now()-start_time)
                total = duration/p
                to_go = (
                    total-duration
                )
                eta = datetime.now() + to_go
            else:
                eta = '?'
                to_go = '?'
            print('\t\t\t\t{:.2f}% ETA: {} (in {})'.format(
                percent, eta, to_go,
            ), end='\r')

    def run(self, debug=DEBUG_NONE):
        t = self.prepare_trial()
        t.run()
        self.result = self.get_result(t)
        self.resulting_trial = t

    def get_result(self, trial):
        return trial

    def print_result(self):
        self.resulting_trial.print_stats(self.resulting_trial.stats())

    def get_graphs(self):
        return {}

    def get_publication_graphs(self):
        return self.get_graphs()

    def get_graph_title(self, graph_name):
        return graph_name

    def get_graph_xaxis_title(self, graph_name):
        return "todo"

    def get_graph_yaxis_title(self, graph_name):
        return "todo"

    def _set_figure_layout(self, graph_name, figure):
        figure.update_layout(
            title=self.get_graph_title(graph_name),
            xaxis_title=self.get_graph_xaxis_title(graph_name),
            yaxis_title=self.get_graph_yaxis_title(graph_name),
        )

    def save_results(self, filename):
        pickle.dump(
            self.result,
            open(filename, 'wb')
        )

    def save_graphs(self, f=None, graph_names=None, graphs=None):
        if graphs is None:
            graphs = self.get_graphs()

        if graph_names is None:
            graph_names = graphs

        for graph_name in graph_names:
            graph = graphs[graph_name]
            if type(graph) is list:
                fig = go.Figure({'data': graph, 'title': "test"})
                self._set_figure_layout(graph_name, fig)
            elif type(graph) is go.Figure:
                fig = graph
            else:
                raise Exception("Unknown graph type")
            if f is not None:
                file_name = f.format(graph_name)
                os.makedirs(os.path.dirname(file_name), exist_ok=True)
                pio.write_image(fig, file_name)
            else:
                pio.write_image(fig, '/tmp/{}.pdf'.format(graph_name))

    def save_publication_graphs(self, publication_dir):
        graphs = self.get_publication_graphs()
        for graph_name, graph in graphs.items():
            if type(graph) is list:
                fig = go.Figure({'data': graph, 'title': "test"})
                self._set_figure_layout(graph_name, fig)
            elif type(graph) is go.Figure:
                fig = graph
            else:
                raise Exception("Unknown graph type")

            fig.update_layout(
                width=400,
                height=300,
                margin=go.layout.Margin(
                    l=0,  # NOQA
                    r=0,
                    b=0,
                    t=0,
                    pad=0
                ),
                template='plotly_white',
            )
            pio.write_image(
                fig,
                os.path.join(publication_dir, graph_name + '.pdf')
            )
