from collections import namedtuple
import plotly.graph_objs as go

from . import BaseExperiment

from optimization import simplesim

WindowConfiguration = namedtuple(
    'WindowConfiguration',
    ' '.join([
        'window_duration',
        'message_size',
    ])
)


class BytesInWindowGraph(BaseExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.configurations = list(map(lambda t: WindowConfiguration(*t), [
            (0.5, 2048),
            (1, 2048),
            (1, 60048),
        ]))

    def run(self, *args, **kwargs):
        self.result = self.prepare_trial()
        self.result.prepare_messages()
        simplesim.latency(self.result.messages, 0)

    def print_result(self):
        print("Total msgs: {}".format(len(self.result.messages)))
        mps = len(self.result.messages)/self.result.messages.t_end
        print("Avg msgs/sec: {} {}".format(
            mps,
            {
                msg_size: msg_size*mps*8/10**6
                for msg_size in set(
                    [conf.message_size for conf in self.configurations]
                )
            }
        ))

    def _bytes_in_window(self, msgs, i, conf):
        j = i
        while j > 0 and msgs[i].t_gen - msgs[j].t_gen <= conf.window_duration:
            j -= 1
        return (i-j)*conf.message_size

    def get_graphs(self):
        r = []
        for conf in self.configurations:
            xs = [m.t_gen for m in self.result.messages.all()]
            ys = [
                8*self._bytes_in_window(
                    self.result.messages.all(),
                    i,
                    conf
                )/conf.window_duration
                for i in range(len(self.result.messages.all()))
            ]
            r.append(go.Scatter(
                name='window={}, msg_size={}'.format(
                    *conf
                ),
                x=xs,
                y=ys,
                line={
                    'width': 1,
                },
                mode='lines',
            ))

        return {
            'bits_in_window': r
        }

    def get_graph_xaxis_title(self, graph_name):
        return "time [s]"

    def get_graph_yaxis_title(self, graph_name):
        return "bps"
