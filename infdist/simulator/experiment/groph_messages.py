from . import BaseExperiment

from optimization import simplesim
from visualization.messageset_plotly import write_messageset_plots


class GraphMessagesExperiment(BaseExperiment):
    def run(self, *args, **kwargs):
        self.result = self.prepare_trial()
        self.result.prepare_messages()

    def print_result(self):
        pass

    def save_graphs(self, f=None):
        msgs = self.result.messages
        mission_context = self.result.ctx
        simplesim.latency(msgs, 0)
        write_messageset_plots(
            msgs,
            mission_context,
            f'/tmp/plots/{self.agents_num}_{self.t_end}_{self.msgset}-',
        )
