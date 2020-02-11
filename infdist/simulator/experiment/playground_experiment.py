from . import BaseExperiment


class PlaygroundExperiment(BaseExperiment):
    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.prepare_network()
        trial.net.add_background_traffic()
        trial.net.packet_size = 20000
        trial.set_throughput(1)
        return trial
