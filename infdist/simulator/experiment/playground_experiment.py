from . import BaseExperiment


class PlaygroundExperiment(BaseExperiment):
    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.prepare_network()
        trial.net.packet_size = 10000
        trial.set_throughput(1)
        return trial
