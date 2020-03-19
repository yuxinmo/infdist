from . import BaseExperiment


class PlaygroundExperiment(BaseExperiment):
    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.set_drop_rate(0.8)
        return trial

    def save_results(self, f):
        print("Saving PlaygroundExperiment results not supported.")
