from . import BaseExperiment

from optimization.video_mission import generate_example_video_mission


class PlaygroundExperiment(BaseExperiment):
    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.set_drop_rate(0.2)
        return trial

    def save_results(self, f):
        print("Saving PlaygroundExperiment results not supported.")
