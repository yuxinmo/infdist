from . import BaseExperiment

from optimization.video_mission import generate_example_video_mission


class PlaygroundExperiment(BaseExperiment):
    def prepare_trial(self, *args, **kwargs):
        trial = super().prepare_trial()
        trial.prepare_network()
        trial.net.packet_size = 20000
        trial.messages, trial.ctx = generate_example_video_mission(
            set(range(trial.nodes_num)),
            t_end=trial.t_end,
        )
        print(trial.ctx)
        return trial
