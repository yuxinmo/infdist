from . import DropRateVsUtilityExperiment, BaseExperiment
from .trial import FixedRatioTrial, TreeTrial


class LimitedThroughputExperiment(DropRateVsUtilityExperiment):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.trial_clss = [FixedRatioTrial]
        self.NETWORK_DATA_RATE = 1
        self.TREE_THROUGHPUT_LIMIT = 0.6

    def run(self, debug=False):
        if debug == self.DEBUG_ALL:
            print("Running drop rate experiments")
        super().run(debug)

        if debug == self.DEBUG_ALL:
            print("Running tree-based experiments")
        tree_trial = BaseExperiment.prepare_trial(
            self, TreeTrial
        )
        self._setup_network(tree_trial)
        tree_trial.set_throughput(self.TREE_THROUGHPUT_LIMIT)
        tree_trial.run()
        self.tree_result = tree_trial

    def print_result(self):
        self.tree_result.print_stats()

    def _setup_network(self, trial):
        trial.network_data_rate = self.NETWORK_DATA_RATE
        trial.prepare_network()
        # trial.net.use_big_packets()
        trial.net.packet_size = 20000
        trial.net.add_background_traffic(0, trial.t_end, 0.025)

    def prepare_trial(self, *args, **kwargs):
        t = super().prepare_trial(*args, **kwargs)
        self._setup_network(t)
        return t

    def get_graph_name(self):
        return 'limited_throughput'
