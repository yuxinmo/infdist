from .trial import Trial


class BaseExperiment:
    def __init__(self, agents_num, t_end):
        self.agents_num = agents_num
        self.t_end = t_end
        self.trial_cls = Trial

    def prepare_trial(self):
        t = self.trial_cls(self.agents_num, self.t_end)
        return t

    def run(self, debug=False):
        t = self.prepare_trial()
        t.run()
        self.result = t

    def print_result(self):
        self.result.print_stats()
