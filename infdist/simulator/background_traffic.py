class BackgroundTrafficPattern:
    def __init__(self, ts, throughputs):
        assert(len(throughputs) == len(ts)-1)
        self.ts = ts
        self.throughputs = throughputs
