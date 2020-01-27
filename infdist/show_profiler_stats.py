import pstats
p = pstats.Stats('/tmp/profiler_stats')
p.strip_dirs().sort_stats("tottime").print_stats()
