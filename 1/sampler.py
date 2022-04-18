import random
# # Mount a seed in order to replicate steps
random.seed(12)

def sample():
    return tuple([random.uniform(0.0, 10.10), random.uniform(0.0, 10.10)])
