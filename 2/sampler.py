import math
import random
random.seed(2)

def sample():
    return tuple([random.uniform(0.0, 10.10), random.uniform(0.0, 10.10), random.uniform(-math.pi, math.pi)])