import random
import math

random.seed(5)
# Velocity is randomly selected between 0-10 unit/dt
v_lower = 0
v_upper = 1
# Angle-Velocity is randomly selected between -0.5 - 0.5 unit/dt
w_lower = -0.5
w_upper = 0.5

def sample_point():
    return tuple([random.uniform(0.0, 10.10), random.uniform(0.0, 10.10), random.uniform(-math.pi, math.pi)])

def sample_u():
    v = random.uniform(v_lower, v_upper)
    w = random.uniform(w_lower, w_upper)
    return (v, w)

def sample_seq(n_lower, n_upper):
    return random.randint(n_lower, n_upper)