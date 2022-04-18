import random
import math

# random.seed(2)
random.seed(1)
# Velocity is randomly selected between 0-10 unit/dt
a_v_lower = -1
a_v_upper = 2
# Angle-Velocity is randomly selected between -0.5 - 0.5 unit/dt
a_r_lower = -1.5
a_r_upper = 1.5


def sample_point():
    return tuple([random.uniform(0.0, 10.10), random.uniform(0.0, 10.10), random.uniform(-math.pi, math.pi)])


def sample_u():
    a_v = random.uniform(a_v_lower, a_v_upper)
    a_r = random.uniform(a_r_lower, a_r_upper)
    return (a_v, a_r)


def sample_seq(n_lower, n_upper):
    return random.randint(n_lower, n_upper)
