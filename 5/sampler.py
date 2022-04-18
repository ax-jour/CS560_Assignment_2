import random
import math

# random.seed(2)
random.seed(3)
# Velocity is randomly selected between 0-10 unit/dt
a_v_lower = -1
a_v_upper = 2
# Angle-Velocity is randomly selected between -0.5 - 0.5 unit/dt
steer_lower = math.pi / 3 # Left 30 degree
steer_upper = - math.pi / 3 # Right 30 degree


def sample_point():
    return tuple([random.uniform(0.0, 10.10), random.uniform(0.0, 10.10), random.uniform(-math.pi, math.pi)])


def sample_u():
    a_v = random.uniform(a_v_lower, a_v_upper)
    steer = random.uniform(steer_lower, steer_upper)
    return (a_v, steer)


def sample_seq(n_lower, n_upper):
    return random.randint(n_lower, n_upper)
