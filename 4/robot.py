import numpy as np
import math

from collision import *
from visualizer import *

class robot:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.translation = (0, 0)
        self.rotation = 0.0
        self.control = (0, 0)

        self.obstacles = None

    def set_pose(self, pose): # Input (x, y, r)
        self.translation = (pose[0], pose[1])
        self.rotation = pose[2]

    def transform(self):
        w = self.width
        l = self.height
        x = self.translation[0]
        y = self.translation[1]
        r = self.rotation

        std_rotation = [[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]]

        A = np.dot(std_rotation, [[-w/2, -w/2, w/2, w/2], [-l/2, l/2, l/2, -l/2]])
        B = A + np.matrix([[x, x, x, x], [y, y, y, y]])

        rob_ls = []
        geo_rob = list(B.T)
        for r in geo_rob:
            rob_ls.extend(r.tolist())

        return rob_ls

    def kinematic(self, state, control):
        x_t = control[0] * math.cos(state[2] + math.pi/2)
        y_t = control[0] * math.sin(state[2] + math.pi/2)
        r_t = control[1]
        return (x_t, y_t, r_t)

    # state: (x_coord, y_coord, angle, x_velocity, y_velocity, r_velocity)
    def dynamics(self, state, control):
        r = state[2]
        x_v = state[3]
        y_v = state[4]
        r_v = state[5]
        x_a = control[0] * math.cos(r + math.pi/2)
        y_a = control[0] * math.sin(r + math.pi/2)
        r_a = control[1]
        return (x_v, y_v, r_v, x_a, y_a, r_a)


    # state: (x_coord, y_coord, angle, x_velocity, y_velocity, r_velocity)
    # control: (trans_v_acceleration, angle_v_acceleration)
    # duration: number of dt
    def propagate(self, state, controls, durations, dt, obstacles):
        # List match check.
        if len(controls) != len(durations):
            print("Controls number is not match with durations number.")
            return None
        curr_state = state

        rtn_controls = []
        rtn_durations = []
        # For each control it has n durations to operate.
        for i in range(len(controls)):
            # Calculate q dot dot for a control
            dyna = self.dynamics(curr_state, controls[i])

            rob_start = robot(self.width, self.height)
            rob_end = robot(self.width, self.height)

            new_state, seq = isDynaPathCollisionFree(dyna, durations[i], dt, rob_start, rob_end, curr_state, obstacles)
            # Check the collision from the current state to target state by discretized check.
            if new_state == curr_state:
                break

            # Note feasible control set and duration set.
            rtn_controls.append(controls[i])
            rtn_durations.append(seq)
            curr_state = new_state

        # Return new state and feasible controls and duration set.
        return curr_state, (rtn_controls, rtn_durations)