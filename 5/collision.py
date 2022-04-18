from helper import *
from matplotlib import pyplot as plt
from visualizer import *

def isCollisionFree(robot_obj, point, obstacles):
    newObs = triangulatePolys(obstacles)
    robot_obj.set_pose(point)
    newRob = robot_obj.transform()

    # Check if the robot collision with boundary
    for rob_vert in newRob:
        if rob_vert[0] > 10 or rob_vert[0] < 0:
            return False
        if rob_vert[1] > 10 or rob_vert[1] < 0:
            return False

    for obs in newObs:
        if isSAToverlap(newRob, obs)[0] == True:
            return False

    return True


def checkPathFree(start_point, end_point, obstacles):
    newObs = triangulatePolys(obstacles)
    line = (start_point, end_point)

    for pt in line:
        if pt[0] > 10 or pt[0] < 0:
            return False
        if pt[1] > 10 or pt[1] < 0:
            return False

    for obs in newObs:
        if isSAToverlap(line, obs)[0] == True:
            return False

    return True


def isPathCollisionFree(robot_obj, start_rob_geo, end_rob_geo, obstacles):
    collisionFree = True
    for i in range(len(start_rob_geo)):
        if not checkPathFree(start_rob_geo[i], end_rob_geo[i], obstacles):
            collisionFree = False
    return collisionFree

def isDynaPathCollisionFree(dyna, sequence, dt, rob_start, rob_end, old_state, obstacles):
    curr_state = old_state
    for i in range(sequence):
        # Acceleration in x axis
        x_a = dyna[3] * math.cos(curr_state[4]) * math.cos(curr_state[2])
        # Acceleration in y axis
        y_a = dyna[3] * math.cos(curr_state[4]) * math.sin(curr_state[2])

        # Distance on x axis: Xv_0 * dt + (1/2) * Xa * dt^2
        S_x = dyna[0] * dt + (1/2) * x_a * dt**2
        # Distance on y axis: Yv_0 * dt + (1/2) * Ya * dt^2
        S_y = dyna[1] * dt + (1/2) * y_a * dt**2

        new_state_x = curr_state[0] + S_x
        new_state_y = curr_state[1] + S_y
        # Angle of car: car angle + car angle velocity * dt
        new_state_r = curr_state[2] + dyna[2] * dt
        # Velocity = acceleration * dt
        new_v = dyna[3] * dt
        # Angle of wheel: wheel angle + wheel angle velocity * dt
        new_eta = curr_state[4] + dyna[4] * dt

        new_state = (new_state_x, new_state_y, new_state_r, new_v, new_eta)

        rob_start.set_pose(curr_state)
        rob_end.set_pose(new_state)

        start_rob_geo = rob_start.transform()
        end_rob_geo = rob_end.transform()

        if not isPathCollisionFree(rob_start, start_rob_geo, end_rob_geo, obstacles):
            return curr_state, i

        curr_state = new_state

    return curr_state, sequence