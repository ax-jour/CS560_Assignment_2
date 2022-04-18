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
        # Distance at x axis
        S_x = dyna[0] * dt + (1/2) * dyna[3] * dt**2
        # Distance at y axis
        S_y = dyna[1] * dt + (1/2) * dyna[4] * dt**2
        # Angle distance
        S_r = dyna[2] * dt + (1/2) * dyna[5] * dt**2
        new_state_x = curr_state[0] + S_x
        new_state_y = curr_state[1] + S_y
        new_state_r = curr_state[2] + S_r

        # Acceleration at x axis
        new_x_v = dyna[0] + dyna[3] * dt
        # Acceleration at y axis
        new_y_v = dyna[1] + dyna[4] * dt
        # Acceleration at angle
        new_r_v = dyna[2] + dyna[5] * dt

        new_state = (new_state_x, new_state_y, new_state_r, new_x_v, new_y_v, new_r_v)

        rob_start.set_pose(curr_state)
        rob_end.set_pose(new_state)

        start_rob_geo = rob_start.transform()
        end_rob_geo = rob_end.transform()

        if not isPathCollisionFree(rob_start, start_rob_geo, end_rob_geo, obstacles):
            return curr_state, i

        # # Plot all collision free path that it tried.
        # start_pt = (curr_state[0], curr_state[1])
        # end_pt = (new_state[0], new_state[1])
        # xp, yp = zip(*[start_pt, end_pt])
        # plt.plot(xp, yp, 'c')

        curr_state = new_state

    return curr_state, sequence

def isKinePathCollisionFree(kine, sequence, dt, rob_start, rob_end, old_state, obstacles, divided_n):
    curr_state = old_state
    for i in range(sequence):

        new_state_x = curr_state[0] + dt * kine[0]
        new_state_y = curr_state[1] + dt * kine[1]
        new_state_r = curr_state[2] + dt * kine[2]

        new_state = (new_state_x, new_state_y, new_state_r)

        rob_start.set_pose(curr_state)
        rob_end.set_pose(new_state)

        start_rob_geo = rob_start.transform()
        end_rob_geo = rob_end.transform()

        if not isPathCollisionFree(rob_start, start_rob_geo, end_rob_geo, obstacles):
            return False

        curr_state = new_state

    return True

def isKinePathCollisionFree_v2(kine, dt, rob_start, rob_end, curr_state, obstacles, divided_n):

    new_state_x = curr_state[0] + dt * kine[0]
    new_state_y = curr_state[1] + dt * kine[1]
    new_state_r = curr_state[2] + dt * kine[2]

    new_state = (new_state_x, new_state_y, new_state_r)

    rob_start.set_pose(curr_state)
    rob_end.set_pose(new_state)

    start_rob_geo = rob_start.transform()
    end_rob_geo = rob_end.transform()

    if not isPathCollisionFree(rob_start, start_rob_geo, end_rob_geo, obstacles):
        return False

    return True