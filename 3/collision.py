from helper import *
from matplotlib import pyplot as plt


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

def isKinePathCollisionFree(kine, sequence, dt, rob_start, rob_end, old_state, obstacles):
    curr_state = old_state
    for i in range(sequence):

        # New X coordinate
        new_state_x = curr_state[0] + dt * kine[0]
        # New Y coordinate
        new_state_y = curr_state[1] + dt * kine[1]
        # New Angle degree
        new_state_r = curr_state[2] + dt * kine[2]

        new_state = (new_state_x, new_state_y, new_state_r)

        rob_start.set_pose(curr_state)
        rob_end.set_pose(new_state)

        start_rob_geo = rob_start.transform()
        end_rob_geo = rob_end.transform()

        if not isPathCollisionFree(rob_start, start_rob_geo, end_rob_geo, obstacles):
            return curr_state, i

        curr_state = new_state

    return curr_state, sequence