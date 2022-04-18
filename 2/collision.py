from helper import *
from robot import robot


# Similar to part 1, using SAT algorithm to check if objects collide or not.
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

# Check the path is collision free or not when transforming robot between obstacles
def isPathCollisionFree(robot_obj, start, end, obstacles):
    # Setting up start and end points for the robot in the path.
    rob_start = robot(robot_obj.width, robot_obj.height)
    rob_start.set_pose(start)
    rob_end = robot(robot_obj.width, robot_obj.height)
    rob_end.set_pose(end)

    # Getting the coordinates of start and end points of the robot.
    start_rob_geo = rob_start.transform()
    end_rob_geo = rob_end.transform()

    # Checking the path is collision free or not.
    collisionFree = True
    for i in range(len(start_rob_geo)):
        if not checkPathFree(start_rob_geo[i], end_rob_geo[i], obstacles):
            collisionFree = False
    return collisionFree

# Check the path collision free specifically by SAT algorithm.
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
