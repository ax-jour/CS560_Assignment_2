from helper import *

# Call this function for collision detection.
def isCollisionFree(robot, point, obstacles):

    # Using "Ear clipping" strategy divides convex/non-convex polygons to triangles.
    newObs = triangulatePolys(obstacles)

    # Placing robot at the point which need to be tested.
    robot_in_place = []
    for pt in robot:
        pt = list(pt)
        pt[0] = pt[0] + point[0]
        pt[1] = pt[1] + point[1]
        robot_in_place.append(tuple(pt))

    # Check if the robot collision with boundary
    for rob_vert in robot_in_place:
        if rob_vert[0] > 10 or rob_vert[0] < 0:
            return False
        if rob_vert[1] > 10 or rob_vert[1] < 0:
            return False

    # Check if the robot collision with any triangulated obstacles by SAT strategy.
    for obs in newObs:
        if isSAToverlap(robot_in_place, obs)[0] == True:
            return False

    return True

# Call this function when extending robot.
def isPathCollisionFree(robot, start, end, obstacles):
    collisionFree = True
    for rob_pt in robot:
        pt_start = (rob_pt[0]+start[0], rob_pt[1]+start[1])
        pt_end = (rob_pt[0]+end[0], rob_pt[1]+end[1])
        if not isCollisionFree([pt_start, pt_end], (0, 0), obstacles):
            collisionFree = False
    return collisionFree
