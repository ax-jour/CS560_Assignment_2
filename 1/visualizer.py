import matplotlib.pyplot as plt

from helper import *
from tree import *


# Visualize the problem in chart
def visualize_problem(robot, obstacles, start, goal):
    # Plot obstacles
    for obs in obstacles:
        obs.append(obs[0])
        xob, yob = zip(*obs)
        plt.plot(xob, yob, 'k')

    # # Mark start state as a green dot.
    plt.plot(start[0], start[1], "go")

    # # Mark goal state as a red square.
    plt.plot(goal[0], goal[1], "rs")


## Visualize the problem in chart
def visualize_points(points, robot, obstacles, start, goal):
    ## Plot obstacles
    for obs in obstacles:
        obs.append(obs[0])
        xob, yob = zip(*obs)
        plt.plot(xob, yob, 'k')

    # # Mark start state as a green dot.
    plt.plot(start[0], start[1], "go")

    # # Mark goal state as a red square.
    plt.plot(goal[0], goal[1], "rs")

    # Place robots on given coordinates.
    for co in points:
        coStateRobot = robotPointState(robot, co)
        coStateRobot.append(coStateRobot[0])
        xcs, ycs = zip(*coStateRobot)
        plt.plot(xcs, ycs)

def visualize_path(robot, obstacles, path):
    # Draw the path.
    for i in range(len(path) - 1):
        xpath, ypath = zip(*[path[i], path[i + 1]])
        plt.plot(xpath, ypath, 'r')
        plot_robot(robot, path[i])

# def visualize_configuration(robot, obstacles, start, goal):

# Visualize the rrt path in planning place.
def visualize_rrt(robot, obstacles, start, goal, iter_n, path, rrtTree):
    visualize_problem(robot, obstacles, start, goal)

    all_path = []
    # Get all points from tree and path between parent and child.
    for i in rrtTree.treeTraversal():
        onePath = []
        while i.data != start:
            onePath.append(i.data)
            i = i.parent_node
        if i != start:
            onePath.append(rrtTree.exists(start).data)
        all_path.append(onePath)

    # Plotting the path in chart.
    for p in all_path:
        if len(p) > 0:
            xp, yp = zip(*p)
            plt.plot(xp, yp, 'b')

    # Get solution's path and mark as red line segments.
    visualize_path(robot, obstacles, path)


# Visualize the rrt* path in planning place. Similar with RRT one.
def visualize_rrt_star(robot, obstacles, start, goal, iter_n, path, rrtTree):
    visualize_problem(robot, obstacles, start, goal)

    all_path = []
    for i in rrtTree.treeTraversal():
        onePath = []
        while i.data != start:
            onePath.append(i.data)
            i = i.parent_node
        if i != start:
            onePath.append(rrtTree.exists(start).data)
        all_path.append(onePath)

    for p in all_path:
        if len(p) > 0:
            xp, yp = zip(*p)
            plt.plot(xp, yp, 'b')

    visualize_path(robot, obstacles, path)


def plot_robot(robot, point):
    robot_in_place = []
    for pt in robot:
        pt = list(pt)
        pt[0] = pt[0] + point[0]
        pt[1] = pt[1] + point[1]
        robot_in_place.append(tuple(pt))
    robot_in_place.append(robot_in_place[0])
    xss, yss = zip(*robot_in_place)
    plt.plot(xss, yss, 'y')


# # Visualize the problem in chart
# def visualize_problem(robot, obstacles, start, goal):
#     # Plot obstacles
#     for obs in obstacles:
#         obs.append(obs[0])
#         xob, yob = zip(*obs)
#         plt.plot(xob, yob, 'k')
#
#     # Place the robot at start state.
#     # Func "robotPointState" return the robot's coordinates at particular point.
#     startRobotState = robotPointState(robot, start)
#     startRobotState.append(startRobotState[0])
#     xss, yss = zip(*startRobotState)
#     plt.plot(xss, yss)
#
#     # Place the robot at goal state.
#     goalRobotState = robotPointState(robot, goal)
#     goalRobotState.append(goalRobotState[0])
#     xgs, ygs = zip(*goalRobotState)
#     plt.plot(xgs, ygs)
