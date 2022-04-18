import matplotlib.pyplot as plt

from robot import *
from helper import *

## Visualize the problem in chart
def visualize_problem(robot_obj, obstacles, start, goal):
    ## Plot obstacles
    for obs in obstacles:
        obs.append(obs[0])
        xob, yob = zip(*obs)
        plt.plot(xob, yob, 'k')

    ## Robot at start state
    robot_obj.set_pose(start)
    rob_ls = robot_obj.transform()
    rob_ls.append(rob_ls[0])
    xss, yss = zip(*rob_ls)
    plt.plot(xss, yss, 'r')

    ## Robot at goal state
    robot_obj.set_pose(goal)
    rob_ls = robot_obj.transform()
    rob_ls.append(rob_ls[0])
    xgs, ygs = zip(*rob_ls)
    plt.plot(xgs, ygs, 'g')


## Visualize the problem in chart
def visualize_points(points, robot_obj, obstacles, start, goal):
    ## Plot obstacles
    for obs in obstacles:
        obs.append(obs[0])
        xob, yob = zip(*obs)
        plt.plot(xob, yob, 'k')

    ## Robot at point state
    for point in points:
        robot_obj.set_pose(point)
        rob_ls = robot_obj.transform()
        rob_ls.append(rob_ls[0])
        xss, yss = zip(*rob_ls)
        plt.plot(xss, yss, 'm')

    ## Robot at start state
    robot_obj.set_pose(start)
    rob_ls = robot_obj.transform()
    rob_ls.append(rob_ls[0])
    xss, yss = zip(*rob_ls)
    plt.plot(xss, yss, 'r')

    ## Robot at goal state
    robot_obj.set_pose(goal)
    rob_ls = robot_obj.transform()
    rob_ls.append(rob_ls[0])
    xgs, ygs = zip(*rob_ls)
    plt.plot(xgs, ygs, 'g')


def visualize_path(robot_obj, obstacles, path):
    for i in range(len(path) - 1):
        xss, yss = zip(*[getCoord(path[i]), getCoord(path[i+1])])
        plt.plot(xss, yss, 'r')
        plot_robot(robot_obj, path[i+1])

# def visualize_configuration(robot, obstacles, start, goal):


# Visualize the rrt path in planning place.
def visualize_rrt(robot, obstacles, start, goal, iter_n, path, rrtTree):
    visualize_problem(robot, obstacles, start, goal)

    all_path = []
    # Get all points from tree and path between parent and child.
    for i in rrtTree.treeTraversal():
        onePath = []
        while i.data != start:
            onePath.append(getCoord(i.data))
            i = i.parent_node
        if i != start:
            onePath.append(getCoord(rrtTree.exists(start).data))
        all_path.append(onePath)

    # Plotting the path in chart.
    for p in all_path:
        if len(p) > 0:
            xp, yp = zip(*p)
            plt.plot(xp, yp, 'b')

    # Get solution's path and mark as red line segments.
    visualize_path(robot, obstacles, path)


# Visualize the rrt* path in planning place.
def visualize_rrt_star(robot, obstacles, start, goal, iter_n, path, rrtTree):
    visualize_problem(robot, obstacles, start, goal)

    all_path = []
    # Get all points from tree and path between parent and child.
    for i in rrtTree.treeTraversal():
        onePath = []
        while i.data != start:
            onePath.append(getCoord(i.data))
            i = i.parent_node
        if i != start:
            onePath.append(getCoord(rrtTree.exists(start).data))
        all_path.append(onePath)

    # Plotting the path in chart.
    for p in all_path:
        if len(p) > 0:
            xp, yp = zip(*p)
            plt.plot(xp, yp, 'b')

    # Get solution's path and mark as red line segments.
    visualize_path(robot, obstacles, path)


def plot_robot(robot, pose):
    robot.set_pose(pose)
    rob_ls = robot.transform()
    rob_ls.append(rob_ls[0])
    xss, yss = zip(*rob_ls)
    plt.plot(xss, yss, 'y')