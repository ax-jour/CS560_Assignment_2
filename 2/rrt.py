import time
from matplotlib import pyplot as plt

from helper import *
from sampler import sample
from tree import Tree
from collision import isCollisionFree, isPathCollisionFree
from visualizer import *


# Similar with Part 1's rrt algorithm
def rrt(robot_obj, obstacles, start, goal, iter_n, step_size):
    total_n = 1
    rrtTree = Tree(robot_obj, obstacles, start, goal)
    while total_n <= iter_n:
        sample_point = sample()
        if isCollisionFree(robot_obj, sample_point, obstacles):
            start_time = time.time()
            neighbour_node = rrtTree.nearest(sample_point)
            extended_node = rrtTree.extend(neighbour_node.data, sample_point, step_size)
            if extended_node is None:
                continue
            if not isCollisionFree(robot_obj, extended_node.data, obstacles):
                continue
            if rrtTree.add_node(neighbour_node, extended_node):
                continue

            # print("Round {step_id}, takes {runtime}s".format(step_id = total_n, runtime = round(time.time() - start_time, 3)))
            # print("")

            if extended_node.data == goal:
                break
            total_n += 1

    # If goal is reached.
    goal_node = rrtTree.exists(goal)
    if goal_node is not None:
        path = []
        pose_path = []
        curr = goal_node
        path.append(getCoord(goal))
        while curr.data != start:
            curr = rrtTree.parent(curr.data)
            path.append(getCoord(curr.data))
            pose_path.append(curr.data)

        dist = rrtTree.get_cost(pose_path[0])

        path.reverse()
        pose_path.reverse()

        return pose_path, dist, rrtTree
    else:
        print("No feasible solution this time.")
        return [], None, None


# Similar with Part 1's rrt* algorithm
def rrt_star(robot_obj, obstacles, start, goal, iter_n, step_size, radius):
    # Initialize a tree
    rrtTree = Tree(robot_obj, obstacles, start, goal)

    total_n = 1
    while total_n <= iter_n:
        sample_point = sample()
        if isCollisionFree(robot_obj, sample_point, obstacles):
            start_time = time.time()
            neighbour_node = rrtTree.nearest(sample_point)
            extended_node = rrtTree.extend(neighbour_node.data, sample_point, step_size)
            if extended_node is None:
                continue

            rewired_neighbour_node = rrtTree.rewire(extended_node.data, radius)
            if rewired_neighbour_node is not None:
                extended_node = rewired_neighbour_node
                if extended_node and extended_node.parent_node is None:
                    continue
                if not isPathCollisionFree(robot_obj, extended_node.parent_node.data, extended_node.data, obstacles):
                    continue
                if rrtTree.add_node(extended_node.parent_node, extended_node):
                    continue
            else:
                if not isPathCollisionFree(robot_obj, neighbour_node.data, extended_node.data, obstacles):
                    continue
                if rrtTree.add_node(neighbour_node, extended_node):
                    continue

            # print("Step {step_id}, takes {runtime}s".format(step_id = total_n, runtime = time.time() - start_time))
            # print("")
            if extended_node.data == goal:
                break
            total_n += 1


    goal_node = rrtTree.exists(goal)
    if goal_node:
        pose_path = []
        curr = goal_node
        pose_path.append(curr.data)
        while curr.data != start:
            curr = curr.parent_node
            pose_path.append(curr.data)

        dist = rrtTree.get_cost(pose_path[0])
        pose_path.reverse()

        return pose_path, dist, rrtTree
    else:
        print("No feasible solution this time.")
        return [], None, None


