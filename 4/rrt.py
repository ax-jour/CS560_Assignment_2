import time
from matplotlib import pyplot as plt
from helper import *
from sampler import *
from tree import Tree
from collision import isCollisionFree, isPathCollisionFree
from visualizer import *


def rrt(robot_obj, obstacles, start, goal, n1, n2, dt, sample_time, iter_n):
    total_n = 1
    rrtTree = Tree(robot_obj, obstacles, start, goal)
    while total_n <= iter_n:
        sample_pt = sample_point()
        if isCollisionFree(robot_obj, sample_pt, obstacles):
            start_time = time.time()
            neighbour_node = rrtTree.nearest(sample_pt)
            # Getting the extended node and set of controls and duration.
            extended_node, rtn_cont_dura = rrtTree.extend(neighbour_node, n1, n2, dt, sample_time)
            if extended_node is None:
                continue
            # Since already checked collision, add the node directly.
            if rrtTree.add_node(neighbour_node, extended_node):
                continue

            # Assign the controls and duration information into the node, for traversal
            extended_node.control_duration = rtn_cont_dura

            # print("Round {step_id}, takes {runtime}s".format(step_id = total_n, runtime = round(time.time() - start_time, 3)))

            if (extended_node.data[0], extended_node.data[1]) == (goal[0], goal[1]):
                break
            total_n += 1


    goal_node = rrtTree.exists(goal)
    if goal_node is not None:
        pose_path = []
        curr = goal_node
        pose_path.append(goal_node)
        while (curr.data[0], curr.data[1], curr.data[2]) != start:
            curr = curr.parent_node
            pose_path.append(curr)

        pose_path.reverse()

        return pose_path
    else:
        return []