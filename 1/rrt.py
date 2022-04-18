import time
from matplotlib import pyplot as plt

from sampler import sample
from tree import Tree
from collision import isCollisionFree, isPathCollisionFree
from visualizer import visualize_problem, visualize_points

# RRT algorithm
# Step_size optional params for lengths moving forward.
def rrt(robot, obstacles, start, goal, iter_n, step_size):
    # Initialize a tree
    rrtTree = Tree(robot, obstacles, start, goal)

    total_n = 1
    while total_n <= iter_n:
        # Sampling a random point ([0-10], [0-10])
        sample_point = sample()
        # Check if the sample point doesn't have collision with any obstacle.
        if isCollisionFree(robot, sample_point, obstacles):
            start_time = time.time()
            # Find the nearest point with the sample point.
            neighbour_node = rrtTree.nearest(sample_point)
            # Extend a step_size point toward the sample point.
            extended_node = rrtTree.extend(neighbour_node.data, sample_point, step_size)
            # Check none.
            if extended_node is None:
                continue
            # If the path from the nearest node to new node will collide with any obstacle, then abort.
            if not isPathCollisionFree(robot, neighbour_node.data, extended_node.data, obstacles):
                continue
            # If the point is valid, then add it into the three.
            if rrtTree.add_node(neighbour_node, extended_node):
                continue

            print("Step {step_id}, takes {runtime}s".format(step_id = total_n, runtime = time.time() - start_time))
            # print("")

            # If the tree can connect to the goal before iteration end, then break out.
            if extended_node.data == goal:
                break
            total_n += 1

    # If goal is reached.
    goal_node = rrtTree.exists(goal)
    if goal_node:
        path = []
        curr = goal_node
        path.append(curr.data)
        while curr.data != start:
            curr = curr.parent_node
            path.append(curr.data)

        distance = rrtTree.get_cost(path[0])
        path.reverse()
        return path, distance, rrtTree
    else:
        return [], None, None

# RRT* algorithm
def rrt_star(robot, obstacles, start, goal, iter_n, step_size, radius):
    # Initialize a tree
    rrtTree = Tree(robot, obstacles, start, goal)

    total_n = 1
    while total_n <= iter_n:
        sample_point = sample()
        if isCollisionFree(robot, sample_point, obstacles):
            start_time = time.time()
            neighbour_node = rrtTree.nearest(sample_point)
            # Instead of extend directly, extend node here just give a sample point for finding the nearest point.
            selected_node = rrtTree.extend(neighbour_node.data, sample_point, step_size)
            if selected_node is None:
                continue
            # Instead of choosing the least distance neighbour, choose the one who cost less.
            if not rrtTree.connLeastCostNeighbour(selected_node, radius):
                continue
            rrtTree.rewire(selected_node.data, radius, total_n)

            print("Step {step_id}, takes {runtime}s".format(step_id = total_n, runtime = time.time() - start_time))
            # print("")

            if selected_node.data == goal:
                break
            total_n += 1

    # If goal is reached.
    goal_node = rrtTree.exists(goal)
    if goal_node:
        path = []
        curr = goal_node
        path.append(curr.data)
        while curr.data != start:
            curr = curr.parent_node
            path.append(curr.data)

        distance = rrtTree.get_cost(path[0])
        path.reverse()

        return path, distance, rrtTree
    else:
        return [], None, None



