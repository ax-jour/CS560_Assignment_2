import time
from collections import deque
from collision import *


class Tree:
    def __init__(self, robot, obstacles, start, goal):
        # Init data.
        self.robot = robot
        self.obstacles = obstacles
        self.start = start
        self.goal = goal

        # Reset data to start point when initializing.
        self.data = start
        # Parent will be none for this moment.
        self.parent_node = None
        # Noting children is good for traversal the tree from root.
        self.children = []
        # Cost for RRT*
        self.cost = 0
        # Tracking temp distance with current node.
        self.temp_dist = 0

    def add(self, point1, point2):
        # Check point1 exist or not and return the node.
        curr_node = self.exists(point1)
        if curr_node is None:
            return False
        # Initialized a new node for point2
        new_node = Tree(self.robot, self.obstacles, point2, self.goal)
        # Add new node to point1 node.
        curr_node.children.append(new_node)
        # Adding the point2 node to point1 node.
        new_node.parent_node = curr_node
        # Calculate the cost from root to point2 for RRT* use.
        new_node.cost = self.get_cost(point2)

    # Seems most of time, node1 is given and no need to check existence.
    # So I made another function which can add node to exist node directly.
    def add_node(self, node1, node2):
        if node1 is None:
            return False
        node1.children.append(node2)
        node2.parent_node = node1
        node2.cost = self.get_cost(node2.data)

    # Traversal the tree to find if the node is exist or not.
    def exists(self, point):
        if self.data == point:
            return self
        if len(self.children) > 0:
            for node in self.children:
                exist_node = node.exists(point)
                if exist_node:
                    return exist_node
            return None

    # Return current node's parent.
    def parent(self, point):
        curr = self.exists(point)
        return curr.parent_node

    # Check nearest node by comparing all nodes Euclidean distance.
    def nearest(self, point):
        # Returning all exist node in this tree.
        exist_points = self.treeTraversal()
        # Initialize a large number for comparing minimum dist.
        min_dist = 100
        nearest_node = None
        for n_point in exist_points:
            dist = EuclideanDist(point, n_point.data)
            n_point.temp_dist = dist
            min_dist = min(min_dist, dist)
            if min_dist == dist:
                nearest_node = n_point

        return nearest_node

    # Using discretized step size version in extending a new node
    def extend(self, point1, point2, step_size=1):
        curr_point = None
        x_diff, y_diff = 0, 0

        # If the Euclidean distance is less than step_size, then take that distance directly.
        if EuclideanDist(point1, point2) <= step_size:
            step_size = EuclideanDist(point1, point2)

        # Considering about the vertical/horizontal extending direction except normal tilt line.
        if (point2[1] - point1[1]) == 0:
            x_diff = step_size
        elif (point2[0] - point1[0]) == 0:
            y_diff = step_size
        else:
            k = (point2[1] - point1[1]) / (point2[0] - point1[0])
            x_diff = math.sqrt((step_size ** 2) / (1 + k ** 2))
            y_diff = math.sqrt((step_size ** 2) / (1 + 1 / (k ** 2)))

        # Considering about the relative position between the nearest node and new node.
        if (point2[1] - point1[1]) >= 0 and (point2[0] - point1[0]) >= 0:
            curr_point = (point1[0] + x_diff, point1[1] + y_diff)
        elif (point2[1] - point1[1]) >= 0 and (point2[0] - point1[0]) < 0:
            curr_point = (point1[0] - x_diff, point1[1] + y_diff)
        elif (point2[1] - point1[1]) < 0 and (point2[0] - point1[0]) < 0:
            curr_point = (point1[0] - x_diff, point1[1] - y_diff)
        elif (point2[1] - point1[1]) < 0 and (point2[0] - point1[0]) >= 0:
            curr_point = (point1[0] + x_diff, point1[1] - y_diff)

        # Check collision with boundaries.
        if curr_point[0] >= 10 or curr_point[0] <= 0 or curr_point[1] >= 10 or curr_point[1] <= 0:
            return None
        # Detecting the extending path collision
        if not isPathCollisionFree(self.robot, point1, curr_point, self.obstacles):
            return None
        # If the goal is within the step_size then connect it directly.
        if self.checkReachGoal(curr_point, step_size):
            return Tree(self.robot, self.obstacles, self.goal, self.goal)

        return Tree(self.robot, self.obstacles, curr_point, self.goal)

    def connLeastCostNeighbour(self, selected_node, radius):
        # Sort all nodes in tree by the least cost.
        sorted_cheapest_path_node_ls = sorted(self.treeTraversal(), key=lambda x: x.cost)
        # Initialize a large value to compare minimum value.
        curr_min_cost = 10000
        min_neighbor = None
        # Find the cheapest exist neighbour by ascending path cost.
        for neighbor in sorted_cheapest_path_node_ls:
            # Filter out any radius neighbour greater than given.
            if neighbor.temp_dist <= radius:
                # Calculate the distance from current node to the least cost neighbour
                # and the distance from start to the least cost neighbour.
                curr_dist = neighbor.cost + EuclideanDist(selected_node.data, neighbor.data)
                # Find out the least length from start to new node through selected neighbour.
                curr_min_cost = min(curr_min_cost, curr_dist)
                if curr_dist == curr_min_cost:
                    min_neighbor = neighbor
        if min_neighbor is not None:
            # Check collision of the path from current node to the least cost neighbour node.
            if isPathCollisionFree(self.robot, min_neighbor.data, selected_node.data, self.obstacles):
                # Add node into the tree.
                self.add_node(min_neighbor, selected_node)
                return True

        return False

    # Calculate the cost from root to selected point.
    def get_cost(self, point):
        total_dist = 0
        # Get current node from exist node in tree.
        curr_node = self.exists(point)
        if curr_node is None:
            return 100000
        # Once the point is start point then return the total cost of this path.
        while curr_node.data != self.start:
            # if curr_node.parent_node in curr_node.children:
            #     curr_node.children.remove(curr_node.parent_node)
            # Adding local distance to total distance.
            total_dist += EuclideanDist(curr_node.data, curr_node.parent_node.data)
            # Move pointer to parent node.
            curr_node = curr_node.parent_node
        return total_dist

    # Rewire the edge.
    def rewire_old(self, point, r):
        # Connect to the cheapest neighbour within an r radius.
        curr_node = Tree(self.robot, self.obstacles, point, self.goal)
        # Sort all nodes in tree by the least cost.
        sorted_cheapest_path_node_ls = sorted(self.treeTraversal(), key=lambda x: x.cost)
        # Sort all nodes from near to far.
        sorted_nearest_node_ls = sorted(self.treeTraversal(), key=lambda x: x.temp_dist)
        # Initialize a large value to compare minimum value.
        curr_min_cost = 10000
        min_neighbor = None
        # Find the cheapest exist neighbour by ascending path cost.
        for neighbour in sorted_cheapest_path_node_ls:
            # Filter out any radius neighbour greater than given.
            if neighbour.temp_dist <= r:
                # Calculate the distance from current node to the least cost neighbour
                # and the distance from start to the least cost neighbour.
                curr_dist = neighbour.cost + EuclideanDist(curr_node.data, neighbour.data)
                # Find out the least length from start to new node through selected neighbour.
                curr_min_cost = min(curr_min_cost, curr_dist)
                if curr_dist == curr_min_cost:
                    min_neighbor = neighbour
        if min_neighbor is not None:
            # Check collision of the path from current node to the least cost neighbour node.
            if isPathCollisionFree(self.robot, min_neighbor.data, curr_node.data, self.obstacles):
                # Add node into the tree.
                curr_node.parent_node = min_neighbor
                curr_node.cost = min_neighbor.cost + EuclideanDist(curr_node.data, min_neighbor.data)
                min_neighbor.children.append(curr_node)
        else:
            return None

        # Rewire the other children to current node.
        # Calculate the distance from start to the least neighbour node.
        dist_min_neighbor = EuclideanDist(min_neighbor.data, curr_node.data)
        for neighbour in sorted_nearest_node_ls:
            if EuclideanDist(neighbour.data, curr_node.data) <= r:
                # Calculate the distance from current node to the rewiring potential node.
                dist_neighbor = EuclideanDist(neighbour.data, curr_node.data)
                # Calculate the distance from start to the least neighbour node
                # and the distance from current to the least neighbour node.
                new_dist = dist_min_neighbor + dist_neighbor
                # Check if rewiring potential node would have less cost by go through current node.
                if new_dist + min_neighbor.cost < neighbour.cost:
                    if isPathCollisionFree(self.robot, neighbour.data, curr_node.data, self.obstacles):
                        # Rewire the node.
                        neighbour.parent_node.children.remove(neighbour)
                        neighbour.parent_node = curr_node
                        neighbour.cost = new_dist + min_neighbor.cost
                        curr_node.children.append(neighbour)

        return curr_node

    # Rewire the exist edge to more optimal path.
    def rewire(self, point, r, i):
        # Get point from tree.
        curr = self.exists(point)
        # Sort all nodes from near to far.
        sorted_nearest_node_ls = sorted(self.treeTraversal(), key=lambda x: x.temp_dist)
        for neighbour in sorted_nearest_node_ls:
            if EuclideanDist(neighbour.data, curr.data) <= r:
                # Calculate the distance from current node to the rewiring potential node.
                dist_neighbour = EuclideanDist(neighbour.data, curr.data)
                # Calculate the distance from start to the least neighbour node
                # and the distance from current to the least neighbour node.
                new_dist = curr.cost + dist_neighbour
                # Check if rewiring potential node would have less cost by go through current node.
                if new_dist < neighbour.cost:
                    if isPathCollisionFree(self.robot, neighbour.data, curr.data, self.obstacles):
                        # Rewire the node.
                        neighbour.parent_node.children.remove(neighbour)
                        self.add_node(curr, neighbour)


    def checkReachGoal(self, curr_point, dist):
        if EuclideanDist(curr_point, self.goal) < dist:
            if isPathCollisionFree(self.robot, curr_point, self.goal, self.obstacles):
                return True
        return False

    # Preorder tree traversal strategy
    def treeTraversal(self):
        Stack = deque([])
        Preorder = []
        Preorder.append(self)
        Stack.append(self)
        while len(Stack) > 0:
            flag = 0
            if len((Stack[len(Stack) - 1]).children) == 0:
                X = Stack.pop()
            else:
                Par = Stack[len(Stack) - 1]
                for i in range(0, len(Par.children)):
                    if Par.children[i] not in Preorder:
                        flag = 1
                        Stack.append(Par.children[i])
                        Preorder.append(Par.children[i])
                        break
                if flag == 0:
                    Stack.pop()
        return Preorder
