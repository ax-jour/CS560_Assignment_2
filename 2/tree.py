import time
from collections import deque
from collision import *


class Tree:
    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot  # robot data structure
        self.obstacles = obstacles
        self.start = start # (x,y,r)
        self.goal = goal # (x,y,r)

        self.data = start
        self.parent_node = None
        self.children = []
        self.cost = 0
        self.temp_dist = 0

    def add(self, point1, point2):
        curr_node = self.exists(point1)
        if curr_node is None:
            return False
        new_node = Tree(self.robot, self.obstacles, point2, self.goal)
        if new_node not in curr_node.children:
            curr_node.children.append(new_node)
        new_node.parent_node = curr_node
        new_node.cost = self.get_cost(point2)

    def add_node(self, node1, node2):
        if node1 is None:
            return False
        if node2 not in node1.children:
            node1.children.append(node2)
        node2.parent_node = node1
        node2.cost = self.get_cost(node2.data)

    def exists(self, point):
        if self.data == point:
            return self
        if len(self.children) > 0:
            for node in self.children:
                exist_node = node.exists(point)
                if exist_node:
                    return exist_node
            return None

    def parent(self, point):
        curr = self.exists(point)
        return curr.parent_node

    # Using sqrt((x1 - x2)^2 + (y_1 - y_2)^2 + (angle difference)^2)
    def nearest(self, point):
        exist_points = self.treeTraversal()
        min_dist = 100
        nearest_node = None
        for n_point in exist_points:
            dist = RotatedTransDist(point, n_point.data)
            n_point.temp_dist = dist
            min_dist = min(min_dist, dist)
            if min_dist == dist:
                nearest_node = n_point

        return nearest_node

    # Using discretized step size version in extending
    def extend(self, pose1, pose2, step_size):

        # Rotate
        rotated_point = (pose1[0], pose1[1], pose2[2])
        self.robot.set_pose(rotated_point)
        rotated_rob_geo = self.robot.transform()

        # Translate
        point1 = (pose1[0], pose1[1])
        point2 = (pose2[0], pose2[1])

        curr_point = None
        x_diff, y_diff = 0, 0

        if EuclideanDist(point1, point2) <= step_size:
            step_size = EuclideanDist(point1, point2)

        if (point2[1] - point1[1]) == 0:
            x_diff = step_size
        elif (point2[0] - point1[0]) == 0:
            y_diff = step_size
        else:
            k = (point2[1] - point1[1]) / (point2[0] - point1[0])
            x_diff = math.sqrt((step_size ** 2) / (1 + k ** 2))
            y_diff = math.sqrt((step_size ** 2) / (1 + 1 / (k ** 2)))

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

        curr_pose = (curr_point[0], curr_point[1], pose2[2])
        if not isPathCollisionFree(self.robot, rotated_point, curr_pose, self.obstacles):
            return None
        if self.checkReachGoal(curr_pose, step_size):
            return Tree(self.robot, self.obstacles, self.goal, self.goal)

        return Tree(self.robot, self.obstacles, curr_pose, self.goal)

    def get_cost(self, point):
        total_dist = 0
        curr_node = self.exists(point)
        if curr_node is None:
            return 100000
        while curr_node.data != self.start:
            if curr_node.parent_node in curr_node.children:
                curr_node.children.remove(curr_node.parent_node)
            total_dist += RotatedTransDist(curr_node.data, curr_node.parent_node.data)
            curr_node = curr_node.parent_node
        return total_dist

    def rewire(self, pose, r):
        # Connect to the cheapest neighbour within an r radius.
        curr_node = Tree(self.robot, self.obstacles, pose, self.goal)
        sorted_cheapest_path_node_ls = sorted(self.treeTraversal(), key=lambda x: x.cost)
        sorted_farthest_node_ls = sorted(self.treeTraversal(), key=lambda x: x.temp_dist, reverse=True)
        curr_min_cost = 1000000
        min_neighbor = None
        # Find the cheapest exist neighbour by ascending path cost.
        for neighbor in sorted_cheapest_path_node_ls:
            # Filter out any radius neighbour greater than given.
            if neighbor.temp_dist <= r:
                curr_dist = neighbor.cost + RotatedTransDist(curr_node.data, neighbor.data)
                # Find out the least length from start to new node through selected neighbour.
                curr_min_cost = min(curr_min_cost, curr_dist)
                if curr_dist == curr_min_cost:
                    min_neighbor = neighbor
        if min_neighbor is not None:
            if isPathCollisionFree(self.robot, min_neighbor.data, curr_node.data, self.obstacles):
                curr_node.parent_node = min_neighbor
                curr_node.cost = min_neighbor.cost + RotatedTransDist(curr_node.data, min_neighbor.data)
                min_neighbor.children.append(curr_node)
        else:
            return None

        # Rewire the other children to current node.
        dist_min_neighbor = RotatedTransDist(min_neighbor.data, curr_node.data)
        for neighbor in sorted_farthest_node_ls:
            dist_neighbor = RotatedTransDist(neighbor.data, curr_node.data)
            new_dist = dist_min_neighbor + dist_neighbor
            if new_dist + min_neighbor.cost < neighbor.cost:
                if isPathCollisionFree(self.robot, neighbor.data, curr_node.data, self.obstacles):
                    neighbor.parent_node.children.remove(neighbor)
                    neighbor.parent_node = curr_node
                    neighbor.cost = new_dist + min_neighbor.cost
                    curr_node.children.append(neighbor)

        return curr_node


    def checkReachGoal(self, curr_point, dist):
        if RotatedTransDist(curr_point, self.goal) < dist:
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
