import time
from collections import deque
from collision import *
from sampler import *
from robot import *
from visualizer import *


class Tree:
    def __init__(self, robot, obstacles, start, goal, control_duration=None):
        self.robot = robot  # robot data structure
        self.obstacles = obstacles
        self.start = start # (x,y,r)
        self.goal = goal # (x,y,r)

        self.data = start
        self.parent_node = None
        self.children = []
        self.control_duration = control_duration

    def setControlDuration(self, control_duration):
        self.control_duration = control_duration

    def add(self, point1, point2):
        curr_node = self.exists(point1)
        if curr_node is None:
            return False
        new_node = Tree(self.robot, self.obstacles, point2, self.goal)
        if new_node not in curr_node.children:
            curr_node.children.append(new_node)
        new_node.parent_node = curr_node

    def add_node(self, node1, node2):
        if node1 is None:
            return False
        if node2 not in node1.children:
            node1.children.append(node2)
        node2.parent_node = node1

    def exists(self, point):
        if self.data == point:
            return self
        if len(self.children) > 0:
            for node in self.children:
                target = node.exists(point)
                if target:
                    return target
            return None

    def parent(self, point):
        curr = self.exists(point)
        return curr.parent_node

    # TODO Can be improved by K-D tree (not done yet)
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
    def extend(self, nearest_node, n1, n2, dt, sample_time):
        # Holding controls and duration samples
        u_list = []
        seq_list = []
        for i in range(sample_time):
            u_sample = sample_u()
            seq_sample = sample_seq(n1, n2)
            u_list.append(u_sample)
            seq_list.append(seq_sample)

        # Getting the new state after propagated, and also feasible controls, duration set.
        new_state, rtn_cont_dura = self.robot.propagate(nearest_node.data, u_list, seq_list, dt, self.obstacles)
        # If the new state is as same as old node, then it cannot move forward due to collision.
        if new_state == nearest_node.data:
            return None, None

        # If the goal is less than one dt of the last control, then connect to the goal directly.
        if self.checkReachGoal(new_state):
            return Tree(self.robot, self.obstacles, self.goal, self.goal), rtn_cont_dura

        return Tree(self.robot, self.obstacles, new_state, self.goal), rtn_cont_dura


    def checkReachGoal(self, curr_point):
        if RotatedTransDist(curr_point, self.goal) < 1:
            curr_rob = robot(self.robot.width, self.robot.height)
            curr_rob.set_pose(curr_point)
            goal_rob = robot(self.robot.width, self.robot.height)
            goal_rob.set_pose(self.goal)
            if isPathCollisionFree(self.robot, curr_rob.transform(), goal_rob.transform(), self.obstacles):
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
