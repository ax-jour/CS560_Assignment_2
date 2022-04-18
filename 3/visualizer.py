import matplotlib.pyplot as plt

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


def visualize_trajectory(robot_obj, obstacles, start, goal, path, dt):
    visualize_problem(robot_obj, obstacles, start, goal)

    curr_state = path[0].data
    for i in range(len(path) - 1):

        # Calculate again the propagate part for replay the trajectory.
        controls = path[i+1].control_duration[0]
        durations = path[i+1].control_duration[1]
        for j in range(len(controls)):
            kine = robot_obj.kinematic(curr_state, controls[j])

            new_state_x = curr_state[0] + (durations[j] * dt) * kine[0]
            new_state_y = curr_state[1] + (durations[j] * dt) * kine[1]
            new_state_r = curr_state[2] + (durations[j] * dt) * kine[2]

            new_state = (new_state_x, new_state_y, new_state_r)

            # Plotting the path and robot to chart.
            start_pt = (curr_state[0], curr_state[1])
            end_pt = (new_state[0], new_state[1])
            xp, yp = zip(*[start_pt, end_pt])
            plt.plot(xp, yp, 'b')
            plot_robot(robot_obj, new_state)

            curr_state = new_state

    # Plotting the goal path
    start_pt = (curr_state[0], curr_state[1])
    end_pt = (goal[0], goal[1])
    xp, yp = zip(*[start_pt, end_pt])
    plt.plot(xp, yp, 'b')


def plot_robot(robot, pose):
    robot.set_pose(pose)
    rob_ls = robot.transform()
    rob_ls.append(rob_ls[0])
    xss, yss = zip(*rob_ls)
    plt.plot(xss, yss, 'y')