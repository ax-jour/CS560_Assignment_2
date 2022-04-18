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
    # Plot the environment first.
    visualize_problem(robot_obj, obstacles, start, goal)

    curr_state = path[0].data
    for i in range(len(path) - 1):

        # Calculate again the propagate part for replay the trajectory.
        controls = path[i+1].control_duration[0]
        durations = path[i+1].control_duration[1]
        for j in range(len(controls)):
            # Calculate q dot dot for a control
            dyna = robot_obj.dynamics(curr_state, controls[j])

            for n in range(durations[j]):
                # Calculate the new state after duration n of dt.
                S_x = dyna[0] * dt + (1 / 2) * dyna[3] * dt ** 2
                S_y = dyna[1] * dt + (1 / 2) * dyna[4] * dt ** 2
                S_r = dyna[2] * dt + (1 / 2) * dyna[5] * dt ** 2
                new_state_x = curr_state[0] + S_x
                new_state_y = curr_state[1] + S_y
                new_state_r = curr_state[2] + S_r
                new_x_v = dyna[0] + dyna[3] * dt
                new_y_v = dyna[1] + dyna[4] * dt
                new_r_v = dyna[2] + dyna[5] * dt

                new_state = (new_state_x, new_state_y, new_state_r, new_x_v, new_y_v, new_r_v)

                # Plotting the path and robot to chart.
                start_pt = (curr_state[0], curr_state[1])
                end_pt = (new_state[0], new_state[1])
                xp, yp = zip(*[start_pt, end_pt])
                plt.plot(xp, yp, 'b')

                curr_state = new_state

            plot_robot(robot_obj, curr_state)

    # Plotting the path to goal(one time).
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