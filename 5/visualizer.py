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
                x_a = dyna[3] * math.cos(curr_state[4]) * math.cos(curr_state[2])  # Acceleration in x axis
                y_a = dyna[3] * math.cos(curr_state[4]) * math.sin(curr_state[2])  # Acceleration in y axis

                S_x = dyna[0] * dt + (1 / 2) * x_a * dt ** 2  # Distance on x axis: Xv_0 * dt + (1/2) * Xa * dt^2
                S_y = dyna[1] * dt + (1 / 2) * y_a * dt ** 2  # Distance on y axis: Yv_0 * dt + (1/2) * Ya * dt^2

                new_state_x = curr_state[0] + S_x
                new_state_y = curr_state[1] + S_y
                new_state_r = curr_state[2] + dyna[2] * dt  # Angle of car: car angle + car angle velocity * dt
                new_v = dyna[3] * dt  # Velocity = acceleration * dt
                new_eta = curr_state[4] + dyna[4] * dt  # Angle of wheel: wheel angle + wheel angle velocity * dt

                new_state = (new_state_x, new_state_y, new_state_r, new_v, new_eta)

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