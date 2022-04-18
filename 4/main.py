import time

from file_parse import *
from sampler import *
from collision import isCollisionFree
from rrt import *

from robot import robot

if __name__ == '__main__':
    parsedProb = parse_problem("robot_env_03.txt", "probs_01.txt")
    rob = robot(parsedProb[0][0], parsedProb[0][1])
    obstacles = parsedProb[1]
    start = parsedProb[2][4][0]
    goal = parsedProb[2][4][1]

    # # Check problem setup.
    # plt.figure()
    # #
    # # # visualize_problem(rob, obstacles, start, goal)
    # #
    # trajectory = rrt(rob, obstacles, start, goal, 0, 10, 1, 10, 800)
    # if len(trajectory) > 0:
    #     visualize_trajectory(rob, obstacles, start, goal, trajectory, 1)
    #
    # plt.show()


    #######################################################################
    #### TEST RRT algorithms of Dynamic car
    #######################################################################
    iter_n_lst = [600, 600, 600, 600, 600, 1000, 1000, 1000, 1000, 1000]
    n1_lst = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    n2_lst = [10, 10, 5, 5, 5, 10, 10, 5, 5, 5]
    dt_lst = [0.5, 0.5, 0.5, 1, 1, 0.5, 0.5, 0.5, 1, 1]
    sample_time_lst = [5, 5, 5, 5, 10, 5, 5, 5, 5, 10]
    rounds = 10
    for n in range(rounds):

        print("Test set: {set}".format(set=n + 1))

        rrt_passed = 0
        rrt_runtime = 0

        total = 60
        iter_n = iter_n_lst[n]
        n1 = n1_lst[n]
        n2 = n2_lst[n]
        dt = dt_lst[n]
        sample_time = sample_time_lst[n]

        start_time = time.time()
        for i in range(total):
            print(i * (100/total),"%")
            temp_goal = None
            while True:
                temp_goal = sample_point()
                # Check if the start and goal is valid or not.
                if isCollisionFree(rob, temp_goal, obstacles):
                    break

            rrt_start = time.time()
            path = rrt(rob, obstacles, start, temp_goal, n1, n2, dt, sample_time, iter_n)
            rrt_end = time.time()
            if len(path) > 0:
                rrt_passed += 1
                rrt_runtime += (rrt_end - rrt_start)

        print("Iteration number: {iter_n}, dt: {dt}".format(iter_n=iter_n, dt=dt))
        print("n1: {n1}, n2: {n2}".format(n1=n1, n2=n2))
        print("Sample_time: {sample_time}".format(sample_time=sample_time))
        print("RRT Successful rate: {passed}/{total}".format(passed=rrt_passed, total=total))
        print("RRT average runtime when found: {time}s".format(time=round(rrt_runtime / rrt_passed, 2)))



