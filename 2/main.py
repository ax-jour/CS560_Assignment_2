import time

from file_parse import *
from visualizer import *
from sampler import sample
from collision import isCollisionFree
from rrt import *

from robot import robot

if __name__ == '__main__':
    parsedProb = parse_problem("robot_env_03.txt", "probs_01.txt")
    rob = robot(parsedProb[0][0], parsedProb[0][1])
    obstacles = parsedProb[1]
    start = parsedProb[2][4][0]
    goal = parsedProb[2][4][1]

    #######################################################################
    #### Visualize tree, path and solution
    #######################################################################

    # plt.figure()
    # #
    # visualize_problem(robot, obstacles, start, goal)
    # #
    # # rrt(rob, parsedProb[1], start, goal, 200, 1)
    # # path, dist, tree = rrt(robot, obstacles, start, goal, 500, 0.6)
    # path, dist, tree = rrt_star(robot, obstacles, start, goal, 500, 0.6, 1)
    # # visualize_path(robot, obstacles, path)
    # visualize_rrt(robot, obstacles, start, goal, 500, path, tree)
    # #
    # # sample_list = []
    # # for i in range(10):
    # #     sample_point = sample()
    # #     sample_list.append(sample_point)
    # #
    # # visualize_points(sample_list, rob, obstacles, start, goal)
    # #
    # plt.show()

    #######################################################################
    #### TEST RRT and RRT* algorithms, comparing success rate and distances
    #######################################################################
    iter_n_lst = [200, 300, 400, 500, 600]
    step_size_lst = [0.5, 0.5, 1, 1, 1]

    rounds = 5
    for n in range(rounds):

        rrt_passed = 0
        star_passed = 0
        rrt_total_dist = 0
        star_total_dist = 0

        rrt_runtime = 0
        star_runtime = 0

        total = 50
        iterate_n = iter_n_lst[n]
        step_size = step_size_lst[n]
        start_time = time.time()
        for i in range(total):
            temp_start, temp_end = None, None
            while True:
                temp_start = sample()
                temp_end = sample()
                if isCollisionFree(robot, temp_start, obstacles) and isCollisionFree(robot, temp_end, obstacles):
                    break

            # print("Round {i}, Start: {start} Goal: {goal}".format(i=i+1, start=temp_start, goal=temp_end))

            rrt_start = time.time()
            rrt_result, rrt_dist, rrt_tree = rrt(robot, obstacles, temp_start, temp_end, iterate_n, step_size)
            rrt_end = time.time()
            if len(rrt_result) > 0:
                rrt_passed += 1
                rrt_total_dist += rrt_dist
                rrt_runtime += (rrt_end - rrt_start)

            star_start = time.time()
            star_result, star_dist, star_tree = rrt_star(robot, obstacles, temp_start, temp_end, iterate_n, step_size,
                                                         1.5)
            star_end = time.time()
            if len(star_result) > 0:
                star_passed += 1
                star_total_dist += star_dist
                star_runtime += (star_end - star_start)

        print("Iteration number: {iter_n}, step_size: {step_size}".format(iter_n=iterate_n, step_size=step_size))
        print("RRT Successful rate: {passed}/{total}".format(passed=rrt_passed, total=total))
        print("RRT* Successful rate: {passed}/{total}".format(passed=star_passed, total=total))
        print("RRT average distance found: {dist}".format(dist=rrt_total_dist / rrt_passed))
        print("RRT* average distance found: {dist}".format(dist=star_total_dist / star_passed))
        print("RRT average runtime when found: {time}s".format(time=round(rrt_runtime / rrt_passed, 2)))
        print("RRT* average runtime when found: {time}s".format(time=round(star_runtime / rrt_passed, 2)))


