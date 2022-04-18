from file_parse import parse_problem
from visualizer import *
from rrt import *

# parsedProb = parse_problem("./1/robot_env_01.txt", "./1/probs_01.txt")
# # print(parsedProb)
#
# sample_point = sample()
# # sample_point = (-0.1,0)
# # print(sample_point)
#
# # visualize_points([sample_point], parsedProb[0], parsedProb[1], (0.0, 0.0), (10.0, 10.0))
# # visualize_points([(0.0, 0.0)], parsedProb[0], parsedProb[1], (3.0, 0.0), (5.6, 8.1))
# # print(isCollisionFree(parsedProb[0], sample_point, parsedProb[1]))
#
# a = Tree(parsedProb[0], parsedProb[1], (0.0, 0.0), (10.0, 10.0))
# a.add((0.0, 0.0), (10.0, 10.0))
# a.add((10.0, 10.0), sample())
# a.add((10.0, 10.0), (8.8, 9.9))
# a.add((0.0, 0.0), sample())
# a.add((0.0, 0.0), (8.9, 9.9))
# a.add((10.0, 10.0), (6.9, 8.1))
#
# # print(a.parent((8.9, 9.9)).root)
# # y = a.traversalsTree(a)
# # a.treeTraversal(a)
# # print(a.nearest((7.0, 8.0)).data)
#
# print(a.extend((3.0, 0.0), (5.6, 8.1)).data)

# print(isCollisionFree(parsedProb[0], (9.963277505858553, 9.984142559348012),parsedProb[1]))

if __name__ == '__main__':
    parsedProb = parse_problem("robot_env_01.txt", "probs_01.txt")
    robot = parsedProb[0]
    obstacles = parsedProb[1]
    start = parsedProb[2][2][0]
    goal = parsedProb[2][2][1]

    #######################################################################
    #### Visualize tree, path and solution
    #######################################################################

    # plt.figure()
    #
    # path, dist, tree = rrt(robot, obstacles, start, goal, 500, 0.8)
    # if path is not None:
    #     visualize_rrt(robot, obstacles, start, goal, 500, path, tree)
    #
    # # path, dist, tree = rrt_star(robot, obstacles, start, goal, 800, 0.8, 1.5)
    # # if path is not None:
    # #     visualize_rrt_star(robot, obstacles, start, goal, 800, path, tree)
    #
    # plt.show()


    #######################################################################
    #### TEST RRT and RRT* algorithms, comparing success rate and distances
    #######################################################################
    rrt_passed = 0
    star_passed = 0
    rrt_total_dist = 0
    star_total_dist = 0

    rrt_runtime = 0
    star_runtime = 0

    total = 50
    iterate_n = 200
    step_size = 0.8
    start_time = time.time()
    for i in range(total):
        temp_start, temp_end = None, None
        while True:
            temp_start = sample()
            temp_end = sample()
            if isCollisionFree(robot, temp_start, obstacles) and isCollisionFree(robot, temp_end, obstacles):
                break

        print("Round {i}, Start: {start} Goal: {goal}".format(i=i + 1, start=temp_start, goal=temp_end))

        rrt_start = time.time()
        rrt_result, rrt_dist, rrt_tree = rrt(robot, obstacles, temp_start, temp_end, iterate_n, step_size)
        rrt_end = time.time()
        if len(rrt_result) > 0:
            rrt_passed += 1
            rrt_total_dist += rrt_dist
            rrt_runtime += (rrt_end - rrt_start)

        star_start = time.time()
        star_result, star_dist, star_tree = rrt_star(robot, obstacles, temp_start, temp_end, iterate_n, step_size, 1.5)
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
    # print("Total runtime: {time}s".format(time=round(time.time() - start_time, 2)))

