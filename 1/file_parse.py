from helper import *

# Parsing world file and problem file then make them as a tuple.
def parse_problem(world_file, problem_file):
    f_world = open(world_file, "r")
    f_prob = open(problem_file, "r")

    rtn_tuple = []

    # Parsing robot.
    # Func "chunks" can cut list as n in one tuple.
    # Func "str2floatInNestedLst" transfers str to float within the list.
    robot = str2floatInNestedLst(chunks(f_world.readline().split(), 2))
    rtn_tuple.append(robot)

    # Parsing obstacles list. Pre-inserted tuples are boundary lines.
    obstacles = [[(0, 0), (0, 10)], [(0, 10), (10, 10)], [(10, 10), (10, 0)], [(10, 0), (0, 0)]]
    for obsLine in f_world:
        obs = str2floatInNestedLst(chunks(obsLine.split(), 2))
        obstacles.append(obs)
    rtn_tuple.append(obstacles)

    # Parsing problems line into list.
    problems = []
    for probLine in f_prob:
        prob = str2floatInNestedLst(chunks(probLine.split(), 2))
        problems.append(prob)
    rtn_tuple.append(problems)

    return tuple(rtn_tuple)
