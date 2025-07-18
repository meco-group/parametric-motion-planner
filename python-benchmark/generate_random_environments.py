import sys
sys.path.append('build/')
import json
import random
import parametric_motion_planner_module as pmp
import numpy as np

# setup json to store data
j = {}
j['params'] = []
j['envs'] = []
j['starts'] = []
j['dests'] = []

# set parameter limits
v_max_lb = 0.5
v_max_ub = 2.0

a_max_lb = 2.0
a_max_ub = 6.5

USE_CELL_POSITIONS = False

# create random parameters and environments
N = 100

# STRUCTURED_1
# env = pmp.Environment(9, 11, 0.12, 0.12)
# start = pmp.Point2Dd(0, 0)
# dest = pmp.Point2Dd(0, 0)
# p = pmp.Parameters(2.0, 6.0, 0.115, 0.115, 0.001)
# mp = pmp.MotionPlanner(pmp.PlannerMethod.OCP, p, env)
# counter = 0
# max_normal_corridors = -1
# max_extended_corridors = -1
# xx = [2, 2, 2, 2, 3, 3, 3, 3, 6, 6, 6, 6, 6, 8, 8, 8, 8, 9, 9, 9, 9]
# yy = [2, 3, 5, 6, 2, 3, 5, 6, 2, 3, 4, 5, 6, 2, 3, 5, 6, 2, 3, 5, 6]
# for x, y in zip(xx, yy):
#     env.DeleteCell(pmp.Point2Di(x, y))

# STRUCTURED_2
# env = pmp.Environment(9, 11, 0.12, 0.12)
# start = pmp.Point2Dd(0, 0)
# dest = pmp.Point2Dd(0, 0)
# p = pmp.Parameters(2.0, 6.0, 0.115, 0.115, 0.001)
# mp = pmp.MotionPlanner(pmp.PlannerMethod.OCP, p, env)
# counter = 0
# max_normal_corridors = -1
# max_extended_corridors = -1
# xx = [2, 3, 4, 6, 7, 8]
# yy = [4, 4, 4, 4, 4, 4]
# for x, y in zip(xx, yy):
#     env.DeleteCell(pmp.Point2Di(x, y))

# STRUCTURED_3
env = pmp.Environment(1+2+2+11+2, 2+1+3+1+2+1+3+3+1+2+1, 0.12, 0.12)
start = pmp.Point2Dd(0, 0)
dest = pmp.Point2Dd(0, 0)
p = pmp.Parameters(2.0, 6.0, 0.115, 0.115, 0.001)
mp = pmp.MotionPlanner(pmp.PlannerMethod.OCP, p, env)
counter = 0
max_normal_corridors = -1
max_extended_corridors = -1
xx = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  
      6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
      9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
      13, 14, 15, 13, 14, 15, 13, 14, 15, 16, 13, 14, 15,
      14, 15, 14, 15,
      17, 18, 17, 18,
      4, 5, 4, 5]
yy = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 
      2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
      2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      2, 2, 2, 5, 5, 5, 8, 8, 8, 8, 11, 11, 11,
      14, 14, 15, 15,
      1, 1, 2, 2,
      15, 15, 16, 16]
assert len(xx) == len(yy)
for x, y in zip(xx, yy):
    env.DeleteCell(pmp.Point2Di(x, y))

min_allowed_distance = 5*0.12

while counter < N:
    # randomize velocity and acceleration limits
    v_max = random.uniform(v_max_lb, v_max_ub)
    a_max = random.uniform(a_max_lb, a_max_ub)
    params = pmp.Parameters(v_max, a_max, 0.115, 0.115, 0.001)

    # randomize obstacles
    # env.AddRandomObstacles(0.25)

    # randomize start and destination points
    good_points_found = False
    while not good_points_found:
        if USE_CELL_POSITIONS:
            env.GetRandomFreeCellPosition(start)
            env.GetRandomFreeCellPosition(dest)
        else:
            env.GetRandomFreeVehiclePosition(start, params.GetVehWidth(), 
                                        params.GetVehHeight(), params.GetMargin())
            env.GetRandomFreeVehiclePosition(dest, params.GetVehWidth(),
                                        params.GetVehHeight(), params.GetMargin())
        # check if start and destination are far enough apart
        if (start.x()-dest.x())**2 + (start.y()-dest.y())**2 >= min_allowed_distance**2:
            good_points_found = True

    # check if a path exists
    mp.SetStart(start); mp.SetDest(dest)
    path_exists = False
    try:
        mp.SetCorridorExtendedMode(True)
        mp.UpdateCorridorSequence()
        extended_len = len(mp.GetCorridorSequence())

        mp.SetCorridorExtendedMode(False)
        mp.UpdateCorridorSequence()
        normal_len = len(mp.GetCorridorSequence())

        assert normal_len <= 10

        max_normal_corridors = max(max_normal_corridors, normal_len)
        max_extended_corridors = max(max_extended_corridors, extended_len)

        path_exists = True
    except:

        pass

    if path_exists:
        j['params'].append(json.loads(params.ToJson()))
        j['envs'].append(json.loads(env.ToJson()))
        j['starts'].append([start.x(), start.y()])
        j['dests'].append([dest.x(), dest.y()])
        counter += 1

print(f"Max normal corridors:   {max_normal_corridors}")
print(f"Max extended corridors: {max_extended_corridors}")

# write to file
# file_appendix = "_cell"
# file_appendix = "_double"
# file_appendix = "_large"
# file_appendix = "_large_double"
# file_appendix = "_large_double_more_obstacles" # 0.15 obstacle probability
# file_appendix = "_large_double_more_obstacles_25"
# file_appendix = "_structured_1"
# file_appendix = "_structured_3_large"
file_appendix = "_none"
        
with open('python-benchmark/files/random_environments' + file_appendix + '.json', 'w') as f:
    json.dump(j, f, indent=4)