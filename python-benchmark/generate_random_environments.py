import sys
sys.path.append('build/')
import json
import random
import parametric_motion_planner_module as pmp

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
N = 500
env = pmp.Environment(15, 15, 0.12, 0.12)
point = pmp.Point2Dd(0, 0)
for i in range(N):
    # randomize velocity and acceleration limits
    v_max = random.uniform(v_max_lb, v_max_ub)
    a_max = random.uniform(a_max_lb, a_max_ub)
    params = pmp.Parameters(v_max, a_max, 0.115, 0.115, 0.001)
    j['params'].append(json.loads(params.ToJson()))

    # randomize obstacles
    env.AddRandomObstacles(0.10)
    j['envs'].append(json.loads(env.ToJson()))

    # randomize start and destination points
    if USE_CELL_POSITIONS:
        env.GetRandomFreeCellPosition(point)
        j['starts'].append([point.x(), point.y()])
        env.GetRandomFreeCellPosition(point)
        j['dests'].append([point.x(), point.y()])
    else:
        env.GetRandomFreeVehiclePosition(point, params.GetVehWidth(), 
                                        params.GetVehHeight(), params.GetMargin())
        j['starts'].append([point.x(), point.y()])
        env.GetRandomFreeVehiclePosition(point, params.GetVehWidth(),
                                        params.GetVehHeight(), params.GetMargin())
        j['dests'].append([point.x(), point.y()])


# write to file
# file_appendix = "_cell"
# file_appendix = "_double"
# file_appendix = "_large"
# file_appendix = "_large_double"
# file_appendix = "_large_double_more_obstacles" # 0.15 obstacle probability
file_appendix = "_large_double_more_obstacles_10"
        
with open('python-benchmark/files/random_environments' + file_appendix + '.json', 'w') as f:
    json.dump(j, f, indent=4)