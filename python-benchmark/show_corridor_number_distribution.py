import json
import sys
sys.path.append('build/')
import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

# Extract the data
file_name_appendix = "_large_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# loop over all environments and compute the corridor sequence using a motion planner object
corridor_number_distribution = []
dist = {"6":[], "5":[], "4":[], "3":[], "2":[], "1":[]}

motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

for i in range(len(envs)):
    motion_planner.SetStart(starts[i])
    motion_planner.SetDest(dests[i])

    local_param.SetVmax(params[i].GetVmax())
    local_param.SetAmax(params[i].GetAmax())
    local_param.SetVehWidth(params[i].GetVehWidth())
    local_param.SetVehHeight(params[i].GetVehHeight())
    local_param.SetMargin(params[i].GetMargin())
    local_env.CopyObstacles(envs[i])

    motion_planner.UpdateCorridorSequence() 
    n = len(motion_planner.GetCorridorSequence())
    corridor_number_distribution.append(n)

    dist[str(n)].append(i)

# make a histogram of the amount of corridors
print(dist)
import matplotlib.pyplot as plt
plt.hist(corridor_number_distribution, bins=range(1, 10))
plt.show()