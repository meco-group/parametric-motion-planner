import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

# Extract the data
# file_name_appendix = ""
file_name_appendix = "_cell"
# file_name_appendix = "_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

# create containers for results
results = {"px": [], "py": []}

# Benchmark
# loop over all environments
for i in range(len(envs)):
    motion_planner.SetStart(starts[i])
    motion_planner.SetDest(dests[i])

    local_param.SetVmax(params[i].GetVmax())
    local_param.SetAmax(params[i].GetAmax())
    local_param.SetVehWidth(params[i].GetVehWidth())
    local_param.SetVehHeight(params[i].GetVehHeight())
    local_param.SetMargin(params[i].GetMargin())
    local_env.CopyObstacles(envs[i])

    motion_planner.Plan()

    # store results
    traj = motion_planner.GetLastSolution()
    nb = traj.nbSamples()
    results["px"].append(traj.Px()[:nb])
    results["py"].append(traj.Py()[:nb])

# store results as a json
import json
with open('python-benchmark/files/all_trajectories' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)