import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
# file_name_appendix = "_large_double"
file_name_appendix = "_large_double_more_obstacles_10"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

# create containers for results
results = {"ARENA_px": [], "ARENA_py": [], "OCP_px": [], "OCP_py": [],
           "P2P_px": [], "P2P_py": [], "envs": [], "corridors": []}

# Benchmark
# loop over all environments
for i in range(len(envs)):
    digit = f'00{i}' if i < 10 else f'0{i}' if i < 100 else f'{i}'
    motion_planner.SetStart(starts[i])
    motion_planner.SetDest(dests[i])

    local_param.SetVmax(params[i].GetVmax())
    local_param.SetAmax(params[i].GetAmax())
    local_param.SetVehWidth(params[i].GetVehWidth())
    local_param.SetVehHeight(params[i].GetVehHeight())
    local_param.SetMargin(params[i].GetMargin())
    local_env.CopyObstacles(envs[i])

    ### ARENA ###
    motion_planner.SetMethod(pmp.PlannerMethod.ARENA)
    motion_planner.Plan()
    motion_planner.DumpToJson(f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/ARENA_{digit}.json', False)

    ### OCP ###
    motion_planner.SetMethod(pmp.PlannerMethod.OCP)
    motion_planner.Plan()
    motion_planner.DumpToJson(f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/OCP_{digit}.json', False)

    ### P2P ###
    motion_planner.SetMethod(pmp.PlannerMethod.P2P)
    motion_planner.Plan()
    motion_planner.DumpToJson(f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/P2P_{digit}.json', False)

# store results as a json
import json
with open('python-benchmark/files/all_trajectories' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)