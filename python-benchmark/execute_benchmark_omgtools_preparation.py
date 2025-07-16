import json
import time
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
# file_name_appendix = "_large_double_more_obstacles_25"
# file_name_appendix = "_structured_1"
file_name_appendix = "_" + json.loads(open('python-benchmark/benchmark_settings.json').read())["benchmark_name"]
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)
# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

# create containers for results
omg_tools_benchmark_preparation = {
    "corridors":[], "start":[], "dest":[], "vmax":[], "amax":[], 
    "veh_width":[], "veh_height":[], "margin":[], "start_vel":[],
    "corridor_construction_time":[]}

# Benchmark
# loop over all environments
for i in range(len(envs)):
    print(f"\n\nRunning environment {i} with method OmgTools")
    
    motion_planner.SetStart(starts[i])
    motion_planner.SetDest(dests[i])

    local_param.SetVmax(params[i].GetVmax())
    local_param.SetAmax(params[i].GetAmax())
    local_param.SetVehWidth(params[i].GetVehWidth())
    local_param.SetVehHeight(params[i].GetVehHeight())
    local_param.SetMargin(params[i].GetMargin())
    local_env.CopyObstacles(envs[i])

    a = time.time()
    motion_planner.UpdateCorridorSequence()
    corridors = motion_planner.GetCorridorSequence()
    b = time.time()

    omg_tools_benchmark_preparation["corridors"].append(corridors)
    omg_tools_benchmark_preparation["start"].append((starts[i].x(), starts[i].y()))
    omg_tools_benchmark_preparation["dest"].append((dests[i].x(), dests[i].y()))
    omg_tools_benchmark_preparation["vmax"].append(params[i].GetVmax())
    omg_tools_benchmark_preparation["amax"].append(params[i].GetAmax())
    omg_tools_benchmark_preparation["veh_width"].append(params[i].GetVehWidth())
    omg_tools_benchmark_preparation["veh_height"].append(params[i].GetVehHeight())
    omg_tools_benchmark_preparation["margin"].append(params[i].GetMargin())
    omg_tools_benchmark_preparation["start_vel"].append((0, 0))
    omg_tools_benchmark_preparation["corridor_construction_time"].append((b - a) * 1000)   

# store results as a json
import json
with open('python-benchmark/files/omg_tools_preparation' + file_name_appendix + '.json', 'w') as f:
    json.dump(omg_tools_benchmark_preparation, f, indent=4)