import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')
import parametric_motion_planner_module as pmp
from solve_omg_tools import omg_example
from load_random_environments import extract_data

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
file_name_appendix = "_large_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

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

    ### OMG ###
    motion_planner.UpdateCorridorSequence()
    corridors = motion_planner.GetCorridorSequence()
    solver_time, travel_time = omg_example(
        corridors, (starts[i].x(), starts[i].y()), 
        (dests[i].x(), dests[i].y()), params[i].GetVmax(), params[i].GetAmax(), 
        params[i].GetVehWidth(), params[i].GetVehHeight(),
        dump_to_json=True, file_name=f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/OMG_{digit}.json')
