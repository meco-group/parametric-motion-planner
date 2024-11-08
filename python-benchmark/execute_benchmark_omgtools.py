import json
import time
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data
from solve_omg_tools import omg_example

# Extract the data
# file_name_appendix = ""
file_name_appendix = "_cell"
# file_name_appendix = "_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)

# create containers for results
results = json.load(open('python-benchmark/files/results' + file_name_appendix + '.json'))
results["OmgTools"] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
                       "corridor_infeasibilities_detected": []}

# Benchmark
# loop over all environments
for i in range(len(envs)):
    print(f"\n\nRunning environment {i} with method OmgTools")
    idx_to_show = -70
    if (i == idx_to_show):
        print(f"Start: {starts[i].x()}, {starts[i].y()}")
        print(f"Dest: {dests[i].x()}, {dests[i].y()}")
        print(f"VehWidth: {params[i].GetVehWidth()}")
        print(f"VehHeight: {params[i].GetVehHeight()}")
        print(f"Margin: {params[i].GetMargin()}")
        print(f"Vmax: {params[i].GetVmax()}")
        print(f"Amax: {params[i].GetAmax()}")
        env = json.loads(envs[i].ToJson())
        print(env)
        grid = env["occupancy_grid"]
        rr = []
        cc = []
        for ii in range(len(grid)):
            for jj in range(len(grid[ii])):
        #         print(i, j)
                if grid[ii][jj] != 0:
                    rr.append(ii)
                    cc.append(jj)
        print(f"rr_test: {rr}")
        print(f"cc_test: {cc}")

    if (i == idx_to_show+1):
        exit()
        # break

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
    solver_time, travel_time = omg_example(
        corridors, (starts[i].x(), starts[i].y()), 
        (dests[i].x(), dests[i].y()), params[i].GetVmax(), params[i].GetAmax(), 
        params[i].GetVehWidth(), params[i].GetVehHeight())
    b = time.time()
    results["OmgTools"]["Tf"].append(travel_time)
    results["OmgTools"]["t_comp_total"].append((b - a) * 1000)
    results["OmgTools"]["t_comp_solver"].append(solver_time * 1000)
    results["OmgTools"]["corridor_infeasibilities_detected"].append(
        False)


# store results as a json
import json
with open('python-benchmark/files/results' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)