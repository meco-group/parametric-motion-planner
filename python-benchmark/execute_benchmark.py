import sys
sys.path.append('build/')
import parametric_motion_planner_module as pmp
from load_random_environments import extract_data
import json

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
file_name_appendix = "_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# List all methods to benchmark
methods = [pmp.PlannerMethod.ARENA, 
        #    pmp.PlannerMethod.OCP, 
        #    pmp.PlannerMethod.OCP, 
        #    pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP,
        #    pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.P2P]
method_names = ["ARENA", 
                # "OCP-5", 
                # "OCP-10", 
                # "OCP-20", 
                "OCP-30", 
                # "OCP-40", 
                "P2P"]
assert len(methods) == len(method_names)

# Create motion planner
motion_planner = pmp.MotionPlanner(methods[0], local_param, local_env)

# create containers for results
results = {}
for m in method_names:
    results[m] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
                  "corridor_infeasibilities_detected": []}

# Benchmark
for method, method_name in zip(methods, method_names):
    motion_planner.SetMethod(method)

    # Set the correct number of points per corridor for the OCP method
    if method_name.startswith("OCP"):
        method_name_split = method_name.split("-")
        assert len(method_name_split) == 2
        n = int(method_name_split[1])
        motion_planner.SetOCPNumberOfPointsPerCorridor(n)

    # loop over all environments
    for i in range(len(envs)):
        idx_to_show = 60
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

        motion_planner.Plan()
        print("Travel time: ", motion_planner.GetTravelTime())

        # store results
        results[method_name]["Tf"].append(motion_planner.GetTravelTime())
        results[method_name]["t_comp_total"].append(motion_planner.GetTotalComputationTime())
        results[method_name]["t_comp_solver"].append(motion_planner.GetSolverTime())
        results[method_name]["corridor_infeasibilities_detected"].append(
            motion_planner.CorridorInfeasibilitiesDetected())

# store results as a json
import json
with open('python-benchmark/files/results' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)