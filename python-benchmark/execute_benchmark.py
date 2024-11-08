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
file_name_appendix = "_large"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# List all methods to benchmark
methods = [pmp.PlannerMethod.ARENA,
        #    pmp.PlannerMethod.ARENA, 
        #    pmp.PlannerMethod.OCP, 
        #    pmp.PlannerMethod.OCP, 
        #    pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP,
        #    pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.P2P]
method_names = ["ARENA", 
                # "ARENA+",
                # "OCP-5", 
                # "OCP-10", 
                # "OCP-20", 
                "OCP-30", 
                # "OCP-40", 
                "P2P"]
assert len(methods) == len(method_names)

# Create motion planner
motion_planner = pmp.MotionPlanner(methods[1], local_param, local_env)

# create containers for results
results = {}
for m in method_names:
    results[m] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
                  "corridor_infeasibilities_detected": []}

# Benchmark
for method, method_name in zip(methods, method_names):
    if method is not None:
        motion_planner.SetMethod(method)

    # Set the correct number of points per corridor for the OCP method
    if method_name.startswith("OCP"):
        method_name_split = method_name.split("-")
        assert len(method_name_split) == 2
        n = int(method_name_split[1])
        motion_planner.SetOCPNumberOfPointsPerCorridor(n)
    
    # Toggle the EliminateSuboptimalityFeature for ARENA+
    if method_name.startswith("ARENA"):
        method_name_split = method_name.split("+")
        if len(method_name_split) == 2:
            motion_planner.SetSuboptimalityEliminationFeature(True)
        else:
            motion_planner.SetSuboptimalityEliminationFeature(False)

    # loop over all environments
    for i in range(len(envs)):
        print(f"\n\nRunning environment {i} with method {method_name}")
        # [70 42 41 80 67 76 13 27 51  1]
        idx_to_show = 437
        method_to_show = "ARENA"#"OCP-30"
        if i == idx_to_show and method_name == method_to_show:
            print(f"\tcorridor_meta_data = ['nominal']*len(corridors)")
            print(f"\tp0 = [{starts[i].x()}, {starts[i].y()}]")
            print(f"\tpf = [{dests[i].x()}, {dests[i].y()}]")
            print(f"\tv0 = [0, 0]")
            print(f"\tparams = {{'a_max': {params[i].GetAmax()}, 'v_max': {params[i].GetVmax()}, 'veh_width': {params[i].GetVehWidth()}, 'veh_height': {params[i].GetVehHeight()}, 'M': {params[i].GetMargin()}}}")

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

        if (i == idx_to_show+1 and method_name == method_to_show):
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

        if method is not None:
            print("Planning...")
            motion_planner.Plan()
            print("Done.")
            print("Travel time: ", motion_planner.GetTravelTime())

            # store results
            results[method_name]["Tf"].append(motion_planner.GetTravelTime())
            results[method_name]["t_comp_total"].append(motion_planner.GetTotalComputationTime())
            results[method_name]["t_comp_solver"].append(motion_planner.GetSolverTime())
            results[method_name]["corridor_infeasibilities_detected"].append(
                motion_planner.CorridorInfeasibilitiesDetected())
        else:
            motion_planner.UpdateCorridorSequence()
            corridors = motion_planner.GetCorridorSequence()
            omg_example(corridors, (starts[i].x(), starts[i].y()), 
                        (dests[i].x(), dests[i].y()), 
                        params[i].GetVmax(), 
                        params[i].GetAmax(), params[i].GetVehWidth(), 
                        params[i].GetVehHeight())
            results[method_name]["Tf"].append(1.5)
            results[method_name]["t_comp_total"].append(0.6)
            results[method_name]["t_comp_solver"].append(0.1)
            results[method_name]["corridor_infeasibilities_detected"].append(
                False)


# store results as a json
import json
with open('python-benchmark/files/results' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)