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
# file_name_appendix = "_large_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# List all methods to benchmark
methods = [pmp.PlannerMethod.ARENA,
           pmp.PlannerMethod.ARENA, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.P2P]
method_names = ["ARENA", 
                "ARENA+",
                "OCP-5", 
                "OCP-10", 
                "OCP-20", 
                "OCP-30", 
                "OCP-40", 
                "P2P"]
default_selection = [1, 0, 0, 0, 0, 1, 0, 1]
arena_selection = [1, 0, 0, 0, 0, 0, 0, 0]
assert len(methods) == len(method_names)

my_selection = arena_selection

# Create motion planner
motion_planner = pmp.MotionPlanner(methods[1], local_param, local_env)

# create containers for results
# results = {}
# for m in method_names:
#     results[m] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
#                   "corridor_infeasibilities_detected": []}
results = json.load(open('python-benchmark/files/results' + file_name_appendix + '.json'))

expected_failures = []

# Benchmark
for method, method_name in zip(methods, method_names):
    if my_selection[method_names.index(method_name)] == 0:
        continue
    else:
        results[method_name] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [],
                                "corridor_infeasibilities_detected": []}

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
            motion_planner.SetSuboptimalityEliminationFeature(True)

    # loop over all environments
    for i in range(len(envs)):
        print(f"\n\nRunning environment {i} with method {method_name}")
        # [large]: suboptimal cases: [229 451 497 438 139 375 267 399 242 437]
        # 437: heuristics fail in last corridor leading to suboptimal AND infeasible solution
        # 242: ARENA+ solves this issue
        # 399: ARENA+ solves this issue
        # 267: ARENA+ solves this issue
        # 375: ARENA+ solves this issue
        # 139: heuristics fail in first corridor
        # ...
        
        # [large_double]: infeasible cases: [45, 46, 66, 69, 80, 88, 96, 98, 105, 110, 117, 119, 127, 130, 137, 138, 141, 165, 172, 211, 220, 225, 233, 236, 241, 248, 265, 281, 301, 309, 313, 314, 347, 371, 373, 394, 403, 404, 407, 431, 439, 443, 445, 451, 462, 469, 472, 482, 490]
        #
        idx_to_show = -66
        # idx_to_show = 162
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
            try:
                motion_planner.Plan()
            except:
                pass
            print("Done.")
            print("Travel time: ", motion_planner.GetTravelTime())

            # store results
            results[method_name]["Tf"].append(motion_planner.GetTravelTime())
            results[method_name]["t_comp_total"].append(motion_planner.GetTotalComputationTime())
            results[method_name]["t_comp_solver"].append(motion_planner.GetSolverTime())
            results[method_name]["corridor_infeasibilities_detected"].append(
                motion_planner.CorridorInfeasibilitiesDetected())
            
            if i == idx_to_show and method_name == method_to_show:
                print("infeasiblities: ", motion_planner.CorridorInfeasibilitiesDetected())
                motion_planner.PrintParametrization()

            corridors = motion_planner.GetCorridorSequence()
            if (len(corridors) > 1):
                first_overlap = [max(corridors[0][0], corridors[1][0]), 
                                min(corridors[0][1], corridors[1][1]),
                                max(corridors[0][2], corridors[1][2]), 
                                min(corridors[0][3], corridors[1][3])]
                last_overlap = [max(corridors[-2][0], corridors[-1][0]),
                                min(corridors[-2][1], corridors[-1][1]),
                                max(corridors[-2][2], corridors[-1][2]),
                                min(corridors[-2][3], corridors[-1][3])]
                if (first_overlap[0] <= starts[i].x() and
                    first_overlap[1] >= starts[i].x() and
                    first_overlap[2] <= starts[i].y() and
                    first_overlap[3] >= starts[i].y()):
                    expected_failures.append(i)

                elif (last_overlap[0] <= dests[i].x() and
                    last_overlap[1] >= dests[i].x() and
                    last_overlap[2] <= dests[i].y() and
                    last_overlap[3] >= dests[i].y()):
                    expected_failures.append(i)            

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


            
print(len(expected_failures))
print(expected_failures)

# store results as a json
import json
with open('python-benchmark/files/results' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)