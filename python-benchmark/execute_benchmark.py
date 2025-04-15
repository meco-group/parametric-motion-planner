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
# file_name_appendix = "_large_double_more_obstacles"
file_name_appendix = "_large_double_more_obstacles_10"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# List all methods to benchmark
methods = [pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.ARENA,
           pmp.PlannerMethod.P2P,
           pmp.PlannerMethod.ARENA,
           pmp.PlannerMethod.ARENA, 
           pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.OCP]
method_names = ["OCP-30",
                "OCP-5", 
                "OCP-10", 
                "OCP-20", 
                "ARENA", 
                "P2P",
                "ARENA-FATROP",
                "ARENA-EXTENDED-FATROP",
                "OCP-30-FATROP", 
                "OCP-30-EXTENDED-FATROP"]
default_selection = [0, 0, 0, 0, 1, 1, 1, 1, 1, 1]
arena_selection = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
extended_selection = [0, 0, 0, 0, 0, 0, 0, 1, 0, 1]
assert len(methods) == len(method_names)

# my_selection = default_selection
my_selection = arena_selection
# my_selection = extended_selection

# Create motion planner
motion_planner = pmp.MotionPlanner(methods[1], local_param, local_env)
motion_planner.SetSolver("ipopt", False)
motion_planner.SetJustInTimePreparationMode(False)
# motion_planner.SetOptimizationApproach("original")
# motion_planner.SetOptimizationApproach("new formulation")

# create containers for results
# results = {}
# for m in method_names:
#     results[m] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
#                   "corridor_infeasibilities_detected": []}
try:
    results = json.load(open('python-benchmark/files/results_new' + file_name_appendix + '.json'))
except FileNotFoundError:
    results = {}
    with open('python-benchmark/files/results_new' + file_name_appendix + '.json', 'w') as f:
        json.dump(results, f, indent=4)
    
    results = json.load(open('python-benchmark/files/results_new' + file_name_appendix + '.json'))


expected_failures = []
failures = []
avg_solver_time = 0
avg_travel_time = 0

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
    if method_name.startswith("OCP") and not method_name.endswith("FATROP"):
        method_name_split = method_name.split("-")
        assert len(method_name_split) == 2
        n = int(method_name_split[1])
        motion_planner.SetOCPNumberOfPointsPerCorridor(n)

    # check if we want extended approach or not
    if "EXTENDED" in method_name:
        motion_planner.SetCorridorExtendedMode(True)
    else:
        motion_planner.SetCorridorExtendedMode(False)

    if method_name == "ARENA-FATROP":
        motion_planner.SetSolver("fatrop", True)

    # set the correct solver
    # if method_name.endswith("FATROP"):
    #     motion_planner.SetSolver("fatrop", False)
    # else:
    #     motion_planner.SetSolver("ipopt", False)

    for i in range(len(envs)):
        print(f"\n\nRunning environment {i} with method {method_name}")

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
            except Exception as e:
                print("\n\n\n\n\n\n\n\nPLANNER FAILED\n\n\n\n\n\n\n\n")
                print(e)

                pass

    # loop over all environments
    for i in range(len(envs)):
        print(f"\n\nRunning environment {i} with method {method_name}")

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
            except Exception as e:
                print("\n\n\n\n\n\n\n\nPLANNER FAILED\n\n\n\n\n\n\n\n")
                print(e)
                pass
            # motion_planner.Plan()
            print("Done.")
            print("Travel time: ", motion_planner.GetTravelTime())
            print("Total computation time: ", motion_planner.GetTotalComputationTime())
            print("Solver time: ", motion_planner.GetSolverTime())

            # store results
            results[method_name]["Tf"].append(motion_planner.GetTravelTime())
            results[method_name]["t_comp_total"].append(motion_planner.GetTotalComputationTime())
            results[method_name]["t_comp_solver"].append(motion_planner.GetSolverTime())
            results[method_name]["corridor_infeasibilities_detected"].append(
                motion_planner.CorridorInfeasibilitiesDetected())
            if motion_planner.GetTotalComputationTime() < 0 or motion_planner.GetSolverTime() < 0:
                failures.append(i)
            else:
                avg_travel_time += motion_planner.GetTravelTime()
            avg_solver_time += motion_planner.GetSolverTime()

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


            
# print(len(expected_failures))
# print(expected_failures)
            
# print(f"Failures: {failures}")
# avg_solver_time /= len(envs)
# print(f"Average solver time: {avg_solver_time}")
# avg_travel_time /= (len(envs) - len(failures))
# print(f"Average travel time: {avg_travel_time}")

# store results as a json
import json
with open('python-benchmark/files/results_new_new' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)