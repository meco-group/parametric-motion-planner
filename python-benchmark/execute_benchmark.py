import sys
sys.path.append('build/')
import parametric_motion_planner_module as pmp
from load_random_environments import extract_data


# Extract the data
envs, params, starts, dests = extract_data()

# List all methods to benchmark
methods = [pmp.PlannerMethod.ARENA, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP, 
           pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.OCP,
           pmp.PlannerMethod.P2P]
method_names = ["ARENA", "OCP-5", "OCP-10", "OCP-20", "OCP-30", "OCP-40", "P2P"]
assert len(methods) == len(method_names)

# Create motion planner
motion_planner = pmp.MotionPlanner(methods[0], params[0], envs[0])

# create containers for results
results = {}
for m in method_names:
    results[m] = {"Tf": [], "t_comp_total": [], "t_comp_solver": []}

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
        motion_planner.SetStart(starts[i])
        motion_planner.SetDest(dests[i])
        if i > 0:
            my_v_max = params[i].GetVmax()
            print(f"v_max: {my_v_max}")
            params[0].SetVmax(my_v_max)
            params[0].SetVmax(params[i].GetVmax())
            params[0].SetAmax(params[i].GetAmax())
            params[0].SetVehWidth(params[i].GetVehWidth())
            params[0].SetVehHeight(params[i].GetVehHeight())
            params[0].SetMargin(params[i].GetMargin())
            envs[0].CopyObstacles(envs[i])
        
        motion_planner.Plan()

        # store results
        results[method_name]["Tf"].append(motion_planner.GetTravelTime())
        results[method_name]["t_comp_total"].append(motion_planner.GetTotalComputationTime())
        results[method_name]["t_comp_solver"].append(motion_planner.GetSolverTime())

# store results as a json
import json
with open('python-benchmark/files/results.json', 'w') as f:
    json.dump(results, f, indent=4)