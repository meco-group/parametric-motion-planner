from load_random_environments import extract_data
import sys
from vp_sto_solver import CollisionEnvironment, TrajectorySolver

# Extract the data
file_name_appendix = "_large_double_more_obstacles_10"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# create solver
env = CollisionEnvironment()
solver = TrajectorySolver()

N_via_elements = [5, 10, 15]

expected_failures = []
failures = []
avg_solver_time = 0
avg_travel_time = 0

# Benchmark
for N_via in N_via_elements:
    results = {"Tf": [], "t_comp_total": [], "t_comp_solver": [],
           "corridor_infeasibilities_detected": []}
    for i in range(len(envs)):
        print(f"\n\nRunning environment {i}")

        env.CopyEnv(json.loads(envs[i].ToJson()))
        solver.SetEnvironment(env)
        # solver.VisualizeDistToObstacle()

        print("Planning...")
        print(params[i].GetVmax(), params[i].GetAmax(),
                params[i].GetVehWidth(), params[i].GetVehHeight())
        solver.Plan(starts[i], dests[i], params[i].GetVmax(), 
                    params[i].GetAmax(), params[i].GetVehWidth(), 
                    params[i].GetVehHeight(), N_via)

        print("Done.")

        # store results
        results["Tf"].append(solver.GetTravelTime())
        results["t_comp_total"].append(solver.GetTotalComputationTime())
        results["t_comp_solver"].append(solver.GetSolverTime())
        results["corridor_infeasibilities_detected"].append(not solver.GetFeasible())

    print(f"Average t_comp: {np.mean(results['t_comp_total'])}")
    print(f"Average travel time: {np.mean(results['Tf'])}")

    # store results as a json
    import json
    with open('python-benchmark/files/vp-sto_' + str(N_via) + file_name_appendix + '.json', 'w') as f:
        json.dump(results, f, indent=4)


    # add these results to the file python-benchmark/files.results_new_new_large_double_more_obstacles_10.json
    with open('python-benchmark/files/results_new_new_large_double_more_obstacles_10.json', 'r') as f:
        all_results = json.load(f)

    all_results['vp-sto-' + str(N_via)] = results
    with open('python-benchmark/files/results_new_new_large_double_more_obstacles_10.json', 'w') as f:
        json.dump(all_results, f, indent=4)