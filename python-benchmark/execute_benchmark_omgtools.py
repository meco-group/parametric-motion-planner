from solve_omg_tools import omg_example
import time
import json

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
# file_name_appendix = "_large_double"
# file_name_appendix = "_large_double_more_obstacles_25"
# file_name_appendix = "_structured_1"
file_name_appendix = "_" + json.loads(open('python-benchmark/benchmark_settings.json').read())["benchmark_name"]

# create containers for results
results = json.load(open('python-benchmark/files/results' + file_name_appendix + '.json'))
results["OmgTools"] = {"Tf": [], "t_comp_total": [], "t_comp_solver": [], 
                       "corridor_infeasibilities_detected": []}

preparation = json.load(open('python-benchmark/files/omg_tools_preparation' + file_name_appendix + '.json'))
travel_times = []

# Benchmark
# loop over all environments
for i in range(len(preparation["corridors"])):
    print(f"\n\nRunning environment {i} with method OmgTools")
    digit = f'00{i}' if i < 10 else f'0{i}' if i < 100 else f'{i}'
    
    a = time.time()
    solver_time, travel_time = omg_example(
        corridors=preparation["corridors"][i], 
        start=preparation["start"][i], 
        goal=preparation["dest"][i],
        v_max=preparation["vmax"][i],
        a_max=preparation["amax"][i],
        veh_w=preparation["veh_width"][i] + preparation["margin"][i],
        veh_h=preparation["veh_height"][i] + preparation["margin"][i],
        dump_to_json=False, 
        file_name=f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/OMG_{digit}.json',
        start_vel=preparation["start_vel"][i])
    b = time.time()
    if travel_time > 20:
        solver_time = -1
    results["OmgTools"]["Tf"].append(travel_time)
    results["OmgTools"]["t_comp_total"].append(preparation["corridor_construction_time"][i] + (b - a) * 1000)
    results["OmgTools"]["t_comp_solver"].append(solver_time * 1000 if solver_time > 0 else -1)
    results["OmgTools"]["corridor_infeasibilities_detected"].append(
        False)
    travel_times.append(travel_time)

travel_times.sort(reverse=True)
print(travel_times)

# store results as a json
import json
with open('python-benchmark/files/results' + file_name_appendix + '.json', 'w') as f:
    json.dump(results, f, indent=4)