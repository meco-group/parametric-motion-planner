import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')
from solve_omg_tools import omg_example

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
file_name_appendix = "_large_double_more_obstacles_10"
preparation = json.load(open('python-benchmark/files/omg_tools_preparation' + file_name_appendix + '.json'))


# Benchmark
# loop over all environments
for i in range(len(preparation["corridors"])):
    digit = f'00{i}' if i < 10 else f'0{i}' if i < 100 else f'{i}'
    print(digit)

    ### OMG ###
    solver_time, travel_time = omg_example(
    corridors=preparation["corridors"][i], 
    start=preparation["start"][i], 
    goal=preparation["dest"][i],
    v_max=preparation["vmax"][i],
    a_max=preparation["amax"][i],
    veh_w=preparation["veh_width"][i] + preparation["margin"][i],
    veh_h=preparation["veh_height"][i] + preparation["margin"][i],
    dump_to_json=True, 
    file_name=f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/OMG_{digit}.json',
    start_vel=preparation["start_vel"][i])
