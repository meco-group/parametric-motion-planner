import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')
import os

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

from vp_sto_solver import CollisionEnvironment, TrajectorySolver

# Extract the data
file_name_appendix = "_large_double_more_obstacles_10"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

env = CollisionEnvironment()
solver = TrajectorySolver()

N_via_elements = [5, 10, 15]

# Benchmark
# loop over all environments
for N_via in N_via_elements:
    for i in range(len(envs)):
        digit = f'00{i}' if i < 10 else f'0{i}' if i < 100 else f'{i}'
        # check if file already exists
        file_name = f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/vp_sto_{N_via}_{digit}.json'
        if os.path.exists(file_name):
            print(f"File {file_name} already exists, skipping...")
            continue

        env.CopyEnv(json.loads(envs[i].ToJson()))
        solver.SetEnvironment(env)

        solver.Plan(starts[i], dests[i], params[i].GetVmax(), 
            params[i].GetAmax(), params[i].GetVehWidth(), 
            params[i].GetVehHeight(), N_via)

        solver.ToJson(file=f'python-benchmark/benchmark_environments/{file_name_appendix[1:]}/json_files/vp_sto_{N_via}_{digit}.json')