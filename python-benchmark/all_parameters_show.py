import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

# Extract the data
# file_name_appendix = ""
file_name_appendix = "_cell"
# file_name_appendix = "_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)
results = json.load(open('python-benchmark/files/results' + file_name_appendix + '.json'))

# loop over all parameters
data = {"Vmax": [], "Amax": [], "speedup": []}

for i in range(len(envs)):
    data["Vmax"].append(params[i].GetVmax())
    data["Amax"].append(params[i].GetAmax())
    if results["ARENA"]["t_comp_solver"][i] == 0:
        data["speedup"].append(25)
    else:
        data["speedup"].append(results["OCP-30"]["t_comp_solver"][i] / results["ARENA"]["t_comp_solver"][i])



# make a scatter plot with speedup as color
import matplotlib.pyplot as plt
plt.figure()
plt.scatter(data["Vmax"], data["Amax"], c=data["speedup"], cmap='viridis')
plt.colorbar()
plt.xlabel("Vmax")
plt.ylabel("Amax")

plt.show()