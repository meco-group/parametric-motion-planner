import sys
sys.path.append('build/')

import json
import parametric_motion_planner_module as pmp

def extract_data(file_name_appendix):
    # read random environments from file
    with open('python-benchmark/files/random_environments' + file_name_appendix + '.json', 'r') as f:
        data = json.load(f)

    # load random environments
    envs = []
    for env_json in data['envs']:
        env = pmp.Environment(env_json['nb_cell_rows'], env_json['nb_cell_cols'],
                            env_json['cell_width'], env_json['cell_height'])
        for i in range(env_json["nb_cell_rows"]):
            for j in range(env_json["nb_cell_cols"]):
                if env_json["occupancy_grid"][i][j] != 0:
                    env.AddObstacle(pmp.Point2Di(i, j))

        envs.append(env)

    local_env = pmp.Environment(data['envs'][0]['nb_cell_rows'], 
                                data['envs'][0]['nb_cell_cols'],
                                data['envs'][0]['cell_width'], 
                                data['envs'][0]['cell_height'])

    # load random parameters
    params = []
    for params_json in data['params']:
        param = pmp.Parameters(params_json['v_max'], params_json['a_max'],
                            params_json['veh_width'], params_json['veh_height'],
                            params_json['margin'])
        params.append(param)

    local_param = pmp.Parameters(data['params'][0]['v_max'], 
                                 data['params'][0]['a_max'],
                                 data['params'][0]['veh_width'], 
                                 data['params'][0]['veh_height'],
                                 data['params'][0]['margin'])

    # load random start and destination points
    starts = []
    dests = []
    for start, dest in zip(data['starts'], data['dests']):
        starts.append(pmp.Point2Dd(start[0], start[1]))
        dests.append(pmp.Point2Dd(dest[0], dest[1]))

    return envs, params, starts, dests, local_env, local_param

# # show some environments
# import sys
# sys.path.append('post-process/')
# from visualization_helpers import *
# import matplotlib.pyplot as plt

# for i in range(10):
#     env_j = envs[i].ToJson()
#     start = starts[i]
#     dest = dests[i]
#     param = params[i]

#     plt.figure()
#     show_environment(json.loads(env_j))
#     plot_vehicle_footprint(plt.gca(), start.x(), start.y(), param.GetVehWidth(), 
#                            param.GetVehHeight(), virtual_position=False)
#     plot_vehicle_footprint(plt.gca(), dest.x(), dest.y(), param.GetVehWidth(), 
#                            param.GetVehHeight(), virtual_position=True)
    
#     plt.show()