import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')


# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
file_name_appendix = "_large_double"
data = json.load(open('python-benchmark/files/all_trajectories' + file_name_appendix + '.json'))

# print(data.keys())

def show_trajectory(px, py, color='royalblue', alpha=1):
    plt.plot(px, py, color=color, alpha=alpha, linewidth=2)
    # plt.plot([px[0], px[-1]], [py[0], py[-1]], 'ko', alpha=alpha)

def show_environment(env):
    occupancy = env["occupancy_grid"]
    cell_width = env["cell_width"]
    cell_height = env["cell_height"]
    for i in range(len(occupancy)):
        for j in range(len(occupancy[i])):
            if occupancy[i][j] == 1:
                color = 'k'
            elif occupancy[i][j] == 2:
                color = 'firebrick'
            else:
                color = 'white'
            
            plt.gca().add_patch(Rectangle((i*cell_width, j*cell_height), 
                                          cell_width, cell_height, fill=True,
                                          facecolor=color, edgecolor=None,
                                          alpha=0.5))
            
    # # plot a light grid showing the cell
    # for i in range(env["nb_cell_cols"]):
    #     plt.plot([i*cell_width, i*cell_width], 
    #              [0, cell_height*env["nb_cell_rows"]], linewidth=0.1, \
    #              color='k', zorder=1)
    # for j in range(env["nb_cell_rows"]):
    #     plt.plot([0, cell_width*env["nb_cell_cols"]], 
    #              [j*cell_height, j*cell_height], linewidth=0.1, \
    #              color='k', zorder=1)

plt.figure()
for i, px, py, env in zip(range(len(data['px'])), data['px'], data['py'], data['envs']):
    if i < 5:
        show_trajectory(px, py, alpha=0.2)
        env_json = json.loads(env)
        show_environment(env_json)

plt.gca().set_aspect('equal', adjustable='box')
plt.show()
