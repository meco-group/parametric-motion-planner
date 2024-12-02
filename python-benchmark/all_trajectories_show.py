import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
import sys
import shapely.geometry as sg
import shapely.ops as so
import matplotlib.ticker as ticker
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp

def plot_vehicle_footprint(ax, px, py, veh_width, veh_height, virtual_position=False):
    alpha = 1.0 if not virtual_position else 1.0
    anchor = (px-veh_width/2, py-veh_height/2)
    width = veh_width
    height = veh_height
    boxstyle = "round,pad=0.0, rounding_size=0.015"
    color = 'black' if not virtual_position else 'lightgray'
    linestyle = '-' if not virtual_position else '--'

    inner_factor = 0.8
    alpha_inner = 1.0 if not virtual_position else 1.0
    anchor_inner = (px-inner_factor*veh_width/2, 
                    py-inner_factor*veh_height/2)
    width_inner = inner_factor*veh_width
    height_inner = inner_factor*veh_height
    boxstyle_inner = "round,pad=0.0, rounding_size=0.002"
    color_inner = 'gainsboro' if not virtual_position else 'whitesmoke'

    # create a fancybox with rounded corners
    rect = FancyBboxPatch(anchor, width, height, boxstyle=boxstyle, 
                            fill=True, facecolor=color, 
                            edgecolor='k', linestyle=linestyle, alpha=alpha, zorder=11)
    ax.add_patch(rect)
    rect = FancyBboxPatch(anchor_inner, width_inner, height_inner, 
                            boxstyle=boxstyle_inner, fill=True, 
                            facecolor=color_inner, edgecolor=color_inner, 
                            alpha=alpha_inner, zorder=11)
    ax.add_patch(rect)
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
                                          facecolor=color, edgecolor=None))
            
    # plot a light grid showing the cell
    # for i in range(env["nb_cell_cols"]+1):
    #     plt.plot([i*cell_width, i*cell_width], 
    #              [0, cell_height*env["nb_cell_rows"]], linewidth=0.1, \
    #              color='gray', zorder=0)
    # for j in range(env["nb_cell_rows"]+1):
    #     plt.plot([0, cell_width*env["nb_cell_cols"]], 
    #              [j*cell_height, j*cell_height], linewidth=0.1, \
    #              color='gray', zorder=0)
        
def set_env_plot_limits(env):
    cell_width = env["cell_width"]
    cell_height = env["cell_height"]
    plt.xlim([0, cell_width*env["nb_cell_cols"]])
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(2*cell_width))
    plt.ylim([0, cell_height*env["nb_cell_rows"]])
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(2*cell_height))
    plt.gca().set_aspect('equal',adjustable='box')

def show_corridors(corridors, color='green', max_alpha=1, clip=False):
    for c in corridors["sequence"]:
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=True, facecolor=color, alpha=0.2*max_alpha, 
                            edgecolor=None, clip_on=clip))
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=False, edgecolor=color, linewidth=1, 
                            clip_on=clip))
        
def show_waypoints(parametrization):
    for w in range(0, parametrization["nb_corridors"] + 1):
        plt.plot([parametrization["waypoints"][w]["x"]], 
                    [parametrization["waypoints"][w]["y"]], 'ok', alpha=0.2,
                    zorder=13)
        plt.plot([parametrization["waypoints_sol"][w]["x"]], 
                    [parametrization["waypoints_sol"][w]["y"]], 'ok',
                    zorder=13,
                    markeredgecolor='white', markersize=7)

def show_trajectory(trajectory, color, with_trace=False, width=0, height=0, 
                    with_footprints=False, nb_samples_to_show=-1,
                    virtual_initial_footprint=False,
                    virtual_final_footprint=True,
                    show_markers=True, linewidth=1, with_line=True):
    if nb_samples_to_show == -1:
        nb_samples_to_show = len(trajectory["px"])

    if with_trace:
        footprints = []
        for j in range(nb_samples_to_show-1):
            px = trajectory["px"][j]
            py = trajectory["py"][j]
            px_next = trajectory["px"][j+1]
            py_next = trajectory["py"][j+1]

            for k in range(0, 100, 10):
                px = px + k/100*(px_next - px)
                py = py + k/100*(py_next - py)            
                footprint = sg.box(px - width/2, 
                                py - height/2,
                                px + width/2, 
                                py + height/2)
                footprints.append(footprint)
        footprint_trace = so.unary_union(footprints)
        try:
            x, y = footprint_trace.exterior.xy
            plt.gca().fill(x, y, color=color, alpha=0.2, edgecolor='none', zorder=10)
            # plt.gca().fill(x, y, color='none', alpha=0.5, edgecolor=colors[i])
            # plt.plot(x, y, color=colors[i], linewidth=1)
        except:
            print("No footprint to plot")

    if show_markers and with_line:
        plt.plot(trajectory["px"][:nb_samples_to_show], 
                trajectory["py"][:nb_samples_to_show], 'o-', color=color, 
                markersize=1, linewidth=linewidth, zorder=12)
    elif show_markers:
        plt.plot(trajectory["px"][:nb_samples_to_show], 
                trajectory["py"][:nb_samples_to_show], 'o', color=color, 
                markersize=1, linewidth=linewidth, zorder=12)
    elif with_line:
        plt.plot(trajectory["px"][:nb_samples_to_show], 
                trajectory["py"][:nb_samples_to_show], '-', color=color, 
                linewidth=linewidth, zorder=12)
    
    if with_footprints:
        # show vehicle footprint
        plot_vehicle_footprint(plt.gca(), trajectory["px"][0], 
                               trajectory["py"][0], width, height, 
                               virtual_position=virtual_initial_footprint)
        final_ind = min(nb_samples_to_show, len(trajectory["px"])-1)
        plot_vehicle_footprint(plt.gca(), trajectory["px"][final_ind], 
                               trajectory["py"][final_ind], width, height,
                               virtual_position=virtual_final_footprint)
        
def visualize_output(env, params, corridors, planner_methods, trajectories, fig_nb, parametrization=None):
    filtered_list = [45, 79, 104, 387]
    suboptimal_list = [108, 416, 203, 172]
    suboptimal_zoomboxes = {108: [0.92, 1.901, 0.826, 1.794],
                            172: [0.612, 1.487, 0.706, 1.581],
                            203: [0.341, 1.348, 0.536, 1.384],
                            416: [1.277, 1.645, 1.058, 1.420]}

    # modify the zoomboxes minimally to make them square
    for key in suboptimal_zoomboxes:
        x_min, x_max, y_min, y_max = suboptimal_zoomboxes[key]
        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        x_diff = x_max - x_min
        y_diff = y_max - y_min
        if x_diff > y_diff:
            suboptimal_zoomboxes[key][2] = y_center - x_diff / 2
            suboptimal_zoomboxes[key][3] = y_center + x_diff / 2
        else:
            suboptimal_zoomboxes[key][0] = x_center - y_diff / 2
            suboptimal_zoomboxes[key][1] = x_center + y_diff

    if fig_nb not in filtered_list and fig_nb not in suboptimal_list:
        return

    # fig_folder = 'post-process/figures/'
    fig_folder = 'python-benchmark/benchmark_environments/large_double/figures/'

    first_arena_idx = 0
    while planner_methods[first_arena_idx] != "ARENA":
        first_arena_idx += 1

        if first_arena_idx >= len(planner_methods):
            first_arena_idx = None
            break

    colors = []
    for i in range(len(trajectories)):
        if planner_methods[i] == "P2P":
            colors.append('orange')
        elif planner_methods[i] == "OCP":
            colors.append('r')
        elif planner_methods[i] == "ARENA":
            colors.append('b')
        else:
            colors.append('k')

    ### plot trajectory ###
    plt.figure(figsize=(4,4))

    # show environment
    show_environment(env)

    # plot corridors
    show_corridors(corridors, clip=i not in suboptimal_list)
                           
    # plot trajectory
    for i in range(len(trajectories)):
        show_trajectory(trajectories[i], colors[i], 
                        planner_methods[i] == "ARENA", params["veh_width"], 
                        params["veh_height"], i == 0)

 
    # pts = [0.511041, 0.172393, 0.511041, 0.177022, 0.54149, 0.1785, 0.84088, 0.178954, 0.953821, 0.155781, 1.01849, 0.1815, 1.24629, 0.181509, 1.25851, 0.30149, 1.25851, 0.30149, 1.29906, 0.3015, 1.29906, 0.3015, 1.29937, 0.336515]
    # pts_x = [pts[2*i] for i in range(len(pts)//2)]
    # pts_y = [pts[2*i+1] for i in range(len(pts)//2)]
    # plt.scatter(pts_x, pts_y)
    
    set_env_plot_limits(env)

    plt.tight_layout()

    plt.savefig(fig_folder + f'traj_{fig_nb:03d}.png', dpi=300)
    
    plt.xticks([])
    plt.yticks([])
    plt.tight_layout()
    
    # remove axes box
    # plt.gca().spines['top'].set_visible(False)
    # plt.gca().spines['bottom'].set_visible(False)
    # plt.gca().spines['left'].set_visible(False)
    # plt.gca().spines['right'].set_visible(False)
    
    if fig_nb in filtered_list:
        plt.savefig(fig_folder[:-1] + f'_filtered/traj_{fig_nb:03d}.png', dpi=300)
        plt.savefig(fig_folder[:-1] + f'_filtered/traj_{fig_nb:03d}.pdf')
    if fig_nb in suboptimal_list:
        show_waypoints(parametrization)
        plt.xlim(suboptimal_zoomboxes[fig_nb][:2])
        plt.ylim(suboptimal_zoomboxes[fig_nb][2:])
        plt.savefig(fig_folder[:-1] + f'_suboptimal/traj_{fig_nb:03d}.png', dpi=300)
        plt.savefig(fig_folder[:-1] + f'_suboptimal/traj_{fig_nb:03d}.pdf')
        # plt.show()

    plt.close()

###############################################################################
###############################################################################
###############################################################################
###############################################################################



N = 500

for i in range(N):
    print(f"{100.0*i/(N-1):.2f}% completion")
    digit = f'00{i}' if i < 10 else f'0{i}' if i < 100 else f'{i}'
    file_prefix = f"python-benchmark/benchmark_environments/large_double/json_files/"
    file_appendix = f"_{digit}.json"
    files = [file_prefix + f"P2P{file_appendix}", 
             file_prefix + f"OMG{file_appendix}", 
             file_prefix + f"OCP{file_appendix}", 
             file_prefix + f"ARENA{file_appendix}"]
             
    data = []
    for output_file in files:
        with open(output_file) as f:
            data.append(json.load(f))
    
    env = data[-1]["environment"]
    params = data[-1]["parameters"]
    corridors = data[-1]["corridor_sequence"]
    planner_methods_list = ["P2P", "OMG", "OCP", "ARENA"]
    trajectories_list = [data[i]["trajectory"] for i in range(len(files))]

    actually_used_methods = [0, 1, 2, 3]
    planner_methods_list = [pm for i, pm in enumerate(planner_methods_list) if i in actually_used_methods]
    trajectories_list = [tr for i, tr in enumerate(trajectories_list) if i in actually_used_methods]

    # if P2P is shown, cut the trajectory at the destination
    if 0 in actually_used_methods:
        dest = data[-1]["corridor_sequence"]["dest"]
        dist = 1.0e10
        for j in range(len(trajectories_list[0]["px"])-1, -1, -1):
            new_dist = (trajectories_list[0]["px"][j] - dest["x"])**2 + \
                       (trajectories_list[0]["py"][j] - dest["y"])**2
            if new_dist > dist:
                trajectories_list[0]["px"] = trajectories_list[0]["px"][:j+1]
                trajectories_list[0]["py"] = trajectories_list[0]["py"][:j+1]
                break
            else:
                dist = new_dist

    visualize_output(env, params, corridors, planner_methods_list, 
                     trajectories_list, i, data[-1]["parametrization"])
