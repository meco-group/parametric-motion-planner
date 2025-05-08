import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
import shapely.geometry as sg
import shapely.ops as so
import matplotlib.ticker as ticker

def load_data(output_file):
    with open(output_file) as f:
        data = json.load(f)

    env = data['environment']
    params = data['parameters']
    corridors = data['corridor_sequence']
    planner_method = data['planner_method']
    trajectory = data['trajectory']

    if planner_method == "ARENA":
        parametrization = data['parametrization']
        return (env, params, corridors, planner_method, trajectory, parametrization)
    else:
        return (env, params, corridors, planner_method, trajectory, None)

def show_environment(env, obstacle_color='firebrick', **kwargs):
    occupancy = env["occupancy_grid"]
    cell_width = env["cell_width"]
    cell_height = env["cell_height"]
    for i in range(len(occupancy)):
        for j in range(len(occupancy[i])):
            if occupancy[i][j] == 1:
                color = 'k'
            elif occupancy[i][j] == 2:
                color = 'firebrick'
            elif occupancy[i][j] == 5:
                color = obstacle_color
            else:
                color = 'white'

            # check if user set the obstacles_only option
            obstacles_only = kwargs.get("obstacles_only", False)
            obstacles_alpha = kwargs.get("obstacles_alpha", 1.0)
            if occupancy[i][j] != 5 and occupancy[i][j] != 2:
                obstacles_alpha = 1.0
            if not obstacles_only or occupancy[i][j] == 2 or occupancy[i][j] == 5:
                plt.gca().add_patch(Rectangle((i*cell_width, j*cell_height), 
                                            cell_width, cell_height, fill=True,
                                            facecolor=color, edgecolor=None,
                                            alpha=obstacles_alpha))
            
    # plot a light grid showing the cell
    show_light_grid = kwargs.get('show_light_grid', False)
    if show_light_grid:
        for i in range(env["nb_cell_cols"]+1):
            plt.plot([i*cell_width, i*cell_width], 
                    [0, cell_height*env["nb_cell_rows"]], linewidth=0.1, \
                    color='gray', zorder=1)
        for j in range(env["nb_cell_rows"]+1):
            plt.plot([0, cell_width*env["nb_cell_cols"]], 
                    [j*cell_height, j*cell_height], linewidth=0.1, \
                    color='gray', zorder=1)
        
def show_original_path(path, cell_width, cell_height, **kwargs):
    for i in range(len(path)):
        plt.gca().add_patch(
            Rectangle((path[i]["x"]*cell_width, path[i]["y"]*cell_height),
                      cell_width, cell_height, fill=True, 
                      facecolor='gray', edgecolor='gray', alpha=0.3))
        plt.gca().add_patch(
            Rectangle((path[i]["x"]*cell_width, path[i]["y"]*cell_height),
                      cell_width, cell_height, fill=False, 
                      edgecolor='k', alpha=1.0))
        with_numbering = kwargs.get("with_numbering", False)
        if with_numbering:
            plt.text(path[i]["x"]*cell_width + cell_width/2, 
                    path[i]["y"]*cell_height + cell_height/2, 
                    str(i), fontsize=8, ha='center', va='center')
        
def set_env_plot_limits(env):
    cell_width = env["cell_width"]
    cell_height = env["cell_height"]
    plt.xlim([0, cell_width*env["nb_cell_cols"]])
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(1*cell_width))
    plt.ylim([0, cell_height*env["nb_cell_rows"]])
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(1*cell_height))
    plt.gca().set_aspect('equal',adjustable='box')

def show_corridors(corridors, color='green', max_alpha=1, clip_on=False, **kwargs):
    hatch = kwargs.get("hatch", None)
    for c in corridors["sequence"]:
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=True, facecolor=color, alpha=0.2*max_alpha, 
                            edgecolor=None, clip_on=clip_on))
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=False, edgecolor=color, linewidth=1,
                            clip_on=clip_on, hatch=hatch))
        
def show_waypoints(parametrization):
    for w in range(0, parametrization["nb_corridors"] + 1):
        plt.plot([parametrization["waypoints"][w]["x"]], 
                    [parametrization["waypoints"][w]["y"]], 'ok', alpha=0.1, zorder=3)
        plt.plot([parametrization["waypoints_sol"][w]["x"]], 
                    [parametrization["waypoints_sol"][w]["y"]], 'ok', zorder=3)
        
        # plot alpha values
        s = 0.02
        plt.arrow(parametrization["waypoints"][w]["x"],
                    parametrization["waypoints"][w]["y"],
                    s*parametrization["alpha_x"][w],
                    s*parametrization["alpha_y"][w],
                    head_width=0.5*s, head_length=0.5*s, fc='k', ec='k', zorder=4)


def show_trajectory(trajectory, color, with_trace=False, width=0, height=0, 
                    with_footprints=False, nb_samples_to_show=-1,
                    virtual_initial_footprint=False,
                    virtual_final_footprint=True,
                    show_markers=True, linewidth=1, with_line=True,
                    unfaded_nb_samples=-1,
                    show_initial_footprint_if_showing_footprints=True,
                    show_final_footprint_if_showing_footprints=True, **kwargs):
    if nb_samples_to_show == -1:
        nb_samples_to_show = len(trajectory["px"])

    if unfaded_nb_samples == -1:
        unfaded_nb_samples = nb_samples_to_show
        start_index = 0
    else:
        start_index = max(0, nb_samples_to_show - unfaded_nb_samples)

    if with_trace:
        footprints = []
        for j in range(start_index, nb_samples_to_show-1):
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
            trace_alpha = kwargs.get("trace_alpha", 0.2)
            plt.gca().fill(x, y, color=color, alpha=0.2, edgecolor='none', zorder=1)
            # plt.gca().fill(x, y, color='none', alpha=0.5, edgecolor=color)
            # plt.plot(x, y, color=colors[i], linewidth=1)
        except:
            print("No footprint to plot")

    if show_markers and with_line:
        plt.plot(trajectory["px"][start_index:nb_samples_to_show], 
                trajectory["py"][start_index:nb_samples_to_show], 'o-', color=color, 
                markersize=1, linewidth=linewidth, zorder=3)
    elif show_markers:
        plt.plot(trajectory["px"][start_index:nb_samples_to_show], 
                trajectory["py"][start_index:nb_samples_to_show], 'o', color=color, 
                markersize=1, linewidth=0, zorder=3)
    elif with_line:
        plt.plot(trajectory["px"][start_index:nb_samples_to_show], 
                trajectory["py"][start_index:nb_samples_to_show], '-', color=color, 
                linewidth=linewidth, zorder=3)
    
    if with_footprints:
        # show vehicle footprint
        if show_initial_footprint_if_showing_footprints:
            plot_vehicle_footprint(plt.gca(), trajectory["px"][start_index], 
                                trajectory["py"][start_index], width, height, 
                                virtual_position=virtual_initial_footprint)
        if show_final_footprint_if_showing_footprints:
            final_ind = min(nb_samples_to_show, len(trajectory["px"])-1)
            plot_vehicle_footprint(plt.gca(), trajectory["px"][final_ind], 
                                trajectory["py"][final_ind], width, height,
                                virtual_position=virtual_final_footprint)
        
def show_moving_obstacle(obstacle, sample_idx, color):
    x = obstacle["travelled_trajectory"]["px"][sample_idx]
    y = obstacle["travelled_trajectory"]["py"][sample_idx]
    width = obstacle["width"]
    height = obstacle["height"]
    plt.gca().add_patch(Rectangle((x-width/2, y-height/2),
                                    width, height, fill=True, 
                                    facecolor=color, edgecolor=None))

def plot_vehicle_footprint(ax, px, py, veh_width, veh_height, virtual_position=False, **kwargs):
    max_alpha = kwargs.get("max_alpha", 1.0)
    zorder = kwargs.get("zorder", 2)

    alpha = 1.0*max_alpha if not virtual_position else 1.0*max_alpha
    anchor = (px-veh_width/2, py-veh_height/2)
    width = veh_width
    height = veh_height
    boxstyle = "round,pad=0.0, rounding_size=0.015"
    color = 'black' if not virtual_position else 'lightgray'
    linestyle = '-' if not virtual_position else '--'

    inner_factor = 0.8
    alpha_inner = 1.0*max_alpha if not virtual_position else 1.0*max_alpha
    anchor_inner = (px-inner_factor*veh_width/2, 
                    py-inner_factor*veh_height/2)
    width_inner = inner_factor*veh_width
    height_inner = inner_factor*veh_height
    boxstyle_inner = "round,pad=0.0, rounding_size=0.002"
    color_inner = kwargs.get("color", 'gainsboro' if not virtual_position else 'whitesmoke')

    # create a fancybox with rounded corners
    rect = FancyBboxPatch(anchor, width, height, boxstyle=boxstyle, 
                            fill=True, facecolor=color, 
                            edgecolor='k', linestyle=linestyle, alpha=alpha, zorder=zorder)
    ax.add_patch(rect)
    rect = FancyBboxPatch(anchor_inner, width_inner, height_inner, 
                            boxstyle=boxstyle_inner, fill=True, 
                            facecolor=color_inner, edgecolor=color_inner, 
                            alpha=alpha_inner, zorder=zorder)
    ax.add_patch(rect)