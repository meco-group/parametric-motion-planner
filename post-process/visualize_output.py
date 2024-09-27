import json
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Rectangle, FancyBboxPatch

def load_data(output_file):
    with open(output_file) as f:
        data = json.load(f)

    env = data['Environment']
    params = data['Parameters']
    corridors = data['CorridorSequence']
    planner_method = data['PlannerMethod']
    trajectory = data['Trajectory']

    if planner_method == "ARENA":
        parametrization = data['Parametrization']
        return (env, params, corridors, planner_method, trajectory, parametrization)
    else:
        return (env, params, corridors, planner_method, trajectory, None)

def plot_vehicle_footprint(ax, px, py, params, virtual_position=False):
    veh_width = params["veh_width"]
    veh_height = params["veh_height"]
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
                            edgecolor='k', linestyle=linestyle, alpha=alpha)
    ax.add_patch(rect)
    rect = FancyBboxPatch(anchor_inner, width_inner, height_inner, 
                            boxstyle=boxstyle_inner, fill=True, 
                            facecolor=color_inner, edgecolor=color_inner, 
                            alpha=alpha_inner)
    ax.add_patch(rect)

def visualize_output(env, params, corridors, planner_methods, 
                     trajectories, parametrizations=[]):
    assert len(planner_methods) == len(trajectories)
    assert len(parametrizations) == len(trajectories)

    # fig_folder = 'post-process/figures/'
    fig_folder = '../post-process/figures/'

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
    plt.figure()

    # show environment
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
    for i in range(env["nb_cell_cols"]):
        plt.plot([i*cell_width, i*cell_width], 
                 [0, cell_height*env["nb_cell_rows"]], linewidth=0.1, \
                 color='k', zorder=1)
    for j in range(env["nb_cell_rows"]):
        plt.plot([0, cell_width*env["nb_cell_cols"]], 
                 [j*cell_height, j*cell_height], linewidth=0.1, \
                 color='k', zorder=1)

    # plot corridors
    for c in corridors["sequence"]:
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=True, facecolor="green", alpha=0.2, 
                            edgecolor=None))
        plt.gca().add_patch(Rectangle((c["x_min"], c["y_min"]), 
                                      c["x_max"]-c["x_min"], 
                                      c["y_max"]-c["y_min"], 
                            fill=False, edgecolor='green', linewidth=1))
        
    # plot waypoints
    for i in range(len(trajectories)):
        if planner_methods[i] == "ARENA":
            for w in range(0, parametrizations[i]["nb_corridors"] + 1):
                plt.plot([parametrizations[i]["waypoints"][w]["x"]], 
                         [parametrizations[i]["waypoints"][w]["y"]], 'ok', alpha=0.1)
                plt.plot([parametrizations[i]["waypoints_sol"][w]["x"]], 
                         [parametrizations[i]["waypoints_sol"][w]["y"]], 'ok')
                
    # show vehicle footprint
    plot_vehicle_footprint(plt.gca(), trajectories[0]["px"][0], 
                           trajectories[0]["py"][0], params, 
                           virtual_position=False)
    plot_vehicle_footprint(plt.gca(), trajectories[0]["px"][-1], 
                           trajectories[0]["py"][-1], params, 
                           virtual_position=True)
            
    # plot trajectory
    for i in range(len(trajectories)):
        plt.plot(trajectories[i]["px"], trajectories[i]["py"], 'o-', 
                 color=colors[i], markersize=1)

    plt.xlim([0, cell_width*env["nb_cell_cols"]])
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(2*cell_width))
    plt.ylim([0, cell_height*env["nb_cell_rows"]])
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(2*cell_height))
    plt.gca().set_aspect('equal',adjustable='box')
    plt.savefig(fig_folder + 'traj.png', dpi=300)

    ### plot velocity ###
    fig, axs = plt.subplots(2, 1)
    for i in range(len(trajectories)):
        axs[0].plot(trajectories[i]["t"], trajectories[i]["vx"], 'o-', 
                    color=colors[i], markersize=1)
        axs[1].plot(trajectories[i]["t"], trajectories[i]["vy"], 'o-', 
                    color=colors[i], markersize=1)
        

    if first_arena_idx is not None:
        t = 0
        axs[0].axvline(t, linewidth=1, color='k')
        axs[1].axvline(t, linewidth=1, color='k')
        for w in range(corridors['nb_of_corridors']+1):
            t += sum(parametrizations[first_arena_idx]["t_x_sol"][w])
            axs[0].axvline(t, linewidth=1, color='k')
            axs[1].axvline(t, linewidth=1, color='k')

    axs[0].set_ylabel('vx')
    axs[1].set_ylabel('vy')
    axs[1].set_xlabel('t')
    plt.suptitle('Velocity')

    plt.savefig(fig_folder + 'velocities.png', dpi=300)

    ### plot controls ###
    fig, axs = plt.subplots(2, 1)
    for i in range(len(trajectories)):
        axs[0].plot(trajectories[i]["t"], trajectories[i]["ax"], 'o-', 
                    color=colors[i], markersize=1)
        axs[1].plot(trajectories[i]["t"], trajectories[i]["ay"], 'o-', 
                    color=colors[i], markersize=1)
        
    if first_arena_idx is not None:
        t = 0
        axs[0].axvline(t, linewidth=1, color='k')
        axs[1].axvline(t, linewidth=1, color='k')
        for w in range(corridors['nb_of_corridors']+1):
            t += sum(parametrizations[first_arena_idx]["t_x_sol"][w])
            axs[0].axvline(t, linewidth=1, color='k')
            axs[1].axvline(t, linewidth=1, color='k')

    plt.suptitle('Controls')

    axs[0].set_ylabel('ax')
    axs[1].set_ylabel('ay')
    axs[1].set_xlabel('t')

    plt.savefig(fig_folder + 'controls.png', dpi=300)

    ### plot computation time and moving time ###
    fig, ax1 = plt.subplots()
    color_moving_time = 'chocolate'
    color_solver_time = 'royalblue'

    # Bar width
    bar_width = 0.4

    # Plot Tf on ax1
    methods = []
    x = []
    tfs = []
    total_computation_times = []
    solver_times = []
    non_solver_times = []
    for i in range(len(trajectories)):
        methods.append(planner_methods[i])
        x.append(i)
        tfs.append(trajectories[i]['Tf'])
        solver_times.append(trajectories[i]['solver_time'])
        total_computation_times.append(trajectories[i]['total_computation_time'])
        non_solver_times.append(total_computation_times[i] - solver_times[i])
    
    ax1.bar([i - bar_width/2 for i in x], tfs, bar_width, 
            label='Moving Time [s]', color=color_moving_time)
    
    # add bar value on top of each bar
    for i in range(len(x)):
        ax1.text(x[i] - bar_width/2, tfs[i] + 0.0, f'{tfs[i]:.3f}', 
                 ha='center', va='bottom', color='black')

    ax1.set_ylabel('Moving Time [s]', color=color_moving_time)
    ax1.tick_params(axis='y', labelcolor=color_moving_time)
    ax1.spines['left'].set_color(color_moving_time)

    ax1.set_xticks(x)
    ax1.set_xticklabels(methods)
    handles1, labels1 = ax1.get_legend_handles_labels()


    # Plot solver_time and total_computation_time on ax2
    ax2 = ax1.twinx()
    ax2.bar([i + bar_width/2 for i in x], solver_times, bar_width, 
            label='Solver Time [ms]', color=color_solver_time)
    ax2.bar([i + bar_width/2 for i in x], non_solver_times, bar_width,
            bottom=solver_times, label='Non-Solver Time [ms]', color=color_solver_time, alpha=0.5)

    # add bar value on top of each bar
    for i in range(len(x)):
        ax2.text(x[i] + bar_width/2, solver_times[i] + 0.0, f'{solver_times[i]:.2f}', 
                 ha='center', va='bottom', color='black')
        ax2.text(x[i] + bar_width/2, total_computation_times[i] + 13, f'{total_computation_times[i]:.2f}', 
                 ha='center', va='top', color='black')

    ax2.set_ylabel('Computation Time [ms]', color=color_solver_time)
    ax2.tick_params(axis='y', labelcolor=color_solver_time)
    ax2.spines['right'].set_color(color_solver_time)

    ax2.set_xticks(x)
    ax2.set_xticklabels(methods)

    handles2, labels2 = ax2.get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels)
    plt.savefig(fig_folder + '/timings.png', dpi=300)



files = ["output/solution_arena.json", "output/solution_ocp.json", "output/solution_p2p.json"]
# files = ["build/output/solution_arena.json", "build/output/solution_ocp.json"]
# files = ["output/solution_ocp.json"]
# files = ["output/solution_arena.json"]

envs_list = []
params_list = []
corridors_list = []
planner_methods_list = []
trajectories_list = []
parametrizations_list = []
for output_file in files:
    env, params, corridors, planner_method, trajectory, parametrization = load_data(output_file)
    envs_list.append(env)
    params_list.append(params)
    corridors_list.append(corridors)
    planner_methods_list.append(planner_method)
    trajectories_list.append(trajectory)
    parametrizations_list.append(parametrization)

visualize_output(envs_list[0], params_list[0], corridors_list[0], 
                 planner_methods_list, trajectories_list, 
                 parametrizations_list)
