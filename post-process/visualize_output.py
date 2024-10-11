import matplotlib.pyplot as plt
from visualization_helpers import *

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
    show_environment(env)

    # plot corridors
    show_corridors(corridors)
        
    # plot waypoints
    for i in range(len(trajectories)):
        if planner_methods[i] == "ARENA":
            show_waypoints(parametrizations[i])
                           
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
    plt.savefig(fig_folder + 'traj.png', dpi=300)


    ### plot positions ###
    fig, axs = plt.subplots(2, 1)
    for i in range(len(trajectories)):
        axs[0].plot(trajectories[i]["t"], trajectories[i]["px"], 'o-', 
                    color=colors[i], markersize=1)
        axs[1].plot(trajectories[i]["t"], trajectories[i]["py"], 'o-', 
                    color=colors[i], markersize=1)
    
    if first_arena_idx is not None:
        t = 0
        axs[0].axvline(t, linewidth=1, color='k')
        axs[1].axvline(t, linewidth=1, color='k')
        for w in range(corridors['nb_of_corridors']+1):
            if w < corridors['nb_of_corridors']:
                axs[0].fill([t, t+sum(parametrizations[first_arena_idx]["t_x_sol"][w]),
                                t+sum(parametrizations[first_arena_idx]["t_x_sol"][w]), t],
                            [corridors['sequence'][w]['x_min'], 
                                corridors['sequence'][w]['x_min'],
                                corridors['sequence'][w]['x_max'],
                                corridors['sequence'][w]['x_max']],
                            color='green', alpha=0.2)
                axs[1].fill([t, t+sum(parametrizations[first_arena_idx]["t_y_sol"][w]),
                                t+sum(parametrizations[first_arena_idx]["t_y_sol"][w]), t],
                            [corridors['sequence'][w]['y_min'], 
                                corridors['sequence'][w]['y_min'],
                                corridors['sequence'][w]['y_max'],
                                corridors['sequence'][w]['y_max']],
                            color='green', alpha=0.2)
            

            t += sum(parametrizations[first_arena_idx]["t_x_sol"][w])
            axs[0].axvline(t, linewidth=1, color='k')
            axs[1].axvline(t, linewidth=1, color='k')

    axs[0].set_ylabel('px')
    axs[1].set_ylabel('py')
    axs[1].set_xlabel('t')
    plt.suptitle('Position')

    plt.savefig(fig_folder + 'positions.png', dpi=300)


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

    axs[0].fill_between([-10, 1000], [-params["v_max"], -params["v_max"]],
                        [-1000, -1000], color='grey')
    axs[0].fill_between([-10, 1000], [params["v_max"], params["v_max"]],
                        [1000, 1000], color='grey')
    axs[0].set_xlim([0, max([max(trajectories[i]["t"]) for i in range(len(trajectories))])])
    axs[0].set_ylim([-1.1*params["v_max"], 1.1*params["v_max"]])

    axs[1].fill_between([-10, 1000], [-params["v_max"], -params["v_max"]],
                        [-1000, -1000], color='grey')
    axs[1].fill_between([-10, 1000], [params["v_max"], params["v_max"]],
                        [1000, 1000], color='grey')
    axs[1].set_xlim([0, max([max(trajectories[i]["t"]) for i in range(len(trajectories))])])
    axs[1].set_ylim([-1.1*params["v_max"], 1.1*params["v_max"]])
    
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

    axs[0].fill_between([-10, 1000], [-params["a_max"], -params["a_max"]],
                        [-1000, -1000], color='grey')
    axs[0].fill_between([-10, 1000], [params["a_max"], params["a_max"]],
                        [1000, 1000], color='grey')
    axs[0].set_xlim([0, max([max(trajectories[i]["t"]) for i in range(len(trajectories))])])
    axs[0].set_ylim([-1.1*params["a_max"], 1.1*params["a_max"]])

    axs[1].fill_between([-10, 1000], [-params["a_max"], -params["a_max"]],
                        [-1000, -1000], color='grey')
    axs[1].fill_between([-10, 1000], [params["a_max"], params["a_max"]],
                        [1000, 1000], color='grey')
    axs[1].set_xlim([0, max([max(trajectories[i]["t"]) for i in range(len(trajectories))])])
    axs[1].set_ylim([-1.1*params["a_max"], 1.1*params["a_max"]])



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



files = ["output/solution_p2p.json", "output/solution_ocp.json", "output/solution_arena.json"]
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
