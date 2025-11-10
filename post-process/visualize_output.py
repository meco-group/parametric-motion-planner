import matplotlib.pyplot as plt
from visualization_helpers import *
from pathlib import Path

# def latexify():
#     params = {#'backend': 'ps',
#               'axes.labelsize': 15,
#               'axes.titlesize': 15,
#               'legend.fontsize': 15,
#               'xtick.labelsize': 15,
#               'ytick.labelsize': 15,
#               'text.usetex': True,
#               'font.family': 'serif',
#               'figure.figsize': [7,5],
#               'text.latex.preamble': r'\usepackage{bm}',
#               }
 
#     plt.rcParams.update(params)
 
# latexify()

def visualize_output(env, params, corridors, planner_methods, 
                     trajectories, parametrizations=[], **kwargs):
    assert len(planner_methods) == len(trajectories)
    assert len(parametrizations) == len(trajectories)

    path_to_this = Path(__file__).resolve().parent
    fig_folder = str(path_to_this.parent / 'figures' / '')

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
    if "ocp_corridors" in kwargs:
        show_corridors(kwargs["ocp_corridors"])
    show_corridors(corridors, hatch=kwargs.get("hatch", None))
    if kwargs.get("show_original_path", False):
        show_original_path(corridors["original_path"], env["cell_width"], 
                           env["cell_height"], with_numbering=True)
        
    # plot waypoints
    # for i in range(len(trajectories)):
    #     if planner_methods[i] == "ARENA":
    #         show_waypoints(parametrizations[i])
        
    # plot start and dest
    plt.plot(corridors["start"]["x"], corridors["start"]["y"], 'ko', markersize=8)
    plt.plot(corridors["dest"]["x"], corridors["dest"]["y"], 'ko', markersize=8)
                           
    # plot trajectory
    for i in range(len(trajectories)):
        show_trajectory(trajectories[i], colors[i], 
                        planner_methods[i] == "ARENA", params["veh_width"], 
                        params["veh_height"], i == 0)
    
    set_env_plot_limits(env)
    plt.xticks([])
    plt.yticks([])
    plt.tight_layout()
    if "figname_appendix" in kwargs:
        plt.savefig(fig_folder + 'traj_' + str(kwargs["figname_appendix"]) + '.png', dpi=600)
    else:
        plt.savefig(fig_folder + 'traj.png', dpi=600)

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

    ### print computation time and moving time for each method ###
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

    print("\nComparison of results:")
    print(f"{'Method':<10} {'Moving Time [s]':<20} {'Solver Time [ms]':<20} {'Non-Solver Time [ms]':<25} {'Total Computation Time [ms]':<30}")
    for i in range(len(methods)):
        print(f"{methods[i]:<10} {tfs[i]:<20.3f} {solver_times[i]:<20.2f} {non_solver_times[i]:<25.2f} {total_computation_times[i]:<30.2f}")

path_to_this = Path(__file__).resolve().parent
path_to_output_files = path_to_this.parent / 'build' / 'output'
files = [str(path_to_output_files) + '/example_problem_ocp.json', str(path_to_output_files) + '/example_problem_pmp.json']
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
    
visualize_output(envs_list[0], params_list[0], corridors_list[min(len(files)-1,2)], 
                planner_methods_list, trajectories_list, 
                parametrizations_list)
plt.show()
plt.close()