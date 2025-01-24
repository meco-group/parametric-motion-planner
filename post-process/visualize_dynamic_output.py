import matplotlib.pyplot as plt
from visualization_helpers import *
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from PIL import Image
import io

def latexify():
    params = {#'backend': 'ps',
              'axes.labelsize': 15,
              'axes.titlesize': 15,
              'legend.fontsize': 15,
              'xtick.labelsize': 15,
              'ytick.labelsize': 15,
              'text.usetex': True,
              'font.family': 'serif',
              'figure.figsize': [7,5],
            #   'text.latex.preamble': [r'\usepackage{bm}'],
              }
 
    plt.rcParams.update(params)
 
latexify()

def visualize_real_time_solution(data, T=None, fig=None, clean=False):
    assert "duration_of_request_since_first_sample_in_ms" in data

    # T: time since the mover started moving

    original_T = T

    if T == None or T > data["travelled_trajectory"]["Tf"] + 0*data["travelled_trajectory"]["dt"]:
        T = data["travelled_trajectory"]["Tf"] + 0*data["travelled_trajectory"]["dt"]
    traj_idx = 0
    while len(data["replanning_times"]) > traj_idx and T >= data["replanning_times"][traj_idx]:
        traj_idx += 1
    local_replanning_times = [0] + data["replanning_times"]
    number_of_samples_read_by_mover = int(min(T/data["travelled_trajectory"]["dt"],
                                       data["travelled_trajectory"]["nb_samples"]))
    
    number_of_samples_provided_to_mover = 0
    while (T > data["duration_of_request_since_first_sample_in_ms"][number_of_samples_provided_to_mover]/1000):
        number_of_samples_provided_to_mover += 1

    plt.figure(fig.number)
    plt.clf()

    env = data["previous_environments"][traj_idx]
    show_environment(env)
    
    # Show the corridor sequence
    if not clean:
        corridors = data["previous_corridor_sequences"][traj_idx]
        show_corridors(corridors)

    # show all previously computed trajectories
    if not clean:
        start_traj_idx = max(0, min(len(data["previous_trajectories"]), traj_idx) - 1)
        for i in range(start_traj_idx, min(len(data["previous_trajectories"]), traj_idx)):
            show_trajectory(data["previous_trajectories"][i], 'gray', False, 
                            data["motion_planner"]["parameters"]["veh_width"], 
                            data["motion_planner"]["parameters"]["veh_height"],
                            with_footprints=False, 
                            virtual_initial_footprint=True,
                            virtual_final_footprint=True,
                            show_markers=False, linewidth=0.5 + 0.5*(i==traj_idx))
            plt.plot(data["previous_trajectories"][i]["px"][0],
                    data["previous_trajectories"][i]["py"][0], 'o', color='k', markersize=5)
        show_trajectory(data["previous_trajectories"][traj_idx], 'gray', False, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=False)
        plt.plot(data["previous_trajectories"][traj_idx]["px"][0],
                    data["previous_trajectories"][traj_idx]["py"][0], 'o', color='k', markersize=5)
    
    # show current destination
    plot_vehicle_footprint(plt.gca(), data["previous_corridor_sequences"][traj_idx]["dest"]["x"],
                           data["previous_corridor_sequences"][traj_idx]["dest"]["y"], 
                           data["motion_planner"]["parameters"]["veh_width"], 
                           data["motion_planner"]["parameters"]["veh_height"], True)

    # Show the trajectory of the mover
    # nb_of_unfaded_samples = 100
    # if original_T > data["travelled_trajectory"]["Tf"]:
    #     normal_unfaded_time = nb_of_unfaded_samples*data["travelled_trajectory"]["dt"]
    #     unfaded_time = max(0, normal_unfaded_time - (original_T - data["travelled_trajectory"]["Tf"]))
    #     nb_of_unfaded_samples = int(unfaded_time/data["travelled_trajectory"]["dt"])
    nb_of_unfaded_samples = max(0, number_of_samples_provided_to_mover - number_of_samples_read_by_mover)
    show_trajectory(data["travelled_trajectory"], 'blue', False, 
                    data["motion_planner"]["parameters"]["veh_width"], 
                    data["motion_planner"]["parameters"]["veh_height"],
                    with_footprints=True, 
                    nb_samples_to_show=number_of_samples_read_by_mover,
                    virtual_initial_footprint=True, 
                    virtual_final_footprint=False,
                    unfaded_nb_samples=nb_of_unfaded_samples,
                    show_initial_footprint_if_showing_footprints=False)
    if data["previous_trajectories"][traj_idx]["emergency_braking"]:
        show_trajectory(data["previous_trajectories"][traj_idx], 'red', False, 
                    data["motion_planner"]["parameters"]["veh_width"], 
                    data["motion_planner"]["parameters"]["veh_height"],
                    with_footprints=False, 
                    virtual_initial_footprint=True,
                    virtual_final_footprint=True,
                    show_markers=False, linewidth=2)
    
    set_env_plot_limits(env)

    if clean:
        plt.xticks([])
        plt.yticks([])
        plt.tight_layout()

    return


def visualize_dynamic_solution(data, T=-1, counter=0, making_mp4=False, fig=None, clean=False):
    DPI = 300
    original_T = T

    if T == -1 or T > data["travelled_trajectory"]["Tf"] + 0*data["travelled_trajectory"]["dt"]:
        T = data["travelled_trajectory"]["Tf"] + 0*data["travelled_trajectory"]["dt"]
    
    # find the index of the last trajectory to (at least partially) show
    traj_idx = 0
    while len(data["replanning_times"]) > traj_idx and T >= data["replanning_times"][traj_idx]:
        traj_idx += 1

    local_replanning_times = [0] + data["replanning_times"]

    # find the index in the travelled trajectory
    travelled_traj_sample_idx = int(min(T/data["travelled_trajectory"]["dt"],
                                    data["travelled_trajectory"]["nb_samples"]))
    # print(f"travelled_traj_sample_idx: {travelled_traj_sample_idx}")
    # print(f"total number of samples: {data['travelled_trajectory']['nb_samples']} ({len(data['travelled_trajectory']['t'])})")

    # print(f"traj_idx = {traj_idx}")
    # print(f"travelled_traj_sample_idx = {travelled_traj_sample_idx}")

    ###############################
    ### Make environment figure ###
    ###############################
    if making_mp4:
        plt.figure(fig.number)
        plt.clf()
    else:
        plt.figure()

    # Show the environment
    # env = data["environment"]
    env = data["previous_environments"][traj_idx]
    show_environment(env)
    
    # Show the corridor sequence
    if not clean:
        corridors = data["previous_corridor_sequences"][traj_idx]
        show_corridors(corridors)

    # Show traces of moving obstacles
    # for obs in data["moving_obstacles"]:
    #     if counter is None:
    #         show_trajectory(obs["travelled_trajectory"], 'firebrick',
    #                         with_trace=True, 
    #                         width=obs["width"], height=obs["height"], with_footprints=False, with_line=False,
    #                         show_markers=False)
    #         plt.gca().add_patch(Rectangle((obs["travelled_trajectory"]["px"][travelled_traj_sample_idx]-obs["width"]/2, 
    #                                        obs["travelled_trajectory"]["py"][travelled_traj_sample_idx]-obs["height"]/2),
    #                                       obs["width"], obs["height"], fill=True, 
    #                                       facecolor='firebrick', edgecolor=None))
            
    #     else:
    #         show_moving_obstacle(obs, travelled_traj_sample_idx, 'firebrick')
        
    # show all previously computed trajectories
    if not clean:
        start_traj_idx = 0 if not making_mp4 else max(0, min(len(data["previous_trajectories"]), traj_idx) - 1)
        for i in range(start_traj_idx, min(len(data["previous_trajectories"]), traj_idx)):
            show_trajectory(data["previous_trajectories"][i], 'gray', False, 
                            data["motion_planner"]["parameters"]["veh_width"], 
                            data["motion_planner"]["parameters"]["veh_height"],
                            with_footprints=False, 
                            virtual_initial_footprint=True,
                            virtual_final_footprint=True,
                            show_markers=False, linewidth=0.5 + 0.5*(i==traj_idx))
            plt.plot(data["previous_trajectories"][i]["px"][0],
                    data["previous_trajectories"][i]["py"][0], 'o', color='k', markersize=5)
        show_trajectory(data["previous_trajectories"][traj_idx], 'gray', False, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=False)
        plt.plot(data["previous_trajectories"][traj_idx]["px"][0],
                    data["previous_trajectories"][traj_idx]["py"][0], 'o', color='k', markersize=5)
    
    # show current destination
    # plt.plot(data["previous_corridor_sequences"][traj_idx]["dest"]["x"],
    #          data["previous_corridor_sequences"][traj_idx]["dest"]["y"], 'o', color='red', markersize=8)
    plot_vehicle_footprint(plt.gca(), data["previous_corridor_sequences"][traj_idx]["dest"]["x"],
                           data["previous_corridor_sequences"][traj_idx]["dest"]["y"], 
                           data["motion_planner"]["parameters"]["veh_width"], 
                           data["motion_planner"]["parameters"]["veh_height"], True)

    # Show the trajectory of the mover
    if making_mp4:
        nb_of_unfaded_samples = 100
        if original_T > data["travelled_trajectory"]["Tf"]:
            normal_unfaded_time = nb_of_unfaded_samples*data["travelled_trajectory"]["dt"]
            unfaded_time = max(0, normal_unfaded_time - (original_T - data["travelled_trajectory"]["Tf"]))
            nb_of_unfaded_samples = int(unfaded_time/data["travelled_trajectory"]["dt"])
        show_trajectory(data["travelled_trajectory"], 'blue', False, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=True, 
                        nb_samples_to_show=travelled_traj_sample_idx,
                        virtual_initial_footprint=True, 
                        virtual_final_footprint=False,
                        unfaded_nb_samples=nb_of_unfaded_samples,
                        show_initial_footprint_if_showing_footprints=False)
        if data["previous_trajectories"][traj_idx]["emergency_braking"]:
            show_trajectory(data["previous_trajectories"][traj_idx], 'red', False, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=False, 
                        virtual_initial_footprint=True,
                        virtual_final_footprint=True,
                        show_markers=False, linewidth=2)
        # if data["previous_trajectories"][start_traj_idx+1]["emergency_braking"]:
        #     show_trajectory(data["previous_trajectories"][start_traj_idx+1], 'red', False, 
        #                 data["motion_planner"]["parameters"]["veh_width"], 
        #                 data["motion_planner"]["parameters"]["veh_height"],
        #                 with_footprints=False, 
        #                 virtual_initial_footprint=True,
        #                 virtual_final_footprint=True,
        #                 show_markers=False, linewidth=2)
    else:
        show_trajectory(data["travelled_trajectory"], 'blue', True, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=True, 
                        nb_samples_to_show=travelled_traj_sample_idx,
                        virtual_initial_footprint=True, 
                        virtual_final_footprint=False)
        
    # if making_mp4:
    #     # show arrow with the current velocity
    #     s = 0.05
    #     vx = s*data["travelled_trajectory"]["vx"][travelled_traj_sample_idx]
    #     vy = s*data["travelled_trajectory"]["vy"][travelled_traj_sample_idx]
    #     plt.arrow(data["travelled_trajectory"]["px"][travelled_traj_sample_idx], 
    #               data["travelled_trajectory"]["py"][travelled_traj_sample_idx], 
    #               vx, vy, head_width=0.05, head_length=0.05, fc='k', ec='k', zorder=99)
        
    #     # show arrow with the current acceleration
    #     # s = 0.05
    #     # ax = s*data["travelled_trajectory"]["ax"][travelled_traj_sample_idx]
    #     # ay = s*data["travelled_trajectory"]["ay"][travelled_traj_sample_idx]
    #     # plt.arrow(data["travelled_trajectory"]["px"][travelled_traj_sample_idx], 
    #     #           data["travelled_trajectory"]["py"][travelled_traj_sample_idx], 
    #     #           ax, ay, head_width=0.05, head_length=0.05, fc='green', ec='green', zorder=99)

    
    set_env_plot_limits(env)

    if clean:
        plt.xticks([])
        plt.yticks([])
        plt.tight_layout()

    if not making_mp4:
        if counter is None:
            plt.savefig(f"post-process/figures/dynamic_solution_traj.png", dpi=200)
        else:
            plt.savefig(f"post-process/figures/animation/traj_frames/dynamic_solution_traj_{counter}.png", dpi=DPI)
        plt.close()

    if making_mp4:
        return

    ############################
    ### Make velocity figure ###
    ############################
    fig, axs = plt.subplots(2, 1)
    # for i in range(min(len(data["previous_trajectories"]), traj_idx+1)):
    #     axs[0].plot([t + local_replanning_times[i] for t in data["previous_trajectories"][i]["t"]],
    #                 data["previous_trajectories"][i]["vx"], '-', 
    #                 color='gray', linewidth=0.5 + 0.5*(i==traj_idx))
    #     axs[1].plot([t + local_replanning_times[i] for t in data["previous_trajectories"][i]["t"]],
    #                 data["previous_trajectories"][i]["vy"], '-', 
    #                 color='gray', linewidth=0.5 + 0.5*(i==traj_idx))
    axs[0].plot([t + local_replanning_times[traj_idx] for t in data["previous_trajectories"][traj_idx]["t"]],
                data["previous_trajectories"][traj_idx]["vx"], '-', 
                color='gray', linewidth=1.0)
    axs[1].plot([t + local_replanning_times[traj_idx] for t in data["previous_trajectories"][traj_idx]["t"]],
                data["previous_trajectories"][traj_idx]["vy"], '-', 
                color='gray', linewidth=1.0)
    
    axs[0].plot(data["travelled_trajectory"]["t"][:travelled_traj_sample_idx],
                data["travelled_trajectory"]["vx"][:travelled_traj_sample_idx],
                '-', color='blue')
    axs[1].plot(data["travelled_trajectory"]["t"][:travelled_traj_sample_idx],
                data["travelled_trajectory"]["vy"][:travelled_traj_sample_idx],
                '-', color='blue')
        
    axs[0].set_ylabel('vx')
    axs[1].set_ylabel('vy')
    axs[1].set_xlabel('t')
    plt.suptitle('Velocity')

    vmax = data["motion_planner"]["parameters"]["v_max"]
    axs[0].fill_between([-10, 1000], [-vmax, -vmax],
                        [-1000, -1000], color='lightgrey')
    axs[0].fill_between([-10, 1000], [vmax, vmax],
                        [1000, 1000], color='lightgrey')
    axs[0].set_xlim([0, max(data["travelled_trajectory"]["t"])])
    axs[0].set_ylim([-1.1*vmax, 1.1*vmax])

    axs[1].fill_between([-10, 1000], [-vmax, -vmax],
                        [-1000, -1000], color='lightgrey')
    axs[1].fill_between([-10, 1000], [vmax, vmax],
                        [1000, 1000], color='lightgrey')
    axs[1].set_xlim([0, max(data["travelled_trajectory"]["t"])])
    axs[1].set_ylim([-1.1*vmax, 1.1*vmax])
    
    if counter is None:
        plt.savefig(f"post-process/figures/dynamic_solution_vel.png", dpi=200)
    else:
        plt.savefig(f"post-process/figures/animation/vel_frames/dynamic_solution_vel_{counter}.png", dpi=DPI)
    plt.close()

    ################################
    ### Make acceleration figure ###
    ################################
    fig, axs = plt.subplots(2, 1)
    # for i in range(min(len(data["previous_trajectories"]), traj_idx+1)):
    #     axs[0].plot([t + local_replanning_times[i] for t in data["previous_trajectories"][i]["t"]],
    #                 data["previous_trajectories"][i]["ax"], '-', 
    #                 color='gray', linewidth=0.5 + 0.5*i==traj_idx)
    #     axs[1].plot([t + local_replanning_times[i] for t in data["previous_trajectories"][i]["t"]],
    #                 data["previous_trajectories"][i]["ay"], '-', 
    #                 color='gray', linewidth=0.5 + 0.5*i==traj_idx)
    axs[0].plot([t + local_replanning_times[traj_idx] for t in data["previous_trajectories"][traj_idx]["t"]],
                data["previous_trajectories"][traj_idx]["ax"], '-', 
                color='gray', linewidth=1.0)
    axs[1].plot([t + local_replanning_times[traj_idx] for t in data["previous_trajectories"][traj_idx]["t"]],
                data["previous_trajectories"][traj_idx]["ay"], '-', 
                color='gray', linewidth=1.0)

    axs[0].plot(data["travelled_trajectory"]["t"][:travelled_traj_sample_idx],
                data["travelled_trajectory"]["ax"][:travelled_traj_sample_idx],
                '-', color='blue')
    axs[1].plot(data["travelled_trajectory"]["t"][:travelled_traj_sample_idx],
                data["travelled_trajectory"]["ay"][:travelled_traj_sample_idx],
                '-', color='blue')
        
    axs[0].set_ylabel('ax')
    axs[1].set_ylabel('ay')
    axs[1].set_xlabel('t')
    plt.suptitle('Acceleration')

    amax = data["motion_planner"]["parameters"]["a_max"]
    axs[0].fill_between([-10, 1000], [-amax, -amax],
                        [-1000, -1000], color='lightgrey')
    axs[0].fill_between([-10, 1000], [amax, amax],
                        [1000, 1000], color='lightgrey')
    axs[0].set_xlim([0, max(data["travelled_trajectory"]["t"])])
    axs[0].set_ylim([-1.1*amax, 1.1*amax])

    axs[1].fill_between([-10, 1000], [-amax, -amax],
                        [-1000, -1000], color='lightgrey')
    axs[1].fill_between([-10, 1000], [amax, amax],
                        [1000, 1000], color='lightgrey')
    axs[1].set_xlim([0, max(data["travelled_trajectory"]["t"])])
    axs[1].set_ylim([-1.1*amax, 1.1*amax])

    if counter is None:
        plt.savefig(f"post-process/figures/dynamic_solution_accel.png", dpi=200)
    else:
        plt.savefig(f"post-process/figures/animation/accel_frames/dynamic_solution_accel_{counter}.png", dpi=DPI)
    plt.close()

def visualize_dynamic_solution_comparison(data_arena, data_ocp, T=-1, counter=0, traj_frames=None, omg_traj=None):
    DPI = 100

    if T == -1 or T > max(data_arena["travelled_trajectory"]["Tf"], data_ocp["travelled_trajectory"]["Tf"]):
        T_arena = data_arena["travelled_trajectory"]["Tf"]
        T_ocp = data_ocp["travelled_trajectory"]["Tf"]
    else:
        T_arena = T
        T_ocp = T
    
    # find the index of the last trajectory to (at least partially) show
    traj_idx_arena = 0
    while len(data_arena["replanning_times"]) > traj_idx_arena and T_arena > data_arena["replanning_times"][traj_idx_arena]:
        traj_idx_arena += 1

    traj_idx_ocp = 0
    while len(data_ocp["replanning_times"]) > traj_idx_ocp and T_ocp > data_ocp["replanning_times"][traj_idx_ocp]:
        traj_idx_ocp += 1

    # find the index in the travelled trajectory
    travelled_traj_sample_idx_arena = int(min(T_arena/data_arena["travelled_trajectory"]["dt"],
                                    data_arena["travelled_trajectory"]["nb_samples"]))
    travelled_traj_sample_idx_ocp = int(min(T_ocp/data_ocp["travelled_trajectory"]["dt"],
                                    data_ocp["travelled_trajectory"]["nb_samples"]))

    ###############################
    ### Make environment figure ###
    ##############################
    if counter is None:
        plt.figure(figsize=(6, 3))
    else:
        plt.figure()

    # Show the environment
    env = data_arena["environment"]
    show_environment(env)

    # Show the corridor sequence
    # corridors = data_arena["previous_corridor_sequences"][traj_idx_arena]
    # show_corridors(corridors, color='cornflowerblue', max_alpha=1.0)
    # corridors = data_ocp["previous_corridor_sequences"][traj_idx_ocp]
    # show_corridors(corridors, color='lightcoral', max_alpha=1.0)

    # Show traces of moving obstacles
    for obs in data_arena["moving_obstacles"]:
        if counter is None:
            show_trajectory(obs["travelled_trajectory"], 'firebrick',
                            with_trace=True, 
                            width=obs["width"], height=obs["height"], with_footprints=False, with_line=False,
                            show_markers=False)
            plt.gca().add_patch(Rectangle((obs["travelled_trajectory"]["px"][travelled_traj_sample_idx_arena]-obs["width"]/2, 
                                           obs["travelled_trajectory"]["py"][travelled_traj_sample_idx_arena]-obs["height"]/2),
                                          obs["width"], obs["height"], fill=True, 
                                          facecolor='firebrick', edgecolor=None))
            
        else:
            show_moving_obstacle(obs, travelled_traj_sample_idx_arena, 'firebrick')
        
    # show current trajectory plans
    for data, traj_idx, color in zip([data_ocp, data_arena], [traj_idx_ocp, traj_idx_arena], ['red', 'blue']):
        for i in range(min(len(data["previous_trajectories"]), traj_idx)):
            plt.plot(data["previous_trajectories"][i]["px"][0],
                    data["previous_trajectories"][i]["py"][0], 'o', color=color, markersize=5)
        show_trajectory(data["previous_trajectories"][traj_idx], color, False, 
                data["motion_planner"]["parameters"]["veh_width"], 
                data["motion_planner"]["parameters"]["veh_height"],
                with_footprints=False, 
                virtual_initial_footprint=False,
                virtual_final_footprint=False,
                show_markers=False, linewidth=0.5)
        plt.plot(data["previous_trajectories"][traj_idx]["px"][0],
                    data["previous_trajectories"][traj_idx]["py"][0], 'o', color=color, markersize=5)

    # Show the trajectory of the mover
    for data, travelled_traj_sample_idx, color in zip([data_ocp, data_arena], 
                                                      [travelled_traj_sample_idx_ocp, travelled_traj_sample_idx_arena], 
                                                      ['red', 'blue']):
        show_trajectory(data["travelled_trajectory"], color, True, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=True, 
                        nb_samples_to_show=travelled_traj_sample_idx,
                        virtual_initial_footprint=False, 
                        virtual_final_footprint=False)
        
    if omg_traj is not None:
        show_trajectory(omg_traj['travelled_trajectory'], 'black', True, 
                        data_arena["motion_planner"]["parameters"]["veh_width"], 
                        data_arena["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=True, 
                        nb_samples_to_show=-1,
                        virtual_initial_footprint=False, 
                        virtual_final_footprint=False)
        
    for i in range(len(omg_traj["previous_trajectories"])):
        plt.plot(omg_traj["previous_trajectories"][i]["px"][0],
                 omg_traj["previous_trajectories"][i]["py"][0], 'o', color='k', markersize=5)
    
    set_env_plot_limits(env)

    if traj_frames is not None:
        canvas = FigureCanvas(plt.gcf())
        img_buffer = io.BytesIO()
        plt.gcf().savefig(img_buffer, format='png', dpi=DPI)
        img_buffer.seek(0)
        img = Image.open(img_buffer)
        traj_frames.append(img)
    else:
        if counter is None:
            plt.xticks([])
            plt.yticks([])
            plt.tight_layout()

            # remove axes box
            # plt.gca().spines['top'].set_visible(False)
            # plt.gca().spines['right'].set_visible(False)
            # plt.gca().spines['bottom'].set_visible(False)
            # plt.gca().spines['left'].set_visible(False)

            plt.annotate(f"appears at\n$t = {data['replanning_times'][0]+0.01:.2f}s$", (0.489, 0.36), (0.36, 0.4510), ha='center',
                         arrowprops=dict(arrowstyle="-|>", 
                                         connectionstyle="arc3,rad=-.1", 
                                         lw=1,
                                         color='darkred'),
                         fontsize=12)
            plt.text(1.08, 0.12, f"appears at\n$t = {data['replanning_times'][1]+0.01:.2f}s$", color='white', 
                     fontsize=12, ha='center', va='center')
            plt.text(1.08, 0.36, f"appears at\n$t = {data['replanning_times'][2]+0.01:.2f}s$", color='white', 
                     fontsize=12, ha='center', va='center')

            print(len(data_arena["previous_trajectories"]))
            print(data_arena["replanning_times"])
            vertical_offsets = [0.02, 0.03, 0.05]
            for i in range(1, len(data_arena["previous_trajectories"])):
                plt.annotate(f"$t = {data['replanning_times'][i-1]+0.01:.2f}s$",
                             (data["previous_trajectories"][i]["px"][0],
                              data["previous_trajectories"][i]["py"][0] - vertical_offsets[i-1]),
                             (data["previous_trajectories"][i]["px"][0], 0.03),
                             arrowprops=dict(arrowstyle="-|>", lw=1, color='k'),
                            color='k', fontsize=12, ha='center')
                
            # Add a below the figure for the different trajectories
            plt.plot([], [], 'o-', color='blue', linewidth=1.0, label='PMP-F')
            plt.plot([], [], 'o-', color='red', linewidth=1.0, label='OCP-F')
            plt.plot([], [], 'o-', color='black', linewidth=1.0, label='OmgTools')

            # make some room for the legend
            plt.subplots_adjust(bottom=0.2)            
            plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3, fontsize=12, frameon=False)

            # plt.show()
        
            plt.savefig(f"post-process/figures/dynamic_solution_traj.png", dpi=200)
            plt.savefig(f"post-process/figures/dynamic_solution_traj.pdf")
        else:
            plt.savefig(f"post-process/figures/animation/traj_frames/dynamic_solution_traj_{counter}.png", dpi=DPI)
    plt.close()


def plot_velocities(trajectories, params):
    colors = ["royalblue", "red"]

    fig, axs = plt.subplots(2, 1)
    for i in range(len(trajectories)):
        axs[0].plot(trajectories[i]["t"], trajectories[i]["vx"], 'o-', 
                    color=colors[i], markersize=1)
        axs[1].plot(trajectories[i]["t"], trajectories[i]["vy"], 'o-', 
                    color=colors[i], markersize=1)
        
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
    
    plt.savefig(f"post-process/figures/dynamic_solution_vel.png", dpi=200)


def visualize_controls(trajectories, params):
    colors = ["royalblue", "red"]

    ### plot controls ###
    fig, axs = plt.subplots(2, 1)
    for i in range(len(trajectories)):
        axs[0].plot(trajectories[i]["t"], trajectories[i]["ax"], 'o-', 
                    color=colors[i], markersize=1)
        axs[1].plot(trajectories[i]["t"], trajectories[i]["ay"], 'o-', 
                    color=colors[i], markersize=1)
        
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

    plt.savefig(f"post-process/figures/dynamic_solution_accel.png", dpi=200)

def translate_method_names(method_names):
    translation = []

    for method in method_names:
        if method == "ARENA":
            translation.append("PMP")
        elif method == "ARENA-FATROP":
            translation.append("PMP-F")
        elif method == "OCP-30":
            translation.append("OCP")
        elif method == "OCP-30-FATROP":
            translation.append("OCP-F")
        else:
            translation.append(method)

    return translation

PLOT_COMPARISON = False


if PLOT_COMPARISON:
    file_arena = "build/output/dynamic_solution.json"
    file_ocp = "build/output/dynamic_solution_ocp.json"

    with open(file_arena) as f:
        data_arena = json.load(f)

    with open(file_ocp) as f:
        data_ocp = json.load(f)

    with open("build/output/dynamic_solution_OMG.json") as f:
        data_omg = json.load(f)

    # print(data_omg)

    ### Make summary figures
    visualize_dynamic_solution_comparison(data_arena, data_ocp, -1, None, omg_traj=data_omg)
    plot_velocities([data_arena["travelled_trajectory"], data_ocp["travelled_trajectory"]], data_arena["motion_planner"]["parameters"])
    visualize_controls([data_arena["travelled_trajectory"], data_ocp["travelled_trajectory"]], data_arena["motion_planner"]["parameters"])

    ### plot computation times
    plt.figure()
    offset = 0.006*max(data_arena["replanning_times"])
    s = 0
    lw = 2
    for i in range(len(data_arena["replanning_times"])):
        plt.plot([data_arena["replanning_times"][i], data_arena["replanning_times"][i]], 
                  [0, data_arena["previous_trajectories"][i]["total_computation_time"]], 
                  'o-', color='blue', markersize=s, lw=lw)
        plt.plot([data_arena["replanning_times"][i], data_arena["replanning_times"][i]], 
                 [0, data_arena["previous_trajectories"][i]["solver_time"]], 
                 'o-', color='midnightblue', markersize=s, lw=lw)
    
    for i in range(len(data_ocp["replanning_times"])):
        plt.plot([data_ocp["replanning_times"][i] + offset, data_ocp["replanning_times"][i] + offset], 
                 [0, data_ocp["previous_trajectories"][i]["total_computation_time"]], 
                 'o-', color='red', markersize=s, lw=lw)
        plt.plot([data_ocp["replanning_times"][i] + offset, data_ocp["replanning_times"][i] + offset], 
                 [0, data_ocp["previous_trajectories"][i]["solver_time"]], 
                 'o-', color='maroon', markersize=s, lw=lw)

    plt.xlabel('t')
    plt.ylabel('solver time [ms]')
    print(f'max_arena: {max(data_arena["previous_trajectories"][i]["total_computation_time"] for i in range(len(data_arena["replanning_times"])))*1.1}')
    print(f'max_ocp: {max(data_ocp["previous_trajectories"][i]["total_computation_time"] for i in range(len(data_ocp["replanning_times"])))*1.1}')

    plt.ylim([0, max(max(data_arena["previous_trajectories"][i]["total_computation_time"] for i in range(len(data_arena["replanning_times"])))*1.1,
                     max(data_ocp["previous_trajectories"][i]["total_computation_time"] for i in range(len(data_ocp["replanning_times"])))*1.1)])
    plt.title('Solver time at replanning times')
    plt.savefig(f"post-process/figures/solver_time.png", dpi=300)

    arena_solver_times = [f"{data_arena['previous_trajectories'][i]['solver_time']:.2f}" for i in range(len(data_arena['previous_trajectories']))]
    ocp_solver_times = [f"{data_ocp['previous_trajectories'][i]['solver_time']:.2f}" for i in range(len(data_ocp['previous_trajectories']))]
    arena_total_times = [f"{data_arena['previous_trajectories'][i]['total_computation_time']:.2f}" for i in range(len(data_arena['previous_trajectories']))]
    ocp_total_times = [f"{data_ocp['previous_trajectories'][i]['total_computation_time']:.2f}" for i in range(len(data_ocp['previous_trajectories']))]
    omg_solver_times = [f"{d:.2f}" for d in data_omg['avg_solver_times']]
    omg_total_times = [f"{d:.2f}" for d in data_omg['total_times']]

    print("ARENA Computation times:")
    print(f"\tSolver times: {arena_solver_times}")
    print(f"\tTotal computation times: {arena_total_times}")

    print("OCP Computation times:")
    print(f"\tSolver times: {ocp_solver_times}")
    print(f"\tTotal computation times: {ocp_total_times}")

    print("OMG Computation times:")
    print(f"\tSolver times: {omg_solver_times}")
    print(f"\tTotal computation times: {omg_total_times}")

    # print a latex table with the computation times
    # the table contains two columns: the solver times and the total computation times
    # each column is split into three subcolumns (ARENA, OCP, OmgTools)
    # the rows show the computation times for each previous trajectory
    print("\\begin{table}")
    print("\t\\centering")
    print("\t\\caption{Computation times for the replanning case (average over 10 runs)}")
    print("\t\\label{tab:computation-times-replanning}")
    print("\t\\setlength{\\tabcolsep}{4pt}")
    print("\t\\renewcommand{\\arraystretch}{1.1}")
    print("\t\\begin{tabular}{c|ccc|ccc}")
    print("\t\t& \\multicolumn{3}{c|}{Solver time [ms]} & \\multicolumn{3}{c}{Total computation time [ms]} \\\\")
    # print("\t\t\\hline")
    print("\t\tTime [s] & PMP-F & OCP-F & OmgTools & PMP-F & OCP & OmgTools\\\\")
    print("\t\t\\hline")
    times = [-0.01] + data_arena["replanning_times"]
    for i in range(len(arena_solver_times)):
        solver_times = [float(arena_solver_times[i]), float(ocp_solver_times[i]), float(omg_solver_times[i])]
        min_idx = solver_times.index(min(solver_times))
        solver_strings = [f"{solver_times[t]:.2f}" if t != min_idx else f"\\textbf{{{solver_times[t]:.2f}}}" for t in range(len(solver_times))]

        total_times = [float(arena_total_times[i]), float(ocp_total_times[i]), float(omg_total_times[i])]
        min_idx = total_times.index(min(total_times))
        total_strings = [f"{total_times[t]:.2f}" if t != min_idx else f"\\textbf{{{total_times[t]:.2f}}}" for t in range(len(total_times))]
        
        print(f"\t\t{times[i]+0.01:.2f} & {solver_strings[0]} & {solver_strings[1]} & {solver_strings[2]} & {total_strings[0]} & {total_strings[1]} & {total_strings[2]} \\\\")
    # print("\t\t\\hline")
    print("\t\\end{tabular}")
    print("\\end{table}")



    ### Make animation frames
    # traj_frames = []

    # total_time_arena = data_arena["travelled_trajectory"]["Tf"]
    # total_time_ocp = data_ocp["travelled_trajectory"]["Tf"]
    # counter = 0
    # curr_time = 0.0
    # step_size = 1
    # while curr_time < max(total_time_arena, total_time_ocp) + data_arena["travelled_trajectory"]["dt"]:
    #     print(f"creating figure at t = {curr_time:.3f} ({curr_time/max(total_time_arena, total_time_ocp)*100:.2f}%) with counter = {counter}")
    #     visualize_dynamic_solution_comparison(data_arena, data_ocp, curr_time, counter, traj_frames=traj_frames)
    #     counter += 1
    #     curr_time += step_size * data_arena["travelled_trajectory"]["dt"]

    # visualize_dynamic_solution_comparison(data_arena, data_ocp, max(total_time_arena, total_time_ocp), counter, traj_frames=traj_frames)
    # print(f"Last figure has count: {counter}")

    # traj_frames[0].save('../post-process/figures/animation/animation_traj.gif', save_all=True, append_images=traj_frames[1:], optimize=False, duration=100, loop=0)





else:
    # file = "output/dynamic_solution_ocp.json"
    # file = "build/output/dynamic_solution_movable_destination.json"
    file = "build/output/dynamic_solution_movable_destination_sampler.json"
    with open(file) as f:
        data = json.load(f)

    ### Make summary figures
    visualize_dynamic_solution(data, -1, None)

    ### plot computation times
    plt.figure()
    ideal_planning_ms = 30
    for i in range(len(data["replanning_times"])):
        # check if key is in dictionary
        if data["record_sample_time"]:
            plt.plot([data["replanning_times"][i], data["replanning_times"][i]], 
                     [0, data["ms_to_retrieve_sample"][i]], 'o-', color='blue')
            
        plt.plot([data["replanning_times"][i], data["replanning_times"][i]], [0, data["previous_trajectories"][i]["total_computation_time"]], 'o-', color='gray')
        plt.plot([data["replanning_times"][i], data["replanning_times"][i]], [0, data["previous_trajectories"][i]["solver_time"]], 'o-k')
        
        if data["previous_trajectories"][i]["solver_time"] > ideal_planning_ms:
            # add value in text
            plt.text(data["replanning_times"][i], 15, 
                     f" {data['previous_trajectories'][i]['solver_time']:.2f}", 
                     fontsize=12, ha='left', va='bottom', color='gray')

        if data["previous_trajectories"][i]["total_computation_time"] > ideal_planning_ms:
            # add value in text
            plt.text(data["replanning_times"][i], 1.8*ideal_planning_ms, 
                     f" {data['previous_trajectories'][i]['total_computation_time']:.2f}", 
                     fontsize=6, ha='left', va='bottom', color='k', rotation=90)
            
            if data["previous_trajectories"][i]["vx"][0] == 0 and data["previous_trajectories"][i]["vy"][0] == 0:
                plt.text(data["replanning_times"][i], 11, "not\ncritical", fontsize=8, ha='center', va='bottom', color='r')

    plt.axhline(y=ideal_planning_ms, color='r', linestyle='-', lw=2)

    plt.xlabel('t')
    plt.ylabel('solver time [ms]')
    # plt.ylim([0, max(data["previous_trajectories"][i]["total_computation_time"] for i in range(len(data["replanning_times"])))*1.1])
    plt.ylim([0, 2*ideal_planning_ms])
    plt.title('Solver time at replanning times')
    plt.savefig(f"post-process/figures/solver_time.png", dpi=300)

    print([data["previous_trajectories"][i]["total_computation_time"] for i in range(len(data["replanning_times"]))])
    print(data["travelled_trajectory"]["Tf"])
    print(data["replanning_times"])
    
    MAKE_FRAMES = 0
    MAKE_SIMULATION_MP4 = 0
    MAKE_REALTIME_PLOT = 1




    if MAKE_FRAMES:
        ## Make animation frames
        total_time = data["travelled_trajectory"]["Tf"]
        counter = 0
        curr_time = 0.0
        step_size = 1
        while curr_time < total_time + data["travelled_trajectory"]["dt"]:
            print(f"creating figure at t = {curr_time:.3f} ({curr_time/total_time*100:.2f}%) with counter = {counter}")
            visualize_dynamic_solution(data, curr_time, counter)
            counter += 1
            curr_time += step_size * data["travelled_trajectory"]["dt"]

        visualize_dynamic_solution(data, total_time, counter)
        print(f"Last figure has count: {counter}")

    if MAKE_SIMULATION_MP4 or MAKE_REALTIME_PLOT:
        fps = 25
        mp4_dt = 1.0/fps
        total_time = data["travelled_trajectory"]["Tf"] + 1.5 
        clean = True

        import matplotlib.animation as animation
        from matplotlib.animation import FFMpegWriter

        writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=['-pix_fmt', 'yuv420p'])
        fig = plt.figure()
        def update(frame):
            if frame % 5 == 0:
                print(f"creating figure at t = {frame*mp4_dt:.3f} ({frame*mp4_dt/total_time*100:.2f}%)")
            
            if MAKE_SIMULATION_MP4:
                visualize_dynamic_solution(data, T=mp4_dt*frame, counter=None, making_mp4=True, fig=fig, clean=clean)
            elif MAKE_REALTIME_PLOT:
                visualize_real_time_solution(data, T=mp4_dt*frame, fig=fig, clean=clean)
            
            return fig
        anim = animation.FuncAnimation(fig, update, range(int((total_time/mp4_dt))), repeat=False)
        if clean:
            anim.save("post-process/figures/animation/animation_traj_clean.mp4", writer=writer)
        else:
            anim.save("post-process/figures/animation/animation_traj.mp4", writer=writer)