import sys
sys.path.append('post-process/')
from visualization_helpers import *


def create_multi_mover_motion_snapshot(data, T, fig=None, **kwargs):
    vehicle_colors = ['b', 'green', 'k', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    
    planners = [d["planner"] for d in data["agents"]]
    travelled_trajectories = [d["travelled_trajectory"] for d in data["agents"]]
    planned_trajectories = [d["planned_trajectories"] for d in data["agents"]]
    planned_times = [d["planned_times"] for d in data["agents"]]
    travelled_final_destinations = [d["travelled_final_destinations"] for d in data["agents"]]
    travelled_states = [d["travelled_states"] for d in data["agents"]]

    environments = [planner["environment"] for planner in planners]
    corridor_sequences = [d["planned_corridor_sequences"] for d in data["agents"]]

    agent_addresses = [d["memory_address"] for d in data["agents"]]
    claimable_destinations_info = data["claimed_destinations_info"]

    SHOW_CORRIDORS = 0
    SHOW_INTERSECTIONS = 0
    SHOW_CLAIMED_CELLS = 1
    SHOW_CURRENT_PLANS = 0
    SHOW_TRAVELLED_TRAJECTORIES = 1


    max_T = max([traj["Tf"] for traj in travelled_trajectories])

    tt = travelled_trajectories[0]["t"]

    original_T = T
    T = max(0, min(max_T, T))
    traj_sample_idx = int(T/travelled_trajectories[0]["dt"])

    plt.figure(fig.number)
    plt.clf()

    # show environment
    for i in range(len(environments)):
        show_environment(environments[i], obstacle_color=vehicle_colors[i], obstacles_alpha=1.0)
        idx = min(traj_sample_idx, travelled_trajectories[i]["nb_samples"] - 2)
        plt.plot(travelled_final_destinations[i][idx]["x"], 
                 travelled_final_destinations[i][idx]["y"], 
                 color=vehicle_colors[i], markersize=5, label="goal")

    # show corridors
    if SHOW_CORRIDORS:
        for i in range(len(corridor_sequences)):
            if (travelled_states[i][min(len(travelled_states[i])-1, traj_sample_idx)] == "IDLING"):
                continue

            replan_idx = 0
            while replan_idx < len(planned_times[i]) - 1 and \
                    T > planned_times[i][replan_idx+1]:
                replan_idx += 1
            corridors = corridor_sequences[i][replan_idx]
            show_corridors(corridors, color=vehicle_colors[i], max_alpha=0.2)

    # show intersections
    if SHOW_INTERSECTIONS:
        for log in data["intersection_logs"]:
            if T >= log["time_entering"] and T <= log["time_leaving"]:
                show_corridors({"sequence":[log["intersection"]]}, color='red')

    # show claimed cells
    if SHOW_CLAIMED_CELLS:
        for (dest_name, destination) in claimable_destinations_info[min(len(claimable_destinations_info)-1, traj_sample_idx)].items():
            cell = destination["location"]
            color = 'lightgray'
            if destination["claimed"]:
                agent_memory_address = destination["claimed_by"]
                if agent_memory_address in agent_addresses:
                    agent_idx = agent_addresses.index(agent_memory_address)
                    color = vehicle_colors[agent_idx]
            
            cell_width = environments[0]["cell_width"]
            cell_height = environments[0]["cell_height"]
            corridor = {"x_min":cell["x"]*cell_width,
                        "x_max":(cell["x"]+1)*cell_width,
                        "y_min":cell["y"]*cell_height,
                        "y_max":(cell["y"]+1)*cell_height}    
            show_corridors({"sequence":[corridor]}, color=color, max_alpha=0.5)


    # show current plans completely
    if SHOW_CURRENT_PLANS:
        for i in range(len(planners)):
            curr_plan_idx = 0
            while curr_plan_idx < len(planned_times[i]) - 1 and T > planned_times[i][curr_plan_idx+1]:
                curr_plan_idx += 1

            show_trajectory(planned_trajectories[i][curr_plan_idx], 'gray', show_markers=False)

    # show travelled trajectories
    if SHOW_TRAVELLED_TRAJECTORIES:
        for i in range(len(planners)):
            nb_of_unfaded_samples = 100
            if original_T > travelled_trajectories[i]["Tf"]:
                normal_unfaded_time = nb_of_unfaded_samples*travelled_trajectories[i]["dt"]
                unfaded_time = max(0, normal_unfaded_time - (original_T - travelled_trajectories[i]["Tf"]))
                nb_of_unfaded_samples = int(unfaded_time/travelled_trajectories[i]["dt"])
            show_trajectory(travelled_trajectories[i], vehicle_colors[i], with_trace=True,
                            width=planners[i]["parameters"]["veh_width"],
                            height=planners[i]["parameters"]["veh_height"],
                            with_footprints=True, 
                            nb_samples_to_show=min(traj_sample_idx, len(travelled_trajectories[i]["px"])),
                            virtual_final_footprint=False, 
                            unfaded_nb_samples=nb_of_unfaded_samples, 
                            show_initial_footprint_if_showing_footprints=False,
                            trace_alpha=0.1)
            if len(travelled_trajectories[i]['px']) > 0:
                # plt.plot(travelled_trajectories[i]["px"][0], travelled_trajectories[i]["py"][0], 'o', 
                #         color=vehicle_colors[i], markersize=5)
                # plt.plot(travelled_trajectories[i]["px"][-1], travelled_trajectories[i]["py"][-1], 'o', 
                #         color=vehicle_colors[i], markersize=5)
                plt.plot(travelled_final_destinations[i][min(traj_sample_idx, len(travelled_final_destinations[i])-1)]["x"],
                        travelled_final_destinations[i][min(traj_sample_idx, len(travelled_final_destinations[i])-1)]["y"], 'o',
                        color=vehicle_colors[i], markersize=5)

    # show vehicle numbers and states
    for i in range(len(planners)):
        if len(travelled_trajectories[i]['px']) > 0:
            # vehicle number
            plt.text(travelled_trajectories[i]["px"][traj_sample_idx] - 
                        planners[i]["parameters"]["veh_width"]/4, 
                     travelled_trajectories[i]["py"][traj_sample_idx] + 
                        planners[i]["parameters"]["veh_height"]/4, 
                     str(i), color=vehicle_colors[i], fontsize=10, ha='center', 
                     va='center')
            # vehicle state
            plt.text(0.66, 0.9 - i*0.05, str(travelled_states[i][min(traj_sample_idx, len(travelled_states[i]) - 1)]), 
                     color=vehicle_colors[i], fontsize=8, ha='left', 
                     va='top', transform=fig.transFigure)

    set_env_plot_limits(environments[0])
    plt.xticks([])
    plt.yticks([])

    plt.title(f"t = {original_T:.3f} s")

    # shift axes to the left
    ax = plt.gca()
    ax.set_position([0.02, 0.1, 0.65, 0.8])

    return

def create_multi_mover_motion_video(data, **kwargs):
    fps = kwargs.get('fps', 25)
    mp4_dt = 1.0/fps
    total_time = max([d["travelled_trajectory"]["Tf"] for d in data["agents"]]) + 1.0

    start_time = 0.0
    stop_time = total_time

    import matplotlib.animation as animation
    from matplotlib.animation import FFMpegWriter

    writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=['-pix_fmt', 'yuv420p'])
    fig = plt.figure()
    def update(frame):
        if frame % 5 == 0:
            print(f"creating figure at t = {frame*mp4_dt:.3f} ({frame*mp4_dt/total_time*100:.2f}%)")
        create_multi_mover_motion_snapshot(data, T=mp4_dt*frame, fig=fig, kwargs=kwargs)
        return fig
    
    anim = animation.FuncAnimation(fig, update, 
                                    range(int((start_time/mp4_dt)), 
                                          int((stop_time/mp4_dt))), 
                                    repeat=False)
    anim.save(f"post-process/multi_mover_simulator/animations/animation.mp4", writer=writer)



# Load the data
file = "build/output/multi_mover_simulator.json"

with open(file) as f:
    data = json.load(f)

create_multi_mover_motion_video(data, fps=25)