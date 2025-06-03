import sys
sys.path.append('post-process/')
from visualization_helpers import *
import matplotlib.patches as mpatches
import numpy as np


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
    SHOW_INTERSECTIONS = 1
    SHOW_CLAIMED_CELLS = 1
    SHOW_CURRENT_PLANS = 0
    SHOW_TRAVELLED_TRAJECTORIES = 0


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
                # show_corridors({"sequence":[log["intersection"]]}, color='red')
                show_corridor_union(log["intersection"], color='red')

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
    for i in range(len(planners)):
        nb_of_unfaded_samples = 100 if SHOW_TRAVELLED_TRAJECTORIES else 0
        if original_T > travelled_trajectories[i]["Tf"]:
            normal_unfaded_time = nb_of_unfaded_samples*travelled_trajectories[i]["dt"]
            unfaded_time = max(0, normal_unfaded_time - (original_T - travelled_trajectories[i]["Tf"]))
            nb_of_unfaded_samples = int(unfaded_time/travelled_trajectories[i]["dt"])
        show_trajectory(travelled_trajectories[i], vehicle_colors[i], with_trace=SHOW_TRAVELLED_TRAJECTORIES,
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
            local_idx = min(traj_sample_idx, len(travelled_states[i]) - 1)
            state_string = str(travelled_states[i][local_idx])
            if "WAIT" in state_string and "FREE" not in state_string:
                state_string += " (" + str(data["agents"][i]["travelled_blocking_agent_idx"][local_idx]) + ")"
            plt.text(0.66, 0.9 - i*0.05, state_string, 
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

def extract_segments(task):
    events = task["task_events"]
    segments = []
    current_state = None
    last_state_switch_time_stamp = None
    current_start = None
    planning_segments = []

    # make sure to sort events by time_stamp
    events.sort(key=lambda x: x["time_stamp"])

    for i, event in enumerate(events):
        evt_type = event["event_type"]
        time = event["time_stamp"]
        meta = event.get("optional_meta_data", 0)

        if evt_type == "TASK_REVEALED":
            segments.append(("TASK_REVEALED", time, time, 2))
        elif evt_type == "TASK_POSTPONED":
            segments.append(("TASK_POSTPONED", time, time + meta, 2))
        elif evt_type == "MOVER_WAITING":
            # if the state is not initilialized yet, update it
            if current_state is None:
                current_state = "MOVER_WAITING"
                last_state_switch_time_stamp = time

            # if we were moving, add a moving segment
            elif current_state == "MOVER_MOVING":
                segments.append(("MOVER_MOVING", last_state_switch_time_stamp, time, 1))
                current_state = "MOVER_WAITING"
                last_state_switch_time_stamp = time

            # if we were already waiting, ignore this event
            else:
                continue
                
        elif evt_type == "MOVER_MOVING":
            # if the state is not initialized yet, update it
            if current_state is None:
                current_state = "MOVER_MOVING"
                last_state_switch_time_stamp = time

            # if we were waiting, add a waiting segment
            elif current_state == "MOVER_WAITING":
                segments.append(("MOVER_WAITING", last_state_switch_time_stamp, time, 1))
                current_state = "MOVER_MOVING"
                last_state_switch_time_stamp = time
            
            # if we were already moving, ignore this event
            elif current_state == "MOVER_MOVING":
                continue

        elif evt_type == "TASK_PLANNING_OCCURED":
            segments.append(("TASK_PLANNING_OCCURED", time, time + 0.001*meta, 2))

        elif evt_type == "TASK_COMPLETED":
            if current_state == "MOVER_MOVING":
                segments.append(("MOVER_MOVING", last_state_switch_time_stamp, time, 1))
            
            elif current_state == "MOVER_WAITING":
                segments.append(("MOVER_WAITING", last_state_switch_time_stamp, time, 1))

            segments.append(("TASK_COMPLETED", time, time, 2))

    return segments


def visualize_task_completion(data):
    event_colors = {
        "TASK_REVEALED": ("black", "TASK_REVEALED"),
        "TASK_POSTPONED": ("gray", "TASK_POSTPONED"),
        "MOVER_WAITING": ("orange", "MOVER_WAITING"),
        "MOVER_MOVING": ("green", "MOVER_MOVING"),
        "TASK_PLANNING_OCCURED": ("blue", "TASK_PLANNING_OCCURED"),
        "TASK_COMPLETED": ("red", "TASK_COMPLETED"),
    }

    # Plotting
    fig, ax = plt.subplots(figsize=(12, 6))

    yticks = []
    yticklabels = []
    offset = 0
    height_of_first_bar_for_this_agent = 0

    # list completion times of all tasks
    total_time = -1
    for task in data["tasks"]:
        for event in task["task_events"]:
            if event["event_type"] == "TASK_COMPLETED":
                total_time = max(total_time, event["time_stamp"])

    # group tasks by agent_idx and sort by time to reveal task (per agent)
    data["tasks"].sort(key=lambda x: (x["agent_idx"], x["time_to_reveal_task"]))

    for idx, task in enumerate(data["tasks"]):
        agent = task["agent_idx"]
        dest = task["destination_name"]
        reveal_time = task["time_to_reveal_task"]

        task_label = f"Agent {agent} to {dest}"
        if idx > 0 and data["tasks"][idx - 1]["agent_idx"] != agent:
            ax.add_patch(mpatches.Rectangle((0, height_of_first_bar_for_this_agent - 0.2), 
                total_time, idx - 1 + offset + 2*0.2 - height_of_first_bar_for_this_agent, 
                color='lightgray', zorder=0, alpha=0.5))

            offset += 1
            height_of_first_bar_for_this_agent = idx + offset
        
        y = idx + offset
        yticks.append(y)
        yticklabels.append(task_label)

        # Expected reveal line
        # ax.axvline(x=reveal_time, color='black', linestyle='--', alpha=0.3)

        segments = extract_segments(task)

        for seg_type, start, end, zorder in segments:
            color, _ = event_colors.get(seg_type, ("purple", seg_type))
            ax.barh(y, end - start if end > start else 0.1, left=start, 
                    color=color, edgecolor='k', height=0.4, alpha=1.0, 
                    zorder=zorder)

    ax.add_patch(mpatches.Rectangle((0, height_of_first_bar_for_this_agent - 0.2), 
        total_time, idx + offset + 2*0.2 - height_of_first_bar_for_this_agent, 
        color='lightgray', zorder=0, alpha=0.5))

    # Formatting
    ax.set_xlabel("Time")
    ax.set_yticks(yticks)
    ax.set_yticklabels(yticklabels)
    ax.set_title("MoverTask Progress Gantt Chart")
    # ax.grid(True, axis='x', linestyle='--', alpha=0.5)

    ax.set_xlim(0, total_time)

    # Legend
    legend_patches = [mpatches.Patch(color=color, label=label) for color, label in event_colors.values()]
    ax.legend(handles=legend_patches, loc='best')

    plt.tight_layout()
    plt.savefig("post-process/multi_mover_simulator/animations/task_completion_gantt_chart.png")
    

def visualize_computation_time_per_simulation_step(data):
    computation_times = data["computation_time_per_simulation_step"]
    total_time = data["nb_simulated_samples"]*data["simulation_time_step"]

    # # compute the amount of buffered samples
    # # every 0.01 seconds, a sample is removed from the buffer
    # fine_time_grid = np.linspace(0, total_time, int(total_time*10000))
    # buffered_samples = np.zeros(len(fine_time_grid))

    # current_nb_in_buffer = 0
    # time_since_last_sample_read = 0.0
    # time_since_computation_started = 0.0
    # computation_time_idx = 0
    # for i in range(len(fine_time_grid)):
    #     # check if new sample is read
    #     if time_since_last_sample_read > data["simulation_time_step"]:
    #         current_nb_in_buffer -= 1
    #         time_since_last_sample_read = 0.0
        
    #     # check if new sample is added
    #     if computation_time_idx < len(computation_times) and \
    #        time_since_computation_started >= 0.0001*computation_times[computation_time_idx]:
    #         current_nb_in_buffer += 1
    #         time_since_computation_started = 0.0
    #         computation_time_idx += 1

    #     # update time counters
    #     time_since_last_sample_read += fine_time_grid[i] - (fine_time_grid[i-1] if i > 0 else 0)
    #     time_since_computation_started += fine_time_grid[i] - (fine_time_grid[i-1] if i > 0 else 0)
    #     buffered_samples[i] = current_nb_in_buffer

    # # Plotting
    # fig, ax = plt.subplots(figsize=(12, 6))
    # ax.plot(fine_time_grid, buffered_samples, color='blue', label='Buffered Samples')
    # ax.set_xlabel("Time (s)")
    # ax.set_ylabel("Number of Buffered Samples")
    # plt.savefig("post-process/multi_mover_simulator/animations/buffered_samples_over_time.png")

    # Plotting: a bar plot with computation times under 10ms in blue and others in red
    fig, ax = plt.subplots(figsize=(12, 6))
    computation_times = np.array(computation_times)
    time_grid = np.arange(0, len(computation_times)) * data["simulation_time_step"]
    colors = np.where(computation_times < 10, 'blue', 'red')
    ax.bar(time_grid, computation_times, color=colors, width=data["simulation_time_step"]*0.8, alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Computation Time (ms)")
    plt.xlim(0, total_time)
    plt.ylim(0, 100)
    plt.savefig("post-process/multi_mover_simulator/animations/buffered_samples_over_time.png")
    
    

# Load the data
file = "build/output/multi_mover_simulator.json"

with open(file) as f:
    data = json.load(f)

visualize_task_completion(data)
visualize_computation_time_per_simulation_step(data)
# exit()
create_multi_mover_motion_video(data, fps=25)