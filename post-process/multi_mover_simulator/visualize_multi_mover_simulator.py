import sys
sys.path.append('post-process/')
from visualization_helpers import *
import matplotlib.patches as mpatches
import numpy as np


def create_multi_mover_motion_snapshot(data, prepared_data, handles_to_clear, footprint_handles, vehicle_nb_handles, vehicle_state_handles, T, fig=None, **kwargs):
    max_T = max([traj["Tf"] for traj in prepared_data["travelled_trajectories"]])

    original_T = T
    T = max(0, min(max_T, T))
    traj_sample_idx = int(T/prepared_data["travelled_trajectories"][0]["dt"])

    # plt.figure(fig.number, dpi=kwargs.get('dpi', 200))
    # plt.clf()

    for h in handles_to_clear:
        if isinstance(h, list):
            for hh in h:
                if hh is not None:
                    hh.remove()
        else:
            h.remove()

    handles_to_clear = []

    # show corridors
    if prepared_data["SHOW_CORRIDORS"]:
        for i in range(len(prepared_data["corridor_sequences"])):
            if (prepared_data["travelled_states"][i][min(len(prepared_data["travelled_states"][i])-1, traj_sample_idx)] == "IDLING"):
                continue

            replan_idx = 0
            while replan_idx < len(prepared_data["planned_times"][i]) - 1 and \
                    T > prepared_data["planned_times"][i][replan_idx+1]:
                replan_idx += 1
            corridors = prepared_data["corridor_sequences"][i][replan_idx]
            h = show_corridors(corridors, color=prepared_data["vehicle_colors"][i], max_alpha=0.2)
            handles_to_clear.extend(h)

    # show intersections
    if prepared_data["SHOW_INTERSECTIONS"]:
        for log in data["intersection_logs"]:
            if (T >= log["time_entering"] and T <= log["time_leaving"]):
                # show_corridors({"sequence":[log["intersection"]]}, color='red')

                h = show_corridor_union(log["intersection"], color='red')
                handles_to_clear.extend(h)
                

    # show claimed cells
    if prepared_data["SHOW_CLAIMED_CELLS"]:
        for (dest_name, destination) in prepared_data["claimable_destinations_info"][min(len(prepared_data["claimable_destinations_info"])-1, traj_sample_idx)].items():
            cell = destination["location"]
            color = 'lightgray'
            if destination["claimed"]:
                agent_memory_address = destination["claimed_by"]
                if agent_memory_address in prepared_data["agent_addresses"]:
                    agent_idx = prepared_data["agent_addresses"].index(agent_memory_address)
                    color = prepared_data["vehicle_colors"][agent_idx]
            
            cell_width = prepared_data["environments"][0]["cell_width"]
            cell_height = prepared_data["environments"][0]["cell_height"]
            corridor = {"x_min":cell["x"]*cell_width,
                        "x_max":(cell["x"]+1)*cell_width,
                        "y_min":cell["y"]*cell_height,
                        "y_max":(cell["y"]+1)*cell_height}    
            h = show_corridors({"sequence":[corridor]}, color=color, max_alpha=0.5)
            handles_to_clear.extend(h)

    # show current plans completely
    if prepared_data["SHOW_CURRENT_PLANS"]:
        for i in range(len(prepared_data["planners"])):
            curr_plan_idx = 0
            while curr_plan_idx < len(prepared_data["planned_times"][i]) - 1 and T > prepared_data["planned_times"][i][curr_plan_idx+1]:
                curr_plan_idx += 1

            h, footprints = show_trajectory(prepared_data["planned_trajectories"][i][curr_plan_idx], 'gray', show_markers=False)
            handles_to_clear.extend(h)
            handles_to_clear.extend(footprints)                

    # show travelled trajectories
    store_footprint_handles = len(footprint_handles) == 0
    if store_footprint_handles:
        footprint_handles = [[] for _ in range(len(prepared_data["planners"]))]
        vehicle_nb_handles = [None for _ in range(len(prepared_data["planners"]))]
        vehicle_state_handles = [None for _ in range(len(prepared_data["planners"]))]
    for i in range(len(prepared_data["planners"])):
        nb_of_unfaded_samples = 100 if prepared_data["SHOW_TRAVELLED_TRAJECTORIES"] else 0
        if original_T >= prepared_data["travelled_trajectories"][i]["Tf"]:
            normal_unfaded_time = nb_of_unfaded_samples*prepared_data["travelled_trajectories"][i]["dt"]
            unfaded_time = max(0, normal_unfaded_time - (original_T - prepared_data["travelled_trajectories"][i]["Tf"]))
            nb_of_unfaded_samples = int(unfaded_time/prepared_data["travelled_trajectories"][i]["dt"])
        [h, footprints] = show_trajectory(prepared_data["travelled_trajectories"][i], prepared_data["vehicle_colors"][i], with_trace=prepared_data["SHOW_TRAVELLED_TRAJECTORIES"],
                        width=prepared_data["planners"][i]["parameters"]["veh_width"],
                        height=prepared_data["planners"][i]["parameters"]["veh_height"],
                        with_footprints=store_footprint_handles, 
                        nb_samples_to_show=min(traj_sample_idx, len(prepared_data["travelled_trajectories"][i]["px"])),
                        virtual_final_footprint=False, 
                        unfaded_nb_samples=nb_of_unfaded_samples, 
                        show_initial_footprint_if_showing_footprints=False,
                        trace_alpha=0.1)
        handles_to_clear.extend(h)
        if store_footprint_handles:
            footprint_handles[i] = footprints
        else:
            handles_to_clear.extend(footprints)
            # update the positions
            # NOTE: assuming we only have the final footprints
            for j in range(len(footprint_handles[i])):
                width=prepared_data["planners"][i]["parameters"]["veh_width"]
                height=prepared_data["planners"][i]["parameters"]["veh_height"]
                if j == 1:
                    width *= 0.8
                    height *= 0.8
                # update the anchor of the fancybox
                footprint_handles[i][j].set_x(
                    prepared_data["travelled_trajectories"][i]["px"][min(traj_sample_idx, len(prepared_data["travelled_trajectories"][i]["px"])-1)]-width/2)
                footprint_handles[i][j].set_y(
                    prepared_data["travelled_trajectories"][i]["py"][min(traj_sample_idx, len(prepared_data["travelled_trajectories"][i]["py"])-1)]-height/2)

        if len(prepared_data["travelled_trajectories"][i]['px']) > 0:
            h = plt.plot(prepared_data["travelled_final_destinations"][i][min(traj_sample_idx, len(prepared_data["travelled_final_destinations"][i])-1)]["x"],
                    prepared_data["travelled_final_destinations"][i][min(traj_sample_idx, len(prepared_data["travelled_final_destinations"][i])-1)]["y"], 'o',
                    color=prepared_data["vehicle_colors"][i], markersize=5)

    # show vehicle numbers and states
    for i in range(len(prepared_data["planners"])):
        if len(prepared_data["travelled_trajectories"][i]['px']) > 0:
            if store_footprint_handles:
                # vehicle number
                h = plt.text(prepared_data["travelled_trajectories"][i]["px"][traj_sample_idx] - 
                            prepared_data["planners"][i]["parameters"]["veh_width"]/4, 
                        prepared_data["travelled_trajectories"][i]["py"][traj_sample_idx] + 
                            prepared_data["planners"][i]["parameters"]["veh_height"]/4, 
                        str(i), color=prepared_data["vehicle_colors"][i], fontsize=10, ha='center', 
                        va='center')
                vehicle_nb_handles[i] = h

                # vehicle state
                local_idx = min(traj_sample_idx, len(prepared_data["travelled_states"][i]) - 1)
                state_string = str(prepared_data["travelled_states"][i][local_idx])
                if "WAIT" in state_string and "FREE" not in state_string:
                    state_string += " (" + str(data["agents"][i]["travelled_blocking_agent_idx"][local_idx]) + ")"
                h = plt.text(0.66, 0.9 - i*0.05, state_string, 
                        color=prepared_data["vehicle_colors"][i], fontsize=8, ha='left', 
                        va='top', transform=fig.transFigure)
                vehicle_state_handles[i] = h
            else:
                # update position of vehicle nb
                vehicle_nb_handles[i].set_position(
                    [prepared_data["travelled_trajectories"][i]["px"][traj_sample_idx] - 
                     prepared_data["planners"][i]["parameters"]["veh_width"]/4, 
                     prepared_data["travelled_trajectories"][i]["py"][traj_sample_idx] + 
                     prepared_data["planners"][i]["parameters"]["veh_height"]/4])
                # update text of vehicle state
                local_idx = min(traj_sample_idx, len(prepared_data["travelled_states"][i]) - 1)
                state_string = str(prepared_data["travelled_states"][i][local_idx])
                if "WAIT" in state_string and "FREE" not in state_string:
                    state_string += " (" + str(data["agents"][i]["travelled_blocking_agent_idx"][local_idx]) + ")"
                vehicle_state_handles[i].set_text(state_string)


    set_env_plot_limits(prepared_data["environments"][0])
    plt.xticks([])
    plt.yticks([])

    plt.title(f"t = {original_T:.3f} s")

    # shift axes to the left
    ax = plt.gca()
    ax.set_position([0.02, 0.1, 0.65, 0.8])

    return handles_to_clear, footprint_handles, vehicle_nb_handles, vehicle_state_handles

def create_multi_mover_motion_video(data, **kwargs):
    fps = kwargs.get('fps', 25)
    dpi = kwargs.get('dpi', 200)
    mp4_dt = 1.0/fps
    total_time = max([d["travelled_trajectory"]["Tf"] for d in data["agents"]]) + 1.0

    start_time = 0.0
    stop_time = total_time

    import matplotlib.animation as animation
    from matplotlib.animation import FFMpegWriter

    # prepare data
    vehicle_colors = ['b', 'green', 'k', 'orange', 'purple', 'powderblue', 'hotpink', 'gold', 'teal', 'indigo', 'grey', 'coral', 'tomato']
    vehicle_colors = vehicle_colors + vehicle_colors + vehicle_colors
    planners = [d["planner"] for d in data["agents"]]
    travelled_trajectories = [d["travelled_trajectory"] for d in data["agents"]]
    planned_trajectories = [d["planned_trajectories"] for d in data["agents"]]
    planned_times = [d["planned_times"] for d in data["agents"]]
    travelled_final_destinations = [d["travelled_final_destinations"] for d in data["agents"]]
    travelled_states = [d["travelled_states"] for d in data["agents"]]
    travelled_blocking_agent_idx = [d["travelled_blocking_agent_idx"] for d in data["agents"]]
    environments = [planner["environment"] for planner in planners]
    corridor_sequences = [d["planned_corridor_sequences"] for d in data["agents"]]
    agent_addresses = [d["memory_address"] for d in data["agents"]]
    claimable_destinations_info = data["claimed_destinations_info"]
    SHOW_CORRIDORS = 0
    SHOW_INTERSECTIONS = 0
    SHOW_CLAIMED_CELLS = 1
    SHOW_CURRENT_PLANS = 0
    SHOW_TRAVELLED_TRAJECTORIES = 0
    prepared_data = {
        "vehicle_colors": vehicle_colors,
        "planners": planners,
        "travelled_trajectories": travelled_trajectories,
        "planned_trajectories": planned_trajectories,
        "planned_times": planned_times,
        "travelled_final_destinations": travelled_final_destinations,
        "travelled_states": travelled_states,
        "travelled_blocking_agent_idx": travelled_blocking_agent_idx,
        "environments": environments,
        "corridor_sequences": corridor_sequences,
        "agent_addresses": agent_addresses,
        "claimable_destinations_info": claimable_destinations_info,
        "SHOW_CORRIDORS": SHOW_CORRIDORS,
        "SHOW_INTERSECTIONS": SHOW_INTERSECTIONS,
        "SHOW_CLAIMED_CELLS": SHOW_CLAIMED_CELLS,
        "SHOW_CURRENT_PLANS": SHOW_CURRENT_PLANS,
        "SHOW_TRAVELLED_TRAJECTORIES": SHOW_TRAVELLED_TRAJECTORIES
    }

    writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=['-pix_fmt', 'yuv420p'])
    fig = plt.figure(dpi=dpi)
    # show environment
    for i in range(len(environments)):
        show_environment(environments[i], obstacle_color=vehicle_colors[i], obstacles_alpha=1.0)
    
    handles = {
    'handles_to_clear' : [],
    'footprint_handles' : [],
    'vehicle_nb_handles' : [],
    'vehicle_state_handles' : [],
    }

    def update(frame, state):
        if frame % 5 == 0:
            print(f"creating figure at t = {frame*mp4_dt:.3f} ({frame*mp4_dt/total_time*100:.2f}%)")
        (state['handles_to_clear'],
        state['footprint_handles'],
        state['vehicle_nb_handles'],
        state['vehicle_state_handles']) = create_multi_mover_motion_snapshot(
            data, prepared_data,
            state['handles_to_clear'],
            state['footprint_handles'],
            state['vehicle_nb_handles'],
            state['vehicle_state_handles'],
            T=mp4_dt*frame, fig=fig, kwargs=kwargs)
        return fig
    
    anim = animation.FuncAnimation(fig, update, 
                                    range(int((start_time/mp4_dt)), 
                                          int((stop_time/mp4_dt))), 
                                    repeat=False,
                                    fargs=(handles,))
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

def visualize_task_completion_pie_chart(data):
    agent_states = [data["agents"][i]["travelled_states"] for i in range(len(data["agents"]))]

    # loop over all agents
    agent_timings = []
    for agent_idx in range(len(agent_states)):
        timings = {
            'idling': agent_states[agent_idx].count("IDLING") * data["simulation_time_step"],
            'moving (final)': agent_states[agent_idx].count("MOVING_TO_FINAL_DESTINATION") * data["simulation_time_step"],
            'moving (waiting point)': agent_states[agent_idx].count("MOVING_TO_WAITING_POINT") * data["simulation_time_step"],
            'waiting (waiting point)': agent_states[agent_idx].count("WAITING_AT_INTERSECTION") * data["simulation_time_step"],
            'waiting (destination)': agent_states[agent_idx].count("WAITING_FOR_FREE_DESTINATION") * data["simulation_time_step"],
        }
        agent_timings.append(timings)
        if sum(timings.values()) == 0:
            timings['idling'] = 1.0
        if sum([t for t, k in zip(timings.values(), timings.keys()) if not k == "idling"]) == 0:
            timings['moving (final)'] = 1.0e-5
        
        show_pie(timings, agent_idx, no_idling=False)
        show_pie(timings, agent_idx, no_idling=True)

    # create average agent
    avg_timings = {
        'idling': np.mean([timings['idling'] for timings in agent_timings]),
        'moving (final)': np.mean([timings['moving (final)'] for timings in agent_timings]),
        'moving (waiting point)': np.mean([timings['moving (waiting point)'] for timings in agent_timings]),
        'waiting (waiting point)': np.mean([timings['waiting (waiting point)'] for timings in agent_timings]),
        'waiting (destination)': np.mean([timings['waiting (destination)'] for timings in agent_timings]),
    }

    # create single figure with all axes
    nb_pies = len(agent_timings) + 1
    rows = int(np.sqrt(nb_pies))
    cols = rows
    counter = 0
    while counter < 100 and rows * cols != nb_pies:
        if rows*cols < nb_pies:
            cols += 1
        else:
            if rows > 1:
                rows -= 1
            else:
                cols -= 1

        counter += 1

    fig, axs = plt.subplots(rows, cols, figsize=(6*cols, 6*rows), squeeze=False)
    for agent_idx in range(len(agent_timings)):
        timings = agent_timings[agent_idx]
        ax = axs[agent_idx // cols, agent_idx % cols]
        show_pie(timings, agent_idx, ax=ax, no_idling=True, save=False, legend=agent_idx==0)
    # show average agent
    ax = axs[rows - 1, cols - 1]
    show_pie(avg_timings, "avg", ax=ax, no_idling=True, save=False, legend=False)
    plt.savefig("post-process/multi_mover_simulator/animations/pie-charts/task_completion_pie_chart_all_agents.png")

def show_pie(timings, agent_idx, ax=None, no_idling=False, save=True, legend=True):
    timings = timings.copy()
    if no_idling:
        timings['idling'] = 0

    colors = {
        "idling": "lightgray",
        "moving (final)": "green",
        "moving (waiting point)": "darkseagreen",
        "waiting (waiting point)": "firebrick",
        "waiting (destination)": "rosybrown",
    }

    if ax is None:
        fig, axs = plt.subplots(1, 1, figsize=(8, 6), squeeze=False)
        ax = axs[0,0]

    # create pie chart
    labels = list(timings.keys())
    sizes = list(timings.values())
    pie_colors = [colors[label] for label in labels]
    ax.pie(sizes, labels=None, colors=pie_colors, autopct='%1.1f%%', textprops={'fontsize':12}, startangle=140)
    ax.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
    ax.set_title(f'Agent {agent_idx}')

    # add a legend
    if legend:
        legend_patches = [mpatches.Patch(color=colors[label], label=label) for label in labels]
        ax.legend(handles=legend_patches, loc='upper right', bbox_to_anchor=(1.2, 1))

    plt.tight_layout()
    if save:
        appendix = "_no_idling" if no_idling else ""
        plt.savefig(f"post-process/multi_mover_simulator/animations/pie-charts/task_completion_pie_chart_{agent_idx}{appendix}.png")

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
    fig, ax = plt.subplots(figsize=(12, 6), squeeze=False)
    computation_times = np.array(computation_times)
    time_grid = np.arange(0, len(computation_times)) * data["simulation_time_step"]
    colors = np.where(computation_times < 10, 'blue', 'red')
    ax.bar(time_grid, computation_times, color=colors, width=data["simulation_time_step"]*0.8, alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Computation Time (ms)")
    plt.xlim(0, total_time)
    plt.ylim(0, 100)
    plt.savefig("post-process/multi_mover_simulator/animations/buffered_samples_over_time.png")
    
def visualize_profilers(data):
    profiler_colors = [[0.8, 0.2, 0.2], [0.2, 0.8, 0.2], [0.2, 0.2, 0.8], [0.6, 0.6, 0.0], [0.6, 0.0, 0.6], [0.0, 0.6, 0.6]]
    profilers = data["profilers"]
    
    for (p, pp) in zip(profilers.keys(), profilers.values()):
        print(f"{p}: {pp.keys()}")

    if len(profilers) > len(profiler_colors):
        raise ValueError("Not enough colors defined for the number of profilers. Please add more colors to the profiler_colors list.")
    
    nb_pies = len(profilers) + 1
    rows = int(np.sqrt(nb_pies))
    cols = rows
    counter = 0
    while counter < 100 and rows * cols != nb_pies:
        if rows*cols < nb_pies:
            cols += 1
        elif rows > 1:
            rows -= 1
        else:
            cols -= 1

        counter += 1

    fig, axs = plt.subplots(rows, cols, figsize=(3*cols, 3*rows), squeeze=False)
    main_averages = []
    main_labels = []
    main_colors = []
    for profiler_idx, profiler_name in zip(range(len(profilers)), profilers.keys()):
        c = profiler_colors[profiler_idx % len(profiler_colors)]
        averages = [a["total_ms"] for k, a in zip(profilers[profiler_name].keys(), profilers[profiler_name].values())]
        labels = profilers[profiler_name].keys()
        colors = [[min(cc*s, 1.0) for cc in c] for s in np.linspace(1.0, 1.6, len(averages))]
        ax = axs[profiler_idx // cols, profiler_idx % cols]
        ax.pie(averages, labels=labels, colors=colors, autopct='%1.1f%%',
               textprops={'fontsize':6}, startangle=140)
        # ax.legend(labels)
        ax.set_title(f'{profiler_name}', fontsize=8)

        main_averages += averages
        main_labels += labels
        main_colors += colors

    # show main profiler
    ax = axs[-1, -1]
    ax.cla()
    ax.pie([profilers["ProcessPotentialVirtualCollision"]["ProcessPotentialVirtualCollision"]["total_ms"],
            profilers["CheckForCollision"]["CheckForCollision"]["total_ms"],
            profilers["DealWithCollision"]["DealWithCollision"]["total_ms"]], 
            labels=["ProcessPotentialVritualCollisions", 
                    "CheckForCollisions",
                    "DealWithCollision"], autopct='%1.1f%%',
           textprops={'fontsize':6}, startangle=140)

    fig.subplots_adjust(bottom=0.1)  # adjust these as needed
    from matplotlib.patches import Patch
    handles = [Patch(facecolor=col, label=label) for col, label in zip(main_colors, main_labels)]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, 0.1),
            ncol=min(len(main_labels), 6), fontsize=6)

    # ax.set_title(f'Global profiling', fontsize=8)

    print(profilers["SimulateAllTasks"])

    plt.show()
    return
    
    nested_profiler = profilers["SimulateAllTasks"]

    # add SimulateSingleStep sub-parts (with overhead)
    simulate_single_step_overhead = nested_profiler["SimulateSingleStep"]["total_ms"] - \
        sum([profilers["SimulateSingleStep"][k]["total_ms"] for k in profilers["SimulateSingleStep"].keys()])
    nested_profiler["SimulateSingleStep"] = profilers["SimulateSingleStep"]
    nested_profiler["SimulateSingleStep"]["Overhead"] = {"total_ms" : max(0, simulate_single_step_overhead)}

    # add ProcessPotentialNewCollisions sub-parts (with overhead)
    process_potential_new_collisions_overhead = nested_profiler["SimulateSingleStep"]["ProcessPotentialNewCollisions"]["total_ms"] - \
        sum([profilers["SimulateSingleStep"]["ProcessPotentialNewCollisions"]["total_ms"] for k in profilers["SimulateSingleStep"]["ProcessPotentialNewCollisions"].keys()])
    nested_profiler["SimulateSingleStep"]["ProcessPotentialNewCollisions"] = {
        "CheckForCollision": profilers["CheckForCollision"]["CheckForCollision"],
        "DealWithCollision": profilers["DealWithCollision"]["DealWithCollision"],
        "ProcessPotentialVirtualCollision": profilers["ProcessPotentialVirtualCollision"]["ProcessPotentialVirtualCollision"],
        "Overhead": max(0, process_potential_new_collisions_overhead)
    }

    print("Nested profiler structure:")
    for (k, v) in nested_profiler.items():
        if isinstance(v, dict):
            for (kk, vv) in v.items():
                if isinstance(vv, dict):
                    for (kkk, vvv) in vv.items():
                        print(f"\t\t\t{kkk}: {vvv}")
                else:
                    print(f"\t\t{kk}: {vv}")
        else:
            print(f"\t{k}: {v}")

    # main_colors = []
    # main_labels = []
    # main_values = []
    # idx = 0

    # # start with copying everything that will not be split
    # for k, v in profilers["SimulateAllTasks"]:
    #     if (k == "SimulateSingleStep"):
    #         continue

    #     main_colors.append(colors[idx])
    #     main_labels.append(k)
    #     main_values = v["total_ms"]
    #     idx += 1
    
    # # compute the SimulateSingleStep overhead
    # overhead = profilers["SimulateAllTasks"]["SimulateSingleStep"]["total_ms"] - \
    #     sum([profilers["SimulateSingleStep"][k]["total_ms"] for k in profilers["SimulateSingleStep"].keys()])


# Load the data
file = "build/output/multi_mover_simulator.json"

with open(file) as f:
    data = json.load(f)

# visualize_task_completion(data)
# visualize_task_completion_pie_chart(data)
# visualize_computation_time_per_simulation_step(data)
visualize_profilers(data)
# exit()
# create_multi_mover_motion_video(data, fps=25, dpi=300)