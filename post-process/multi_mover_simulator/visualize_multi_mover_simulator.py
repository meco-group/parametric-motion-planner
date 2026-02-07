import sys
sys.path.append('post-process/')
from visualization_helpers import *
import matplotlib.patches as mpatches
import numpy as np
from collections.abc import Iterable
from matplotlib.artist import Artist
import time

def create_multi_mover_motion_snapshot(data, prepared_data, handles_to_clear, footprint_handles, vehicle_nb_handles, vehicle_state_handles, T, fig=None, **kwargs):
    print_profiler = False

    max_T = max([traj["Tf"] for traj in prepared_data["travelled_trajectories"]])

    original_T = T
    T = max(0, min(max_T, T))
    traj_sample_idx = int(T/prepared_data["travelled_trajectories"][0]["dt"])  

    # plt.figure(fig.number, dpi=kwargs.get('dpi', 200))
    # plt.clf()

    for h in handles_to_clear:
        if isinstance(h, Iterable) and not isinstance(h, Artist):
            for hh in h:
                if hh is not None:
                    hh.remove()
        else:
            h.remove()

    handles_to_clear = []

    # show corridors
    time_start = time.time()
    if prepared_data["SHOW_CORRIDORS"]:
        corridors_to_show = prepared_data.get("corridors_to_show", list(range(len(prepared_data["corridor_sequences"]))))
        for i in corridors_to_show:
            if (prepared_data["travelled_states"][i][min(len(prepared_data["travelled_states"][i])-1, traj_sample_idx)] == "IDLING"):
                continue

            replan_idx = -1
            while replan_idx < len(prepared_data["planned_times"][i]) - 1 and \
                    T > prepared_data["planned_times"][i][replan_idx+1]:
                replan_idx += 1
            if replan_idx == -1:
                continue
            corridors = prepared_data["corridor_sequences"][i][replan_idx]
            h = show_corridors(corridors, color=prepared_data["vehicle_colors"][(i) % len(prepared_data["vehicle_colors"])], max_alpha=prepared_data.get("corridors_alpha",0.2))
            handles_to_clear.extend(h)
    time_end = time.time()
    if (print_profiler): print(f"Time to show corridors: {time_end - time_start:.4f} seconds")

    # show intersections
    time_start = time.time()
    if prepared_data["SHOW_INTERSECTIONS"]:
        for log in data["intersection_logs"]:
            if (T >= log["time_entering"] and T <= log["time_leaving"]):
                # show_corridors({"sequence":[log["intersection"]]}, color='red')

                h = show_corridor_union(log["intersection"], color='red')
                handles_to_clear.extend(h)
    time_end = time.time()
    if (print_profiler): print(f"Time to show intersections: {time_end - time_start:.4f} seconds")
                

    # show claimed cells
    time_start = time.time()
    if prepared_data["SHOW_CLAIMED_CELLS"]:
        for (dest_name, destination) in prepared_data["claimable_destinations_info"][min(len(prepared_data["claimable_destinations_info"])-1, traj_sample_idx)].items():
            cell = destination["location"]
            color = 'lightgray'
            if destination["claimed"]:
                agent_memory_address = destination["claimed_by"]
                if agent_memory_address in prepared_data["agent_addresses"]:
                    agent_idx = prepared_data["agent_addresses"].index(agent_memory_address)
                    color = prepared_data["vehicle_colors"][(agent_idx) % len(prepared_data["vehicle_colors"])]
            
            cell_width = prepared_data["environments"][0]["cell_width"]
            cell_height = prepared_data["environments"][0]["cell_height"]
            corridor = {"x_min":cell["x"]*cell_width,
                        "x_max":(cell["x"]+1)*cell_width,
                        "y_min":cell["y"]*cell_height,
                        "y_max":(cell["y"]+1)*cell_height}    
            h = show_corridors({"sequence":[corridor]}, color=color, max_alpha=0.5)
            handles_to_clear.extend(h)

            if prepared_data["SHOW_DESTINATION_NAMES"]:
                # replace SN by $m_{N}$ where N is a number
                dest_name_mod = dest_name
                if dest_name.startswith("S"):
                    dest_name_mod = r"$m_{" + dest_name[1:] + "}$"
                h = plt.text((cell["x"]+0.5)*cell_width, (cell["y"]+0.7)*cell_height, dest_name_mod,
                            color='black', fontsize=8, ha='right', va='bottom', zorder=1)
                handles_to_clear.append(h)
    time_end = time.time()
    if (print_profiler): print(f"Time to show claimed cells: {time_end - time_start:.4f} seconds")

    # show station locations
    if prepared_data.get("SHOW_STATION_LOCATIONS", False):
        time_start = time.time()
        for (dest_name, destination) in prepared_data["claimable_destinations_info"][min(len(prepared_data["claimable_destinations_info"])-1, traj_sample_idx)].items():
            cell = destination["location"]
            cell_width = prepared_data["environments"][0]["cell_width"]
            cell_height = prepared_data["environments"][0]["cell_height"]
            plt.plot((cell["x"]+0.5)*cell_width, (cell["y"]+0.5)*cell_height, 'o',
                     color='black', markersize=5, markeredgecolor='white', markeredgewidth=0, zorder=999)
        time_end = time.time()
        if (print_profiler): print(f"Time to show station locations: {time_end - time_start:.4f} seconds")

    # show current plans completely
    time_start = time.time()
    if prepared_data["SHOW_CURRENT_PLANS"]:
        for i in range(len(prepared_data["planners"])):
            # if the agent is idling, do not show anything
            if (prepared_data["travelled_states"][i][min(len(prepared_data["travelled_states"][i])-1, traj_sample_idx)] == "IDLING"):
                continue
            curr_plan_idx = -1
            while curr_plan_idx < len(prepared_data["planned_times"][i]) - 1 and T > prepared_data["planned_times"][i][curr_plan_idx+1]:
                curr_plan_idx += 1

            if curr_plan_idx == -1:
                continue

            # h, footprints = show_trajectory(prepared_data["planned_trajectories"][i][curr_plan_idx], 'gray', show_markers=False)
            h, footprints = show_trajectory(prepared_data["planned_trajectories"][i][curr_plan_idx], prepared_data['vehicle_colors'][(i) % len(prepared_data["vehicle_colors"])], show_markers=False, linewidth=0.3)
            handles_to_clear.extend(h)
            handles_to_clear.extend(footprints)                
    time_end = time.time()
    if (print_profiler): print(f"Time to show current plans: {time_end - time_start:.4f} seconds")

    # show travelled trajectories
    time_start = time.time()
    augmented_data = kwargs['kwargs'].get('augmented_data', False)
    nb_agents = len(prepared_data["planners"])
    store_footprint_handles = len(footprint_handles) == 0
    if store_footprint_handles:
        footprint_handles = [[] for _ in range(len(prepared_data["planners"]))]
        vehicle_nb_handles = [None for _ in range(len(prepared_data["planners"]))]
        vehicle_state_handles = [None for _ in range(len(prepared_data["planners"]))]
    for i in range(len(prepared_data["planners"])):
        nb_of_unfaded_samples = kwargs['kwargs'].get("nb_of_unfaded_samples", 100) if prepared_data["SHOW_TRAVELLED_TRAJECTORIES"] else 0
        if prepared_data.get("PRINT_SHOW_TRAVELLED_TRAJECTORIES_PROGRESS", False):
            print(f"Showing travelled trajectory for agent {i}/{len(prepared_data['planners'])}")
        if original_T >= prepared_data["travelled_trajectories"][i]["Tf"]:
            normal_unfaded_time = nb_of_unfaded_samples*prepared_data["travelled_trajectories"][i]["dt"]
            unfaded_time = max(0, normal_unfaded_time - (original_T - prepared_data["travelled_trajectories"][i]["Tf"]))
            nb_of_unfaded_samples = int(unfaded_time/prepared_data["travelled_trajectories"][i]["dt"]) if nb_of_unfaded_samples >= 0 else -1
        i_offset = int(-nb_agents/2) if augmented_data and i >= nb_agents/2 else 0
        [h, footprints] = show_trajectory(prepared_data["travelled_trajectories"][i], 
                        prepared_data["vehicle_colors"][(i + i_offset) % len(prepared_data["vehicle_colors"])], 
                        with_trace=prepared_data["SHOW_TRAVELLED_TRAJECTORIES"],
                        width=prepared_data["planners"][i]["parameters"]["veh_width"],
                        height=prepared_data["planners"][i]["parameters"]["veh_height"],
                        with_footprints=store_footprint_handles, 
                        nb_samples_to_show=min(traj_sample_idx, len(prepared_data["travelled_trajectories"][i]["px"])),
                        virtual_final_footprint=False or (augmented_data and i >= nb_agents/2), 
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
                
        if prepared_data["SHOW_PAST_GHOSTS"]:
            nb_ghosts = 10
            ghost_time_gap = 0.02
            for g in range(nb_ghosts):
                xx = prepared_data["travelled_trajectories"][i]["px"]
                yy = prepared_data["travelled_trajectories"][i]["py"]
                dt = prepared_data["travelled_trajectories"][i]["dt"]

                ghost_time = max(T - (g+1)*ghost_time_gap, 0)
                ghost_sample_idx = int(ghost_time/dt)
                if ghost_sample_idx >= len(xx):
                    ghost_sample_idx = len(xx) - 1

                h = plot_vehicle_footprint(plt.gca(), 
                    xx[ghost_sample_idx], 
                    yy[ghost_sample_idx], 
                    prepared_data["planners"][i]["parameters"]["veh_width"], 
                    prepared_data["planners"][i]["parameters"]["veh_height"], 
                    color=prepared_data["vehicle_colors"][(i + i_offset) % len(prepared_data["vehicle_colors"])],
                    max_alpha=0.2*(1 - g/nb_ghosts),
                    zorder=1)
                handles_to_clear.extend(h)
                               
        if len(prepared_data["travelled_trajectories"][i]['px']) > 0:
            h = plt.plot(prepared_data["travelled_final_destinations"][i][min(traj_sample_idx, len(prepared_data["travelled_final_destinations"][i])-1)]["x"],
                    prepared_data["travelled_final_destinations"][i][min(traj_sample_idx, len(prepared_data["travelled_final_destinations"][i])-1)]["y"], 'o',
                    color=prepared_data["vehicle_colors"][(i) % len(prepared_data["vehicle_colors"])], markersize=5)
            handles_to_clear.extend(h)
    time_end = time.time()
    if (print_profiler): print(f"Time to show travelled trajectories: {time_end - time_start:.4f} seconds")

    # show vehicle numbers and states
    time_start = time.time()
    for i in range(len(prepared_data["planners"])):
        i_offset = int(-nb_agents/2) if augmented_data and i >= nb_agents/2 else 0
        if len(prepared_data["travelled_trajectories"][i]['px']) > 0:
            if store_footprint_handles:
                # vehicle number
                if prepared_data.get("SHOW_VEHICLE_NUMBERS", True):
                    h = plt.text(prepared_data["travelled_trajectories"][i]["px"][traj_sample_idx] - 
                                prepared_data["planners"][i]["parameters"]["veh_width"]/4, 
                            prepared_data["travelled_trajectories"][i]["py"][traj_sample_idx] + 
                                prepared_data["planners"][i]["parameters"]["veh_height"]/4, 
                            str(i + i_offset), color=prepared_data["vehicle_colors"][(i + i_offset) % len(prepared_data["vehicle_colors"])], fontsize=10, ha='center', 
                            va='center')
                    vehicle_nb_handles[i] = h

                # vehicle state
                if prepared_data["SHOW_VEHICLE_STATES"]:
                    local_idx = min(traj_sample_idx, len(prepared_data["travelled_states"][i]) - 1)
                    state_string = str(prepared_data["travelled_states"][i][local_idx])
                    if "WAIT" in state_string and "FREE" not in state_string:
                        state_string += " (" + str(data["agents"][i]["travelled_blocking_agent_idx"][local_idx]) + ")"
                    h = plt.text(0.7, 0.9 - i*0.05, state_string, 
                            color=prepared_data["vehicle_colors"][(i) % len(prepared_data["vehicle_colors"])], fontsize=8, ha='left', 
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
                if prepared_data["SHOW_VEHICLE_STATES"]:
                    local_idx = min(traj_sample_idx, len(prepared_data["travelled_states"][i]) - 1)
                    state_string = str(prepared_data["travelled_states"][i][local_idx])
                    if "WAIT" in state_string and "FREE" not in state_string:
                        state_string += " (" + str(data["agents"][i]["travelled_blocking_agent_idx"][local_idx]) + ")"
                    vehicle_state_handles[i].set_text(state_string)
    time_end = time.time()
    if (print_profiler): print(f"Time to show vehicle numbers and states: {time_end - time_start:.4f} seconds")

    time_start = time.time()
    set_env_plot_limits(prepared_data["environments"][0])
    plt.xticks([])
    plt.yticks([])

    if prepared_data["SHOW_TITLE"]:
        plt.title(f"t = {original_T:.3f} s")

    # shift axes to the left
    if prepared_data["SHOW_VEHICLE_STATES"]:
        ax = plt.gca()
        ax.set_position([0.02, 0.1, 0.65, 0.8])
    time_end = time.time()
    if (print_profiler): print(f"Time to finalize snapshot: {time_end - time_start:.4f} seconds")

    return handles_to_clear, footprint_handles, vehicle_nb_handles, vehicle_state_handles

def create_multi_mover_motion_video(data, **kwargs):
    fps = kwargs.get('fps', 25)
    dpi = kwargs.get('dpi', 200)
    mp4_dt = 1.0/fps
    total_time = max([d["travelled_trajectory"]["Tf"] for d in data["agents"]]) + 3.0

    start_time = kwargs.get("start_time", 0.0)
    stop_time = kwargs.get("stop_time", total_time)

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
    SHOW_CURRENT_PLANS = 1
    SHOW_TRAVELLED_TRAJECTORIES = 0
    SHOW_VEHICLE_STATES = 1
    SHOW_DESTINATION_NAMES = 0
    SHOW_PAST_GHOSTS = 0
    SHOW_TITLE = 0
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
        "SHOW_TRAVELLED_TRAJECTORIES": SHOW_TRAVELLED_TRAJECTORIES,
        "SHOW_VEHICLE_STATES": SHOW_VEHICLE_STATES,
        "SHOW_DESTINATION_NAMES": SHOW_DESTINATION_NAMES,
        "SHOW_PAST_GHOSTS": SHOW_PAST_GHOSTS,
        "SHOW_TITLE": SHOW_TITLE
    }

    writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=['-pix_fmt', 'yuv420p'])
    fig = plt.figure(dpi=dpi, figsize=(7, 4.8))
    # show environment
    for i in range(len(environments) if len(planners) < 25 else 1):
        show_environment(environments[i], obstacle_color=vehicle_colors[i%len(vehicle_colors)], obstacles_alpha=1.0)
    
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
    animation_file = kwargs.get("animation_file", "")
    if animation_file == "":
        animation_file = f"post-process/multi_mover_simulator/animations/animation.mp4"
    anim.save(animation_file, writer=writer)

# Load the data
file = "build/my_multi_mover_simulator_output.json"
with open(file) as f:
    data = json.load(f)

# create animation
create_multi_mover_motion_video(data, fps=25, dpi=200)
