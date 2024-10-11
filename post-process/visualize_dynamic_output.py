import matplotlib.pyplot as plt
from visualization_helpers import *


def visualize_dynamic_solution(data, T=-1, counter=0):
    if T == -1:
        T = data["travelled_trajectory"]["Tf"]
    
    # find the index of the last trajectory to (at least partially) show
    traj_idx = 0
    while len(data["replanning_times"]) > traj_idx and T > data["replanning_times"][traj_idx]:
        traj_idx += 1

    # find the index in the travelled trajectory
    travelled_traj_sample_idx = int(T/data["travelled_trajectory"]["dt"])

    print(f"traj_idx = {traj_idx}")
    print(f"travelled_traj_sample_idx = {travelled_traj_sample_idx}")

    plt.figure()

    # Show the environment
    env = data["environment"]
    show_environment(env)
    
    # Show the corridor sequence
    corridors = data["previous_corridor_sequences"][traj_idx]
    show_corridors(corridors)

    # Show traces of moving obstacles
    for obs in data["moving_obstacles"]:
        # show_trajectory(obs["travelled_trajectory"], 'darkred', True, 
        #                 obs["width"], obs["height"])
        show_moving_obstacle(obs, travelled_traj_sample_idx, 'darkred')
        
    # show all previously computed trajectories
    for i in range(min(len(data["previous_trajectories"]), traj_idx+1)):
        show_trajectory(data["previous_trajectories"][i], 'gray', True, 
                        data["motion_planner"]["parameters"]["veh_width"], 
                        data["motion_planner"]["parameters"]["veh_height"],
                        with_footprints=False)
        
    # Show the trajectory of the mover
    show_trajectory(data["travelled_trajectory"], 'blue', True, 
                    data["motion_planner"]["parameters"]["veh_width"], 
                    data["motion_planner"]["parameters"]["veh_height"],
                    with_footprints=True, 
                    nb_samples_to_show=travelled_traj_sample_idx)
    
    set_env_plot_limits(env)
    if counter is None:
        plt.savefig(f"../post-process/figures/dynamic_solution_traj.png", dpi=200)
    else:
        plt.savefig(f"../post-process/figures/animation/dynamic_solution_traj_{counter}.png", dpi=200)
    plt.close()


file = "output/dynamic_solution.json"
with open(file) as f:
    data = json.load(f)

visualize_dynamic_solution(data, -1, None)

total_time = data["travelled_trajectory"]["Tf"]
counter = 0
curr_time = 0.0
while curr_time < total_time:
    print(f"creating figure at t = {curr_time:.3f} ({curr_time/total_time*100:.2f}%)")
    visualize_dynamic_solution(data, curr_time, counter)
    counter += 1
    curr_time += data["travelled_trajectory"]["dt"]