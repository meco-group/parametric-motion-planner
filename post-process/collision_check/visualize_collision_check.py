import sys
sys.path.append('post-process/')
from visualization_helpers import *

def visualize_collision_check(data, output_folder, **kwargs):

    vehicle_colors = ['b', 'green', 'k', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    counter = 0
    for iteration in data["iterations"]:
        plt.figure()

        planners = iteration["vehicle_planners"]
        for i in range(len(planners)):

            # show environment
            env = iteration["vehicle_planners"][i]["environment"]
            show_environment(env, obstacle_color=vehicle_colors[i], obstacles_only=(i > 0), obstacles_alpha=0.5)


            # show corridors
            if "show_corridors" in kwargs and kwargs["show_corridors"]:
                corridors = planners[i]["corridor_sequence"]
                show_corridors(corridors, color=vehicle_colors[i])

            trajectory = planners[i]["trajectory"]
            show_trajectory(trajectory, vehicle_colors[i], with_trace=True,
                            with_footprints=True, 
                            width=planners[i]["parameters"]["veh_width"],
                            height=planners[i]["parameters"]["veh_height"])

        for i in range(len(iteration["collisions"])):
            point = iteration["collisions"][i]["point_of_collision"][i]
            if point["x"] >= 0 and point["y"] >= 0:
                plt.plot(point["x"], point["y"], 'rx')

        set_env_plot_limits(env)

        plt.xticks([])
        plt.yticks([])
        plt.savefig(output_folder + "collision_check_" + str(counter) + ".png", dpi=300)
        counter += 1
    
    plt.show()

def create_multi_mover_motion_snapshot(iteration, T, fig=None, **kwargs):
    vehicle_colors = ['b', 'green', 'k', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    planners = iteration["vehicle_planners"]
    trajectories = [planner["trajectory"] for planner in planners]
    environments = [planner["environment"] for planner in planners]
    max_T = max([traj["Tf"] for traj in trajectories])

    original_T = T
    T = max(0, min(max_T, T))
    traj_sample_idx = int(T/trajectories[0]["dt"])

    plt.figure(fig.number)
    plt.clf()

    # show environment
    for i in range(len(environments)):
        show_environment(environments[i], obstacle_color=vehicle_colors[i], obstacles_only=(i > 0), obstacles_alpha=0)

    # show trajectories
    for i in range(len(planners)):
        nb_of_unfaded_samples = 100*1.0e5
        if original_T > trajectories[i]["Tf"]:
            normal_unfaded_time = nb_of_unfaded_samples*trajectories[i]["dt"]
            unfaded_time = max(0, normal_unfaded_time - (original_T - trajectories[i]["Tf"]))
            nb_of_unfaded_samples = int(unfaded_time/trajectories[i]["dt"])
        show_trajectory(trajectories[i], vehicle_colors[i], with_trace=True,
                        width=planners[i]["parameters"]["veh_width"],
                        height=planners[i]["parameters"]["veh_height"],
                        with_footprints=True, 
                        nb_samples_to_show=min(traj_sample_idx, len(trajectories[i]["px"])),
                        virtual_final_footprint=False, 
                        unfaded_nb_samples=nb_of_unfaded_samples, 
                        show_initial_footprint_if_showing_footprints=False,
                        trace_alpha=0.1)
        plt.plot(trajectories[i]["px"][0], trajectories[i]["py"][0], 'o', 
                 color=vehicle_colors[i], markersize=5)
        plt.plot(trajectories[i]["px"][-1], trajectories[i]["py"][-1], 'o', 
                 color=vehicle_colors[i], markersize=5)
        
    # show collisions
    for collision in iteration["collisions"]:
        if T >= collision["collision_time"]:
            point = collision["point_of_collision"]
            plt.plot(point["x"], point["y"], 'rx')

            for i in range(2):
                pos = collision["vehicle_positions"][i]
                veh_idx = collision["vehicle_indices"][i]
                width = planners[veh_idx]["parameters"]["veh_width"]
                height = planners[veh_idx]["parameters"]["veh_height"]
                color = vehicle_colors[veh_idx]

                plot_vehicle_footprint(plt.gca(), pos["x"], pos["y"], width, 
                                       height, virtual_position=False, 
                                       color=color, max_alpha=0.2, zorder=1)


    set_env_plot_limits(environments[0])
    plt.xticks([])
    plt.yticks([])

    return

def create_multi_mover_motion_video(data, **kwargs):
    iteration_nb = kwargs.get("iteration", -1)
    fps = kwargs.get('fps', 25)
    mp4_dt = 1.0/fps
    total_time = max([p["trajectory"]["Tf"] for p in data["iterations"][iteration_nb]["vehicle_planners"]]) + 1.0

    start_time = 0.0
    stop_time = total_time

    import matplotlib.animation as animation
    from matplotlib.animation import FFMpegWriter

    writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=['-pix_fmt', 'yuv420p'])
    fig = plt.figure()
    def update(frame):
        if frame % 5 == 0:
            print(f"creating figure at t = {frame*mp4_dt:.3f} ({frame*mp4_dt/total_time*100:.2f}%)")
        create_multi_mover_motion_snapshot(data["iterations"][iteration_nb], T=mp4_dt*frame, fig=fig, kwargs=kwargs)
        return fig
    
    anim = animation.FuncAnimation(fig, update, 
                                    range(int((start_time/mp4_dt)), 
                                          int((stop_time/mp4_dt))), 
                                    repeat=False)
    name_appendix = "_" + str(iteration_nb) if iteration_nb >= 0 else "_final"
    anim.save(f"post-process/collision_check/animations/animation{name_appendix}.mp4", writer=writer)
  

# Load the data
file = "build/output/trajectory_collision_check.json"

with open(file) as f:
    data = json.load(f)

print("number of iterations: ", len(data["iterations"]))

# visualize_collision_check(data, "post-process/collision_check/figures/")
create_multi_mover_motion_video(data, iteration=0, fps=25)
create_multi_mover_motion_video(data, iteration=1, fps=25)
create_multi_mover_motion_video(data, iteration=2, fps=25)
create_multi_mover_motion_video(data, fps=25)