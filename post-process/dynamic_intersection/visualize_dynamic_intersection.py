import sys
sys.path.append('post-process/')
from visualization_helpers import *


def create_multi_mover_motion_snapshot(data, T, fig=None, **kwargs):
    vehicle_colors = ['b', 'green', 'k', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    planners = [data["vehicle_1"]["planner"], data["vehicle_2"]["planner"]]
    travelled_trajectories = [data["vehicle_1"]["travelled_trajectory"],
                              data["vehicle_2"]["travelled_trajectory"]]
    planned_trajectories = [data["vehicle_1"]["planned_trajectories"],
                            data["vehicle_2"]["planned_trajectories"]]
    planned_times = [data["vehicle_1"]["planned_times"],
                     data["vehicle_2"]["planned_times"]]
    final_destinations = [data["vehicle_1"]["final_dest"],
                          data["vehicle_2"]["final_dest"]]

    environments = [planner["environment"] for planner in planners]
    corridor_sequences = [data["vehicle_1"]["planned_corridor_sequences"],
                          data["vehicle_2"]["planned_corridor_sequences"]]
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
        plt.plot(final_destinations[i]["x"], final_destinations[i]["y"], color=vehicle_colors[i], markersize=5, label="goal")

    # show corridors
    for i in range(len(corridor_sequences)):
        corridors = corridor_sequences[i][0]
        show_corridors(corridors, color=vehicle_colors[i], max_alpha=0.5)

    # show intersection
    show_corridors({"sequence":[data["intersection"]]}, color='red')

    # show current plans completely
    for i in range(len(planners)):
        curr_plan_idx = 0
        while curr_plan_idx < len(planned_times[i]) - 1 and T > planned_times[i][curr_plan_idx+1]:
            curr_plan_idx += 1

        show_trajectory(planned_trajectories[i][curr_plan_idx], 'gray', show_markers=False)

    # show travelled trajectories
    for i in range(len(planners)):
        nb_of_unfaded_samples = 100*1.0e5
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
            plt.plot(travelled_trajectories[i]["px"][0], travelled_trajectories[i]["py"][0], 'o', 
                    color=vehicle_colors[i], markersize=5)
            plt.plot(travelled_trajectories[i]["px"][-1], travelled_trajectories[i]["py"][-1], 'o', 
                    color=vehicle_colors[i], markersize=5)

    set_env_plot_limits(environments[0])
    plt.xticks([])
    plt.yticks([])

    plt.title(f"t = {original_T:.3f} s")

    return

def create_multi_mover_motion_video(data, **kwargs):
    fps = kwargs.get('fps', 25)
    mp4_dt = 1.0/fps
    total_time = max(data["vehicle_1"]["travelled_trajectory"]["Tf"], 
                     data["vehicle_2"]["travelled_trajectory"]["Tf"]) + 1.0

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
    anim.save(f"post-process/dynamic_intersection/animations/animation.mp4", writer=writer)



# Load the data
file = "build/output/dynamic_intersection.json"

with open(file) as f:
    data = json.load(f)

create_multi_mover_motion_video(data, fps=25)