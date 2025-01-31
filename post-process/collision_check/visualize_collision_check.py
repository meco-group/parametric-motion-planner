import sys
sys.path.append('post-process/')
from visualization_helpers import *

def visualize_collision_check(data, output_folder):

    vehicle_colors = ['b', 'green', 'k', 'orange', 'green']
    counter = 0
    for iteration in data["iterations"]:
        plt.figure()

        # show environment
        env = iteration["vehicle_planners"][0]["environment"]
        show_environment(env)

        planners = iteration["vehicle_planners"]
        for i in range(len(planners)):
            corridors = planners[i]["corridor_sequence"]
            show_corridors(corridors, color=vehicle_colors[i])

            trajectory = planners[i]["trajectory"]
            show_trajectory(trajectory, vehicle_colors[i], with_trace=True,
                            with_footprints=True, 
                            width=planners[i]["parameters"]["veh_width"],
                            height=planners[i]["parameters"]["veh_height"])

        for i in range(len(iteration["point_of_collision"])):
            point = iteration["point_of_collision"][i]
            if point["x"] >= 0 and point["y"] >= 0:
                plt.plot(point["x"], point["y"], 'rx')

        set_env_plot_limits(env)

        plt.savefig(output_folder + "collision_check_" + str(counter) + ".png", dpi=300)
        counter += 1
    
    plt.show()

  

# Load the data
file = "build/output/trajectory_collision_check.json"

with open(file) as f:
    data = json.load(f)

visualize_collision_check(data, "post-process/collision_check/figures/")