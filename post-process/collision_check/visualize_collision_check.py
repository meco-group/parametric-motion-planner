import sys
sys.path.append('post-process/')
from visualization_helpers import *

def visualize_collision_check(data):

    vehicle_colors = ['b', 'green', 'k', 'orange', 'green']

    for iteration in data["iterations"]:
        plt.figure()

        # print([len(d) for d in data["vehicle_planners"]])
        # print(data["vehicle_planners"][iteration])
        env = iteration["vehicle_planners"][0]["environment"]
        show_environment(env)

        planners = iteration["vehicle_planners"]
        for i in range(len(planners)):
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
    
    plt.show()

  

# Load the data
file = "build/output/trajectory_collision_check.json"

with open(file) as f:
    data = json.load(f)

visualize_collision_check(data)