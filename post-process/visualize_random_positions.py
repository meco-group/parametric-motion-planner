import matplotlib.pyplot as plt
from visualization_helpers import *

def visualize_random_positions(env, positions, validity):

    # fig_folder = 'post-process/figures/'
    fig_folder = '../post-process/figures/'

    ### plot trajectory ###
    plt.figure()

    # show environment
    show_environment(env)

    # show random positions
    for i, pos in enumerate(positions):
        plot_vehicle_footprint(plt.gca(), pos["x"], pos["y"], 0.115, 0.115, not validity[i])
    
    set_env_plot_limits(env)
    plt.savefig(fig_folder + 'env-with-random-positions.png', dpi=300)
    plt.show()


file = "output/random_positions.json"

data = json.load(open(file))
env = data['Environment']
positions = data['Positions']
validity = data["Valid"]
visualize_random_positions(env, positions, validity)
