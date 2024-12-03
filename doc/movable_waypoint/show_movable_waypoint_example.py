import sys
sys.path.append('post-process/')
sys.path.append('build/')
from visualization_helpers import load_data, show_environment, set_env_plot_limits, show_corridors, show_trajectory, show_waypoints
import parametric_motion_planner_module as pmp
import matplotlib.pyplot as plt

def latexify():
    params = {#'backend': 'ps',
              'axes.labelsize': 15,
              'axes.titlesize': 15,
              'legend.fontsize': 15,
              'xtick.labelsize': 15,
              'ytick.labelsize': 15,
              'text.usetex': True,
              'font.family': 'serif',
              'figure.figsize': [7,5],
              'text.latex.preamble': r'\usepackage{bm}',
              }
 
    plt.rcParams.update(params)
 
latexify()

def annotate_waypoint(parametrization, idx, offset_x, offset_y):
    waypoint = (parametrization["waypoints"][idx]["x"],
                parametrization["waypoints"][idx]["y"])
    xy_text = (waypoint[0]+offset_x, waypoint[1]+offset_y)
    
    text = r"$\bm{p}_{" + str(idx) + "}$"
    plt.annotate(text, xy=waypoint, xytext=xy_text, ha='center', va='center')

def visualize_output(env, params, corridors, planner_methods, 
                     trajectories, parametrizations=[]):
    assert len(planner_methods) == len(trajectories)
    assert len(parametrizations) == len(trajectories)

    # fig_folder = 'post-process/figures/'
    fig_folder = 'doc/movable_waypoint/'

    first_arena_idx = 0
    while planner_methods[first_arena_idx] != "ARENA":
        first_arena_idx += 1

        if first_arena_idx >= len(planner_methods):
            first_arena_idx = None
            break

    colors = []
    for i in range(len(trajectories)):
        if planner_methods[i] == "P2P":
            colors.append('orange')
        elif planner_methods[i] == "OCP":
            colors.append('r')
        elif planner_methods[i] == "ARENA":
            colors.append('b')
        else:
            colors.append('k')

    colors[1] = 'navy'

    ### plot trajectory ###
    plt.figure(figsize=(6, 3))

    # show environment
    show_environment(env)

    # plot corridors
    show_corridors(corridors)
        
    # plot waypoints
    for i in range(len(trajectories)):
        if i == 2:
            show_waypoints(parametrizations[i])
                           
    # plot trajectory
    for i in range(len(trajectories)):
        show_trajectory(trajectories[i], colors[i], 
                        with_trace=(i == 2), 
                        width=params["veh_width"], 
                        height=params["veh_height"],
                        with_footprints=(i == 0),
                        nb_samples_to_show=-1,
                        virtual_initial_footprint=False,
                        virtual_final_footprint=False,
                        show_markers=True,
                        linewidth=1,
                        with_line=(i != 1))


    movable_waypoint = (parametrizations[2]["waypoints"][6]["x"],
                        parametrizations[2]["waypoints"][6]["y"])
    plt.annotate("modified\nwaypoint", xy=movable_waypoint,
                #  xytext=(movable_waypoint[0]+0.05, movable_waypoint[1]+0.12),
                 xytext=(movable_waypoint[0]-0.12*2-0.05, movable_waypoint[1]+0),
                 ha='center', va='center',
                 arrowprops=dict(arrowstyle="-|>", 
                                 connectionstyle="arc3,rad=.2", 
                                 lw=1, color='white'),
                 color='white')
    
    suboptimal_waypoint = (parametrizations[2]["waypoints"][2]["x"],
                           parametrizations[2]["waypoints"][2]["y"])
    plt.annotate("suboptimal\nwaypoint", xy=suboptimal_waypoint,
                 xytext=(suboptimal_waypoint[0]+0.18, suboptimal_waypoint[1]-0.12),
                 ha='center', va='center',
                 arrowprops=dict(arrowstyle="-|>",
                                 connectionstyle="arc3,rad=-.4",
                                 lw=1, color='white'),
                 color='white')
    
    annotate_waypoint(parametrizations[2], 2, 0, 0.05)
    annotate_waypoint(parametrizations[2], 6, 0, 0.05)

    set_env_plot_limits(env)
    plt.xticks([])
    plt.yticks([])

    # remove axis box
    # plt.gca().spines['top'].set_visible(False)
    # plt.gca().spines['right'].set_visible(False)
    # plt.gca().spines['bottom'].set_visible(False)
    # plt.gca().spines['left'].set_visible(False)

    plt.tight_layout()

    
    # # make some space above the figure for the zoombox
    # plt.subplots_adjust(top=0.7)


    # # create a zoom-box around p2 and put it to the right side on top of the original axes
    # from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset
    # axins = zoomed_inset_axes(plt.gca(), 1.5, loc='center', bbox_to_anchor=(0.5, 0.9), bbox_transform=plt.gcf().transFigure)
    # axins.set_xlim(0.0, 0.15*3+0.05)
    # axins.set_ylim(0.15*2-0.01, 0.15*4-0.05)
    # show_environment(env)
    # show_corridors(corridors, clip_on=True)
    # show_waypoints(parametrizations[2])
    # for i in range(len(trajectories)):
    #     show_trajectory(trajectories[i], colors[i], 
    #                     with_trace=(i == 2), 
    #                     width=params["veh_width"], 
    #                     height=params["veh_height"],
    #                     with_footprints=(i == 0),
    #                     nb_samples_to_show=-1,
    #                     virtual_initial_footprint=False,
    #                     virtual_final_footprint=False,
    #                     show_markers=True,
    #                     linewidth=1,
    #                     with_line=(i != 1))
    # # for i in range(len(trajectories)):
    # #     if i == 2:
    # #         show_waypoints(parametrizations[i])
    # annotate_waypoint(parametrizations[2], 2, 0, 0.05)
    # axins.set_xticks([])
    # axins.set_yticks([])
    # plt.yticks([])
    # plt.xticks([])

    # mark_inset(plt.gca(), axins, loc1=2, loc2=4, fc="none", ec="0.5")
    # plt.gca().spines['top'].set_visible(False)
    # plt.gca().spines['right'].set_visible(False)
    # plt.gca().spines['bottom'].set_visible(False)
    # plt.gca().spines['left'].set_visible(False)
    # plt.tight_layout()

    # plt.show()
    

    plt.savefig(fig_folder + 'changes_to_parametrization.png', dpi=300)
    plt.savefig(fig_folder + 'changes_to_parametrization.pdf')

# Create objects
env = pmp.Environment(5, 9, 0.12, 0.12)
params = pmp.Parameters(2.0, 6.0, 0.115, 0.115, 0.001)
mp = pmp.MotionPlanner(pmp.PlannerMethod.OCP, params, env)

# Add obstacles
x_vals = [0, 1, 1, 2, 2, 3, 4, 5]
y_vals = [4, 0, 1, 1, 2, 2, 3, 3]
cell = pmp.Point2Di(0, 0)
for i in range(len(x_vals)):
    cell.SetX(x_vals[i])
    cell.SetY(y_vals[i])
    print(f"{cell.x()}, {cell.y()}")
    env.AddObstacle(cell)

# Plan
mp.SetStart(pmp.Point2Dd(0.06, 0.06))
mp.SetDest(pmp.Point2Dd(0.12*3+0.06, 0.06))
mp.Plan()
mp.DumpToJson("doc/movable_waypoint/ocp.json", False)
Tf_ocp = mp.GetTravelTime()
t_comp_ocp = mp.GetSolverTime()

mp.SetMethod(pmp.PlannerMethod.ARENA)
mp.SetSuboptimalityEliminationFeature(False)
mp.Plan()
mp.DumpToJson("doc/movable_waypoint/arena.json", False)
Tf_arena = mp.GetTravelTime()
t_comp_arena = mp.GetSolverTime()

mp.SetSuboptimalityEliminationFeature(True)
mp.Plan()
mp.DumpToJson("doc/movable_waypoint/arena_plus.json", False)
Tf_arena_plus = mp.GetTravelTime()
t_comp_arena_plus = mp.GetSolverTime()

print(f"OCP: \tTf={Tf_ocp:.3f}, t_comp={t_comp_ocp:.3f}")
print(f"ARENA: \tTf={Tf_arena:.3f}, t_comp={t_comp_arena:.3f}")
print(f"ARENA+:\tTf={Tf_arena_plus:.3f}, t_comp={t_comp_arena_plus:.3f}")

# write a latex table with the results
print(r"\t\begin{tabular}{|c|c|c|c|c|}" + "\n")
print(r"\t\t\hline" + "\n")
print(r"\t\tMethod & $T_f$ [s] & $t_{\text{comp}}$ [s] & $T_f$ [s] & $t_{\text{comp}}$ [s] \\" + "\n")
print(r"\t\t& \multicolumn{2}{c|}{ARENA} & \multicolumn{2}{c|}{ARENA+} \\" + "\n")
print(r"\t\t\hline" + "\n")
print(r"\t\tOCP & \multicolumn{2}{c|}{" + f"{Tf_ocp:.3f} & {t_comp_ocp:.3f}" + r"} \\" + "\n")
print(r"\t\t\hline" + "\n")
print(r"\t\t\end{tabular}" + "\n")


# Load and show results
files = ["doc/movable_waypoint/ocp.json", "doc/movable_waypoint/arena.json", "doc/movable_waypoint/arena_plus.json"]

envs_list = []
params_list = []
corridors_list = []
planner_methods_list = []
trajectories_list = []
parametrizations_list = []
for output_file in files:
    env, params, corridors, planner_method, trajectory, parametrization = load_data(output_file)
    envs_list.append(env)
    params_list.append(params)
    corridors_list.append(corridors)
    planner_methods_list.append(planner_method)
    trajectories_list.append(trajectory)
    parametrizations_list.append(parametrization)


visualize_output(envs_list[0], params_list[0], corridors_list[0], 
                 planner_methods_list, trajectories_list, 
                 parametrizations_list)

