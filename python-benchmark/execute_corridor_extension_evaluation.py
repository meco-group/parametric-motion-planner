import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')
sys.path.append('post-process/')
from visualization_helpers import show_environment, plot_vehicle_footprint, set_env_plot_limits, show_trajectory

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

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
              'text.latex.preamble': '\\usepackage{bm}',
              }
 
    plt.rcParams.update(params)
 
latexify()

def get_results(envs, mp, local_param):
    mp.SetMethod(pmp.PlannerMethod.OCP)
    
    normal_corridors = []
    normal_ocp_trajs = []
    extended_corridors = []
    extended_ocp_trajs = []
    for i in range(len(envs)):
        print(f"\n\nRunning environment {i}")

        mp.SetStart(starts[i])
        mp.SetDest(dests[i])

        local_param.SetVmax(params[i].GetVmax())
        local_param.SetAmax(params[i].GetAmax())
        local_param.SetVehWidth(params[i].GetVehWidth())
        local_param.SetVehHeight(params[i].GetVehHeight())
        local_param.SetMargin(params[i].GetMargin())
        local_env.CopyObstacles(envs[i])

        mp.SetCorridorExtendedMode(False)
        mp.UpdateCorridorSequence()
        # normal_corridors.append(json.loads(mp.ToJson())["corridor_sequence"])
        normal_corridors.append(mp.GetCorridorSequence())

        try:
            mp.Plan()
            normal_ocp_trajs.append(json.loads(mp.ToJson())["trajectory"])
        except:
            normal_ocp_trajs.append(None)

        mp.SetCorridorExtendedMode(True)
        mp.UpdateCorridorSequence()
        # extended_corridors.append(json.loads(mp.ToJson())["corridor_sequence"])
        extended_corridors.append(mp.GetCorridorSequence())

        try:
            mp.Plan()
            extended_ocp_trajs.append(json.loads(mp.ToJson())["trajectory"])
        except:
            extended_ocp_trajs.append(None)
    
    return {
        "normal_corridors" : normal_corridors,
        "extended_corridors" : extended_corridors,
        "normal_ocp_trajs" : normal_ocp_trajs,
        "extended_ocp_trajs" : extended_ocp_trajs
    }

def check_if_sequence_contains_point(sequence, point):
    for corridor in sequence:
        if point.x() >= corridor[0] and point.x() <= corridor[1] and point.y() >= corridor[2] and point.y() <= corridor[3]:
            return True
    return False

def get_all_cells_in_corridor_sequence(env, sequence):
    cells = []

    center_of_cell = pmp.Point2Dd(0, 0)
    for i in range(env["nb_cell_cols"]):
        for j in range(env["nb_cell_rows"]):
            center_of_cell.SetX(env["cell_width"] * i + env["cell_width"] / 2)
            center_of_cell.SetY(env["cell_height"] * j + env["cell_height"] / 2)

            if check_if_sequence_contains_point(sequence, center_of_cell):
                cells.append((i, j))
            # if sequence.ContainsPoint(center_of_cell):
            #     cells.append((i, j))

    return cells

def get_corridor_sequence_area(env, cells):
    return len(cells) * env["cell_width"] * env["cell_height"]

def check_subset(normal_cells, extended_cells):
    for cell in normal_cells:
        if cell not in extended_cells:
            return False
    return True

def process_all_corridors(envs, processed_results):
    normal_corridors = processed_results["normal_corridors"]
    extended_corridors = processed_results["extended_corridors"]
    normal_ocp_trajs = processed_results["normal_ocp_trajs"]
    extended_ocp_trajs = processed_results["extended_ocp_trajs"]

    assert len(envs) == len(normal_corridors)
    assert len(envs) == len(extended_corridors)

    subset_check = [False for i in range(len(envs))]
    area_normal = [0 for i in range(len(envs))]
    area_extended = [0 for i in range(len(envs))]
    area_ratio = [0 for i in range(len(envs))]
    nb_cells_normal = [0 for i in range(len(envs))]
    nb_cells_extended = [0 for i in range(len(envs))]
    nb_corridors_normal = [len(c) for c in normal_corridors]
    nb_corridors_extended = [len(c) for c in extended_corridors]
    travel_times_normal = [None for i in range(len(envs))]
    travel_times_extended = [None for i in range(len(envs))]
    travel_time_relative_improvement = [None for i in range(len(envs))]
    comp_times_normal = [None for i in range(len(envs))]
    comp_times_extended = [None for i in range(len(envs))]
    comp_times_speedup = [None for i in range(len(envs))]

    for i in range(len(envs)):
        normal_cells = get_all_cells_in_corridor_sequence(envs[i], normal_corridors[i])
        extended_cells = get_all_cells_in_corridor_sequence(envs[i], extended_corridors[i])

        subset_check[i] = check_subset(normal_cells, extended_cells)
        area_normal[i] = get_corridor_sequence_area(envs[i], normal_cells)
        area_extended[i] = get_corridor_sequence_area(envs[i], extended_cells)
        area_ratio[i] = area_extended[i] / area_normal[i] if area_normal[i] != 0 else -1
        nb_cells_normal[i] = len(normal_cells)
        nb_cells_extended[i] = len(extended_cells)

        if normal_ocp_trajs[i] is not None:
            travel_times_normal[i] = normal_ocp_trajs[i]["Tf"]
            comp_times_normal[i] = normal_ocp_trajs[i]["total_computation_time"]
        
        if extended_ocp_trajs[i] is not None:
            travel_times_extended[i] = extended_ocp_trajs[i]["Tf"]
            comp_times_extended[i] = extended_ocp_trajs[i]["total_computation_time"]

        if normal_ocp_trajs[i] is not None and extended_ocp_trajs[i] is not None:
            # travel_times_speedup[i] = travel_times_normal[i] / travel_times_extended[i] if travel_times_extended[i] != 0 else -1
            travel_time_relative_improvement[i] = 100*(travel_times_normal[i] - travel_times_extended[i])/travel_times_normal[i] if travel_times_extended[i] != 0 else -1
            comp_times_speedup[i] = comp_times_normal[i] / comp_times_extended[i] if comp_times_extended[i] != 0 else -1

        # if not subset_check[i]:
        # if area_ratio[i] < 1:
        # if area_ratio[i] > 2:
        #     print(f"Environment {i}")
        #     visualize_environment(envs[i], normal_corridors[i], 
        #                         extended_corridors[i], normal_ocp_trajs[i],
        #                         extended_ocp_trajs[i])

    results = {
        "subset_check": subset_check,
        "area_normal": area_normal,
        "area_extended": area_extended,
        "area_ratio": area_ratio,
        "nb_cells_normal": nb_cells_normal,
        "nb_cells_extended": nb_cells_extended,
        "nb_corridors_normal": nb_corridors_normal,
        "nb_corridors_extended": nb_corridors_extended,
        "travel_times_normal": travel_times_normal,
        "travel_times_extended": travel_times_extended,
        "travel_time_relative_improvement": travel_time_relative_improvement,
        "comp_times_normal": comp_times_normal,
        "comp_times_extended": comp_times_extended,
        "comp_times_speedup": comp_times_speedup,
        "percentage_envs_with_one_percent_improvement" : 100*sum([1 for t in travel_time_relative_improvement if (t if t is not None else -1) > 1]) / len(travel_time_relative_improvement),
        "percentage_envs_with_five_percent_improvement" : 100*sum([1 for t in travel_time_relative_improvement if (t if t is not None else -1) > 5]) / len(travel_time_relative_improvement),
        "percentage_envs_with_ten_percent_improvement" : 100*sum([1 for t in travel_time_relative_improvement if (t if t is not None else -1) > 10]) / len(travel_time_relative_improvement),
        "nb_failures_normal" : sum([1 for t in normal_ocp_trajs if t is None]),
        "nb_failures_extended" : sum([1 for t in extended_ocp_trajs if t is None]),
    }
    print(results["percentage_envs_with_one_percent_improvement"])
    print(results["percentage_envs_with_five_percent_improvement"])
    print(results["percentage_envs_with_ten_percent_improvement"])
    return results

def show_corridors(corridors, color='green', **kwargs):
    hatch = kwargs.get('hatch', None)
    max_alpha = kwargs.get('max_alpha', 1)
    lw = kwargs.get('linewidth', 1)
    clip_on = False
    for c in corridors:
        if max_alpha > 0:
            plt.gca().add_patch(Rectangle((c[0], c[2]), c[1]-c[0], c[3]-c[2], 
                                fill=True, facecolor=color, alpha=0.2*max_alpha, 
                                edgecolor=None, clip_on=clip_on))
        plt.gca().add_patch(Rectangle((c[0], c[2]), 
                                      c[1]-c[0], 
                                      c[3]-c[2], 
                            fill=False, edgecolor=color, linewidth=lw,
                            clip_on=clip_on, hatch=hatch))

def show_covered_cells(env, cells, color='green', **kwargs):
    hatch = kwargs.get('hatch', None)
    max_alpha = 1
    alpha_factor = kwargs.get('alpha_factor', 0.5)
    clip_on = False
    w = env["cell_width"]; h = env["cell_height"]
    for c in cells:
        plt.gca().add_patch(Rectangle((c[0]*w, c[1]*h), w, h, 
                            fill=True, facecolor=color, 
                            alpha=alpha_factor*max_alpha, edgecolor=None, 
                            clip_on=clip_on))

def visualize_environment(env, corridors_normal, corridors_extended,
                          normal_ocp_traj, extended_ocp_traj, **kwargs):
    plt.figure(figsize=(4,4))

    show_environment(env)
    show_corridors(corridors_extended, color='green', max_alpha=0, hatch='///', linewidth=0.5)
    show_corridors(corridors_normal, color='green', max_alpha=1.5, linewidth=0)

    if normal_ocp_traj is not None:
        show_trajectory(normal_ocp_traj, 'red', with_trace=False, 
                        width=env["cell_width"], height=env["cell_height"], 
                        with_footprints=True)
    if extended_ocp_traj is not None:
        show_trajectory(extended_ocp_traj, 'darkred', with_trace=False,
                        width=env["cell_width"], height=env["cell_height"], 
                        with_footprints=True)

    set_env_plot_limits(env)
    plt.xticks([])
    plt.yticks([])

    if normal_ocp_traj is not None and extended_ocp_traj is not None:
        print(f"Normal travel time:   {normal_ocp_traj['Tf']:.3f}")
        print(f"Extended travel time: {extended_ocp_traj['Tf']:.3f}\n")
    
    plt.tight_layout()

    show = kwargs.get("show", True)
    if show:
        plt.show()

def visualize_results(results):
    # extract results
    subset_check = results["subset_check"]
    area_normal = results["area_normal"]
    area_extended = results["area_extended"]
    area_ratio = results["area_ratio"]
    nb_cells_normal = results["nb_cells_normal"]
    nb_cells_extended = results["nb_cells_extended"]
    nb_corridors_normal = results["nb_corridors_normal"]
    nb_corridors_extended = results["nb_corridors_extended"]
    travel_times_normal = results["travel_times_normal"]
    travel_times_extended = results["travel_times_extended"]
    travel_time_relative_improvement = results["travel_time_relative_improvement"]
    comp_times_normal = results["comp_times_normal"]
    comp_times_extended = results["comp_times_extended"]
    comp_times_speedup = results["comp_times_speedup"]

    # make a histogram showing the area expressed in number of cells
    # plt.figure()
    # bins = np.arange(0, max(max(nb_cells_normal), max(nb_cells_extended)) + 1) - 0.5
    # plt.hist(nb_cells_normal, bins=bins, alpha=0.5, label='Normal', color='green')
    # plt.hist(nb_cells_extended, bins=bins, alpha=0.5, label='Extended', color='teal')
    # plt.legend()

    plt.figure(figsize=(6, 3))
    bar_x_vals = np.arange(0, max(max(nb_corridors_normal), max(nb_corridors_extended)) + 1)
    nb_corridors_normal_count = [len([1 for n in nb_corridors_normal if n == i]) for i in bar_x_vals]
    nb_corridors_extended_count = [len([1 for n in nb_corridors_extended if n == i]) for i in bar_x_vals]
    width = 0.25
    plt.bar(bar_x_vals-width/2, nb_corridors_normal_count, width=width, alpha=0.5, label='$\\bm{C}$', color='green', align='center')
    plt.bar(bar_x_vals+width/2, nb_corridors_extended_count, width=width, alpha=1, label='$\\bm{C}^+$', color='green', fill=False, edgecolor='green', hatch='///', align='center')
    plt.xlim([0, max(max(nb_corridors_normal), max(nb_corridors_extended)) + 1])
    plt.xticks(np.arange(1, 14))
    plt.xlabel('\# corridors in a single sequence')
    plt.ylabel('\# of occurences\nin the benchmark')
    plt.tight_layout()
    plt.legend()
    # plt.savefig('python-benchmark/figures/corridor-evaluation/nb_corridors.png', dpi=300)

    # make a histogram showing the area ratios
    # plt.figure()
    # plt.hist(area_ratio, bins=50)
    # plt.title('extended area / normal area')

    # make a histogram showing the subset check
    # plt.figure()
    # plt.hist([0 if s==False else 1 for s in subset_check], bins=2)

    # make a histogram of computation time speedups
    # plt.figure()
    # plt.hist([1/s for s in comp_times_speedup if s is not None], bins=50)
    # plt.title('1 / Computation time speedup')

    # make a histogram of travel time speedups
    # plt.figure()
    # plt.hist([s for s in travel_time_relative_improvement if s is not None], bins=150)
    # plt.title('Travel time speedup')

    # plt.figure()
    # absolute_travel_time_improvement = [n - e for n, e in zip(travel_times_normal, travel_times_extended) if n is not None and e is not None]
    # plt.hist(absolute_travel_time_improvement, bins=50)
    # plt.title('Absolute travel time improvement')

    plt.show()

def create_latex_table(processed_results):
    # create a table comparing simplified\ncorridors and extended\ncorridors
    # rows in the table:
    # - median number of corridors
    # - average number of cells
    # - average area ratio
    # - failures
    # - average computation time

    avg_comp_times_normal = np.mean([t for t in processed_results["comp_times_normal"] if t is not None])
    avg_comp_times_extended = np.mean([t for t in processed_results["comp_times_extended"] if t is not None])

    avg_travel_times_normal = np.mean([t for t in processed_results["travel_times_normal"] if t is not None])
    avg_travel_times_extended = np.mean([t for t in processed_results["travel_times_extended"] if t is not None])

    rows = {
        "median number of corridors": [np.median(processed_results["nb_corridors_normal"]), np.median(processed_results["nb_corridors_extended"])],
        "average number of cells": [np.mean(processed_results["nb_cells_normal"]), np.mean(processed_results["nb_cells_extended"])],
        # "average area ratio": [np.mean(processed_results["area_ratio"])],
        "\# solver failures": [processed_results["nb_failures_normal"], processed_results["nb_failures_extended"]],
        "average computation time": [avg_comp_times_normal, avg_comp_times_extended],
        "average travel time": [avg_travel_times_normal, avg_travel_times_extended],
    }

    # begin table
    table = "\n\\begin{table}\n"
    table += "\t\\centering\n"
    table += "\t\\caption{Comparison of normal and extended corridors}\n"
    table += "\t\\label{tab:corridor_comparison}\n"
    table += "\t\\begin{tabular}{r|cc}\n"
    table += "\t\\toprule\n"
    
    # header row
    table += "\t\t& normal corridors & extended corridors \\\\\n"
    table += "\t\t\\midrule\n"

    # rows
    for row_name, row_values in rows.items():
        table += f"\t\t{row_name} & {row_values[0]} & {row_values[1]} \\\\\n"

    # end table
    table += "\t\\bottomrule\n"
    table += "\t\\end{tabular}\n"
    table += "\\end{table}\n"
    print(table)

    # create a separate table showing:
    # - % envs in which the travel time was reduced by at least 1%
    # - % envs in which the travel time was reduced by at least 5%
    # - % envs in which the travel time was reduced by at least 10%

    rows = {
        "1\% reduction": [processed_results["percentage_envs_with_one_percent_improvement"]],
        "5\% reduction": [processed_results["percentage_envs_with_five_percent_improvement"]],
        "10\% reduction": [processed_results["percentage_envs_with_ten_percent_improvement"]],
    }

    # begin table
    table = "\n\\begin{table}\n"
    table += "\t\\centering\n"
    table += "\t\\caption{Evaluation of the effect of using extended corridors on $t_{\mathrm{move}}$. For different levels of relative reduction of $t_{\mathrm{move}}$, the percentage of benchmark environments that show at least that reduction is shown.\n"
    table += "\t\\begin{tabular}{r|c}\n"
    table += "\t\\toprule\n"

    # header row
    table += "\t\trelative reduction of $t_{\mathrm{move}}$ & percentage of environments\\\\\n"
    table += "\t\t\\midrule\n"

    # rows
    for row_name, row_values in rows.items():
        table += f"\t\t{row_name} & {row_values[0]}\% \\\\\n"

    # end table
    table += "\t\\bottomrule\n"
    table += "\t\\end{tabular}\n"
    table += "\\end{table}\n"
    print(table)



# extract data
file_name_appendix = "_large_double_more_obstacles_10"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# construct motion planner
mp = pmp.MotionPlanner(pmp.PlannerMethod.OCP, local_param, local_env)
RUN_BENCHMARK_AGAIN = False
mp.SetJustInTimePreparationMode(not RUN_BENCHMARK_AGAIN)

# get results
try:
    if RUN_BENCHMARK_AGAIN:
        raise FileNotFoundError
    results = json.load(open('python-benchmark/files/corridor_extension_evaluation.json'))
except FileNotFoundError:
    results = {}
    
    # get corridors
    results = get_results(envs, mp, local_param)
    with open('python-benchmark/files/corridor_extension_evaluation.json', 'w') as f:
        json.dump(results, f, indent=4)
envs = [json.loads(e.ToJson()) for e in envs]

# process corridors
processed_results = process_all_corridors(envs, results)

# postprocess
create_latex_table(processed_results)
visualize_results(processed_results)

# visualize envrionments
# for i in [0, 21, 43, 154, 407, 473]:
for i in [158, 134, 264, 491]:
    visualize_environment(envs[i], results["normal_corridors"][i], 
                          results["extended_corridors"][i], 
                          results["normal_ocp_trajs"][i], 
                          results["extended_ocp_trajs"][i],
                          show=False)
    # plt.savefig(f'python-benchmark/figures/corridor-evaluation/env_{i}.png', dpi=300)
    # plt.close()
    plt.show()
    
