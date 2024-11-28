import json
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

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
            #   'text.latex.preamble': [r'\usepackage{bm}'],
              }
 
    plt.rcParams.update(params)
 
latexify()

# with open('python-benchmark/files/results.json', 'r') as f:
# with open('python-benchmark/files/results_cell.json', 'r') as f:
# with open('python-benchmark/files/results_double.json', 'r') as f:
# with open('python-benchmark/files/results_large.json', 'r') as f:
with open('python-benchmark/files/results_large_double.json', 'r') as f:
    results = json.load(f)

def scatter(results, method, x, y, color):
    xx = np.array(results[method][x]); yy = np.array(results[method][y])
    infeasibles = np.array(results[method]["corridor_infeasibilities_detected"])
    failed = xx < 0
    success = np.logical_and(np.logical_not(infeasibles), np.logical_not(failed))

    xx_failed = xx[failed]; yy_failed = yy[failed]
    xx_infeasible = xx[infeasibles]; yy_infeasible = yy[infeasibles]
    xx_success = xx[success]
    yy_success = yy[success]

    plt.scatter(xx_success, yy_success, color=color, label=method, alpha=0.5)
    if (len(xx_failed) > 0):
        plt.scatter(xx_failed, yy_failed, color=color, label=f"{method} failures ({len(xx_failed)})", marker='x', alpha=0.5)
    if (len(xx_infeasible) > 0):
        plt.scatter(xx_infeasible, yy_infeasible, color=color, label=f"{method} infeasibles ({len(xx_infeasible)})", marker='+', alpha=0.5)
    plt.xlabel(x)
    plt.ylabel(y)
    plt.legend()

    x_avg = np.mean(xx_success)
    y_avg = np.mean(yy_success)

    plt.scatter(x_avg, y_avg, color='k', label='Average')

def optimality_comparison(results, method1, method2, color1, color2):
    Tf_1 = np.array(results[method1]["Tf"])
    Tf_2 = np.array(results[method2]["Tf"])

    diff = Tf_2 - Tf_1
    max_idx = np.argmax(diff)
    print("max suboptimality at: ", max_idx)
    print("Tf_2: ", Tf_2[max_idx])
    print("Tf_1: ", Tf_1[max_idx])
    # print(Tf_2)

    # get indices where both methods are succesfull
    success = np.logical_not(
        np.logical_or(
            np.logical_or(
                np.array(results[method1]["corridor_infeasibilities_detected"]), 
                np.array(results[method2]["corridor_infeasibilities_detected"]),
            ),
            np.logical_or(
                np.array(results[method1]["t_comp_solver"]) < 0,
                np.array(results[method2]["t_comp_solver"]) < 0
            )
    ))
    Tf_1 = Tf_1[success]
    Tf_2 = Tf_2[success]

    # sort Tf_1 in ascending order and change the order of Tf_2 accordingly
    idx = np.argsort(Tf_1)
    Tf_1 = Tf_1[idx]
    Tf_2 = Tf_2[idx]

    # for i in range(len(Tf_1)):
    #     # plt.plot([i, i], [0, Tf_2[i] - Tf_1[i]], color=color2, linestyle='-', linewidth=2)
    #     plt.plot([i, i], [0, (Tf_2[i] - Tf_1[i])/Tf_1[i]], color=color2, linestyle='-', linewidth=3)

    rel_error = (Tf_2 - Tf_1) / Tf_1
    abs_error = Tf_2 - Tf_1
    error = rel_error
    idx = np.argsort(error)
    error = error[idx]
    plt.fill_between(np.linspace(0, 1, len(error)), -0.01, 0.01, color='gray', alpha=0.5)
    plt.fill_between(np.linspace(0, 1, len(error)), 0, error, color=color2, alpha=1.0)

    idx = np.where(error > 0.01)[0]
    plt.axvline(idx[0]/len(error), color='k', linestyle='-')

    plt.axhline(0, color='k', linestyle='-')
    plt.xlim([0, 1])
    plt.ylim([-0.02, 0.06])

def optimality_comparison_extended(results, method1, method2, method3, color1, color2, color3):
    Tf_1 = np.array(results[method1]["Tf"])
    Tf_2 = np.array(results[method2]["Tf"])
    Tf_3 = np.array(results[method3]["Tf"])

    diff2 = Tf_2 - Tf_1
    max_idx = np.argmax(diff2)
    print("max suboptimality at: ", max_idx)
    print("Tf_2: ", Tf_2[max_idx])
    print("Tf_1: ", Tf_1[max_idx])
    # print(Tf_2)

    diff3 = Tf_3 - Tf_1
    max_idx = np.argmax(diff3)
    print("max suboptimality at: ", max_idx)
    print("Tf_3: ", Tf_3[max_idx])
    print("Tf_1: ", Tf_1[max_idx])

    # get indices where both methods are succesfull
    success = np.logical_not(
        np.logical_or(
            np.logical_or(
                np.array(results[method1]["corridor_infeasibilities_detected"]), 
                np.array(results[method2]["corridor_infeasibilities_detected"]),
                np.array(results[method3]["corridor_infeasibilities_detected"])
            ),
            np.logical_or(
                np.array(results[method1]["t_comp_solver"]) < 0,
                np.array(results[method2]["t_comp_solver"]) < 0,
                np.array(results[method3]["t_comp_solver"]) < 0
            )
    ))
    Tf_1 = Tf_1[success]
    Tf_2 = Tf_2[success]
    Tf_3 = Tf_3[success]

    # sort Tf_1 in ascending order and change the order of Tf_2 accordingly
    idx = np.argsort(Tf_1)
    Tf_1 = Tf_1[idx]
    Tf_2 = Tf_2[idx]
    Tf_3 = Tf_3[idx]

    rel_error = (Tf_3 - Tf_1) / Tf_1
    abs_error = Tf_3 - Tf_1
    error = rel_error
    plt.fill_between(np.linspace(0, 1, len(error)), -0.01, 0.01, color='gray', alpha=0.5)
    idx = np.argsort(error)
    error = error[idx]
    plt.fill_between(np.linspace(0, 1, len(error)), 0, error, color=color3, alpha=1.0, label=method3)
    idx = np.where(error > 0.01)[0]
    plt.axvline(idx[0]/len(error), color='k', linestyle='-')

    rel_error = (Tf_2 - Tf_1) / Tf_1
    abs_error = Tf_2 - Tf_1
    error = rel_error
    idx = np.argsort(error)
    error = error[idx]
    plt.fill_between(np.linspace(0, 1, len(error)), 0, error, color=color2, alpha=1.0, label=method2)
    idx = np.where(error > 0.01)[0]
    plt.axvline(idx[0]/len(error), color='k', linestyle='-')

    plt.axhline(0, color='k', linestyle='-')
    plt.xlim([0, 1])
    plt.ylim([-0.02, 0.06])
    plt.legend(loc='best')

def optimality_comparison_extended_new(results, baseline_method, methods, colors, use_abs_error=False):
    Tf_baseline = np.array(results[baseline_method]["Tf"])
    Tfs = [np.array(results[method]["Tf"]) for method in methods]
    og_idxs = [np.arange(len(Tf_baseline)) for Tf in Tfs]

    # get indices where all methods are succesfull
    idx = np.array(results[baseline_method]["t_comp_solver"]) >= 0
    for Tf in Tfs:
        idx = np.logical_and(idx, Tf >= 0)
    Tf_baseline = Tf_baseline[idx]
    Tfs = [Tf[idx] for Tf in Tfs]
    og_idxs = [og_idx[idx] for og_idx in og_idxs]

    # compute errors
    rel_errors = [100*(Tf - Tf_baseline) / Tf_baseline for Tf in Tfs]
    if use_abs_error:
        rel_errors = [Tf - Tf_baseline for Tf in Tfs]
        
    # sort errors in ascending order
    print()
    for i in range(len(rel_errors)):
        idx = np.argsort(rel_errors[i])
        rel_errors[i] = rel_errors[i][idx]
        og_idxs[i] = og_idxs[i][idx]
        my_dict = {og_idxs[i][j]: round(rel_errors[i][j],2) for j in range(len(rel_errors[i])-10, len(rel_errors[i]))}
        print(f"10 most suboptimal cases for method {methods[i]}:")
        # print(og_idxs[i][-10:])
        print(my_dict)
        
        my_dict = {og_idxs[i][j]: round(rel_errors[i][j],2) for j in range(10)}
        print(f"10 most optimal cases for method {methods[i]}:")
        print(my_dict)


    # visualize
    plt.figure(figsize=(6,3))
    # plt.fill_between(np.linspace(0, 1, len(rel_errors[0])), -1, 1, color='gray', alpha=0.5)
    for i in range(len(rel_errors)):
        print("Method: ", methods[i])
        plt.plot(np.linspace(0, 100, len(rel_errors[i])), rel_errors[i], color=colors[i], label=methods[i])
        plt.fill_between(np.linspace(0, 100, len(rel_errors[i])), 0, rel_errors[i], color=colors[i], alpha=0.5 if colors[i] != "black" else 0.2, label=None)

    plt.axhline(0, color='k', linestyle='-')
    plt.xlim([0, 100])
    plt.ylim([-5, 100.5])
    plt.xlabel("\% of Benchmark Environments")
    if use_abs_error:
        plt.ylabel("Absolute suboptimality [s]")
    else:
        plt.ylabel("Relative\nsuboptimality [\%]")
    plt.gcf().legend(loc='lower center', ncol = 3)
    plt.tight_layout(rect=[0, 0.15, 1, 1]) 

    # highlight the 0.95 percentile
    for i in range(len(rel_errors)):
        if methods[i] == "ARENA":
            p = 1
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='royalblue', markersize=4)
            arrow_start = (100*idx[0]/len(rel_errors[i]), p)
            arrow_end = (56, 34.6)
            plt.annotate(f"({idx[0]/len(rel_errors[i])*100:.0f}\%, {p:.0f}\%)",
                        xy=arrow_start, xycoords='data',
                        xytext=arrow_end, textcoords='data',
                        # arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.2"),
                        # show a slightly curved arrow with a solid arrowhead
                        arrowprops=dict(arrowstyle="-|>", 
                                        connectionstyle="arc3,rad=-.1", 
                                        lw=1,
                                        color='royalblue'),
                        fontsize=15, color='k'
                        )
            
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='royalblue', markersize=4)
            arrow_start = (100*idx[0]/len(rel_errors[i]), p)
            arrow_end = (70, 52)
            plt.annotate(f"({idx[0]/len(rel_errors[i])*100:.0f}\%, {p:.0f}\%)",
                        xy=arrow_start, xycoords='data',
                        xytext=arrow_end, textcoords='data',
                        # arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.2"),
                        # show a slightly curved arrow with a solid arrowhead
                        arrowprops=dict(arrowstyle="-|>", 
                                        connectionstyle="arc3,rad=-.1", 
                                        lw=1,
                                        color='royalblue'),
                        fontsize=15, color='k'
                        )
            
        if methods[i] == "OmgTools":
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='black', markersize=4)
            arrow_start = (100*idx[0]/len(rel_errors[i]), p)
            arrow_end = (20, 50)
            plt.annotate(f"({idx[0]/len(rel_errors[i])*100:.0f}\%, {p:.0f}\%)",
                        xy=arrow_start, xycoords='data',
                        xytext=arrow_end, textcoords='data',
                        # arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.2"),
                        # show a slightly curved arrow with a solid arrowhead
                        arrowprops=dict(arrowstyle="-|>", 
                                        connectionstyle="arc3,rad=-.1", 
                                        lw=1,
                                        color='black'),
                        fontsize=15, color='k'
                        )

    # plt.figure()
    # plt.plot(Tf_baseline, Tfs[2], 'o')
    # plt.plot([0, 5], [0, 5], 'k', label='0%')
    # plt.plot([0, 5], [0, 5*1.1], 'gray', label='10%')
    # plt.plot([0, 5], [0, 5*2], 'lightgray', label='100%')
    # plt.legend()

def computation_time_comparison(results, method1, method2, color1, color2):
    t_comp_total_1 = np.array(results[method1]["t_comp_total"])
    t_comp_total_2 = np.array(results[method2]["t_comp_total"])

    # filter out failed plans (solver time = -1)
    idx = np.logical_and(np.array(results[method1]["t_comp_solver"]) >= 0, np.array(results[method2]["t_comp_solver"]) >= 0)
    t_comp_total_1 = t_comp_total_1[idx]
    t_comp_total_2 = t_comp_total_2[idx]

    speedup = t_comp_total_1 / t_comp_total_2
    idx = np.argsort(speedup)
    speedup = speedup[idx]

    # plt.fill_between(range(len(speedup)), 0.5, 2, color='gray', alpha=0.5)
    plt.fill_between(range(len(speedup)), 1, speedup, color=color2, alpha=1.0)

    plt.xlim([0, len(speedup)-1])
    plt.ylim([1, 30])

    plt.axhline(10, color='k', linestyle='-')
    plt.axhline(20, color='k', linestyle='-')

def computation_time_comparison_extended(results, method1, method2, method3, color1, color2, color3):
    t_comp_total_1 = np.array(results[method1]["t_comp_total"])
    t_comp_total_2 = np.array(results[method2]["t_comp_total"])
    t_comp_total_3 = np.array(results[method3]["t_comp_total"])

    # filter out failed plans (solver time = -1)
    idx = np.logical_and(np.array(results[method1]["t_comp_solver"]) >= 0, np.array(results[method2]["t_comp_solver"]) >= 0, np.array(results[method3]["t_comp_solver"]) >= 0)
    t_comp_total_1 = t_comp_total_1[idx]
    t_comp_total_2 = t_comp_total_2[idx]
    t_comp_total_3 = t_comp_total_3[idx]

    speedup2 = t_comp_total_1 / t_comp_total_2
    idx = np.argsort(speedup2)
    speedup2 = speedup2[idx]

    speedup3 = t_comp_total_1 / t_comp_total_3
    idx = np.argsort(speedup3)
    speedup3 = speedup3[idx]

    plt.fill_between(range(len(speedup3)), 1, speedup3, color=color3, alpha=1.0, label=method3)
    plt.fill_between(range(len(speedup2)), 1, speedup2, color=color2, alpha=1.0, label=method2)

    plt.xlim([0, len(speedup2)-1])
    plt.ylim([1, 30])

    plt.axhline(10, color='k', linestyle='-')
    plt.axhline(20, color='k', linestyle='-')

    plt.legend(loc='best')

def computation_time_comparison_extended_new(results, baseline_method, methods, colors, use_solver_time=False):
    # Provide methods from slow to fast
    if use_solver_time:
        t_comp_baseline = np.array(results[baseline_method]["t_comp_solver"])
        t_comps = [np.array(results[method]["t_comp_solver"]) for method in methods]
        for t_comp in t_comps:
            idx = t_comp == 0
            t_comp[idx] = 1.0e-10
    else:    
        t_comp_baseline = np.array(results[baseline_method]["t_comp_total"])
        t_comps = [np.array(results[method]["t_comp_total"]) for method in methods]

    # filter out failed plans (solver time = -1)
    idx = np.array(results[baseline_method]["t_comp_solver"]) >= 0
    for t_comp in t_comps:
        idx = np.logical_and(idx, t_comp >= 0)
    t_comp_baseline = t_comp_baseline[idx]
    t_comps = [t_comp[idx] for t_comp in t_comps]

    speedups = [t_comp_baseline / t_comp for t_comp in t_comps]
    for i in range(len(speedups)):
        idx = np.argsort(speedups[i])
        speedups[i] = speedups[i][idx]
    
    for i in range(len(speedups)):
        plt.fill_between(range(len(speedups[i])), 1, speedups[i], color=colors[i], alpha=1.0, label=methods[i])
    
    plt.xlim([0, len(speedups[0])-1])
    plt.ylim([0, 30])

    plt.xlabel("Random environments")
    if use_solver_time:
        plt.ylabel("Speedup of solver time")
    else:
        plt.ylabel("Speedup of total computation time")

    plt.axhline(10, color='k', linestyle='-')
    plt.axhline(20, color='k', linestyle='-')

    plt.legend(loc='best')

def show_histogram_densities(results, methods, colors):
    # # Travel time
    # Tf = [np.array(results[method]["Tf"]) for method in methods]
    # plot_densities(Tf, colors, methods, "Travel time [s]", "Density")


    fig, axes = plt.subplots(2, 1)  # Two subplots stacked vertically

    # Total computation time
    plt.sca(axes[1])
    t_comp_total = [np.array(results[method]["t_comp_total"]) for method in methods]
    t_comp_total.reverse()
    colors.reverse()
    methods.reverse()
    plot_densities(t_comp_total, colors, methods, "Total computation time [ms]", "Density")
    colors.reverse()
    methods.reverse()
    
    # Solver computation time
    plt.sca(axes[0])
    t_comp_solver = [np.array(results[method]["t_comp_solver"]) for method in methods]
    method_sequence = {"OCP-30":2, "OmgTools":1, "ARENA":0, "P2P":3}
    idx = [method_sequence[method] for method in method_sequence]
    idx = [methods.index(method) for method in method_sequence]
    t_comp_solver = [t_comp_solver[i] for i in idx]
    colors = [colors[i] for i in idx]
    methods = [methods[i] for i in idx]
    print(methods)
    extra_handles, extra_labels = plot_densities(t_comp_solver, colors, methods, "Solver computation time [ms]", "Density")
    # colors.reverse()
    # methods.reverse()

    handles, labels = axes[0].get_legend_handles_labels()  # Get legend items from one subplot
    all_handles = handles + extra_handles
    all_labels = labels + extra_labels
    # permutation = [4, 2, 0, 1, 3]
    permutation = [2, 3, 1, 4, 0]
    all_handles = [all_handles[i] for i in permutation]
    all_labels = [all_labels[i] for i in permutation]
    fig.legend(all_handles, all_labels, loc='lower center', ncol=3)  # Shared legend below
    plt.tight_layout(rect=[0, 0.15, 1, 1])  # Adjust layout to fit legend
    axes[1].set_xlim(right=650)
    axes[0].set_xlim(right=150)
    # plt.tight_layout()
    # plt.subplots_adjust(bottom=0.2)
    # plt.show()

def plot_densities(data, colors, labels, xlabel, ylabel):
    # plt.figure()
    density_lines = []
    for i in range(len(data)):
        if labels[i] == "P2P":
            continue
        
        # plot density
        sns.kdeplot(data[i], color=colors[i], label=labels[i], fill=True, alpha=0.5)

        # compute the mean
        mean = np.mean(data[i])

        # find the value of the density plot at the mean
        kde = sns.kdeplot(data[i], color=colors[i], label=None, fill=False, alpha=0)
        density_lines.append(kde.get_lines()[-1])
        xdata, ydata = kde.get_lines()[-1].get_data()
        mean_density = np.interp(mean, xdata, ydata)

        # plot the mean
        plt.plot([mean, mean], [0, mean_density], color=colors[i], 
                 linestyle='-', linewidth=2, label=None, zorder=i)
        # plt.plot(mean, mean_density, 'o', color=colors[i], label=f"{labels[i]} mean", zorder=i)
        plt.plot(mean, mean_density, 'o', color=colors[i], label=None, zorder=i)

    for i in range(len(data)):
        if labels[i] == "P2P":
            continue

        # plot worst-case
        worst_case = np.max(data[i])

        # find max height of density_lines at worst_case
        max_height = 0.01
        for line in density_lines:
            xdata, ydata = line.get_data()
            height = np.interp(worst_case, xdata, ydata)
            if height > max_height:
                max_height = height

        # plt.plot(worst_case, 0*2*max_height, 'x', color=colors[i], label=f"{labels[i]} worst case", zorder=99, clip_on=False)
        plt.plot(worst_case, 0, 'x', color=colors[i], label=None, zorder=99, clip_on=False)

    extra_legend_handles = [
        plt.Line2D([0], [0], linestyle='', marker='o', color='gray', label="Mean"),
        plt.Line2D([0], [0], linestyle='', marker='x', color='gray', label="Worst case")
    ]
    extra_labels = ["Mean", "Worst case"]

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    # plt.legend(loc='best', ncol=3)
    plt.xlim(left=0)
    # plt.gca().set_xlim(left=0, right=150)

    return extra_legend_handles, extra_labels

def compare_travel_time_plus_total_comp_time(results, baseline_method, other_method, baseline_color, other_color):

    # compute the sum of the travel time and the total computation time
    Tf_baseline = np.array(results[baseline_method]["Tf"])
    t_comp_total_baseline = 0.001*np.array(results[baseline_method]["t_comp_total"])
    sum_baseline = Tf_baseline + t_comp_total_baseline

    Tf_other = np.array(results[other_method]["Tf"])
    t_comp_total_other = 0.001*np.array(results[other_method]["t_comp_total"])
    sum_other = Tf_other + t_comp_total_other

    # find indices of all successful cases
    idx = np.logical_and(
        np.logical_and(
            np.array(results[baseline_method]["t_comp_solver"]) >= 0,
            np.array(results[other_method]["t_comp_solver"]) >= 0
        ),
        np.logical_and(
            Tf_baseline >= 0,
            Tf_other >= 0
        )
    )


    # compute relative difference
    rel_diff = (sum_other[idx] - sum_baseline[idx]) / sum_baseline[idx] * 100

    # sort the relative difference
    idx = np.argsort(rel_diff)

    # plot the relative difference
    plt.fill_between(np.linspace(0, 1, len(rel_diff)), 0, rel_diff[idx], color=other_color, alpha=1.0)

    # plot the zero line
    plt.axhline(0, color='k', linestyle='-')
    
    # set the limits
    plt.xlim([0, 1])

    # set the labels
    plt.xlabel("Random environments")
    plt.ylabel("Relative difference [%]")

    # find cases where the relative difference is larger than 0%
    idx = np.where(rel_diff > 0)[0]

    print(idx)

def create_latex_table(results):
    # create a table with result.keys() (methods) as columns
    # the rows are:
    # - average t_solver
    # - average t_total
    # - average Tf
    # - worst case t_solver
    # - worst case t_total
    # - number of infeasible cases

    # create a dictionary with the data
    data = {}
    for method in results.keys():
        data[method] = {
            "t_solver": np.array(results[method]["t_comp_solver"]),
            "t_total": np.array(results[method]["t_comp_total"]),
            "Tf": np.array(results[method]["Tf"]),
            "infeasible": np.array(results[method]["corridor_infeasibilities_detected"])
        }

    row_names = ["average $t_\mathrm{solver}$ [ms]", 
                 "average $t_\mathrm{total}$ [ms]",
                 "average $T^*$ [s]", 
                 "worst case $t_\mathrm{solver}$ [ms]", 
                 "worst case $t_\mathrm{total}$ [ms]", 
                 "\# infeasible cases",
                 "total moving time [min]"]

    feasible_idx = np.logical_and(
        np.array(results["OCP-30"]["t_comp_solver"]) >= 0,
        np.array(results["OCP-30"]["t_comp_total"]) >= 0)

    # create the table
    table = {}
    for method in ["ARENA", "OmgTools", "OCP-30", "P2P"]:
    # for method in data.keys():
        table[method] = {
            "average $t_\mathrm{solver}$ [ms]": np.mean(data[method]["t_solver"]),
            "average $t_\mathrm{total}$ [ms]": np.mean(data[method]["t_total"]),
            "average $T^*$ [s]": np.mean(data[method]["Tf"]),
            "worst case $t_\mathrm{solver}$ [ms]": np.max(data[method]["t_solver"]),
            "worst case $t_\mathrm{total}$ [ms]": np.max(data[method]["t_total"]),
            "\# infeasible cases": np.sum(data[method]["infeasible"]),
            "total moving time [min]": np.sum(data[method]["Tf"][feasible_idx]/60.0),
        }

    # print the table
    print("\t\\begin{tabular}{l|ccc|c}")
    # print("\\hline")
    
    # print header (method names)
    print("\t\t" + " & ".join([""] + list(table.keys())) + " \\\\")
    print(f"\t\t\\hline")

    # print rows
    for row_name in row_names:
        row_values = [table[method][row_name] for method in table.keys()]
        min_idx = np.argmin(row_values[:-1]) # discard P2P
        row_value_strings = [f"{table[method][row_name]:.2f}" if row_name != "\# infeasible cases" else f"{table[method][row_name]}" for method in table.keys()]
        row_value_strings[min_idx] = "\\textbf{" + row_value_strings[min_idx] + "}"

        for i in range(len(row_value_strings)):
            if row_value_strings[i] == "0.00":
                row_value_strings[i] = "-"

        row = [row_name] + row_value_strings
        print("\t\t" + f" & ".join(row) + " \\\\")
        # print("\\hline")

    print("\t\\end{tabular}")

# print out all infeasible ARENA cases
infeasibles = []
for i in range(len(results["ARENA"]["corridor_infeasibilities_detected"])):
    if results["ARENA"]["corridor_infeasibilities_detected"][i]:
        infeasibles.append(i)
print(f"ARENA infeasible cases ({len(infeasibles)}): {infeasibles}")

# print out all infeasible OCP cases
infeasibles = []
for i in range(len(results["OCP-30"]["corridor_infeasibilities_detected"])):
    if results["OCP-30"]["corridor_infeasibilities_detected"][i]:
        infeasibles.append(i)
print(f"OCP-30 infeasible cases ({len(infeasibles)}): {infeasibles}")

# print out all infeasible P2P cases
infeasibles = []
for i in range(len(results["P2P"]["corridor_infeasibilities_detected"])):
    if results["P2P"]["corridor_infeasibilities_detected"][i]:
        infeasibles.append(i)
print(f"P2P infeasible cases ({len(infeasibles)}): {infeasibles}")

# print out all infeasible OmgTools cases
infeasibles = []
for i in range(len(results["OmgTools"]["corridor_infeasibilities_detected"])):
    if results["OmgTools"]["corridor_infeasibilities_detected"][i]:
        infeasibles.append(i)
print(f"OmgTools infeasible cases ({len(infeasibles)}): {infeasibles}")


import matplotlib.pyplot as plt

# plt.figure(figsize=(6,2))
# computation_time_comparison_extended_new(results, "OCP-30", 
#                                          ["ARENA", "OmgTools"], 
#                                          ["navy", "red"])

# plt.figure(figsize=(6,2))
# computation_time_comparison_extended_new(results, "OCP-30", 
#                                          ["ARENA", "OmgTools"], 
#                                          ["navy", "red"],
#                                          use_solver_time=True)


# plt.figure(figsize=(6,2))
# # scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# # scatter(results, "OCP-10", "t_comp_total", "Tf", "red")
# # scatter(results, "OCP-20", "t_comp_total", "Tf", "red")
# scatter(results, "OCP-30", "t_comp_total", "Tf", "red")
# # scatter(results, "OCP-40", "t_comp_total", "Tf", "red")
# scatter(results, "ARENA", "t_comp_total", "Tf", "royalblue")
# scatter(results, "P2P", "t_comp_total", "Tf", "orange")
# scatter(results, "OmgTools", "t_comp_total", "Tf", "black")
# plt.savefig("python-benchmark/figures/t_comp_total_vs_Tf.png", dpi=300)

# plt.figure(figsize=(6,2))
# # scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# # scatter(results, "OCP-10", "t_comp_solver", "Tf", "red")
# # scatter(results, "OCP-20", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-30", "t_comp_solver", "Tf", "red")
# # scatter(results, "OCP-40", "t_comp_solver", "Tf", "red")
# scatter(results, "ARENA", "t_comp_solver", "Tf", "royalblue")
# scatter(results, "P2P", "t_comp_solver", "Tf", "orange")
# scatter(results, "OmgTools", "t_comp_solver", "Tf", "black")
# plt.savefig("python-benchmark/figures/t_comp_solver_vs_Tf.png", dpi=300)

optimality_comparison_extended_new(results, "OCP-30", 
                                   ["P2P", "OmgTools", "ARENA"], 
                                   ["orange", "black", "royalblue"])
plt.savefig("python-benchmark/figures/optimality_comparison.png", dpi=300)
plt.savefig("python-benchmark/figures/optimality_comparison.pdf")

# plt.figure()
# compare_travel_time_plus_total_comp_time(results, "OCP-30", "ARENA", "red", "royalblue")

show_histogram_densities(results, ["ARENA", "OCP-30", "P2P", "OmgTools"], ["royalblue", "red", "orange", "black"])
plt.savefig("python-benchmark/figures/densities.png", dpi=300)
plt.savefig("python-benchmark/figures/densities.pdf")

create_latex_table(results)

plt.show()