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

def optimality_comparison_extended_new_new(results, baseline_method, methods, colors, use_abs_error=False):
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
        plt.plot(np.linspace(0, 100, len(rel_errors[i])), rel_errors[i], color=colors[i], label=translate_method_names(methods)[i], linewidth=2)
        alpha = 0.4 if colors[i] != "black" else 0.2
        plt.fill_between(np.linspace(0, 100, len(rel_errors[i])), 0, rel_errors[i], color=colors[i], alpha=alpha, label=None)

    plt.axhline(0, color='k', linestyle='-')
    plt.xlim([0, 100])
    plt.ylim([1.0e-2, 2.0e2]); 
    # plt.axhline(50, c='k', ls='-', lw=1, zorder=-1)
    a = 8
    w = 3.5
    plt.hlines(5, xmin=0, xmax=a-w, colors='k', ls='-', lw=1, zorder=-1)
    plt.hlines(5, xmin=a+w, xmax=100, colors='k', ls='-', lw=1, zorder=-1)
    plt.hlines(1, xmin=0, xmax=a-w, colors='k', ls='-', lw=1, zorder=-1)
    plt.hlines(1, xmin=a+w, xmax=100, colors='k', ls='-', lw=1, zorder=-1)

    # plt.text(15, 50, f"$10\%$", fontsize=15, ha='center', va='bottom', color='k')
    plt.text(a, 5, f"$5\%$", fontsize=15, ha='center', va='center', color='k')
    plt.text(a, 1, f"$1\%$", fontsize=15, ha='center', va='center', color='k')

    plt.vlines(27.3, ymin=0.001, ymax=5, colors='k', ls='-', lw=1, zorder=10)
    plt.vlines(89.5, ymin=0.001, ymax=1, colors='k', ls='-', lw=1, zorder=10)
    plt.vlines(96.3, ymin=0.001, ymax=5, colors='k', ls='-', lw=1, zorder=10)
    plt.xticks([0, 20, 27, 40, 60, 80, 90, 100])
    plt.yscale('log'); 
    plt.yticks([1.0e-2, 1.0e-1, 1.0e0, 1.0e1, 1.0e2])


    plt.xlabel("\% of Benchmark Environments")
    if use_abs_error:
        plt.ylabel("Absolute suboptimality [s]")
    else:
        # plt.ylabel("Relative\nsuboptimality [\%]")
        plt.ylabel("Relative error\non $T^*$ [\%]")
    # plt.gcf().legend(loc='lower center', ncol = 3, frameon=False)
    plt.gcf().legend(bbox_to_anchor=(0.98, 0.18), ncol = 3, frameon=False)


    plt.tight_layout(rect=[0, 0.12, 1, 1])

    REMOVE_ANNOTATION = True

    # highlight the 0.95 percentile
    for i in range(len(rel_errors)):
        if methods[i] == "ARENA":
            p = 1
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='royalblue', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (100*idx[0]/len(rel_errors[i]), p)
                arrow_end = (56, 34.6)
                arrow_end = (40, 96)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx[0]/len(rel_errors[i])*100:.0f}\% of\nenvironments",
                            xy=arrow_start, xycoords='data',
                            xytext=arrow_end, textcoords='data',
                            # arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.2"),
                            # show a slightly curved arrow with a solid arrowhead
                            arrowprops=dict(arrowstyle="-|>", 
                                            connectionstyle="arc3,rad=.1", 
                                            lw=1,
                                            color='royalblue'),
                            fontsize=15, color='royalblue'
                            )
            
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='royalblue', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (100*idx[0]/len(rel_errors[i]), p)
                arrow_end = (71, 252)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx[0]/len(rel_errors[i])*100:.0f}\% of\nenvironments",
                            xy=arrow_start, xycoords='data',
                            xytext=arrow_end, textcoords='data',
                            # arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0.2"),
                            # show a slightly curved arrow with a solid arrowhead
                            arrowprops=dict(arrowstyle="-|>", 
                                            connectionstyle="arc3,rad=-.1", 
                                            lw=1,
                                            color='royalblue'),
                            fontsize=15, color='royalblue'
                            )
            
        if methods[i] == "OmgTools":
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            plt.plot(100*idx[0]/len(rel_errors[i]), p, 'o', color='black', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (100*idx[0]/len(rel_errors[i]), p)
                arrow_end = (4, 85)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx[0]/len(rel_errors[i])*100:.0f}\% of\nenvironments",
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

def show_histogram_densities(results, methods, colors):
    # # Travel time
    # Tf = [np.array(results[method]["Tf"]) for method in methods]
    # plot_densities(Tf, colors, methods, "Travel time [s]", "Density")

    OMIT_TOTAL_TIME = True

    if OMIT_TOTAL_TIME:
        fig, axes = plt.subplots(1, 1, figsize=(6, 3))

        # Solver computation time
        plt.sca(axes)
        t_comp_solver = [np.array(results[method]["t_comp_solver"]) for method in methods]
        # method_sequence = {"OCP-30-FATROP":2, "OmgTools":3, "ARENA":1, "ARENA-FATROP":0, "P2P":4}
        method_sequence = {2:"OCP-30-FATROP", 1:"OmgTools", 3:"ARENA", 4:"ARENA-FATROP", 0:"P2P"}
        idx = [methods.index(method_sequence[i]) for i in range(len(method_sequence))]
        t_comp_solver = [t_comp_solver[i] for i in idx]
        colors = [colors[i] for i in idx]
        methods = [methods[i] for i in idx]
        extra_handles, extra_labels = plot_densities(t_comp_solver, colors, methods, "Solver computation time [ms]", "Density")

        handles, labels = axes.get_legend_handles_labels()  # Get legend items from one subplot
        all_handles = handles + extra_handles
        all_labels = labels + extra_labels
        # permutation = [4, 2, 0, 1, 3]
        permutation = [2, 3, 1, 4, 0, 5]
        all_handles = [all_handles[i] for i in permutation]
        all_labels = [all_labels[i] for i in permutation]
        all_labels = translate_method_names(all_labels)
        fig.legend(all_handles, all_labels, loc='lower center', ncol=3, frameon=False)  # Shared legend below
        plt.tight_layout(rect=[0, 0.2, 1, 1])  # Adjust layout to fit legend
        axes.set_xlim(right=78)
    else:
        fig, axes = plt.subplots(2, 1)  # Two subplots stacked vertically

        # Total computation time
        plt.sca(axes[1])
        t_comp_total = [np.array(results[method]["t_comp_total"]) for method in methods]
        t_comp_total.reverse()
        colors.reverse()
        methods.reverse()
        omg_idx = methods.index("OmgTools")
        # remove omg tools from lists
        t_comp_total_copy = t_comp_total.copy()
        colors_copy = colors.copy()
        methods_copy = methods.copy()
        t_comp_total_copy.pop(omg_idx)
        colors_copy.pop(omg_idx)
        methods_copy.pop(omg_idx)
        plot_densities(t_comp_total_copy, colors_copy, methods_copy, "Total computation time [ms]", "Density")
        colors.reverse()
        methods.reverse()
        
        # Solver computation time
        plt.sca(axes[0])
        t_comp_solver = [np.array(results[method]["t_comp_solver"]) for method in methods]
        # method_sequence = {"OCP-30-FATROP":2, "OmgTools":3, "ARENA":1, "ARENA-FATROP":0, "P2P":4}
        method_sequence = {2:"OCP-30-FATROP", 1:"OmgTools", 3:"ARENA", 4:"ARENA-FATROP", 0:"P2P"}
        idx = [methods.index(method_sequence[i]) for i in range(len(method_sequence))]
        t_comp_solver = [t_comp_solver[i] for i in idx]
        colors = [colors[i] for i in idx]
        methods = [methods[i] for i in idx]
        extra_handles, extra_labels = plot_densities(t_comp_solver, colors, methods, "Solver computation time [ms]", "Density")

        handles, labels = axes[0].get_legend_handles_labels()  # Get legend items from one subplot
        all_handles = handles + extra_handles
        all_labels = labels + extra_labels
        # permutation = [4, 2, 0, 1, 3]
        permutation = [2, 3, 1, 4, 0, 5]
        all_handles = [all_handles[i] for i in permutation]
        all_labels = [all_labels[i] for i in permutation]
        all_labels = translate_method_names(all_labels)
        fig.legend(all_handles, all_labels, loc='lower center', ncol=3, frameon=False)  # Shared legend below
        plt.tight_layout(rect=[0, 0.15, 1, 1])  # Adjust layout to fit legend
        axes[1].set_xlim(right=85)
        axes[0].set_xlim(right=85)

def plot_densities(data, colors, labels, xlabel, ylabel):
    # plt.figure()
    density_lines = []
    for i in range(len(data)):
        if labels[i] == "P2P":
            continue
        
        # plot density
        if labels[i] == "ARENA-FATROP":
            sns.kdeplot(data[i], color=colors[i], label=labels[i], fill=True, alpha=0., hatch='//')
        else:
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

    # filter out all entries where ocp fails
    failures = np.logical_or(
        np.logical_or(
            np.array(results["OCP-30"]["t_comp_solver"]) < 0,
            np.array(results["OCP-30"]["t_comp_total"]) < 0),
        np.logical_or(
            np.array(results["ARENA-FATROP"]["t_comp_solver"]) < 0,
            np.array(results["ARENA-FATROP"]["t_comp_total"]) < 0))
    failures = np.logical_or(
        failures,
        np.logical_or(
            np.array(results["OmgTools"]["t_comp_solver"]) < 0,
            np.array(results["OmgTools"]["t_comp_total"]) < 0),
    )
    
    for method in results.keys():
        results[method]["t_comp_solver"] = np.array(results[method]["t_comp_solver"])
        results[method]["t_comp_total"] = np.array(results[method]["t_comp_total"])
        results[method]["Tf"] = np.array(results[method]["Tf"])
        results[method]["corridor_infeasibilities_detected"] = np.array(results[method]["corridor_infeasibilities_detected"])

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
                 "\# solver failures",
                 "total moving time [min]"]

    # create the table
    table = {}
    for method in ["ARENA", "ARENA-FATROP", "OCP-30-FATROP", "OmgTools", "P2P"]:
    # for method in data.keys():
        table[method] = {
            "average $t_\mathrm{solver}$ [ms]": np.mean(data[method]["t_solver"][~failures]),
            "average $t_\mathrm{total}$ [ms]": np.mean(data[method]["t_total"][~failures]),
            "average $T^*$ [s]": np.mean(data[method]["Tf"][~failures]),
            "worst case $t_\mathrm{solver}$ [ms]": np.max(data[method]["t_solver"][~failures]),
            "worst case $t_\mathrm{total}$ [ms]": np.max(data[method]["t_total"][~failures]),
            "\# infeasible cases": np.sum(data[method]["infeasible"][~failures]),
            "\# solver failures": np.sum(data[method]["t_solver"] < 0),
            "total moving time [min]": np.sum(data[method]["Tf"][~failures]/60.0),
        }

    # print the table
    print("\t\\begin{tabular}{l|cccc|c}")
    # print("\\hline")
    
    # print header (method names)
    method_names = list(table.keys())
    print("\t\t" + " & ".join([""] + translate_method_names(method_names)) + " \\\\")
    print(f"\t\t\\hline")

    # print rows
    for row_name in row_names:
        row_values = [table[method][row_name] for method in table.keys()]
        min_idx = np.argmin(row_values[:-1]) # discard P2P
        row_value_strings = [f"{table[method][row_name]:.2f}" 
                if row_name != "\# infeasible cases" and row_name != "\# solver failures" 
                else f"{table[method][row_name]}" for method in table.keys()]
        row_value_strings[min_idx] = "\\textbf{" + row_value_strings[min_idx] + "}"

        for i in range(len(row_value_strings)):
            if row_value_strings[i] == "0.00":
                row_value_strings[i] = "-"

        row = [row_name] + row_value_strings
        print("\t\t" + f" & ".join(row) + " \\\\")
        # print("\\hline")

    print("\t\\end{tabular}")

def filter_results_for_fair_comparison(results):
    filtered_results = {}

    # start by keeping all results
    failures = np.array([False]*len(results["OCP-30"]["t_comp_solver"]))

    for method in results.keys():
        failures = np.logical_or(failures, np.array(results[method]["t_comp_solver"]) < 0)
        failures = np.logical_or(failures, np.array(results[method]["t_comp_total"]) < 0)
        failures = np.logical_or(failures, np.array(results[method]["Tf"]) < 0)

    for method in results.keys():
        filtered_results[method] = {}
        filtered_results[method]["t_comp_solver"] = np.array(results[method]["t_comp_solver"])[~failures]
        filtered_results[method]["t_comp_total"] = np.array(results[method]["t_comp_total"])[~failures]
        filtered_results[method]["Tf"] = np.array(results[method]["Tf"])[~failures]
        filtered_results[method]["corridor_infeasibilities_detected"] = np.array(results[method]["corridor_infeasibilities_detected"])[~failures]

    return filtered_results

def translate_method_names(method_names):
    translation = []

    for method in method_names:
        if method == "ARENA":
            translation.append("PMP")
        elif method == "ARENA-FATROP":
            translation.append("PMP-F")
        elif method == "OCP-30":
            translation.append("OCP")
        elif method == "OCP-30-FATROP":
            translation.append("OCP-F")
        else:
            translation.append(method)

    return translation

# print out all infeasible cases
for method in ["ARENA", "ARENA-FATROP", "OCP-30", "OCP-30-FATROP", "P2P", "OmgTools"]:
    infeasibles = []
    for i in range(len(results[method]["corridor_infeasibilities_detected"])):
        if results[method]["corridor_infeasibilities_detected"][i]:
            infeasibles.append(i)
    print(f"{method} infeasible cases ({len(infeasibles)}): {infeasibles}")

import matplotlib.pyplot as plt

filtered_results = filter_results_for_fair_comparison(results)

# tfs_arena = np.array(filtered_results["ARENA"]["Tf"])
# tfs_arena_fatrop = np.array(filtered_results["ARENA-FATROP"]["Tf"])
# diff = tfs_arena - tfs_arena_fatrop
# plt.figure()
# sns.kdeplot(diff, color='royalblue', label="ARENA - ARENA-FATROP", fill=True, alpha=0.5)
# plt.show()

optimality_comparison_extended_new_new(filtered_results, "OCP-30", 
                                   ["P2P", "OmgTools", "ARENA"], 
                                   ["orange", "black", "royalblue"])
plt.savefig("python-benchmark/figures/optimality_comparison.png", dpi=300)
plt.savefig("python-benchmark/figures/optimality_comparison.pdf")

# plt.figure()
# compare_travel_time_plus_total_comp_time(results, "OCP-30", "ARENA", "red", "royalblue")

show_histogram_densities(filtered_results, ["ARENA-FATROP", "ARENA", "OCP-30-FATROP", "P2P", "OmgTools"], ["royalblue", "royalblue", "red", "orange", "black"])
plt.savefig("python-benchmark/figures/densities.png", dpi=300)
plt.savefig("python-benchmark/figures/densities.pdf")

create_latex_table(results)

plt.show()