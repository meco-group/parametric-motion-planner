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

SAVE_FIGURES = True

# with open('python-benchmark/files/results.json', 'r') as f:
# with open('python-benchmark/files/results_cell.json', 'r') as f:
# with open('python-benchmark/files/results_double.json', 'r') as f:
# with open('python-benchmark/files/results_large.json', 'r') as f:
# with open('python-benchmark/files/results_large_double.json', 'r') as f:
# with open('python-benchmark/files/results_large_double_more_obstacles.json', 'r') as f:
# with open('python-benchmark/files/results_large_double_more_obstacles_10.json', 'r') as f:
with open('python-benchmark/files/results_new_large_double_more_obstacles_10.json', 'r') as f:
    results = json.load(f)

def optimality_comparison_extended_new_new(results, baseline_method, methods, colors, idx_map, use_abs_error=False):
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
        if methods[i] == "ARENA":
            for j in range(len(rel_errors[i])):
                if rel_errors[i][j] < -0.1:
                    print(f"[{idx_map[og_idxs[i][j]]:3d}] ARENA: {Tfs[i][j]:.4f}\t-\tOCP: {Tf_baseline[j]}\t({rel_errors[i][j]:.4f})")
        rel_errors[i] = rel_errors[i][idx]
        og_idxs[i] = og_idxs[i][idx]
        my_dict = {idx_map[og_idxs[i][j]]: round(rel_errors[i][j],2) for j in range(len(rel_errors[i])-10, len(rel_errors[i]))}
        print(f"10 most suboptimal cases for method {methods[i]}:")
        # print(og_idxs[i][-10:])
        print(my_dict)
        
        my_dict = {idx_map[og_idxs[i][j]]: round(rel_errors[i][j],2) for j in range(10)}
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


    plt.xlabel("\% of Benchmark Environments")
    if use_abs_error:
        plt.ylabel("Absolute suboptimality [s]")
    else:
        # plt.ylabel("Relative\nsuboptimality [\%]")
        plt.ylabel("Relative error\non $t_{\mathrm{move}}$ [\%]")
    # plt.gcf().legend(loc='lower center', ncol = 3, frameon=False)
    plt.gcf().legend(bbox_to_anchor=(0.98, 0.18), ncol = len(methods), frameon=False)

    plt.yscale('log'); 
    plt.yticks([1.0e-2, 1.0e-1, 1.0e0, 1.0e1, 1.0e2])

    plt.tight_layout(rect=[0, 0.12, 1, 1])

    REMOVE_ANNOTATION = True

    my_xticks = [0, 20, 40, 60, 80]

    # highlight the 0.95 percentile
    for i in range(len(rel_errors)):
        if methods[i] == "ARENA":
            p = 1
            idx = np.where(rel_errors[i] > p)[0]
            idx1 = 100*idx[0]/len(rel_errors[i])
            plt.plot(idx1, p, 'o', color='royalblue', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (idx1, p)
                arrow_end = (56, 34.6)
                arrow_end = (40, 96)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx1:.0f}\% of\nenvironments",
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
            plt.vlines(idx1, ymin=0.001, ymax=1, colors='k', ls='-', lw=1, zorder=10)
            
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            idx2 = 100*idx[0]/len(rel_errors[i])
            plt.plot(idx2, p, 'o', color='royalblue', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (idx2, p)
                arrow_end = (71, 252)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx2:.0f}\% of\nenvironments",
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
            plt.vlines(idx2, ymin=0.001, ymax=5, colors='k', ls='-', lw=1, zorder=10)
                
            my_xticks.append(round(idx1))
            my_xticks.append(round(idx2))
            my_xticks.sort()
            
        if methods[i] == "OmgTools":
            p = 5
            idx = np.where(rel_errors[i] > p)[0]
            idx3 = 100*idx[0]/len(rel_errors[i])
            plt.plot(idx3, p, 'o', color='black', markersize=7, zorder=11)
            if not REMOVE_ANNOTATION:
                arrow_start = (idx3, p)
                arrow_end = (4, 85)
                plt.annotate(f"error less\nthan {p:.0f}\%\nin {idx3:.0f}\% of\nenvironments",
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
            plt.vlines(idx3, ymin=0.001, ymax=5, colors='k', ls='-', lw=1, zorder=10)
            my_xticks.append(round(idx3))
            my_xticks.sort()

    plt.xticks(my_xticks, my_xticks)

def show_relative_difference_density(results, baseline_method, other_method, colors, idx_map, use_abs_error=False):
    Tf_baseline = np.array(results[baseline_method]["Tf"]) + 0*np.array(results[baseline_method]["t_comp_total"]) / 1000
    Tf = np.array(results[other_method]["Tf"]) + 0*np.array(results[other_method]["t_comp_total"]) / 1000

    # get indices where all methods are succesfull
    idx = np.array(results[baseline_method]["t_comp_solver"]) >= 0
    idx = np.logical_and(idx, Tf >= 0)
    Tf_baseline = Tf_baseline[idx]
    Tf = Tf[idx]

    # compute errors
    rel_errors = 100*(Tf - Tf_baseline) / Tf_baseline
    if use_abs_error:
        rel_errors = Tf - Tf_baseline

    # find the indices of the 3 largest rel_errors
    idx = np.argsort(rel_errors)
    idx = idx[-5:]
    print([idx_map[i] for i in idx])
    print(rel_errors[idx])
    print(Tf_baseline[idx])

    # plot a density of the relative error
    # fig, axes = plt.subplots(1, 1, figsize=(6, 3))
    # plt.sca(axes)
    # plot_densities(rel_errors, colors, methods, "Relative error on $t_{\mathrm{move}}$ [\%]", "Density")
    # plt.tight_layout(rect=[0, 0.2, 1, 1])  # Adjust layout to fit legend
    
    # plt.figure(figsize=(6,3))
    # plt.hist(rel_errors, bins=50, density=True, color=colors, label=translate_method_names(methods), alpha=0.5)
        
    tresholds = np.linspace(1, 35, 1000)
    percentage_of_environments = np.zeros((len(tresholds)))
    for i in range(len(tresholds)):
        percentage_of_environments[i] = np.sum(rel_errors > tresholds[i]) / len(rel_errors) * 100
        if percentage_of_environments[i] > 0:
            max_trheshold = tresholds[i]
    
    plt.figure(figsize=(6,3))
    # plt.plot(tresholds, percentage_of_environments, color=colors[0], label=translate_method_names([other_method])[0], linewidth=2)
    plt.fill_between(tresholds, 0, percentage_of_environments, color=colors[0], alpha=0.5)

    # draw vertical line at 5% and 10% and horizontal lines at the corresponding value
    percs = [1, 5, 10]
    for perc in percs:
        idx = np.where(tresholds > perc)[0][0]
        plt.plot([perc, perc], [0, percentage_of_environments[idx]], color='k', linestyle='-', linewidth=1)
        plt.plot([0, perc], [percentage_of_environments[idx], percentage_of_environments[idx]], color='k', linestyle='-', linewidth=1)
        plt.plot(perc, percentage_of_environments[idx], 'o', color='k')
        plt.text(perc+0.5, percentage_of_environments[idx], f"{percentage_of_environments[idx]:.1f}\%", fontsize=15, ha='left', va='bottom', color='k')

    plt.xlim([tresholds[0], max_trheshold+5])
    plt.ylim([0, 1.1*np.max(percentage_of_environments)])
    plt.xlabel("Relative reduction on $t_{\mathrm{move}}$ [\%]")
    plt.ylabel("Percentage of environments\nachieving the reduction [\%]")

    # make sure to add 1 as an xtick
    my_xticks = [1, 5, 10, 15, 20, 25, 30]
    plt.xticks(my_xticks, my_xticks)

    plt.tight_layout()
    

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
        if labels[i] == "ARENA-FATROP":
            plt.scatter(worst_case, 0, s=70, marker='D', facecolor='white', color=colors[i], label=None, zorder=99, clip_on=False, alpha=0.2)
            plt.scatter(worst_case, 0, s=70, marker='D', hatch='////', facecolor='none', color=colors[i], label=None, zorder=99, clip_on=False)
        else:
            plt.plot(worst_case, 0, 'D', ms=8, color=colors[i], label=None, zorder=99, clip_on=False, alpha=0.5)
        plt.plot(worst_case, 0, 'D', ms=8, mfc='none', color=colors[i], label=None, zorder=99, clip_on=False, alpha=1.0)

    extra_legend_handles = [
        plt.Line2D([0], [0], linestyle='', marker='o', color='gray', label="Mean"),
        plt.Line2D([0], [0], linestyle='', marker='D', color='gray', label="Worst case")
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

def create_latex_table(results, corridor_evaluation=False):
    # create a table with result.keys() (methods) as columns
    # the rows are:
    # - average t_solver
    # - average t_total
    # - average Tf
    # - worst case t_solver
    # - worst case t_total
    # - number of infeasible cases

    # filter out all entries where ocp fails
    failures = np.array([False]*len(results["OCP-30"]["t_comp_solver"]))
    for method in results.keys():
        failures = np.logical_or(failures, np.array(results[method]["t_comp_solver"]) < 0)
        failures = np.logical_or(failures, np.array(results[method]["t_comp_total"]) < 0)
        failures = np.logical_or(failures, np.array(results[method]["Tf"]) < 0)

    
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

    row_names = ["avg $t_\mathrm{solver}$ [ms]", 
                 "max $t_\mathrm{solver}$ [ms]", 
                 "avg $t_\mathrm{total}$ [ms]",
                 "max $t_\mathrm{total}$ [ms]", 
                 "avg $t_\mathrm{move}$ [s]", 
                 "total $t_\mathrm{move}$ [min]",
                 "\# infeasible cases",
                 "\# solver failures"]

    # create the table
    table = {}
    if corridor_evaluation:
        methods = ["ARENA-FATROP", "OCP-30-FATROP", "OCP-30-EXTENDED-FATROP"]
    else:
        methods = ["ARENA", "ARENA-FATROP", "OCP-30-FATROP", "OmgTools", "P2P"]
    
    for method in methods:
    # for method in data.keys():
        table[method] = {
            "avg $t_\mathrm{solver}$ [ms]": np.mean(data[method]["t_solver"][~failures]),
            "max $t_\mathrm{solver}$ [ms]": np.max(data[method]["t_solver"][~failures]),
            "avg $t_\mathrm{total}$ [ms]": np.mean(data[method]["t_total"][~failures]),
            "max $t_\mathrm{total}$ [ms]": np.max(data[method]["t_total"][~failures]),
            "avg $t_\mathrm{move}$ [s]": np.mean(data[method]["Tf"][~failures]),
            "total $t_\mathrm{move}$ [min]": np.sum(data[method]["Tf"][~failures]/60.0),
            "\# infeasible cases": np.sum(data[method]["infeasible"][~failures]),
            "\# solver failures": np.sum(data[method]["t_solver"] < 0),
        }

    # print the table
    print("\n")
    if corridor_evaluation:
        print("\t\\begin{tabular}{r|cc|c}")
    else:
        print("\t\\begin{tabular}{r|cccc|c}")
    print("\t\\toprule")
    
    # print header (method names)
    method_names = list(table.keys())
    print("\t\t& \\multicolumn{2}{c|}{$\\bm{C}$} & $\\bm{C}^+$ \\\\")
    print("\t\t" + " & ".join([""] + translate_method_names(method_names)) + " \\\\")
    print(f"\t\t\\midrule")

    # print rows
    for r in range(len(row_names)):
        row_name = row_names[r]
        row_values = [table[method][row_name] for method in table.keys()]
        min_idx = np.argmin(
            row_values[:-2] if not corridor_evaluation else row_values
        )
        row_value_strings = [(f"{table[method][row_name]:.2f}" 
                if row_name != "avg $t_\mathrm{move}$ [s]"
                else f"{table[method][row_name]:.3f}")
                if row_name != "\# infeasible cases" and row_name != "\# solver failures" 
                else f"{table[method][row_name]}" for method in table.keys()]
        row_value_strings[min_idx] = "\\textbf{" + row_value_strings[min_idx] + "}"

        for i in range(len(row_value_strings)):
            if row_value_strings[i] == "0.00":
                row_value_strings[i] = "-"

        row = [row_name] + row_value_strings
        if r in [3, 5]:
            print("\t\t" + f" & ".join(row) + " \\\\ [1em]")
        else:
            print("\t\t" + f" & ".join(row) + " \\\\")
    print("\t\t\\bottomrule")

    print("\t\\end{tabular}")
    print("\n")

def filter_results_for_fair_comparison(results):
    filtered_results = {}

    # start by keeping all results
    failures = np.array([False]*len(results["OCP-30"]["t_comp_solver"]))

    # print out indices where t_comp_solver < 0 for OCP-30-FATROP
    print(np.where(np.array(results['OCP-30-FATROP']['t_comp_solver']) < 0))

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

    # create a int-inr dictoniary showing filtered_results_idx->original_results_idx
    idx_map = {}
    for i in range(len(failures)):
        if not failures[i]:
            idx_map[len(idx_map)] = i

    return filtered_results, idx_map

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
        elif method == "OCP-30-EXTENDED-FATROP":
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

filtered_results, idx_map = filter_results_for_fair_comparison(results)

# tfs_arena = np.array(filtered_results["ARENA"]["Tf"])
# tfs_arena_fatrop = np.array(filtered_results["ARENA-FATROP"]["Tf"])
# diff = tfs_arena - tfs_arena_fatrop
# plt.figure()
# sns.kdeplot(diff, color='royalblue', label="ARENA - ARENA-FATROP", fill=True, alpha=0.5)
# plt.show()

# print(f"OCP failure case: {np.where(np.array(results['OCP-30-FATROP']['t_comp_solver']) < 0)}")
optimality_comparison_extended_new_new(filtered_results, "OCP-30", 
                                   ["P2P", "OmgTools", "ARENA"], 
                                   ["orange", "black", "royalblue"], idx_map)
if SAVE_FIGURES:
    plt.savefig("python-benchmark/figures/optimality_comparison.png", dpi=300)
    plt.savefig("python-benchmark/figures/optimality_comparison.pdf")

# plt.figure()
# compare_travel_time_plus_total_comp_time(results, "OCP-30", "ARENA", "red", "royalblue")

show_histogram_densities(filtered_results, ["ARENA-FATROP", "ARENA", "OCP-30-FATROP", "P2P", "OmgTools"], ["royalblue", "royalblue", "red", "orange", "black"])
if SAVE_FIGURES:
    plt.savefig("python-benchmark/figures/densities.png", dpi=300)
    plt.savefig("python-benchmark/figures/densities.pdf")

create_latex_table(results)
create_latex_table(results, True)

## CORRIDOR EVALUATION
optimality_comparison_extended_new_new(filtered_results, "OCP-30-EXTENDED-FATROP",
                                ["ARENA-FATROP", "OCP-30-FATROP"], 
                                ["royalblue", "red"], idx_map)
# if SAVE_FIGURES:
#     plt.savefig("python-benchmark/figures/optimality_comparison_corridor_extension.png", dpi=300)

# filtered_results["OCP-30-FATROP"] = filtered_results["OCP-30-EXTENDED-FATROP"]
# show_histogram_densities(filtered_results, 
#     ["ARENA-FATROP", "ARENA", "OCP-30-FATROP", "P2P", "OmgTools"], 
#     ["royalblue", "royalblue", "red", "orange", "black"])
show_relative_difference_density(filtered_results, "OCP-30-EXTENDED-FATROP",
    "OCP-30-FATROP", ["red"], idx_map)
if SAVE_FIGURES:
    plt.savefig("python-benchmark/figures/relative-reduction-corridor-extension.png", dpi=300)

plt.show()