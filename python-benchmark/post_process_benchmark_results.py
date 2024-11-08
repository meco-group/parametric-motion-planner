import json
import matplotlib.pyplot as plt
import numpy as np

# with open('python-benchmark/files/results.json', 'r') as f:
# with open('python-benchmark/files/results_cell.json', 'r') as f:
# with open('python-benchmark/files/results_double.json', 'r') as f:
with open('python-benchmark/files/results_large.json', 'r') as f:
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
    for i in range(len(rel_errors)):
        idx = np.argsort(rel_errors[i])
        rel_errors[i] = rel_errors[i][idx]
        og_idxs[i] = og_idxs[i][idx]
        print(f"10 most suboptimal cases for method {methods[i]}:")
        print(og_idxs[i][-10:])

    # visualize
    plt.fill_between(np.linspace(0, 1, len(rel_errors[0])), -1, 1, color='gray', alpha=0.5)
    for i in range(len(rel_errors)):
        plt.fill_between(np.linspace(0, 1, len(rel_errors[i])), 0, rel_errors[i], color=colors[i], alpha=1.0, label=methods[i])

    plt.axhline(0, color='k', linestyle='-')
    plt.xlim([0, 1])
    plt.ylim([-1.2, 30])
    plt.xlabel("Random environments")
    if use_abs_error:
        plt.ylabel("Absolute suboptimality [s]")
    else:
        plt.ylabel("Relative suboptimality [%]")
    plt.legend(loc='best')

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


import matplotlib.pyplot as plt
# plt.figure()
# optimality_comparison(results, "OCP-30", "ARENA", "red", "royalblue")

# plt.figure()
# optimality_comparison_extended(results, "OCP-30", "ARENA+", "ARENA", "red", "royalblue", "navy")

# plt.figure(figsize=(6,2))
# computation_time_comparison_extended_new(results, "OCP-30", 
#                                          ["ARENA", "ARENA+", "OmgTools"], 
#                                          ["navy", "royalblue", "maroon"])

# plt.figure(figsize=(6,2))
# computation_time_comparison_extended_new(results, "OCP-30", 
#                                          ["ARENA", "ARENA+", "OmgTools"], 
#                                          ["navy", "royalblue", "maroon"],
#                                          use_solver_time=True)
plt.figure(figsize=(6,2))
computation_time_comparison_extended_new(results, "OCP-30", 
                                         ["ARENA", "OmgTools"], 
                                         ["navy", "red"])

plt.figure(figsize=(6,2))
computation_time_comparison_extended_new(results, "OCP-30", 
                                         ["ARENA", "OmgTools"], 
                                         ["navy", "red"],
                                         use_solver_time=True)


plt.figure(figsize=(6,2))
# scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-10", "t_comp_total", "Tf", "red")
# scatter(results, "OCP-20", "t_comp_total", "Tf", "red")
scatter(results, "OCP-30", "t_comp_total", "Tf", "red")
# scatter(results, "OCP-40", "t_comp_total", "Tf", "red")
scatter(results, "ARENA", "t_comp_total", "Tf", "royalblue")
scatter(results, "P2P", "t_comp_total", "Tf", "orange")
scatter(results, "OmgTools", "t_comp_total", "Tf", "black")
plt.savefig("python-benchmark/figures/t_comp_total_vs_Tf.png", dpi=300)

plt.figure(figsize=(6,2))
# scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-10", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-20", "t_comp_solver", "Tf", "red")
scatter(results, "OCP-30", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-40", "t_comp_solver", "Tf", "red")
scatter(results, "ARENA", "t_comp_solver", "Tf", "royalblue")
scatter(results, "P2P", "t_comp_solver", "Tf", "orange")
scatter(results, "OmgTools", "t_comp_solver", "Tf", "black")
plt.savefig("python-benchmark/figures/t_comp_solver_vs_Tf.png", dpi=300)

# plt.figure()
# computation_time_comparison(results, "OCP-30", "ARENA", "red", "royalblue")

# plt.figure()
# computation_time_comparison_extended(results, "OCP-30", "ARENA+", "ARENA", "red", "royalblue", "navy")

# plt.figure(figsize=(6,2))
# optimality_comparison_extended_new(results, "OCP-30", 
#                                    ["P2P", "OmgTools", "ARENA", "ARENA+"], 
#                                    ["orange", "maroon", "navy", "royalblue"])
plt.figure(figsize=(6,2))
optimality_comparison_extended_new(results, "OCP-30", 
                                   ["P2P", "OmgTools", "ARENA"], 
                                   ["orange", "red", "navy"])
# plt.figure()
# optimality_comparison_extended_new(results, "OCP-30", 
#                                    ["P2P", "OmgTools", "ARENA+", "ARENA"], 
#                                    ["orange", "maroon", "royalblue", "navy"],
#                                    use_abs_error=True)

plt.show()