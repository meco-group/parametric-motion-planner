import json
import matplotlib.pyplot as plt
import numpy as np

# with open('python-benchmark/files/results.json', 'r') as f:
# with open('python-benchmark/files/results_cell.json', 'r') as f:
with open('python-benchmark/files/results_double.json', 'r') as f:
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



import matplotlib.pyplot as plt
plt.figure()
optimality_comparison(results, "OCP-30", "ARENA", "red", "royalblue")

plt.figure()
# scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-10", "t_comp_total", "Tf", "red")
# scatter(results, "OCP-20", "t_comp_total", "Tf", "red")
scatter(results, "OCP-30", "t_comp_total", "Tf", "red")
# scatter(results, "OCP-40", "t_comp_total", "Tf", "red")
scatter(results, "ARENA", "t_comp_total", "Tf", "royalblue")
scatter(results, "P2P", "t_comp_total", "Tf", "orange")
plt.savefig("python-benchmark/figures/t_comp_total_vs_Tf.png", dpi=300)

plt.figure()
# scatter(results, "OCP-5", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-10", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-20", "t_comp_solver", "Tf", "red")
scatter(results, "OCP-30", "t_comp_solver", "Tf", "red")
# scatter(results, "OCP-40", "t_comp_solver", "Tf", "red")
scatter(results, "ARENA", "t_comp_solver", "Tf", "royalblue")
scatter(results, "P2P", "t_comp_solver", "Tf", "orange")
plt.savefig("python-benchmark/figures/t_comp_solver_vs_Tf.png", dpi=300)


plt.figure()
computation_time_comparison(results, "OCP-30", "ARENA", "red", "royalblue")

plt.show()