import json
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

import parametric_motion_planner_module as pmp
from load_random_environments import extract_data

def print_stats(Tf, t_comp_total, t_comp_solver):
    print(f"\tTf: \t\t\t\t{Tf:.3f} s")
    print(f"\tTotal computation time: \t{t_comp_total:.3f} ms")
    print(f"\tSolver time: \t\t\t{t_comp_solver:.3f} ms")    

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
# file_name_appendix = "_double"
# file_name_appendix = "_large"
file_name_appendix = "_large_double"
envs, params, starts, dests, local_env, local_param = extract_data(file_name_appendix)

# decide which environment to run
# ARENA infeasible cases (4): 393, 417, 484
benchmark_idx = 46 #484
# TODO: 46

# Create motion planner
motion_planner = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, local_param, local_env)
motion_planner.SetSolver("ipopt")

motion_planner.SetSuboptimalityEliminationFeature(True)
motion_planner.SetPrintLevel(5)
# motion_planner.SetMaxIter(50)

motion_planner.SetStart(starts[benchmark_idx])
dests[benchmark_idx].SetX(dests[benchmark_idx].x() + 0.0)
dests[benchmark_idx].SetY(dests[benchmark_idx].y() - 0.0)
motion_planner.SetDest(dests[benchmark_idx])
local_param.SetVmax(params[benchmark_idx].GetVmax())
local_param.SetAmax(params[benchmark_idx].GetAmax())
local_param.SetVehWidth(params[benchmark_idx].GetVehWidth())
local_param.SetVehHeight(params[benchmark_idx].GetVehHeight())
local_param.SetMargin(params[benchmark_idx].GetMargin())
local_env.CopyObstacles(envs[benchmark_idx])

# Run the planner
try:
    motion_planner.Plan()
except Exception as e:
    print(f"Exception: {e}")

tf_arena = motion_planner.GetTravelTime()
t_comp_total_arena = motion_planner.GetTotalComputationTime()
t_comp_solver_arena = motion_planner.GetSolverTime()
motion_planner.DumpToJson("python-benchmark/files/single/single_case_ARENA.json", False)

# motion_planner.PrintParametrization()
# motion_planner.ShowInitialization()
# motion_planner.SetPrintLevel(0)
# motion_planner.SetMaxIter(3000)
# print(f"v_max: {params[benchmark_idx].GetVmax()}")
# print(f"a_max: {params[benchmark_idx].GetAmax()}")

motion_planner.SetMethod(pmp.PlannerMethod.OCP)
motion_planner.Plan()
tf_ocp = motion_planner.GetTravelTime()
t_comp_total_ocp = motion_planner.GetTotalComputationTime()
t_comp_solver_ocp = motion_planner.GetSolverTime()
motion_planner.DumpToJson("python-benchmark/files/single/single_case_OCP.json", False)

motion_planner.SetMethod(pmp.PlannerMethod.P2P)
motion_planner.Plan()
tf_p2p = motion_planner.GetTravelTime()
t_comp_total_p2p = motion_planner.GetTotalComputationTime()
t_comp_solver_p2p = motion_planner.GetSolverTime()
motion_planner.DumpToJson("python-benchmark/files/single/single_case_P2P.json", False)

corridors = motion_planner.GetCorridorSequence()

print("\n=========================================================")
print(f"Information for python implementation:")
print(f"\tcorridors = {corridors}")
print(f"\tcorridor_meta_data = ['nominal']*len(corridors)")
print(f"\tp0 = [{starts[benchmark_idx].x()}, {starts[benchmark_idx].y()}]")
print(f"\tpf = [{dests[benchmark_idx].x()}, {dests[benchmark_idx].y()}]")
print(f"\tv0 = [0, 0]")
print(f"\tparams = {{'a_max': {params[benchmark_idx].GetAmax()}, 'v_max': {params[benchmark_idx].GetVmax()}, 'veh_width': {params[benchmark_idx].GetVehWidth()}, 'veh_height': {params[benchmark_idx].GetVehHeight()}, 'M': {params[benchmark_idx].GetMargin()}}}")
print("=========================================================")

print("\n=========================================================")
print(f"ARENA:")
print_stats(tf_arena, t_comp_total_arena, t_comp_solver_arena)
print(f"OCP:")
print_stats(tf_ocp, t_comp_total_ocp, t_comp_solver_ocp)
print(f"P2P:")
print_stats(tf_p2p, t_comp_total_p2p, t_comp_solver_p2p)
print("\nOverall results:")
print(f"\tSuboptimality: \t\t{100.0*(tf_arena - tf_ocp)/tf_ocp:.3f}% ({tf_arena - tf_ocp:.3f} ms)")
print(f"\tTotal speedup: \t\t{t_comp_total_ocp/t_comp_total_arena:.3f}")
print(f"\tSolver speedup: \t{t_comp_solver_ocp/t_comp_solver_arena:.3f}")
print("=========================================================")