#include <iostream>
#include <cmath>

#include "motion_planner.hpp"
#include "environment.hpp"

void SolveAllMethods(MotionPlanner &motion_planner, std::string const &filename){
	motion_planner.SetMethod(ARENA);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_arena.json");

    double tf_arena = motion_planner.GetLastSolution().Tf();
    double t_comp_total_arena = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_arena = motion_planner.GetLastSolution().SolverTime();

    motion_planner.SetMethod(OCP);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_ocp.json");

    double tf_ocp = motion_planner.GetLastSolution().Tf();
    double t_comp_total_ocp = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_ocp = motion_planner.GetLastSolution().SolverTime();

    motion_planner.SetMethod(P2P);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_p2p.json");

    double tf_p2p = motion_planner.GetLastSolution().Tf();
    double t_comp_total_p2p = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_p2p = motion_planner.GetLastSolution().SolverTime();

    std::cout << std::endl;
	std::cout << "==============================================================" << std::endl;
    std::cout << "ARENA summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_arena);
    printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_arena);
    printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_arena);

    std::cout << "OCP summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_ocp);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_ocp);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_ocp);

    std::cout << "P2P summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_p2p);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_p2p);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_p2p);
    std::cout << std::endl;

	std::cout << "Overall results:" << std::endl;
	printf("\tSuboptimality: \t%.3f %% (%.3f ms)\n", 100.0*(tf_arena - tf_ocp)/tf_arena, tf_arena - tf_ocp);
	printf("\tTotal speedup: \t%.3f\n", t_comp_total_ocp/t_comp_total_arena);
	printf("\tSolver speedup:\t%.3f\n", t_comp_solver_ocp/t_comp_solver_arena);
	std::cout << "==============================================================" << std::endl;
	std::cout << std::endl;
}

int main(){
    Environment environment = Environment();
    Parameters params = Parameters();

    // Add some obstacles
    // environment.AddObstacle(Point2D<int>(6, 9));
    // environment.AddObstacle(Point2D<int>(3, 3));
    // environment.AddObstacle(Point2D<int>(0, 6));
    // environment.AddObstacle(Point2D<int>(1, 6));
    // environment.AddObstacle(Point2D<int>(2, 6));

    // environment.AddObstacle(Point2D<int>(5, 0));
    // environment.AddObstacle(Point2D<int>(7, 2));
    // environment.AddObstacle(Point2D<int>(7, 3));
    // environment.AddObstacle(Point2D<int>(9, 3));

    
	// environment.AddObstacle(Point2D<int>(2, 6));
    // environment.AddObstacle(Point2D<int>(7, 1));


	// environment.AddObstacle(Point2D<int>(0, 4));
	// environment.AddObstacle(Point2D<int>(1, 4));
	// environment.AddObstacle(Point2D<int>(2, 4));
	// environment.AddObstacle(Point2D<int>(3, 4));


    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	environment.AddRandomObstacles(0.05);

    std::cout << "Created motion planner in environment " << environment << std::endl;

	Point2D<double> start = Point2D<double>(1.3, 0.24);
    Point2D<double> dest = Point2D<double>(0.75, 1.08);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);

    my_motion_planner.SetRandomStart();
    my_motion_planner.SetRandomDest();

    SolveAllMethods(my_motion_planner, "solution");

	// my_motion_planner.Plan();
	// my_motion_planner.Plan();

}


/*
// TODO: fix these cases!
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
# . . . . . . . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . . . . . . 
. # . . . # . . . . . # 
. # . . . . . . . . . . 
. # . . . . . . . . . . 

Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using ARENA method
Point (0.516624, 0.305713) is out of corridor [0.24, 0.6] x [0, 0.36]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  55.00us (  1.00us)  53.86us (979.31ns)        55
       nlp_g  | 353.00us (  6.42us) 353.26us (  6.42us)        55
  nlp_grad_f  |  91.00us (  2.22us)  87.54us (  2.14us)        41
  nlp_hess_l  | 466.00us ( 11.95us) 466.32us ( 11.96us)        39
   nlp_jac_g  | 807.00us ( 19.68us) 809.10us ( 19.73us)        41
       total  |   6.03ms (  6.03ms)   6.03ms (  6.03ms)         1
Planning computation time: 21.5785 ms
Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  47.00us (810.34ns)  46.73us (805.62ns)        58
       nlp_g  |  10.67ms (183.93us)  10.67ms (183.92us)        58
  nlp_grad_f  |  55.00us (  2.04us)  53.64us (  1.99us)        27
  nlp_hess_l  |  28.00ms (777.86us)  28.02ms (778.22us)        36
   nlp_jac_g  |  45.05ms (938.50us)  45.06ms (938.77us)        48
       total  | 116.01ms (116.01ms) 116.01ms (116.01ms)         1
An error occurred: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
Planning computation time: 216.651 ms
Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using P2P method
Planning computation time: 0.027963 ms

==============================================================
ARENA summary: 
        Tf:                             0.627 s
        Total computation time:         21.578 ms
        Solver time:                    6.033 ms
OCP summary: 
        Tf:                             1.679 s
        Total computation time:         216.651 ms
        Solver time:                    -1.000 ms				----> Why does this fail?
P2P summary: 
        Tf:                             1.126 s
        Total computation time:         0.028 ms
        Solver time:                    0.000 ms

Overall results:
        Suboptimality:  -167.745 % (-1.052 ms)
        Total speedup:  10.040
        Solver speedup: -0.166
==============================================================
*/