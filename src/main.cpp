#include <iostream>

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
    std::cout << "ARENA summary: " << std::endl;
    std::cout << "\tTf: " << tf_arena << std::endl;
    std::cout << "\tTotal computation time: " << t_comp_total_arena << std::endl;
    std::cout << "\tSolver time: " << t_comp_solver_arena << std::endl;

    std::cout << "OCP summary: " << std::endl;
    std::cout << "\tTf: " << tf_ocp << std::endl;
    std::cout << "\tTotal computation time: " << t_comp_total_ocp << std::endl;
    std::cout << "\tSolver time: " << t_comp_solver_ocp << std::endl;

    std::cout << "P2P summary: " << std::endl;
    std::cout << "\tTf: " << tf_p2p << std::endl;
    std::cout << "\tTotal computation time: " << t_comp_total_p2p << std::endl;
    std::cout << "\tSolver time: " << t_comp_solver_p2p << std::endl;
    std::cout << std::endl;
}

int main(){
    Environment environment = Environment();
    Parameters params = Parameters();

    // Add some obstacles
    environment.AddObstacle(Point2D<int>(6, 9));
    environment.AddObstacle(Point2D<int>(3, 3));
    environment.AddObstacle(Point2D<int>(0, 6));
    environment.AddObstacle(Point2D<int>(1, 6));
    environment.AddObstacle(Point2D<int>(2, 6));

    environment.AddObstacle(Point2D<int>(5, 0));
    environment.AddObstacle(Point2D<int>(7, 2));
    environment.AddObstacle(Point2D<int>(7, 3));
    environment.AddObstacle(Point2D<int>(9, 3));


    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    const Environment& my_environment = my_motion_planner.GetEnvironment();
    std::cout << "Created motion planner in environment " << my_environment << std::endl;

    // Point2D<double> start = Point2D<double>(0.25, 0.15);
    // Point2D<double> dest = Point2D<double>(0.5, 1.2-0.12);    
    // Point2D<double> start_vel = Point2D<double>(0, 0);

    // Point2D<double> start = Point2D<double>(0.549371, 1.14159);  // CASE TO CHECK ! (ocp infeasible)
    // Point2D<double> dest = Point2D<double>(1.22551, 0.204132);   // CASE TO CHECK ! (ocp infeasible)
    // Point2D<double> start_vel = Point2D<double>(0, 0);           // CASE TO CHECK ! (ocp infeasible)

    Point2D<double> start = Point2D<double>(0.116088, 0.420836);
    Point2D<double> dest = Point2D<double>(0.205415, 1.02913);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);

    my_motion_planner.SetRandomStart();
    my_motion_planner.SetRandomDest();

    SolveAllMethods(my_motion_planner, "solution");
    // SolveAllMethods(my_motion_planner, "solution_" + std::to_string(0));

    my_motion_planner.PrintCorridorSequence();
}


/*
===============================================================================
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.216965, 0.507278) to (0.18696, 1.0004) with start velocity (0, 0)
Planning using ARENA method
Point (0.18696, 0.665978) is out of corridor [0, 0.36] x [0, 0.72]
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/nlpsol.cpp:501: Assertion "lb <= ub && lb!=inf && ub!=-inf" failed:
Ill-posed problem detected: LBG[24] <= UBG[24] was violated. Got LBG[24] = 0.123 and UBG[24] = 0.
terminate called after throwing an instance of 'casadi::CasadiException'
  what():  Error in Opti::value [OptiNode] at .../casadi/core/optistack.cpp:175:
.../casadi/core/optistack_internal.cpp:870: Assertion "solved()" failed:
This action is forbidden since you have not solved the Opti stack yet (with calling 'solve').
Aborted

===============================================================================
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.0940935, 0.0812651) to (1.33096, 0.186266) with start velocity (0, 0)
Planning using ARENA method
Point (0.56076, 0.186266) is out of corridor [0, 0.6] x [0, 0.36]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   5.46ms (  1.31us)   5.40ms (  1.29us)      4171
       nlp_g  |  35.71ms (  8.56us)  35.53ms (  8.52us)      4171
  nlp_grad_f  |   1.17ms (  2.47us)   1.16ms (  2.44us)       474
  nlp_hess_l  |  65.80ms ( 22.03us)  65.74ms ( 22.01us)      2987
   nlp_jac_g  | 103.07ms ( 34.24us) 103.09ms ( 34.25us)      3010
       total  | 942.72ms (942.72ms) 942.75ms (942.75ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Planning computation time: 968.982 ms
Planning from (0.0940935, 0.0812651) to (1.33096, 0.186266) with start velocity (0, 0)

===============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.353025, 0.193586) to (0.991037, 0.111455) with start velocity (0, 0)
Planning using ARENA method
Point (0.372225, 0.174386) is out of corridor [0, 0.84] x [0.12, 0.36]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   4.14ms (  1.27us)   4.09ms (  1.25us)      3269
       nlp_g  |  21.94ms (  6.71us)  21.82ms (  6.67us)      3269
  nlp_grad_f  | 676.00us (  2.28us) 661.14us (  2.23us)       297
  nlp_hess_l  |  45.31ms ( 15.17us)  45.28ms ( 15.16us)      2986
   nlp_jac_g  |  67.96ms ( 22.57us)  67.95ms ( 22.57us)      3011
       total  | 835.06ms (835.06ms) 835.08ms (835.08ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Planning computation time: 848.52 ms

===============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.570622, 1.12697) to (1.34423, 0.165214) with start velocity (0, 0)
Planning using ARENA method
Point (0.667822, 1.02977) is out of corridor [0, 0.72] x [0.96, 1.2]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  11.49ms (  1.56us)  11.35ms (  1.54us)      7375
       nlp_g  | 106.73ms ( 14.47us) 106.42ms ( 14.43us)      7375
  nlp_grad_f  |   3.42ms (  3.18us)   3.42ms (  3.18us)      1075
  nlp_hess_l  | 154.97ms ( 51.99us) 154.97ms ( 51.99us)      2981
   nlp_jac_g  | 256.45ms ( 85.06us) 256.53ms ( 85.08us)      3015
       total  |   2.16 s (  2.16 s)   2.16 s (  2.16 s)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Planning computation time: 2212.17 ms

===============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.265206, 0.0684898) to (0.916453, 0.153241) with start velocity (0, 0)
Planning using ARENA method
Point (0.553506, 0.153241) is out of corridor [0, 0.6] x [0, 0.36]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   5.18ms (  1.36us)   5.10ms (  1.34us)      3802
       nlp_g  |  33.53ms (  8.82us)  33.38ms (  8.78us)      3803
  nlp_grad_f  |   1.14ms (  2.61us)   1.11ms (  2.55us)       435
  nlp_hess_l  |  66.76ms ( 22.40us)  66.73ms ( 22.39us)      2981
   nlp_jac_g  | 104.95ms ( 34.76us) 104.96ms ( 34.77us)      3019
       total  |   1.10 s (  1.10 s)   1.10 s (  1.10 s)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Planning computation time: 1124.77 ms

===============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . # . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# # # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . # . # . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 
. . . . . # . . . . . . 

Planning from (0.367645, 1.04098) to (1.22277, 0.132742) with start velocity (0, 0)
Planning using ARENA method
Point (0.426445, 0.98218) is out of corridor [0, 0.48] x [0.84, 1.2]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   7.66ms (  1.64us)   7.55ms (  1.61us)      4678
       nlp_g  |  70.78ms ( 15.13us)  70.60ms ( 15.09us)      4678
  nlp_grad_f  |   1.29ms (  3.41us)   1.29ms (  3.40us)       379
  nlp_hess_l  | 168.58ms ( 56.49us) 168.58ms ( 56.49us)      2984
   nlp_jac_g  | 245.43ms ( 81.46us) 245.61ms ( 81.52us)      3013
       total  |   1.74 s (  1.74 s)   1.74 s (  1.74 s)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Planning computation time: 1802.55 ms
*/