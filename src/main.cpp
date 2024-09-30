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
    // environment.AddObstacle(Point2D<int>(6, 9));
    // environment.AddObstacle(Point2D<int>(3, 3));
    // environment.AddObstacle(Point2D<int>(0, 6));
    // environment.AddObstacle(Point2D<int>(1, 6));
    // environment.AddObstacle(Point2D<int>(2, 6));

    // environment.AddObstacle(Point2D<int>(5, 0));
    // environment.AddObstacle(Point2D<int>(7, 2));
    // environment.AddObstacle(Point2D<int>(7, 3));
    // environment.AddObstacle(Point2D<int>(9, 3));

    environment.AddObstacle(Point2D<int>(2, 6));
    environment.AddObstacle(Point2D<int>(7, 1));

    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    const Environment& my_environment = my_motion_planner.GetEnvironment();
    std::cout << "Created motion planner in environment " << my_environment << std::endl;

    // Point2D<double> start = Point2D<double>(1.3, 0.3);
    // Point2D<double> dest = Point2D<double>(0.8, 0.96+0.12);
    // Point2D<double> start_vel = Point2D<double>(0, 0);

    Point2D<double> start = Point2D<double>(0.812933, 1.09531);
    Point2D<double> dest = Point2D<double>(1.02775, 0.384806);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);

    // my_motion_planner.SetRandomStart();
    // my_motion_planner.SetRandomDest();

    SolveAllMethods(my_motion_planner, "solution");
    // SolveAllMethods(my_motion_planner, "solution_" + std::to_string(0));

    my_motion_planner.PrintCorridorSequence();
}


/*
// TODO: fix these cases!
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

Planning from (0.507537, 0.161001) to (1.29938, 0.336505) with start velocity (0, 0)
Planning using ARENA method
0: [0, 0.6] x [0, 0.36]
1: [0.36, 1.08] x [0.12, 0.24]
2: [0.96, 1.32] x [0, 0.36]
3: [1.2, 1.44] x [0.24, 0.48]

Point (0.543837, 0.197301) is out of corridor [0, 0.6] x [0, 0.36]
checking line of sight between (0.507537, 0.161001) and (1.0185, 0.1815)
checking line of sight between (0.5415, 0.1785) and (1.2585, 0.3015)
checking line of sight between (1.0185, 0.1815) and (1.29938, 0.336505)
Parametrization (4/20)
Waypoint 0
        Position:       (0.507537, 0.161001)     (4)
        Acceleration:   (1, 1)
Waypoint 1
        Position:       (0.5415, 0.1785)         (1)
        Acceleration:   (1, -1)
Waypoint 2
        Position:       (1.0185, 0.1815)         (3)
        Acceleration:   (-1, 1)
Waypoint 3
        Position:       (1.2585, 0.3015)         (3)
        Acceleration:   (-1, 1)
Waypoint 4
        Position:       (1.29938, 0.336505)      (5)
        Acceleration:   (-1, -1)

t_x_init_ (acc): 
0.0333333 0.186483 0.186483 
0.186483 2.57148 2.57148 
2.57148 3.77148 3.77148 
3.77148 3.9592 3.99253 

t_y_init_ (acc): 
0.0333333 0.186483 0.186483 
0.203508 2.57148 2.57148 
2.58812 3.77148 3.77148 
3.78361 3.9592 3.99253 

alpha_0_init_: 0.515236
alpha_f_init_: -0.867136
waypoint_velocities_init_: 
(0, 0)
(0.2, 0.103047)
(0.2, 0.000893242)
(0.2, 0.100692)
(0, 0)

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   4.79ms (  1.57us)   4.76ms (  1.56us)      3053
       nlp_g  |  33.10ms ( 10.84us)  32.94ms ( 10.79us)      3053
  nlp_grad_f  | 309.00us (  3.22us) 306.35us (  3.19us)        96
  nlp_hess_l  |  91.49ms ( 30.58us)  91.45ms ( 30.56us)      2992
   nlp_jac_g  | 142.93ms ( 47.53us) 142.88ms ( 47.52us)      3007
       total  |   1.30 s (  1.30 s)   1.30 s (  1.30 s)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Maximum_Iterations_Exceeded'
Point (0.446705, 0.0561663) is out of corridor [0, 0.6] x [0, 0.36]
Planning computation time: 1345.82 ms
Planning from (0.507537, 0.161001) to (1.29938, 0.336505) with start velocity (0, 0)
Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  90.00us (  1.06us)  88.99us (  1.05us)        85
       nlp_g  |  25.95ms (305.26us)  25.95ms (305.30us)        85
  nlp_grad_f  | 230.00us (  2.74us) 234.38us (  2.79us)        84
  nlp_hess_l  | 104.47ms (  1.27ms) 104.48ms (  1.27ms)        82
   nlp_jac_g  | 128.47ms (  1.53ms) 128.50ms (  1.53ms)        84
       total  | 350.15ms (350.15ms) 350.15ms (350.15ms)         1
Planning computation time: 514.273 ms
Planning from (0.507537, 0.161001) to (1.29938, 0.336505) with start velocity (0, 0)
Planning using P2P method
Planning computation time: 0.015198 ms

ARENA summary: 
        Tf: 33.04
        Total computation time: 1345.82
        Solver time: -1
OCP summary: 
        Tf: 0.72
        Total computation time: 514.273
        Solver time: 350.149
P2P summary: 
        Tf: 1.29
        Total computation time: 0.015198
        Solver time: 0

0: [0, 0.6] x [0, 0.36]
1: [0.36, 1.08] x [0.12, 0.24]
2: [0.96, 1.32] x [0, 0.36]
3: [1.2, 1.44] x [0.24, 0.48]

==============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . . . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
. . # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . . . . . . . . . . 
. . . . . . . . . . . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 

Planning from (0.294042, 0.925528) to (1.06051, 0.395898) with start velocity (0, 0)
Planning using ARENA method
0: [0, 0.48] x [0.84, 1.2]
1: [0.36, 0.48] x [0.36, 0.96]
2: [0.36, 1.2] x [0.24, 0.48]

Point (0.324042, 0.895528) is out of corridor [0, 0.48] x [0.84, 1.2]
Parametrization (3/20)
Waypoint 0
        Position:       (0.294042, 0.925528)     (4)
        Acceleration:   (1, -1)
Waypoint 1
        Position:       (0.4185, 0.8985)         (0)
        Acceleration:   (-1, -1)
Waypoint 2
        Position:       (0.4215, 0.4215)         (2)
        Acceleration:   (1, 1)
Waypoint 3
        Position:       (1.06051, 0.395898)      (5)
        Acceleration:   (-1, 1)

t_x_init_ (acc): 
0.0333333 0.638956 0.638956 
0.672313 3.03417 3.03417 
3.06753 6.22924 6.26257 

t_y_init_ (acc): 
0.0333333 0.638956 0.638956 
0.66505 3.03417 3.03417 
3.06634 6.22924 6.26257 

alpha_0_init_: -0.217164
alpha_f_init_: 0.0350245
waypoint_velocities_init_: 
(0, 0)
(0.2, -0.0434329)
(-0.000141131, -0.2)
(0, 0)
Point (0.4218, 0.894543) is out of corridor [0.36, 0.48] x [0.36, 0.96]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  45.00us (  1.50us)  46.30us (  1.54us)        30
       nlp_g  | 322.00us ( 10.73us) 325.14us ( 10.84us)        30
  nlp_grad_f  | 147.00us (  4.74us) 147.82us (  4.77us)        31
  nlp_hess_l  |   1.25ms ( 43.17us)   1.25ms ( 43.24us)        29
   nlp_jac_g  |   1.36ms ( 43.81us)   1.35ms ( 43.44us)        31
       total  |   8.41ms (  8.41ms)   8.42ms (  8.42ms)         1
Point (0.487498, 0.293255) is out of corridor [0.36, 1.2] x [0.24, 0.48]
Planning computation time: 34.512 ms
Planning from (0.294042, 0.925528) to (1.06051, 0.395898) with start velocity (0, 0)
Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  35.00us (  1.03us)  33.73us (991.97ns)        34
       nlp_g  |   7.68ms (225.79us)   7.66ms (225.39us)        34
  nlp_grad_f  |  71.00us (  2.45us)  71.87us (  2.48us)        29
  nlp_hess_l  |  25.75ms (953.63us)  25.76ms (953.96us)        27
   nlp_jac_g  |  33.41ms (  1.15ms)  33.42ms (  1.15ms)        29
       total  |  87.05ms ( 87.05ms)  87.05ms ( 87.05ms)         1
Planning computation time: 218.495 ms
Planning from (0.294042, 0.925528) to (1.06051, 0.395898) with start velocity (0, 0)
Planning using P2P method
Planning computation time: 0.016334 ms

ARENA summary: 
        Tf: 1.19
        Total computation time: 34.512
        Solver time: 8.42074
OCP summary: 
        Tf: 1.2
        Total computation time: 218.495
        Solver time: 87.0498
P2P summary: 
        Tf: 1.5
        Total computation time: 0.016334
        Solver time: 0

0: [0, 0.48] x [0.84, 1.2]
1: [0.36, 0.48] x [0.36, 0.96]
2: [0.36, 1.2] x [0.24, 0.48]

==============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . . . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
. . # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . . . . . . . . . . 
. . . . . . . . . . . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 

Planning from (0.812933, 1.09531) to (1.02775, 0.384806) with start velocity (0, 0)
Planning using ARENA method
0: [0, 0.96] x [0.96, 1.2]
1: [0.36, 0.48] x [0.36, 1.2]
2: [0.36, 1.2] x [0.24, 0.48]

Point (0.899633, 1.00861) is out of corridor [0, 0.96] x [0.96, 1.2]
Parametrization (3/20)
Waypoint 0
        Position:       (0.812933, 1.09531)      (4)
        Acceleration:   (-1, -1)
Waypoint 1
        Position:       (0.4215, 1.0185)         (1)
        Acceleration:   (1, -1)
Waypoint 2
        Position:       (0.4215, 0.4215)         (2)
        Acceleration:   (1, 1)
Waypoint 3
        Position:       (1.02775, 0.384806)      (5)
        Acceleration:   (-1, 1)

t_x_init_ (acc): 
0.0333333 1.97383 1.97383 
2.00735 4.9696 4.9696 
5.00274 8.00064 8.03397 

t_y_init_ (acc): 
0.0333333 1.97383 1.97383 
2.00062 4.9696 4.9696 
5.00109 8.00064 8.03397 

alpha_0_init_: -0.196221
alpha_f_init_: 0.0553188
waypoint_velocities_init_: 
(0, 0)
(-0.2, -0.0392442)
(0.00112524, -0.2)
(0, 0)
Point (0.4182, 1.01465) is out of corridor [0.36, 0.48] x [0.36, 1.2]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 110.00us (  3.24us) 108.38us (  3.19us)        34
       nlp_g  |   1.07ms ( 31.50us)   1.06ms ( 31.20us)        34
  nlp_grad_f  | 226.00us (  6.65us) 221.04us (  6.50us)        34
  nlp_hess_l  |   4.15ms (129.72us)   4.15ms (129.66us)        32
   nlp_jac_g  |   3.23ms ( 95.06us)   3.23ms ( 95.10us)        34
       total  |  19.46ms ( 19.46ms)  19.46ms ( 19.46ms)         1
Point (0.726023, 1.14567) is out of corridor [0, 0.96] x [0.96, 1.2]
Point (0.56585, 0.298031) is out of corridor [0.36, 1.2] x [0.24, 0.48]
Planning computation time: 96.086 ms
Planning from (0.812933, 1.09531) to (1.02775, 0.384806) with start velocity (0, 0)
Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  48.00us (  1.17us)  43.88us (  1.07us)        41
       nlp_g  |   9.95ms (242.56us)   9.95ms (242.62us)        41
  nlp_grad_f  |  92.00us (  2.71us)  94.09us (  2.77us)        34
  nlp_hess_l  |  31.08ms (971.31us)  31.09ms (971.46us)        32
   nlp_jac_g  |  40.72ms (  1.20ms)  40.74ms (  1.20ms)        34
       total  | 109.09ms (109.09ms) 109.09ms (109.09ms)         1
Planning computation time: 236.179 ms
Planning from (0.812933, 1.09531) to (1.02775, 0.384806) with start velocity (0, 0)
Planning using P2P method
Planning computation time: 0.017647 ms

ARENA summary: 
        Tf: 1.42
        Total computation time: 96.086
        Solver time: 19.4568
OCP summary: 
        Tf: 1.43
        Total computation time: 236.179
        Solver time: 109.091
P2P summary: 
        Tf: 1.81
        Total computation time: 0.017647
        Solver time: 0

0: [0, 0.96] x [0.96, 1.2]
1: [0.36, 0.48] x [0.36, 1.2]
2: [0.36, 1.2] x [0.24, 0.48]

--> infeasible solution in first corridor!

==============================================================================
Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
. . . . . . . . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
. . # . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . . . . . . . . . . 
. . . . . . . . . . . . 
. . . . . . . # . . . . 
. . . . . . . . . . . . 

Planning from (0.866052, 1.03481) to (0.192454, 0.328792) with start velocity (0, 0)
Planning using ARENA method
0: [0, 0.96] x [0.96, 1.2]
1: [0, 0.24] x [0.24, 1.08]
2: [0, 0.48] x [0, 0.6]

Point (0.846852, 1.01561) is out of corridor [0, 0.96] x [0.96, 1.2]
Parametrization (3/20)
Waypoint 0
        Position:       (0.866052, 1.03481)      (4)
        Acceleration:   (-1, -1)
Waypoint 1
        Position:       (0.1815, 1.0185)         (1)
        Acceleration:   (1, -1)
Waypoint 2
        Position:       (0.1815, 0.2985)         (1)
        Acceleration:   (1, -1)
Waypoint 3
        Position:       (0.192454, 0.328792)     (5)
        Acceleration:   (-1, -1)

t_x_init_ (acc): 
0.0233333 4.90132 4.90132 
4.92471 10.0553 10.0553 
10.0664 10.12 10.19 

t_y_init_ (acc): 
0.0233333 4.90132 4.90132 
4.9241 10.0553 10.0553 
10.102 10.12 10.19 

alpha_0_init_: -0.0238316
alpha_f_init_: -0.158843
waypoint_velocities_init_: 
(0, 0)
(-0.14, -0.00333643)
(0.00031835, -0.14)
(-1.38778e-17, -0.84)

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 499.00us (  1.44us) 489.54us (  1.41us)       346
       nlp_g  |   5.32ms ( 15.39us)   5.31ms ( 15.33us)       346
  nlp_grad_f  | 141.00us (  3.53us) 137.58us (  3.44us)        40
  nlp_hess_l  |  23.03ms ( 84.05us)  23.04ms ( 84.10us)       274
   nlp_jac_g  |  16.02ms ( 57.20us)  16.01ms ( 57.18us)       280
       total  | 125.54ms (125.54ms) 125.55ms (125.55ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
terminate called after throwing an instance of 'std::runtime_error'
  what():  Trajectory  is too long to be updated
Aborted
*/