#ifndef __BASICS__
#define __BASICS__
#include <iostream>
#include <cmath>

#include "../motion_planner.hpp"

void SolveAllMethods(MotionPlanner &motion_planner, std::string const &filename){
    motion_planner.SetJustInTimePreparationMode(false);

	motion_planner.SetMethod(ARENA);
    motion_planner.SetCorridorExtendedMode(false);
    try{ motion_planner.Plan();} catch (std::exception &e){ std::cout << e.what() << std::endl; return;}
    motion_planner.DumpToJson(filename + "_arena.json");

    // double tf_arena = motion_planner.GetLastSolution()->Tf();
    double tf_arena = motion_planner.GetTravelTime();
    double t_comp_total_arena = motion_planner.GetLastSolution()->TotalComputationTime();
    double t_comp_solver_arena = motion_planner.GetLastSolution()->SolverTime();
    double t_comp_corridors_arena = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_arena = motion_planner.GetLastSolution()->SamplingTime();

    motion_planner.SetCorridorExtendedMode(true);
    motion_planner.SetMethod(OCP);
    try{ motion_planner.Plan();} catch (std::exception &e){ std::cout << e.what() << std::endl; return;}
    motion_planner.DumpToJson(filename + "_ocp.json");

    double tf_ocp = motion_planner.GetLastSolution()->Tf();
    double t_comp_total_ocp = motion_planner.GetLastSolution()->TotalComputationTime();
    double t_comp_solver_ocp = motion_planner.GetLastSolution()->SolverTime();
    double t_comp_corridors_ocp = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_ocp = motion_planner.GetLastSolution()->SamplingTime();

    motion_planner.SetMethod(P2P);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_p2p.json");

    double tf_p2p = motion_planner.GetLastSolution()->Tf();
    double t_comp_total_p2p = motion_planner.GetLastSolution()->TotalComputationTime();
    double t_comp_solver_p2p = motion_planner.GetLastSolution()->SolverTime();
    double t_comp_corridors_p2p = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_p2p = motion_planner.GetLastSolution()->SamplingTime();

    double t_comp_total_and_move_arena = 0.001*t_comp_total_arena + tf_arena;
    double t_comp_total_and_move_ocp = 0.001*t_comp_total_ocp + tf_ocp;

    std::cout << std::endl;
	std::cout << "==============================================================" << std::endl;
    std::cout << "ARENA summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_arena);
    printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_arena);
    printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_arena);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_arena);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_arena);

    std::cout << "OCP summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_ocp);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_ocp);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_ocp);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_ocp);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_ocp);

    std::cout << "P2P summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_p2p);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_p2p);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_p2p);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_p2p);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_p2p);
    std::cout << std::endl;

	std::cout << "Overall results:" << std::endl;
	printf("\tSuboptimality: \t%.3f %% (%.3f ms)\n", 100.0*(tf_arena - tf_ocp)/tf_arena, tf_arena - tf_ocp);
	printf("\tTotal speedup: \t%.3f\n", t_comp_total_ocp/t_comp_total_arena);
	printf("\tSolver speedup:\t%.3f\n", t_comp_solver_ocp/t_comp_solver_arena);
    if (t_comp_total_and_move_arena > t_comp_total_and_move_ocp){
        printf("\tARENA is %.3fs slower to destination\n", t_comp_total_and_move_arena - t_comp_total_and_move_ocp);
    } else {
        printf("\tARENA is %.3fs faster to destination\n", t_comp_total_and_move_ocp - t_comp_total_and_move_arena);
    }
	std::cout << "==============================================================" << std::endl;
	std::cout << std::endl;
}

void SolveRandomProblem(int nb_runs){
    // Environment environment = Environment();
    Environment environment = Environment(15, 15, 0.12, 0.12);
    // Environment environment = Environment(25, 25, 0.12, 0.12);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	environment.AddRandomObstacles(0.1);

    // std::vector<int> rr = {};
    // std::vector<int> cc = {};
    // for (int i = 0; i < environment.NbCellCols(); i++){
    //     for (int j = 0; j < environment.NbCellRows(); j++){
    //         if (!environment.IsFree(Point2D<int>(i, j))){
    //             rr.push_back(i);
    //             cc.push_back(j);
    //         }
    //     }
    // }
    // std::cout << "std::vector<int> rr_test = {";
    // for (int i = 0; i < rr.size(); i++){
    //     std::cout << rr[i];
    //     if (i < rr.size() - 1){
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "};" << std::endl;
    // std::cout << "std::vector<int> cc_test = {";
    // for (int i = 0; i < cc.size(); i++){
    //     std::cout << cc[i];
    //     if (i < cc.size() - 1){
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "};" << std::endl;

    std::cout << "Created motion planner in environment " << environment << std::endl;

    for (int i = 0; i < nb_runs; i++){
    environment.AddRandomObstacles(0.1);    
	// Point2D<double> start = Point2D<double>(1.74, 0.06);
    // Point2D<double> dest = Point2D<double>(0.06, 1.38);
    // Point2D<double> start = Point2D<double>(0.46368271444046694, 0.5541697302378621);
    // Point2D<double> dest = Point2D<double>(1.4481433377924342, 0.7777571510707014);
    // my_motion_planner.SetStart(start);
    // my_motion_planner.SetDest(dest);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetRandomStart();
    my_motion_planner.SetRandomDest();
    my_motion_planner.SetStartVel(start_vel);
    my_motion_planner.SetSuboptimalityEliminationFeature(true);

    SolveAllMethods(my_motion_planner, "solution");

    // wait for user to press spacebar
    if (i < nb_runs - 1){getchar();}
    }
}

void SolveFatropFailureCase(){
    // Environment environment = Environment();
    // Environment environment = Environment(10, 12, 0.12, 0.12);
    // Environment environment = Environment(20, 20, 0.12, 0.12);
    Environment environment = Environment(25, 25, 0.12, 0.12);
    // Parameters params = Parameters(1.7743775335572929, 2.2106078993268388, 0.115, 0.115, 0.001);
    Parameters params = Parameters(0.7696381510903181, 5.5568675872914195, 0.115, 0.115, 0.001);
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	// environment.AddRandomObstacles(0.25);
    // std::vector<int> rr_test = {1, 2, 4, 6, 6, 7, 9, 9, 11, 12, 12, 13};
    // std::vector<int> cc_test = {1, 12, 10, 7, 13, 13, 11, 12, 0, 2, 6, 3};
    std::vector<int> rr_test = {1, 1, 6, 7, 9, 9, 10, 12};
    std::vector<int> cc_test = {0, 1, 5, 3, 10, 11, 9, 3};
    for (int i = 0; i < rr_test.size(); i++){
        environment.AddObstacle(Point2D<int>(rr_test[i], cc_test[i]));
    }

    std::vector<int> rr = {};
    std::vector<int> cc = {};
    for (int i = 0; i < environment.NbCellCols(); i++){
        for (int j = 0; j < environment.NbCellRows(); j++){
            if (!environment.IsFree(Point2D<int>(i, j))){
                rr.push_back(i);
                cc.push_back(j);
            }
        }
    }
    std::cout << "std::vector<int> rr_test = {";
    for (int i = 0; i < rr.size(); i++){
        std::cout << rr[i];
        if (i < rr.size() - 1){
            std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    std::cout << "std::vector<int> cc_test = {";
    for (int i = 0; i < cc.size(); i++){
        std::cout << cc[i];
        if (i < cc.size() - 1){
            std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;

    std::cout << "Created motion planner in environment " << environment << std::endl;

	// Point2D<double> start = Point2D<double>(1.74, 0.06);
    // Point2D<double> dest = Point2D<double>(0.06, 1.38);
    Point2D<double> start = Point2D<double>(0.46368271444046694, 0.5541697302378621);
    Point2D<double> dest = Point2D<double>(1.4481433377924342, 0.7777571510707014);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);
    my_motion_planner.SetSuboptimalityEliminationFeature(true);

    // my_motion_planner.SetRandomStart();
    // my_motion_planner.SetRandomDest();
    SolveAllMethods(my_motion_planner, "solution");
}

#endif