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

    // environment.AddObstacle(Point2D<int>(2, 6));
    // environment.AddObstacle(Point2D<int>(7, 1));

    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	environment.AddRandomObstacles(0.05);

    std::cout << "Created motion planner in environment " << environment << std::endl;

	Point2D<double> start = Point2D<double>(0.416194, 0.5647);
    Point2D<double> dest = Point2D<double>(0.364563, 1.08152);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);

    my_motion_planner.SetRandomStart();
    my_motion_planner.SetRandomDest();

    SolveAllMethods(my_motion_planner, "solution");
    // SolveAllMethods(my_motion_planner, "solution_" + std::to_string(0));

    // my_motion_planner.PrintCorridorSequence();
}


/*
// TODO: fix these cases!
*/