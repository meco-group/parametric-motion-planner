#include <iostream>

#include "motion_planner.hpp"
#include "environment.hpp"

void SolveAllMethods(MotionPlanner &motion_planner, std::string const &filename){
    motion_planner.SetMethod(ARENA);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_arena.json");

    motion_planner.SetMethod(OCP);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_ocp.json");

    motion_planner.SetMethod(P2P);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_p2p.json");
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

    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    const Environment& my_environment = my_motion_planner.GetEnvironment();
    std::cout << "Created motion planner in environment " << my_environment << std::endl;

    // Point2D<double> start = Point2D<double>(0.25, 0.15);
    // Point2D<double> dest = Point2D<double>(0.5, 1.2-0.12);    
    // Point2D<double> start_vel = Point2D<double>(0, 0);

    Point2D<double> start = Point2D<double>(0.549371, 1.14159);  // CASE TO CHECK ! (ocp infeasible)
    Point2D<double> dest = Point2D<double>(1.22551, 0.204132);   // CASE TO CHECK ! (ocp infeasible)
    Point2D<double> start_vel = Point2D<double>(0, 0);           // CASE TO CHECK ! (ocp infeasible)

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);

    // my_motion_planner.SetRandomStart();
    // my_motion_planner.SetRandomDest();

    SolveAllMethods(my_motion_planner, "solution");
    // SolveAllMethods(my_motion_planner, "solution_" + std::to_string(0));

    my_motion_planner.PrintCorridorSequence();
}