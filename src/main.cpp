#include <iostream>

#include "motion_planner.hpp"
#include "environment.hpp"

int main(){
    Environment environment = Environment();
    Parameters params = Parameters();

    // Add some obstacles
    // environment.AddObstacle(Point2D<int>(6, 9));
    environment.AddObstacle(Point2D<int>(3, 3));

    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    const Environment& my_environment = my_motion_planner.GetEnvironment();
    std::cout << "Created motion planner in environment " << my_environment << std::endl;

    Point2D<double> start = Point2D<double>(1.25, 0.3);
    Point2D<double> dest = Point2D<double>(0.9, 1.14);
    // Point2D<double> start = Point2D<double>(0.25, 0.15); // infeasible case
    // Point2D<double> dest = Point2D<double>(0.12, 1.2-0.12);
    
    Point2D<double> start_vel = Point2D<double>(0, 0);
    // Point2D<double> start_vel = Point2D<double>(1.0, 0.4);


    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);
    my_motion_planner.Plan();

    my_motion_planner.SetMethod(OCP);
    my_motion_planner.Plan();
    my_motion_planner.DumpToJson("solution_ocp.json");

    my_motion_planner.SetMethod(ARENA);
    my_motion_planner.Plan();
    my_motion_planner.DumpToJson("solution_arena.json");
}