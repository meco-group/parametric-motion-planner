#include <iostream>

#include "motion_planner.hpp"
#include "environment.hpp"

int main(){
    Environment environment = Environment();
    Parameters params = Parameters();

    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    const Environment& my_environment = my_motion_planner.GetEnvironment();
    std::cout << "Created motion planner in environment " << my_environment << std::endl;

    Point2D<double> start = Point2D<double>(0.9, 0.06);
    Point2D<double> dest = Point2D<double>(0.06, 0.9);
    // Point2D<double> start = Point2D<double>(1.2, 0.06);
    // Point2D<double> dest = Point2D<double>(0.6, 1.05);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.Plan();

    my_motion_planner.SetMethod(OCP);
    my_motion_planner.Plan();

    my_motion_planner.SetMethod(ARENA);
    my_motion_planner.Plan();

    my_motion_planner.DumpToJson("solution.json");
}