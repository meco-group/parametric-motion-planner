#include <casadi/casadi.hpp>
#include <iostream>

#include "motion_planner.hpp"
#include "environment.hpp"

int main(){
    MotionPlanner my_motion_planner = MotionPlanner();

    std::cout << "Created motion planner in environment " << *my_motion_planner.GetEnvironment() << std::endl;

    Point2D<double> start = Point2D<double>(0.9, 0.06);
    Point2D<double> dest = Point2D<double>(0.06, 0.9);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.UpdateCorridorSequence();
    my_motion_planner.Plan();

    my_motion_planner.PrintCorridorSequence();

    my_motion_planner.SetMethod(OCP);
    // my_motion_planner.Plan(start, Point2D<double>(0.5, 0.1), Point2D<double>(0.0, 0.0));
    my_motion_planner.Plan();

    casadi::MX x = casadi::MX::sym("x");
    std::cout << "x = " << x << std::endl;

}