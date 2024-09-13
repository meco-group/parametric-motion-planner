#include <casadi/casadi.hpp>
#include <iostream>

#include "motion_planner.hpp"
#include "environment.hpp"

int main(){
    MotionPlanner my_motion_planner = MotionPlanner();

    std::cout << "Created motion planner in environment " << *my_motion_planner.GetEnvironment() << std::endl;

    Point2D<double> start = Point2D<double>(1.0, 0.1);
    Point2D<double> dest = Point2D<double>(0.1, 0.5);

    my_motion_planner.GetEnvironment()->GetCorridorSequence(start, dest, my_motion_planner.GetVehWidth(),
                                                            my_motion_planner.GetVehHeight());

    my_motion_planner.Plan();

    casadi::MX x = casadi::MX::sym("x");
    std::cout << "x = " << x << std::endl;

}