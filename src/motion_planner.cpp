#include <iostream>

#include "motion_planner.hpp"

MotionPlanner::MotionPlanner(){
    environment_ = Environment();
    params_ = new Parameters();
    method_ = ARENA;
}

void MotionPlanner::Plan(){
    // Determine which function to invoke based on the selected method of the 
    // planner

    std::cout << "Planning from " << start_ << " to " << dest_ << " with start velocity " << start_vel_ << std::endl;

    switch(method_){
        case P2P:
            return PlanP2P();
        case OCP:
            return PlanOCP();
        case ARENA:
            return PlanARENA();
        default:
            std::cout << "Invalid method selected" << std::endl;
    }
}

void MotionPlanner::Plan(Point2D<double> start, Point2D<double> dest, 
                         Point2D<double> start_vel){
    SetStart(start);
    SetDest(dest);
    SetStartVel(start_vel);
    Plan();
}

void MotionPlanner::PlanP2P(){
    std::cout << "Planning using P2P method" << std::endl;
}

void MotionPlanner::PlanOCP(){
    std::cout << "Planning using OCP method" << std::endl;
}

void MotionPlanner::PlanARENA(){
    std::cout << "Planning using ARENA method" << std::endl;
}