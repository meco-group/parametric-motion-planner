#ifndef __MOTION_PLANNER__
#define __MOTION_PLANNER__

#include "parameters.hpp"
#include "helper_types.hpp"
#include "environment.hpp"

class MotionPlanner{
    public:
        MotionPlanner();

        void SetMethod(PlannerMethod method){
            method_ = method;
        };

        // Update the starting position
        void SetStart(Point2D<double> start){
            if (!environment_.isValidPosition(start)){
                throw InvalidPositionInEnvironmentException("Invalid starting position");
            }
            start_ = start;
        };

        // Update the destination
        void SetDest(Point2D<double> dest){
            if (!environment_.isValidPosition(dest)){
                throw InvalidPositionInEnvironmentException("Invalid destination");
            }
            dest_ = dest;
        };

        // Update the starting velocity
        void SetStartVel(Point2D<double> start_vel){
            start_vel_ = start_vel;
        };

        // Plan a trajectory using the selected method and the current
        // start, destination, and start velocity
        void Plan();

        // Plan a trajectory using the selected method with the provided
        // start, destination, and start velocity
        void Plan(Point2D<double> start, Point2D<double> dest, 
                  Point2D<double> start_vel);

        
        // Basic getters
        Environment* environment(){ return &environment_;};
        Point2D<double> start(){ return start_;};
        Point2D<double> dest(){ return dest_;};
        Point2D<double> start_vel(){ return start_vel_;};

    private:
        Environment environment_;

        // Plan a simple trajectory, moving from corridor to corridor in 
        // straight lines
        void PlanP2P();

        // Plan a trajectory by solving an Optimal Control Problem
        void PlanOCP();

        // Plan a trajectory using the ARENA method
        void PlanARENA();

        Parameters* params_;
        PlannerMethod method_;

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;



};

#endif