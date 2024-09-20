#ifndef __MOTION_PLANNER__
#define __MOTION_PLANNER__

#include <casadi/casadi.hpp>

#include "parameters.hpp"
#include "helper_types.hpp"
#include "environment.hpp"
#include "corridor.hpp"
#include "helper_methods.hpp"
#include "trajectory.hpp"
#include "parametrization.hpp"

using namespace casadi;

class MotionPlanner{
    public:
        MotionPlanner(Parameters const &params, 
                      Environment const &environment)
            : MotionPlanner(ARENA, params, environment){};

        MotionPlanner(PlannerMethod method, Parameters const &params, 
                      Environment const &environment);
            

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

        // Compute a corridor sequence in the environment using the current
        // start and destination
        void UpdateCorridorSequence();

        // Compute a corridor sequence in the environment
        void UpdateCorridorSequence(const Point2D<double> &start, 
                                    const Point2D<double> &dest);

        // Plan a trajectory using the selected method and the current
        // start, destination, and start velocity
        void Plan();

        // Plan a trajectory using the selected method with the provided
        // start, destination, and start velocity
        void Plan(const Point2D<double> &start, const Point2D<double> &dest, 
                  const Point2D<double> &start_vel);

        
        // Basic getters
        const Environment& GetEnvironment() const { return environment_;};
        Point2D<double> GetStart() const { return start_;};
        Point2D<double> GetDest() const { return dest_;};
        Point2D<double> GetStart_vel() const { return start_vel_;};
        double GetVehWidth() const { return params_.GetVehWidth();};
        double GetVehHeight() const { return params_.GetVehHeight();};

        // Basic setters
        void SetPrintLevel(int print_level) { opts_solver_["print_level"] = print_level;};

        // Printing
        void PrintEnvironment(){
            std::cout << environment_ << std::endl;
        };
        void PrintCorridorSequence(){
            std::cout << corridor_sequence_ << std::endl;
        };


    private:
        void SampleSolution();

        // Plan a simple trajectory, moving from corridor to corridor in 
        // straight lines
        void PlanP2P();
        // Sample the P2P solution
        void SampleP2PSolution();

        // Plan a trajectory by solving an Optimal Control Problem
        void PlanOCP();
        // Sample the OCP solution
        void SampleOCPSolution(DM &xx_sol, DM &uu_sol, DM &tt_sol);

        // Plan a trajectory using the ARENA method
        void PlanARENA();
        // Sample ARENA solution
        void SampleARENASolution();

        // Initialize the rk4 integrator
        void InitializeRK4();

        
        const Environment& environment_;               
        CorridorSequence corridor_sequence_;    // contains a reference to the environment
        Parametrization parametrization_;       // contains a reference to the corridor sequence

        const Parameters& params_;
        PlannerMethod method_;
        Helper helper_;

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;

        Trajectory last_solution_ = Trajectory();

        // P2P method attributes

        // OCP method attributes
        Function rk4_;
        std::vector<MX> rk4_arguments_ = std::vector<MX>(3);
        std::vector<MX> rk4_outputs_ = std::vector<MX>(1);

        // ARENA method attributes

        // other attributes
        Dict opts_casadi_;
        Dict opts_solver_;


};

#endif