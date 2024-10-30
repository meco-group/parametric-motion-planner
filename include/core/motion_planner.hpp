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
        void SetStart(Point2D<double> start);
        void SetRandomStart();

        // Update the destination
        void SetDest(Point2D<double> dest);
        void SetRandomDest();

        // Update the starting velocity
        void SetStartVel(Point2D<double> start_vel){ start_vel_ = start_vel;};

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
        const CorridorSequence& GetCorridorSequence() const { return corridor_sequence_;};
        const Parametrization& GetParametrization() const { return parametrization_;};
        Point2D<double> GetStart() const { return start_;};
        Point2D<double> GetDest() const { return dest_;};
        Point2D<double> GetStart_vel() const { return start_vel_;};
        double GetVehWidth() const { return params_.GetVehWidth();};
        double GetVehHeight() const { return params_.GetVehHeight();};
        const Trajectory& GetLastSolution() const { return last_solution_;};
        double GetTotalComputationTime() const { return last_solution_.TotalComputationTime();};
        double GetSolverTime() const { return last_solution_.SolverTime();};
        double GetTravelTime() const { return last_solution_.Tf();};

        // Basic setters
        void SetPrintLevel(int print_level) { opts_solver_["print_level"] = print_level;};
        void SetOCPNumberOfPointsPerCorridor(int nb_points_per_corridor){ 
            nb_points_per_corridor_ = nb_points_per_corridor;};

        // Printing
        void PrintEnvironment(){
            std::cout << environment_ << std::endl;
        };
        void PrintCorridorSequence(){
            std::cout << corridor_sequence_ << std::endl;
        };

        json ToJson() const;
        void DumpToJson(const std::string &filename) const;


    private:
        // Plan a simple trajectory, moving from corridor to corridor in 
        // straight lines
        void PlanP2P();
        void PlanP2PLine(int start_waypoint_idx);

        // Plan a trajectory by solving an Optimal Control Problem
        void PlanOCP();

        // Plan a trajectory using the ARENA method
        void PlanARENA();

        void ComputeEmergencyBrakingTrajectory();

        // Initialize the rk4 integrator
        void InitializeRK4();

        std::set<int> CheckOutOfCorridor(double solver_time);
        bool EliminateSubOptimalParametrization();

        int sign(double x){
            return (x > 0) ? 1 : -1;
        };

        std::string PlannerMethodToString() const;
        
        const Environment& environment_;               
        CorridorSequence corridor_sequence_;    // contains a reference to the environment
        Parametrization parametrization_;       // contains a reference to the corridor sequence
        Parametrization::UpdateToken parametrization_update_token_; // token to update the parametrization
        CorridorSequence::UpdateToken sequence_update_token_; // token to update the corridor sequence

        const Parameters& params_;
        PlannerMethod method_;
        Helper helper_;

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;

        Trajectory last_solution_ = Trajectory();
        

        // P2P method attributes
        std::vector<Point2D<double>> p2p_waypoints_;
        std::vector<double> coarse_samples_time_;
        std::vector<Point2D<double>> coarse_samples_position_;
        std::vector<Point2D<double>> coarse_samples_velocity_;
        std::vector<Point2D<double>> coarse_samples_acceleration_;
        Point2D<double> curr_pos_;
        Point2D<double> next_pos_;
        Point2D<double> curr_vel_;
        Point2D<double> curr_acc_;

        // OCP method attributes
        Function rk4_;
        std::vector<MX> rk4_arguments_ = std::vector<MX>(3);
        std::vector<MX> rk4_outputs_ = std::vector<MX>(1);
        int nb_points_per_corridor_ = 5;

        // ARENA method attributes
        std::set<int> add_constraints_list_;
        int max_nb_iterations_ = 4;

        // other attributes
        Dict opts_casadi_;
        Dict opts_solver_;


};

#endif