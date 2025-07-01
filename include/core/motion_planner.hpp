#ifndef __MOTION_PLANNER__
#define __MOTION_PLANNER__

#include <casadi/casadi.hpp>

#include "parameters.hpp"
#include "helper_types.hpp"
#include "environment.hpp"
#include "corridor.hpp"
#include "trajectory.hpp"
#include "parametrization.hpp"
#include "ocp_solver.hpp"
#include "logger.hpp"

using namespace casadi;

enum PlannerState{
    NORMAL = 0,
    EMERGENCY = 1,  // emergency braking
    STRUGGLING = 2  // struggling to find a solution - trajectory broken down in segments
};

class MotionPlanner{
    public:
        MotionPlanner(Parameters const &params, 
                      Environment &environment)
            : MotionPlanner(ARENA, params, environment){};

        MotionPlanner(PlannerMethod method, Parameters const &params, 
                      Environment &environment);

        // This copy is needed to maintain references
        // MotionPlanner(const MotionPlanner& other) 
        //     : params_(other.params_),
        //       environment_(other.environment_),
        //       corridor_sequence_(other.environment_, params_),
        //       parametrization_(corridor_sequence_, params_),
        //       ocp_solver_(corridor_sequence_, params_){};
        MotionPlanner(const MotionPlanner& other) = delete;
            

        void SetMethod(PlannerMethod method){
            method_ = method;
            SetSolver(solver_name_);
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

        void PlanSafely(int max_allowed_ms=0);

        void GetSample(double &time, Point2D<double> &pos, 
                       Point2D<double> &vel, Point2D<double> &acc);
        int GetCurrentSampleIdx() const;
        
        // Basic getters
        const PlannerMethod GetMethod() const { return method_;};
        const Environment& GetEnvironment() const { return environment_;};
        const CorridorSequence& GetCorridorSequence() const { return corridor_sequence_;};
        const Parametrization& GetParametrization() const { return parametrization_;};
        const Parameters& GetParameters() const { return params_;};
        Point2D<double> GetStart() const { return start_;};
        Point2D<double> GetDest() const { return dest_;};
        Point2D<double> GetStart_vel() const { return start_vel_;};
        double GetVehWidth() const { return params_.GetVehWidth();};
        double GetVehHeight() const { return params_.GetVehHeight();};
        const Trajectory& GetLastSolution() const { return emergency_mode_ ? emergency_solution_ : last_solution_;};
        double GetTotalComputationTime() const { return last_solution_.TotalComputationTime();};
        double GetSolverTime() const { return last_solution_.SolverTime();};
        double GetTravelTime() const { return last_solution_.Tf();};
        double GetCorridorSequenceConstructionTime() const { 
            return corridor_sequence_.GetCorridorSequenceConstructionTime();};
        bool CorridorInfeasibilitiesDetected() const { 
            return last_solution_.CorridorInfeasibilitiesDetected();};
        bool EmergencyMode() const { return emergency_mode_;};
        bool GetJustInTimePreparationMode(){ return just_in_time_preparation_mode_;};

        // Basic setters
        void SetPrintLevel(int print_level) { print_level_ = print_level;};
        void SetMaxIter(int max_iter) { max_iter_ = max_iter;};
        void SetOCPNumberOfPointsPerCorridor(int nb_points_per_corridor){ 
            nb_points_per_corridor_ = nb_points_per_corridor;};
        void SetSuboptimalityEliminationFeature(bool set){ 
            eliminate_suboptimalities_ = set;};
        void SetSolver(std::string solver_name, 
                       bool update_prepared_opti_instances=false);
        void SetParametrizationOptimizationApproach(std::string name){
            parametrization_.SetParametrizationOptimizationApproach(name);};
        void SetJustInTimePreparationMode(bool set);
        void SetSilentMode(bool set){ silent_mode_ = set; SetSolver(solver_name_);};
        void SetCorridorExtendedMode(bool set){ corridor_sequence_.SetCorridorExtendedMode(set);};
        void SetTrajectoryT0(double t0){ last_solution_.SetT0(t0);};
        void SetMaxNbCorridorGrowingIterations(int max_nb_grow_iterations){
            max_nb_corridor_growing_iterations_ = max_nb_grow_iterations;};
        void LockCorridorSequence() { locked_corridor_sequence_ = true; };
        void UnlockCorridorSequence() { 
            locked_corridor_sequence_ = false; 
            corridor_sequence_.ResetCorridorIdxs(); };

        // Printing
        void PrintEnvironment(){
            std::cout << environment_ << std::endl;
        };
        void PrintCorridorSequence(){
            std::cout << corridor_sequence_ << std::endl;
        };
        void PrintParametrization(){
            std::cout << parametrization_ << std::endl;
        };
        void PrintInitialization(){
            parametrization_.ShowInitialization();
        }

        json ToJson() const;
        void DumpToJson(const std::string &filename, 
                        bool create_output_folder=true) const;

        void PrintLog(int print_level_=2, bool compact=false) { 
            logger_.SetPrintLevel(print_level_);
            logger_.SetCompact(compact);
            logger_.PrintLog();};
        void LogCleanlyFinishedPlanningSequence() { 
            logger_.LogEvent(CleanlyFinishedPlanningSequenceEvent());};

        void ComputeEmergencyBrakingTrajectory(double T_scaling_factor=1.0);
        bool CanAvoidCorridors(const std::vector<Corridor>& corridors, 
                               const Point2D<double>& pos,
                               const Point2D<double>& vel) const;
        void PlanConcatenatedSections(bool resursive=false);

        bool AreSequencesSeparable(MotionPlanner const &other,
                Point2D<double> const &collision_point, double& angle) const;
        void SeparateVehicleFreeSpace(MotionPlanner &other,
                Point2D<double> const &collision_point,
                Point2D<double> const &pos_this_at_collision,
                Point2D<double> const &pos_other_at_collision,
                double separation_angle);

        void InsertInitialWaitingTime(double waiting_time){
            last_solution_.InsertInitialWaitingTime(waiting_time);
        };

        void StoreResetTrajectory();
        void ResetTrajectory();
        void RevertToPreviousTrajectory();

    private:

        void LogEmergencyBrakingComputation(bool print,
            std::vector<Point2D<double>>& p1_samples, 
            std::vector<Point2D<double>>& p2_samples,
            std::vector<std::vector<double>>& safe_alpha_intervals, 
            double alpha, std::vector<Point2D<double>>& obstacle_centers,
            std::vector<double>& obstacle_widths, 
            std::vector<double>& obstacle_heights);        

        // Plan a simple trajectory, moving from corridor to corridor in 
        // straight lines
        void PlanP2P();
        void PlanP2PLine(int start_waypoint_idx);

        // Plan a trajectory by solving an Optimal Control Problem
        void PlanOCP();

        // Plan a trajectory using the ARENA method
        void PlanARENA();

        std::set<int> CheckOutOfCorridor(double solver_time);
        bool EliminateSubOptimalParametrization();

        int sign(double x){
            return (x > 0) ? 1 : -1;
        };

        std::string PlannerMethodToString() const;

        void PrintPythonImplementationInfo() const;

        bool LineSegmentsIntersect(Point2D<double> const &p1, 
                                   Point2D<double> const &p2, 
                                   Point2D<double> const &q1, 
                                   Point2D<double> const &q2) const;
        
        Environment& environment_;               
        CorridorSequence corridor_sequence_;    // contains a reference to the environment
        Parametrization parametrization_;       // contains a reference to the corridor sequence
        OCPSolver ocp_solver_;                  // contains a reference to the corridor sequence
        bool locked_corridor_sequence_ = false; // whether the corridor sequence is locked for updates
        bool currently_planning_concatenated_sections_ = false;

        Parametrization::UpdateToken parametrization_update_token_; // token to update the parametrization
        CorridorSequence::UpdateToken sequence_update_token_; // token to update the corridor sequence
        OCPSolver::UpdateToken ocp_solver_update_token_; // token to update the ocp solver

        const Parameters& params_;
        PlannerMethod method_;

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;

        Trajectory last_solution_ = Trajectory(); // current solution
        int sample_ptr_ = 0;
        Trajectory emergency_solution_ = Trajectory();
        int emergency_sample_ptr_ = 0;
        bool emergency_mode_ = false;
        Trajectory previous_solution_ = Trajectory();
        int previous_sample_ptr_ = 0;

        Trajectory reset_trajectory_ = Trajectory();
        int reset_sample_ptr_ = 0;
        
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
        int nb_points_per_corridor_ = 30;

        // ARENA method attributes
        std::set<int> add_constraints_list_;
        int max_nb_iterations_ = 4;
        bool eliminate_suboptimalities_ = true;

        // logger
        PlannerLogger logger_ = PlannerLogger();

        // other attributes
        // std::string solver_name_ = "ipopt";
        std::string solver_name_ = "fatrop";
        Dict opts_casadi_;
        Dict opts_solver_;
        int print_level_ = 0;
        int max_iter_ = 1000;//100;
        bool just_in_time_preparation_mode_ = true;
        bool silent_mode_ = false;
        int max_nb_corridor_growing_iterations_ = 4;

        std::vector<std::vector<Point2D<double>>> emergency_trajs_1_;
        std::vector<std::vector<Point2D<double>>> emergency_trajs_2_;
        std::vector<std::vector<std::vector<double>>> emergency_safe_intervals_;
        std::vector<double> emergency_alphas_;
        std::vector<std::vector<Point2D<double>>> emergency_obstacle_centers_;
        std::vector<std::vector<double>> emergency_obstacle_widths_;
        std::vector<std::vector<double>> emergency_obstacle_heights_;
};

#endif