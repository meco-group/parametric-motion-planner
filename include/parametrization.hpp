#ifndef __PARAMETRIZATION__
#define __PARAMETRIZATION__

#include <vector>
#include <nlohmann/json.hpp>
#include <casadi/casadi.hpp>
#include <optional>

#include "helper_methods.hpp"
#include "helper_types.hpp"
#include "corridor.hpp"
#include "trajectory.hpp"

using json = nlohmann::json;

// forward declaration
class MotionPlanner;

enum WaypointLocation{
    Bottom_Left = 0,
    Bottom_Right = 1,
    Top_Right = 2,
    Top_Left = 3,
    Start = 4,
    Dest = 5
};

class Parametrization{
    public:
        Parametrization(CorridorSequence const &corridor_sequence,
                        Parameters const &params);

        // Create access token such that only the motion planner can update
        // the parametrization
        class UpdateToken{
            public: 
                void Invalidate(){is_valid_ = false;};
                void Validate(){is_valid_ = true;};

            friend class MotionPlanner; 
            private: 
                UpdateToken() {};
                bool is_valid_ = true;
        };

        // only the motion planner can update the parametrization
        void UpdateParametrization(const UpdateToken&);
        void OptimizeParametrization(const UpdateToken&, 
                                     casadi::Dict const &opts_casadi, 
                                     casadi::Dict const &opts_solver);
        void AddOvershootingConstraints(std::set<int> &add_list);
        void OptimizeSingleArc(const UpdateToken&);

        // basic getters
        int MaxNbCorridors() const {return max_nb_corridors_;};
        int NbCorridors() const { return corridor_sequence_.NbCorridors();};
        Point2D<double> GetWaypoint(int idx) const;
        Point2D<double> GetWaypointOffset(int idx) const;
        WaypointLocation GetWaypointLocation(int idx) const;
        bool IsWaypointMovable(int idx) const;
        double GetAlphaX(int idx) const { return alpha_x_[idx];};
        double GetAlphaY(int idx) const { return alpha_y_[idx];};
        std::vector<double>& GetAlphaXSol() { return alpha_x_sol_;};
        std::vector<double>& GetAlphaYSol() { return alpha_y_sol_;};
        std::vector<Point2D<double>>& GetWaypointsSol();
        std::vector<std::vector<double>>& GetTxSol() { return t_x_sol_;};
        std::vector<std::vector<double>>& GetTySol() { return t_y_sol_;};
        std::vector<Point2D<double>>& GetWaypointVelocitiesSol();
        double GetSolverTime() const { return solver_time_;};

        // printing overload
        friend std::ostream& operator<<(std::ostream &out, 
                            Parametrization const &parametrization);

        json ToJson() const;

        Trajectory initialized_trajectory_;

    private:
        // waypoint with index waypoint_idx is in the overlapping region of 
        // corridor waypoint_idx - 1 and corridor waypoint_idx
        void ComputeSingleWaypoint(int waypoint_idx, bool second_sweep=false);

        void ComputeCandidateWaypoints();

        // Function that applies the heuristic to select the best waypoint
        // and the acceleration
        // This function only considers the point in the overlap between
        // corridor waypoint_idx - 1 and corridor waypoint_idx
        void ApplyHeuristic(int waypoint_idx, bool second_sweep=false);

        // Update the parametrization based on line of sights
        void UpdateParametrizationWithLineOfSight(int waypoint_idx);

        bool LineOfSightInCorridors(Point2D<double> const &point1, 
                                    Point2D<double> const &point2, 
                                    Corridor const &corridor1,
                                    Corridor const &corridor2) const;

        // Optimization helper functions
        void IntegrateOverCorridor(Point2D<casadi::MX> const &start, 
                                   Point2D<casadi::MX> const &start_vel, 
                                   casadi::MX const &t_x, 
                                   casadi::MX const &t_y,
                                   casadi::MX const &alpha_x,
                                   casadi::MX const &alpha_y,
                                   casadi::MX const &alpha_x_next,
                                   casadi::MX const &alpha_y_next);
        void ApplyOvershootingPreventionConstraint(casadi::Opti &opti, 
                                                   casadi::MX &t_x, 
                                                   casadi::MX &t_y);
        void InitializeParabolicSegmentConstraintFunction();
        void ConstrainParabolicSegment(casadi::Opti &opti, casadi::MX T, 
                                       casadi::MX p0, casadi::MX v0, 
                                       casadi::MX alpha, double min_val, 
                                       double max_val, double offset,
                                       MX &obj);
        void Solve();


        // Initialization functions
        void InitializeOptimization();
        bool InitializeArc(int corridor_idx, double v_des,
                           Point2D<double> const &start_vel);
        void ShowInitialization();

        void OptimizeSingleArc1D(std::vector<double> &t_sol_vector, 
                                 std::vector<double> &alpha_sol_vector,
                                 double p0, double pf, double v0);

        const CorridorSequence& corridor_sequence_; // Reference to the corridor sequence object
        const Parameters& params_;

        const int max_nb_corridors_;

        // true parametrization variables
        std::vector<double> alpha_x_;                   // acceleration in x-direction
        std::vector<double> alpha_y_;                   // acceleration in y-direction
        std::vector<Point2D<double>> waypoints_;        // waypoints
        std::vector<bool> movable_waypoints_;           // flag to indicate if a waypoint is movable
        std::vector<Point2D<double>> max_waypoint_offsets_; // maximum waypoint offsets
        std::vector<WaypointLocation> waypoint_locations_;  // naming (debugging purposes)
        int nb_movable_waypoints_;

        // mx objects to be used in the optimization
        casadi::Opti opti_;
        casadi::MX t_x_;
        casadi::MX t_y_;
        casadi::MX v_x_;
        casadi::MX v_y_;
        casadi::MX alpha_x_mx_;
        casadi::MX alpha_y_mx_;
        std::vector<Point2D<casadi::MX>> waypoints_mx_;

        // initialization containers
        std::vector<std::vector<double>> t_x_init_;
        std::vector<std::vector<double>> t_y_init_;
        std::vector<Point2D<double>> waypoint_velocities_init_;
        double alpha_0_init_;
        double alpha_f_init_;

        // optimization options
        bool RELAX_INITIAL_VELOCITY_ = true;
        
        // optimized values
        std::optional<casadi::OptiSol> sol_;
        std::vector<double> alpha_x_sol_;
        std::vector<double> alpha_y_sol_;
        std::vector<Point2D<double>> waypoints_sol_;
        std::vector<Point2D<double>> waypoint_velocities_sol_;
        std::vector<std::vector<double>> t_x_sol_;
        std::vector<std::vector<double>> t_y_sol_;
        double solver_time_;

        // scratch space
        Corridor overlap_;
        Corridor next_overlap_;
        Corridor curr_corridor_;
        Corridor next_corridor_;

        std::vector<Point2D<double>> candidate_waypoints_;
        std::vector<double> candidate_alpha_x_;
        std::vector<double> candidate_alpha_y_;
        std::vector<bool> candidate_valid_;
        Point2D<double> next_point_;
        Point2D<double> curr_point_;
        Point2D<double> prev_point_;

        std::vector<Point2D<casadi::MX>> intermediate_positions_;
        std::vector<Point2D<casadi::MX>> intermediate_velocities_;
        int nb_fine_grid_samples_ = 0;

        casadi::Function parabolic_segment_extremum_;

        std::vector<casadi::MX> p_extremes_ = {};
        

};

#endif