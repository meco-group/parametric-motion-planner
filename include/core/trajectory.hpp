#ifndef __TRAJECTORY__
#define __TRAJECTORY__

#include <vector>
#include <casadi/casadi.hpp>

#include "helper_types.hpp"
#include "corridor.hpp"
#include "parameters.hpp"
// #include "parametrization.hpp"

// forward declaration
class CorridorSequence;

using namespace casadi;

const double DT_DEFAULT = 0.01;
const double MAX_TRAJECTORY_TIME_DEFAULT = 600;


class Trajectory{
    public:
        Trajectory();

        // Update the trajectory with the P2P solution
        void Update(int nb_corridors,
                    std::vector<Point2D<double>> const &waypoints,
                    std::vector<Point2D<double>> const &positions,
                    std::vector<Point2D<double>> const &velocities,
                    std::vector<Point2D<double>> const &accelerations,
                    std::vector<double> const &time_durations,
                    double solver_time);

        // Update the trajectory with the ocp solution
        void Update(DM const  &xx_ocp, DM const &uu_ocp, 
                    std::vector<double> const &tt_ocp, double solver_time,
                    CorridorSequence const &corridor_sequence,
                    Parameters const &params);

        // Update the trajectory with the arena solution
        // returns false (and aborts update) if a point is found that does
        // not lie within the corridor
        std::set<int> Update(CorridorSequence const &corridor_sequence,
                             std::vector<std::vector<double>> const &t_x,
                             std::vector<std::vector<double>> const &t_y,
                             std::vector<double> const &alpha_x,
                             std::vector<double> const &alpha_y,
                             std::vector<Point2D<double>> const &waypoints,
                             std::vector<Point2D<double>> 
                                    const &waypoint_velocities,
                             Parameters const &params,
                             double solver_time);

        void Update(Point2D<double> const &start, 
                    Point2D<double> const &start_vel,
                    std::vector<double> const &accel_x,
                    std::vector<double> const &accel_y,
                    std::vector<double> const &t_x,
                    std::vector<double> const &t_y);

        // Function to reset the trajectory.
        // To be used when no trajectory is found. The starting position is set
        // and the total time of the trajectory is set to 0
        void Reset(Point2D<double> const &start);

        // Function to append single data elements to a trajectory
        // To be used as a recording of a travelled trajectory
        void Append(double t, double px, double py, double vx, double vy, 
                    double ax, double ay);

        // printing
        friend std::ostream& operator<<(std::ostream &out, Trajectory &trajectory);

        // Define the copy assignment operator
        Trajectory& operator=(const Trajectory& other) {
            if (this == &other) return *this;  // Check for self-assignment

            // Copy the data
            curr_nb_samples_ = other.curr_nb_samples_;
            t_ = other.t_;
            px_ = other.px_;
            py_ = other.py_;
            vx_ = other.vx_;
            vy_ = other.vy_;
            ax_ = other.ax_;
            ay_ = other.ay_;
            tf_ = other.tf_;
            total_computation_time_ = other.total_computation_time_;
            solver_time_ = other.solver_time_;
            corridor_infeasibilities_detected_ = other.corridor_infeasibilities_detected_;
            emergency_braking_ = other.emergency_braking_;

            return *this;
        }

        // Function to sample the current trajectory
        void GetSample(int idx, double &time, Point2D<double> &pos, 
                       Point2D<double> &vel, Point2D<double> &acc) const;

        // Check if two vehicles will collide
        void CheckCollision(Trajectory const &other, 
                            Parameters const &params_this, 
                            Parameters const &params_other,
                            Point2D<double>& collision_point);

        // append this trajectory with the other trajectory
        void Concatenate(Trajectory const &other);

        // insert the initial waiting time (rounded upwards to a multiple of dt)
        void InsertInitialWaitingTime(double waiting_time);

        // Basic getters
        double Dt() const { return dt_;};
        double Tf() const { return tf_;};
        int NbSamples() const { return curr_nb_samples_;};
        std::vector<double> T() const { return t_;};
        std::vector<double> Px() const { return px_;};
        std::vector<double> Py() const { return py_;};
        std::vector<double> Vx() const { return vx_;};
        std::vector<double> Vy() const { return vy_;};
        std::vector<double> Ax() const { return ax_;};
        std::vector<double> Ay() const { return ay_;};
        double TotalComputationTime() const { return total_computation_time_;};
        double SolverTime() const { return solver_time_;};
        bool CorridorInfeasibilitiesDetected() const { 
            return corridor_infeasibilities_detected_;};

        // basic setters
        void SetTotalComputationTime(double total_computation_time){
            total_computation_time_ = total_computation_time;
        };
        void SetSolverTime(double solver_time){
            solver_time_ = solver_time;
        };

        json ToJson() const;


    private:
        bool CheckPointInCorridors(Point2D<double> const &point, 
                            CorridorSequence const &corridor_sequence,
                            int corridor_idx, Parameters const &params) const;

        const double dt_;
        const double max_trajectory_time_;
        const int max_nb_samples_;
        int curr_nb_samples_ = 0;

        std::vector<double> t_ = {};
        std::vector<double> px_ = {};
        std::vector<double> py_ = {};
        std::vector<double> vx_ = {};
        std::vector<double> vy_ = {};
        std::vector<double> ax_ = {};
        std::vector<double> ay_ = {};
        double tf_ = 0;

        double total_computation_time_ = 0;     // expressed in ms
        double solver_time_ = 0;                // expressed in ms
        bool corridor_infeasibilities_detected_ = false;

        bool emergency_braking_ = false;
};

#endif