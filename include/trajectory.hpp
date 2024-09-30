#ifndef __TRAJECTORY__
#define __TRAJECTORY__

#include <vector>
#include <casadi/casadi.hpp>

#include "helper_types.hpp"
#include "corridor.hpp"
// #include "parametrization.hpp"

using namespace casadi;

const double DT_DEFAULT = 0.01;
const double MAX_TRAJECTORY_TIME_DEFAULT = 60;


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
                    std::vector<double> const &tt_ocp, double solver_time);

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

        // printing
        friend std::ostream& operator<<(std::ostream &out, Trajectory &trajectory);

        // Basic getters
        double Tf() const { return t_[curr_nb_samples_ - 1];};
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

        // basic setters
        void SetTotalComputationTime(double total_computation_time){
            total_computation_time_ = total_computation_time;
        };

        json ToJson() const;


    private:
        const double dt_;
        const double max_trajectory_time_;
        const int max_nb_samples_;
        int curr_nb_samples_ = 0;

        std::vector<double> t_;
        std::vector<double> px_;
        std::vector<double> py_;
        std::vector<double> vx_;
        std::vector<double> vy_;
        std::vector<double> ax_;
        std::vector<double> ay_;

        double total_computation_time_;     // expressed in ms
        double solver_time_;                // expressed in ms
};

#endif