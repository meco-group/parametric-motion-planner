#ifndef __TRAJECTORY__
#define __TRAJECTORY__

#include <vector>
#include <casadi/casadi.hpp>

#include "parametrization.hpp"

using namespace casadi;

const double DT_DEFAULT = 0.01;
const double MAX_TRAJECTORY_TIME_DEFAULT = 60;


class Trajectory{
    public:
        Trajectory();

        void Update(DM const  &xx_ocp, DM const &uu_ocp, 
                    std::vector<double> const &tt_ocp);

        void Update(int nb_corridors,
                    std::vector<std::vector<double>> const &t_x,
                    std::vector<std::vector<double>> const &t_y,
                    std::vector<double> const &alpha_x,
                    std::vector<double> const &alpha_y,
                    std::vector<Point2D<double>> const &waypoints,
                    std::vector<Point2D<double>> const &waypoint_velocities,
                    double a_max);

        // printing
        friend std::ostream& operator<<(std::ostream &out, Trajectory &trajectory);

        // Basic getters
        double Tf() const { return t_[t_.size() - 1];};
        int NbSamples() const { return curr_nb_samples_;};
        std::vector<double> T() const { return t_;};
        std::vector<double> Px() const { return px_;};
        std::vector<double> Py() const { return py_;};
        std::vector<double> Vx() const { return vx_;};
        std::vector<double> Vy() const { return vy_;};
        std::vector<double> Ax() const { return ax_;};
        std::vector<double> Ay() const { return ay_;};



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
};

#endif