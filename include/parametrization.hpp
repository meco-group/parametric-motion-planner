#ifndef __PARAMETRIZATION__
#define __PARAMETRIZATION__

#include <vector>

#include "helper_types.hpp"

class Parametrization{
    public:
        Parametrization() : max_nb_corridors_(20){};
        Parametrization(int max_nb_corridors) : max_nb_corridors_(max_nb_corridors){};

    private:
        const int max_nb_corridors_;
        int curr_nb_corridors_;

        std::vector<double> alpha_x_;
        std::vector<double> alpha_y_;
        std::vector<Point2D<double>> waypoints_;
        std::vector<double> waypoint_offsets_;
};

#endif