#ifndef __PARAMETRIZATION__
#define __PARAMETRIZATION__

#include <vector>

#include "helper_types.hpp"
#include "corridor.hpp"

class Parametrization{
    public:
        Parametrization(CorridorSequence const &corridor_sequence);

        void UpdateParametrization(CorridorSequence const &corridor_sequence);

    private:
        void ComputeWaypoints(CorridorSequence const &corridor_sequence);

        const CorridorSequence& corridor_sequence_; // Reference to the corridor sequence object

        const int max_nb_corridors_;

        std::vector<double> alpha_x_;
        std::vector<double> alpha_y_;
        std::vector<Point2D<double>> waypoints_;
        std::vector<Point2D<double>> waypoint_offsets_;
        
        std::vector<std::vector<double>> t_x_;
        std::vector<std::vector<double>> t_y_;
};

#endif