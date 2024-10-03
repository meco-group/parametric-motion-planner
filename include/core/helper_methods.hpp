#ifndef __HELPER_METHODS__
#define __HELPER_METHODS__

#include <vector>

#include "helper_types.hpp"
#include "corridor.hpp"

// This class stores basic functions to hide them from the implementation
// in other classes
class Helper {
    public:
        Helper(){};

        // Function to get a list of points in the overlap of corridors
        // This list also includes start and end point and is to be used to
        // initialize the trajectory in the OCP case
        std::vector<Point2D<double>> GetCorridorOverlapCenters(
            CorridorSequence const &corridor_sequence, 
            Point2D<double> const &start,
            Point2D<double> const &dest);
};

#endif