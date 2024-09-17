#include <vector>

#include "helper_methods.hpp"
#include "corridor.hpp"

std::vector<Point2D<double>> Helper::GetCorridorOverlapCenters(
        CorridorSequence const &corridor_sequence, 
        Point2D<double> const &start, 
        Point2D<double> const &dest){

    // Initialize points
    std::vector<Point2D<double>> centers(1 + corridor_sequence.NbCorridors());
    centers[0].CopyValues(start);
    centers[centers.size() - 1].CopyValues(dest);

    Corridor current_corridor, next_corridor, overlap;
    for (int i = 0; i < corridor_sequence.NbCorridors() - 1; i++){
        // Get the relevant corridors and the overlap
        current_corridor = corridor_sequence.GetCorridor(i);
        next_corridor = corridor_sequence.GetCorridor(i+1);
        current_corridor.GetOverlap(next_corridor, overlap);

        // Add the new point
        centers[i+1].SetX((overlap.Xmin() + overlap.Xmax())/2);
        centers[i+1].SetY((overlap.Ymin() + overlap.Ymax())/2);
    }

    return centers;
}