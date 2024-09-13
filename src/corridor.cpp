#include <stdexcept>

#include "corridor.hpp"


// Corridor class

bool Corridor::GetOverlap(Corridor &other, Corridor &overlap){
    // Check if there is overlap
    if (other.Xmax() <= x_min_ || other.Xmin() >= x_max_ || 
        other.Ymax() <= y_min_ || other.Ymin() >= y_max_){
        return false;
    }

    // If there is overlap, return the overlapping region
    overlap.SetXmin(std::max(x_min_, other.Xmin()));
    overlap.SetXmax(std::min(x_max_, other.Xmax()));
    overlap.SetYmin(std::max(y_min_, other.Ymin()));
    overlap.SetYmax(std::min(y_max_, other.Ymax()));

    return true;
}


// CorridorSequence class

void CorridorSequence::InitializeFromCellPath(std::vector<Point2D<int>> &path, 
                                              const double &cell_width, 
                                              const double &cell_height){
    return;
};

void CorridorSequence::AddCorridor(double x_min, double x_max, double y_min, 
                                   double y_max){

    // Check if there is still space to add a corridor
    if (last_corridor_idx_ >= max_len_){
        throw FullCorridorSequenceException();
    }
    
    // If so, add the new Corridor
    sequence_[last_corridor_idx_] = Corridor(x_min, x_max, y_min, y_max);
    last_corridor_idx_++;
};

void CorridorSequence::RemoveCorridor(int idx){
    // Check if the index is valid
    if (idx < 0 || idx >= last_corridor_idx_){
        throw std::out_of_range("Invalid index of corridor to remove");
    }

    // If so, remove the corridor
    sequence_.erase(sequence_.begin() + idx);
    last_corridor_idx_--;
};