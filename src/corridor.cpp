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
    ClearAll();
    Point2D<int> curr_start_cell = path[0];
    Point2D<int> curr_end_cell = path[1];
    Point2D<int> curr_direction = 
                        Point2D<int>(curr_end_cell.x() - curr_start_cell.x(), 
                                     curr_end_cell.y() - curr_start_cell.y());
    Point2D<int> next_direction;
    for (int i = 2; i < path.size(); i++){
        Point2D<int> next_cell = path[i];
        next_direction.SetX(next_cell.x() - curr_end_cell.x());
        next_direction.SetY(next_cell.y() - curr_end_cell.y());
        if (!(next_direction == curr_direction)){
            AddCorridorFromCells(curr_start_cell, curr_end_cell, cell_width, cell_height);
            curr_start_cell.CopyValues(curr_end_cell);
            curr_direction.CopyValues(next_direction);
        }

        curr_end_cell.CopyValues(next_cell);
    }

    AddCorridorFromCells(curr_start_cell, curr_end_cell, cell_width, cell_height);
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

void CorridorSequence::AddCorridorFromCells(Point2D<int> &start_cell, 
                                           Point2D<int> &end_cell,
                                           const double &cell_width, 
                                           const double &cell_height){
    Point2D<double> start = start_cell.ConvertCellToWorld(cell_width, cell_height);
    Point2D<double> end = end_cell.ConvertCellToWorld(cell_width, cell_height);

    AddCorridor(std::min(start.x(), end.x()) - cell_width/2,
                std::max(start.x(), end.x()) + cell_width/2,
                std::min(start.y(), end.y()) - cell_height/2,
                std::max(start.y(), end.y()) + cell_height/2);
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