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

bool Corridor::IsCompletelyWithin(Corridor* const &other) const {
    return x_min_ >= other->Xmin() && x_max_ <= other->Xmax() && 
           y_min_ >= other->Ymin() && y_max_ <= other->Ymax();
}

bool Corridor::IsCompletelyWithin(Corridor* const &other1,
                                  Corridor* const &other2) const {
    Corridor possible_parent;
    
    // Check if this corridor fits inside an enlarged version of other1
    // ('enlarged' meaning that the overlap region is maximized)
    // First check if other1 can be enlarged vertically
    if (other1->Xmin() >= other2->Xmin() && other1->Xmax() <= other2->Xmax()){
        if (other1->Ymin() <= other2->Ymax()){
            possible_parent.SetYmin(std::min(other1->Ymin(), other2->Ymin()));
        } else {
            possible_parent.SetYmin(other1->Ymin());
        }
        if (other1->Ymax() >= other2->Ymin()){
            possible_parent.SetYmax(std::max(other1->Ymax(), other2->Ymax()));
        } else {
            possible_parent.SetYmax(other1->Ymax());
        }
        possible_parent.SetXmin(other1->Xmin());
        possible_parent.SetXmax(other1->Xmax());
        if (IsCompletelyWithin(&possible_parent)){
            return true;
        }
    }

    // then check if other1 can be enlarged horizontally
    if (other1->Ymin() >= other2->Ymin() && other1->Ymax() <= other2->Ymax()){
        if (other1->Xmin() <= other2->Xmax()){
            possible_parent.SetXmin(std::min(other1->Xmin(), other2->Xmin()));
        } else {
            possible_parent.SetXmin(other1->Xmin());
        }
        if (other1->Xmax() >= other2->Xmin()){
            possible_parent.SetXmax(std::max(other1->Xmax(), other2->Xmax()));
        } else {
            possible_parent.SetXmax(other1->Xmax());
        }
        possible_parent.SetYmin(other1->Ymin());
        possible_parent.SetYmax(other1->Ymax());
        if (IsCompletelyWithin(&possible_parent)){
            return true;
        }
    }

    // Check if this corridor fits inside an enlarged version of other2
    // ('enlarged' meaning that the overlap region is maximized)
    // First check if other2 can be enlarged vertically
    if (other2->Xmin() >= other1->Xmin() && other2->Xmax() <= other1->Xmax()){
        if (other2->Ymin() <= other1->Ymax()){
            possible_parent.SetYmin(std::min(other2->Ymin(), other1->Ymin()));
        } else {
            possible_parent.SetYmin(other2->Ymin());
        }
        if (other2->Ymax() >= other1->Ymin()){
            possible_parent.SetYmax(std::max(other2->Ymax(), other1->Ymax()));
        } else {
            possible_parent.SetYmax(other2->Ymax());
        }
        possible_parent.SetXmin(other2->Xmin());
        possible_parent.SetXmax(other2->Xmax());
        if (IsCompletelyWithin(&possible_parent)){
            return true;
        }
    }

    // then check if other2 can be enlarged horizontally
    if (other2->Ymin() >= other1->Ymin() && other2->Ymax() <= other1->Ymax()){
        if (other2->Xmin() <= other1->Xmax()){
            possible_parent.SetXmin(std::min(other2->Xmin(), other1->Xmin()));
        } else {
            possible_parent.SetXmin(other2->Xmin());
        }
        if (other2->Xmax() >= other1->Xmin()){
            possible_parent.SetXmax(std::max(other2->Xmax(), other1->Xmax()));
        } else {
            possible_parent.SetXmax(other2->Xmax());
        }
        possible_parent.SetYmin(other2->Ymin());
        possible_parent.SetYmax(other2->Ymax());
        if (IsCompletelyWithin(&possible_parent)){
            return true;
        }
    }

    return false;
}

void Corridor::UpdateDirection(){
    if (std::abs(x_max_ - x_min_) > std::abs(y_max_ - y_min_)){
        if (x_max_ - x_min_ > 0){
            direction_ = Point2D<int>(1, 0);
        } else {
            direction_ = Point2D<int>(-1, 0);
        }
    } else {
        if (y_max_ - y_min_ > 0){
            direction_ = Point2D<int>(0, 1);
        } else {
            direction_ = Point2D<int>(0, -1);
        }
    }
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

void CorridorSequence::InflateCorridors(Environment &environment){
    bool made_change = true;
    int grow_counter = 0;
    int max_nb_grow_iterations = 3;

    // grow corridors
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = false;
        
        for (int i = 0; i < last_corridor_idx_; i++){
            made_change = made_change || GrowCorridorSideways(i, environment);
        }

        if (made_change){
            made_change = made_change || MergeCorridors();
        }
        grow_counter++;
    }

    // grow first corridor even more
    max_nb_grow_iterations = 3; grow_counter = 0;
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = false;
        made_change = made_change || GrowCorridorSideways(0, environment);
        grow_counter++;
    }

    // remove irrelevant corridors
    RemoveIrrelevantCorridors();
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

bool CorridorSequence::GrowCorridorSideways(int idx, Environment &environment){
    // A corridor cannot become fat (wider than it's length) unless it is the 
    // first corridor
    if (idx > 0 && sequence_[idx].Width() >= sequence_[idx].Height()){
        return false;
    }

    // Check if we can grow
    bool growing_left_possible = CheckCellsOnLeftSide(idx, environment);
    bool growing_right_possible = CheckCellsOnrightSide(idx, environment);

    // if not able to grow, stop
    if (!growing_left_possible && !growing_right_possible){
        return false;
    }

    // else, grow the corridor
    // if vertical corridor
    if (sequence_[idx].Direction().x() == 0){
        // upward corridor
        if (sequence_[idx].Direction().y() > 0){
            if (growing_left_possible){
                sequence_[idx].SetXmin(sequence_[idx].Xmin() - 
                                       environment.CellWidth());
            }
            if (growing_right_possible){
                sequence_[idx].SetXmax(sequence_[idx].Xmax() + 
                                       environment.CellWidth());
            }
        // downward corridor
        } else {
            if (growing_left_possible){
                sequence_[idx].SetXmax(sequence_[idx].Xmax() + 
                                       environment.CellWidth());
            }
            if (growing_right_possible){
                sequence_[idx].SetXmin(sequence_[idx].Xmin() - 
                                       environment.CellWidth());
            }
        }

    // if horizontal corridor
    } else {
        // rightward corridor
        if (sequence_[idx].Direction().x() > 0){
            if (growing_left_possible){
                sequence_[idx].SetYmax(sequence_[idx].Ymax() + 
                                       environment.CellHeight());
            }
            if (growing_right_possible){
                sequence_[idx].SetYmin(sequence_[idx].Ymin() - 
                                       environment.CellHeight());
            }
        // leftward corridor
        } else {
            if (growing_left_possible){
                sequence_[idx].SetYmin(sequence_[idx].Ymin() - 
                                       environment.CellHeight());
            }
            if (growing_right_possible){
                sequence_[idx].SetYmax(sequence_[idx].Ymax() + 
                                       environment.CellHeight());
            }
        }
    }

    return true;
};

int CorridorSequence::GetCellsOnLeftSide(int corridor_idx, 
                                         Environment &environment){

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment.CellWidth(), 
                                environment.CellHeight());

    // if vertical corridor
    if (direction.x() == 0){

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetY(corridor->Ymin() + 
                                            environment.CellHeight()/2 +
                                            i*environment.CellHeight());
            if (direction.y() > 0){
                cells_along_corridor_[i].SetX(corridor->Xmin() - 
                                            environment.CellWidth()/2);
            } else {
                cells_along_corridor_[i].SetX(corridor->Xmax() + 
                                            environment.CellWidth()/2);
            }
        }
    // if horizontal corridor
    } else {

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetX(corridor->Xmin() + 
                                            environment.CellWidth()/2 + 
                                            i*environment.CellWidth());
            if (direction.x() > 0){
                cells_along_corridor_[i].SetY(corridor->Ymax() + 
                                            environment.CellHeight()/2);
            } else {
                cells_along_corridor_[i].SetY(corridor->Ymin() - 
                                            environment.CellHeight()/2);
            }
        }
    }

    return corridor_cell_length;
};

int CorridorSequence::GetCellsOnRightSide(int corridor_idx, 
                                          Environment &environment){

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment.CellWidth(), 
                                environment.CellHeight());

    // if vertical corridor
    if (direction.x() == 0){

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetY(corridor->Ymin() + 
                                            environment.CellHeight()/2 +
                                            i*environment.CellHeight());
            if (direction.y() > 0){
                cells_along_corridor_[i].SetX(corridor->Xmax() + 
                                            environment.CellWidth()/2);
            } else {
                cells_along_corridor_[i].SetX(corridor->Xmin() - 
                                            environment.CellWidth()/2);
            }
        }
    // if horizontal corridor
    } else {

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetX(corridor->Xmin() + 
                                            environment.CellWidth()/2 + 
                                            i*environment.CellWidth());
            if (direction.x() > 0){
                cells_along_corridor_[i].SetY(corridor->Ymin() - 
                                            environment.CellHeight()/2);
            } else {
                cells_along_corridor_[i].SetY(corridor->Ymax() + 
                                            environment.CellHeight()/2);
            }
        }
    }

    return corridor_cell_length;
};

bool CorridorSequence::CheckCellsOnLeftSide(int corridor_idx, 
                                            Environment &environment){
    int corridor_cell_length = GetCellsOnLeftSide(corridor_idx, environment);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment.IsFree(cells_along_corridor_[i])){
            return false;
        }
    }

    return true;
};

bool CorridorSequence::CheckCellsOnrightSide(int corridor_idx, 
                                            Environment &environment){
    int corridor_cell_length = GetCellsOnRightSide(corridor_idx, environment);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment.IsFree(cells_along_corridor_[i])){
            return false;
        }
    }

    return true;
};

bool CorridorSequence::RemoveIrrelevantCorridors(){
    bool made_change = false;

    Corridor* previous_corridor;
    Corridor* current_corridor;
    Corridor* next_corridor;
    Corridor overlap;
    for (int i = last_corridor_idx_ - 2; i >= 1; i--){
        previous_corridor = &sequence_[i-1];
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        // The current corridor has to be removed if
        // - it is completely within the previous corridor
        // - it is completely within the next corridor
        // - it is completely within a union of the previous and next corridor
        // - it overlaps both with the previous and the next corridor
        if (current_corridor->IsCompletelyWithin(previous_corridor) ||
                current_corridor->IsCompletelyWithin(next_corridor) ||
                current_corridor->IsCompletelyWithin(previous_corridor, 
                                                    next_corridor) ||
                previous_corridor->GetOverlap(*next_corridor, overlap)){
            RemoveCorridor(i);
            made_change = true;
            continue; // move on to the next corridor
        }
    }

    // The first/last corridor can be removed if the next/previous corridor
    // contains the start/destination
    // TODO: implement this


    return made_change;
};

bool CorridorSequence::MergeCorridors(){
    double tolerance = 1e-10;

    bool made_change = false;
    
    Corridor* current_corridor;
    Corridor* next_corridor;
    for (int i = last_corridor_idx_ - 2; i >= 0; i--){
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        if (std::abs(current_corridor->Xmin() - next_corridor->Xmin()) &&
                std::abs(current_corridor->Xmax() - next_corridor->Xmax())){
            current_corridor->SetYmin(std::min(current_corridor->Ymin(), 
                                               next_corridor->Ymin()));
            current_corridor->SetYmax(std::max(current_corridor->Ymax(),
                                               next_corridor->Ymax()));
            RemoveCorridor(i+1);
            made_change = true;
            continue;
        }

        if (std::abs(current_corridor->Ymin() - next_corridor->Ymin()) &&
                std::abs(current_corridor->Ymax() - next_corridor->Ymax())){
            current_corridor->SetXmin(std::min(current_corridor->Xmin(), 
                                               next_corridor->Xmin()));
            current_corridor->SetXmax(std::max(current_corridor->Xmax(),
                                               next_corridor->Xmax()));
            RemoveCorridor(i+1);
            made_change = true;
            continue;
        }
    }
    return made_change;
};