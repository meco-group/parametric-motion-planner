#include <stdexcept>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fstream>

#include "core/corridor.hpp"

using json = nlohmann::json;

// Corridor class

bool Corridor::GetOverlap(Corridor &other, Corridor &overlap) const {
    double tolerance = 1e-6;
    // Check if there is overlap
    if (other.Xmax() <= x_min_ + tolerance || 
        other.Xmin() >= x_max_  - tolerance || 
        other.Ymax() <= y_min_  + tolerance || 
        other.Ymin() >= y_max_ - tolerance){
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

bool Corridor::ContainsPoint(Point2D<double> const &point) const {
    double tolerance = 1.0e-4;
    return point.x() >= x_min_ - tolerance && 
           point.x() <= x_max_ + tolerance &&
           point.y() >= y_min_ - tolerance && 
           point.y() <= y_max_ + tolerance;
}

bool Corridor::ContainsVehicle(const Point2D<double> &vehicle_position, 
                               const Parameters &params) const {
    std::vector<double> violations(4);
    violations[0] = vehicle_position.x() - params.GetVehWidth()/2.0 - params.GetMargin() - x_min_;
    violations[1] = x_max_ - vehicle_position.x() - params.GetVehWidth()/2.0 - params.GetMargin();
    violations[2] = vehicle_position.y() - params.GetVehHeight()/2.0 - params.GetMargin() - y_min_;
    violations[3] = y_max_ - vehicle_position.y() - params.GetVehHeight()/2.0 - params.GetMargin();

    double tolerance = 1.0e-4;
    for (double violation : violations){
        if (violation < -tolerance){
            // std::cout << "corridor bound violation detected: " << violation << std::endl;
            return false;
        }
    }
    return true;

    // return vehicle_position.x() - params.GetVehWidth()/2.0 - params.GetMargin() >= x_min_ - tolerance &&
    //        vehicle_position.x() + params.GetVehWidth()/2.0 + params.GetMargin() <= x_max_  + tolerance &&
    //        vehicle_position.y() - params.GetVehHeight()/2.0 - params.GetMargin() >= y_min_ - tolerance &&
    //        vehicle_position.y() + params.GetVehHeight()/2.0 + params.GetMargin() <= y_max_ + tolerance;
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

void CorridorSequence::UpdateSequence(Point2D<double> const &start,
                                      Point2D<double> const &dest,
                                      Point2D<double> const &start_vel,
                                      Parameters const &params,
                                      UpdateToken const &token){
    sequence_available_ = false;
    // Input checks
    if (!environment_.isValidPosition(start) || 
        !environment_.isValidPosition(dest)){
        throw InvalidPositionInEnvironmentException("Invalid starting position or destination");
    }

    // Check if we really need to do update
    if (use_smart_update_ &&
            start_ == start && dest_ == dest && start_vel_ == start_vel &&
            environment_.GetVersion() == latest_envrionment_version_){
        // No need to update the sequence
        std::cout << "NOTE: skipped update of corridor sequence." << std::endl;
        sequence_available_ = true;
        return;
    }

    start_.CopyValues(start);
    dest_.CopyValues(dest);
    start_vel_.CopyValues(start_vel);

    // Reset
    ClearAll();

    // Convert start and destination to cell points
    Point2D<int> start_cell = start.ConvertWorldToCell(
                                                environment_.CellWidth(), 
                                                environment_.CellHeight());
    Point2D<int> dest_cell = dest.ConvertWorldToCell(
                                                environment_.CellWidth(),
                                                environment_.CellHeight());

    // Compute a path in the cell environment from start to dest
    std::vector<Point2D<int>> path = environment_.PerformBreadthFirstSearch(start_cell, dest_cell);
    if (path.size() == 0){
        sequence_available_ = false;
        return;
    }

    // std::vector<Point2D<int>> obstacles = {};
    // Point2D<int> obstacle;
    // for (int i = 0; i < environment_.NbCellCols(); i++){
    //     for (int j = 0; j < environment_.NbCellRows(); j++){
    //         obstacle.SetX(i); obstacle.SetY(j);
    //         // std::cout << "checking obstacle: " << obstacle << std::endl;
    //         if (!environment_.IsFree(obstacle)){
    //             // std::cout << "obstacle!" << std::endl;
    //             obstacles.push_back(obstacle);
    //         }
    //     }
    // }

    // std::cout << "start = " << start << std::endl;
    // std::cout << "dest = " << dest << std::endl;

    // std::cout << "number_of_rows = " << environment_.NbCellRows() << std::endl;
    // std::cout << "number_of_columns = " << environment_.NbCellCols() << std::endl;

    // std::cout << "obstacles = [";
    // if (obstacles.size() > 0){
    //     for (int i = 0; i < obstacles.size()-1; i++){
    //         std::cout << obstacles[i] << ", ";
    //     }
    //     std::cout << obstacles[obstacles.size()-1];
    // }
    // std::cout << "]" << std::endl;

    // std::cout << "cell_width = " << environment_.CellWidth() << std::endl;
    // std::cout << "cell_height = " << environment_.CellHeight() << std::endl;

    // std::cout << "path = [";
    // for (int i = 0; i < path.size()-1; i++){
    //     std::cout << path[i] << ", ";
    // }
    // std::cout << path[path.size()-1] << "]" << std::endl;

    // Add cells to ensure initial footprint of the vehicle is included
    AddInitialFootprint(path);

    // Add cells to ensure final footprint of the vehicle is included
    AddFinalFootprint(path);

    // std::cout << "path_prime = [";
    // for (int i = 0; i < path.size()-1; i++){
    //     std::cout << path[i] << ", ";
    // }
    // std::cout << path[path.size()-1] << "]" << std::endl;

    // Loop over path and add corridors
    Point2D<int> curr_start_cell = path[0];
    Point2D<int> curr_end_cell = path[1];
    Point2D<int> curr_direction = 
                        Point2D<int>(curr_end_cell.x() - curr_start_cell.x(), 
                                     curr_end_cell.y() - curr_start_cell.y());
    // std::cout << "corridors_before = [";
    // for (int i = 0; i < nb_of_corridors_-1; i++){
    //     std::cout << sequence_[i] << ", ";
    // }
    // std::cout << sequence_[nb_of_corridors_-1] << "]" << std::endl;

    
    Point2D<int> next_direction;
    for (int i = 2; i < path.size(); i++){
        Point2D<int> next_cell = path[i];
        next_direction.SetX(next_cell.x() - curr_end_cell.x());
        next_direction.SetY(next_cell.y() - curr_end_cell.y());
        if (!(next_direction == curr_direction)){
            AddCorridorFromCells(curr_start_cell, curr_end_cell);
            curr_start_cell.CopyValues(curr_end_cell);
            curr_direction.CopyValues(next_direction);
        }
        curr_end_cell.CopyValues(next_cell);
    }

    AddCorridorFromCells(curr_start_cell, curr_end_cell);

    // std::cout << "corridors_narrow = [";
    // for (int i = 0; i < nb_of_corridors_-1; i++){
    //     std::cout << sequence_[i] << ", ";
    // }
    // std::cout << sequence_[nb_of_corridors_-1] << "]" << std::endl;

    // Inflate the corridors
    InflateCorridors();

    // std::cout << "corridors_final = [";
    // for (int i = 0; i < nb_of_corridors_-1; i++){
    //     std::cout << sequence_[i] << ", ";
    // }
    // std::cout << sequence_[nb_of_corridors_-1] << "]" << std::endl;

    sequence_available_ = true;
    latest_envrionment_version_ = environment_.GetVersion();
    UpdateVersion();
};

bool CorridorSequence::ContainsPoint(const Point2D<double> &point) const {
    if (!sequence_available_){
        throw std::runtime_error("Corridor sequence not available");
    }
    for (int i = 0; i < nb_of_corridors_; i++){
        if (sequence_[i].ContainsPoint(point)){
            return true;
        }
    }
    return false;
};

Corridor CorridorSequence::GetCorridor(int idx) const { 
    if (idx < 0 || idx >= nb_of_corridors_){
        throw std::out_of_range("Invalid index of corridor to get");
    }
    return sequence_[idx].Copy();
};

void CorridorSequence::GetCorridor(int idx, Corridor &corridor) const {
    if (idx < 0 || idx >= nb_of_corridors_){
        throw std::out_of_range("Invalid index of corridor to get");
    }
    corridor.CopyValues(sequence_[idx]);
};

void CorridorSequence::GetStart(Point2D<double> &point) const {
    point.CopyValues(start_);
};

void CorridorSequence::GetStartVel(Point2D<double> &point) const {
    point.CopyValues(start_vel_);
};

void CorridorSequence::GetDest(Point2D<double> &point) const {
    point.CopyValues(dest_);
};

json CorridorSequence::ToJson() const {
    // Create a new JSON entry for the Environment class
    json corridor_sequence_json;
    corridor_sequence_json["start"] = start_.ToJson();
    corridor_sequence_json["dest"] = dest_.ToJson();
    corridor_sequence_json["start_vel"] = start_vel_.ToJson();
    corridor_sequence_json["nb_of_corridors"] = nb_of_corridors_;
    std::vector<json> sequence_json = std::vector<json>(nb_of_corridors_);
    for (int i = 0; i < nb_of_corridors_; i++){
        sequence_json[i] = sequence_[i].ToJson();
    }
    corridor_sequence_json["sequence"] = sequence_json;

    return corridor_sequence_json;
}

void CorridorSequence::AddInitialFootprint(std::vector<Point2D<int>> &path) const {
    std::unordered_set<Point2D<int>, Point2DHash<int>> occupied_cells_set = 
        environment_.GetOccupiedFootprintCells(start_, params_.GetVehWidth(), 
                                               params_.GetVehHeight());

    std::vector<Point2D<int>> occupied_cells(occupied_cells_set.begin(), 
                                             occupied_cells_set.end());    
    
    // Filter out cells that are in the path
    for (int i =  occupied_cells.size()-1; i >= 0; i--){
        for (int j = 0; j < path.size(); j++){
            if (occupied_cells[i] == path[j]){
                occupied_cells.erase(occupied_cells.begin() + i);
                break;
            }
        }
    }

    // Make sure to add the cells in the correct sequence (this makes the 
    // resulting corridors nicer)
    if (occupied_cells.size() == 1){
        path.insert(path.begin(), occupied_cells.begin(), occupied_cells.end());
    } else if (occupied_cells.size() == 2){
        if (path[0].ManhattanDistance(occupied_cells[0]) < 
                path[0].ManhattanDistance(occupied_cells[1])){
            std::reverse(occupied_cells.begin(), occupied_cells.end());
        }
        path.insert(path.begin(), occupied_cells.begin(), occupied_cells.end());
    } else if (occupied_cells.size() == 3){
        // find the diagonal cell
        int diagonal_idx;
        if (path[0].ManhattanDistance(occupied_cells[0]) == 2){
            path.insert(path.begin(), occupied_cells[1]);
            path.insert(path.begin(), occupied_cells[0]);
            path.insert(path.begin(), occupied_cells[2]);
        } else if (path[0].ManhattanDistance(occupied_cells[1]) == 2){
            path.insert(path.begin(), occupied_cells[0]);
            path.insert(path.begin(), occupied_cells[1]);
            path.insert(path.begin(), occupied_cells[2]);
        } else {
            path.insert(path.begin(), occupied_cells[0]);
            path.insert(path.begin(), occupied_cells[2]);
            path.insert(path.begin(), occupied_cells[1]);
        }
    }
}

void CorridorSequence::AddFinalFootprint(std::vector<Point2D<int>> &path) const {
    std::unordered_set<Point2D<int>, Point2DHash<int>> occupied_cells_set = 
        environment_.GetOccupiedFootprintCells(dest_, params_.GetVehWidth(), 
                                               params_.GetVehHeight());

    std::vector<Point2D<int>> occupied_cells(occupied_cells_set.begin(),
                                             occupied_cells_set.end());

    // Filter out cells that are in the path
    for (int i =  occupied_cells.size()-1; i >= 0; i--){
        for (int j = 0; j < path.size(); j++){
            if (occupied_cells[i] == path[j]){
                occupied_cells.erase(occupied_cells.begin() + i);
                break;
            }
        }
    }

    // Make sure to add the cells in the correct sequence (this makes the 
    // resulting corridors nicer)
    if (occupied_cells.size() == 1){
        path.insert(path.end(), occupied_cells.begin(), occupied_cells.end());
    } else if (occupied_cells.size() == 2){
        if (path[path.size()-1].ManhattanDistance(occupied_cells[0]) > 
                path[path.size()-1].ManhattanDistance(occupied_cells[1])){
            std::reverse(occupied_cells.begin(), occupied_cells.end());
        }
        path.insert(path.end(), occupied_cells.begin(), occupied_cells.end());
    } else if (occupied_cells.size() == 3){
        // find the diagonal cell
        int diagonal_idx;
        if (path[path.size()-1].ManhattanDistance(occupied_cells[0]) == 2){
            path.insert(path.end(), occupied_cells[1]);
            path.insert(path.end(), occupied_cells[0]);
            path.insert(path.end(), occupied_cells[2]);
        } else if (path[path.size()-1].ManhattanDistance(occupied_cells[1]) == 2){
            path.insert(path.end(), occupied_cells[0]);
            path.insert(path.end(), occupied_cells[1]);
            path.insert(path.end(), occupied_cells[2]);
        } else {
            path.insert(path.end(), occupied_cells[0]);
            path.insert(path.end(), occupied_cells[2]);
            path.insert(path.end(), occupied_cells[1]);
        }
    }
}

void CorridorSequence::InflateCorridors(){
    bool made_change = true;
    int grow_counter = 0;
    int max_nb_grow_iterations = 3;

    // grow corridors
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = false;
        
        for (int i = 0; i < nb_of_corridors_; i++){
            made_change = GrowCorridorSideways(i) || made_change;
        }

        if (made_change){
            while (MergeCorridors()){
                continue;
            }
            // made_change = MergeCorridors() || made_change;
        }
        grow_counter++;
    }

    // grow first corridor even more
    max_nb_grow_iterations = 3; grow_counter = 0;
    made_change = true;
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = GrowCorridorSideways(0);

        sequence_[0].FlipDirection();
        made_change = GrowCorridorSideways(0);
        sequence_[0].FlipDirection();
        grow_counter++;
    }

    // remove irrelevant corridors
    RemoveIrrelevantCorridors();

    // do a final merging
    while (MergeCorridors()){
        continue;
    }
    // TODO: now the corridors are properly merged, but I get an infeasible problem. Why is this?
};

void CorridorSequence::AddCorridor(double x_min, double x_max, double y_min, 
                                   double y_max){

    // Check if there is still space to add a corridor
    if (nb_of_corridors_ >= max_len_){
        throw FullCorridorSequenceException();
    }
    
    // If so, add the new Corridor
    sequence_[nb_of_corridors_] = Corridor(x_min, x_max, y_min, y_max);
    nb_of_corridors_++;
};

void CorridorSequence::AddCorridorFromCells(Point2D<int> &start_cell, 
                                           Point2D<int> &end_cell){
    double cell_width = environment_.CellWidth();
    double cell_height = environment_.CellHeight();

    Point2D<double> start = start_cell.ConvertCellToWorld(cell_width, cell_height);
    Point2D<double> end = end_cell.ConvertCellToWorld(cell_width, cell_height);

    AddCorridor(std::min(start.x(), end.x()) - cell_width/2,
                std::max(start.x(), end.x()) + cell_width/2,
                std::min(start.y(), end.y()) - cell_height/2,
                std::max(start.y(), end.y()) + cell_height/2);
};

void CorridorSequence::RemoveCorridor(int idx){
    // Check if the index is valid
    if (idx < 0 || idx >= nb_of_corridors_){
        throw std::out_of_range("Invalid index of corridor to remove");
    }

    // If so, remove the corridor
    for (int i = idx; i < nb_of_corridors_-1; i++){
        sequence_[i].CopyValues(sequence_[i+1]);
    }
    // sequence_.erase(sequence_.begin() + idx);

    nb_of_corridors_--;
};

bool CorridorSequence::GrowCorridorSideways(int idx){
    // A corridor cannot become fat (wider than it's length) unless it is the 
    // first corridor
    if (idx > 0 && 
        (sequence_[idx].Direction().x() == 0 && 
            sequence_[idx].Width() >= sequence_[idx].Height() || 
        sequence_[idx].Direction().y() == 0 &&
            sequence_[idx].Height() >= sequence_[idx].Width())){
        return false;
    }

    // Check if we can grow
    bool growing_left_possible = CheckCellsOnLeftSide(idx);
    bool growing_right_possible = CheckCellsOnrightSide(idx);

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
                                       environment_.CellWidth());
            }
            if (growing_right_possible){
                sequence_[idx].SetXmax(sequence_[idx].Xmax() + 
                                       environment_.CellWidth());
            }
        // downward corridor
        } else {
            if (growing_left_possible){
                sequence_[idx].SetXmax(sequence_[idx].Xmax() + 
                                       environment_.CellWidth());
            }
            if (growing_right_possible){
                sequence_[idx].SetXmin(sequence_[idx].Xmin() - 
                                       environment_.CellWidth());
            }
        }

    // if horizontal corridor
    } else {
        // rightward corridor
        if (sequence_[idx].Direction().x() > 0){
            if (growing_left_possible){
                sequence_[idx].SetYmax(sequence_[idx].Ymax() + 
                                       environment_.CellHeight());
            }
            if (growing_right_possible){
                sequence_[idx].SetYmin(sequence_[idx].Ymin() - 
                                       environment_.CellHeight());
            }
        // leftward corridor
        } else {
            if (growing_left_possible){
                sequence_[idx].SetYmin(sequence_[idx].Ymin() - 
                                       environment_.CellHeight());
            }
            if (growing_right_possible){
                sequence_[idx].SetYmax(sequence_[idx].Ymax() + 
                                       environment_.CellHeight());
            }
        }
    }

    return true;
};

int CorridorSequence::GetCellsOnLeftSide(int corridor_idx){

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment_.CellWidth(), 
                                environment_.CellHeight());

    // if vertical corridor
    if (direction.x() == 0){

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetY(corridor->Ymin() + 
                                            environment_.CellHeight()/2 +
                                            i*environment_.CellHeight());
            if (direction.y() > 0){
                cells_along_corridor_[i].SetX(corridor->Xmin() - 
                                            environment_.CellWidth()/2);
            } else {
                cells_along_corridor_[i].SetX(corridor->Xmax() + 
                                            environment_.CellWidth()/2);
            }
        }
    // if horizontal corridor
    } else {

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetX(corridor->Xmin() + 
                                            environment_.CellWidth()/2 + 
                                            i*environment_.CellWidth());
            if (direction.x() > 0){
                cells_along_corridor_[i].SetY(corridor->Ymax() + 
                                            environment_.CellHeight()/2);
            } else {
                cells_along_corridor_[i].SetY(corridor->Ymin() - 
                                            environment_.CellHeight()/2);
            }
        }
    }

    return corridor_cell_length;
};

int CorridorSequence::GetCellsOnRightSide(int corridor_idx){

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment_.CellWidth(), 
                                environment_.CellHeight());

    // if vertical corridor
    if (direction.x() == 0){

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetY(corridor->Ymin() + 
                                            environment_.CellHeight()/2 +
                                            i*environment_.CellHeight());
            if (direction.y() > 0){
                cells_along_corridor_[i].SetX(corridor->Xmax() + 
                                            environment_.CellWidth()/2);
            } else {
                cells_along_corridor_[i].SetX(corridor->Xmin() - 
                                            environment_.CellWidth()/2);
            }
        }
    // if horizontal corridor
    } else {

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetX(corridor->Xmin() + 
                                            environment_.CellWidth()/2 + 
                                            i*environment_.CellWidth());
            if (direction.x() > 0){
                cells_along_corridor_[i].SetY(corridor->Ymin() - 
                                            environment_.CellHeight()/2);
            } else {
                cells_along_corridor_[i].SetY(corridor->Ymax() + 
                                            environment_.CellHeight()/2);
            }
        }
    }

    return corridor_cell_length;
};

bool CorridorSequence::CheckCellsOnLeftSide(int corridor_idx){
    int corridor_cell_length = GetCellsOnLeftSide(corridor_idx);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment_.IsFree(cells_along_corridor_[i])){
            return false;
        }
    }

    return true;
};

bool CorridorSequence::CheckCellsOnrightSide(int corridor_idx){
    int corridor_cell_length = GetCellsOnRightSide(corridor_idx);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment_.IsFree(cells_along_corridor_[i])){
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
    for (int i = nb_of_corridors_ - 2; i >= 1; i--){
        previous_corridor = &sequence_[i-1];
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        // The current corridor has to be removed if
        // - it is completely within the previous corridor
        // - it is completely within the next corridor
        // - it is completely within a union of the previous and next corridor
        // - the previous and the next corridor overlap
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
    while (nb_of_corridors_ >= 2 &&
           sequence_[1].ContainsVehicle(start_, params_)){
        RemoveCorridor(0);
    }
    while (nb_of_corridors_ >= 2 && 
           sequence_[nb_of_corridors_ - 2].ContainsVehicle(dest_, params_)){
        RemoveCorridor(nb_of_corridors_ - 1);
    }

    return made_change;
};

bool CorridorSequence::MergeCorridors(){
    double tolerance = 1e-10;

    bool made_change = false;
    
    Corridor* current_corridor;
    Corridor* next_corridor;
    for (int i = nb_of_corridors_ - 2; i >= 0; i--){
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        std::cout << std::endl << "i: " << i << std::endl;
        std::cout<< "current corridor: " << *current_corridor << std::endl;
        std::cout<< "next corridor: " << *next_corridor << std::endl;
        std::cout << std::abs(current_corridor->Xmin() - 
                     next_corridor->Xmin()) << std::endl;
        std::cout << std::abs(current_corridor->Xmax() - 
                     next_corridor->Xmax()) << std::endl;

        if (std::abs(current_corridor->Xmin() - 
                     next_corridor->Xmin()) < tolerance &&
            std::abs(current_corridor->Xmax() - 
                     next_corridor->Xmax()) < tolerance){
            current_corridor->SetYmin(std::min(current_corridor->Ymin(), 
                                               next_corridor->Ymin()));
            current_corridor->SetYmax(std::max(current_corridor->Ymax(),
                                               next_corridor->Ymax()));
            std::cout << "removing!" << std::endl;
            RemoveCorridor(i+1);
            made_change = true;
        } else if (std::abs(current_corridor->Ymin() - 
                     next_corridor->Ymin()) < tolerance &&
            std::abs(current_corridor->Ymax() - 
                     next_corridor->Ymax()) < tolerance){
            current_corridor->SetXmin(std::min(current_corridor->Xmin(), 
                                               next_corridor->Xmin()));
            current_corridor->SetXmax(std::max(current_corridor->Xmax(),
                                               next_corridor->Xmax()));
            RemoveCorridor(i+1);
            made_change = true;
        }
    }
    return made_change;
};