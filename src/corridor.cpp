#include <stdexcept>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fstream>

#include "core/corridor.hpp"

using json = nlohmann::json;

// Corridor class

bool Corridor::GetOverlap(Corridor const &other, Corridor &overlap) const {
    double tolerance = 1e-5;
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









// CorridorUnion class
bool CorridorUnion::ContainsPoint(Point2D<double> const &point) const {
    for (const auto &corridor : union_){
        if (corridor.ContainsPoint(point)){
            return true;
        }
    }
    return false;
}

bool CorridorUnion::ContainsVehicle(const Point2D<double> &vehicle_position, 
                                    const Parameters &params) const {
    for (const auto &corridor : union_){
        if (corridor.ContainsVehicle(vehicle_position, params)){
            return true;
        }
    }
    return false;
}

bool CorridorUnion::OverlapsWith(const Corridor& other) const {
    Corridor overlap;
    for (const auto &corridor : union_){
        if (corridor.GetOverlap(other, overlap)){
            return true;
        }
    }
    return false;
}

json CorridorUnion::ToJson() const {
    json j;
    j["sequence"] = json::array();
    for (const auto &corridor : union_){
        j["sequence"].push_back(corridor.ToJson());
    }
    return j;
}

void CorridorUnion::MinimizeRepresentation(){
    // TODO
}



















// CorridorSequence class

void CorridorSequence::UpdateSequence(Point2D<double> const &start,
                                      Point2D<double> const &dest,
                                      Point2D<double> const &start_vel,
                                      Parameters const &params,
                                      UpdateToken const &token,
                                      int max_nb_grow_iterations){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot update corridor sequence when not considering full sequence");
    }
    auto start_corridor_sequence_update_time = std::chrono::high_resolution_clock::now();

    sequence_available_ = false;
    // Input checks
    // if (!environment_.isValidPosition(start) || 
    //     !environment_.isValidPosition(dest)){
    if (!environment_.isValidVehiclePosition(start, params.GetVehWidth(), 
                                             params.GetVehHeight(), 
                                             params.GetMargin())){
        Point2D<double> test_point;

        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                test_point.SetX(i > 0 ? start.x() + i*params.GetVehWidth()/2 + params.GetMargin() : 
                                        start.x() + i*params.GetVehWidth()/2 - params.GetMargin());
                test_point.SetY(j > 0 ? start.y() + j*params.GetVehHeight()/2 + params.GetMargin() : 
                                        start.y() + j*params.GetVehHeight()/2 - params.GetMargin());
                if (!environment_.IsFree(test_point)){
                    std::cout << "found test point that is not free: " << test_point << std::endl;
                }
            }
        }
        throw InvalidPositionInEnvironmentException("Invalid STARTING POSITION or destination");
    }
    if (!environment_.isValidVehiclePosition(dest, params.GetVehWidth(), 
                                             params.GetVehHeight(), 
                                             params.GetMargin())){
        throw InvalidPositionInEnvironmentException("Invalid starting position or DESTINATION");
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
    true_start_.CopyValues(start);
    true_dest_.CopyValues(dest);
    true_start_vel_.CopyValues(start_vel);


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
    path_ = environment_.PerformBreadthFirstSearch(start_cell, dest_cell, dest);
    if (path_.size() == 0){
        sequence_available_ = false;
        throw std::runtime_error("No path found from start to destination");
        return;
    }

    // Add cells to ensure initial footprint of the vehicle is included
    AddInitialFootprint(path_);

    // Add cells to ensure final footprint of the vehicle is included
    AddFinalFootprint(path_);

    if (path_.size() == 1){
        std::cout << "NOTE: path size is 1" << std::endl;
        AddCorridorFromCells(path_[0], path_[0]);
    } else if (extended_corridors_mode_){
        // add a new corridor for all neighbouring cells
        for (int i = 2; i < path_.size(); i++){
            AddCorridorFromCells(path_[i-1], path_[i]);
        }
    } else {
        // as long as cells are on the same row/column, add them to the same
        // corridor
        Point2D<int> curr_start_cell = path_[0];
        Point2D<int> curr_end_cell = path_[1];
        Point2D<int> curr_direction = 
                            Point2D<int>(curr_end_cell.x() - curr_start_cell.x(), 
                                     curr_end_cell.y() - curr_start_cell.y());

        Point2D<int> next_direction;
        for (int i = 2; i < path_.size(); i++){
            Point2D<int> next_cell = path_[i];
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
    }

    // Inflate the corridors
    InflateCorridors(max_nb_grow_iterations);

    sequence_available_ = true;
    latest_envrionment_version_ = environment_.GetVersion();
    UpdateVersion();

    auto end_corridor_sequence_update_time = std::chrono::high_resolution_clock::now();
    corridor_sequence_construction_time_ = 
        std::chrono::duration<double, std::milli>(
            end_corridor_sequence_update_time - 
            start_corridor_sequence_update_time).count();
};

bool CorridorSequence::ContainsPoint(const Point2D<double> &point) const {
    if (!sequence_available_){
        throw InvalidCorridorSequenceOperationException("Corridor sequence not available");
    }
    for (int i = 0; i < nb_of_corridors_; i++){
        if (sequence_[i].ContainsPoint(point)){
            return true;
        }
    }
    return false;
};

void CorridorSequence::SetFirstCorridorIdx(int idx){
    if (idx < 0 || idx > last_corridor_idx_){
        std::cout << "idx: " << idx << std::endl;
        std::cout << "last corridor index: " << last_corridor_idx_ << std::endl;
        std::cout << "sequence: " << std::endl;
        std::cout << *this << std::endl;
        throw std::out_of_range("Invalid index of first corridor");
    }

    if (idx == 0){
        start_ = true_start_;
        start_vel_ = true_start_vel_;
        first_corridor_idx_ = 0;
        UpdateVersion();
        return;
    }

    Corridor overlap;
    Corridor prev_corridor = GetCorridorByRawIndex(idx - 1);
    GetCorridorByRawIndex(idx).GetOverlap(prev_corridor, overlap);
    overlap.GetCenter(start_);
    start_vel_.SetX(0.0);
    start_vel_.SetY(0.0);

    first_corridor_idx_ = idx;
    UpdateVersion();
};

void CorridorSequence::SetLastCorridorIdx(int idx){
    if (idx < first_corridor_idx_ || idx >= nb_of_corridors_){
        throw std::out_of_range("Invalid index of last corridor");
    }

    if (idx == nb_of_corridors_ - 1){
        dest_ = true_dest_;
        last_corridor_idx_ = nb_of_corridors_ - 1;
        UpdateVersion();
        return;
    }

    Corridor overlap;
    Corridor next_corridor = GetCorridorByRawIndex(idx + 1);
    GetCorridorByRawIndex(idx).GetOverlap(next_corridor, overlap);
    overlap.GetCenter(dest_);

    last_corridor_idx_ = idx;
    UpdateVersion();
}

void CorridorSequence::ResetCorridorIdxs(){
    first_corridor_idx_ = 0;
    last_corridor_idx_ = nb_of_corridors_ - 1;
    
    start_ = true_start_;
    start_vel_ = true_start_vel_;
    dest_ = true_dest_;
    
    UpdateVersion();
};

void CorridorSequence::UpdateCorridorIdxs(Point2D<double> const &start,
                                          Point2D<double> const &dest,
                                          Point2D<double> const &start_vel) {
    int first_idx = nb_of_corridors_ - 1;
    while (first_idx > 0 && 
           !sequence_[first_idx].ContainsVehicle(start, params_)){
        first_idx--;
    }

    int last_idx = first_idx;
    while (last_idx < nb_of_corridors_ - 1 && 
           !sequence_[last_idx].ContainsVehicle(dest, params_)){
        last_idx++;
    }

    // little hack to make sure corridor indices can be updated
    if (first_idx > last_corridor_idx_){ last_corridor_idx_ = first_idx; }
    if (last_idx < first_corridor_idx_){ first_corridor_idx_ = last_idx; }

    // update indices
    SetFirstCorridorIdx(first_idx);
    SetLastCorridorIdx(last_idx);
    start_ = start;
    dest_ = dest;
    start_vel_ = start_vel;
};

Corridor CorridorSequence::GetCorridor(int idx) const {
    if (idx < 0 || idx >= NbCorridors()){
        // print the name of the function that called this function
        throw std::out_of_range("Invalid index of corridor to get");
    }
    return sequence_[idx + first_corridor_idx_].Copy();
};

void CorridorSequence::GetCorridor(int idx, Corridor &corridor) const {
    if (idx < 0 || idx >= NbCorridors()){
        throw std::out_of_range("Invalid index of corridor to get");
    }
    corridor.CopyValues(sequence_[idx + first_corridor_idx_]);
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

int CorridorSequence::GetIdxOfCorridorThatContainsPoint(
        const Point2D<double> &point) const {
    // TODO: consider the case where the point is in multiple corridors
    Corridor curr_corridor;
    for (int i = 0; i < NbCorridors(); i++){
        GetCorridor(i, curr_corridor);
        if (curr_corridor.ContainsPoint(point)){
            return i;
        }
    }
    return -1;
};

std::vector<Point2D<double>> CorridorSequence::GetCorridorOverlapCenters() const {
    // Initialize points
    std::vector<Point2D<double>> centers(1 + NbCorridors());
    centers[0].CopyValues(GetStart());
    centers[centers.size() - 1].CopyValues(GetDest());

    Corridor current_corridor, next_corridor, overlap;
    for (int i = 0; i < NbCorridors() - 1; i++){
        // Get the relevant corridors and the overlap
        current_corridor = GetCorridor(i);
        next_corridor = GetCorridor(i+1);
        current_corridor.GetOverlap(next_corridor, overlap);

        // Add the new point
        centers[i+1].SetX((overlap.Xmin() + overlap.Xmax())/2);
        centers[i+1].SetY((overlap.Ymin() + overlap.Ymax())/2);
    }

    return centers;
}

CorridorUnion CorridorSequence::GetOverlap(CorridorSequence& other) {
    CorridorUnion overlaps;
    // for (int i = 0; i < nb_of_corridors_; i++){
    //     for (int j = 0; j < other.nb_of_corridors_; j++){
    //         Corridor overlap;
    //         if (GetCorridorByRawIndex(i).GetOverlap(other.GetCorridorByRawIndex(j), overlap)){
    //             overlaps.AddCorridor(overlap);
    //         }
    //     }
    // }
    for (int i = 0; i < NbCorridors(); i++){
        for (int j = 0; j < other.NbCorridors(); j++){
            Corridor overlap;
            if (GetCorridor(i).GetOverlap(other.GetCorridor(j), overlap)){
                overlaps.AddCorridor(overlap);
            }
        }
    }
    return overlaps;
}

Point2D<double> CorridorSequence::GetWaitingPosition(
                                        Point2D<double> const &curr_pos,
                                        CorridorUnion const &intersection, 
                                        Parameters const &params,
                                        double cell_width, double cell_height) const {
    // if the vehicle is already in the intersection, just wait
    // if (intersection.ContainsPoint(curr_pos)){
    //     return curr_pos;
    // }
    
    // find first corridor that overlaps with the intersection
    int idx = -1;
    Corridor o;
    // for (int i = 0; i < nb_of_corridors_; i++){
    for (int i = 0; i < NbCorridors(); i++){
        // if (intersection.OverlapsWith(GetCorridorByRawIndex(i))){
        if (intersection.OverlapsWith(GetCorridor(i))){
            idx = i;
            break;
        }
    }

    if (idx == -1){
        throw UnableToFindWaitingPoint("No corridor overlaps with the intersection");
    }

    // Corridor c = GetCorridorByRawIndex(idx);
    Corridor c = GetCorridor(idx);

    // find the candidate that is closest to the reference point
    Point2D<double> reference_point = curr_pos;
    // if (idx == 0){ reference_point = start_; }
    // else {
    //     // GetCorridorByRawIndex(idx - 1).GetOverlap(GetCorridorByRawIndex(idx), o);
    //     GetCorridor(idx - 1).GetOverlap(GetCorridor(idx), o);
    //     o.GetCenter(reference_point);
    // }

    // Construct candidates around the intersection
    std::vector<Point2D<double>> candidates;
    std::vector<double> distances;
    Point2D<double> candidate;
    double current_best_distance = 1.0e20;
    Point2D<double> waiting_position;

    // for every corridor in the intersection list
    for (Corridor intersection_corridor : intersection.GetCorridors()){
        int corridor_cell_width = std::round((intersection_corridor.Xmax() - 
                                    intersection_corridor.Xmin())/cell_width);
        int corridor_cell_height = std::round((intersection_corridor.Ymax() - 
                                    intersection_corridor.Ymin())/cell_height);
        // check horizontally
        for (int i = -1; i < corridor_cell_width + 1; i++){
            candidate.SetX(intersection_corridor.Xmin() + i*cell_width + cell_width/2);
            candidate.SetY(intersection_corridor.Ymin() - cell_height/2);
            if (c.ContainsVehicle(candidate, params) && 
                    !intersection.ContainsPoint(candidate) &&
                    candidate.ManhattanDistance(reference_point) < current_best_distance){
                waiting_position.CopyValues(candidate);
                current_best_distance = candidate.ManhattanDistance(reference_point);
            }

            candidate.SetY(intersection_corridor.Ymax() + cell_height/2);
            if (c.ContainsVehicle(candidate, params) && 
                    !intersection.ContainsPoint(candidate) &&
                    candidate.ManhattanDistance(reference_point) < current_best_distance){
                waiting_position.CopyValues(candidate);
                current_best_distance = candidate.ManhattanDistance(reference_point);
            }
        }
        // check vertically
        for (int i = 0; i < corridor_cell_height; i++){
            candidate.SetX(intersection_corridor.Xmin() - cell_width/2);
            candidate.SetY(intersection_corridor.Ymin() + i*cell_height + cell_height/2);
            if (c.ContainsVehicle(candidate, params) && 
                    !intersection.ContainsPoint(candidate) &&
                    candidate.ManhattanDistance(reference_point) < current_best_distance){
                waiting_position.CopyValues(candidate);
                current_best_distance = candidate.ManhattanDistance(reference_point);
            }

            candidate.SetX(intersection_corridor.Xmax() + cell_width/2);
            if (c.ContainsVehicle(candidate, params) && 
                    !intersection.ContainsPoint(candidate) &&
                    candidate.ManhattanDistance(reference_point) < current_best_distance){
                waiting_position.CopyValues(candidate);
                current_best_distance = candidate.ManhattanDistance(reference_point);
            }
        }
    }

    if (current_best_distance > 1.0e10){
        std::cout << "WARNING: no waiting position found" << std::endl;
        throw UnableToFindWaitingPoint("No waiting position found in corridor sequence");
    }

    return waiting_position;    
}

json CorridorSequence::ToJson() const {
    // Create a new JSON entry for the Environment class
    json corridor_sequence_json;
    corridor_sequence_json["start"] = start_.ToJson();
    corridor_sequence_json["dest"] = dest_.ToJson();
    corridor_sequence_json["start_vel"] = start_vel_.ToJson();
    corridor_sequence_json["nb_of_corridors"] = nb_of_corridors_;
    std::vector<json> sequence_json = std::vector<json>(nb_of_corridors_);
    for (int i = 0; i < nb_of_corridors_; i++){ sequence_json[i] = sequence_[i].ToJson();}
    corridor_sequence_json["sequence"] = sequence_json;
    std::vector<json> path_json = std::vector<json>(path_.size());
    for (int i = 0; i < path_.size(); i++){ path_json[i] = path_[i].ToJson();}
    corridor_sequence_json["original_path"] = path_json;
    corridor_sequence_json["corridor_sequence_construction_time"] = 
        corridor_sequence_construction_time_;
    corridor_sequence_json["parameters"] = params_.ToJson();

    return corridor_sequence_json;
}

bool CorridorSequence::CurrentlyConsideringFullSequence() const {
    return first_corridor_idx_ == 0 && last_corridor_idx_ == nb_of_corridors_ - 1;
}

Corridor CorridorSequence::GetCorridorByRawIndex(int idx) const {
    if (idx < 0 || idx >= max_len_){
        throw std::out_of_range("Invalid index of corridor to get");
    }
    return sequence_[idx].Copy();
}

void CorridorSequence::AddInitialFootprint(std::vector<Point2D<int>> &path) const {
    std::unordered_set<Point2D<int>, Point2DHash<int>> occupied_cells_set = 
        environment_.GetOccupiedFootprintCells(start_, params_.GetVehWidth(), 
                                               params_.GetVehHeight(),
                                               params_.GetMargin());

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
                                               params_.GetVehHeight(),
                                               params_.GetMargin());

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

void CorridorSequence::InflateCorridors(int max_nb_grow_iterations){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot inflate a subset of the sequence");
    }
    bool made_change = true;
    int grow_counter = 0;
    if (extended_corridors_mode_) {max_nb_grow_iterations = 100;}

    // grow corridors
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = false;
        
        for (int i = 0; i < nb_of_corridors_; i++){
            made_change = GrowCorridorSideways(i) || made_change;
        }

        if (!extended_corridors_mode_ && made_change){
            while (MergeCorridors()){ continue;}
            // made_change = MergeCorridors() || made_change;
        }
        grow_counter++;

        // TODO: reduce overlap of consecutive corridors
        // This would allow for better growing
    }

    // grow first corridor even more
    // max_nb_grow_iterations = extended_corridors_mode_ ? 100 : 4; grow_counter = 0;
    made_change = true;
    while (made_change && grow_counter < max_nb_grow_iterations){
        made_change = GrowCorridorSideways(0);

        sequence_[0].FlipDirection();
        made_change = GrowCorridorSideways(0);
        sequence_[0].FlipDirection();
        grow_counter++;
    }

    // // grow last corridor even more
    // max_nb_grow_iterations = extended_corridors_mode_ ? 100 : 4; grow_counter = 0;
    // made_change = true;
    // int last_corridor_idx = nb_of_corridors_ - 1;
    // while (made_change && grow_counter < max_nb_grow_iterations){
    //     made_change = GrowCorridorSideways(last_corridor_idx);

    //     sequence_[last_corridor_idx].FlipDirection();
    //     made_change = GrowCorridorSideways(last_corridor_idx);
    //     sequence_[last_corridor_idx].FlipDirection();
    //     grow_counter++;
    // }

    // remove irrelevant corridors
    if (!extended_corridors_mode_){ RemoveIrrelevantCorridors();}

    // do a final merging
    while (MergeCorridors()){ continue;}
};

void CorridorSequence::AddCorridor(double x_min, double x_max, double y_min, 
                                   double y_max){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot add a corridor in a subset of the sequence");
    }
    // Check if there is still space to add a corridor
    if (nb_of_corridors_ >= max_len_){
        throw FullCorridorSequenceException();
    }
    
    // If so, add the new Corridor
    sequence_[nb_of_corridors_] = Corridor(x_min, x_max, y_min, y_max);
    nb_of_corridors_++;
    last_corridor_idx_ = nb_of_corridors_ - 1;
};

void CorridorSequence::AddCorridorFromCells(Point2D<int> const &start_cell, 
                                           Point2D<int> const &end_cell){
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
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot remove a corridor in a subset of the sequence");
    }
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
    last_corridor_idx_ = nb_of_corridors_ - 1;
};

bool CorridorSequence::GrowCorridorSideways(int idx){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot grow a corridor in a subset of the sequence");
    }
    // A corridor cannot become fat (wider than it's length) unless it is the 
    // first corridor (or we use extended corridors)
    if (idx > 0 && idx < nb_of_corridors_-1 && 
        !extended_corridors_mode_ &&
        (sequence_[idx-first_corridor_idx_].Direction().x() == 0 && 
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
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot perform operation on a subset of the sequence");
    }

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment_.CellWidth(), 
                                environment_.CellHeight());
    
    if (corridor_cell_length > MAX_CORRIDOR_CELL_LENGTH){
        throw InvalidCorridorSequenceOperationException("Corridor is too long");
    }

    // if vertical corridor
    if (direction.x() == 0){

        // loop over all cells along the corridor
        for (int i = 0; i < corridor_cell_length; i++){
            cells_along_corridor_[i].SetY(corridor->Ymin() + 
                                            environment_.CellHeight()/2 +
                                            i*environment_.CellHeight());
            // std::cout << "b" << std::endl;
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
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot perform operation on a subset of the sequence");
    }

    // Get the direction of the corridor
    Corridor* corridor = &sequence_[corridor_idx];
    Point2D<int> direction = corridor->Direction();

    int corridor_cell_length = 
        corridor->GetCellLength(environment_.CellWidth(), 
                                environment_.CellHeight());

    if (corridor_cell_length > MAX_CORRIDOR_CELL_LENGTH){
        throw InvalidCorridorSequenceOperationException("Corridor is too long");
    }

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
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot perform operation on a subset of the sequence");
    }

    int corridor_cell_length = GetCellsOnLeftSide(corridor_idx);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment_.IsFree(cells_along_corridor_[i])){
            return false;
        }
    }

    return true;
};

bool CorridorSequence::CheckCellsOnrightSide(int corridor_idx){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot perform operation on a subset of the sequence");
    }

    int corridor_cell_length = GetCellsOnRightSide(corridor_idx);

    for (int i = 0; i < corridor_cell_length; i++){
        if (!environment_.IsFree(cells_along_corridor_[i])){
            return false;
        }
    }

    return true;
};

bool CorridorSequence::RemoveIrrelevantCorridors(){
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot remove irrelevant corridors of a subset of the sequence");
    }

    bool made_change = false;

    Corridor* previous_corridor;
    Corridor* current_corridor;
    Corridor* next_corridor;
    Corridor overlap;
    for (int i = nb_of_corridors_ - 2; i > 0; i--){
        previous_corridor = &sequence_[i-1];
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        // The current corridor has to be removed if
        // - it is completely within the previous corridor
        // - it is completely within the next corridor
        // - it is completely within a union of the previous and next corridor 
        //      (that share sufficient overlap)
        // - the previous and the next corridor overlap
        if (current_corridor->IsCompletelyWithin(previous_corridor)
            ||
            current_corridor->IsCompletelyWithin(next_corridor)
            ||
            current_corridor->IsCompletelyWithin(previous_corridor, 
                                                 next_corridor) 
                &&
                std::min(previous_corridor->Xmax(), next_corridor->Xmax()) -
                std::max(previous_corridor->Xmin(), next_corridor->Xmin()) 
                    > params_.GetVehWidth() + 2*params_.GetMargin() 
                &&
                std::min(previous_corridor->Ymax(), next_corridor->Ymax()) -
                std::max(previous_corridor->Ymin(), next_corridor->Ymin()) 
                    > params_.GetVehHeight() + 2*params_.GetMargin()            
            ||
            previous_corridor->GetOverlap(*next_corridor, overlap)){
                RemoveCorridor(i);
                made_change = true;
                continue; // move on to the next corridor
        }
    }
    
    // check if first corridor is completely within the second corridor
    // and remove it if so
    if (nb_of_corridors_ >= 2 && 
        sequence_[0].IsCompletelyWithin(&sequence_[1])){
        RemoveCorridor(0);
        made_change = true;
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
    if (!CurrentlyConsideringFullSequence()){
        throw InvalidCorridorSequenceOperationException("Cannot merge corridors of a subset of the sequence");
    }

    double tolerance = 1e-10;

    bool made_change = false;
    
    Corridor* current_corridor;
    Corridor* next_corridor;
    for (int i = nb_of_corridors_ - 2; i >= 0; i--){
        current_corridor = &sequence_[i];
        next_corridor = &sequence_[i+1];

        if (std::abs(current_corridor->Xmin() - 
                     next_corridor->Xmin()) < tolerance &&
            std::abs(current_corridor->Xmax() - 
                     next_corridor->Xmax()) < tolerance){
            current_corridor->SetYmin(std::min(current_corridor->Ymin(), 
                                               next_corridor->Ymin()));
            current_corridor->SetYmax(std::max(current_corridor->Ymax(),
                                               next_corridor->Ymax()));
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