#include <queue>
#include <set>
#include <fstream>
#include <nlohmann/json.hpp>
#include <random>

#include "core/environment.hpp"
#include "core/corridor.hpp"

using json = nlohmann::json;

Environment::Environment(){
    nb_cell_rows_ = 10;
    nb_cell_cols_ = 12;
    cell_width_ = 0.120;
    cell_height_ = 0.120;

    occupancy_grid_ = 
        std::vector<std::vector<CellOccupancy>>(nb_cell_cols_, 
            std::vector<CellOccupancy>(nb_cell_rows_, FREE));
    
    // Delete some cells
    for (int i = 4; i <= 11; i++){
        for (int j = 4; j <= 7; j++){
            DeleteCell(Point2D<int>(i, j));
        }
    }
    for (int i = 8; i <= 11; i++){
        for (int j = 8; j <= 9; j++){
            DeleteCell(Point2D<int>(i, j));
        }
    }
}

Environment::Environment(int nb_cell_rows, int nb_cell_cols, double cell_width, 
                         double cell_height){
    nb_cell_rows_ = nb_cell_rows;
    nb_cell_cols_ = nb_cell_cols;
    cell_width_ = cell_width;
    cell_height_ = cell_height;

    occupancy_grid_ = 
        std::vector<std::vector<CellOccupancy>>(nb_cell_cols_, 
            std::vector<CellOccupancy>(nb_cell_rows_, FREE));
}

bool Environment::isValidCell(Point2D<int> cell) const {
    return cell.x() >= 0 && cell.x() < nb_cell_cols_ && 
           cell.y() >= 0 && cell.y() < nb_cell_rows_;
}

bool Environment::isValidCell(int x, int y) const {
    return x >= 0 && x < nb_cell_cols_ && y >= 0 && y < nb_cell_rows_;
}

bool Environment::isValidPosition(Point2D<double> pos) const {
    return pos.x() >= 0 && pos.x() < nb_cell_cols_ * cell_width_ && 
           pos.y() >= 0 && pos.y() < nb_cell_rows_ * cell_height_;
}

bool Environment::isValidVehiclePosition(Point2D<double> pos, 
                                         double vehicle_width, 
                                         double vehicle_length) const {
    Point2D<double> test_point;

    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            test_point.SetX(pos.x() + i*vehicle_width/2);
            test_point.SetY(pos.y() + j*vehicle_length/2);
            if (!IsFree(test_point)){
                return false;
            }
        }
    }

    return true;
};

bool Environment::IsFree(Point2D<int> const  &cell) const {
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[cell.x()][cell.y()] == FREE;
}

bool Environment::IsFree(int x, int y) const {
    if (!isValidCell(x, y)){
        throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[x][y] == FREE;
}

void Environment::DeleteCell(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot delete a cell outside of the environment");
    }
    occupancy_grid_[cell.x()][cell.y()] = DELETED;
    UpdateVersion();
}

void Environment::AddCell(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot add a cell outside of the environment");
    }
    occupancy_grid_[cell.x()][cell.y()] = FREE;
    UpdateVersion();
}

void Environment::AddObstacle(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot add an obstacle outside of the environment");
    }
    occupancy_grid_[cell.x()][cell.y()] = OCCUPIED;
    UpdateVersion();
}

void Environment::RemoveObstacle(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot remove an obstacle outside of the environment");
    }
    occupancy_grid_[cell.x()][cell.y()] = FREE;
    UpdateVersion();
}

void Environment::ClearAllObstacles(){
    for (int i = 0; i < nb_cell_cols_; i++){
        for (int j = 0; j < nb_cell_rows_; j++){
            if (occupancy_grid_[i][j] == OCCUPIED){
                occupancy_grid_[i][j] = FREE;
            }
        }
    }
    UpdateVersion();
}

void Environment::AddRandomObstacles(double obstacle_probability){
    ClearAllObstacles();

    // Initialize the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0, 1);

    // Add obstacles
    for (int i = 0; i < nb_cell_cols_; i++){
        for (int j = 0; j < nb_cell_rows_; j++){
            if (occupancy_grid_[i][j] == FREE &&
                    dis(gen) < obstacle_probability){
                AddObstacle(Point2D<int>(i, j));
            }
        }
    }
    UpdateVersion();
}

std::vector<Point2D<int>> Environment::PerformBreadthFirstSearch (
    const Point2D<int> &start, const Point2D<int> &dest) const {
    // Initialize the queue and the set of visited cells
    std::queue<std::vector<Point2D<int>>> path_queue;
    std::set<Point2D<int>> visited;

    // Add the start cell to the queue
    path_queue.push(std::vector<Point2D<int>>{start});

    // Perform the search
    std::vector<Point2D<int>> current_path;
    Point2D<int> current;
    Point2D<int> neighbour;
    while (!path_queue.empty()){
        current_path = path_queue.front();
        current = current_path.back();
        path_queue.pop();

        // Check if the destination has been reached
        if (current.x() == dest.x() && current.y() == dest.y()){
            return current_path;
        }

        // Add the neighbours of the current cell to the queue
        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                // Skip the current cell
                if (i == 0 && j == 0){
                    continue;
                }

                // Skip diagonal cells
                if (i != 0 && j != 0){
                    continue;
                }

                // Compute the neighbour cell
                neighbour.SetX(current.x() + i);
                neighbour.SetY(current.y() + j);

                // Check if the neighbour is valid and has not been visited
                if (isValidCell(neighbour) && !visited.count(neighbour) && IsFree(neighbour)){
                    visited.insert(neighbour);
                    
                    // append a copy of current path with the neighbour
                    std::vector<Point2D<int>> new_path = current_path;
                    new_path.push_back(neighbour);
                    path_queue.push(new_path);
                }
            }
        }
    } 
    return std::vector<Point2D<int>>();
};

std::vector<Point2D<int>> Environment::GetOccupiedFootprintCells(
    const Point2D<double> &point, const double &vehicle_width,
    const double &vehicle_length) const {
    // Initialize the set of occupied cells
    std::set<Point2D<int>> occupied_cells;

    // Compute the occupied cells
    Point2D<double> vehicle_edge_point;
    Point2D<int> vehicle_edge_cell;
    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            vehicle_edge_point.SetX(point.x() + i*vehicle_width/2);
            vehicle_edge_point.SetY(point.y() + j*vehicle_length/2);
            vehicle_edge_point.ConvertWorldToCell(cell_width_, cell_height_, 
                                                  vehicle_edge_cell);
            if (isValidCell(vehicle_edge_cell) && 
                    occupied_cells.count(vehicle_edge_cell) == 0){
                occupied_cells.insert(vehicle_edge_cell);
            }
        }
    }

    return std::vector<Point2D<int>>(occupied_cells.begin(), occupied_cells.end());
}

std::ostream& operator<<(std::ostream &out, Environment const &environment){
    out << environment.NbCellRows() << " x " << environment.NbCellCols() 
        << " environment (" << environment.NbCellRows()*environment.CellWidth() 
        << " x " << environment.NbCellCols()*environment.CellHeight() << ")" 
        << std::endl;
    for (int j = environment.NbCellRows() - 1; j >= 0 ; j--){
        for (int i = 0; i < environment.NbCellCols() ; i++){
            switch(environment.GetOccupancy(i, j)){
                case FREE:
                    out << ". ";
                    break;
                case DELETED:
                    out << "X ";
                    break;
                case OCCUPIED:
                    out << "# ";
                    break;
            }
        }
        out << std::endl;
    }
    return out;
}

CellOccupancy Environment::GetOccupancy(Point2D<int> cell) const {
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot get occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[cell.x()][cell.y()];
}
CellOccupancy Environment::GetOccupancy(int x, int y) const {
    if (!isValidCell(x, y)){
        throw InvalidEnvironmentOperationException("Cannot get occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[x][y];
}

void Environment::GetRandomFreeVehiclePosition(Point2D<double> &pos, 
                                               double vehicle_width, 
                                               double vehicle_length) const {
    // Initialize the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_x(0, nb_cell_cols_*cell_width_);
    std::uniform_real_distribution<double> dis_y(0, nb_cell_rows_*cell_height_);

    // Initialize the position
    pos.SetX(dis_x(gen));
    pos.SetY(dis_y(gen));

    // Check if the position is valid
    while (!isValidVehiclePosition(pos, vehicle_width, vehicle_length)){
        pos.SetX(dis_x(gen));
        pos.SetY(dis_y(gen));
    }
}

json Environment::ToJson() const {
    json j;

    j["nb_cell_rows"] = nb_cell_rows_;
    j["nb_cell_cols"] = nb_cell_cols_;
    j["cell_width"] = cell_width_;
    j["cell_height"] = cell_height_;
    j["occupancy_grid"] = occupancy_grid_;

    return j;
}