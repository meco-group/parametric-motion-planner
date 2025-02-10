#include <queue>
#include <set>
#include <unordered_set>
#include <fstream>
#include <nlohmann/json.hpp>

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
                                         double vehicle_length,
                                         double margin) const {
    Point2D<double> test_point;

    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            test_point.SetX(i > 0 ? pos.x() + i*vehicle_width/2 + margin : 
                                    pos.x() + i*vehicle_width/2 - margin);
            test_point.SetY(j > 0 ? pos.y() + j*vehicle_length/2 + margin : 
                                    pos.y() + j*vehicle_length/2 - margin);
            if (!IsFree(test_point)){
                return false;
            }
        }
    }

    return true;
};

bool Environment::IsFree(Point2D<int> const  &cell) const {
    if (!isValidCell(cell)){
        // throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
        return false;
    }
    return occupancy_grid_[cell.x()][cell.y()] == FREE;
}

bool Environment::IsFree(int x, int y) const {
    if (!isValidCell(x, y)){
        // throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
        return false;
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

    // Do not add obstacles in deleted parts
    if (occupancy_grid_[cell.x()][cell.y()] == DELETED){
        return;
    }

    occupancy_grid_[cell.x()][cell.y()] = OCCUPIED_STATIC;
    UpdateVersion();
}

void Environment::RemoveObstacle(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot remove an obstacle outside of the environment");
    }
    occupancy_grid_[cell.x()][cell.y()] = FREE;
    UpdateVersion();
}

void Environment::AddVirtualObstacle(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot add a virtual obstacle outside of the environment");
    }

    // Only add if the cell is free
    if (occupancy_grid_[cell.x()][cell.y()] == FREE){
        occupancy_grid_[cell.x()][cell.y()] = VIRTUAL_OBS;
        UpdateVersion();
    }
}

void Environment::RemoveVirtualObstacle(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot remove a virtual obstacle outside of the environment");
    }

    // Only remove if the cell is a virtual obstacle
    if (occupancy_grid_[cell.x()][cell.y()] == VIRTUAL_OBS){
        occupancy_grid_[cell.x()][cell.y()] = FREE;
        UpdateVersion();
    }
}

void Environment::ClearAllObstacles(){
    for (int i = 0; i < nb_cell_cols_; i++){
        for (int j = 0; j < nb_cell_rows_; j++){
            if (occupancy_grid_[i][j] == OCCUPIED_STATIC || 
                occupancy_grid_[i][j] == VIRTUAL_OBS){
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

void Environment::AddMovingObstacle(MovingObstacleOperationsToken&, 
                                    const Point2D<int> &cell){
    if (!isValidCell(cell)){
        // We allow the user to place dynamic obstacles outside of the environment
        // In that case, this operation is just ignored
        return;
        // throw InvalidEnvironmentOperationException("Cannot add a moving obstacle outside of the environment");
    }

    // Do not add obstacles in deleted parts
    if (occupancy_grid_[cell.x()][cell.y()] == DELETED){
        return;
    }

    if (occupancy_grid_[cell.x()][cell.y()] == OCCUPIED_STATIC){
        occupancy_grid_[cell.x()][cell.y()] = OCCUPIED_STATIC_AND_DYNAMIC;
    } else {
        occupancy_grid_[cell.x()][cell.y()] = OCCUPIED_DYNAMIC;
    }
    UpdateVersion();
}

void Environment::RemoveMovingObstacle(MovingObstacleOperationsToken&, 
                                       const Point2D<int> &cell){
    if (!isValidCell(cell)){
        // We allow the user to remove dynamic obstacles outside of the environment
        // In that case, this operation is just ignored
        return;
        // throw InvalidEnvironmentOperationException("Cannot remove a moving obstacle outside of the environment");
    }
    if (occupancy_grid_[cell.x()][cell.y()] == OCCUPIED_STATIC_AND_DYNAMIC){
        occupancy_grid_[cell.x()][cell.y()] = OCCUPIED_STATIC;
    } else {
        occupancy_grid_[cell.x()][cell.y()] = FREE;
    }
    UpdateVersion();
}

void Environment::ClearAllMovingObstacles(){
    for (int i = 0; i < nb_cell_cols_; i++){
        for (int j = 0; j < nb_cell_rows_; j++){
            if (occupancy_grid_[i][j] == OCCUPIED_DYNAMIC){
                occupancy_grid_[i][j] = FREE;
            } else if (occupancy_grid_[i][j] == OCCUPIED_STATIC_AND_DYNAMIC){
                occupancy_grid_[i][j] = OCCUPIED_STATIC;
            }
        }
    }
    UpdateVersion();
}

std::vector<Point2D<int>> Environment::PerformBreadthFirstSearch (
        const Point2D<int> &start, const Point2D<int> &dest,
        const Point2D<double> &true_dest) const {
    // Initialize the queue and the set of visited cells
    std::queue<std::vector<Point2D<int>>> path_queue;
    std::set<Point2D<int>> visited;

    // Add the start cell to the queue
    path_queue.push(std::vector<Point2D<int>>{start});

    // Perform the search
    std::vector<Point2D<int>> current_path;
    Point2D<int> current;
    // Point2D<int> neighbour;
    std::vector<Point2D<int>> neighbours(4);
    while (!path_queue.empty()){
        current_path = path_queue.front();
        current = current_path.back();
        path_queue.pop();

        // Check if the destination has been reached
        if (current.x() == dest.x() && current.y() == dest.y()){
            return current_path;
        }

        // Add the neighbours of the current cell to the queue
        // for (int i = -1; i <= 1; i++){
        // //     for (int j = -1; j <= 1; j++){
        //         // Skip the current cell
        //         if (i == 0 && j == 0){
        //             continue;
        //         }

        //         // Skip diagonal cells
        //         if (i != 0 && j != 0){
        //             continue;
        //         }
        //         // Compute the neighbour cell
        //         neighbour.SetX(current.x() + i);
        //         neighbour.SetY(current.y() + j);

        neighbours[0].SetX(current.x() - 1); neighbours[0].SetY(current.y());
        neighbours[1].SetX(current.x() + 1); neighbours[1].SetY(current.y());
        neighbours[2].SetX(current.x()); neighbours[2].SetY(current.y() - 1);
        neighbours[3].SetX(current.x()); neighbours[3].SetY(current.y() + 1);
        // sort the neighbours based on distance to the destination
        std::sort(neighbours.begin(), neighbours.end(), 
            [true_dest, this](Point2D<int> a, Point2D<int> b){
                // return a.ManhattanDistance(dest) < b.ManhattanDistance(dest);
                return a.ConvertCellToWorld(cell_width_, cell_height_).Distance(true_dest) < b.ConvertCellToWorld(cell_width_, cell_height_).Distance(true_dest);
            });
        
        for (Point2D<int> neighbour : neighbours){
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
    return std::vector<Point2D<int>>();
};

std::unordered_set<Point2D<int>, Point2DHash<int>> Environment::GetOccupiedFootprintCells(
    const Point2D<double> &point, const double &vehicle_width,
    const double &vehicle_length, const double &margin) const {
    // Initialize the set of occupied cells
    std::unordered_set<Point2D<int>, Point2DHash<int>> occupied_cells = 
        std::unordered_set<Point2D<int>, Point2DHash<int>>();

    // Compute the occupied cells
    Point2D<double> vehicle_edge_point;
    Point2D<int> vehicle_edge_cell;
    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            //TODO: In some cases, the point grid must be finer than this (large obstacles)
            vehicle_edge_point.SetX(point.x() + i*(vehicle_width/2 + margin));
            vehicle_edge_point.SetY(point.y() + j*(vehicle_length/2 + margin));
            vehicle_edge_point.ConvertWorldToCell(cell_width_, cell_height_, 
                                                  vehicle_edge_cell);
            if (isValidCell(vehicle_edge_cell) && 
                    occupied_cells.count(vehicle_edge_cell) == 0){
                occupied_cells.insert(vehicle_edge_cell);
            }
        }
    }

    return occupied_cells;
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
                case OCCUPIED_STATIC:
                    out << "# ";
                    break;
                case OCCUPIED_STATIC_AND_DYNAMIC:
                    out << "#";
                    break;
                case OCCUPIED_DYNAMIC:
                    out << "= ";
                    break;
            }
        }
        out << std::endl;
    }
    std::vector<int> rr = {};
    std::vector<int> cc = {};
    for (int i = 0; i < environment.NbCellCols(); i++){
        for (int j = 0; j < environment.NbCellRows(); j++){
            if (!environment.IsFree(Point2D<int>(i, j))){
                rr.push_back(i);
                cc.push_back(j);
            }
        }
    }
    out << "std::vector<int> rr = {";
    for (int i = 0; i < rr.size(); i++){
        out << rr[i];
        if (i < rr.size() - 1){
            out << ", ";
        }
    }
    out << "};" << std::endl;
    out << "std::vector<int> cc = {";
    for (int i = 0; i < cc.size(); i++){
        out << cc[i];
        if (i < cc.size() - 1){
            out << ", ";
        }
    }
    out << "};" << std::endl;

    return out;
}

bool Environment::operator==(const Environment &other) const {
    return this == &other;
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

double TransformXPosition(double x, double x_min, double x_max){
    double b = 0.8;
    double x_normalized = (x - x_min)/(x_max - x_min);
    return x_min + (x_max - x_min)*(-b*std::pow(-x_normalized+1, 4) + 1);
}

double TransformYPosition(double y, double y_min, double y_max, bool upper){
    double y_normalized = (y - y_min)/(y_max - y_min);
    return upper ? y_min + (0.5 + 0.5*y_normalized)*(y_max - y_min) : 
                   y_min + (0.5 - 0.5*y_normalized)*(y_max - y_min);
}

void Environment::GetRandomFreeVehiclePosition(Point2D<double> &pos, 
                                               double vehicle_width, 
                                               double vehicle_length,
                                               double margin) const {
    bool upper = pos.y() < 0.5*(nb_cell_rows_*cell_height_);
    
    // Initialize the position
    if (nb_cell_rows_ == 10 && nb_cell_cols_ == 12){
        pos.SetX(TransformXPosition(nb_cell_cols_*cell_width_*dis_x_(gen_), 
                 0, nb_cell_cols_*cell_width_));
        pos.SetY(TransformYPosition(nb_cell_rows_*cell_height_*dis_y_(gen_),
                 0, nb_cell_rows_*cell_height_, upper));
    } else {
        pos.SetX(nb_cell_cols_*cell_width_*dis_x_(gen_));
        pos.SetY(nb_cell_rows_*cell_height_*dis_y_(gen_));
    }

    // Check if the position is valid
    while (!isValidVehiclePosition(pos, vehicle_width, vehicle_length, margin+1.0e-6)){
        if (nb_cell_rows_ == 10 && nb_cell_cols_ == 12){
            pos.SetX(TransformXPosition(nb_cell_cols_*cell_width_*dis_x_(gen_), 
                                        0, nb_cell_cols_*cell_width_));
            pos.SetY(TransformYPosition(nb_cell_rows_*cell_height_*dis_y_(gen_), 
                                        0, nb_cell_rows_*cell_height_, upper));
        } else {
            pos.SetX(nb_cell_cols_*cell_width_*dis_x_(gen_));
            pos.SetY(nb_cell_rows_*cell_height_*dis_y_(gen_));
        }
    }
}

void Environment::GetRandomFreeCellPosition(Point2D<double> &pos) const {
    // Initialize the position
    Point2D<int> cell;
    cell.SetX(nb_cell_cols_*dis_x_(gen_));
    cell.SetY(nb_cell_rows_*dis_y_(gen_));

    while(!isValidCell(cell) || !IsFree(cell)){
        cell.SetX(nb_cell_cols_*dis_x_(gen_));
        cell.SetY(nb_cell_rows_*dis_y_(gen_));
    }

    pos = cell.ConvertCellToWorld(cell_width_, cell_height_);
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