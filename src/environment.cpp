#include <queue>
#include <set>

#include "environment.hpp"
#include "corridor.hpp"

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

void Environment::GetCorridorSequence(const Point2D<double> &start, 
                                      const Point2D<double> &dest,
                                      const double &vehicle_width,
                                      const double &vehicle_height){
    // Convert start and destination to cell points
    Point2D<int> start_cell = start.ConvertWorldToCell(cell_width_, cell_height_);
    Point2D<int> dest_cell = dest.ConvertWorldToCell(cell_width_, cell_height_);

    // Compute a path in the cell environment from start to dest
    std::vector<Point2D<int>> path = PerformBreadthFirstSearch(start_cell, dest_cell);

    // Add cells to ensure initial footprint of the vehicle is included
    std::vector<Point2D<int>> occupied_cells = 
        GetOccupiedStartingCells(start_cell, vehicle_width, vehicle_height);

    // Initialize the corridor sequence
    CorridorSequence corridor_sequence = CorridorSequence();
    corridor_sequence.InitializeFromCellPath(path, cell_width_, cell_height_);

    
    // World coordinates
        // Use the path to determine the corridors of minimal width

        // Grow corridors sideways and merge if needed

        // Grow the first corridor as much as possible

        // Filter corridors that are completely within neighbouring corridors

        // Potentially remove first/last corridor if they are redundant

        // Another sweep of merging (?)
}

std::ostream& operator<<(std::ostream &out, Environment &environment){
    out << environment.NbCellRows() << " x " << environment.NbCellCols() << " environment" << std::endl;
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

bool Environment::IsFree(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[cell.x()][cell.y()] == FREE;
}
bool Environment::IsFree(int x, int y){
    if (!isValidCell(x, y)){
        throw InvalidEnvironmentOperationException("Cannot check occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[x][y] == FREE;
}
CellOccupancy Environment::GetOccupancy(Point2D<int> cell){
    if (!isValidCell(cell)){
        throw InvalidEnvironmentOperationException("Cannot get occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[cell.x()][cell.y()];
}
CellOccupancy Environment::GetOccupancy(int x, int y){
    if (!isValidCell(x, y)){
        throw InvalidEnvironmentOperationException("Cannot get occupancy of a cell outside of the environment");
    }
    return occupancy_grid_[x][y];
}

std::vector<Point2D<int>> Environment::PerformBreadthFirstSearch(
    const Point2D<int> &start, const Point2D<int> &dest){
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

std::vector<Point2D<int>> Environment::GetOccupiedStartingCells(
    const Point2D<int> &start, const double &vehicle_width,
    const double &vehicle_length){
    // Initialize the set of occupied cells
    std::set<Point2D<int>> occupied_cells;

    // Compute the occupied cells
    Point2D<double> vehicle_edge_point;
    Point2D<int> vehicle_edge_cell;
    for (int i = -1; i <= 1; i++){
        for (int j = -1; j <= 1; j++){
            vehicle_edge_point.SetX(start.x() + i);
            vehicle_edge_point.SetY(start.y() + j);
            vehicle_edge_point.ConvertWorldToCell(cell_width_, cell_height_, 
                                                  vehicle_edge_cell);
            if (isValidCell(vehicle_edge_cell) && occupied_cells.count(vehicle_edge_cell) == 0){
                occupied_cells.insert(vehicle_edge_cell);
            }
        }
    }

    return std::vector<Point2D<int>>(occupied_cells.begin(), occupied_cells.end());
}