#ifndef __ENVIRONMENT__
#define __ENVIRONMENT__

#include <vector>
#include <nlohmann/json.hpp>

#include "helper_types.hpp"
#include "exceptions.hpp"
#include "corridor.hpp"

using json = nlohmann::json;

class CorridorSequence;

// Class representing the environment
class Environment{
    public:
        Environment();
        Environment(int nb_cell_rows, int nb_cell_cols, double cell_width, 
                    double cell_height);

        
        // Validity checks
        bool isValidCell(Point2D<int> cell) const {
            return cell.x() >= 0 && cell.x() < nb_cell_cols_ && 
                   cell.y() >= 0 && cell.y() < nb_cell_rows_;
        }
        bool isValidCell(int x, int y) const {
            return x >= 0 && x < nb_cell_cols_ && 
                   y >= 0 && y < nb_cell_rows_;
        }
        bool isValidPosition(Point2D<double> pos) const {
            return pos.x() >= 0 && pos.x() < nb_cell_cols_ * cell_width_ && 
                   pos.y() >= 0 && pos.y() < nb_cell_rows_ * cell_height_;
        }
        bool isValidVehiclePosition(Point2D<double> pos, double vehicle_width, 
                                    double vehicle_length) const;


        bool IsFree(Point2D<double> const &pos) const {
            if (!isValidPosition(pos)){ return false;}
            Point2D<int> cell = pos.ConvertWorldToCell(cell_width_, cell_height_);
            return IsFree(cell);
        }
        bool IsFree(Point2D<int> const &cell) const;
        bool IsFree(int x, int y) const;

        // Environment operations
        void DeleteCell(Point2D<int> cell){
            if (!isValidCell(cell)){
                throw InvalidEnvironmentOperationException("Cannot delete a cell outside of the environment");
            }
            occupancy_grid_[cell.x()][cell.y()] = DELETED;
        }

        void AddCell(Point2D<int> cell){
            if (!isValidCell(cell)){
                throw InvalidEnvironmentOperationException("Cannot add a cell outside of the environment");
            }
            occupancy_grid_[cell.x()][cell.y()] = FREE;
        }

        void AddObstacle(Point2D<int> cell){
            if (!isValidCell(cell)){
                throw InvalidEnvironmentOperationException("Cannot add an obstacle outside of the environment");
            }
            occupancy_grid_[cell.x()][cell.y()] = OCCUPIED;
        }

        void RemoveObstacle(Point2D<int> cell){
            if (!isValidCell(cell)){
                throw InvalidEnvironmentOperationException("Cannot remove an obstacle outside of the environment");
            }
            occupancy_grid_[cell.x()][cell.y()] = FREE;
        } 


        // Function to perform a breadth-first search in the environment
        std::vector<Point2D<int>> PerformBreadthFirstSearch(
            const Point2D<int> &start,const Point2D<int> &dest) const;

        // Return the cells that are occupied by the footprint of the vehicle
        std::vector<Point2D<int>> GetOccupiedFootprintCells(
            const Point2D<double> &start, const double &vehicle_width,
            const double &vehicle_length) const;

        // Function to print the occupancy grid
        friend std::ostream& operator<<(std::ostream &out, Environment const &environment);

        // basic getters
        int NbCellRows() const { return nb_cell_rows_;};
        int NbCellCols() const { return nb_cell_cols_;};
        double CellWidth() const { return cell_width_;};
        double CellHeight() const { return cell_height_;};
        
        CellOccupancy GetOccupancy(Point2D<int> cell) const;
        CellOccupancy GetOccupancy(int x, int y) const;

        void GetRandomFreeVehiclePosition(Point2D<double> &pos,
                                          double vehicle_width,
                                          double vehicle_height) const;

        json ToJson() const;

    private:

        int nb_cell_rows_;
        int nb_cell_cols_;
        double cell_width_;
        double cell_height_;

        // Occupancy grid representing the environment
        std::vector<std::vector<CellOccupancy>> occupancy_grid_;

};

#endif