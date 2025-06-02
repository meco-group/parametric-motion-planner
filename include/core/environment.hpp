#ifndef __ENVIRONMENT__
#define __ENVIRONMENT__

#include <vector>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <random>

#include "helper_types.hpp"
#include "exceptions.hpp"
#include "corridor.hpp"

using json = nlohmann::json;

class CorridorSequence;

// Class representing a "lockable" destination in the environment
// This location is considered an obstacle for all vehicles but can be 
class ClaimableDestination {
    public:
        ClaimableDestination() = default;
        ClaimableDestination(Point2D<int> location) : 
            location_(location) {};

        bool DestinationIsAt(int x, int y) const {
            return location_.x() == x && location_.y() == y;
        }

        const Point2D<int>& GetLocation() const {return location_;}

        bool Claimed() const { return claimed_;}

        bool ClaimedByCaller(const void* caller) const {
            return claimed_ && claimed_by_ == caller;
        }

        const void* ClaimedBy() const {
            if (!claimed_){
                throw InvalidEnvironmentOperationException("Destination is not claimed");
            }
            return claimed_by_;
        }

        bool Claim(const void* caller) {
            if (claimed_ && ClaimedBy() != caller){
                return false;
            }
            claimed_ = true;
            claimed_by_ = caller;
            return true;
        }

        void Release(const void* caller){
            if (!claimed_ || claimed_by_ != caller){
                throw InvalidEnvironmentOperationException("Destination not claimed by this caller");
            }
            claimed_ = false;
            claimed_by_ = nullptr;
        }

        json ToJson() const {
            json j;
            j["location"] = location_.ToJson();
            j["claimed"] = claimed_;
            j["claimed_by"] = std::to_string(reinterpret_cast<std::uintptr_t>(claimed_by_));
            return j;
        };

    private:
        Point2D<int> location_;
        bool claimed_ = false;
        const void* claimed_by_ = nullptr;
};

// Class representing the environment
// Environments are always time-invariant. All obstacles are static.
class Environment{
    public:
        Environment();
        Environment(int nb_cell_rows, int nb_cell_cols, double cell_width, 
                    double cell_height);

        
        // Validity checks
        bool isValidCell(Point2D<int> cell) const;
        bool isValidCell(int x, int y) const;
        bool isValidPosition(Point2D<double> pos) const;
        bool isValidVehiclePosition(Point2D<double> pos, double vehicle_width, 
                                    double vehicle_length, 
                                    double margin) const;

        bool IsFree(Point2D<double> const &pos) const {
            if (!isValidPosition(pos)){ return false;}
            Point2D<int> cell = pos.ConvertWorldToCell(cell_width_, cell_height_);
            return IsFree(cell);
        }
        bool IsFree(Point2D<int> const &cell) const;
        bool IsFree(int x, int y) const;
        bool IsClaimable(int x, int y) const;
        bool IsClaimedByClaimingObject(int x, int y) const;

        // Environment operations
        void DeleteCell(Point2D<int> cell);
        void AddCell(Point2D<int> cell);
        void AddObstacle(Point2D<int> cell);
        void RemoveObstacle(Point2D<int> cell);
        void AddVirtualObstacle(Point2D<int> cell);
        void RemoveVirtualObstacle(Point2D<int> cell);
        void ClearAllObstacles();
        void AddRandomObstacles(double obstacle_probability);
        void AddClaimableDestination(
            const std::string &name, const Point2D<int> location);
        Point2D<double> GetClaimableDestinationLocation(
            const std::string &name) const;
        bool ClaimDestination(
            const std::string &name, const void* caller);
        void ReleaseDestination(
            const std::string &name, const void* caller);
        void SetClaimingObject(const void* claiming_object) {
            claiming_object_ = claiming_object;
        }
        void ClearClaimingObject() { claiming_object_ = nullptr;}
        const void* GetClaimingObject() const { return claiming_object_;}
        const void* GetObjectClaimingDestination(const std::string &destination) const;
        std::string GetNearestFreeClaimableDestination(const Point2D<double> &pos) const;

        // Moving obstacle operations
        class MovingObstacleOperationsToken {
            friend class DynamicSimulator;
            private: MovingObstacleOperationsToken() {};
        };
        void AddMovingObstacle(MovingObstacleOperationsToken&, 
                               const Point2D<int> &cell);
        void RemoveMovingObstacle(MovingObstacleOperationsToken&, 
                                  const Point2D<int> &cell);
        void ClearAllMovingObstacles();

        // Function to perform a breadth-first search in the environment
        std::vector<Point2D<int>> PerformBreadthFirstSearch(
            const Point2D<int> &start, const Point2D<int> &dest, 
            const Point2D<double> &true_dest) const;

        // Return the cells that are occupied by the footprint of the vehicle
        std::unordered_set<Point2D<int>, Point2DHash<int>> GetOccupiedFootprintCells(
            const Point2D<double> &start, const double &vehicle_width,
            const double &vehicle_length, const double &margin) const;

        // Function to print the occupancy grid
        friend std::ostream& operator<<(std::ostream &out, 
                                        Environment const &environment);
        bool operator==(const Environment &other) const;

        // basic getters
        int NbCellRows() const { return nb_cell_rows_;};
        int NbCellCols() const { return nb_cell_cols_;};
        double CellWidth() const { return cell_width_;};
        double CellHeight() const { return cell_height_;};
        int GetVersion() const { return version_;};
        
        CellOccupancy GetOccupancy(Point2D<int> cell) const;
        CellOccupancy GetOccupancy(int x, int y) const;

        void GetRandomFreeVehiclePosition(Point2D<double> &pos,
                                          double vehicle_width,
                                          double vehicle_height,
                                          double margin) const;
        Point2D<double> GetRandomFreeVehiclePosition(double vehicle_width,
                                double vehicle_height, double margin) const{
            Point2D<double> pos;
            GetRandomFreeVehiclePosition(pos, vehicle_width, vehicle_height, margin);
            return pos;
        };

        void GetRandomFreeCellPosition(Point2D<double> &pos) const;
        Point2D<double> GetRandomFreeCellPosition() const{
            Point2D<double> pos;
            GetRandomFreeCellPosition(pos);
            return pos;
        };
        Point2D<int> GetRandomFreeCellPositionAtEnvironmentEdge() const;

        json ToJson() const;

        json ClaimableDestinationsToJson() const;

    private:
        // Function to be called whenever a modification is made to the 
        // environment
        void UpdateVersion(){ version_++;};

        int nb_cell_rows_;
        int nb_cell_cols_;
        double cell_width_;
        double cell_height_;

        // Occupancy grid representing the environment
        std::vector<std::vector<CellOccupancy>> occupancy_grid_;
        std::map<std::string, ClaimableDestination> claimable_destinations_;

        // version tracker such that CorridorSequence knows if it needs 
        // updating
        int version_ = 0;

        // pointer to an object that might claim destinations. If this pointer
        // points to an actual object, the environment will consider the
        // destinations claimed by that object as free (and all others as
        // occupied)
        const void* claiming_object_ = nullptr;
};

#endif