#ifndef __CORRIDOR__
#define __CORRIDOR__

#include <vector>

#include "exceptions.hpp"
#include "helper_types.hpp"
#include "environment.hpp"

class Environment;
class CorridorSequence;

const int MAX_NB_CORRIDORS = 50;
const int MAX_CORRIDOR_CELL_LENGTH = 20;

// Class to represent corridors
class Corridor{
    public:
        Corridor() : x_min_(0.0), x_max_(0.0), y_min_(0.0), y_max_(0.0){
            UpdateDirection();
        };

        Corridor(double x_min, double x_max, double y_min, double y_max){
            x_min_ = x_min; x_max_ = x_max;
            y_min_ = y_min; y_max_ = y_max;
            
            UpdateDirection();
        };

        // Compute overlapping region with another corridor and store it in 
        // overlap
        // Function returns a boolean indicating if there is overlap. If not, 
        // overlap is not modified
        bool GetOverlap(Corridor &other, Corridor &overlap);

        bool IsCompletelyWithin(Corridor* const &other) const;
        bool IsCompletelyWithin(Corridor* const &other1, 
                                Corridor* const &other2) const;

        // Getters
        double Xmin() const { return x_min_;};
        double Xmax() const { return x_max_;};
        double Ymin() const { return y_min_;};
        double Ymax() const { return y_max_;};
        Point2D<int> Direction() const { return direction_;};
        int GetCellLength(double cell_width, double cell_height) const {
            return std::max(std::abs(x_max_ - x_min_)/cell_width, 
                            std::abs(y_max_ - y_min_)/cell_height);
        };

        // Setters
        void SetXmin(double x_min){ x_min_ = x_min;};
        void SetXmax(double x_max){ x_max_ = x_max;};
        void SetYmin(double y_min){ y_min_ = y_min;};
        void SetYmax(double y_max){ y_max_ = y_max;};

        // printing
        friend std::ostream& operator<<(std::ostream &out, Corridor &corridor) {
            out << "[" << corridor.Xmin() << ", " << 
                    corridor.Xmax() << "] x [" << corridor.Ymin() << 
                    ", " << corridor.Ymax() << "]";
            return out;
        }

        // Copy function
        Corridor Copy() const { return Corridor(x_min_, x_max_, y_min_, y_max_);};

    private:
        void UpdateDirection();

        double x_min_;
        double x_max_;

        double y_min_;
        double y_max_;

        Point2D<int> direction_;
};

// Sequence of corridors
class CorridorSequence{
    public:
        CorridorSequence() : max_len_(MAX_NB_CORRIDORS), 
            sequence_(MAX_NB_CORRIDORS), 
            cells_along_corridor_(MAX_CORRIDOR_CELL_LENGTH){};
        CorridorSequence(int max_len) : max_len_(max_len), sequence_(max_len_),
            cells_along_corridor_(MAX_CORRIDOR_CELL_LENGTH){};

        // Initialize corridors from cell path
        void InitializeFromCellPath(std::vector<Point2D<int>> &path, 
                                    const double &cell_width, 
                                    const double &cell_height);

        // Inflate corridors as much as possible
        void InflateCorridors(Environment &environment);

        // Add a corridor to the sequence
        void AddCorridor(double x_min, double x_max, double y_min, double y_max);

        // Add a corridor to the sequence based on two cells
        void AddCorridorFromCells(Point2D<int> &start_cell, 
                                  Point2D<int> &end_cell,
                                  const double &cell_width, 
                                  const double &cell_height);

        // Remove a corridor from the sequence
        void RemoveCorridor(int idx);

        // Getters
        Corridor GetCorridor(int idx) const { return sequence_[idx].Copy();};
        int NbCorridors() const { return last_corridor_idx_;};

        // printing
        friend std::ostream& operator<<(std::ostream &out, CorridorSequence &sequence) {
            for (int i = 0; i < sequence.last_corridor_idx_; i++){
                out << i << ": " << sequence.sequence_[i] << std::endl;
            }
            return out;
        }

    private:
        void ClearAll(){ last_corridor_idx_ = 0;};

        bool GrowCorridorSideways(int idx, Environment &environment);

        int GetCellsOnLeftSide(int corridor_idx, Environment &environment);
        int GetCellsOnRightSide(int corridor_idx, Environment &environment);

        bool CheckCellsOnLeftSide(int corridor_idx, Environment &environment);
        bool CheckCellsOnrightSide(int corridor_idx, Environment &environment);

        bool RemoveIrrelevantCorridors();
        bool MergeCorridors();

        const int max_len_;                 // maximum length of the sequence
        std::vector<Corridor> sequence_;    // sequence of corridors

        int last_corridor_idx_ = 0;         // index of the last corridor in the sequence

        std::vector<Point2D<double>> cells_along_corridor_;
};

#endif