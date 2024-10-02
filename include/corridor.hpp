#ifndef __CORRIDOR__
#define __CORRIDOR__

#include <vector>
#include <nlohmann/json.hpp>
#include "exceptions.hpp"
#include "helper_types.hpp"
#include "environment.hpp"
#include "parameters.hpp"

using json = nlohmann::json;

class Environment;
class CorridorSequence;

const int MAX_NB_CORRIDORS = 20;
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
        bool GetOverlap(Corridor &other, Corridor &overlap) const;

        bool IsCompletelyWithin(Corridor* const &other) const;
        bool IsCompletelyWithin(Corridor* const &other1, 
                                Corridor* const &other2) const;
        bool ContainsVehicle(const Point2D<double> &vehicle_position, 
                             const Parameters &params) const;

        // Getters
        double Xmin() const { return x_min_;};
        double Xmax() const { return x_max_;};
        double Ymin() const { return y_min_;};
        double Ymax() const { return y_max_;};
        double Width() const { return x_max_ - x_min_;};
        double Height() const { return y_max_ - y_min_;};
        Point2D<int> Direction() const { return direction_;};
        int GetCellLength(double cell_width, double cell_height) const {
            return direction_.y() == 0 ? 
                    std::abs(x_max_ - x_min_)/cell_width :
                    std::abs(y_max_ - y_min_)/cell_height;
        };
        Point2D<double> GetCenter() const {
            return Point2D<double>((x_min_ + x_max_)/2, (y_min_ + y_max_)/2);
        };
        void GetCenter(Point2D<double> &center) const {
            center.SetX((x_min_ + x_max_)/2);
            center.SetY((y_min_ + y_max_)/2);
        };

        // Setters
        void SetXmin(double x_min){ x_min_ = x_min; UpdateDirection();};
        void SetXmax(double x_max){ x_max_ = x_max; UpdateDirection();};
        void SetYmin(double y_min){ y_min_ = y_min; UpdateDirection();};
        void SetYmax(double y_max){ y_max_ = y_max; UpdateDirection();};

        // printing
        // friend std::ostream& operator<<(std::ostream &out, Corridor &corridor) {
        //     out << "[" << corridor.Xmin() << ", " << 
        //             corridor.Xmax() << "] x [" << corridor.Ymin() << 
        //             ", " << corridor.Ymax() << "]";
        //     return out;
        // }
        friend std::ostream& operator<<(std::ostream &out, Corridor corridor) {
            out << "[" << corridor.Xmin() << ", " << 
                    corridor.Xmax() << "] x [" << corridor.Ymin() << 
                    ", " << corridor.Ymax() << "]";
            return out;
        }

        // Copy function
        Corridor Copy() const { return Corridor(x_min_, x_max_, y_min_, y_max_);};

        // take the values of another corridor
        void CopyValues(Corridor const &other) {
            SetXmin(other.Xmin()); SetXmax(other.Xmax());
            SetYmin(other.Ymin()); SetYmax(other.Ymax());
        }

        json ToJson() const {
            return json{{"x_min", x_min_}, {"x_max", x_max_}, 
                        {"y_min", y_min_}, {"y_max", y_max_}};};

        void FlipDirection(){
            direction_ = Point2D<int>(direction_.y(), direction_.x());
        }

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
        CorridorSequence(Environment const &environment,
                         Parameters const &params) : 
            CorridorSequence(environment, params, MAX_NB_CORRIDORS) {};
        CorridorSequence(Environment const &environment, 
                         Parameters const &params, int max_len) : 
            environment_(environment),
            params_(params),
            max_len_(max_len), sequence_(max_len_),
            cells_along_corridor_(MAX_CORRIDOR_CELL_LENGTH){};

        // Create access token such that only the motion planner can update
        // the corridor sequence
        class UpdateToken{
            friend class MotionPlanner; 
            private: UpdateToken() {};
        };

        // Initialize corridors from cell path
        void UpdateSequence(Point2D<double> const &start,
                            Point2D<double> const &dest,
                            Point2D<double> const &start_vel,
                            Parameters const &params,
                            UpdateToken const &token);

        // Getters
        int MaxNbCorridors() const { return max_len_;};
        Corridor GetCorridor(int idx) const;
        void GetCorridor(int idx, Corridor &corridor) const;
        int NbCorridors() const { return nb_of_corridors_;};

        Point2D<double> GetStart() const { return start_.Copy();};
        Point2D<double> GetStartVel() const { return start_vel_.Copy();};
        Point2D<double> GetDest() const { return dest_.Copy();};
        void GetStart(Point2D<double> &point) const;
        void GetStartVel(Point2D<double> &point) const;
        void GetDest(Point2D<double> &point) const;

        // printing
        friend std::ostream& operator<<(std::ostream &out, CorridorSequence &sequence) {
            for (int i = 0; i < sequence.nb_of_corridors_; i++){
                out << i << ": " << sequence.sequence_[i] << std::endl;
            }
            return out;
        }

        json ToJson() const;

    private:
        void ClearAll(){ nb_of_corridors_ = 0; made_change_ = false;};

        void AddInitialFootprint(std::vector<Point2D<int>> &path);
        void AddFinalFootprint(std::vector<Point2D<int>> &path);

        // Inflate corridors as much as possible
        void InflateCorridors();

        // Add a corridor to the sequence
        void AddCorridor(double x_min, double x_max, double y_min, double y_max);

        // Add a corridor to the sequence based on two cells
        void AddCorridorFromCells(Point2D<int> &start_cell, 
                                  Point2D<int> &end_cell);

        // Remove a corridor from the sequence
        void RemoveCorridor(int idx);

        bool GrowCorridorSideways(int idx);

        int GetCellsOnLeftSide(int corridor_idx);
        int GetCellsOnRightSide(int corridor_idx);

        bool CheckCellsOnLeftSide(int corridor_idx);
        bool CheckCellsOnrightSide(int corridor_idx);

        bool RemoveIrrelevantCorridors();
        bool MergeCorridors();

        const Environment& environment_;    // Reference to the environment object
        const Parameters& params_;          // Reference to the parameters object

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;

        const int max_len_;                 // maximum length of the sequence
        std::vector<Corridor> sequence_;    // sequence of corridors

        int nb_of_corridors_ = 0;         // index of the last corridor in the sequence

        bool made_change_ = false;          // flag to indicate if a change was made (since last parametrization update)

        // scratch space
        std::vector<Point2D<double>> cells_along_corridor_;
};

#endif