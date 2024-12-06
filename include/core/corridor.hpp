#ifndef __CORRIDOR__
#define __CORRIDOR__

#include <vector>
#include <nlohmann/json.hpp>
#include "exceptions.hpp"
#include "helper_types.hpp"
#include "environment.hpp"
#include "parameters.hpp"
#include "casadi/casadi.hpp"

using json = nlohmann::json;

class Environment;
class CorridorSequence;

const int MAX_NB_CORRIDORS = 20;
const int MAX_CORRIDOR_CELL_LENGTH = 25;

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
        bool ContainsPoint(Point2D<double> const &point) const;
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
                    std::abs(x_max_ - x_min_ + 1.0e-3)/cell_width :
                    std::abs(y_max_ - y_min_ + 1.0e-3)/cell_height;
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
            // out << "[" << corridor.Xmin() << ", " << 
            //         corridor.Xmax() << "] x [" << corridor.Ymin() << 
            //         ", " << corridor.Ymax() << "]";
            out << "[" << corridor.Xmin() << ", " << 
                    corridor.Xmax() << ", " << corridor.Ymin() << 
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

using namespace casadi;
class Corridor_MX{
    public:
        Corridor_MX() : x_min_(0.0), x_max_(0.0), y_min_(0.0), y_max_(0.0){};

        Corridor_MX(MX x_min, MX x_max, MX y_min, MX y_max){
            x_min_ = x_min; x_max_ = x_max;
            y_min_ = y_min; y_max_ = y_max;
        };

        // Getters
        MX Xmin() const { return x_min_;};
        MX Xmax() const { return x_max_;};
        MX Ymin() const { return y_min_;};
        MX Ymax() const { return y_max_;};
        MX Width() const { return x_max_ - x_min_;};
        MX Height() const { return y_max_ - y_min_;};

        // Setters
        void SetXmin(MX x_min){ x_min_ = x_min;};
        void SetXmax(MX x_max){ x_max_ = x_max;};
        void SetYmin(MX y_min){ y_min_ = y_min;};
        void SetYmax(MX y_max){ y_max_ = y_max;};

        // Copy function
        Corridor_MX Copy() const { return Corridor_MX(x_min_, x_max_, y_min_, y_max_);};

        // take the values of another corridor
        void CopyValues(Corridor_MX const &other) {
            SetXmin(other.Xmin()); SetXmax(other.Xmax());
            SetYmin(other.Ymin()); SetYmax(other.Ymax());
        }

    private:
        MX x_min_;
        MX x_max_;
        MX y_min_;
        MX y_max_;
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

        // Function to check if a given point is within the current corridor
        // sequence
        bool ContainsPoint(Point2D<double> const &point) const;

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
        bool SequenceAvailable() const { return sequence_available_;};
        int GetVersion() const { return version_;};

        std::vector<Point2D<double>> GetCorridorOverlapCenters() const;

        // printing
        friend std::ostream& operator<<(std::ostream &out, CorridorSequence const &sequence) {
            for (int i = 0; i < sequence.nb_of_corridors_; i++){
                out << i << ": " << sequence.sequence_[i] << std::endl;
            }
            return out;
        }

        json ToJson() const;

    private:
        void UpdateVersion(){version_++;};

        void ClearAll(){ nb_of_corridors_ = 0; UpdateVersion();};

        void AddInitialFootprint(std::vector<Point2D<int>> &path) const;
        void AddFinalFootprint(std::vector<Point2D<int>> &path) const;

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
        bool use_smart_update_ = false;
        int latest_envrionment_version_ = -1; // version of the environment when the sequence was last updated

        const Parameters& params_;          // Reference to the parameters object

        Point2D<double> start_;
        Point2D<double> dest_;
        Point2D<double> start_vel_;

        const int max_len_;                 // maximum length of the sequence
        std::vector<Corridor> sequence_;    // sequence of corridors
        bool sequence_available_;

        int nb_of_corridors_ = 0;         // index of the last corridor in the sequence

        int version_ = 0;               // version tracker such that the parametrization knows if it needs updating

        // scratch space
        std::vector<Point2D<double>> cells_along_corridor_;
};

#endif