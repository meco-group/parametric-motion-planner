#ifndef __PARAMETRIZATION__
#define __PARAMETRIZATION__

#include <vector>

#include "helper_types.hpp"
#include "corridor.hpp"

class Parametrization{
    public:
        Parametrization(CorridorSequence const &corridor_sequence,
                        Parameters const &params);

        void UpdateParametrization();

        // basic getters
        int MaxNbCorridors() const {return max_nb_corridors_;};
        int NbCorridors() const { return corridor_sequence_.NbCorridors();};
        Point2D<double> GetWaypoint(int idx) const { 
            return waypoints_[idx].Copy();};
        Point2D<double> GetWaypointOffset(int idx) const {
            return waypoint_offsets_[idx].Copy();};
        bool IsWaypointMovable(int idx) const { 
            return movable_waypoints_[idx];};
        double GetAlphaX(int idx) const { return alpha_x_[idx];};
        double GetAlphaY(int idx) const { return alpha_y_[idx];};

        // printing overload
        friend std::ostream& operator<<(std::ostream &out, 
                            Parametrization const &parametrization);

    private:
        // waypoint with index waypoint_idx is in the overlapping region of 
        // corridor waypoint_idx - 1 and corridor waypoint_idx
        void ComputeSingleWaypoint(int waypoint_idx, bool second_sweep=false);

        void ComputeCandidateWaypoints();

        // Function that applies the heuristic to select the best waypoint
        // and the acceleration
        // This function only considers the point in the overlap between
        // corridor waypoint_idx - 1 and corridor waypoint_idx
        void ApplyHeuristic(int waypoint_idx, bool second_sweep=false);

        // Update the parametrization based on line of sights
        void UpdateParametrizationWithLineOfSight(int waypoint_idx);

        bool LineOfSightInCorridors(Point2D<double> const &point1, 
                                    Point2D<double> const &point2, 
                                    Corridor const &corridor1,
                                    Corridor const &corridor2) const;

        const CorridorSequence& corridor_sequence_; // Reference to the corridor sequence object
        const Parameters& params_;

        const int max_nb_corridors_;

        std::vector<double> alpha_x_;
        std::vector<double> alpha_y_;
        std::vector<Point2D<double>> waypoints_;
        std::vector<Point2D<double>> waypoint_offsets_;
        std::vector<bool> movable_waypoints_;
        
        std::vector<std::vector<double>> t_x_;
        std::vector<std::vector<double>> t_y_;

        // scratch space
        Corridor overlap_;
        Corridor curr_corridor_;
        Corridor next_corridor_;

        std::vector<Point2D<double>> candidate_waypoints_;
        std::vector<double> candidate_alpha_x_;
        std::vector<double> candidate_alpha_y_;
        std::vector<bool> candidate_valid_;
        Point2D<double> next_point_;
        Point2D<double> curr_point_;
        Point2D<double> prev_point_;
};

#endif