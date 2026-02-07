#ifndef __DYNAMIC_INTERSECTION_MANAGER_HPP__
#define __DYNAMIC_INTERSECTION_MANAGER_HPP__

#include "motion_planner.hpp"

enum IntersectionCase{
    VEHICLE_1_MUST_WAIT,
    VEHICLE_2_MUST_WAIT,
    NO_TRUE_OVERLAP, // corridors overlap, but not both vehicles enter the intersecion
    NO_OVERLAP_IN_TIME, // vehicles only enter intersection at different times
    INVALID_CASE,
};

class DynamicIntersectionManager {
    public:
        DynamicIntersectionManager(MotionPlanner& planner1, 
                                   MotionPlanner& planner2);

        void SimulateSafely(const Point2D<double>& start1, 
            const Point2D<double>& dest1, const Point2D<double>& start2, 
            const Point2D<double>& dest2, const Point2D<double>& start_vel1, 
            const Point2D<double>& start_vel2);

        void DumpToJson(std::string const &filename) const;

    private:
        // store the intersection in intersection_ and return wether an
        // is actually present
        bool GetIntersection();

        // simulate some amount of time
        void Simulate(int nb_time_Steps);

        // simulate some amount of time but abort if safe trajectory is found
        // (return true in that case)
        bool SimulateImpatiently(int nb_time_Steps, int waiting_veh_idx);

        bool CheckForCollision();

        double GetTimeLeftToSimulate();
        int TimeToNbTimeSteps(double time);

        // find first vehicle leaving the intersection
        void GetIntersectionCase();
        std::map<std::string, double> GetTimeEnteringAndLeavingIntersection(MotionPlanner& planner);

        // attributes for first vehicle
        MotionPlanner& planner_1_;
        Point2D<double> final_dest_1_;
        Trajectory travelled_trajectory_1_;
        std::vector<std::shared_ptr<const Trajectory>> planned_trajectories_1;
        std::vector<CorridorSequence> planned_corridor_sequences_1;
        std::vector<double> planned_times_1;
        Point2D<double> latest_simulated_pos_1_;
        Point2D<double> latest_simulated_vel_1_;

        // attributes for second vehicle
        MotionPlanner& planner_2_;
        Point2D<double> final_dest_2_;
        Trajectory travelled_trajectory_2_;
        std::vector<std::shared_ptr<const Trajectory>> planned_trajectories_2;
        std::vector<CorridorSequence> planned_corridor_sequences_2;
        std::vector<double> planned_times_2;
        Point2D<double> latest_simulated_pos_2_;
        Point2D<double> latest_simulated_vel_2_;

        // attributes for the intersection
        Corridor intersection_;
        IntersectionCase intersection_case_;
        std::map<std::string, std::map<std::string, double>> intersection_times_;
        bool intersection_present_;
        Point2D<double> waiting_position_;

        // options
        double additional_intersection_waiting_time_ = 0.0;
        bool wait_for_clear_intersection_ = false;
        double collision_check_margin_ = 0.01;

        // scratch space
        int nb_simulated_samples_ = 0;
        double simulation_time_step_ = 0.01;

};

#endif