#ifndef __DYNAMIC_INTERSECTION_MANAGER_HPP__
#define __DYNAMIC_INTERSECTION_MANAGER_HPP__

#include "motion_planner.hpp"

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

        int TimeToNbTimeSteps(double time);

        // find first vehicle leaving the intersection
        int GetFirstVehicleLeavingIntersection(double& first_leaving_time);
        double GetTimeLeavingIntersection(MotionPlanner& planner);

        MotionPlanner& planner_1_;
        Point2D<double> final_dest_1_;
        Trajectory travelled_trajectory_1_;
        std::vector<Trajectory> planned_trajectories_1;
        std::vector<CorridorSequence> planned_corridor_sequences_1;
        std::vector<double> planned_times_1;
        Point2D<double> latest_simulated_pos_1_;
        Point2D<double> latest_simulated_vel_1_;

        MotionPlanner& planner_2_;
        Point2D<double> final_dest_2_;
        Trajectory travelled_trajectory_2_;
        std::vector<Trajectory> planned_trajectories_2;
        std::vector<CorridorSequence> planned_corridor_sequences_2;
        std::vector<double> planned_times_2;
        Point2D<double> latest_simulated_pos_2_;
        Point2D<double> latest_simulated_vel_2_;

        Corridor intersection_;
        double additional_intersection_waiting_time_ = 0.0;

        int nb_simulated_samples_ = 0;
        double simulation_time_step_ = 0.01;
        bool intersection_present_;

};

#endif