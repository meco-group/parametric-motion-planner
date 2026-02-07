#ifndef __DYNANMIC_SIMULATOR__
#define __DYNANMIC_SIMULATOR__

#include "environment.hpp"
#include "motion_planner.hpp"
#include "helper_types.hpp"
#include "moving_obstacle.hpp"


// Class to simulate a dynamic environment
// It will automatically update the static environment based on the location
// of moving obstacles and will replan if needed.
class DynamicSimulator{
    public:
        DynamicSimulator(Environment &environment, MotionPlanner &motion_planner):
                environment_(environment), motion_planner_(motion_planner){
            if (!(motion_planner.GetEnvironment() == environment)){
                throw std::runtime_error(
                    "The environment of the motion planner must be the same as the "
                    "environment of the dynamic simulator");
            }
        };

        bool Plan(const Point2D<double> &start, const Point2D<double> &dest, 
                  const Point2D<double> &start_vel);

        void MoveDestination(Point2D<double> start, Point2D<double> start_vel,
                             int number_of_destination_switches);

        void Reset();

        void AddMovingObstacle(std::shared_ptr<MovingObstacle> obstacle);
        void RemoveMovingObstacle(std::shared_ptr<MovingObstacle> obstacle);

        json ToJson() const;
        void DumpToJson(const std::string &filename) const;
        void PrintMotionPlannerLog() const { motion_planner_.PrintLog();};

    private:
        void UpdateEnvironment();

        void UpdateMovingObstacles(double dt);

        bool CheckReplanTrigger();

        void PrintPythonImplementationInfo(Point2D<double>& replan_position, 
                                           Point2D<double>& replan_velocity,
                                           const Point2D<double>& dest, 
                                           const Parameters& params) const;

        Environment& environment_;
        MotionPlanner& motion_planner_;

        Trajectory travelled_trajectory_;
        std::vector<double> replanning_times_ = {};
        std::vector<Trajectory> previous_trajectories_ = {};
        std::vector<CorridorSequence> previous_corridor_sequences_ = {}; 
        std::vector<json> previous_environments_ = {};

        // List of movable obstacles
        std::unordered_set<std::shared_ptr<MovingObstacle>> moving_obstacles_;
        std::unordered_map<std::shared_ptr<MovingObstacle>, 
                           std::unordered_set<Point2D<int>, Point2DHash<int>>>
            cells_covered_by_moving_obstacles_;
};

#endif