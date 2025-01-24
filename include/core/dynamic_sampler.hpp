#ifndef __DYNAMIC_SAMPLER__
#define __DYNAMIC_SAMPLER__

#include "environment.hpp"
#include "motion_planner.hpp"
#include "trajectory.hpp"
#include "helper_types.hpp"

// Class to be used in the C++ sampler to provide samples for the buffer
// It will automatically take care of calling the planner to replan if needed
class DynamicSampler{
    public:
        DynamicSampler(Environment &environment, MotionPlanner &motion_planner):
                environment_(environment), motion_planner_(motion_planner){
            if (!(motion_planner.GetEnvironment() == environment)){
                throw std::runtime_error(
                    "The environment of the motion planner must be the same as the "
                    "environment of the dynamic simulator");
            }
        };

        bool GetSample(Point2D<double> &pos, Point2D<double> &vel, 
                       Point2D<double> &acc);

        void AddTrigger(std::function<bool()> trigger,
                        std::function<void()> action);
        void ClearTriggers(){ trigger_action_pairs_.clear();};
        void AddInitialization();
        void AddBasicReplanningTriggers();
        void RecordTrigger(std::string&& trigger_statement);

        void MoveDestinationDemo(int max_nb_replans, bool alternate_dest=false);

        // functions to be used in trigger and action specifications
        int GetCurrentNbSamples() const ;
        int GetCurrentRemainingNbSamples() const ;
        int GetTotalNbSamplesProvided() const { return nb_samples_provided_;};

        void SetInitialStart();
        void SetRandomDestination(){ motion_planner_.SetRandomDest();};
        void Plan();

        json ToJson() const;
        void DumpToJson(const std::string &filename) const;

        void FirstBufferDeployedToPLC();

    private:     
        void ExecuteTriggerActions();
        void Finish();

        Environment& environment_;
        MotionPlanner& motion_planner_;

        std::vector<std::pair<std::function<bool()>, std::function<void()>>> 
            trigger_action_pairs_;

        // logging attributes
        double curr_time_ = 0.0;
        Point2D<double> curr_pos_;
        Point2D<double> curr_vel_;
        int nb_replans_ = 0;
        int nb_samples_provided_ = 0;
        bool finished_ = false;
        std::vector<std::string> events_;

        bool record_sample_time_ = 1;
        std::vector<double> ms_to_retrieve_sample_ = {};
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point stop = std::chrono::high_resolution_clock::now();

        bool record_sample_request_times_ = 1;
        std::vector<double> duration_of_request_since_first_sample_in_ms_ = {};
        std::chrono::high_resolution_clock::time_point time_of_first_sample_request_ = std::chrono::high_resolution_clock::now();
        double mover_started_moving_since_first_sample_in_ms_;

        std::vector<double> replanning_times_ = {};
        std::vector<Trajectory> previous_trajectories_ = {};
        std::vector<CorridorSequence> previous_corridor_sequences_ = {};
        std::vector<json> previous_environments_ = {};
        Trajectory travelled_trajectory_;
        
        int nb_samples_provided_since_last_replan_ = 0;
        bool last_planning_succeeded_ = true;
        double last_succesfull_planning_time_ = 0.0;


        std::vector<Point2D<int>> movable_obstacle_position_ = 
            {Point2D<int>(1, 5), Point2D<int>(3, 7), Point2D<int>(0, 7)};

        // temporary variables
        std::vector<Point2D<double>> travelled_positions_ = {};
};

#endif