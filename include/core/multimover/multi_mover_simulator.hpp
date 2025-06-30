#ifndef __MULTI_MOVER_SIMULATOR_HPP__
#define __MULTI_MOVER_SIMULATOR_HPP__

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"
#include "core/multimover/agent.hpp"
#include "core/multimover/mover_task.hpp"

#include <vector>
#include <fstream>

enum CollisionResolutionDecision {
    AGENT_1_MUST_WAIT,
    AGENT_2_MUST_WAIT,
    NO_OVERLAP, // corridors overlap, but not both vehicles enter the intersection
    INVALID,
};


// Class to store intersection information (for debugging and visualization)
class IntersectionLog {
    public:
        IntersectionLog(int agent_idx_1, int agent_idx_2,
                        double time_entering, double time_leaving,
                        CorridorUnion intersection) :
            agent_idx_1_(agent_idx_1), agent_idx_2_(agent_idx_2),
            time_entering_(time_entering), time_leaving_(time_leaving),
            intersection_(intersection) {};

        bool operator==(const IntersectionLog& other) const {
            return (agent_idx_1_ == other.agent_idx_1_ &&
                    agent_idx_2_ == other.agent_idx_2_ &&
                    intersection_ == other.intersection_);};

        json ToJson() const {
            json j;
            j["agent_idx_1"] = agent_idx_1_;
            j["agent_idx_2"] = agent_idx_2_;
            j["time_entering"] = time_entering_;
            j["time_leaving"] = time_leaving_;
            j["intersection"] = intersection_.ToJson();
            return j;
        };

    private:
        int agent_idx_1_ = -1;
        int agent_idx_2_ = -1;
        double time_entering_ = -1;
        double time_leaving_ = -1;
        CorridorUnion intersection_;
};

// Class to log vehicle actions
class MultiMoverLogger {
    public:
        MultiMoverLogger() = default;

        void LogEvent(double time_stamp, const std::string& event) {
            time_stamps_.push_back(time_stamp);
            events_.push_back(event);
        };

        void PrintLog() const {
            std::ofstream log_file("multi_mover_log.txt");
            log_file << "MultiMover Logged actions:" << std::endl;
            for (size_t i = 0; i < time_stamps_.size(); ++i) {
                log_file << "[" << time_stamps_[i] << "] " 
                          << events_[i] << std::endl;
            }
            log_file.close();
        };

        json ToJson() const {
            json j;
            j["time_stamps"] = time_stamps_;
            j["events"] = events_;
            return j;
        };
    private:
        std::vector<double> time_stamps_;
        std::vector<std::string> events_;
};

// Class to simulate multiple movers preventing collisions
class MultiMoverSimulator {
    public:
        MultiMoverSimulator(Environment& environment,
                            std::vector<const Parameters*> params,
                            std::map<std::string, Point2D<int>> possible_destinations,
                            std::vector<std::string> starting_positions,
                            std::vector<MoverTask> tasks);

        void SimulateSteps(int nb_steps, bool stop_when_all_idling=true);
        void SimulateAllTasks();

        void DumpToJson(std::string const &filename) const;
        void PrintLog() const;

    private:
        bool SanityCheckOnPossibleDestinations(Environment& environment, 
            std::vector<const Parameters*> params, 
            std::map<std::string, Point2D<int>> possible_destinations) const;

        // Function to capture all that needs to happen to simulate a single
        // time-step (update trajectories, process potential collisions, 
        // check for deadlock and update agent positions/velocities)
        // returns whether deadlock has been detected
        bool SimulateSingleStep();

        // Update positions of all agents for a single time-step
        void UpdateSingleStep();

        // If any new trajectories introduce collisions
        // (already existing trajectories are assumed to be collision-free)
        // Deal with the collision by instructing agents to wait
        bool ProcessPotentialNewCollsions();

        bool CheckForCollision(int agent_idx_1, int agent_idx_2);
        std::pair<bool, bool> DealWithCollision(int agent_idx_1, int agent_idx_2);

        bool GetIntersection(int agent_idx_1, int agent_idx_2, 
                             CorridorUnion& intersection);
        CollisionResolutionDecision GetIntersectionCase(int agent_idx_1, 
            int agent_idx_2, CorridorUnion const &intersection);
        std::map<std::string, double> GetTimeEnteringAndLeavingIntersection(
            int agent_idx, CorridorUnion const &intersection);

        // Check if agents are waiting for each other
        bool CheckIfDeadlockPresent(std::vector<MoverTask>& deadlock_resolving_tasks);

        // Check if all agents are idling
        bool AllAgentsIdling() const;

        // 
        void ProcessPotentialNewTasks();

        // Check if all provided tasks are revealed
        bool AllTasksRevealed() const;

        Environment& env_;
        std::vector<const Parameters*> params_;
        std::vector<std::shared_ptr<Agent>> agents_;
        std::map<std::string, Point2D<int>> possible_destinations_;

        // simulation attributes
        int nb_simulated_samples_ = 0;
        double simulation_time_step_ = 0.01;
        std::vector<std::shared_ptr<MoverTask>> tasks_;
        int nb_consecutive_deadlocks_found_ = 0;

        // options
        double collision_check_margin_ = 0.01;

        // stored information
        std::vector<IntersectionLog> intersection_logs_;
        std::vector<json> claimed_destinations_info_;
        MultiMoverLogger logger_;
        std::vector<double> simulation_step_computation_times_;

        // scratch space
        std::vector<bool> new_trajectories_;
};


#endif