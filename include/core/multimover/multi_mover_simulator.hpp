#ifndef __MULTI_MOVER_SIMULATOR_HPP__
#define __MULTI_MOVER_SIMULATOR_HPP__

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"
#include "core/multimover/agent.hpp"
#include "core/multimover/mover_task.hpp"
#include "core/multimover/profiler.hpp"

#include <vector>
#include <fstream>
#include <mutex>

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
                            std::vector<Parameters*> params,
                            std::map<std::string, Point2D<int>> possible_destinations,
                            std::vector<std::string> starting_positions,
                            std::vector<MoverTask> tasks);

        // returns whether all tasks have been completed
        bool SimulateSteps(int nb_steps, bool stop_when_all_idling=true);
        void SimulateAllTasks();

        int GetNbAgents() const {return agents_.size();};
        bool AllTasksCompleted() const;

        void UpdateSetpoints(std::vector<Point2D<double>>& curr_pos,
                             std::vector<Point2D<double>>& curr_vel,
                             std::vector<Point2D<double>>& curr_acc) const;

        void DumpToJson(std::string const &filename) const;
        void PrintProfilerInfo() const;
        void PrintLog() const;

        void SetPerformanceMode(bool performance_mode);
        void SetIgnoreWaitingPoints(bool ignore_waiting_points){
            ignore_waiting_points_ = ignore_waiting_points;
        };
        void SetIgnoreCollisions(bool ignore_collisions){
            ignore_collisions_ = ignore_collisions;
        };
        void SetMaxUpdateTrajectoryTime(double max_time_in_ms){
            max_time_update_trajectory_in_ms_ = max_time_in_ms;
        };
        void PrepareOptiInstances(){
            int i = 0;
            for (auto& agent : agents_){
                std::cout << "agent " << i << ": preparing opti instance..." << std::endl;
                agent->PrepareOptiInstances();
                i++;
            }
        };
        void SetPlanWhileMoving(bool set){
            for (auto& agent : agents_){
                agent->SetPlanWhileMoving(set);
            }
        }
        void SetMaxNbCorridorGrowingIterations(int n){
            for (auto& agent : agents_){
                agent->SetMaxNbCorridorGrowingIterations(n);
            }
        }

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

        // Check if the agent collides with any virtual trajectory
        void ProcessPotentialVirtualCollision(int agent_idx);

        bool CheckForCollision(int agent_idx_1, int agent_idx_2);
        std::pair<bool, bool> DealWithCollision(int agent_idx_1, int agent_idx_2);

        bool CheckForCollisionBinary(int agent_idx_1, int agent_idx_2, int start_idx=0, int stop_idx=-1);

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

        void AddPrioritizedAgent(std::shared_ptr<VirtualAgent>& agent);

        Environment& env_;
        std::vector<Parameters*> params_;
        std::vector<std::shared_ptr<Agent>> agents_;
        std::vector<std::shared_ptr<VirtualAgent>> prioritized_agents_;
        std::map<std::string, Point2D<int>> possible_destinations_;

        // simulation attributes
        int nb_simulated_samples_ = 0;
        double simulation_time_step_ = 0.01;
        std::vector<std::shared_ptr<MoverTask>> tasks_;
        int nb_consecutive_deadlocks_found_ = 0;
        std::vector<std::unordered_set<int>> deadlock_cycles_;
        std::vector<std::map<int, bool>> deadlocked_agents_attempted_resolution_;
        int nb_simulation_steps_since_last_attempted_resolution_ = 0;

        std::mutex mutex_;

        // profiling of simulation
        constexpr static bool perform_profiling_ = true;
        std::map<std::string, Profiler> profilers_;
        std::vector<int> nb_new_trajectories_;

        // options
        double collision_check_margin_ = 0.01;
        bool write_simulation_progress_to_file_ = false;
        bool performance_mode_ = false;
        bool silent_mode_ = false;
        double max_time_update_trajectory_in_ms_ = 1.0e10; //TODO: use this value to limit computation time in a single simulation step
        int next_agent_to_be_allowed_to_plan = 0;
        
        bool ignore_waiting_points_ = false;
        bool ignore_collisions_ = false; // only to be used for some testing

        // stored information
        std::vector<IntersectionLog> intersection_logs_;
        std::vector<json> claimed_destinations_info_;
        MultiMoverLogger logger_;
        std::vector<double> simulation_step_computation_times_;

        // scratch space
        std::vector<bool> new_trajectories_;
};


#endif