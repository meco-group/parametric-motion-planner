#ifndef __MULTI_MOVER_SIMULATOR_HPP__
#define __MULTI_MOVER_SIMULATOR_HPP__

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"

#include <vector>

enum AgentState {
    MOVING_TO_FINAL_DESTINATION,    // moving towards the final destination
    MOVING_TO_WAITING_POINT,        // moving towards the waiting point
    WAITING_AT_INTERSECTION,        // waiting for another agent to pass
    IDLING,                         // reached final destination
    READY_TO_PLAN,                  // attempting to plan to final destination
    FAILED_TO_PLAN_TO_DEST,         // planner failed, trying emergency stop
    FAILED_TO_PLAN_TO_WAITING_POINT,// planner failed, trying emergency stop
    WAITING_FOR_FREE_DESTINATION,   // destination is not free so needs to wait
};

enum CollisionResolutionDecision {
    AGENT_1_MUST_WAIT,
    AGENT_2_MUST_WAIT,
    NO_OVERLAP, // corridors overlap, but not both vehicles enter the intersection
    INVALID,
};

// Wrapper around a mover with some additional attributes needed for it to 
// function in a multi-mover environment
class Agent {
    public:
        // Agent() : Agent(0.01, Point2D<double>(0, 0)) {};
        Agent(Environment& env, const Parameters& params, 
              double collision_check_margin, 
              Point2D<double> starting_position);

        // Basic Setters
        // Instruct this agent to move to a destination. Returns false if the
        // destination could not be claimed.
        bool InstructToDestination(const std::string& destination_name,
                                   const Point2D<double>& final_dest);

        // Basic Getters
        const Point2D<double>& GetFinalDestination() const;
        const Point2D<double> GetCurrentPosition() const;
        bool Stationary() const { return curr_vel_.Norm() <= 1.0e-2; }
        const AgentState& GetState() const;
        const Point2D<double>& GetWaitingPosition() const;
        const Agent& GetBlockingAgent() const;
        int GetBlockingAgentIdx() const {
            return blocking_agent_idx_;
        }
        const Corridor& GetIntersection() const;
        int GetRemainingTimeSteps() const;
        const CorridorSequence& GetCorridorSequence() const {
            return planner_.GetCorridorSequence();
        }
        double GetTimeAtTimeStep(int future_time_step) const;
        
        // Instruct this agent to wait for another agent. This agent is assumed
        // to continue moving once the other agent has passed.
        void WaitForAgent(std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx_, 
                          Corridor& intersection);

        // Simulate a time-step and potentially update the current state
        void SimulateStep();

        // Update trajectory if needed
        bool UpdateTrajectory();

        // Get the vehicle footprint at a point in the future
        void GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin);

        json ToJson() const;

    private:
        void PlanToDestination(bool to_waiting_point=false);

        bool CheckForCollisionWithBlockingAgent();

        Environment& env_;
        MotionPlanner planner_;
        AgentState state_;
        Point2D<double> final_dest_;

        // attributes related to waiting for agent
        Point2D<double> waiting_position_;
        std::shared_ptr<Agent> blocking_agent_;
        int blocking_agent_idx_ = -1;
        Corridor intersection_;

        // attributes related to claiming a destination
        bool currently_claiming_ = false; // should actually always be true 
                                          // because either the agent is
                                          // moving towards a claimed 
                                          // destination or it is idling on a
                                          // claimed position
        std::string claimed_destination_name_;
        std::vector<std::string> destinations_to_be_released_;

        // Storing info
        //  every simulation step
        Trajectory travelled_trajectory_;
        std::vector<AgentState> travelled_states_;
        std::vector<int> travelled_blocking_agent_idx_;
        std::vector<Point2D<double>> travelled_final_destinations_;

        //  every time a trajectory is planned
        std::vector<Trajectory> planned_trajectories_;
        std::vector<CorridorSequence> planned_corridor_sequences_;
        std::vector<double> planned_times_;

        // current info
        Point2D<double> curr_pos_;
        Point2D<double> curr_vel_;
        Point2D<double> curr_acc_;
        double curr_time_;
        int nb_simulated_samples_ = 0;

        // options
        int replanning_frequency_ = 5; // in number of time-steps
        int replanning_step_counter_ = 0;
        bool wait_for_clear_intersection_ = false;
        double collision_check_margin_ = 0.01;

        // scratch space
        double t;
        Corridor footprint_;
        Corridor blocking_footprint_;
        Corridor o;
};

class MoverTask{
    public:
        MoverTask(int agent_idx, const std::string& destination, 
                  double time_to_reveal_task) :
            agent_idx_(agent_idx), destination_name_(destination), 
            time_to_reveal_task_(time_to_reveal_task) {};
        
        // check if now is the time to reveal the task
        bool RevealTask(double current_time) {
            if (current_time >= (time_to_reveal_task_ + task_delay_)){
                has_been_revealed_ = true;
                return true;
            }
            return false;
        }

        // if a task cannot be processed because the agent is not yet ready,
        // postpone the task
        void PostponeTask(double time_to_wait) {
            task_delay_ += time_to_wait;
            has_been_revealed_ = false;
        }

        // Basic getters
        int GetAgentIdx() const { return agent_idx_;}
        const std::string& GetDestinationName() const { return destination_name_; }
        bool HasBeenRevealed() const { return has_been_revealed_; }

        // printing
        friend std::ostream& operator<<(std::ostream& os, const MoverTask& task) {
            os << "MoverTask(agent_idx: " << task.agent_idx_ 
               << ", destination: " << task.destination_name_ 
               << ", time_to_reveal_task: " << task.time_to_reveal_task_ 
               << ", task_delay: " << task.task_delay_ 
               << ", has_been_revealed: " << task.has_been_revealed_ << ")";
            return os;
        }

    private:
        int agent_idx_ = -1;
        std::string destination_name_;
        double time_to_reveal_task_;
        double task_delay_ = 0;
        bool has_been_revealed_ = false;
};

// Class to store intersection information (for debugging and visualization)
class IntersectionLog {
    public:
        IntersectionLog(int agent_idx_1, int agent_idx_2,
                        double time_entering, double time_leaving,
                        Corridor intersection) :
            agent_idx_1_(agent_idx_1), agent_idx_2_(agent_idx_2),
            time_entering_(time_entering), time_leaving_(time_leaving),
            intersection_(intersection) {};

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
        Corridor intersection_;
};

// Class to simulate multiple movers preventing collisions
class MultiMoverSimulator {
    public:
        MultiMoverSimulator(Environment& environment,
                            std::vector<const Parameters*> params,
                            std::map<std::string, Point2D<int>> possible_destinations,
                            std::vector<std::string> starting_positions,
                            std::vector<MoverTask> tasks);

        void InstructAgentToDestination(int agent_idx, 
                                        const std::string& destination_name);

        void SimulateSteps(int nb_steps, bool stop_when_all_idling=true);
        void SimulateAllTasks();

        void DumpToJson(std::string const &filename) const;

    private:
        // Function to capture all that needs to happen to simulate a single
        // time-step (update trajectories, process potential collisions, 
        // check for deadlock and update agent positions/velocities)
        void SimulateSingleStep();

        // Update positions of all agents for a single time-step
        void UpdateSingleStep();

        // If any new trajectories introduce collisions
        // (already existing trajectories are assumed to be collision-free)
        // Deal with the collision by instructing agents to wait
        bool ProcessPotentialNewCollsions();

        bool CheckForCollision(int agent_idx_1, int agent_idx_2);
        void DealWithCollision(int agent_idx_1, int agent_idx_2);

        bool GetIntersection(int agent_idx_1, int agent_idx_2, 
                             Corridor& intersection);
        CollisionResolutionDecision GetIntersectionCase(int agent_idx_1, 
            int agent_idx_2, Corridor const &intersection);
        std::map<std::string, double> GetTimeEnteringAndLeavingIntersection(
            int agent_idx, Corridor const &intersection);

        // Check if agents are waiting for each other
        bool CheckIfDeadlockPresent();

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
        std::vector<MoverTask> tasks_;

        // options
        double collision_check_margin_ = 0.01;

        // stored information
        std::vector<IntersectionLog> intersection_logs_;
        std::vector<json> claimed_destinations_info_;

        // scratch space
        std::vector<bool> new_trajectories_;
};


#endif