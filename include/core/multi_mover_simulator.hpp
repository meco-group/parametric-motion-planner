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
        void SetFinalDestination(const Point2D<double>& final_dest);

        // Basic Getters
        const Point2D<double>& GetFinalDestination() const;
        const Point2D<double> GetCurrentPosition() const;
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

        // Environment my_env_;
        // Parameters my_params_;
        MotionPlanner planner_;
        AgentState state_;
        Point2D<double> final_dest_;

        // attributes related to waiting
        Point2D<double> waiting_position_;
        std::shared_ptr<Agent> blocking_agent_;
        int blocking_agent_idx_ = -1;
        Corridor intersection_;

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
        int replanning_frequency_ = 1; // in number of time-steps
        int replanning_step_counter_ = 0;
        bool wait_for_clear_intersection_ = false;
        double collision_check_margin_ = 0.01;

        // scratch space
        double t;
        Corridor footprint_;
        Corridor blocking_footprint_;
        Corridor o;
};

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
                            std::vector<Point2D<double>> starting_positions,
                            std::vector<Point2D<double>> final_destinations);

        void InstructAgentToDestination(int agent_idx, 
                                        const Point2D<double> final_dest);

        void SimulateSteps(int nb_steps, bool stop_when_all_idling=true);

        void DumpToJson(std::string const &filename) const;

    private:
        // Update positions of all agents for a single time-step
        void UpdateSingleStep();

        // If any new trajectories introduce collisions
        // (already existing trajectories are assumed to be collision-free)
        // Deal with the collision by instructing agents to wait
        bool ProcessPotentialNewCollsions(std::vector<bool>& new_trajectories);

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

        Environment& env_;
        std::vector<const Parameters*> params_;
        std::vector<std::shared_ptr<Agent>> agents_;

        // simulation attributes
        int nb_simulated_samples_ = 0;
        double simulation_time_step_ = 0.01;

        // options
        double collision_check_margin_ = 0.01;

        // stored information
        std::vector<IntersectionLog> intersection_logs_;
};


#endif