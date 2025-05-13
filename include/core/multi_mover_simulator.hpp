#ifndef __MULTI_MOVER_SIMULATOR_HPP__
#define __MULTI_MOVER_SIMULATOR_HPP__

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"

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
        Agent(){Agent(nullptr);};
        Agent(MotionPlanner* planner);

        // Basic Setters
        void SetFinalDestination(const Point2D<double>& final_dest);

        // Basic Getters
        const Point2D<double>& GetFinalDestination() const;
        const Point2D<double> GetCurrentPosition() const;
        const AgentState& GetState() const;
        const Point2D<double>& GetWaitingPosition() const;
        const Agent& GetBlockingAgent() const;
        const Corridor& GetIntersection() const;
        int GetRemainingTimeSteps() const;
        const CorridorSequence& GetCorridorSequence() const {
            return planner_->GetCorridorSequence();
        }
        double GetTimeAtTimeStep(int future_time_step) const;
        
        // Instruct this agent to wait for another agent. This agent is assumed
        // to continue moving once the other agent has passed.
        void WaitForAgent(Agent* blocking_agent, Corridor& intersection);

        // Simulate a time-step and potentially update the current state
        void SimulateStep();

        // Update trajectory if needed
        bool UpdateTrajectory();

        // Get the vehicle footprint at a point in the future
        void GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin);

    private:
        void Plan();

        MotionPlanner* planner_;
        AgentState state_;
        Point2D<double> final_dest_;

        // attributes related to waiting
        Point2D<double> waiting_position_;
        Agent* blocking_agent_;
        Corridor intersection_;

        // Storing info
        Trajectory travelled_trajectory_;
        std::vector<Trajectory> planned_trajectories_;
        std::vector<CorridorSequence> planned_corridor_sequences_;
        std::vector<double> planned_times_;
};



// Class to simulate multiple movers preventing collisions
class MultiMoverSimulator {
    public:
        MultiMoverSimulator(std::vector<MotionPlanner>& planners);

        void InstructAgentToDestination(int agent_idx, 
                                        const Point2D<double>& final_dest);

        void SimulateSteps(int nb_steps);

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
        CollisionResolutionDecision GetIntersectionCase(int agent_idx_1, int agent_idx_2, 
                                             Corridor const &intersection);
        std::map<std::string, double> GetTimeEnteringAndLeavingIntersection(
            int agent_idx, Corridor const &intersection);

        // Check if agents are waiting for each other
        bool CheckIfDeadlockPresent();

        std::vector<Agent> agents_;

        // simulation attributes
        double current_time_;
        double simulation_time_step_ = 0.01;

        // options
        double collision_check_margin_ = 0.01;
};


#endif