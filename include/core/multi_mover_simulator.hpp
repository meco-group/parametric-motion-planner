#ifndef __MULTI_MOVER_SIMULATOR_HPP__
#define __MULTI_MOVER_SIMULATOR_HPP__

#include "core/motion_planner.hpp"

enum AgentState {
    MOVING_TO_FINAL_DESTINATION,    // moving towards the final destination
    MOVING_TO_WAITING_POINT,        // moving towards the waiting point
    WAITING_AT_INTERSECTION,        // waiting for another agent to pass
    IDLING,                         // reached final destination
    PLANNING_TO_FINAL_DESTINATION,  // attempting to plan to final destination
};

// Wrapper around a mover with some additional attributes needed for it to 
// function in a multi-mover environment
class Agent {
    public:
        Agent(MotionPlanner* planner);

        // Basic Setters
        void SetFinalDestination(const Point2D<double>& final_dest);

        // Basic Getters
        const Point2D<double>& GetFinalDestination() const;
        const Point2D<double>& GetCurrentPosition() const;
        const AgentState& GetState() const;
        const Point2D<double>& GetWaitingPosition() const;
        const Agent& GetBlockingAgent() const;
        const Corridor& GetIntersection() const;
        
        // Instruct this agent to wait for another agent. This agent is assumed
        // to continue moving once the other agent has passed.
        void WaitForAgent(const Point2D<double>& waiting_position, 
                          Agent& blocking_agent, Corridor& intersection);

        // Simulate a time-step and potentially update the current state
        void SimulateStep(Point2D<double>& pos, 
                          Point2D<double>& vel, 
                          Point2D<double>& acc, 
                          double& time);

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

        // Allow each agent that has no trajectory to plan now
        void UpdateTrajectoryPlans();

        // Check if any new trajectories introduce collisions
        // (already existing trajectories are assumed to be collision-free)
        // Deal with the collision by instructing agents to wait
        bool CheckForNewCollision();

        void DealWithCollision(int agent_idx_1, int agent_idx_2);

        // Check if agents are waiting for each other
        bool CheckIfDeadlockPresent();

        std::vector<Agent> agents_;

        // other simulation attributes
        double current_time_;
        double simulation_time_step_ = 0.01;
};


#endif