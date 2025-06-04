#ifndef __AGENT_HPP__
#define __AGENT_HPP__
#include "core/motion_planner.hpp"
#include "core/multimover/mover_task.hpp"

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

inline std::string AgentStateToString(AgentState s){
    if (s == MOVING_TO_FINAL_DESTINATION){
        return "MOVING_TO_FINAL_DESTINATION";
    } else if (s == MOVING_TO_WAITING_POINT){
        return "MOVING_TO_WAITING_POINT";
    } else if (s == WAITING_AT_INTERSECTION){
        return "WAITING_AT_INTERSECTION";
    } else if (s == IDLING){
        return "IDLING";
    } else if (s == READY_TO_PLAN){
        return "READY_TO_PLAN";
    } else if (s == FAILED_TO_PLAN_TO_DEST){
        return "FAILED_TO_PLAN_TO_DEST";
    } else if (s == FAILED_TO_PLAN_TO_WAITING_POINT){
        return "FAILED_TO_PLAN_TO_WAITING_POINT";
    } else if (s == WAITING_FOR_FREE_DESTINATION){
        return "WAITING_FOR_FREE_DESTINATION";
    } else {
        return "?";
    }
}


// Wrapper around a mover with some additional attributes needed for it to 
// function in a multi-mover environment
class Agent {
    public:
        // Agent() : Agent(0.01, Point2D<double>(0, 0)) {};
        Agent(int idx, Environment& env, const Parameters& params, 
              double collision_check_margin, 
              Point2D<double> starting_position);

        // Basic Setters
        // Instruct this agent to move to a destination. Returns false if the
        // destination could not be claimed.
        bool InstructToDestination(const std::string& destination_name,
                                   const Point2D<double>& final_dest,
                                   std::shared_ptr<MoverTask>& task);

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
        const void* GetPtrToObjectClaimingDestination() const;
        const CorridorUnion& GetIntersection() const;
        int GetRemainingTimeSteps() const;
        const CorridorSequence& GetCorridorSequence() const {
            return planner_.GetCorridorSequence();
        }
        double GetTimeAtTimeStep(int future_time_step) const;
        
        // Instruct this agent to wait for another agent. This agent is assumed
        // to continue moving once the other agent has passed.
        void WaitForAgent(std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx_, 
                          CorridorUnion const &intersection);

        // Simulate a time-step and potentially update the current state
        void SimulateStep();

        // Update trajectory if needed
        bool UpdateTrajectory();

        // Get the vehicle footprint at a point in the future
        void GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin);

        // check if the agent can avoid an intersection
        bool CanAvoidIntersection(const CorridorUnion& intersection) const;

        json ToJson() const;

    private:
        void PlanToDestination(bool to_waiting_point=false);

        bool CheckForCollisionWithBlockingAgent();

        Environment& env_;
        MotionPlanner planner_;
        AgentState state_;
        Point2D<double> final_dest_;
        const int my_agent_idx_ = -1;

        // attributes related to waiting for agent
        Point2D<double> waiting_position_;
        std::shared_ptr<Agent> blocking_agent_;
        int blocking_agent_idx_ = -1;
        CorridorUnion intersection_;

        // attributes related to claiming a destination
        bool currently_claiming_ = false; // should actually always be true 
                                          // because either the agent is
                                          // moving towards a claimed 
                                          // destination or it is idling on a
                                          // claimed position
        std::string claimed_destination_name_;
        std::vector<std::string> destinations_to_be_released_;
        const void* ptr_to_object_claiming_destination_ = nullptr; // the agent that is currently claiming our destination

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

        // mover task logging
        std::shared_ptr<MoverTask> curr_task_; // the current task that is being executed by this agent

        // current info
        Point2D<double> curr_pos_;
        Point2D<double> curr_vel_;
        Point2D<double> curr_acc_;
        double curr_time_;
        int nb_simulated_samples_ = 0;

        // options
        int replanning_frequency_ = 15; // in number of time-steps
        int replanning_step_counter_ = 0;
        bool wait_for_clear_intersection_ = false;
        double collision_check_margin_ = 0.01;
        bool jit_planner_ = true;

        // scratch space
        double t;
        Corridor footprint_;
        Corridor blocking_footprint_;
        Corridor o;
};

#endif