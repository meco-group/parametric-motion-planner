#ifndef __AGENT_HPP__
#define __AGENT_HPP__
#include "core/motion_planner.hpp"
#include "core/multimover/mover_task.hpp"
#include "core/multimover/profiler.hpp"

enum AgentState {
    MOVING_TO_FINAL_DESTINATION,    // moving towards the final destination
    MOVING_TO_WAITING_POINT,        // moving towards the waiting point
    WAITING_AT_INTERSECTION,        // waiting for another agent to pass
    IDLING,                         // reached final destination
    READY_TO_PLAN,                  // attempting to plan to final destination
    FAILED_TO_PLAN_TO_DEST,         // planner failed, trying emergency stop
    FAILED_TO_PLAN_TO_WAITING_POINT,// planner failed, trying emergency stop
    WAITING_FOR_FREE_DESTINATION,   // destination is not free so needs to wait
    WAITING_FOR_PRIORITIZED_VEHICLE,// some vehicle is in an implicit deadlock, so we wait until it starts to move
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
    } else if (s == WAITING_FOR_PRIORITIZED_VEHICLE){
        return "WAITING_FOR_PRIORITIZED_VEHICLE";
    } else {
        return "?";
    }
}

// Light-weight class for an agent. This is used by the MultiMoverSimulator to
// store priorizited trajectories (= trajectories that keep being rejeceted)
class VirtualAgent {
    public:
        VirtualAgent(int idx, const Parameters& params, 
                     double collision_check_margin, std::shared_ptr<const Trajectory> trajectory);

        std::shared_ptr<const Trajectory> GetTrajectory() const {return prioritized_trajectory_;};
        int GetAgentIdx() const {return my_agent_idx_;};

    private:
        std::shared_ptr<const Trajectory> prioritized_trajectory_;
        Parameters params_;
        const int my_agent_idx_ = -1;
        double collision_check_margin_ = 0.01;
};



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

        // Deadlock can occur if two agents were waiting for another agent
        // and now find themselves blocking each others trajectories
        // Resolve by aborting the current task and executing the provided
        // task (moving to a different destination)
        bool ResolveDeadlock(std::shared_ptr<MoverTask>& task);

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
        bool SubmittedTrajectoryWhileWaiting() const {
            return submitted_new_trajectory_while_waiting_;
        }
        bool VehicleIsAtStation() const {
            return Stationary() && env_.VehicleIsAtClaimableDestination(
                    GetCurrentPosition(), planner_.GetParameters());
        }
        int GetNbTasksCompleted() const { return nb_tasks_completed;};
        std::shared_ptr<const Trajectory> GetTrajectory() const {return planner_.GetLastSolution();}
        
        // Instruct this agent to wait for another agent. This agent is assumed
        // to continue moving once the other agent has passed.
        std::shared_ptr<VirtualAgent> WaitForAgent(
            std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx_, 
            CorridorUnion &intersection, bool wait_at_station=false);
        // Special case: if this agent was waiting and found a collision-free
        // way around the blocking agent, but still collides with another
        // moving vehicle, it must just continue to the waiting point is was
        // already moving towards. --> Might lead to deadlock!
        std::shared_ptr<VirtualAgent> WaitForAgent(
            std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx_,
            bool wait_at_station=false);
        // Special case: if this agent was waiting and found a collision-free
        // way around the blocking agent, but still collides with another
        // moving vehicle, just keep waiting for the original agent and try
        // again later (this happens while this agent is moving)
        void ResetWaitForAgent();

        void WaitForPrioritizedVehicle(std::shared_ptr<Agent> prioritized_agent,
                                       int prioritized_agent_idx);

        // Simulate a time-step and potentially update the current state
        void SimulateStep();

        #ifdef PARALLELIZE
        // Function called before actually updating the trajectory to be used
        // in case of parallel computation of trajectories such that all data
        // can be stored locally (such as current view of environment)
        void PrepareUpdateTrajectory();
        #endif

        // Update trajectory if needed
        bool UpdateTrajectory();

        // Get the vehicle footprint at a point in the future
        void GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin);

        void GetPosAndVel(int nb_time_steps_from_now, 
                          Point2D<double>& position, 
                          Point2D<double>& velocity);
        void UpdateSetpoint(Point2D<double>& pos, Point2D<double>& vel, Point2D<double>& acc) const;
        void GetFootprintBoundingBox(int start_idx, int stop_idx, Corridor& box) const;

        // check if the agent can avoid an intersection
        bool CanAvoidIntersection(const CorridorUnion& intersection) const;

        std::shared_ptr<VirtualAgent> GetVirtualAgent() const {
            return std::make_unique<VirtualAgent>(my_agent_idx_, planner_.GetParameters(), 
                                collision_check_margin_, 
                                planner_.GetLastSolution());
        }

        void PrintProfilerInfo() const {
            for (const auto& pair : profilers_){
                std::cout << pair.first << ":" << std::endl;
                pair.second.PrintLastStepInfo();
            }
        }

        void PrintEnvironment() const {
            std::cout << "agent has environment " << std::endl << env_ << std::endl;
        }

        json ToJson() const;

        void SetPerformanceMode(bool performance_mode){
            if (performance_mode){
                silent_mode_ = true;
                planner_.SetSilentMode(true);
            }
        }
        void PrepareOptiInstances(){ planner_.SetJustInTimePreparationMode(false); }

        void SetPlanWhileMoving(bool set){ wait_until_stationary_ = !set; }
        void SetMaxNbCorridorGrowingIterations(int n){ planner_.SetMaxNbCorridorGrowingIterations(n); }

    private:
        void PlanToDestination(bool to_waiting_point=false);

        void UpdateWaitingPosition(bool wait_at_station, CorridorUnion const &intersection);

        bool CheckForCollisionWithBlockingAgent(int start_idx=0, int stop_idx=-1);

        Environment& env_;          // true environment
        #ifdef PARALLELIZE
        Environment env_view_;     // environment used for planning based on 
                                    // claimed destinations
        #endif
        MotionPlanner planner_;
        AgentState state_;
        Point2D<double> final_dest_;
        const int my_agent_idx_ = -1;

        // attributes related to waiting for agent
        Point2D<double> waiting_position_;
        std::shared_ptr<Agent> blocking_agent_;
        int blocking_agent_idx_ = -1;
        CorridorUnion intersection_;
        bool submitted_new_trajectory_while_waiting_ = false;
        
        // attributes related to prioitized vehicles
        int curr_nb_rejections_ = 0;
        int max_nb_accepted_rejections_ = 3000000;//3;
        std::shared_ptr<Agent> prioritized_agent_;
        int prioritized_agent_idx_ = -1;

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
        std::vector<std::shared_ptr<const Trajectory>> planned_trajectories_;
        std::vector<CorridorSequence> planned_corridor_sequences_;
        std::vector<double> planned_times_;

        // caching of trajectories
        bool cached_trajectory_available_ = false;
        std::shared_ptr<const Trajectory> cached_trajectory_;

        // mover task logging
        std::shared_ptr<MoverTask> curr_task_; // the current task that is being executed by this agent
        int nb_tasks_completed = 0;

        // while resolving deadlock, store the aborted task
        std::vector<std::shared_ptr<MoverTask>> aborted_tasks_;

        // profiling
        bool perform_profiling_ = true;
        std::map<std::string, Profiler> profilers_;

        // current info
        Point2D<double> curr_pos_;
        Point2D<double> curr_vel_;
        Point2D<double> curr_acc_;
        double curr_time_;
        int nb_simulated_samples_ = 0;

        // options
        int replanning_frequency_ = 15; // in number of time-steps
        int replanning_step_counter_ = 0;
        bool wait_for_clear_intersection_ = false; // doesn't work well since the intersection will often only be clear for a short time
        bool wait_until_stationary_ = false; // wait until the agent is stationary before planning
        double collision_check_margin_ = 0.01;
        bool jit_planner_ = true;
        bool silent_mode_ = false;
        double waiting_point_distance_cell_lengths_ = 0.5;//1.5;

        // scratch space
        double t;
        Corridor footprint_;
        Corridor blocking_footprint_;
        Corridor o;
};

#endif