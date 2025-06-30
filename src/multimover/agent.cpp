#include "core/multimover/agent.hpp"
#include "core/multimover/mover_task.hpp"

Agent::Agent(int idx, Environment& env, const Parameters& params,
             double collision_check_margin, 
             Point2D<double> starting_position) : 
        my_agent_idx_(idx), env_(env), planner_(params, env), blocking_agent_(nullptr){
    state_ = IDLING;
    final_dest_ = starting_position;
    curr_pos_ = starting_position;
    collision_check_margin_ = collision_check_margin;
    planner_.SetMaxNbCorridorGrowingIterations(1);
    if (!jit_planner_){
        planner_.SetJustInTimePreparationMode(false);
    }

    travelled_trajectory_.Reset(starting_position);
}

bool Agent::InstructToDestination(const std::string& destination_name,
                                  const Point2D<double>& final_dest,
                                  std::shared_ptr<MoverTask>& task){
    if (state_ != IDLING && state_ != WAITING_FOR_FREE_DESTINATION){
        throw std::runtime_error("Cannot change final destination when not in IDLING state");
    }

    // if we have a task, store it such that we can notify it later
    curr_task_ = task;

    // Check if we can claim the destination in the environment
    if (!env_.ClaimDestination(destination_name, this)){
        // if we were unable to claim the destination, update the state and
        // return false
        ptr_to_object_claiming_destination_ = env_.GetObjectClaimingDestination(destination_name);
        state_ = WAITING_FOR_FREE_DESTINATION;
        return false;
    }

    // check if we were already claiming a destination, if so store it such
    // that we can release it after replanning
    // we cannot release it yet because that would cause an invalid starting 
    // point in the environment
    if (currently_claiming_){
        // only add it if it is not already in the list
        if (std::find(destinations_to_be_released_.begin(), 
                      destinations_to_be_released_.end(), 
                      claimed_destination_name_) == destinations_to_be_released_.end()){
            // add the destination to the list of destinations to be released
            // later
            destinations_to_be_released_.push_back(claimed_destination_name_);
        }
    }

    // if we were able to claim the destination, take note of this such that
    // we can release it later
    currently_claiming_ = true;
    claimed_destination_name_ = destination_name;

    final_dest_ = final_dest;
    state_ = READY_TO_PLAN;

    // update the corridor sequence and lock it
    env_.SetClaimingObject(this);
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(final_dest_);
    planner_.UpdateCorridorSequence();
    planner_.LockCorridorSequence();

    return true;
}

const Point2D<double>& Agent::GetFinalDestination() const{
    return final_dest_;
}

const Point2D<double> Agent::GetCurrentPosition() const{
    return Point2D<double>(
        planner_.GetLastSolution().Px()[planner_.GetCurrentSampleIdx()],
        planner_.GetLastSolution().Py()[planner_.GetCurrentSampleIdx()]
    );
}

const AgentState& Agent::GetState() const{
    return state_;
}

const Point2D<double>& Agent::GetWaitingPosition() const{
    return waiting_position_;
}

const Agent& Agent::GetBlockingAgent() const{
    return *blocking_agent_;
}

const void* Agent::GetPtrToObjectClaimingDestination() const{
    if (state_ != WAITING_FOR_FREE_DESTINATION){
        throw std::runtime_error("No object is currently claiming the destination");
    }
    return ptr_to_object_claiming_destination_;
}

const CorridorUnion& Agent::GetIntersection() const{
    return intersection_;
}

int Agent::GetRemainingTimeSteps() const{
    return planner_.GetLastSolution().NbSamples() - planner_.GetCurrentSampleIdx();
}

double Agent::GetTimeAtTimeStep(int future_time_step) const{
    double local_time = planner_.GetLastSolution().T()[planner_.GetCurrentSampleIdx() + future_time_step];
    return local_time + planner_.GetLastSolution().T0();
}

void Agent::WaitForAgent(std::shared_ptr<Agent> blocking_agent, 
                         int blocking_agent_idx, 
                         CorridorUnion &intersection){
    if (state_ == IDLING){
        throw std::runtime_error("Agent cannot wait for another agent when idling");
    }
    if (state_ == WAITING_AT_INTERSECTION || state_ == MOVING_TO_WAITING_POINT){
        std::cout << "WARNING: This agent was already waiting for another agent" << std::endl;
        // if the current agent was already waiting for the blocking agent,
        // it must be somehow colliding with the blocking agent on it's way to
        // the waiting position. Add the cell in which the current waiting
        // point is located to the intersection and find a new waiting position
        if (blocking_agent_idx == blocking_agent_idx_){
            std::cout << "enlarging the intersection with the current waiting position" << std::endl;
            // TODO: Somehow, this change in the intersection should be made persistant, otherwise we can cycle infinitely long
            double cw = env_.CellWidth();
            double ch = env_.CellHeight();
            intersection.AddCorridor(
                Corridor(waiting_position_.x() - cw/2,
                         waiting_position_.x() + cw/2,
                         waiting_position_.y() - ch/2,
                         waiting_position_.y() + ch/2)
                );
        }

    }
    if (blocking_agent_idx < 0){
        throw std::runtime_error("Invalid blocking agent index");
    }
    try{
        waiting_position_ = planner_.GetCorridorSequence().GetWaitingPosition(
            planner_.GetStart(), intersection, 
            planner_.GetParameters(), 
            planner_.GetEnvironment().CellWidth(),
            planner_.GetEnvironment().CellHeight());
        std::cout << "waiting position: " << waiting_position_ << std::endl;
    } catch (UnableToFindWaitingPoint& e){
        // If we cannot find a waiting point but we're stationary, just wait here
        if (Stationary()){ waiting_position_ = curr_pos_; }
        // otherwise the other agent will have to wait
        else { throw e;}
    }
    
    state_ = MOVING_TO_WAITING_POINT;
    blocking_agent_ = blocking_agent;
    blocking_agent_idx_ = blocking_agent_idx;
    intersection_ = intersection;

    PlanToDestination(true);
}

void Agent::WaitForAgent(std::shared_ptr<Agent> blocking_agent, 
                         int blocking_agent_idx){
    if (state_ == IDLING){
        throw std::runtime_error("Agent cannot wait for another agent when idling");
    }
    if (blocking_agent_idx < 0){
        throw std::runtime_error("Invalid blocking agent index");
    }
    
    state_ = MOVING_TO_WAITING_POINT;
    blocking_agent_ = blocking_agent;
    blocking_agent_idx_ = blocking_agent_idx;

    PlanToDestination(true);
}

void Agent::ResetWaitForAgent(){
    if (!submitted_new_trajectory_while_waiting_){
        throw std::runtime_error("Cannot reset wait for agent when not waiting for an agent");
    }

    // reset the state and the blocking agent
    state_ = MOVING_TO_WAITING_POINT;
    planner_.RevertToPreviousTrajectory();
    std::cout << "AGENT " << my_agent_idx_ << " (" << this << 
        ") resetting wait for agent state, reverting to previous trajectory." << std::endl;
}

void Agent::SimulateStep(){
    nb_simulated_samples_++;
    planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
    curr_time_ = nb_simulated_samples_*planner_.GetLastSolution().Dt();
    travelled_trajectory_.Append(curr_time_, curr_pos_.x(), curr_pos_.y(), 
                                 curr_vel_.x(), curr_vel_.y(), 
                                 curr_acc_.x(), curr_acc_.y());

    submitted_new_trajectory_while_waiting_ = false;
    
    // check for potential state changes
    if (state_ == MOVING_TO_FINAL_DESTINATION && 
            curr_pos_.Distance(final_dest_) <= 1.0e-3 && 
            curr_vel_.Norm() <= 1.0e-2){
        // we have reached the final destination
        state_ = IDLING;
        planner_.UnlockCorridorSequence();
        
        if (curr_task_.get() != nullptr){
            // if we have a task, mark it as completed
            curr_task_->NotifyCompleted(curr_time_);
        }

    } else if (state_ == MOVING_TO_WAITING_POINT && 
            curr_pos_.Distance(waiting_position_) <= 1.0e-3 && 
            curr_vel_.Norm() <= 1.0e-2){
        // we have reached the waiting point
        state_ = WAITING_AT_INTERSECTION;

        if (curr_task_.get() != nullptr){
            curr_task_->NotifyStartedToWait(curr_time_);
        }
        
    }

    // Check if we can release a destination
    std::vector<int> indices_to_be_released;
    Point2D<double> destination;
    Corridor cell, footprint, o;
    for (int i = 0; i < destinations_to_be_released_.size(); i++){
        destination = env_.GetClaimableDestinationLocation(destinations_to_be_released_[i]);
        cell.SetXmin(destination.x() - planner_.GetParameters().GetWidthOffset());
        cell.SetXmax(destination.x() + planner_.GetParameters().GetWidthOffset());
        cell.SetYmin(destination.y() - planner_.GetParameters().GetHeightOffset());
        cell.SetYmax(destination.y() + planner_.GetParameters().GetHeightOffset());
        GetVehicleFootprint(0, footprint, 0.0);
        if (!cell.GetOverlap(footprint, o)){
            // we can release the destination
            indices_to_be_released.push_back(i);
            std::cout << "agent " << my_agent_idx_ << "(" << this << ") releasing destination " << 
                destinations_to_be_released_[i] << " at " << destination <<
                " currently claimed by " << env_.GetObjectClaimingDestination(destinations_to_be_released_[i]) << std::endl;
            env_.ReleaseDestination(destinations_to_be_released_[i], this);
        }
    }
    // remove the destinations that can be released
    for (int i = indices_to_be_released.size() - 1; i >= 0; i--){
        destinations_to_be_released_.erase(destinations_to_be_released_.begin() + 
                                           indices_to_be_released[i]);
    }

    // store logging info
    travelled_states_.push_back(state_);
    travelled_blocking_agent_idx_.push_back(blocking_agent_idx_);
    travelled_final_destinations_.push_back(final_dest_);
}

bool Agent::UpdateTrajectory(){
    // Check if we need to replan
    if (state_ == MOVING_TO_FINAL_DESTINATION || state_ == IDLING || 
            state_ == WAITING_FOR_FREE_DESTINATION){
        // nothing to be done
        return false;
    }

    // if we're in emergency state but came to a stop, we can plan again
    if (state_ == FAILED_TO_PLAN_TO_WAITING_POINT || 
            state_ == FAILED_TO_PLAN_TO_DEST){
        if (Stationary()){
            state_ = READY_TO_PLAN;
        } else {
            // wait until vehicle stops completely
            return false;
        }
    }

    // if we've been given a destination and are ready to plan, plan a new
    // trajectory
    if (state_ == READY_TO_PLAN){
        // set the start and destination
        PlanToDestination();
        if (state_ == MOVING_TO_FINAL_DESTINATION && 
            curr_task_.get() != nullptr){
            // notify the task that we started moving
            curr_task_->NotifyStartedToMove(curr_time_);

        }
        return true;
    }

    // otherwise, we are waiting for the blocking agent, so we can try to see
    // if we can continue
    replanning_step_counter_++;
    if (replanning_step_counter_ >= replanning_frequency_){
        if (wait_until_stationary_ && !Stationary()){
            // we are not stationary, so we cannot replan yet
            return false;
        }

        replanning_step_counter_ = 0;

        // check if we can replan
        if (wait_for_clear_intersection_){
            blocking_agent_->GetVehicleFootprint(0, blocking_footprint_, 
                                                 collision_check_margin_);
            if (intersection_.OverlapsWith(blocking_footprint_)){
                // blocking agent is still in the intersection
                return false;
            } else {
                // blocking agent is out of the intersection
                PlanToDestination();
                if (curr_task_.get() != nullptr){
                    curr_task_->NotifyStartedToMove(curr_time_);
                }
                return true;
            }
        } else {
            // Attempt to plan a new trajectory and check for collision
            PlanToDestination();

            if (CheckForCollisionWithBlockingAgent()){
                std::cout << "Collision detected with blocking agent, replanning..." << std::endl;
                /*
                PlanToDestination(true);
                // discard first trajectory sample
                planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);

                return false; // TODO: actually, the new trajectory might be
                // different than the one before. It would be better to 
                // actually revert to the previous trajectory. This would also
                // be much more efficient
                */
                
                planner_.RevertToPreviousTrajectory();
                planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
                state_ = MOVING_TO_WAITING_POINT;
                return false;
                

            } else {
                // no collision detected, we can continue
                if (curr_task_.get() != nullptr){
                    curr_task_->NotifyStartedToMove(curr_time_);
                }
                submitted_new_trajectory_while_waiting_ = true;
                return true;
                // TODO: this trajectory should still be checked against other vehicles
            }
        }
    }

    return false;
}

void Agent::GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin){
    nb_time_steps_from_now = std::min(nb_time_steps_from_now, 
        planner_.GetLastSolution().NbSamples() - 1 - planner_.GetCurrentSampleIdx());
    double px = planner_.GetLastSolution().Px()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now];
    double py = planner_.GetLastSolution().Py()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now];
    double width_offset = planner_.GetParameters().GetWidthOffset() + collision_check_margin;
    double height_offset = planner_.GetParameters().GetHeightOffset() + collision_check_margin;
    footprint.SetXmin(px - width_offset);
    footprint.SetXmax(px + width_offset);
    footprint.SetYmin(py - height_offset);
    footprint.SetYmax(py + height_offset);
}

bool Agent::CanAvoidIntersection(const CorridorUnion& intersection) const {
    // check if the agent can avoid the intersection
    return planner_.CanAvoidCorridors(intersection.GetCorridors(), 
                                      curr_pos_, curr_vel_);
}

json Agent::ToJson() const {
    json j;
    j["planner"] = planner_.ToJson();
    j["travelled_trajectory"] = travelled_trajectory_.ToJson();
    j["travelled_states"] = json::array();
    for (const auto& state : travelled_states_){
        j["travelled_states"].push_back(AgentStateToString(state));
    }
    j["travelled_blocking_agent_idx"] = json::array();
    for (const auto& idx : travelled_blocking_agent_idx_){
        j["travelled_blocking_agent_idx"].push_back(idx);
    }
    j["travelled_final_destinations"] = json::array();
    for (const auto& dest : travelled_final_destinations_){
        j["travelled_final_destinations"].push_back(dest.ToJson());
    }
    j["planned_trajectories"] = json::array();
    for (const auto& traj : planned_trajectories_){
        j["planned_trajectories"].push_back(traj.ToJson());
    }
    j["planned_corridor_sequences"] = json::array();
    for (const auto& seq : planned_corridor_sequences_){
        j["planned_corridor_sequences"].push_back(seq.ToJson());
    }
    j["planned_times"] = json::array();
    for (const auto& time : planned_times_){
        j["planned_times"].push_back(time);
    }
    j["memory_address"] = std::to_string(reinterpret_cast<std::uintptr_t>(this));
    return j;
}

void Agent::PlanToDestination(bool to_waiting_point){
    // plan
    std::cout << "Agent " << this << " planning to " 
              << (to_waiting_point ? "waiting point" : "final destination") 
              << std::endl;
    env_.SetClaimingObject(this);
    state_ = to_waiting_point ? MOVING_TO_WAITING_POINT : MOVING_TO_FINAL_DESTINATION;
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(to_waiting_point ? waiting_position_ : final_dest_);
    try{
        planner_.PlanSafely();

        // update mover task logging
        double planning_time = planner_.GetTotalComputationTime();
        if (curr_task_.get() != nullptr){
            curr_task_->NotifyPlanningOccured(curr_time_, planning_time);
        }

    } catch (std::exception & e){
        std::cout << "Planning failed: " << e.what() << std::endl;
        // if we cannot plan, we need to set the state accordingly
        state_ = to_waiting_point ? FAILED_TO_PLAN_TO_WAITING_POINT : 
            FAILED_TO_PLAN_TO_DEST;
        env_.ClearClaimingObject();

        // update mover task logging
        double planning_time = planner_.GetTotalComputationTime();
        if (curr_task_.get() != nullptr){
            curr_task_->NotifyPlanningOccured(curr_time_, planning_time);
        }
        return;
    }
    
    // check if something went wrong
    if (planner_.EmergencyMode()){
        state_ = to_waiting_point ? FAILED_TO_PLAN_TO_WAITING_POINT : 
            FAILED_TO_PLAN_TO_DEST;
    }

    // update stored info
    planned_trajectories_.push_back(planner_.GetLastSolution());
    planned_corridor_sequences_.push_back(planner_.GetCorridorSequence());
    planned_times_.push_back(curr_time_);
    env_.ClearClaimingObject();
}

bool Agent::CheckForCollisionWithBlockingAgent(){
    int nb_time_steps_to_check = std::max(
        GetRemainingTimeSteps(),
        blocking_agent_->GetRemainingTimeSteps()
    );
    
    for (int nb_steps_in_future = 0; 
            nb_steps_in_future < nb_time_steps_to_check; nb_steps_in_future++){
        GetVehicleFootprint(nb_steps_in_future, footprint_, 0);
        blocking_agent_->GetVehicleFootprint(nb_steps_in_future, 
                                blocking_footprint_, 0);

        if (footprint_.GetOverlap(blocking_footprint_, o)){
            std::cout << "[" << nb_steps_in_future << "] " << footprint_ << " " << blocking_footprint_ << std::endl;
            return true;
        }
    }
    return false;
};