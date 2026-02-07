#include "core/multimover/agent.hpp"
#include "core/multimover/mover_task.hpp"

Agent::Agent(int idx, Environment& env, const Parameters& params,
             double collision_check_margin, 
             Point2D<double> starting_position) : 
        #ifdef PARALLELIZE
        my_agent_idx_(idx), env_(env), env_view_(env_.Clone()), planner_(params, env_view_), blocking_agent_(nullptr){
        #else
        my_agent_idx_(idx), env_(env), planner_(params, env_), blocking_agent_(nullptr){
        #endif
    state_ = IDLING;
    final_dest_ = starting_position;
    curr_pos_ = starting_position;
    collision_check_margin_ = collision_check_margin;
    planner_.SetMaxNbCorridorGrowingIterations(1);
    if (!jit_planner_){
        planner_.SetJustInTimePreparationMode(false);
    }

    travelled_trajectory_.Reset(starting_position);

    if (perform_profiling_){
        profilers_["UpdateTrajectory"] = Profiler({"StateUpdate", 
            "PlanToDestination", "InBetweenPlanning", "PlanWhileWaiting", 
            "CollisionCheck", "Terminal"});
    }

    planned_trajectories_.reserve(1000);
    planned_corridor_sequences_.reserve(1000);
    planned_times_.reserve(1000);
}

bool Agent::InstructToDestination(const std::string& destination_name,
                                  const Point2D<double>& final_dest,
                                  std::shared_ptr<MoverTask>& task){
    std::cout << "Agent " << my_agent_idx_ << 
        " instructed to destination " << destination_name << std::endl;
    if (state_ != IDLING && state_ != WAITING_FOR_FREE_DESTINATION){
        throw std::runtime_error("Cannot change final destination when not in IDLING state");
    }

    // if we have a task, store it such that we can notify it later
    curr_task_ = task;
    cached_trajectory_available_ = false;

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
    #ifdef PARALLELIZE
    env_.UpdateLocalView(env_view_, this);
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(final_dest_);
    planner_.UpdateCorridorSequence();
    planner_.LockCorridorSequence();
    #else
    env_.SetClaimingObject(this);
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(final_dest_);
    planner_.UpdateCorridorSequence();
    Point2D<double> desired_dest = final_dest_;
    Point2D<double> corridor_dest = planner_.GetCorridorSequence().GetDest();
    if (desired_dest.x() != corridor_dest.x() ||
        desired_dest.y() != corridor_dest.y()){
        throw std::runtime_error("Corridor sequence destination does not match final destination");
    }
    planner_.LockCorridorSequence();
    env_.ClearClaimingObject();
    #endif

    return true;
}

bool Agent::ResolveDeadlock(std::shared_ptr<MoverTask>& task) {
    if (!task->IsDeadlockResolutionTask() || (state_ != WAITING_AT_INTERSECTION && state_ != WAITING_FOR_FREE_DESTINATION)){
        std::cout << "agent " << my_agent_idx_ << 
            " (current state: " << AgentStateToString(state_) << ")" << 
            "cannot move to destination " << task->GetDestinationName() << 
            " (deadlock resolving mark: " << task->IsDeadlockResolutionTask() << ")" << std::endl;
        throw std::runtime_error("Cannot perform deadlock resolution when not waiting at an intersection");
    }
    cached_trajectory_available_ = false;

    // Check if we can claim the destination in the environment
    if (!env_.ClaimDestination(task->GetDestinationName(), this)){
        // if we were unable to claim the destination, update the state and
        // return false
        ptr_to_object_claiming_destination_ = env_.GetObjectClaimingDestination(task->GetDestinationName());
        state_ = WAITING_FOR_FREE_DESTINATION;
        return false;
    }
    // release the previously claimed destination
    if (state_ != WAITING_FOR_FREE_DESTINATION){
        env_.ReleaseDestination(claimed_destination_name_, this);
    } else {
        // make sure we will release the destination we're currently at once 
        // we clear it
        if (std::find(destinations_to_be_released_.begin(), 
                      destinations_to_be_released_.end(), 
                      claimed_destination_name_) == destinations_to_be_released_.end()){
            // add the destination to the list of destinations to be released
            // later
            destinations_to_be_released_.push_back(claimed_destination_name_);
        }
    }
    
    // abort the current task for now and store the new task
    curr_task_->NotifyAborted(curr_time_);
    if (!curr_task_->IsDeadlockResolutionTask() && // don't try to finish deadlock resolution tasks
            !curr_task_->HasBeenCompleted()){ // don't try to finish already completed tasks 
        aborted_tasks_.push_back(curr_task_);
    }

    curr_task_ = task;

    // if we were able to claim the destination, take note of this such that
    // we can release it later
    currently_claiming_ = true;
    claimed_destination_name_ = task->GetDestinationName();

    final_dest_ = env_.GetClaimableDestinationLocation(
        claimed_destination_name_);
    state_ = READY_TO_PLAN;

    // update the corridor sequence and lock it
    #ifdef PARALLELIZE
    env_.UpdateLocalView(env_view_, this);
    // planner_.SetMaxNbCorridorGrowingIterations(5);
    planner_.UnlockCorridorSequence();
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(final_dest_);
    planner_.UpdateCorridorSequence();
    planner_.LockCorridorSequence();
    // planner_.SetMaxNbCorridorGrowingIterations(1);
    #else
    env_.SetClaimingObject(this);
    // planner_.SetMaxNbCorridorGrowingIterations(5);
    planner_.UnlockCorridorSequence();
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(final_dest_);
    planner_.UpdateCorridorSequence();
    planner_.LockCorridorSequence();
    // planner_.SetMaxNbCorridorGrowingIterations(1);
    env_.ClearClaimingObject();
    #endif

    return true;
}

const Point2D<double>& Agent::GetFinalDestination() const{
    return final_dest_;
}

const Point2D<double> Agent::GetCurrentPosition() const{
    // return Point2D<double>(
    //     planner_.GetLastSolution()->Px()[planner_.GetCurrentSampleIdx()],
    //     planner_.GetLastSolution()->Py()[planner_.GetCurrentSampleIdx()]
    // );
    return curr_pos_;
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
    return planner_.GetLastSolution()->NbSamples() - planner_.GetCurrentSampleIdx();
}

double Agent::GetTimeAtTimeStep(int future_time_step) const{
    double local_time = planner_.GetLastSolution()->T()[planner_.GetCurrentSampleIdx() + future_time_step];
    return local_time + planner_.GetLastSolution()->T0();
}

std::shared_ptr<VirtualAgent> Agent::WaitForAgent(
        std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx, 
        CorridorUnion &intersection, bool wait_at_station){
    cached_trajectory_available_ = false;
    std::cout << "Agent " << my_agent_idx_ << " instructed to wait for agent " << blocking_agent_idx << 
        " (" << blocking_agent.get() << ")" << std::endl;
    if (state_ == IDLING){
        throw std::runtime_error("Agent cannot wait for another agent when idling");
    }
    if (blocking_agent_idx < 0){
        throw std::runtime_error("Invalid blocking agent index");
    }
    std::shared_ptr<VirtualAgent> virtual_agent = nullptr;
    if (state_ == WAITING_AT_INTERSECTION || state_ == MOVING_TO_WAITING_POINT){
        if (!silent_mode_) std::cout << "WARNING: This agent was already waiting for another agent" << std::endl;
        // if the current agent was already waiting for the blocking agent,
        // it must be somehow colliding with the blocking agent on it's way to
        // the waiting position. Add the cell in which the current waiting
        // point is located to the intersection and find a new waiting position
        if (false || blocking_agent_idx == blocking_agent_idx_){
            if (!silent_mode_) std::cout << "enlarging the intersection with the current waiting position" << std::endl;
            // TODO: Somehow, this change in the intersection should be made persistant, otherwise we can cycle infinitely long
            double cw = env_.CellWidth();
            double ch = env_.CellHeight();
            intersection.AddCorridor(
                Corridor(waiting_position_.x() - cw/2,
                         waiting_position_.x() + cw/2,
                         waiting_position_.y() - ch/2,
                         waiting_position_.y() + ch/2)
                );
        } else if (Stationary()) {
            // if we are instructed to wait for another agent, increment our 
            // rejection counter
            curr_nb_rejections_++;
            if (curr_nb_rejections_ > max_nb_accepted_rejections_){
                // too many rejections --> return a virtual agent containing
                // our desired trajectory such that the multimoversimulator
                // can consider it
                virtual_agent = GetVirtualAgent();
            }
        }

    }
    try{
        UpdateWaitingPosition(wait_at_station, intersection);
    } catch (UnableToFindWaitingPoint& e){
        // If we cannot find a waiting point but we're stationary, just wait here
        if (Stationary()){ waiting_position_ = curr_pos_; }
        // otherwise the other agent will have to wait
        else { throw e;}
        std::cout << "Couldn't find a waiting point (" << Stationary() << ")" << std::endl;
    }
    
    state_ = MOVING_TO_WAITING_POINT;
    blocking_agent_ = blocking_agent;
    blocking_agent_idx_ = blocking_agent_idx;
    intersection_ = intersection;

    PlanToDestination(true);

    return virtual_agent;
}

std::shared_ptr<VirtualAgent> Agent::WaitForAgent(
        std::shared_ptr<Agent> blocking_agent, int blocking_agent_idx,
        bool wait_at_station){
    cached_trajectory_available_ = false;
    if (state_ == IDLING){
        throw std::runtime_error("Agent cannot wait for another agent when idling");
    }
    if (blocking_agent_idx < 0){
        throw std::runtime_error("Invalid blocking agent index");
    }

    std::shared_ptr<VirtualAgent> virtual_agent = nullptr;
    if (Stationary()){
        curr_nb_rejections_++;
        if (curr_nb_rejections_ > max_nb_accepted_rejections_){
            // too many rejections --> return a virtual agent containing
            // our desired trajectory such that the multimoversimulator
            // can consider it
            virtual_agent = GetVirtualAgent();
        }
    }
    // TODO: update the waiting position!
    // UpdateWaitingPosition(wait_at_station, CorridorUnion());
    
    state_ = MOVING_TO_WAITING_POINT;
    blocking_agent_ = blocking_agent;
    blocking_agent_idx_ = blocking_agent_idx;


    PlanToDestination(true);

    return virtual_agent;
}

void Agent::ResetWaitForAgent(){
    std::cout << "agent " << my_agent_idx_ << " instructed to reset wait for agent state" << std::endl;
    cached_trajectory_available_ = false;
    if (!submitted_new_trajectory_while_waiting_){
        // throw std::runtime_error("Cannot reset wait for agent when not waiting for an agent");
        planner_.ResetTrajectory();
        return;
    }

    // reset the state and the blocking agent
    state_ = MOVING_TO_WAITING_POINT;
    planner_.ResetTrajectory();
    if (!silent_mode_) std::cout << "AGENT " << my_agent_idx_ << " (" << this << 
        ") resetting wait for agent state, reverting to previous trajectory." << std::endl;
}

void Agent::WaitForPrioritizedVehicle(std::shared_ptr<Agent> prioritized_agent,
                                      int prioritized_agent_idx){
    cached_trajectory_available_ = false;
    if (!Stationary()){
        throw std::runtime_error("Cannot wait for a prioritized vehicle when not moving");
    }

    waiting_position_ = curr_pos_;
    PlanToDestination(true);

    // set the state and the prioritized agent
    state_ = WAITING_FOR_PRIORITIZED_VEHICLE;
    prioritized_agent_ = prioritized_agent;
    prioritized_agent_idx_ = prioritized_agent_idx;

    if (!silent_mode_) std::cout << "\n\n\nWaiting for prioritized vehicle: " << prioritized_agent_idx << "\n\n\n" << std::endl;
}

void Agent::SimulateStep(){
    nb_simulated_samples_++;
    Point2D<double> prev_pos = curr_pos_;
    planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
    if (state_ == WAITING_AT_INTERSECTION ||
            state_ == WAITING_FOR_FREE_DESTINATION ||
            state_ == READY_TO_PLAN){
        // make sure we don't move while waiting (could happen due to numerical issues)
        curr_vel_ = Point2D<double>(0.0, 0.0);
        }

    if (prev_pos.x() != curr_pos_.x() && prev_pos.y() != curr_pos_.y()){
        cached_trajectory_available_ = false;
    }

    curr_time_ = nb_simulated_samples_*planner_.GetLastSolution()->Dt();
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
        curr_vel_ = Point2D<double>(0.0, 0.0);
        
        if (curr_task_.get() != nullptr){
            // if we have a task, mark it as completed
            curr_task_->NotifyCompleted(curr_time_);
            
            // if this is a task in a sequence, increment the number of completed tasks
            if (curr_task_->GetTaskSequenceNb() >= 0){
                nb_tasks_completed++;
            }
        }

        // check if we have some aborted tasks waiting to be continued
        if (!aborted_tasks_.empty()){
            curr_task_ = aborted_tasks_.front();
            curr_task_->NotifyResumed(curr_time_);
            Point2D<double> dest = env_.GetClaimableDestinationLocation(
                curr_task_->GetDestinationName());
            bool succes = InstructToDestination(curr_task_->GetDestinationName(), dest, curr_task_);
            if (!succes){
                curr_task_->PostponeTask(0.05, curr_time_);
            }
            aborted_tasks_.erase(aborted_tasks_.begin());
        }

    } else if (state_ == MOVING_TO_WAITING_POINT && 
            curr_pos_.Distance(waiting_position_) <= 1.0e-3 && 
            curr_vel_.Norm() <= 1.0e-2){
        // we have reached the waiting point
        state_ = WAITING_AT_INTERSECTION;
        curr_vel_ = Point2D<double>(0.0, 0.0);

        if (curr_task_.get() != nullptr){
            curr_task_->NotifyStartedToWait(curr_time_);
        }
        
    }

    // If we are currently moving, reset the rejection counter
    if (!Stationary()) { curr_nb_rejections_ = 0; }

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
            if (!silent_mode_) std::cout << "agent " << my_agent_idx_ << "(" << this << ") releasing destination " << 
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

#ifdef PARALLELIZE
void Agent::PrepareUpdateTrajectory(){
    // update local view of the environment
    env_.UpdateLocalView(env_view_, this);
}
#endif

bool Agent::UpdateTrajectory(){
    if (perform_profiling_) { profilers_["UpdateTrajectory"].StartSimulationStep(); }
    // Check if we need to replan
    if (state_ == MOVING_TO_FINAL_DESTINATION || state_ == IDLING || 
            state_ == WAITING_FOR_FREE_DESTINATION){
        // nothing to be done
        if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
        return false;
    }

    // if we were waiting for a prioritized vehicle that has started moving, 
    // we can proceed again
    if (state_ == WAITING_FOR_PRIORITIZED_VEHICLE){
        if (prioritized_agent_ != nullptr && !prioritized_agent_->Stationary()){
            state_ = READY_TO_PLAN;
        } else {
            if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
            return false;
        }
    }

    // if we're in emergency state but came to a stop, we can plan again
    if (state_ == FAILED_TO_PLAN_TO_WAITING_POINT || 
            state_ == FAILED_TO_PLAN_TO_DEST){
        if (Stationary()){
            state_ = READY_TO_PLAN;
        } else {
            // wait until vehicle stops completely
            if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
            return false;
        }
    }

    // if we've been given a destination and are ready to plan, plan a new
    // trajectory
    if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // StateUpdate
    if (state_ == READY_TO_PLAN){
        // set the start and destination
        PlanToDestination();
        if (state_ == MOVING_TO_FINAL_DESTINATION && 
            curr_task_.get() != nullptr){
            // notify the task that we started moving
            curr_task_->NotifyStartedToMove(curr_time_);

        }
        if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
        return true;
    }
    if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // PlanToDestination

    // otherwise, we are waiting for the blocking agent, so we can try to see
    // if we can continue
    replanning_step_counter_++;
    if (replanning_step_counter_ >= replanning_frequency_){
        if (wait_until_stationary_ && !Stationary()){
            // we are not stationary, so we cannot replan yet
            if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
            return false;
        }

        replanning_step_counter_ = 0;

        // check if we can replan
        if (wait_for_clear_intersection_){
            blocking_agent_->GetVehicleFootprint(0, blocking_footprint_, 
                                                 0*collision_check_margin_);
            if (intersection_.OverlapsWith(blocking_footprint_)){
                // blocking agent is still in the intersection
                if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
                return false;
            } else {
                // blocking agent is out of the intersection
                if (perform_profiling_) {profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // InBetweenPlanning
                PlanToDestination();
                if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // PlanWhileWaiting

                if (curr_task_.get() != nullptr){
                    curr_task_->NotifyStartedToMove(curr_time_);
                }
                submitted_new_trajectory_while_waiting_ = true;
                if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
                return true;
            }
        } else {
            // Attempt to plan a new trajectory and check for collision
            if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // InBetweenPlanning
            PlanToDestination();
            if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // PlanWhileWaiting

            if (CheckForCollisionWithBlockingAgent()){
                if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // CollisionCheck
                if (!silent_mode_) std::cout << "Collision detected with blocking agent, replanning..." << std::endl;
                
                planner_.ResetTrajectory();
                // planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
                state_ = MOVING_TO_WAITING_POINT;
                if (perform_profiling_) { profilers_["UpdateTrajectory"].EndSimulationStep(); }
                return false;
                

            } else {
                if (perform_profiling_) { profilers_["UpdateTrajectory"].RecordIntermediateSimulationStep(); } // CollisionCheck
                // no collision detected, we can continue
                if (curr_task_.get() != nullptr){
                    curr_task_->NotifyStartedToMove(curr_time_);
                }
                submitted_new_trajectory_while_waiting_ = true;
                if (perform_profiling_) { profilers_["UpdateTrajectory"].EndSimulationStep(); }
                return true;
                // TODO: this trajectory should still be checked against other vehicles
            }
        }
    }

    if (perform_profiling_) { profilers_["UpdateTrajectory"].AbortStep(); }
    return false;
}

void Agent::GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin){
    nb_time_steps_from_now = std::min(nb_time_steps_from_now, 
        planner_.GetLastSolution()->NbSamples() - 1 - planner_.GetCurrentSampleIdx());
    double px = planner_.GetLastSolution()->Px()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now];
    double py = planner_.GetLastSolution()->Py()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now];
    double width_offset = planner_.GetParameters().GetWidthOffset() + collision_check_margin;
    double height_offset = planner_.GetParameters().GetHeightOffset() + collision_check_margin;
    footprint.SetXmin(px - width_offset);
    footprint.SetXmax(px + width_offset);
    footprint.SetYmin(py - height_offset);
    footprint.SetYmax(py + height_offset);
}

void Agent::GetPosAndVel(int nb_time_steps_from_now,
                         Point2D<double>& position,
                         Point2D<double>& velocity){
    nb_time_steps_from_now = std::min(nb_time_steps_from_now,
        planner_.GetLastSolution()->NbSamples() - 1 - planner_.GetCurrentSampleIdx());
    position.SetX(planner_.GetLastSolution()->Px()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now]);
    position.SetY(planner_.GetLastSolution()->Py()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now]);
    velocity.SetX(planner_.GetLastSolution()->Vx()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now]);
    velocity.SetY(planner_.GetLastSolution()->Vy()[planner_.GetCurrentSampleIdx() + nb_time_steps_from_now]);
}

void Agent::UpdateSetpoint(Point2D<double>& pos, Point2D<double>& vel, Point2D<double>& acc) const {
    std::shared_ptr<const Trajectory> traj = planner_.GetLastSolution();
    int idx = std::min(traj->NbSamples() - 1, planner_.GetCurrentSampleIdx());
    pos.SetX(traj->Px()[idx]);
    pos.SetY(traj->Py()[idx]);
    vel.SetX(traj->Vx()[idx]);
    vel.SetY(traj->Vy()[idx]);
    acc.SetX(traj->Ax()[idx]);
    acc.SetY(traj->Ay()[idx]);
}

void Agent::GetFootprintBoundingBox(int start_idx, int stop_idx, Corridor& box) const {
    int curr_sample_idx = planner_.GetCurrentSampleIdx();
    planner_.GetLastSolution()->GetBoundingBox(
        curr_sample_idx + start_idx, 
        curr_sample_idx + stop_idx, box);
    
    double width_offset = planner_.GetParameters().GetWidthOffset();
    box.SetXmin(box.Xmin() - width_offset);
    box.SetXmax(box.Xmax() + width_offset);

    double height_offset = planner_.GetParameters().GetHeightOffset();
    box.SetYmin(box.Ymin() - height_offset);
    box.SetYmax(box.Ymax() + height_offset);
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
        j["planned_trajectories"].push_back(traj->ToJson());
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
    if (perform_profiling_){
        j["detailed_profilers"] = json::object();
        for (const auto& pair : profilers_){
            j["detailed_profilers"][pair.first] = pair.second.ToJsonDetailed();
        }
    }
    return j;
}

void Agent::PlanToDestination(bool to_waiting_point){
    // plan
    if (!silent_mode_) std::cout << "Agent " << my_agent_idx_ << " (" << this << ") planning to " 
              << (to_waiting_point ? "waiting point" : "final destination") 
              << std::endl;
    auto time_1 = std::chrono::high_resolution_clock::now();
    #ifdef PARALLELIZE
    env_.UpdateLocalView(env_view_, this);
    #else
    env_.SetClaimingObject(this);
    #endif
    AgentState state_before = state_;
    state_ = to_waiting_point ? MOVING_TO_WAITING_POINT : MOVING_TO_FINAL_DESTINATION;

    if (cached_trajectory_available_){
        // check if we are at the starting position of the cached trajectory
        if (curr_pos_.Distance(Point2D<double>(cached_trajectory_->Px()[0],
                                               cached_trajectory_->Py()[0])) > 1.0e-6){
            cached_trajectory_available_ = false;
        }

        // check if we want to go to the endpoint of the cached trajectory
        Point2D<double> end_point = to_waiting_point ? waiting_position_ : final_dest_;
        double d = end_point.Distance(Point2D<double>(
                cached_trajectory_->Px()[cached_trajectory_->NbSamples() - 1],
                cached_trajectory_->Py()[cached_trajectory_->NbSamples() - 1]));
        if (d > 1.0e-3){
            std::cout << "This happened or agent " << my_agent_idx_ << " (" << d << ")" << std::endl;
            cached_trajectory_available_ = false;
        }
    }

    planner_.StoreResetTrajectory();
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(to_waiting_point ? waiting_position_ : final_dest_);
    bool created_cached_trajectory_this_iteration = false;
    auto time_2 = std::chrono::high_resolution_clock::now();
    try{
        if (cached_trajectory_available_){
            // avoid replanning a trajectory we already planned
            planner_.SetCachedTrajectory(std::const_pointer_cast<Trajectory>(cached_trajectory_));

        } else {
            planner_.PlanSafely();
            if (Stationary() && planner_.GetLastSolution()->NbSamples() > 2
                    && curr_pos_.Distance(planner_.GetDest()) > 1.0e-2
                    && !planner_.EmergencyMode()){
                cached_trajectory_ = planner_.GetLastSolution();
                cached_trajectory_available_ = true;

                created_cached_trajectory_this_iteration = true;
            }

            // update mover task logging
            double planning_time = planner_.GetTotalComputationTime();
            if (curr_task_.get() != nullptr){
                curr_task_->NotifyPlanningOccured(curr_time_, planning_time);
            }
        }
    } catch (std::exception & e){
        if (!silent_mode_) std::cout << "Planning failed: " << e.what() << std::endl;
        // if we cannot plan, we need to set the state accordingly
        state_ = to_waiting_point ? FAILED_TO_PLAN_TO_WAITING_POINT : 
            FAILED_TO_PLAN_TO_DEST;
        #ifndef PARALLELIZE
        env_.ClearClaimingObject();
        #endif

        // update mover task logging
        double planning_time = planner_.GetTotalComputationTime();
        if (curr_task_.get() != nullptr){
            curr_task_->NotifyPlanningOccured(curr_time_, planning_time);
        }
        planner_.ResetTrajectory();
        return;
    }
    auto time_3 = std::chrono::high_resolution_clock::now();
    
    // check if something went wrong
    if (planner_.EmergencyMode()){
        // state_ = to_waiting_point ? FAILED_TO_PLAN_TO_WAITING_POINT : 
        //     FAILED_TO_PLAN_TO_DEST;

        // just ignore this planning attempt and revert to previous trajectory
        planner_.ResetEmergencyMode();
        state_ = state_before;
        planner_.ResetTrajectory();
    } else if (created_cached_trajectory_this_iteration) {
        // update stored info
        auto t1 = std::chrono::high_resolution_clock::now();
        planned_trajectories_.push_back(planner_.GetLastSolution());
        auto t2 = std::chrono::high_resolution_clock::now();
        planned_corridor_sequences_.push_back(planner_.GetCorridorSequence());
        auto t3 = std::chrono::high_resolution_clock::now();
        planned_times_.push_back(curr_time_);
        auto t4 = std::chrono::high_resolution_clock::now();
        /*
        std::cout << "\t\tTime for storing planned trajectory: " 
                  << std::chrono::duration<double, std::milli>(t2 - t1).count() 
                  << " ms" << std::endl;
        std::cout << "\t\tTime for storing planned corridor sequence: "
                    << std::chrono::duration<double, std::milli>(t3 - t2).count() 
                    << " ms" << std::endl;
        std::cout << "\t\tTime for storing planned time: "  
                    << std::chrono::duration<double, std::milli>(t4 - t3).count() 
                    << " ms" << std::endl;
        */
    }

    #ifndef PARALLELIZE
    env_.ClearClaimingObject();
    #endif
    /*
    auto time_4 = std::chrono::high_resolution_clock::now();
    double time_prep = std::chrono::duration<double, std::milli>(
        time_2 - time_1).count();
    double time_plan = std::chrono::duration<double, std::milli>(
        time_3 - time_2).count();
    double time_store = std::chrono::duration<double, std::milli>(
        time_4 - time_3).count();
    if (!silent_mode_) std::cout << "\t\tTime for preparation:  " << time_prep << " ms" << std::endl;
    if (!silent_mode_) std::cout << "\t\tTime for planning:     " << time_plan << " ms" << std::endl;
    if (!silent_mode_) std::cout << "\t\tTime for storing info: " << time_store << " ms" << std::endl;
    */
}

void Agent::UpdateWaitingPosition(bool wait_at_station, CorridorUnion const &intersection){
    if (wait_at_station){
        waiting_position_ = curr_pos_;
    } else {
        // if the current position is within the intersection, we are likely in deadlock. 
        // In that case, just take the current position as waiting position
        // which will make sure there are no collisions and the deadlock 
        // resolution will deal with the deadlock
        if (intersection.ContainsVehicle(curr_pos_, planner_.GetParameters())){
            waiting_position_ = curr_pos_;
        } else {
            waiting_position_ = planner_.GetCorridorSequence().GetWaitingPosition(
                planner_.GetStart(), intersection, 
                planner_.GetParameters(), 
                planner_.GetEnvironment().CellWidth(),
                planner_.GetEnvironment().CellHeight());
        }
        std::cout << "Got waiting position: " << waiting_position_ << " (current position: " << curr_pos_ << ")" << std::endl;

        // Check if this waiting point is reasonable far away, otherwise
        // we might be blocking an agent while not even making any 
        // significant progress
        double d = waiting_position_.Distance(curr_pos_);
        if (d < waiting_point_distance_cell_lengths_*env_.CellWidth()){
            // waiting_position_ = curr_pos_.ConvertWorldToCell(
            //     env_.CellWidth(), env_.CellHeight()).ConvertCellToWorld(
            //     env_.CellWidth(), env_.CellHeight());
            // std::cout << "modified waiting position to be (approximately) cell center: " << std::endl;
            // std::cout << "\twaiting position: " << waiting_position_ << std::endl;
            // std::cout << "\tcurrent position: " << curr_pos_ << std::endl;
            waiting_position_ = curr_pos_;
        }
    }
}

bool Agent::CheckForCollisionWithBlockingAgent(int start_idx, int stop_idx){
    // int nb_time_steps_to_check = std::max(
    //     GetRemainingTimeSteps(),
    //     blocking_agent_->GetRemainingTimeSteps()
    // );
    
    // for (int nb_steps_in_future = 0; 
    //         nb_steps_in_future < nb_time_steps_to_check; nb_steps_in_future++){
    //     GetVehicleFootprint(nb_steps_in_future, footprint_, 0);
    //     blocking_agent_->GetVehicleFootprint(nb_steps_in_future, 
    //                             blocking_footprint_, 0);

    //     if (footprint_.GetOverlap(blocking_footprint_, o)){
    //         if (!silent_mode_) std::cout << "[" << nb_steps_in_future << "] " << footprint_ << " " << blocking_footprint_ << std::endl;
    //         return true;
    //     }
    // }
    // return false;
    if (stop_idx < 0) stop_idx = std::max(
        GetRemainingTimeSteps(),
        blocking_agent_->GetRemainingTimeSteps()
    );

    // allocate containers
    Corridor footprint_this;
    Corridor footprint_other;
    Corridor o;

    // base case
    if (start_idx >= stop_idx){
        GetVehicleFootprint(start_idx, footprint_this, 0);
        blocking_agent_->GetVehicleFootprint(start_idx, footprint_other, 0);
        return footprint_this.GetOverlap(footprint_other, o);
    }

    // Otherwise, check bounding box around trajectories
    GetFootprintBoundingBox(start_idx, stop_idx, footprint_this);
    blocking_agent_->GetFootprintBoundingBox(start_idx, stop_idx, footprint_other);

    if (footprint_this.GetOverlap(footprint_other, o)){
        // recurse
        int mid_idx = (start_idx + stop_idx) / 2;
        return CheckForCollisionWithBlockingAgent(start_idx, mid_idx) ||
               CheckForCollisionWithBlockingAgent(mid_idx+1, stop_idx);
    } else {
        return false;
    }
};


















VirtualAgent::VirtualAgent(int idx, const Parameters& params,
             double collision_check_margin, std::shared_ptr<const Trajectory> trajectory) : 
        my_agent_idx_(idx), params_(params), prioritized_trajectory_(trajectory){
    collision_check_margin_ = collision_check_margin;
}
