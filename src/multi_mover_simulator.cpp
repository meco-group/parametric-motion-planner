#include "core/multi_mover_simulator.hpp"
#include "core/corridor.hpp"

std::string DecisionToString(CollisionResolutionDecision d){
    if (d == AGENT_1_MUST_WAIT){
        return "AGENT_1_MUST_WAIT";
    } else if (d == AGENT_2_MUST_WAIT){
        return "AGENT_2_MUST_WAIT";
    } else if (d == NO_OVERLAP){
        return "NO_OVERLAP";
    } else if (d == INVALID){
        return "INVALID";
    } else {
        return "?";
    }
}

std::string AgentStateToString(AgentState s){
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


Agent::Agent(Environment& env, const Parameters& params,
             double collision_check_margin, 
             Point2D<double> starting_position) : 
        env_(env), planner_(params, env), blocking_agent_(nullptr){
    state_ = IDLING;
    final_dest_ = starting_position;
    curr_pos_ = starting_position;
    collision_check_margin_ = collision_check_margin;
    planner_.SetMaxNbCorridorGrowingIterations(1);

    travelled_trajectory_.Reset(starting_position);
}

bool Agent::InstructToDestination(const std::string& destination_name,
                                  const Point2D<double>& final_dest){
    if (state_ != IDLING && state_ != WAITING_FOR_FREE_DESTINATION){
        throw std::runtime_error("Cannot change final destination when not in IDLING state");
    }

    // Check if we can claim the destination in the environment
    if (!env_.ClaimDestination(destination_name, this)){
        // if we were unable to claim the destination, update the state and
        // return false
        state_ = WAITING_FOR_FREE_DESTINATION;
        return false;
    }

    // check if we were already claiming a destination, if so store it such
    // that we can release it after replanning
    // we cannot release it yet because that would cause an invalid starting 
    // point in the environment
    if (currently_claiming_){
        destinations_to_be_released_.push_back(claimed_destination_name_);
    }

    // if we were able to claim the destination, take note of this such that
    // we can release it later
    currently_claiming_ = true;
    claimed_destination_name_ = destination_name;

    final_dest_ = final_dest;
    state_ = READY_TO_PLAN;

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

const Corridor& Agent::GetIntersection() const{
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
                         Corridor& intersection){
    if (state_ == IDLING){
        throw std::runtime_error("Agent cannot wait for another agent when idling");
    }
    if (state_ == WAITING_AT_INTERSECTION || state_ == MOVING_TO_WAITING_POINT){
        std::cout << "WARNING: This agent was already waiting for another agent" << std::endl;
        // throw std::runtime_error("Agent cannot wait for another agent when idling or already waiting");
    }
    if (blocking_agent_idx < 0){
        throw std::runtime_error("Invalid blocking agent index");
    }
    state_ = MOVING_TO_WAITING_POINT;
    waiting_position_ = planner_.GetCorridorSequence().GetWaitingPosition(
        planner_.GetStart(), intersection, 
        planner_.GetParameters(), 
        planner_.GetEnvironment().CellWidth(),
        planner_.GetEnvironment().CellHeight());
    blocking_agent_ = blocking_agent;
    blocking_agent_idx_ = blocking_agent_idx;
    intersection_ = intersection;

    PlanToDestination(true);
}

void Agent::SimulateStep(){
    nb_simulated_samples_++;
    planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
    curr_time_ = nb_simulated_samples_*planner_.GetLastSolution().Dt();
    travelled_trajectory_.Append(curr_time_, curr_pos_.x(), curr_pos_.y(), 
                                 curr_vel_.x(), curr_vel_.y(), 
                                 curr_acc_.x(), curr_acc_.y());
    
    // check for potential state changes
    if (state_ == MOVING_TO_FINAL_DESTINATION && 
            curr_pos_.Distance(final_dest_) <= 1.0e-3 && 
            curr_vel_.Norm() <= 1.0e-2){
        // we have reached the final destination
        state_ = IDLING;
    } else if (state_ == MOVING_TO_WAITING_POINT && 
            curr_pos_.Distance(waiting_position_) <= 1.0e-3 && 
            curr_vel_.Norm() <= 1.0e-2){
        // we have reached the waiting point
        state_ = WAITING_AT_INTERSECTION;
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
        GetVehicleFootprint(0, footprint, 0.01);
        if (!cell.GetOverlap(footprint, o)){
            // we can release the destination
            indices_to_be_released.push_back(i);
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
        return true;
    }

    // otherwise, we are waiting for the blocking agent, so we can try to see
    // if we can continue
    replanning_step_counter_++;
    if (replanning_step_counter_ >= replanning_frequency_){
        replanning_step_counter_ = 0;

        // check if we can replan
        if (wait_for_clear_intersection_){
            blocking_agent_->GetVehicleFootprint(0, blocking_footprint_, 
                                                 collision_check_margin_);
            if (blocking_footprint_.GetOverlap(intersection_, o)){
                // blocking agent is still in the intersection
                return false;
            } else {
                // blocking agent is out of the intersection
                PlanToDestination();
                return true;
            }
        } else {
            // Attempt to plan a new trajectory and check for collision
            PlanToDestination();

            if (CheckForCollisionWithBlockingAgent()){
                std::cout << "Collision detected with blocking agent, replanning..." << std::endl;
                PlanToDestination(true);
                // discard first trajectory sample
                planner_.GetSample(t, curr_pos_, curr_vel_, curr_acc_);
                return false; // TODO: actually, the new trajectory might be
                // different than the one before. It would be better to 
                // actually revert to the previous trajectory. This would also
                // be much more efficient
            } else {
                // no collision detected, we can continue
                return true;
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
    env_.SetClaimingObject(this);
    state_ = to_waiting_point ? MOVING_TO_WAITING_POINT : MOVING_TO_FINAL_DESTINATION;
    planner_.SetStart(curr_pos_);
    planner_.SetStartVel(curr_vel_);
    planner_.SetDest(to_waiting_point ? waiting_position_ : final_dest_);
    planner_.PlanSafely();
    
    // check if something went wrong
    if (planner_.EmergencyMode()){
        state_ = to_waiting_point ? FAILED_TO_PLAN_TO_WAITING_POINT : 
            FAILED_TO_PLAN_TO_DEST;
    }

    // // if we planned succesfully, release destinations to be released
    // if (state_ != FAILED_TO_PLAN_TO_WAITING_POINT && 
    //         state_ != FAILED_TO_PLAN_TO_DEST){
    //     for (const auto& dest_name : destinations_to_be_released_){
    //         env_.ReleaseDestination(dest_name, this);
    //     }
    //     destinations_to_be_released_.clear();
    // }

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
        GetVehicleFootprint(nb_steps_in_future, footprint_, 
                            collision_check_margin_);
        blocking_agent_->GetVehicleFootprint(nb_steps_in_future, 
                                blocking_footprint_, collision_check_margin_);

        if (nb_steps_in_future == 124){
            std::cout << "[" << nb_steps_in_future << "] " << footprint_ << " " << blocking_footprint_ << std::endl;
        }

        if (footprint_.GetOverlap(blocking_footprint_, o)){
            std::cout << "[" << nb_steps_in_future << "] " << footprint_ << " " << blocking_footprint_ << std::endl;
            return true;
        }
    }
    return false;
};














MultiMoverSimulator::MultiMoverSimulator(Environment& environment,
                std::vector<const Parameters*> params,
                std::map<std::string, Point2D<int>> possible_destinations,
                std::vector<std::string> starting_positions,
                std::vector<MoverTask> tasks)
                : agents_(), env_(environment), params_(params),
                  possible_destinations_(possible_destinations){   
    // check arguments
    if (params.size() != starting_positions.size()){
        throw std::invalid_argument("Number of planners and starting positions must match");
    }
    if (possible_destinations.size() <= starting_positions.size()){
        throw std::invalid_argument("Number of starting positions must match number of possible destinations");
    }
    
    // initialize agents
    for (int i = 0; i < params.size(); i++){
        agents_.emplace_back(
            std::make_shared<Agent>(env_, *params_[i], collision_check_margin_, 
                possible_destinations_[starting_positions[i]].ConvertCellToWorld(
                                        env_.CellWidth(), env_.CellHeight())
            )
        );
    }
    new_trajectories_ = std::vector<bool>(agents_.size(), false);
    
    // initialize tasks
    tasks_ = tasks;

    // add all possible destinations to the environment
    for (const auto& [name, pos] : possible_destinations){
        env_.AddClaimableDestination(name, pos);
    }

    // set final destinations as current positions
    bool success = false;
    for (int i = 0; i < agents_.size(); i++){
        success = agents_[i]->InstructToDestination(starting_positions[i],
            possible_destinations[starting_positions[i]].ConvertCellToWorld(
                env_.CellWidth(), env_.CellHeight()
            )
        );
        if (!success){
            std::cout << "Failed to set initial destination for agent " 
                + std::to_string(i) + " to " + starting_positions[i] << std::endl;
            throw std::runtime_error("Failed to set initial destination for agent " 
                                     + std::to_string(i) + " to " 
                                     + starting_positions[i]);
        }
    }
}

void MultiMoverSimulator::InstructAgentToDestination(int agent_idx,
                                        const std::string& destination_name){
    agents_[agent_idx]->InstructToDestination(destination_name,
        possible_destinations_[destination_name].ConvertCellToWorld(
                                        env_.CellWidth(), env_.CellHeight()));
}

void MultiMoverSimulator::SimulateSteps(int nb_steps, bool stop_when_all_idling){
    for (int i = 0; i < nb_steps; i++){
        SimulateSingleStep();

        // Check if all agents are idling
        if (AllAgentsIdling()){
            std::cout << "All agents are idling --> returning (" << i << "/" << nb_steps << ")" << std::endl;
            return;
        }

        nb_simulated_samples_++;
    }
}

void MultiMoverSimulator::SimulateAllTasks(){
    bool all_tasks_completed = false;
    int max_nb_steps = 1000;//10000;
    int step_counter = 0;
    while (!all_tasks_completed && step_counter < max_nb_steps){
        // std::cout << "Simulating step " << step_counter << std::endl;
        // check if new tasks are available
        ProcessPotentialNewTasks();

        SimulateSingleStep();

        // check if all tasks are completed
        if (AllTasksRevealed() && AllAgentsIdling()){
            all_tasks_completed = true;
        }

        nb_simulated_samples_++;
        step_counter++;
    }
}

void MultiMoverSimulator::DumpToJson(std::string const &filename) const {
    json j;
    j["agents"] = json::array();
    for (int i = 0; i < agents_.size(); i++){
        j["agents"].push_back(agents_[i]->ToJson());
    }
    j["intersection_logs"] = json::array();
    for (int i = 0; i < intersection_logs_.size(); i++){
        j["intersection_logs"].push_back(intersection_logs_[i].ToJson());
    }
    j["claimed_destinations_info"] = json::array();
    for (const auto& info : claimed_destinations_info_){
        j["claimed_destinations_info"].push_back(info);
    }
    std::ofstream o(filename);
    o << std::setw(4) << j << std::endl;
}

void MultiMoverSimulator::SimulateSingleStep(){
    // Update trajectories if needed
    for (int j = 0; j < agents_.size(); j++){
        new_trajectories_[j] = agents_[j]->UpdateTrajectory();
    }
    // std::cout << "updated trajectories: " << new_trajectories << std::endl;

    // Check for new collisions
    ProcessPotentialNewCollsions();

    // Check for deadlocks
    if (CheckIfDeadlockPresent()){
        std::cout << "Deadlock detected" << std::endl;
        throw std::runtime_error("Deadlock detected");
    }

    claimed_destinations_info_.push_back(env_.ClaimableDestinationsToJson());

    // Update positions of all agents
    UpdateSingleStep();
}

void MultiMoverSimulator::UpdateSingleStep(){
    for (auto& agent : agents_){
        agent->SimulateStep();
    }
}

bool MultiMoverSimulator::ProcessPotentialNewCollsions(){
    // TODO: improve the implementation below to make sure you always deal 
    // with the correct collisions
    // for example: if veh1 will first collide with veh2 and then with veh3, 
    // first process the collision with veh2. Only if veh2 will wait for veh1
    // should you then process the veh1-veh3 collision.
    // This example show the order in which the collisions are processed is
    // important. Otherwise veh3 might be waiting for veh1 while veh1 is 
    // waiting for veh2.
    // return false;
    for (int i = 0; i < agents_.size(); i++){
        if (new_trajectories_[i]){
            for (int j = 0; j < agents_.size(); j++){
                if (i == j){
                    continue;
                }
                if (CheckForCollision(i, j)){
                    DealWithCollision(i, j);
                }
            }
        }
    }

    return false;
}

bool MultiMoverSimulator::CheckForCollision(int agent_idx_1, int agent_idx_2){
    std::cout << "Checking for collision between agents " << agent_idx_1;
    std::cout << " and " << agent_idx_2 << std::endl;
    int nb_time_steps_to_check = std::max(
        agents_[agent_idx_1]->GetRemainingTimeSteps(),
        agents_[agent_idx_2]->GetRemainingTimeSteps()
    );
    
    Corridor footprint_1;
    Corridor footprint_2;
    Corridor o;
    for (int nb_steps_in_future = 0; 
            nb_steps_in_future < nb_time_steps_to_check; nb_steps_in_future++){
        agents_[agent_idx_1]->GetVehicleFootprint(nb_steps_in_future, footprint_1, collision_check_margin_);
        agents_[agent_idx_2]->GetVehicleFootprint(nb_steps_in_future, footprint_2, collision_check_margin_);

        if (footprint_1.GetOverlap(footprint_2, o)){
            return true;
        }
    }
    std::cout << "\tno collision!" << std::endl;

    return false;
}

void MultiMoverSimulator::DealWithCollision(int agent_idx_1, int agent_idx_2){
    // get the intersection of the two vehicles
    Corridor intersection;
    bool intersection_present = GetIntersection(agent_idx_1, agent_idx_2, intersection);
    if (!intersection_present){
        std::cout << "no intersection found between agents " << agent_idx_1;
        std::cout << " and " << agent_idx_2 << std::endl;
        return;
        // throw std::runtime_error("No intersection found between agent " + 
        //     std::to_string(agent_idx_1) + " and agent " + std::to_string(agent_idx_2));
    }

    // get the intersection case
    std::cout << "Dealing with collision between agents " << agent_idx_1;
    std::cout << " and " << agent_idx_2 << std::endl;
    CollisionResolutionDecision intersection_case = 
                GetIntersectionCase(agent_idx_1, agent_idx_2, intersection);
    Point2D<double> waiting_position;
    std::cout << "Decision: " << DecisionToString(intersection_case) << std::endl; 
    
    if (intersection_case == INVALID){
        // throw std::runtime_error("Invalid intersection case");
        std::cout << "Invalid intersection case" << std::endl;

    } else if (intersection_case == NO_OVERLAP){
        // no action needed
        return;

    } else if (intersection_case == AGENT_1_MUST_WAIT){
        // instruct vehicle 1 to wait
        agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], agent_idx_2,
                                          intersection);

    } else if (intersection_case == AGENT_2_MUST_WAIT){
        // instruct vehicle 2 to wait
        agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], agent_idx_1,
                                          intersection);
    } else {
        throw std::runtime_error("Invalid CollisionResolutionDecision: Don't know what to do");
    }
}

bool MultiMoverSimulator::GetIntersection(int agent_idx_1, int agent_idx_2, 
                                          Corridor& intersection){
    CorridorSequence seq_1 = agents_[agent_idx_1]->GetCorridorSequence();
    CorridorSequence seq_2 = agents_[agent_idx_2]->GetCorridorSequence();
    std::vector<Corridor> overlaps = seq_1.GetOverlap(seq_2);

    if (overlaps.size() == 0){
        return false;
    }

    intersection = overlaps[0].Copy();
    for (int i = 1; i < overlaps.size(); i++){
        intersection.SetXmin(std::min(intersection.Xmin(), overlaps[i].Xmin()));
        intersection.SetXmax(std::max(intersection.Xmax(), overlaps[i].Xmax()));
        intersection.SetYmin(std::min(intersection.Ymin(), overlaps[i].Ymin()));
        intersection.SetYmax(std::max(intersection.Ymax(), overlaps[i].Ymax()));
    }

    // intersections can never cover edges of the environment
    double cell_width = env_.CellWidth();
    intersection.SetXmin(std::max(intersection.Xmin(), cell_width));
    intersection.SetXmax(std::min(intersection.Xmax(), env_.NbCellCols() - cell_width));
    intersection.SetYmin(std::max(intersection.Ymin(), cell_width));
    intersection.SetYmax(std::min(intersection.Ymax(), env_.NbCellRows() - cell_width));
    if (intersection.GetArea() == 0){
        return false;
    }

    return true;
}

CollisionResolutionDecision MultiMoverSimulator::GetIntersectionCase(
        int agent_idx_1, int agent_idx_2, Corridor const &intersection){
    std::map<std::string, double> times_1 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_1, intersection);
    std::map<std::string, double> times_2 =
            GetTimeEnteringAndLeavingIntersection(agent_idx_2, intersection);
    std::cout << "intersection: " << intersection << std::endl;
    std::cout << "times_1: " << times_1 << std::endl;
    std::cout << "times_2: " << times_2 << std::endl;

    // Check if both vehicles actually enter the intersection
    if (times_1["entering_time"] < 0 || times_2["entering_time"] < 0){
        return NO_OVERLAP;
    }

    // If both vehicles stay in the intersection we're in trouble
    if (times_1["leaving_time"] < 0 && times_2["leaving_time"] < 0){
        return INVALID;
    }

    IntersectionLog log(agent_idx_1, agent_idx_2,
        nb_simulated_samples_*simulation_time_step_, 
        std::max(times_1["leaving_time"], times_2["leaving_time"]), 
        intersection);
    intersection_logs_.push_back(log);

    // If one vehicle never leaves, that one must wait
    if (times_1["leaving_time"] < 0){
        return AGENT_1_MUST_WAIT;
    }

    if (times_2["leaving_time"] < 0){
        return AGENT_2_MUST_WAIT;
    }

    // Check if vehicles plan to be in intersection at the same time
    if (times_1["leaving_time"] < times_2["entering_time"] ||
            times_2["leaving_time"] < times_1["entering_time"]){
        return NO_OVERLAP;
    }

    // Determine which vehicle must wait
    if (agents_[agent_idx_1]->Stationary() && 
            agents_[agent_idx_2]->Stationary()){
        if (times_1["leaving_time"] < times_2["leaving_time"]){
            return AGENT_2_MUST_WAIT;
        } else {
            return AGENT_1_MUST_WAIT;
        }
    } else if (agents_[agent_idx_1]->Stationary()){
        // only agent_1 is not moving
        return AGENT_1_MUST_WAIT;
    } else if (agents_[agent_idx_2]->Stationary()){
        return AGENT_2_MUST_WAIT;
    } else {
        // TODO: if both vehicles are moving, determine which one can safely
        // brake (actually, this is not allowed to happen)
        std::cout << "\tWARNING: Both vehicles are moving" << std::endl;
        if (times_1["leaving_time"] < times_2["leaving_time"]){
            return AGENT_2_MUST_WAIT;
        } else {
            return AGENT_1_MUST_WAIT;
        }
    }
}

std::map<std::string, double> MultiMoverSimulator::GetTimeEnteringAndLeavingIntersection(
        int agent_idx, Corridor const &intersection){
    std::map<std::string, double> result;
    result["entering_time"] = -1;
    result["leaving_time"] = -1;

    // find the time when the vehicle enters the intersection
    Corridor footprint;
    Corridor o;
    int nb_steps_remaining = agents_[agent_idx]->GetRemainingTimeSteps();
    int nb_time_steps_from_now = 0;
    agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    while (!intersection.GetOverlap(footprint, o)){
        nb_time_steps_from_now++;
        if (nb_time_steps_from_now >= nb_steps_remaining){
            return result;
        }
        agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    }
    // result["entering_time"] = agents_[agent_idx]->GetTimeAtTimeStep(nb_time_steps_from_now);
    result["entering_time"] = (nb_simulated_samples_ + nb_time_steps_from_now)*
        simulation_time_step_;

    // find the time when the vehicle leaves the intersection
    // (start at the end of the trajectory)
    int nb_steps_until_entering = nb_time_steps_from_now;
    nb_time_steps_from_now = nb_steps_remaining - 1;
    agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    while (!intersection.GetOverlap(footprint, o)){
        nb_time_steps_from_now--;
        if (nb_time_steps_from_now <= nb_steps_until_entering){
            return result;
        }
        agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    }
    // result["leaving_time"] = agents_[agent_idx]->GetTimeAtTimeStep(nb_time_steps_from_now);
    result["leaving_time"] = (nb_simulated_samples_ + nb_time_steps_from_now)*
        simulation_time_step_;

    return result;
}

bool MultiMoverSimulator::CheckIfDeadlockPresent(){
    std::vector<bool> processed_agents(agents_.size(), false);

    // keep going as long as not all agents have been checked
    int curr_agent_idx = 0;
    while (std::find(processed_agents.begin(), processed_agents.end(), false) != processed_agents.end()){
        // find the first agent that is not processed yet
        int original_agent_idx = std::find(processed_agents.begin(), processed_agents.end(), false) - processed_agents.begin();

        processed_agents[original_agent_idx] = true;
        while (agents_[curr_agent_idx]->GetState() == MOVING_TO_WAITING_POINT || 
                agents_[curr_agent_idx]->GetState() == WAITING_AT_INTERSECTION){
            // get the agent for which the current agent is waiting
            int temp = curr_agent_idx;
            curr_agent_idx = agents_[curr_agent_idx]->GetBlockingAgentIdx();

            // check if the current agent is waiting for the original agent
            if (curr_agent_idx == original_agent_idx){
                // deadlock found
                return true;
            }
        }
    }

    return false;
}

bool MultiMoverSimulator::AllAgentsIdling() const{
    for (int i = 0; i < agents_.size(); i++){
        if (agents_[i]->GetState() != IDLING){
            return false;
        }
    }
    return true;
}

void MultiMoverSimulator::ProcessPotentialNewTasks(){
    for (MoverTask& task : tasks_){
        if (!task.HasBeenRevealed() && 
                task.RevealTask(nb_simulated_samples_*simulation_time_step_)){
            // check if the agent is ready for the new task
            if (agents_[task.GetAgentIdx()]->GetState() == IDLING || 
                    agents_[task.GetAgentIdx()]->GetState() == WAITING_FOR_FREE_DESTINATION){
                // set the final destination
                std::string name = task.GetDestinationName();
                Point2D<double> dest = possible_destinations_[name].
                    ConvertCellToWorld(env_.CellWidth(), env_.CellHeight());
                bool success = agents_[task.GetAgentIdx()]->
                                            InstructToDestination(name, dest);
                if (!success){
                    // unable to claim destination, try again later
                    std::cout << "agent " << task.GetAgentIdx() << " unable to claim destination " 
                        << name << std::endl;
                    std::cout << "agents state: " << AgentStateToString(
                        agents_[task.GetAgentIdx()]->GetState()) << std::endl;
                    task.PostponeTask(0.1);
                }
            } else {
                // postpone the task
                task.PostponeTask(
                    agents_[task.GetAgentIdx()]->GetRemainingTimeSteps()*
                    simulation_time_step_);
            }
        }
    }
}

bool MultiMoverSimulator::AllTasksRevealed() const{
    for (int i = 0; i < tasks_.size(); i++){
        if (!tasks_[i].HasBeenRevealed()){
            return false;
        }
    }
    return true;
}