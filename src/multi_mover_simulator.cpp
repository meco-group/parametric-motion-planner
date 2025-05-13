#include "core/multi_mover_simulator.hpp"
#include "core/corridor.hpp"

Agent::Agent(MotionPlanner* planner) : planner_(planner), blocking_agent_(nullptr){
    state_ = IDLING;
    final_dest_ = planner->GetStart();
}

void Agent::SetFinalDestination(const Point2D<double>& final_dest){
    final_dest_ = final_dest;
    state_ = READY_TO_PLAN;
}

const Point2D<double>& Agent::GetFinalDestination() const{
    return final_dest_;
}

const Point2D<double> Agent::GetCurrentPosition() const{
    return Point2D<double>(
        planner_->GetLastSolution().Px()[planner_->GetCurrentSampleIdx()],
        planner_->GetLastSolution().Py()[planner_->GetCurrentSampleIdx()]
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
    return planner_->GetLastSolution().NbSamples() - planner_->GetCurrentSampleIdx();
}

double Agent::GetTimeAtTimeStep(int future_time_step) const{
    double local_time = planner_->GetLastSolution().T()[planner_->GetCurrentSampleIdx() + future_time_step];
    return local_time + planner_->GetLastSolution().T0();
}

void Agent::WaitForAgent(Agent* blocking_agent, Corridor& intersection){
    state_ = MOVING_TO_WAITING_POINT;
    waiting_position_ = planner_->GetCorridorSequence().GetWaitingPosition(
        planner_->GetStart(), intersection, 
        planner_->GetParameters(), 
        planner_->GetEnvironment().CellWidth(),
        planner_->GetEnvironment().CellHeight());
    blocking_agent_ = blocking_agent;
    intersection_ = intersection;
}


void Agent::GetVehicleFootprint(int nb_time_steps_from_now, 
                                 Corridor& footprint, 
                                 double collision_check_margin){
    nb_time_steps_from_now = std::min(nb_time_steps_from_now, 
        planner_->GetLastSolution().NbSamples() - 1);
    double px = planner_->GetLastSolution().Px()[planner_->GetCurrentSampleIdx() + nb_time_steps_from_now];
    double py = planner_->GetLastSolution().Py()[planner_->GetCurrentSampleIdx() + nb_time_steps_from_now];
    double width_offset = planner_->GetParameters().GetWidthOffset() + collision_check_margin;
    double height_offset = planner_->GetParameters().GetHeightOffset() + collision_check_margin;
    footprint.SetXmin(px - width_offset);
    footprint.SetXmax(px + width_offset);
    footprint.SetYmin(py - height_offset);
    footprint.SetYmax(py + height_offset);
}














MultiMoverSimulator::MultiMoverSimulator(std::vector<MotionPlanner>& planners) : agents_(){
    for (auto& planner : planners){
        agents_.emplace_back(&planner);
    }
}

void MultiMoverSimulator::InstructAgentToDestination(int agent_idx, 
                                                      const Point2D<double>& final_dest){
    if (agent_idx < 0 || agent_idx >= agents_.size()){
        throw std::out_of_range("Agent index out of range");
    }
    agents_[agent_idx].SetFinalDestination(final_dest);
}

void MultiMoverSimulator::SimulateSteps(int nb_steps){
    std::vector<bool> new_trajectories(agents_.size(), false);
    for (int i = 0; i < nb_steps; i++){
        // Update trajectories if needed
        for (int j = 0; j < agents_.size(); j++){
            new_trajectories[j] = agents_[j].UpdateTrajectory();
        }

        // Check for new collisions
        ProcessPotentialNewCollsions(new_trajectories);

        // Check for deadlocks
        if (CheckIfDeadlockPresent()){
            std::cout << "Deadlock detected" << std::endl;
            throw std::runtime_error("Deadlock detected");
        }

        // Update positions of all agents
        UpdateSingleStep();
    }
}

void MultiMoverSimulator::UpdateSingleStep(){
    for (auto& agent : agents_){
        agent.SimulateStep();
    }
}

bool MultiMoverSimulator::ProcessPotentialNewCollsions(std::vector<bool>& new_trajectories){
    // TODO: improve the implementation below to make sure you always deal 
    // with the correct collisions
    // for example: if veh1 will first collide with veh2 and then with veh3, 
    // first process the collision with veh2. Only if veh2 will wait for veh1
    // should you then process the veh1-veh3 collision.
    // This example show the order in which the collisions are processed is
    // important. Otherwise veh3 might be waiting for veh1 while veh1 is 
    // waiting for veh2.
    for (int i = 0; i < agents_.size(); i++){
        if (new_trajectories[i]){
            for (int j = i + 1; j < agents_.size(); j++){
                if (CheckForCollision(i, j)){
                    DealWithCollision(i, j);
                }
            }
        }
    }

    return false;
}

bool MultiMoverSimulator::CheckForCollision(int agent_idx_1, int agent_idx_2){
    int nb_time_steps_to_check = std::max(
        agents_[agent_idx_1].GetRemainingTimeSteps(),
        agents_[agent_idx_2].GetRemainingTimeSteps()
    );
    
    Corridor footprint_1;
    Corridor footprint_2;
    Corridor o;
    for (int nb_steps_in_future = 0; 
            nb_steps_in_future < nb_time_steps_to_check; nb_steps_in_future++){
        agents_[agent_idx_1].GetVehicleFootprint(nb_steps_in_future, footprint_1, collision_check_margin_);
        agents_[agent_idx_2].GetVehicleFootprint(nb_steps_in_future, footprint_2, collision_check_margin_);

        if (footprint_1.GetOverlap(footprint_2, o)){
            return true;
        }
    }

    return false;
}

void MultiMoverSimulator::DealWithCollision(int agent_idx_1, int agent_idx_2){
    // get the intersection of the two vehicles
    Corridor intersection;
    bool intersection_present = GetIntersection(agent_idx_1, agent_idx_2, intersection);
    if (!intersection_present){
        throw std::runtime_error("No intersection found between agent " + 
            std::to_string(agent_idx_1) + " and agent " + std::to_string(agent_idx_2));
    }

    // get the intersection case
    CollisionResolutionDecision intersection_case = 
                GetIntersectionCase(agent_idx_1, agent_idx_2, intersection);
    Point2D<double> waiting_position;
    
    if (intersection_case == INVALID){
        throw std::runtime_error("Invalid intersection case");

    } else if (intersection_case == NO_OVERLAP){
        // no action needed
        return;

    } else if (intersection_case == AGENT_1_MUST_WAIT){
        // instruct vehicle 1 to wait
        agents_[agent_idx_1].WaitForAgent(&agents_[agent_idx_2], intersection);

    } else if (intersection_case == AGENT_2_MUST_WAIT){
        // instruct vehicle 2 to wait
        agents_[agent_idx_2].WaitForAgent(&agents_[agent_idx_1], intersection);
    }
}

bool MultiMoverSimulator::GetIntersection(int agent_idx_1, int agent_idx_2, 
                                          Corridor& intersection){
    CorridorSequence seq_1 = agents_[agent_idx_1].GetCorridorSequence();
    CorridorSequence seq_2 = agents_[agent_idx_2].GetCorridorSequence();
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

    return true;
}

CollisionResolutionDecision MultiMoverSimulator::GetIntersectionCase(int agent_idx_1, int agent_idx_2, 
                                          Corridor const &intersection){
    std::map<std::string, double> times_1 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_1, intersection);
    std::map<std::string, double> times_2 =
            GetTimeEnteringAndLeavingIntersection(agent_idx_2, intersection);

    // Check if both vehicles actually enter the intersection
    if (times_1["entering_time"] < 0 && times_2["entering_time"] < 0){
        return NO_OVERLAP;
    }

    // If both vehicles stay in the intersection we're in trouble
    if (times_1["leaving_time"] < 0 && times_2["leaving_time"] < 0){
        return INVALID;
    }

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
    if (times_1["leaving_time"] < times_2["leaving_time"]){
        return AGENT_2_MUST_WAIT;
    } else {
        return AGENT_1_MUST_WAIT;
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
    int nb_steps_remaining = agents_[agent_idx].GetRemainingTimeSteps();
    int nb_time_steps_from_now = 0;
    agents_[agent_idx].GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    while (!intersection.GetOverlap(footprint, o)){
        nb_time_steps_from_now++;
        if (nb_time_steps_from_now >= nb_steps_remaining){
            return result;
        }
    }
    result["entering_time"] = agents_[agent_idx].GetTimeAtTimeStep(nb_time_steps_from_now);

    // find the time when the vehicle leaves the intersection
    while (intersection.GetOverlap(footprint, o)){
        nb_time_steps_from_now++;
        if (nb_time_steps_from_now >= nb_steps_remaining){
            return result;
        }
    }
    result["leaving_time"] = agents_[agent_idx].GetTimeAtTimeStep(nb_time_steps_from_now);

    return result;
}

bool MultiMoverSimulator::CheckIfDeadlockPresent(){
    std::vector<bool> processed_agents(agents_.size(), false);

    // keep going as long as not all agents have been checked
    Agent curr_agent;
    while (std::find(processed_agents.begin(), processed_agents.end(), false) != processed_agents.end()){
        // find the first agent that is not processed yet
        int original_agent_idx = std::find(processed_agents.begin(), processed_agents.end(), false) - processed_agents.begin();

        curr_agent = agents_[original_agent_idx];
        processed_agents[original_agent_idx] = true;
        while (curr_agent.GetState() == MOVING_TO_WAITING_POINT || 
                curr_agent.GetState() == WAITING_AT_INTERSECTION){
            // get the agent for which the current agent is waiting
            curr_agent = curr_agent.GetBlockingAgent();
            int curr_agent_idx = std::find(agents_.begin(), agents_.end(), curr_agent) - agents_.begin();
            processed_agents[curr_agent_idx] = true;

            // check if the current agent is waiting for the original agent
            if (curr_agent_idx == original_agent_idx){
                // deadlock found
                return true;
            }
        }
    }

    return false;
}