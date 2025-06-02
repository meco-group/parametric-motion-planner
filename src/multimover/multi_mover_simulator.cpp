#include "core/multimover/multi_mover_simulator.hpp"
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
            std::make_shared<Agent>(i, env_, *params_[i], collision_check_margin_, 
                possible_destinations_[starting_positions[i]].ConvertCellToWorld(
                                        env_.CellWidth(), env_.CellHeight())
            )
        );
    }
    new_trajectories_ = std::vector<bool>(agents_.size(), false);
    
    // initialize tasks
    for (const auto& task : tasks){
        tasks_.emplace_back(std::make_unique<MoverTask>(task));
    }

    // add all possible destinations to the environment
    for (const auto& [name, pos] : possible_destinations){
        env_.AddClaimableDestination(name, pos);
    }

    // set final destinations as current positions
    bool success = false;
    for (int i = 0; i < agents_.size(); i++){
        std::shared_ptr<MoverTask> task = nullptr;
        success = agents_[i]->InstructToDestination(starting_positions[i],
            possible_destinations[starting_positions[i]].ConvertCellToWorld(
                env_.CellWidth(), env_.CellHeight()
            ), 
            // no task with logging required
            task
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
    bool deadlock_detected = false;
    while (!all_tasks_completed && step_counter < max_nb_steps && !deadlock_detected){
        // std::cout << "Simulating step " << step_counter << std::endl;
        // check if new tasks are available
        ProcessPotentialNewTasks();

        deadlock_detected = SimulateSingleStep();

        // check if all tasks are completed
        if (AllTasksRevealed() && AllAgentsIdling()){
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                             "All tasks completed and all agents are idling");
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
    j["tasks"] = json::array();
    for (const auto& task : tasks_){
        j["tasks"].push_back(task->ToJson());
    }
    std::ofstream o(filename);
    o << std::setw(4) << j << std::endl;
}

bool MultiMoverSimulator::SimulateSingleStep(){
    // Update trajectories if needed
    for (int j = 0; j < agents_.size(); j++){
        new_trajectories_[j] = agents_[j]->UpdateTrajectory();
        if (new_trajectories_[j]){
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Agent " + std::to_string(j) + " updated trajectory");
        }
    }
    // std::cout << "updated trajectories: " << new_trajectories << std::endl;

    // Check for new collisions
    ProcessPotentialNewCollsions();

    // Check for deadlocks
    std::vector<MoverTask> deadlock_resolving_tasks;
    if (CheckIfDeadlockPresent(deadlock_resolving_tasks)){
        std::cout << "Deadlock detected" << std::endl;
        // throw std::runtime_error("Deadlock detected");
        if (deadlock_resolving_tasks.size() > 0){
            std::cout << "Attempting to resolve deadlock" << std::endl;
            // tasks_.insert(tasks_.end(), deadlock_resolving_tasks.begin(), 
            //               deadlock_resolving_tasks.end());
            for (const auto& task : deadlock_resolving_tasks){
                tasks_.emplace_back(std::make_shared<MoverTask>(task));
            }
        } else {
            return true;
        }
    }

    claimed_destinations_info_.push_back(env_.ClaimableDestinationsToJson());

    // Update positions of all agents
    UpdateSingleStep();

    return false;
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
    CorridorUnion intersection;
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
    logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
        "Collision found between agents " + std::to_string(agent_idx_1) + 
        " and " + std::to_string(agent_idx_2) + 
        " with decision: " + DecisionToString(intersection_case));
    
    if (intersection_case == INVALID){
        // throw std::runtime_error("Invalid intersection case");
        std::cout << "Invalid intersection case" << std::endl;

    } else if (intersection_case == NO_OVERLAP){
        // no action needed
        return;

    } else if (intersection_case == AGENT_1_MUST_WAIT){
        // instruct vehicle 1 to wait
        try{
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                               agent_idx_2, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Instructed agent " + std::to_string(agent_idx_1) + 
                " to wait for agent " + std::to_string(agent_idx_2));
        } catch (UnableToFindWaitingPoint& e){
            // agent_idx_1 cannot find a waiting point, so agent_idx_2 must 
            // longer
            if (!agents_[agent_idx_2]->Stationary()){
                std::cout << "Agent " << agent_idx_1 << " cannot find a waiting point, so agent " 
                          << agent_idx_2 << " must wait longer" << std::endl;
            }
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Agent " + std::to_string(agent_idx_1) + 
                " cannot find a waiting point, so agent " + std::to_string(agent_idx_2) + 
                " must wait longer");
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                               agent_idx_1, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Instructed agent " + std::to_string(agent_idx_2) + 
                " to wait for agent " + std::to_string(agent_idx_1));
        }

    } else if (intersection_case == AGENT_2_MUST_WAIT){
        // instruct vehicle 2 to wait
        try{
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                               agent_idx_1, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Instructed agent " + std::to_string(agent_idx_2) + 
                " to wait for agent " + std::to_string(agent_idx_1));
        } catch (UnableToFindWaitingPoint& e){
            // agent_idx_2 cannot find a waiting point, so agent_idx_1 must 
            // longer
            if (!agents_[agent_idx_1]->Stationary()){
                std::cout << "Agent " << agent_idx_2 << " cannot find a waiting point, so agent " 
                          << agent_idx_1 << " must wait longer" << std::endl;
            }
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Agent " + std::to_string(agent_idx_2) + 
                " cannot find a waiting point, so agent " + std::to_string(agent_idx_1) + 
                " must wait longer");
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                               agent_idx_2, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Instructed agent " + std::to_string(agent_idx_1) + 
                " to wait for agent " + std::to_string(agent_idx_2));
        }
    } else {
        throw std::runtime_error("Invalid CollisionResolutionDecision: Don't know what to do");
    }
}

bool MultiMoverSimulator::GetIntersection(int agent_idx_1, int agent_idx_2, 
                                          CorridorUnion& intersection){
    CorridorSequence seq_1 = agents_[agent_idx_1]->GetCorridorSequence();
    CorridorSequence seq_2 = agents_[agent_idx_2]->GetCorridorSequence();
    intersection = seq_1.GetOverlap(seq_2);

    // if (overlaps.size() == 0){
    //     return false;
    // }

    // Create single corridor around all overlaps --> too conservative
    // intersection = overlaps[0].Copy();
    // for (int i = 1; i < overlaps.size(); i++){
    //     intersection.SetXmin(std::min(intersection.Xmin(), overlaps[i].Xmin()));
    //     intersection.SetXmax(std::max(intersection.Xmax(), overlaps[i].Xmax()));
    //     intersection.SetYmin(std::min(intersection.Ymin(), overlaps[i].Ymin()));
    //     intersection.SetYmax(std::max(intersection.Ymax(), overlaps[i].Ymax()));
    // }

    // return true;

    return !intersection.IsEmpty();
}

CollisionResolutionDecision MultiMoverSimulator::GetIntersectionCase(
        int agent_idx_1, int agent_idx_2, 
        CorridorUnion const &intersection){
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
    bool new_log_entry = true;
    for (IntersectionLog l : intersection_logs_){
        if (l == log){
            new_log_entry = false;
            break;
        }
    }
    if (new_log_entry){ intersection_logs_.push_back(log);}

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
        int agent_idx, CorridorUnion const &intersection){
    std::map<std::string, double> result;
    result["entering_time"] = -1;
    result["leaving_time"] = -1;

    // find the time when the vehicle enters the intersection
    Corridor footprint;
    int nb_steps_remaining = agents_[agent_idx]->GetRemainingTimeSteps();
    int nb_time_steps_from_now = 0;
    agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    while (!intersection.OverlapsWith(footprint)){
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
    while (!intersection.OverlapsWith(footprint)){
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

bool MultiMoverSimulator::CheckIfDeadlockPresent(std::vector<MoverTask> &deadlock_resolving_tasks){
    std::vector<bool> processed_agents(agents_.size(), false);

    deadlock_resolving_tasks = {};

    // keep going as long as not all agents have been checked
    int curr_agent_idx = 0;
    while (std::find(processed_agents.begin(), processed_agents.end(), false) != processed_agents.end()){
        // find the first agent that is not processed yet
        int curr_agent_idx = std::find(processed_agents.begin(), processed_agents.end(), false) - processed_agents.begin();
        std::vector<int> waiting_chain = {curr_agent_idx};

        processed_agents[curr_agent_idx] = true;
        AgentState state = agents_[curr_agent_idx]->GetState();
        std::vector<std::string> states_chain = {AgentStateToString(state)};
        while (state == MOVING_TO_WAITING_POINT || 
                state == WAITING_AT_INTERSECTION ||
                state == WAITING_FOR_FREE_DESTINATION){
            // get the agent for which the current agent is waiting
            int temp = curr_agent_idx;
            if (state == WAITING_FOR_FREE_DESTINATION){
                const void* claiming_object = agents_[curr_agent_idx]->GetPtrToObjectClaimingDestination();
                // find the agent that is claiming the destination
                curr_agent_idx = -1;
                for (int i = 0; i < agents_.size(); i++){
                    if (agents_[i].get() == claiming_object){
                        curr_agent_idx = i;
                        break;
                    }
                }
            } else {
                curr_agent_idx = agents_[curr_agent_idx]->GetBlockingAgentIdx();
            }
            processed_agents[curr_agent_idx] = true;

            // check if the current agent is waiting for the original agent
            if (std::find(waiting_chain.begin(), waiting_chain.end(), curr_agent_idx) != waiting_chain.end()){
                // deadlock found
                waiting_chain.push_back(curr_agent_idx);
                state = agents_[curr_agent_idx]->GetState();
                states_chain.push_back(AgentStateToString(state));
                std::cout << "deadlock found." << std::endl;
                std::cout << "Waiting chain: " << waiting_chain << std::endl;
                std::cout << "States chain:  " << states_chain << std::endl;
                return true;
            }
            waiting_chain.push_back(curr_agent_idx);
            state = agents_[curr_agent_idx]->GetState();
            states_chain.push_back(AgentStateToString(state));

            // check if the current agent is idling
            if (state == IDLING){
                // the current agent is idling, so we are in a temporary deadlock
                // this agent should move to the closest possible unclaimed destination

                try{
                    std::cout << "trying to resolve deadlock" << std::endl;
                    std::string nearest_free_destination = 
                        env_.GetNearestFreeClaimableDestination(
                            agents_[curr_agent_idx]->GetCurrentPosition()
                        );

                    deadlock_resolving_tasks.push_back(
                        MoverTask(curr_agent_idx, nearest_free_destination,
                                  nb_simulated_samples_*simulation_time_step_, 
                                  true)); // indicate this is a deadlock resolving task
                    std::cout << "resolving task:" << deadlock_resolving_tasks[deadlock_resolving_tasks.size() - 1] << std::endl;
                } catch (InvalidEnvironmentOperationException &e){
                    // no free destination available, so we cannot resolve
                    // deadlock
                    std::cout << "unable to resolve deadlock (" << e.what() << ")" << std::endl;
                }
                std::cout << "deadlock found. Waiting chain: " << waiting_chain << std::endl;
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
    for (auto& task : tasks_){
        if (!task->HasBeenRevealed() && 
                task->RevealTask(nb_simulated_samples_*simulation_time_step_)){
            // check if the agent is ready for the new task
            if (agents_[task->GetAgentIdx()]->GetState() == IDLING || 
                    agents_[task->GetAgentIdx()]->GetState() == WAITING_FOR_FREE_DESTINATION){
                // set the final destination
                std::string name = task->GetDestinationName();
                Point2D<double> dest = possible_destinations_[name].
                    ConvertCellToWorld(env_.CellWidth(), env_.CellHeight());
                bool success = agents_[task->GetAgentIdx()]->
                    InstructToDestination(name, dest, task);
                if (!success){
                    // unable to claim destination, try again later
                    std::cout << "agent " << task->GetAgentIdx() << " unable to claim destination " 
                        << name << std::endl;
                    std::cout << "agents state: " << AgentStateToString(
                        agents_[task->GetAgentIdx()]->GetState()) << std::endl;
                    task->PostponeTask(0.1, 
                        nb_simulated_samples_*simulation_time_step_);
                    logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                        "Postponing task for agent " + 
                        std::to_string(task->GetAgentIdx()) + " to " + name + 
                        " (unable to claim destination)");
                } else {
                    logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                        "Agent " + std::to_string(task->GetAgentIdx()) + 
                        " instructed to destination " + name);
                }
            } else {
                // postpone the task
                task->PostponeTask(
                    agents_[task->GetAgentIdx()]->GetRemainingTimeSteps()*
                    simulation_time_step_,
                    nb_simulated_samples_*simulation_time_step_);
                logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                    "Postponing task for agent " + 
                    std::to_string(task->GetAgentIdx()) + " to " + 
                    task->GetDestinationName() + 
                    " (agent not ready, state: " + 
                    AgentStateToString(agents_[task->GetAgentIdx()]->GetState()) + ")");
            }
        }
    }
}

bool MultiMoverSimulator::AllTasksRevealed() const{
    for (int i = 0; i < tasks_.size(); i++){
        if (!tasks_[i]->HasBeenRevealed()){
            return false;
        }
    }
    return true;
}