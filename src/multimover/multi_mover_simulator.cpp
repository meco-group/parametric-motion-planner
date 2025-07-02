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

    // perform check on possible destinations
    // if (!SanityCheckOnPossibleDestinations(env_, params, possible_destinations)){
    //     throw std::runtime_error("Sanity check on possible destinations failed");
    // }

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
    int max_nb_steps = 600;//10000;
    int step_counter = 0;
    bool deadlock_detected = false;
    simulation_step_computation_times_.reserve(max_nb_steps);
    while (!all_tasks_completed && step_counter < max_nb_steps && !deadlock_detected){
        auto start = std::chrono::high_resolution_clock::now();

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

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        simulation_step_computation_times_.push_back(elapsed.count());
    }

    logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                     "Clean exit");
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
    j["nb_simulated_samples"] = nb_simulated_samples_;
    j["simulation_time_step"] = simulation_time_step_;
    j["computation_time_per_simulation_step"] = json::array();
    for (int i = 0; i < nb_simulated_samples_; i++){
        j["computation_time_per_simulation_step"].push_back(
            simulation_step_computation_times_[i]
        );
    }
    std::ofstream o(filename);
    o << std::setw(4) << j << std::endl;
}

void MultiMoverSimulator::PrintLog() const {
    logger_.PrintLog();

    double total_duration = 0.0;
    double total_duration_gt_1ms = 0.0;
    int count_gt_1ms = 0;
    for (int i = 0; i < nb_simulated_samples_; i++){
        total_duration += simulation_step_computation_times_[i];
        if (simulation_step_computation_times_[i] > 1.0){
            total_duration_gt_1ms += simulation_step_computation_times_[i];
            count_gt_1ms++;
        }
    }

    std::cout << "average duration per simulation step: ";
    if (nb_simulated_samples_ > 0){
        std::cout << total_duration / nb_simulated_samples_ << " ms" << std::endl;
    } else {
        std::cout << "N/A" << std::endl;
    }
    std::cout << "average duration of simulation steps > 1ms: ";
    if (count_gt_1ms > 0){
        std::cout << total_duration_gt_1ms / count_gt_1ms << " ms" << std::endl;
    } else {
        std::cout << "N/A" << std::endl;
    }

    std::cout << "max duration of a simulation step: ";
    if (nb_simulated_samples_ > 0){
        double max_duration = *std::max_element(simulation_step_computation_times_.begin(), 
                                                simulation_step_computation_times_.begin() + nb_simulated_samples_);
        std::cout << max_duration << " ms" << std::endl;
    } else {
        std::cout << "N/A" << std::endl;
    };
}

bool MultiMoverSimulator::SanityCheckOnPossibleDestinations(
        Environment& environment, std::vector<const Parameters*> params,
        std::map<std::string, Point2D<int>> possible_destinations) const {

    // Check for all possible parameters (possibly different sizes)
    for (int p = 0; p < params.size(); p++){
        // Create a local MotionPlanner to check the reachability
        MotionPlanner mp(*params[0], environment);

        // list all destinations
        std::vector<Point2D<double>> destinations;
        std::vector<std::string> destination_names;
        for (const auto& [name, pos] : possible_destinations){
            destinations.push_back(pos.ConvertCellToWorld(environment.CellWidth(), 
                                                        environment.CellHeight()));
            destination_names.push_back(name);
        }

        // Check reachability
        environment.SetClaimingObject(&mp);
        for (int i = 0; i < destinations.size(); i++){
            environment.ClaimDestination(destination_names[i], &mp);
            for (int j = i+1; j < destinations.size(); j++){
                // TODO: claim destinations
                environment.ClaimDestination(destination_names[j], &mp);
                mp.SetStart(destinations[i]);
                mp.SetDest(destinations[j]);
                try {
                    mp.PlanSafely();
                } catch (std::exception & e) {
                    environment.ReleaseDestination(destination_names[i], &mp);
                    environment.ReleaseDestination(destination_names[j], &mp);
                    environment.ClearClaimingObject();
                    return false;
                }
                environment.ReleaseDestination(destination_names[j], &mp);
            }
            environment.ReleaseDestination(destination_names[i], &mp);
        }
        environment.ClearClaimingObject();
    }

    return true;
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
        nb_consecutive_deadlocks_found_++;
        if (nb_consecutive_deadlocks_found_ > 20){
            std::cout << "Too many consecutive deadlocks found, stopping simulation" << std::endl;
            return true; // return true to indicate deadlock
        }
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
            // return true
            return nb_consecutive_deadlocks_found_ > 20;
        }
    } else { nb_consecutive_deadlocks_found_ = 0; }

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

    // This code is not accurate, since "DealWithCollision" creates new
    // trajectories which are not yet checked
    // for (int i = 0; i < agents_.size(); i++){
    //     if (new_trajectories_[i]){
    //         for (int j = 0; j < agents_.size(); j++){
    //             if (i == j){
    //                 continue;
    //             }
    //             if (CheckForCollision(i, j)){
    //                 DealWithCollision(i, j);
    //             }
    //         }
    //     }
    // }

    // as long as there collisions found, keep checking
    bool collision_found = true;
    int counter = 0;
    while (collision_found){
        if (counter  > 10){
            throw std::runtime_error("Too many iterations in collision resolution, possible deadlock detected");
        }

        collision_found = false;
        for (int i = 0; i < agents_.size(); i++){
            if (new_trajectories_[i]){
                for (int j = 0; j < agents_.size(); j++){
                    if (i == j){
                        continue;
                    }
                    if (CheckForCollision(i, j)){
                        DealWithCollision(i, j);
                        collision_found = true;
                    }
                }
            }
        }   
        // new_trajectories_ = updated_trajectories;
        // updated_trajectories = std::vector<bool>(agents_.size(), false);
        // new_trajectories_ = std::vector<bool>(agents_.size(), false);
        counter++;
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
        agents_[agent_idx_1]->GetVehicleFootprint(nb_steps_in_future, footprint_1, 0*collision_check_margin_);
        agents_[agent_idx_2]->GetVehicleFootprint(nb_steps_in_future, footprint_2, 0*collision_check_margin_);

        if (footprint_1.GetOverlap(footprint_2, o)){
            std::cout << "\tcollision detected: " << std::endl;
            std::cout << "\t << agent " << agent_idx_1 << " has footprint: " 
                      << footprint_1 << std::endl;
            std::cout << "\t << agent " << agent_idx_2 << " has footprint: "
                        << footprint_2 << std::endl;
            return true;
        }
    }
    std::cout << "\tno collision!" << std::endl;

    return false;
}

std::pair<bool, bool> MultiMoverSimulator::DealWithCollision(int agent_idx_1, int agent_idx_2){
    // get the intersection of the two vehicles
    CorridorUnion intersection;
    bool intersection_present = GetIntersection(agent_idx_1, agent_idx_2, intersection);
    if (!intersection_present){
        std::cout << "no intersection found between agents " << agent_idx_1;
        std::cout << " and " << agent_idx_2 << std::endl;
        return std::make_pair(false, false);
        // throw std::runtime_error("No intersection found between agent " + 
        //     std::to_string(agent_idx_1) + " and agent " + std::to_string(agent_idx_2));
    } else {
        std::map<std::string, double> times_1 = GetTimeEnteringAndLeavingIntersection(agent_idx_1, intersection);
        std::map<std::string, double> times_2 = GetTimeEnteringAndLeavingIntersection(agent_idx_2, intersection);
        IntersectionLog log(agent_idx_1, agent_idx_2,
        nb_simulated_samples_*simulation_time_step_, 
            std::max(times_1["leaving_time"], times_2["leaving_time"]), intersection);
        bool new_log_entry = true;
        for (IntersectionLog l : intersection_logs_){
            if (l == log){
                new_log_entry = false;
                break;
            }
        }
        if (new_log_entry){ intersection_logs_.push_back(log);}
    }

    //////////////////////////////
    // Simplied decision-making //
    //////////////////////////////
    bool vehicle_1_at_station = agents_[agent_idx_1]->VehicleIsAtStation();
    bool vehicle_2_at_station = agents_[agent_idx_2]->VehicleIsAtStation();
    bool vehicle_1_submitted_ = new_trajectories_[agent_idx_1];
    bool vehicle_2_submitted_ = new_trajectories_[agent_idx_2];

    std::cout << "vehicle_1_at_station: " << vehicle_1_at_station << std::endl;
    std::cout << "vehicle_2_at_station: " << vehicle_2_at_station << std::endl;
    std::cout << "vehicle_1_submitted_: " << vehicle_1_submitted_ << std::endl;
    std::cout << "vehicle_2_submitted_: " << vehicle_2_submitted_ << std::endl;

    if (!vehicle_1_at_station && !vehicle_1_submitted_ && 
            !vehicle_2_at_station && !vehicle_2_submitted_){
        throw std::runtime_error("Something is wrong: both vehicles are away"
            " from a station and neither submitted a new trajectory but they"
            " still collide.");
    }

    // If both vehicles are at a station, pick one to wait
    if (vehicle_1_at_station && vehicle_2_at_station){
        std::map<std::string, double> times_1 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_1, intersection);
        std::map<std::string, double> times_2 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_2, intersection);
        if (times_1["leaving_time"] < times_2["leaving_time"]){
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                                agent_idx_1, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Agent " + std::to_string(agent_idx_2) + " is waiting for agent " 
                + std::to_string(agent_idx_1));
            return std::make_pair(false, true);
        } else {
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                            agent_idx_2, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Agent " + std::to_string(agent_idx_1) + " is waiting for agent " 
                + std::to_string(agent_idx_2));
            return std::make_pair(true, false);
        }
    }

    // If one vehicle is at a station, it must wait
    if (vehicle_1_at_station){
        agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                           agent_idx_2, intersection);
        return std::make_pair(true, false);
    }
    if (vehicle_2_at_station){
        agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                           agent_idx_1, intersection);
        return std::make_pair(false, true);
    }

    // If both vehicles submitted a new trajectory, reject both
    if (vehicle_1_submitted_ && vehicle_2_submitted_){
        if (agents_[agent_idx_1]->Stationary()){
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], agent_idx_2);
        } else {
            agents_[agent_idx_1]->ResetWaitForAgent();

        }
        if (agents_[agent_idx_2]->Stationary()){
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], agent_idx_1);
        } else {
            agents_[agent_idx_2]->ResetWaitForAgent();
        }
        logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
            "Both agents " + std::to_string(agent_idx_1) + " and " + 
            std::to_string(agent_idx_2) + " instructed to reset waiting state");
        return std::make_pair(true, true);
    }

    // If only one vehicle submitted a new trajectory, that one must wait
    if (vehicle_1_submitted_){
        if (agents_[agent_idx_1]->Stationary()){
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], agent_idx_2);
        } else {
            agents_[agent_idx_1]->ResetWaitForAgent();
        }
        logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
            "Agent " + std::to_string(agent_idx_1) + " instructed to reset waiting state");
        return std::make_pair(true, false);
    }
    if (vehicle_2_submitted_){
        if (agents_[agent_idx_2]->Stationary()){
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], agent_idx_1);
        } else {
            agents_[agent_idx_2]->ResetWaitForAgent();
        }
        logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
            "Agent " + std::to_string(agent_idx_2) + " instructed to reset waiting state");
        return std::make_pair(true, false);
    }


    // We shouldn't reach this point
    throw std::runtime_error("Something is wrong: this point should not be reached. ");

    /*
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
        return std::make_pair(false, false);

    } else if (intersection_case == NO_OVERLAP){
        // no action needed
        return std::make_pair(false, false);

    } else if (intersection_case == AGENT_1_MUST_WAIT){
        // instruct vehicle 1 to wait
        try{
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                               agent_idx_2, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Instructed agent " + std::to_string(agent_idx_1) + 
                " to wait for agent " + std::to_string(agent_idx_2));
            return std::make_pair(true, false);
        } catch (UnableToFindWaitingPoint& e){
            // agent_idx_1 cannot find a waiting point, so agent_idx_2 must 
            // longer
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Agent " + std::to_string(agent_idx_1) + 
                " cannot find a waiting point, so agent " + std::to_string(agent_idx_2) + 
                " must wait longer");
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                               agent_idx_1, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Instructed agent " + std::to_string(agent_idx_2) + 
                " to wait for agent " + std::to_string(agent_idx_1));
            return std::make_pair(false, true);
        }

    } else if (intersection_case == AGENT_2_MUST_WAIT){
        // instruct vehicle 2 to wait
        try{
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                               agent_idx_1, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Instructed agent " + std::to_string(agent_idx_2) + 
                " to wait for agent " + std::to_string(agent_idx_1));
            return std::make_pair(false, true);
        } catch (UnableToFindWaitingPoint& e){
            // agent_idx_2 cannot find a waiting point, so agent_idx_1 must 
            // longer
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Agent " + std::to_string(agent_idx_2) + 
                " cannot find a waiting point, so agent " + std::to_string(agent_idx_1) + 
                " must wait longer");
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                               agent_idx_2, intersection);
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                "Instructed agent " + std::to_string(agent_idx_1) + 
                " to wait for agent " + std::to_string(agent_idx_2));
            return std::make_pair(true, false);
        }

    } else {
        throw std::runtime_error("Invalid CollisionResolutionDecision: Don't know what to do");
        return std::make_pair(false, false);
    }
    */
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

    // If both vehicles are already in the intersection, we're in trouble
    // Let the stationary one wait
    if ((times_1["entering_time"] == 0 && times_2["entering_time"] == 0)){
        if (agents_[agent_idx_1]->Stationary() && 
                agents_[agent_idx_2]->Stationary()){
            // both vehicles are stationary, let the one not waiting wait
            if (agents_[agent_idx_1]->GetState() == WAITING_AT_INTERSECTION){
                return AGENT_2_MUST_WAIT;
            } else if (agents_[agent_idx_2]->GetState() == WAITING_AT_INTERSECTION){
                return AGENT_1_MUST_WAIT;
            }
        }
        if (agents_[agent_idx_1]->Stationary()){
            return AGENT_1_MUST_WAIT;
        } else if (agents_[agent_idx_2]->Stationary()){
            return AGENT_2_MUST_WAIT;
        } else {
            return INVALID;
        }
    }

    // If both vehicles never leave, make the stationary one wait
    if (times_1["leaving_time"] < 0 && times_2["leaving_time"] < 0){
        if (agents_[agent_idx_1]->Stationary() && 
                agents_[agent_idx_2]->Stationary()){
            // both vehicles are stationary, so tell the one entering latest to wait
            if (times_1["entering_time"] < times_2["entering_time"]){
                return AGENT_2_MUST_WAIT;
            } else {
                return AGENT_1_MUST_WAIT;
            }
        } else if (agents_[agent_idx_1]->Stationary()){
            return AGENT_1_MUST_WAIT;
        } else if (agents_[agent_idx_2]->Stationary()){
            return AGENT_2_MUST_WAIT;
        } else {
            // both vehicles are moving, so we can choose which one must wait
            std::cout << "\tWARNING: Both vehicles are moving" << std::endl;
            if (times_1["entering_time"] < times_2["entering_time"]){
                return AGENT_2_MUST_WAIT;
            } else {
                return AGENT_1_MUST_WAIT;
            }
        }
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


    // If one vehicle is already in the intersection, the other must wait
    if (times_1["entering_time"] == 0 && times_2["entering_time"] > 0){
        return AGENT_2_MUST_WAIT;
    } else if (times_2["entering_time"] == 0 && times_1["entering_time"] > 0){
        return AGENT_1_MUST_WAIT;
    }

    // Otherwise, if one vehicle never leaves, that one must wait
    if (times_1["leaving_time"] < 0){
        return AGENT_1_MUST_WAIT;
    }

    if (times_2["leaving_time"] < 0){
        return AGENT_2_MUST_WAIT;
    }

    // Check if vehicles plan to be in intersection at the same time
    if (times_1["leaving_time"] < times_2["entering_time"] ||
            times_2["leaving_time"] < times_1["entering_time"]){
        throw std::runtime_error("Vehicles are not in the intersection at the same time, but we detected a collision");
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
    if (intersection.OverlapsWith(footprint)){
        result["entering_time"] = 0;
    } else {
        while (!intersection.OverlapsWith(footprint)){
            nb_time_steps_from_now++;
            if (nb_time_steps_from_now >= nb_steps_remaining){
                // throw std::runtime_error("How can it be that a vehicle never enters intersection if we already detected collision?");
                return result;
            }
            agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
        }
        // result["entering_time"] = agents_[agent_idx]->GetTimeAtTimeStep(nb_time_steps_from_now);
        result["entering_time"] = (nb_simulated_samples_ + nb_time_steps_from_now)*
            simulation_time_step_;
    }

    // find the time when the vehicle leaves the intersection
    // (start at the end of the trajectory)
    int nb_steps_until_entering = nb_time_steps_from_now;
    nb_time_steps_from_now = nb_steps_remaining - 1;
    agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    if (intersection.OverlapsWith(footprint)){
        // the vehicle is already in the intersection at the end of the trajectory
        // so it will never leave
        result["leaving_time"] = -1;
        return result;
    }
    while (!intersection.OverlapsWith(footprint)){
        nb_time_steps_from_now--;
        if (nb_time_steps_from_now <= nb_steps_until_entering){
            return result;
        }
        agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    }
    // result["leaving_time"] = agents_[agent_idx]->GetTimeAtTimeStep(nb_time_steps_from_now);
    result["leaving_time"] = std::max(
        (nb_simulated_samples_ + nb_time_steps_from_now)*simulation_time_step_,
        result["entering_time"]
    );

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
        while (state == WAITING_AT_INTERSECTION || 
                //state == MOVING_TO_WAITING_POINT ||
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

                // a chain has been found. Try to resolve it by moving one agent
                // to the closest free destination
                std::string destination = 
                    env_.GetNearestFreeClaimableDestination(
                        agents_[curr_agent_idx]->GetCurrentPosition());
                std::shared_ptr<MoverTask> deadlock_resolving_task = 
                    std::make_shared<MoverTask>(curr_agent_idx, destination,
                        nb_simulated_samples_*simulation_time_step_, true);
                deadlock_resolving_task->RevealTask(nb_simulated_samples_*simulation_time_step_);
                tasks_.push_back(deadlock_resolving_task);
                agents_[curr_agent_idx]->ResolveDeadlock(deadlock_resolving_task);
                logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                    "Deadlock found, resolving by moving agent " + 
                    std::to_string(curr_agent_idx) + " to destination " + 
                    destination);
                return false;
                // TODO: if this task still leads to deadlock, we should pick
                // another agent or another destination
            }
            waiting_chain.push_back(curr_agent_idx);
            state = agents_[curr_agent_idx]->GetState();
            states_chain.push_back(AgentStateToString(state));

            // check if the current agent is idling
            if (state == IDLING){
                // the current agent is idling, so we are in a temporary deadlock
                // this agent should move to the closest possible unclaimed destination
                // Note: only do this if the deadlock seems to persist
                if (nb_consecutive_deadlocks_found_ < 10){
                    return true;
                }

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