#include <filesystem>
#include <future>
#include <mutex>

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


MultiMoverSimulator:: MultiMoverSimulator(Environment& environment,
                std::vector<Parameters*> params,
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
        std::cout << "adding claimable destination: " << name << std::endl;
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
    // have all agents plan already
    for (int i = 0; i < agents_.size(); i++){
        agents_[i]->UpdateTrajectory();
    }

    // create profilers
    if (perform_profiling_){
        profilers_["SimulateAllTasks"] = Profiler(
            {"ProcessPotentialNewTasks", "SimulateSingleStep", "Finish loop", "Write simulation progress to file"}
        );

        profilers_["SimulateSingleStep"] = Profiler(        // breakdown of SimulateSingleStep in SimulateAllTasks
            {"UpdateTrajectory", "ProcessPotentialNewCollisions", 
             "CheckIfDeadlockPresent", "UpdateSingleStep"}
        );

        // processpotentialnewcollision consists of [processpotentialvirtualcollision + checkforcollision + dealwithcollision]
        profilers_["ProcessPotentialVirtualCollision"] = Profiler(
            {"ProcessPotentialVirtualCollision"}
        );
        profilers_["CheckForCollision"] = Profiler({"CheckForCollision"});
        // profilers_["DealWithCollision"] = Profiler({"DealWithCollision"});
        profilers_["DealWithCollision"] = Profiler({"GetIntersection", 
            "SetupDecisionMaking", "GetTimes", "BothAtStationCase", "OneAtStationCase", 
            "BothSubmittedCase", "OneSubmittedCase"});

        profilers_["GetTimeEnteringAndLeaving"] = Profiler({"GetTimeEnteringAndLeaving"});
    }

    std::cout << env_ << std::endl;
}

bool MultiMoverSimulator::SimulateSteps(int nb_steps, bool stop_when_all_idling){
    for (int i = 0; i < nb_steps; i++){
        ProcessPotentialNewTasks();
        SimulateSingleStep();
        nb_simulated_samples_++;

        // Check if all agents are idling
        if (AllAgentsIdling()){
            std::cout << "All agents are idling --> returning (" << i << "/" << nb_steps << ")" << std::endl;
            std::cout << "Tasks: " << std::endl;
            for (const auto& task : tasks_){
                std::cout << *task << std::endl;
            }
            return true;
        }
    }
    return false;
}

void MultiMoverSimulator::SimulateAllTasks(){
    bool all_tasks_completed = false;
    int max_nb_steps = 5990;
    int step_counter = 0;
    bool deadlock_detected = false;
    simulation_step_computation_times_.reserve(max_nb_steps);
    while (!all_tasks_completed && step_counter < max_nb_steps && !deadlock_detected){
        if (perform_profiling_){ profilers_["SimulateAllTasks"].StartSimulationStep();}

        // check if new tasks are available
        ProcessPotentialNewTasks();
        if (perform_profiling_){ profilers_["SimulateAllTasks"].RecordIntermediateSimulationStep();}

        deadlock_detected = SimulateSingleStep();
        if (perform_profiling_){ 
            double step_simulation_time = profilers_["SimulateAllTasks"].RecordIntermediateSimulationStep();
            /*
            if (step_simulation_time > 30*1e3){
                // check why it took so long
                std::cout << std::endl;
                profilers_["SimulateSingleStep"].PrintLastStepInfo();
                std::cout << std::endl;
                profilers_["ProcessPotentialVirtualCollision"].PrintLastStepInfo();
                std::cout << std::endl;
                profilers_["CheckForCollision"].PrintLastStepInfo();
                std::cout << std::endl;
                profilers_["DealWithCollision"].PrintLastStepInfo();
                // for each agent that planned a new trajectory, print profiler info
                for (int j = 0; j < new_trajectories_.size(); j++){
                    if (new_trajectories_[j] || true){
                        std::cout << std::endl;
                        std::cout << "Profiler info for agent " << j << ":" << std::endl;
                        agents_[j]->PrintProfilerInfo();
                    } else {
                        std::cout << std::endl;
                        std::cout << "Agent " << j << " did not plan a new trajectory." << std::endl;
                    }
                }
                throw std::runtime_error("Simulation step took too long: " + 
                    std::to_string(step_simulation_time) + " microseconds");
            }
            */
        }

        // check if all tasks are completed
        if (AllTasksRevealed() && AllAgentsIdling()){
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                             "All tasks completed and all agents are idling");
            all_tasks_completed = true;
        }

        nb_simulated_samples_++;
        step_counter++;

        if (perform_profiling_){ profilers_["SimulateAllTasks"].RecordIntermediateSimulationStep();}

        if (write_simulation_progress_to_file_ /*&& nb_steps%10 == 0*/){
            std::string filename = "simulation_progress.txt";
            double current_time = nb_simulated_samples_ * simulation_time_step_;
            // clear the file and write the current time 
            std::ofstream file(filename, std::ios::trunc);
            file << "current simulation time: " << current_time << " seconds" << std::endl;
            file.close();                        
        }
        if (perform_profiling_){ profilers_["SimulateAllTasks"].EndSimulationStep();}
    }

    std::cout << "Tasks: " << std::endl;
    for (const auto& task : tasks_){
        std::cout << *task << std::endl;
    }
    std::cout << "All tasks completed: " << AllTasksCompleted() << std::endl;

    logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                     "Clean exit");
}

bool MultiMoverSimulator::AllTasksCompleted() const {
    // return AllTasksRevealed() && AllAgentsIdling();
    for (const auto& task : tasks_){
        if (!task->HasBeenCompleted() && !task->IsDeadlockResolutionTask()){
            return false;
        }
    }
    return true;
}

void MultiMoverSimulator::UpdateSetpoints(std::vector<Point2D<double>>& curr_pos,
                                          std::vector<Point2D<double>>& curr_vel,
                                          std::vector<Point2D<double>>& curr_acc) const {
    for (int i = 0; i < GetNbAgents(); i++){
        agents_[i]->UpdateSetpoint(curr_pos[i], curr_vel[i], curr_acc[i]);
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
    j["nb_simulated_samples"] = nb_simulated_samples_;
    j["simulation_time_step"] = simulation_time_step_;
    j["computation_time_per_simulation_step"] = json::array();
    for (int i = 0; i < nb_simulated_samples_; i++){
        j["computation_time_per_simulation_step"].push_back(
            simulation_step_computation_times_[i]
        );
    }
    if (perform_profiling_){
        j["profilers"] = json::object();
        for (const auto& [name, profiler] : profilers_){
            j["profilers"][name] = profiler.ToJson();
        }

        j["profilers"]["detailed"] = json::object();
        for (const auto& [name, profiler] : profilers_){
            j["profilers"]["detailed"][name] = profiler.ToJsonDetailed();
        }
    }
    j["nb_new_trajectories_per_step"] = json::array();
    for (int i = 0; i < nb_new_trajectories_.size(); i++){
        j["nb_new_trajectories_per_step"].push_back(nb_new_trajectories_[i]);
    }
    std::ofstream o(filename);
    o << std::setw(4) << j << std::endl;
}

void MultiMoverSimulator::SetPerformanceMode(bool performance_mode){
    performance_mode_ = performance_mode;
    if (performance_mode_){
        write_simulation_progress_to_file_ = false;
        // perform_profiling_ = false;
        silent_mode_ = true;
        for (auto& agent : agents_){
            agent->SetPerformanceMode(true);
        }
    }
}

void MultiMoverSimulator::PrintProfilerInfo() const {
    if (perform_profiling_){
        for (const auto& [name, profiler] : profilers_){
            std::cout << "Profiler info for " << name << ":" << std::endl;
            std::cout << profiler.ToJson().dump(4) << std::endl;
        }
    } else {
        std::cout << "Profiling not enabled." << std::endl;
    }
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
    std::cout << "-------------------\nsimulating step\n-------------------" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    if (perform_profiling_){ profilers_["SimulateSingleStep"].StartSimulationStep();}
    // Update trajectories if needed
    #ifdef PARALLELIZE
    for (int j = 0; j < agents_.size(); j++){
        agents_[j]->PrepareUpdateTrajectory(); // not sure if this is really still necessary
    }
    std::vector<std::future<bool>> futures;
    for (int j = 0; j < agents_.size(); j++){
        futures.push_back(
            std::async(std::launch::async, 
                &Agent::UpdateTrajectory, agents_[j].get())
        );
        // futures.push_back(
        //     std::async(std::launch::async, [this, j](){
        //         std::lock_guard<std::mutex> lock(mutex_); // protect shared resources
        //         bool result = agents_[j]->UpdateTrajectory();
        //         return result;
        //     })
        // );
    }
    for (int j = 0; j < agents_.size(); j++){
        new_trajectories_[j] = futures[j].get();
        if (new_trajectories_[j]){
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Agent " + std::to_string(j) + " updated trajectory");
        }
    }
    #else
    new_trajectories_.assign(agents_.size(), false);
    for (int j = 0; j < agents_.size(); j++){
        int idx = (j + next_agent_to_be_allowed_to_plan) % agents_.size();
        new_trajectories_[idx] = agents_[idx]->UpdateTrajectory();
        if (new_trajectories_[idx]){
            logger_.LogEvent(nb_simulated_samples_*simulation_time_step_, 
                "Agent " + std::to_string(idx) + " updated trajectory");
        }

        // get time so far
        auto curr = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = curr - start;
        if (elapsed.count() > max_time_update_trajectory_in_ms_){
            next_agent_to_be_allowed_to_plan = (idx + 1) % agents_.size();
            break;
        }
    }
    #endif

    if (perform_profiling_){ profilers_["SimulateSingleStep"].RecordIntermediateSimulationStep();}
    int nb_new_trajectories = 0;
    for (int j = 0; j < new_trajectories_.size(); j++){
        if (new_trajectories_[j]){
            nb_new_trajectories++;
        }
    }
    nb_new_trajectories_.push_back(nb_new_trajectories);

    // Check for new collisions
    if (!ignore_collisions_){
        ProcessPotentialNewCollsions();
    }
    if (perform_profiling_){ profilers_["SimulateSingleStep"].RecordIntermediateSimulationStep();}

    // Check for deadlocks
    std::vector<MoverTask> deadlock_resolving_tasks;
    if (CheckIfDeadlockPresent(deadlock_resolving_tasks)){
        nb_consecutive_deadlocks_found_++;
        if (nb_consecutive_deadlocks_found_ > 20){
            std::cout << "Too many consecutive deadlocks found, stopping simulation" << std::endl;
            if (perform_profiling_){ profilers_["SimulateSingleStep"].AbortStep();}
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
            if (perform_profiling_){ profilers_["SimulateSingleStep"].AbortStep();}
            return nb_consecutive_deadlocks_found_ > 20;
        }
    } else { nb_consecutive_deadlocks_found_ = 0; }
    if (perform_profiling_){ profilers_["SimulateSingleStep"].RecordIntermediateSimulationStep();}

    claimed_destinations_info_.push_back(env_.ClaimableDestinationsToJson());

    // Update positions of all agents
    UpdateSingleStep();
    if (perform_profiling_){ profilers_["SimulateSingleStep"].EndSimulationStep();}

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    simulation_step_computation_times_.push_back(elapsed.count());

    // abort if this took too long
    if (elapsed.count() > 1e10*100){
        std::cout << "Simulation step took too long: " 
                  << elapsed.count() << " ms" << std::endl;
        // show latest profiler step info
        profilers_["SimulateSingleStep"].PrintLastStepInfo();
        std::cout << std::endl;
        profilers_["ProcessPotentialVirtualCollision"].PrintLastStepInfo();
        std::cout << std::endl;
        profilers_["CheckForCollision"].PrintLastStepInfo();
        std::cout << std::endl;
        profilers_["DealWithCollision"].PrintLastStepInfo();
        // for each agent that planned a new trajectory, print profiler info
        for (int j = 0; j < new_trajectories_.size(); j++){
            if (new_trajectories_[j]){
                std::cout << std::endl;
                std::cout << "Profiler info for agent " << j << ":" << std::endl;
                agents_[j]->PrintProfilerInfo();
            } else {
                // std::cout << "Agent " << j << " did not plan a new trajectory." << std::endl;
            }
        }
        throw std::runtime_error("aborting this simulation");
    }

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

    // update virtual trajectories (= remove virtual agents that are not stationary)
    for (int i = 0; i < prioritized_agents_.size(); i++){
        if (!agents_[prioritized_agents_[i]->GetAgentIdx()]->Stationary()){
            prioritized_agents_.erase(prioritized_agents_.begin() + i);
            i--;
        }
    }
    // std::cout << "number of prioritized agents: " 
    //           << prioritized_agents_.size() << std::endl;

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

                // check collision with prioritized trajectories
                if (perform_profiling_){ profilers_["ProcessPotentialVirtualCollision"].StartSimulationStep();}
                ProcessPotentialVirtualCollision(i);
                if (perform_profiling_){ profilers_["ProcessPotentialVirtualCollision"].EndSimulationStep();}

                // check collision with other agents
                for (int j = 0; j < agents_.size(); j++){
                    if (i == j){
                        continue;
                    }
                    if (perform_profiling_){ profilers_["CheckForCollision"].StartSimulationStep();}
                    // if (CheckForCollision(i, j)){
                    if (CheckForCollisionBinary(i, j)){
                        if (perform_profiling_){ profilers_["CheckForCollision"].EndSimulationStep();}
                        // if (perform_profiling_){ profilers_["DealWithCollision"].StartSimulationStep();}
                        DealWithCollision(i, j);
                        // if (perform_profiling_){ profilers_["DealWithCollision"].EndSimulationStep();}
                        collision_found = true;
                    } else {
                        if (perform_profiling_){ profilers_["CheckForCollision"].EndSimulationStep();}
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

void MultiMoverSimulator::ProcessPotentialVirtualCollision(int agent_idx){
    if (!agents_[agent_idx]->Stationary()){
        return;
    }

    for (int i = 0; i < prioritized_agents_.size(); i++){
        // std::cout << "checking prioitized agents (i = " << i << ")" << std::endl;
        if (prioritized_agents_[i]->GetAgentIdx() == agent_idx){
            continue;
        }
        if (prioritized_agents_[i]->GetTrajectory()->CheckGeometricCollision(
                *(agents_[agent_idx]->GetTrajectory()), *params_[agent_idx], 
                *params_[prioritized_agents_[i]->GetAgentIdx()])){
            // reject the new trajectory
            // std::cout << "interesting: wait for prioritized agent "
                    //   << prioritized_agents_[i]->GetAgentIdx() << std::endl;
            agents_[agent_idx]->WaitForPrioritizedVehicle(
                agents_[prioritized_agents_[i]->GetAgentIdx()], 
                prioritized_agents_[i]->GetAgentIdx());
            return;
        }
    }
}

bool MultiMoverSimulator::CheckForCollision(int agent_idx_1, int agent_idx_2){
    // Check if there is any corridor overlap at all
    CorridorUnion cu;
    bool overlap_present = GetIntersection(agent_idx_1, agent_idx_2, cu);
    if (!overlap_present || agents_[agent_idx_1]->GetState() == IDLING ||
        agents_[agent_idx_2]->GetState() == IDLING){
        return false;
    }

    // std::cout << "Checking for collision between agents " << agent_idx_1;
    // std::cout << " and " << agent_idx_2 << std::endl;
    int nb_time_steps_to_check = std::max(
        agents_[agent_idx_1]->GetRemainingTimeSteps(),
        agents_[agent_idx_2]->GetRemainingTimeSteps()
    );
    // TODO: if the shortest trajectory is towards a station, we don't need to
    // take max but can get away with min
    
    Corridor footprint_1;
    Corridor footprint_2;
    Corridor o;
    int nb_steps_in_future = 0;
    Point2D<double> pos_1, vel_1;
    Point2D<double> pos_2, vel_2;
    double distance_x, distance_y;
    double delta_v_x, delta_v_y;
    double a_max_1 = params_[agent_idx_1]->GetAmax();
    double a_max_2 = params_[agent_idx_2]->GetAmax();
    while (nb_steps_in_future < nb_time_steps_to_check){
        agents_[agent_idx_1]->GetVehicleFootprint(nb_steps_in_future, footprint_1, 0*collision_check_margin_);
        agents_[agent_idx_2]->GetVehicleFootprint(nb_steps_in_future, footprint_2, 0*collision_check_margin_);

        if (footprint_1.GetOverlap(footprint_2, o)){
            return true;
        }

        agents_[agent_idx_1]->GetPosAndVel(nb_steps_in_future, pos_1, vel_1);
        agents_[agent_idx_2]->GetPosAndVel(nb_steps_in_future, pos_2, vel_2);
        distance_x = std::abs(pos_1.x() - pos_2.x()) - 
                        (params_[agent_idx_1]->GetWidthOffset() + 
                         params_[agent_idx_2]->GetWidthOffset() + 
                         2*collision_check_margin_);
        distance_y = std::abs(pos_1.y() - pos_2.y()) -
                        (params_[agent_idx_1]->GetHeightOffset() + 
                         params_[agent_idx_2]->GetHeightOffset() + 
                         2*collision_check_margin_);

        delta_v_x = vel_2.x() - vel_1.x();
        if (pos_1.x() < pos_2.x()) delta_v_x = -delta_v_x;
        delta_v_y = vel_2.y() - vel_1.y();
        if (pos_1.y() < pos_2.y()) delta_v_y = -delta_v_y;

        // consider acceleration
        // nb_steps_in_future += std::max(1, std::max(
        //     int((delta_v_x - sqrt(delta_v_x*delta_v_x+2*(a_max_1 + a_max_2)*distance_x))/(-(a_max_1 + a_max_2))/simulation_time_step_),
        //     int((delta_v_y - sqrt(delta_v_y*delta_v_y+2*(a_max_1 + a_max_2)*distance_y))/(-(a_max_1 + a_max_2))/simulation_time_step_)
        //     )
        // );
        nb_steps_in_future += 1;
    }
    // std::cout << "\tno collision!" << std::endl;

    return false;
}

bool MultiMoverSimulator::CheckForCollisionBinary(int agent_idx_1, int agent_idx_2, int start_idx, int stop_idx){
    if (stop_idx < 0) stop_idx = std::max(
        agents_[agent_idx_1]->GetRemainingTimeSteps(),
        agents_[agent_idx_2]->GetRemainingTimeSteps()
    );

    // allocate containers
    Corridor footprint_this;
    Corridor footprint_other;
    Corridor o;

    // base case
    if (start_idx >= stop_idx){
        agents_[agent_idx_1]->GetVehicleFootprint(start_idx, footprint_this, 0);
        agents_[agent_idx_2]->GetVehicleFootprint(start_idx, footprint_other, 0);
        return footprint_this.GetOverlap(footprint_other, o);
    }

    // Otherwise, check bounding box around trajectories
    agents_[agent_idx_1]->GetFootprintBoundingBox(start_idx, stop_idx, footprint_this);
    agents_[agent_idx_2]->GetFootprintBoundingBox(start_idx, stop_idx, footprint_other);

    if (footprint_this.GetOverlap(footprint_other, o)){
        // recurse
        int mid_idx = (start_idx + stop_idx) / 2;
        return CheckForCollisionBinary(agent_idx_1, agent_idx_2, start_idx, mid_idx) ||
               CheckForCollisionBinary(agent_idx_1, agent_idx_2, mid_idx+1, stop_idx);
    } else {
        return false;
    }
}

std::pair<bool, bool> MultiMoverSimulator::DealWithCollision(int agent_idx_1, int agent_idx_2){
    // get the intersection of the two vehicles
    if (perform_profiling_){ profilers_["DealWithCollision"].StartSimulationStep();}
    CorridorUnion intersection;
    bool intersection_present = GetIntersection(agent_idx_1, agent_idx_2, intersection);
    if (!intersection_present){
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(false, false);
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
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // GetIntersection

    //////////////////////////////
    // Simplied decision-making //
    //////////////////////////////
    bool vehicle_1_at_station = agents_[agent_idx_1]->VehicleIsAtStation();
    bool vehicle_2_at_station = agents_[agent_idx_2]->VehicleIsAtStation();
    bool vehicle_1_submitted_ = new_trajectories_[agent_idx_1];
    bool vehicle_2_submitted_ = new_trajectories_[agent_idx_2];

    std::cout << "DEALING WITH COLLISION BETWEEN AGENT " << agent_idx_1 
              << " AND AGENT " << agent_idx_2 << std::endl;
    std::cout << "\tvehicle_1_at_station: " << vehicle_1_at_station << std::endl;
    std::cout << "\tvehicle_2_at_station: " << vehicle_2_at_station << std::endl;
    std::cout << "\tvehicle_1_submitted_: " << vehicle_1_submitted_ << std::endl;
    std::cout << "\tvehicle_2_submitted_: " << vehicle_2_submitted_ << std::endl;

    if (!vehicle_1_at_station && !vehicle_1_submitted_ && 
            !vehicle_2_at_station && !vehicle_2_submitted_){
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        throw std::runtime_error("Something is wrong: both vehicles are away"
            " from a station and neither submitted a new trajectory but they"
            " still collide.");
    }

    double t = nb_simulated_samples_*simulation_time_step_;
    std::string idx1 = " " + std::to_string(agent_idx_1) + " ";
    std::string idx2 = " " + std::to_string(agent_idx_2) + " ";
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // SetupDecisionMaking

    // If both vehicles are at a station, pick one to wait
    if (vehicle_1_at_station && vehicle_2_at_station){
        std::map<std::string, double> times_1 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_1, intersection);
        std::map<std::string, double> times_2 = 
            GetTimeEnteringAndLeavingIntersection(agent_idx_2, intersection);
        if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // GetTimes
        if (times_1["leaving_time"] < times_2["leaving_time"]){
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                                   agent_idx_1, intersection,
                                                   ignore_waiting_points_);
            AddPrioritizedAgent(prioritized_agent);

            logger_.LogEvent(t, "Agent" + idx2 + "is waiting for agent" + idx1);
            if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
            return std::make_pair(false, true);
        } else {
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                                   agent_idx_2, intersection,
                                                   ignore_waiting_points_);
            AddPrioritizedAgent(prioritized_agent);
            
            logger_.LogEvent(t, "Agent" + idx1 + "is waiting for agent" + idx2);
            if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
            return std::make_pair(true, false);
        }
    }
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // GetTimes
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // BothAtStationCase

    // If one vehicle is at a station, it must wait
    if (vehicle_1_at_station){
        std::shared_ptr<VirtualAgent> prioritized_agent = 
            agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                               agent_idx_2, intersection,
                                               ignore_waiting_points_);
        AddPrioritizedAgent(prioritized_agent);
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(true, false);
    }
    if (vehicle_2_at_station){
        std::shared_ptr<VirtualAgent> prioritized_agent = 
            agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                               agent_idx_1, intersection,
                                               ignore_waiting_points_);
        AddPrioritizedAgent(prioritized_agent);
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(false, true);
    }
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // OneAtStationCase

    // If both vehicles submitted a new trajectory, reject both
    if (vehicle_1_submitted_ && vehicle_2_submitted_){
        if (agents_[agent_idx_1]->Stationary()){
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                // agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                //                                    agent_idx_2,
                //                                    ignore_waiting_points_);
                agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                                   agent_idx_2,
                                                   intersection,
                                                   ignore_waiting_points_);

            AddPrioritizedAgent(prioritized_agent);
        } else {
            agents_[agent_idx_1]->ResetWaitForAgent();

        }
        if (agents_[agent_idx_2]->Stationary()){
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                // agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                //                                    agent_idx_1,
                //                                    ignore_waiting_points_);
                agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                                   agent_idx_1,
                                                   intersection,
                                                   ignore_waiting_points_);
            AddPrioritizedAgent(prioritized_agent);
        } else {
            agents_[agent_idx_2]->ResetWaitForAgent();
        }
        logger_.LogEvent(t, "Both agents" + idx1 + "and" + idx2 + "instructed to reset waiting state");
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(true, true);
    }
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // BothSubmittedCase

    // If only one vehicle submitted a new trajectory, that one must wait
    if (vehicle_1_submitted_){
        if (agents_[agent_idx_1]->Stationary()){
            std::cout << "Agent " << agent_idx_1 << " is stationary and must wait for agent " 
                      << agent_idx_2 << std::endl;
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                // agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                //                                    agent_idx_2,
                //                                    ignore_waiting_points_);
                agents_[agent_idx_1]->WaitForAgent(agents_[agent_idx_2], 
                                                   agent_idx_2,
                                                   intersection,
                                                   ignore_waiting_points_);
            AddPrioritizedAgent(prioritized_agent);
            std::cout << "Added prioritized agent for agent " 
                      << agent_idx_1 << std::endl;
        } else {
            agents_[agent_idx_1]->ResetWaitForAgent();
        }
        logger_.LogEvent(t, "Agent" + idx1 + "instructed to reset waiting state");
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(true, false);
    }
    if (vehicle_2_submitted_){
        if (agents_[agent_idx_2]->Stationary()){
            std::shared_ptr<VirtualAgent> prioritized_agent = 
                // agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                //                                    agent_idx_1,
                //                                    ignore_waiting_points_);
                agents_[agent_idx_2]->WaitForAgent(agents_[agent_idx_1], 
                                                   agent_idx_1,
                                                   intersection,
                                                   ignore_waiting_points_);
                                
            AddPrioritizedAgent(prioritized_agent);
            std::cout << "Added prioritized agent for agent " 
                      << agent_idx_1 << std::endl;
        } else {
            agents_[agent_idx_2]->ResetWaitForAgent();
        }
        logger_.LogEvent(t, "Agent" + idx2 + "instructed to reset waiting state");
        if (perform_profiling_){ profilers_["DealWithCollision"].AbortStep();}
        return std::make_pair(true, false);
    }
    if (perform_profiling_){ profilers_["DealWithCollision"].RecordIntermediateSimulationStep();} // OneSubmittedCase


    // We shouldn't reach this point
    throw std::runtime_error("Something is wrong: this point should not be reached. ");
}

bool MultiMoverSimulator::GetIntersection(int agent_idx_1, int agent_idx_2, 
                                          CorridorUnion& intersection){
    CorridorSequence seq_1 = agents_[agent_idx_1]->GetCorridorSequence();
    CorridorSequence seq_2 = agents_[agent_idx_2]->GetCorridorSequence();
    intersection = seq_1.GetOverlap(seq_2);

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
    if (perform_profiling_) { profilers_["GetTimeEnteringAndLeaving"].StartSimulationStep(); }
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
                if (perform_profiling_) { profilers_["GetTimeEnteringAndLeaving"].AbortStep(); }
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
        if (perform_profiling_) { profilers_["GetTimeEnteringAndLeaving"].AbortStep(); }
        return result;
    }
    while (!intersection.OverlapsWith(footprint)){
        nb_time_steps_from_now--;
        if (nb_time_steps_from_now <= nb_steps_until_entering){
            if (perform_profiling_) { profilers_["GetTimeEnteringAndLeaving"].AbortStep(); }
            return result;
        }
        agents_[agent_idx]->GetVehicleFootprint(nb_time_steps_from_now, footprint, 0);
    }
    // result["leaving_time"] = agents_[agent_idx]->GetTimeAtTimeStep(nb_time_steps_from_now);
    result["leaving_time"] = std::max(
        (nb_simulated_samples_ + nb_time_steps_from_now)*simulation_time_step_,
        result["entering_time"]
    );
    
    if (perform_profiling_) { profilers_["GetTimeEnteringAndLeaving"].EndSimulationStep(); }
    return result;
}

bool MultiMoverSimulator::CheckIfDeadlockPresent(std::vector<MoverTask> &deadlock_resolving_tasks){
    std::vector<bool> processed_agents(agents_.size(), false);

    deadlock_resolving_tasks = {};

    // keep going as long as not all agents have been checked
    int curr_agent_idx = 0;
    bool deadlock_found = false;
    bool deadlock_found_but_attempted_resolution = false;
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

                // make sure to keep only the loop itself
                while (waiting_chain.front() != curr_agent_idx){
                    waiting_chain.erase(waiting_chain.begin());
                    states_chain.erase(states_chain.begin());
                }

                std::cout << "\tcleaned up version:" << std::endl;
                std::cout << "\tWaiting chain: " << waiting_chain << std::endl;
                std::cout << "\tStates chain:  " << states_chain << std::endl;

                // a chain has been found. Try to resolve it by moving one agent
                // to the closest free destination. Make sure to pick a new
                // agent if this deadlock has previously been attempted to be
                // resolved
                std::vector<double> manhatten_distances(waiting_chain.size()-1);
                std::string destination;
                Point2D<double> dest_pos;
                for (int i = 0; i < waiting_chain.size()-1; i++){
                    destination = env_.GetNearestFreeClaimableDestination(
                        agents_[waiting_chain[i]]->GetCurrentPosition(), true);
                    dest_pos = possible_destinations_[destination].
                        ConvertCellToWorld(env_.CellWidth(), env_.CellHeight());
                    manhatten_distances[i] = std::abs(dest_pos.x() - 
                        agents_[waiting_chain[i]]->GetCurrentPosition().x()) +
                        std::abs(dest_pos.y() - 
                        agents_[waiting_chain[i]]->GetCurrentPosition().y());

                    if (agents_[waiting_chain[i]]->GetState() == WAITING_FOR_FREE_DESTINATION){
                        manhatten_distances[i] += 5000;
                    }
                }
                // Figure out which agent should move. If we already attempted
                // to resolve this deadlock cycle, pick another agent to move
                bool found_existing_deadlock = false;
                for (int cycle_idx = 0; cycle_idx < deadlock_cycles_.size(); cycle_idx++){
                    std::unordered_set<int> cycle = deadlock_cycles_[cycle_idx];
                    if (cycle == std::unordered_set<int>(waiting_chain.begin(), waiting_chain.end())){
                        // this deadlock cycle already existed, make sure to not pick an agent we already tried
                        for (int i = 0; i < waiting_chain.size()-1; i++){
                            if (deadlocked_agents_attempted_resolution_[cycle_idx][waiting_chain[i]]){
                                // we already tried to move this agent
                                manhatten_distances[i] += 1000;
                            }
                        }
                        found_existing_deadlock = true;
                        std::cout << "found existing deadlock" << std::endl;
                        for (int i = 0; i < waiting_chain.size()-1; i++){
                            std::cout << "\tagent " << waiting_chain[i] 
                                      << " attempted resolution: " 
                                      << deadlocked_agents_attempted_resolution_[cycle_idx][waiting_chain[i]] 
                                      << ", manhatten distance: " 
                                      << manhatten_distances[i] << std::endl;
                        }
                    }
                }
                if (!found_existing_deadlock) {
                    // this is a new deadlock cycle, store it
                    deadlock_cycles_.push_back(
                        std::unordered_set<int>(waiting_chain.begin(), waiting_chain.end())
                    );
                    deadlocked_agents_attempted_resolution_.push_back(
                        std::map<int, bool>()
                    );
                    for (int i = 0; i < waiting_chain.size()-1; i++){
                        deadlocked_agents_attempted_resolution_.back()[waiting_chain[i]] = false;
                    }
                    std::cout << "looks like a new cycle (" << deadlock_cycles_.size() << ")" << std::endl;
                } 
                std::cout << "manhatten distances before: " << manhatten_distances << std::endl;

                // Pick the agent closest to a free destination
                int idx_min = waiting_chain[
                    std::min_element(
                        manhatten_distances.begin(), manhatten_distances.end()
                    )
                     - manhatten_distances.begin()
                ];

                // Make sure we pick another agent next time
                for (int cycle_idx = 0; cycle_idx < deadlock_cycles_.size(); cycle_idx++){
                    std::unordered_set<int> cycle = deadlock_cycles_[cycle_idx];
                    if (cycle == std::unordered_set<int>(waiting_chain.begin(), waiting_chain.end())){
                        deadlocked_agents_attempted_resolution_[cycle_idx][idx_min] = true;
                        std::cout << "Updated attempted resolution: " 
                                  << deadlocked_agents_attempted_resolution_[cycle_idx] << std::endl;
                    }
                }
                std::cout << "manhatten distances after: " << manhatten_distances << std::endl;

                destination = 
                    env_.GetNearestFreeClaimableDestination(
                        agents_[idx_min]->GetCurrentPosition(), true);
                std::shared_ptr<MoverTask> deadlock_resolving_task = 
                    std::make_shared<MoverTask>(idx_min, destination,
                        nb_simulated_samples_*simulation_time_step_, true);
                std::cout << "created deadlock resolving task for agent " 
                          << idx_min << " to destination " 
                          << destination << " (deadlock resolving mark: " << deadlock_resolving_task->IsDeadlockResolutionTask() << ") " << std::endl;
                deadlock_resolving_task->RevealTask(nb_simulated_samples_*simulation_time_step_);
                tasks_.push_back(deadlock_resolving_task);
                int nb_destinations_claimed_before = env_.GetNbDestinationsClaimedBy(&(*agents_[idx_min]));
                agents_[idx_min]->ResolveDeadlock(deadlock_resolving_task);
                int nb_destinations_claimed_after = env_.GetNbDestinationsClaimedBy(&(*agents_[idx_min]));
                logger_.LogEvent(nb_simulated_samples_*simulation_time_step_,
                    "Deadlock found, resolving by moving agent " + 
                    std::to_string(idx_min) + " to destination " + 
                    destination + " (waiting chain: " + str(waiting_chain) +")" +
                    " (" + std::to_string(nb_destinations_claimed_before) + " - " + std::to_string(nb_destinations_claimed_after) + ")");
                // return false;
                deadlock_found_but_attempted_resolution = true;
                break;
            }
            waiting_chain.push_back(curr_agent_idx);
            state = agents_[curr_agent_idx]->GetState();
            states_chain.push_back(AgentStateToString(state));

            // check if the current agent is idling
            if (state == IDLING){
                // the current agent is idling, so we are in a temporary deadlock
                // this agent should move to the closest possible unclaimed destination

                // however, if the agent waiting for this IDLING agent is not 
                // in a WAITING_FOR_FREE_DESTINATION state, but instead in a
                // WAITING_AT_INTERSECTION, then that agent just hasn't been
                // allowed to replan yet, and we don't need to do anything here
                // if (states_chain[states_chain.size() - 2] == AgentStateToString(WAITING_AT_INTERSECTION)){
                if (agents_[waiting_chain[waiting_chain.size() - 2]]->GetState() != WAITING_FOR_FREE_DESTINATION){
                    break;
                }

                // Note: only do this if the deadlock seems to persist
                if (nb_consecutive_deadlocks_found_ < 10){
                    // return true;
                    deadlock_found = true;
                    break;
                }

                try{
                    std::cout << "trying to resolve deadlock" << std::endl;
                    std::string nearest_free_destination = 
                        env_.GetNearestFreeClaimableDestination(
                            agents_[curr_agent_idx]->GetCurrentPosition(),
                            true//false
                        );

                    deadlock_resolving_tasks.push_back(
                        MoverTask(curr_agent_idx, nearest_free_destination,
                                  0.0*nb_simulated_samples_*simulation_time_step_, 
                                  true)); // indicate this is a deadlock resolving task
                    std::cout << "resolving task: " << deadlock_resolving_tasks[deadlock_resolving_tasks.size() - 1] << std::endl;
                } catch (InvalidEnvironmentOperationException &e){
                    // no free destination available, so we cannot resolve
                    // deadlock
                    std::cout << "unable to resolve deadlock (" << e.what() << ")" << std::endl;
                }
                std::cout << "deadlock found. Waiting chain: " << waiting_chain << std::endl;
                std::cout << "States chain:  " << states_chain << std::endl;
                // return true;
                deadlock_found = true;
                break;
            }
        }
    }

    // return false;
    if (!deadlock_found && 
            deadlock_resolving_tasks.size() == 0 &&
            !deadlock_found_but_attempted_resolution &&
            nb_simulation_steps_since_last_attempted_resolution_ > 5){
        deadlock_cycles_ = {};
        deadlocked_agents_attempted_resolution_ = {};
        nb_simulation_steps_since_last_attempted_resolution_ = 0;
    } else {
        nb_simulation_steps_since_last_attempted_resolution_++;
    }
    return deadlock_found;
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
                task->RevealTask(nb_simulated_samples_*simulation_time_step_,
                                 agents_[task->GetAgentIdx()]->GetNbTasksCompleted())){
            // check if the agent is ready for the new task
            if (agents_[task->GetAgentIdx()]->GetState() == IDLING || 
                    agents_[task->GetAgentIdx()]->GetState() == WAITING_FOR_FREE_DESTINATION){
                // set the final destination
                std::string name = task->GetDestinationName();
                Point2D<double> dest = possible_destinations_[name].
                    ConvertCellToWorld(env_.CellWidth(), env_.CellHeight());
                if (task->GetAMax() > 0){params_[task->GetAgentIdx()]->SetAmax(task->GetAMax());}
                if (task->GetVMax() > 0){params_[task->GetAgentIdx()]->SetVmax(task->GetVMax());}
                bool success = agents_[task->GetAgentIdx()]->
                    InstructToDestination(name, dest, task);
                if (!success){
                    // unable to claim destination, try again later
                    std::cout << "agent " << task->GetAgentIdx() << " unable to claim destination " 
                        << name << std::endl;
                    task->PostponeTask(0.0, 
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

void MultiMoverSimulator::AddPrioritizedAgent(std::shared_ptr<VirtualAgent>& agent){
    if (!agent){
        return;
    }

    for (int i = 0; i < prioritized_agents_.size(); i++){
        if (prioritized_agents_[i]->GetAgentIdx() == agent->GetAgentIdx()){
            // already in the list, so do not add again
            return;
        }
    }

    // add the agent to the prioitized list
    prioritized_agents_.push_back(agent);
};