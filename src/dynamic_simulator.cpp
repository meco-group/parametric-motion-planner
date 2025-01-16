#include "core/dynamic_simulator.hpp"
#include "core/environment.hpp"
#include "core/corridor.hpp"

bool DynamicSimulator::Plan(const Point2D<double> &start, const Point2D<double> &dest, 
                        const Point2D<double> &start_vel){
    // Update the environment with the movable obstacles
    UpdateEnvironment();
    
    // Compute a trajectory
    double acc_solver_time = 0.0;
    int nb_runs = 10;
    for (int i = 0; i < nb_runs; i++){
        motion_planner_.Plan(start, dest, start_vel);
        acc_solver_time += motion_planner_.GetLastSolution().SolverTime();
    }

    // Get the current trajectory to simulate
    Trajectory curr_trajectory = motion_planner_.GetLastSolution();
    curr_trajectory.SetSolverTime(acc_solver_time/nb_runs);
    int current_trajectory_sample_idx = 0;
    
    // Store all trajectories computed (for visualizations)
    previous_trajectories_.clear();
    previous_corridor_sequences_.clear();
    previous_environments_.clear();

    previous_trajectories_.push_back(curr_trajectory);
    previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());
    previous_environments_.push_back(environment_.ToJson());

    // Store travelled trajectory
    travelled_trajectory_.Reset(start);

    // Simulate the environment and replan if necessary
    bool need_to_replan = false;
    replanning_times_.clear();
    Point2D<double> replan_position;
    Point2D<double> replan_velocity;
    double replan_time = 0.0;
    bool aborted = false;
    while (current_trajectory_sample_idx < curr_trajectory.NbSamples()){
        // std::cout << curr_trajectory.Px()[current_trajectory_sample_idx] << ", " <<
        //              curr_trajectory.Py()[current_trajectory_sample_idx] << std::endl;

        // Update the recording
        travelled_trajectory_.Append(
            replan_time + curr_trajectory.T()[current_trajectory_sample_idx], 
            curr_trajectory.Px()[current_trajectory_sample_idx],
            curr_trajectory.Py()[current_trajectory_sample_idx],
            curr_trajectory.Vx()[current_trajectory_sample_idx],
            curr_trajectory.Vy()[current_trajectory_sample_idx],
            curr_trajectory.Ax()[current_trajectory_sample_idx],
            curr_trajectory.Ay()[current_trajectory_sample_idx]);

        // Update the moving obstacle positions
        // std::cout << "updating moving obstacles" << std::endl;
        UpdateMovingObstacles(curr_trajectory.Dt());

        // Update the environment
        // UpdateEnvironment();

        // Check if we need to replan
        // std::cout << "checking replan trigger" << std::endl;
        need_to_replan = CheckReplanTrigger();
        // std::cout << "done" << std::endl;

        if (need_to_replan){
            std::cout << "REPLANNING" << std::endl;
            // Set replan parameters
            replan_position = Point2D<double>(
                curr_trajectory.Px()[current_trajectory_sample_idx],
                curr_trajectory.Py()[current_trajectory_sample_idx]);
            replan_velocity = Point2D<double>(
                curr_trajectory.Vx()[current_trajectory_sample_idx],
                curr_trajectory.Vy()[current_trajectory_sample_idx]);

            // Update the time of replan
            replan_time += curr_trajectory.T()[current_trajectory_sample_idx];
            replanning_times_.push_back(replan_time);

            // Update the environment
            UpdateEnvironment();

            // Replan
            const Environment& env = motion_planner_.GetEnvironment();
            try{
                Parameters params = motion_planner_.GetParameters();
                motion_planner_.SetStart(replan_position);
                motion_planner_.SetDest(dest);
                motion_planner_.SetStartVel(replan_velocity);
                motion_planner_.UpdateCorridorSequence();
                PrintPythonImplementationInfo(replan_position, replan_velocity, 
                                              dest, params);

                acc_solver_time = 0.0;
                for (int i = 0; i < nb_runs; i++){
                    motion_planner_.Plan(replan_position, dest, replan_velocity);
                    acc_solver_time += motion_planner_.GetLastSolution().SolverTime();
                }
                curr_trajectory.Reset(replan_position);
                curr_trajectory = motion_planner_.GetLastSolution();
                curr_trajectory.SetSolverTime(acc_solver_time/nb_runs);
                previous_trajectories_.push_back(curr_trajectory);
                previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());
                previous_environments_.push_back(env.ToJson());
                if (curr_trajectory.SolverTime() == -1){
                    // The planner was aborted
                    return true;
                }


            } catch (std::runtime_error &e){
                std::cerr << "Something unexpected happened during replanning" << std::endl;
                std::cerr << e.what() << std::endl;
                return true;
            }

            // Reset the current trajectory sample index
            current_trajectory_sample_idx = 1;
        } else {
            // Update the mover state
            current_trajectory_sample_idx++;
        }
    }

    return false;
}

void DynamicSimulator::MoveDestination(Point2D<double> start, 
                                       Point2D<double> start_vel,
                                       int number_of_destination_switches){    
    Reset();
    travelled_trajectory_.Reset(start);

    double curr_time = 0;
    double local_time;
    Point2D<double> curr_pos = start;
    Point2D<double> curr_vel = start_vel;
    Point2D<double> curr_acc;
    Point2D<double> dest;
    int nb_samples_to_simulate;
    bool curr_emergency_mode = false;
    bool unable_to_plan_to_dest = false;
    double traveled_time_on_previous_trajectory = 0.0;

    // Start the main
    int planner_counter = 0;
    // for (int i = 0; i < number_of_destination_switches; i++){
    while (planner_counter < number_of_destination_switches){
        // set a destination
        if (!curr_emergency_mode && !unable_to_plan_to_dest){
            environment_.GetRandomFreeVehiclePosition(dest, 
                    motion_planner_.GetParameters().GetVehWidth(),
                    motion_planner_.GetParameters().GetVehHeight(),
                    motion_planner_.GetParameters().GetMargin());
            motion_planner_.SetDest(dest);
        }

        // plan towards the destination
        motion_planner_.SetStart(curr_pos);
        motion_planner_.SetStartVel(curr_vel);
        unable_to_plan_to_dest = false;
        try{
            motion_planner_.PlanSafely(10);
        } catch (InvalidPositionInEnvironmentException &e){
            std::cerr << "Error: " << e.what() << std::endl;
            return;
        } catch (UnableToPlanEmergencyBrakingTrajectoryException &e){
            // just continue for a while on this trajectory
            // planner_counter--;
            unable_to_plan_to_dest = true;
            std::cerr << "Planner failed to plan emergency trajectory: " << e.what() << std::endl;
        } catch (std::exception &e){
            std::cerr << "Planner failed to plan: " << e.what() << std::endl;
            return;
        }
        curr_emergency_mode = motion_planner_.EmergencyMode();

        // Store replanning info
        if (curr_time > 0){replanning_times_.push_back(curr_time);}
        previous_trajectories_.push_back(motion_planner_.GetLastSolution());
        previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());
        previous_environments_.push_back(environment_.ToJson());

        // Simulate the trajectory
        nb_samples_to_simulate = motion_planner_.GetLastSolution().NbSamples();
        std::cout << "Simulating " << nb_samples_to_simulate << " samples" << std::endl;
        if (!motion_planner_.EmergencyMode() && planner_counter < number_of_destination_switches - 1){
            nb_samples_to_simulate = int(0.7*nb_samples_to_simulate);
        }
        if (unable_to_plan_to_dest){
            curr_time -= traveled_time_on_previous_trajectory;
        }
        for (int k = 1; k < nb_samples_to_simulate; k++){
            motion_planner_.GetSample(local_time, curr_pos, curr_vel, curr_acc);
            if (k == 0){
                std::cout << "\t\tpos: " << curr_pos << "\t\tvel: " << curr_vel << std::endl;    
            }
            traveled_time_on_previous_trajectory = local_time;
            travelled_trajectory_.Append(curr_time + local_time, curr_pos.x(), 
                                         curr_pos.y(), curr_vel.x(), 
                                         curr_vel.y(), curr_acc.x(), 
                                         curr_acc.y());
        }
        curr_time = curr_time + local_time;

        if (!curr_emergency_mode && !unable_to_plan_to_dest){
            planner_counter++;
        }
    }

    std::cout << "final position: " << curr_pos << std::endl;
    std::cout << "final velocity: " << curr_vel << std::endl;
}

void DynamicSimulator::Reset(){
    // reset self
    travelled_trajectory_.Reset(Point2D<double>(0, 0));
    replanning_times_.clear();
    previous_trajectories_.clear();
    previous_corridor_sequences_.clear();
    previous_environments_.clear();

    // reset obstacles
    for (auto obstacle : moving_obstacles_){
        obstacle->Reset();
    }

    // Update the cells covered by the moving obstacles
    for (auto obstacle : moving_obstacles_){
        cells_covered_by_moving_obstacles_[obstacle] = 
            environment_.GetOccupiedFootprintCells(obstacle->GetPosition(), 
                                                   obstacle->GetWidth(), 
                                                   obstacle->GetHeight(),
                                                   0.0);
    }

    // Update the environment
    UpdateEnvironment();
}

void DynamicSimulator::AddMovingObstacle(std::shared_ptr<MovingObstacle> obstacle){
    moving_obstacles_.insert(obstacle);

    // Update the cells covered by the obstacle
    cells_covered_by_moving_obstacles_[obstacle] = 
        environment_.GetOccupiedFootprintCells(obstacle->GetPosition(), 
                                               obstacle->GetWidth(), 
                                               obstacle->GetHeight(),
                                               0.0);
}

void DynamicSimulator::RemoveMovingObstacle(std::shared_ptr<MovingObstacle> obstacle){
    moving_obstacles_.erase(obstacle);

    // Remove the cells covered by the obstacle
    cells_covered_by_moving_obstacles_.erase(obstacle);
}

json DynamicSimulator::ToJson() const {
    json dynamic_simulator_json;

    // Add the moving obstacles
    json moving_obstacles_json = json::array();
    for (auto obstacle : moving_obstacles_){
        moving_obstacles_json.push_back(obstacle->ToJson());
    }
    dynamic_simulator_json["moving_obstacles"] = moving_obstacles_json;
    dynamic_simulator_json["environment"] = environment_.ToJson();
    dynamic_simulator_json["motion_planner"] = motion_planner_.ToJson();
    dynamic_simulator_json["replanning_times"] = replanning_times_;
    dynamic_simulator_json["previous_trajectories"] = json::array();
    for (auto trajectory : previous_trajectories_){
        dynamic_simulator_json["previous_trajectories"].push_back(trajectory.ToJson());
    }
    dynamic_simulator_json["previous_corridor_sequences"] = json::array();
    for (auto corridor_sequence : previous_corridor_sequences_){
        dynamic_simulator_json["previous_corridor_sequences"].push_back(corridor_sequence.ToJson());
    }
    dynamic_simulator_json["travelled_trajectory"] = travelled_trajectory_.ToJson();

    dynamic_simulator_json["previous_environments"] = json::array();
    for (auto env_json : previous_environments_){
        dynamic_simulator_json["previous_environments"].push_back(env_json);
    }

    return dynamic_simulator_json;
}

void DynamicSimulator::DumpToJson(const std::string &filename) const {
    // Create output directory if it doesn't exist
    std::filesystem::create_directories("output");

    // Define the full path
    std::string full_path = "output/" + filename;

    json j = ToJson();

    // Write the updated content back to the file, creating it if it doesn't exist
    std::ofstream outFile(full_path);
    outFile << j.dump(4);  // Automatically creates the file if it doesn't exist
    outFile.close();
}

void DynamicSimulator::UpdateEnvironment(){
    Environment::MovingObstacleOperationsToken token = 
        Environment::MovingObstacleOperationsToken();

    // Clear all moving obstacles
    environment_.ClearAllMovingObstacles();

    // Add the current occupied cells as static obstacles
    for (auto obstacle : moving_obstacles_){
        for (auto cell : cells_covered_by_moving_obstacles_[obstacle]){
            environment_.AddMovingObstacle(token, cell);
        }
    }
}

void DynamicSimulator::UpdateMovingObstacles(double dt){
    // Update positions of all moving obstacles
    for (auto obstacle : moving_obstacles_){
        obstacle->Update(dt);
    }

    // Update the cells covered by the moving obstacles
    for (auto obstacle : moving_obstacles_){
        cells_covered_by_moving_obstacles_[obstacle] = 
            environment_.GetOccupiedFootprintCells(obstacle->GetPosition(), 
                                                   obstacle->GetWidth(), 
                                                   obstacle->GetHeight(),
                                                   0.0);
    }
}

bool DynamicSimulator::CheckReplanTrigger(){
    // Check if any of the cells occupied by a moving obstacle is within the
    // current corridor sequence
    CorridorSequence curr_sequence = motion_planner_.GetCorridorSequence();

    Point2D<double> point;
    for (auto obstacle : moving_obstacles_){
        for (auto cell : cells_covered_by_moving_obstacles_[obstacle]){
            point = cell.ConvertCellToWorld(environment_.CellWidth(), 
                                            environment_.CellHeight());
            if (curr_sequence.ContainsPoint(point)){
                std::cout << "Obstacle at " << point << " is in corridor" << std::endl;
                std::cout << "cell occupancy: " << environment_.GetOccupancy(cell) << std::endl;
                return true;
            }
        }
    }
    return false;
}

void DynamicSimulator::PrintPythonImplementationInfo(
        Point2D<double>& replan_position, Point2D<double>& replan_velocity,
        const Point2D<double>& dest, const Parameters& params) const {
    std::cout << "=================================================" << std::endl;
    std::cout << "Information for python implementation" << std::endl;
    std::cout << "\t corridors = [";
    for (int i = 0; i < motion_planner_.GetCorridorSequence().NbCorridors(); i++){
        std::cout << "[" << motion_planner_.GetCorridorSequence().GetCorridor(i).Xmin() << ", ";
        std::cout << motion_planner_.GetCorridorSequence().GetCorridor(i).Xmax() << ", ";
        std::cout << motion_planner_.GetCorridorSequence().GetCorridor(i).Ymin() << ", ";
        std::cout << motion_planner_.GetCorridorSequence().GetCorridor(i).Ymax() << "]";
        if (i < motion_planner_.GetCorridorSequence().NbCorridors() - 1){
            std::cout << ", ";
        }
    } std::cout << "]" << std::endl;
    std::cout << "\tcorridor_meta_data = ['nominal']*len(corridors)" << std::endl;
    std::cout << "\tp0 = [" << replan_position.x() << ", " << replan_position.y() << "]" << std::endl;
    std::cout << "\tpf = [" << dest.x() << ", " << dest.y() << "]" << std::endl;
    std::cout << "\tv0 = [" << replan_velocity.x() << ", " << replan_velocity.y() << "]" << std::endl;
    std::cout << "\tparams = {'a_max': " << params.GetAmax() << ", 'v_max': " << params.GetVmax() << ", 'veh_width': " << params.GetVehWidth() << ", 'veh_height': " << params.GetVehHeight() << ", 'M': " << params.GetMargin() << "}" << std::endl;
    std::cout << "=================================================" << std::endl;   
}