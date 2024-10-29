#include "core/dynamic_simulator.hpp"
#include "core/environment.hpp"
#include "core/corridor.hpp"

bool DynamicSimulator::Plan(const Point2D<double> &start, const Point2D<double> &dest, 
                        const Point2D<double> &start_vel){
    // Update the environment with the movable obstacles
    UpdateEnvironment();
    
    // Compute a trajectory
    motion_planner_.Plan(start, dest, start_vel);

    // Get the current trajectory to simulate
    Trajectory curr_trajectory = motion_planner_.GetLastSolution();
    int current_trajectory_sample_idx = 0;
    
    // Store all trajectories computed (for visualizations)
    previous_trajectories_.clear();
    previous_corridor_sequences_.clear();
    previous_trajectories_.push_back(curr_trajectory);
    previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());

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
            Environment env = motion_planner_.GetEnvironment();
            try{
                motion_planner_.Plan(replan_position, dest, replan_velocity);
                curr_trajectory.Reset(replan_position);
                curr_trajectory = motion_planner_.GetLastSolution();
                previous_trajectories_.push_back(curr_trajectory);
                previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());
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

void DynamicSimulator::AddMovingObstacle(std::shared_ptr<MovingObstacle> obstacle){
    moving_obstacles_.insert(obstacle);

    // Update the cells covered by the obstacle
    cells_covered_by_moving_obstacles_[obstacle] = 
        environment_.GetOccupiedFootprintCells(obstacle->GetPosition(), 
                                               obstacle->GetWidth(), 
                                               obstacle->GetHeight());
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
                                                   obstacle->GetHeight());
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