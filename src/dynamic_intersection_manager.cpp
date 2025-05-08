#include "core/dynamic_intersection_manager.hpp"
#include "nlohmann/json.hpp"

DynamicIntersectionManager::DynamicIntersectionManager(MotionPlanner& planner1, 
                                                       MotionPlanner& planner2)
    : planner_1_(planner1), planner_2_(planner2) {
};

void DynamicIntersectionManager::SimulateSafely(const Point2D<double>& start1, 
        const Point2D<double>& dest1, const Point2D<double>& start_vel1,
        const Point2D<double>& start2, const Point2D<double>& dest2,
        const Point2D<double>& start_vel2){
    // plan trajectory 1
    planner_1_.SetStart(start1);
    planner_1_.SetDest(dest1);
    planner_1_.SetStartVel(start_vel1);
    planner_1_.PlanSafely();
    planned_trajectories_1.push_back(planner_1_.GetLastSolution());
    planned_corridor_sequences_1.push_back(planner_1_.GetCorridorSequence());
    planned_times_1.push_back(0);
    final_dest_1_ = dest1.Copy();

    // plan trajectory 2
    planner_2_.SetStart(start2);
    planner_2_.SetDest(dest2);
    planner_2_.SetStartVel(start_vel2);
    planner_2_.PlanSafely();
    planned_trajectories_2.push_back(planner_2_.GetLastSolution());
    planned_corridor_sequences_2.push_back(planner_2_.GetCorridorSequence());
    planned_times_2.push_back(0);
    final_dest_2_ = dest2.Copy();

    // check if an intersection is needed
    intersection_present_ = GetIntersection();
    if (intersection_present_){
        std::cout << "Intersection found: " << intersection_ << std::endl;
    } else {
        std::cout << "No intersection found" << std::endl;
    }

    nb_simulated_samples_ = 0;
    simulation_time_step_ = planned_trajectories_1[0].Dt();
    if (planner_2_.GetLastSolution().Dt() != simulation_time_step_){
        throw std::runtime_error("Simulation time step is not the same for both vehicles");
    }

    if (!intersection_present_){
        Simulate(TimeToNbTimeSteps(
            std::max(planner_1_.GetLastSolution().Tf(),
                     planner_2_.GetLastSolution().Tf())));
        return;
    }


    // If needed, decide which vehicle can clear the intersection first
    double first_leaving_time;
    int first_vehicle_leaving = GetFirstVehicleLeavingIntersection(first_leaving_time);

    // Tell the other vehicle to come to a stop before the intersection
    // for now, manually pick the point
    Point2D<int> stop_point;
    if (first_vehicle_leaving == 1){
        // stop_point = Point2D<int>(2, 0);
        // planner_2_.SetDest(stop_point.ConvertCellToWorld(
        //     planner_2_.GetEnvironment().CellWidth(), 
        //     planner_2_.GetEnvironment().CellHeight()));
        planner_2_.SetDest(planner_2_.GetCorridorSequence().GetWaitingPosition(
            intersection_, planner_2_.GetParameters(),
            planner_2_.GetEnvironment().CellWidth(),
            planner_2_.GetEnvironment().CellHeight()));
        planner_2_.PlanSafely();
        planner_2_.SetTrajectoryT0(0);
        planned_trajectories_2.push_back(planner_2_.GetLastSolution());
        planned_times_2.push_back(0);
    } else {
        // stop_point = Point2D<int>(1, 1);
        // planner_1_.SetDest(stop_point.ConvertCellToWorld(
        //     planner_1_.GetEnvironment().CellWidth(), 
        //     planner_1_.GetEnvironment().CellHeight()));
        planner_1_.SetDest(planner_1_.GetCorridorSequence().GetWaitingPosition(
            intersection_, planner_1_.GetParameters(), 
            planner_1_.GetEnvironment().CellWidth(),
            planner_1_.GetEnvironment().CellHeight()));
        planner_1_.PlanSafely();
        planner_2_.SetTrajectoryT0(0);
        planned_trajectories_1.push_back(planner_1_.GetLastSolution());
        planned_times_1.push_back(0);
    }

    // Simulate until the first vehicle leaves the intersection
    Simulate(TimeToNbTimeSteps(first_leaving_time + additional_intersection_waiting_time_));

    // Make the other vehicle continue
    double time_left = 0;
    if (first_vehicle_leaving == 1){
        planner_2_.SetStart(latest_simulated_pos_2_);
        planner_2_.SetStartVel(latest_simulated_vel_2_);
        planner_2_.SetDest(final_dest_2_);
        planner_2_.PlanSafely();
        planner_2_.SetTrajectoryT0(first_leaving_time);
        time_left = planner_2_.GetLastSolution().Tf();
        planned_trajectories_2.push_back(planner_2_.GetLastSolution());
        planned_times_2.push_back(first_leaving_time);

        // discard first sample
        Point2D<double> pos, vel, acc;
        double temp;
        planner_2_.GetSample(temp, pos, vel, acc);
    } else {
        planner_1_.SetStart(latest_simulated_pos_1_);
        planner_1_.SetStartVel(latest_simulated_vel_1_);
        planner_1_.SetDest(final_dest_1_);
        planner_1_.PlanSafely();
        planner_1_.SetTrajectoryT0(first_leaving_time);
        time_left = planner_1_.GetLastSolution().Tf();
        planned_trajectories_1.push_back(planner_1_.GetLastSolution());
        planned_times_1.push_back(first_leaving_time);

        // discard first sample
        Point2D<double> pos, vel, acc;
        double temp;
        planner_1_.GetSample(temp, pos, vel, acc);
    }

    // Simulate until the end of the trajectory
    Simulate(TimeToNbTimeSteps(time_left));
};

void DynamicIntersectionManager::DumpToJson(std::string const &filename) const {
    json j;
    j["intersection"] = intersection_.ToJson();

    j["vehicle_1"] = {};
    j["vehicle_1"]["planner"] = planner_1_.ToJson();
    j["vehicle_1"]["travelled_trajectory"] = travelled_trajectory_1_.ToJson();
    j["vehicle_1"]["planned_trajectories"] = json::array();
    for (const auto& traj : planned_trajectories_1){
        j["vehicle_1"]["planned_trajectories"].push_back(traj.ToJson());
    }
    j["vehicle_1"]["planned_times"] = json::array();
    for (const auto& time : planned_times_1){
        j["vehicle_1"]["planned_times"].push_back(time);
    }
    j["vehicle_1"]["planned_corridor_sequences"] = json::array();
    for (const auto& seq : planned_corridor_sequences_1){
        j["vehicle_1"]["planned_corridor_sequences"].push_back(seq.ToJson());
    }
    j["vehicle_1"]["final_dest"] = final_dest_1_.ToJson();

    j["vehicle_2"] = {};
    j["vehicle_2"]["planner"] = planner_2_.ToJson();
    j["vehicle_2"]["travelled_trajectory"] = travelled_trajectory_2_.ToJson();
    j["vehicle_2"]["planned_trajectories"] = json::array();
    for (const auto& traj : planned_trajectories_2){
        j["vehicle_2"]["planned_trajectories"].push_back(traj.ToJson());
    }
    j["vehicle_2"]["planned_times"] = json::array();
    for (const auto& time : planned_times_2){
        j["vehicle_2"]["planned_times"].push_back(time);
    }
    j["vehicle_2"]["planned_corridor_sequences"] = json::array();
    for (const auto& seq : planned_corridor_sequences_2){
        j["vehicle_2"]["planned_corridor_sequences"].push_back(seq.ToJson());
    }
    j["vehicle_2"]["final_dest"] = final_dest_2_.ToJson();
    
    std::ofstream outFile(filename);
    outFile << j.dump(4);  // Automatically creates the file if it doesn't exist
    outFile.close();
};

bool DynamicIntersectionManager::GetIntersection(){
    // Get the corridor sequences
    CorridorSequence corridors_1 = planner_1_.GetCorridorSequence();
    CorridorSequence corridors_2 = planner_2_.GetCorridorSequence();

    // compute overlapping regions
    std::vector<Corridor> overlaps = corridors_1.GetOverlap(corridors_2);

    // for now, only keep the first overlapping region
    if (overlaps.size() > 0){
        intersection_ = overlaps[0];
        return true;
    } else {
        return false;
    }
};

void DynamicIntersectionManager::Simulate(int nb_time_steps){
    Point2D<double> pos, vel, acc;
    double simulated_time = 0;
    double temp;

    for (int i = 0; i < nb_time_steps; i++){
        planner_1_.GetSample(temp, pos, vel, acc);
        travelled_trajectory_1_.Append(nb_simulated_samples_*simulation_time_step_, 
                                       pos.x(), pos.y(), vel.x(), vel.y(), 
                                       acc.x(), acc.y());
        latest_simulated_pos_1_ = pos;
        latest_simulated_vel_1_ = vel;

        planner_2_.GetSample(temp, pos, vel, acc);
        travelled_trajectory_2_.Append(nb_simulated_samples_*simulation_time_step_, 
                                       pos.x(), pos.y(), vel.x(), vel.y(), 
                                       acc.x(), acc.y());
        latest_simulated_pos_2_ = pos;
        latest_simulated_vel_2_ = vel;

        nb_simulated_samples_++;
    }
};

int DynamicIntersectionManager::TimeToNbTimeSteps(double time){
    return std::ceil(time/simulation_time_step_);
}

int DynamicIntersectionManager::GetFirstVehicleLeavingIntersection(double &first_leaving_time){
    double time_1 = GetTimeLeavingIntersection(planner_1_);
    double time_2 = GetTimeLeavingIntersection(planner_2_);

    first_leaving_time = std::min(time_1, time_2);
    
    return (time_1 < time_2) ? 1 : 2;
};

double DynamicIntersectionManager::GetTimeLeavingIntersection(MotionPlanner& planner){
    Point2D<double> pos;
    double curr_time = 0;
    Trajectory traj = planner.GetLastSolution();
    // std::cout << "traj: " << traj.ToJson() << std::endl;
    std::vector<double> px = traj.Px();
    std::vector<double> py = traj.Py();
    std::vector<double> t = traj.T();
    int nb_samples = traj.NbSamples();

    // wait until the vehicle is in the intersection
    int sample_ptr = 0;
    pos.SetValues(px[sample_ptr], py[sample_ptr]);
    while (!intersection_.ContainsPoint(pos)){
        sample_ptr++;
        if (sample_ptr >= nb_samples){
            std::cout << "Cannot find time for vehicle to enter intersection" << std::endl;
            throw std::runtime_error("Cannot find time for vehicle to enter intersection");
        }
        pos.SetValues(px[sample_ptr], py[sample_ptr]);
    }

    // wait until the vehicle is out of the intersection
    Corridor vehicle_footprint(px[sample_ptr]-planner.GetParameters().GetWidthOffset(),
                               px[sample_ptr]+planner.GetParameters().GetWidthOffset(),
                               py[sample_ptr]-planner.GetParameters().GetHeightOffset(),
                               py[sample_ptr]+planner.GetParameters().GetHeightOffset());
    Corridor o;
    while (intersection_.GetOverlap(vehicle_footprint, o)){
        sample_ptr++;
        if (sample_ptr >= nb_samples){
            std::cout << "Cannot find time for vehicle to leave intersection" << std::endl;
            throw std::runtime_error("Cannot find time for vehicle to leave intersection");
        }
        pos.SetValues(px[sample_ptr], py[sample_ptr]);
        vehicle_footprint.SetXmin(px[sample_ptr]-planner.GetParameters().GetWidthOffset());
        vehicle_footprint.SetXmax(px[sample_ptr]+planner.GetParameters().GetWidthOffset());
        vehicle_footprint.SetYmin(py[sample_ptr]-planner.GetParameters().GetHeightOffset());
        vehicle_footprint.SetYmax(py[sample_ptr]+planner.GetParameters().GetHeightOffset());
    }

    return t[sample_ptr];
};