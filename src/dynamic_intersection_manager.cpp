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
    if (intersection_present_ && CheckForCollision()){
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


    // Check how to deal with the intersection
    GetIntersectionCase();
    if (intersection_case_ == NO_TRUE_OVERLAP || 
            intersection_case_ == NO_OVERLAP_IN_TIME){
        // no intersection
        std::cout << "No intersection" << std::endl;
        Simulate(TimeToNbTimeSteps(
        std::max(planner_1_.GetLastSolution().Tf(),
                    planner_2_.GetLastSolution().Tf())));
        return;
    } else if (intersection_case_ == INVALID_CASE){
        std::cout << "Invalid intersection case" << std::endl;
        throw std::runtime_error("Invalid intersection case");
    } else if (intersection_case_ != VEHICLE_1_MUST_WAIT &&
            intersection_case_ != VEHICLE_2_MUST_WAIT){
        std::cout << "Unclear what to do" << std::endl;
        throw std::runtime_error("Unclear what to do");
    }

    // Deal with waiting vehicles
    double first_leaving_time;
    if (intersection_case_ == VEHICLE_1_MUST_WAIT){
        std::cout << "Vehicle 1 must wait" << std::endl;
        
        // replan
        waiting_position_ = planner_1_.GetCorridorSequence().GetWaitingPosition(
                            planner_1_.GetStart(),
                            intersection_, planner_1_.GetParameters(), 
                            planner_1_.GetEnvironment().CellWidth(),
                            planner_1_.GetEnvironment().CellHeight());
        planner_1_.SetDest(waiting_position_);
        planner_1_.PlanSafely();
        planner_2_.SetTrajectoryT0(0);
        planned_trajectories_1.push_back(planner_1_.GetLastSolution());
        planned_times_1.push_back(0);    

        first_leaving_time = intersection_times_["vehicle_2"]["leaving_time"];
    } else {
        std::cout << "Vehicle 2 must wait" << std::endl;
        
        // replan
        waiting_position_ = planner_2_.GetCorridorSequence().GetWaitingPosition(
                            planner_2_.GetStart(),
                            intersection_, planner_2_.GetParameters(), 
                            planner_2_.GetEnvironment().CellWidth(),
                            planner_2_.GetEnvironment().CellHeight());
        planner_2_.SetDest(waiting_position_);
        planner_2_.PlanSafely();
        planner_1_.SetTrajectoryT0(0);
        planned_trajectories_2.push_back(planner_2_.GetLastSolution());
        planned_times_2.push_back(0);    

        first_leaving_time = intersection_times_["vehicle_1"]["leaving_time"];
    }

    if (!wait_for_clear_intersection_){
        // simulate impatiently
        bool succeeded = SimulateImpatiently(
            TimeToNbTimeSteps(first_leaving_time), 
            intersection_case_ == VEHICLE_1_MUST_WAIT ? 1 : 2);
        if (succeeded){
            Simulate(TimeToNbTimeSteps(GetTimeLeftToSimulate()));
            return;
        }
    } else {
        // Simulate until the first vehicle leaves the intersection
        Simulate(TimeToNbTimeSteps(first_leaving_time + additional_intersection_waiting_time_));
    }

    // Make the vehicle continue
    double time_left = 0;
    if (intersection_case_ == VEHICLE_1_MUST_WAIT){
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
    } else {
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
    }

    // Simulate until the end of the trajectory
    Simulate(TimeToNbTimeSteps(GetTimeLeftToSimulate()));
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

    if (overlaps.size() > 0){
        // for now, only keep the first overlapping region
        // intersection_ = overlaps[0];

        // Create a new corridor that contains all the overlaps
        // (if multiple separate intersections are present, this is a bad idea)
        intersection_ = overlaps[0].Copy();
        for (int i = 1; i < overlaps.size(); i++){
            intersection_.SetXmin(std::min(intersection_.Xmin(), overlaps[i].Xmin()));
            intersection_.SetXmax(std::max(intersection_.Xmax(), overlaps[i].Xmax()));
            intersection_.SetYmin(std::min(intersection_.Ymin(), overlaps[i].Ymin()));
            intersection_.SetYmax(std::max(intersection_.Ymax(), overlaps[i].Ymax()));
        }

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

bool DynamicIntersectionManager::SimulateImpatiently(int nb_time_steps, 
                                                     int waiting_veh_idx){
    if (wait_for_clear_intersection_){
        throw std::runtime_error("Cannot simulate impatiently when waiting for clear intersection");
    }

    Point2D<double> pos, vel, acc;
    double simulated_time = 0;
    double temp;
    bool collision_found = false;

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

        // Attempt to let the waiting vehicle go
        if (waiting_veh_idx == 1){
            planner_1_.SetStart(latest_simulated_pos_1_);
            planner_1_.SetStartVel(latest_simulated_vel_1_);
            planner_1_.SetDest(final_dest_1_);
            planner_1_.PlanSafely();

            collision_found = CheckForCollision();

            if (collision_found){
                // revert to the last solution
                planner_1_.SetDest(waiting_position_);
                planner_1_.PlanSafely();
            } else {
                // abort simulation
                planned_corridor_sequences_1.push_back(planner_1_.GetCorridorSequence());
                planned_trajectories_1.push_back(planner_1_.GetLastSolution());
                planned_times_1.push_back(nb_simulated_samples_*simulation_time_step_);
                return true;
            }
        } else {
            planner_2_.SetStart(latest_simulated_pos_2_);
            planner_2_.SetStartVel(latest_simulated_vel_2_);
            planner_2_.SetDest(final_dest_2_);
            planner_2_.PlanSafely();

            collision_found = CheckForCollision();

            if (collision_found){
                // revert to the last solution
                planner_2_.SetDest(waiting_position_);
                planner_2_.PlanSafely();
            } else {
                // abort simulation
                planned_corridor_sequences_2.push_back(planner_2_.GetCorridorSequence());
                planned_trajectories_2.push_back(planner_2_.GetLastSolution());
                planned_times_2.push_back(nb_simulated_samples_*simulation_time_step_);
                return true;
            }
        }
    }

    return false;
};

bool DynamicIntersectionManager::CheckForCollision(){
    std::vector<double> px1 = planner_1_.GetLastSolution().Px();
    std::vector<double> py1 = planner_1_.GetLastSolution().Py();
    int sample_ptr_1 = planner_1_.GetCurrentSampleIdx();
    int nb_samples_1 = planner_1_.GetLastSolution().NbSamples();
    double wo_1 = planner_1_.GetParameters().GetWidthOffset() + collision_check_margin_/2;
    double ho_1 = planner_1_.GetParameters().GetHeightOffset() + collision_check_margin_/2;

    std::vector<double> px2 = planner_2_.GetLastSolution().Px();
    std::vector<double> py2 = planner_2_.GetLastSolution().Py();
    int sample_ptr_2 = planner_2_.GetCurrentSampleIdx();
    int nb_samples_2 = planner_2_.GetLastSolution().NbSamples();
    double wo_2 = planner_2_.GetParameters().GetWidthOffset() + collision_check_margin_/2;
    double ho_2 = planner_2_.GetParameters().GetHeightOffset() + collision_check_margin_/2;

    Corridor footprint_1;
    Corridor footprint_2;
    Corridor o;
    while (sample_ptr_1 < nb_samples_1 || sample_ptr_2 < nb_samples_2){
        footprint_1.SetXmin(px1[std::min(nb_samples_1-1, sample_ptr_1)] - wo_1);
        footprint_1.SetXmax(px1[std::min(nb_samples_1-1, sample_ptr_1)] + wo_1);
        footprint_1.SetYmin(py1[std::min(nb_samples_1-1, sample_ptr_1)] - ho_1);
        footprint_1.SetYmax(py1[std::min(nb_samples_1-1, sample_ptr_1)] + ho_1);

        footprint_2.SetXmin(px2[std::min(nb_samples_2-1, sample_ptr_2)] - wo_2);
        footprint_2.SetXmax(px2[std::min(nb_samples_2-1, sample_ptr_2)] + wo_2);
        footprint_2.SetYmin(py2[std::min(nb_samples_2-1, sample_ptr_2)] - ho_2);
        footprint_2.SetYmax(py2[std::min(nb_samples_2-1, sample_ptr_2)] + ho_2);

        if (footprint_1.GetOverlap(footprint_2, o)){
            std::cout << "Collision detected" << std::endl;
            return true;
        }

        sample_ptr_1++;
        sample_ptr_2++;
    }

    return false;
};

double DynamicIntersectionManager::GetTimeLeftToSimulate(){
    return std::max(
        planner_1_.GetLastSolution().Tf() - planner_1_.GetLastSolution().T()[planner_1_.GetCurrentSampleIdx()],
        planner_2_.GetLastSolution().Tf() - planner_2_.GetLastSolution().T()[planner_2_.GetCurrentSampleIdx()]
    );
}

int DynamicIntersectionManager::TimeToNbTimeSteps(double time){
    return std::ceil(time/simulation_time_step_);
}

void DynamicIntersectionManager::GetIntersectionCase(){
    intersection_times_.clear();
    intersection_times_["vehicle_1"] = GetTimeEnteringAndLeavingIntersection(planner_1_);
    intersection_times_["vehicle_2"] = GetTimeEnteringAndLeavingIntersection(planner_2_);   

    std::cout << "intersection_times: " << intersection_times_ << std::endl; 
    std::cout << "intersection_times[vehicle_1]: " << intersection_times_["vehicle_1"] << std::endl;
    std::cout << "intersection_times[vehicle_2]: " << intersection_times_["vehicle_2"] << std::endl;
    std::cout << "intersection_times[vehicle_1][entering_time]: " << intersection_times_["vehicle_1"]["entering_time"] << std::endl;
    std::cout << "intersection_times[vehicle_2][entering_time]: " << intersection_times_["vehicle_2"]["entering_time"] << std::endl;
    std::cout << intersection_times_["vehicle_1"]["entering_time"] << " " << intersection_times_["vehicle_2"]["entering_time"] << std::endl;
    std::cout << (intersection_times_["vehicle_1"]["entering_time"]  < 0) << " " << (intersection_times_["vehicle_2"]["entering_time"] < 0) << std::endl;

    // check if both vehicles actually enter the intersection
    if (intersection_times_["vehicle_1"]["entering_time"]  < 0 ||
            intersection_times_["vehicle_2"]["entering_time"] < 0){
        std::cout << "NO TRUE OVERLAP" << std::endl;
        intersection_case_ = NO_TRUE_OVERLAP;
    
    // if both vehicles stay in the intersection, we're in trouble
    } else if (intersection_times_["vehicle_1"]["leaving_time"] < 0 &&
                intersection_times_["vehicle_2"]["leaving_time"] < 0){
        std::cout << "INVALID CASE" << std::endl;
        intersection_case_ = INVALID_CASE;

    // if one vehicle never leaves, that one must wait
    } else if (intersection_times_["vehicle_1"]["leaving_time"] < 0){
        std::cout << "VEHICLE 1 MUST WAIT" << std::endl;
        intersection_case_ = VEHICLE_1_MUST_WAIT;
    } else if (intersection_times_["vehicle_2"]["leaving_time"] < 0){
        std::cout << "VEHICLE 2 MUST WAIT" << std::endl;
        intersection_case_ = VEHICLE_2_MUST_WAIT;

    // check if vehicles plan to be in intersection at the same time
    } else if (intersection_times_["vehicle_1"]["leaving_time"] < 
                intersection_times_["vehicle_2"]["entering_time"] ||
               intersection_times_["vehicle_2"]["leaving_time"] < 
                intersection_times_["vehicle_1"]["entering_time"]){
        std::cout << "NO OVERLAP IN TIME" << std::endl;
        intersection_case_ = NO_OVERLAP_IN_TIME;

    // determine which vehicle must wait
    } else if (intersection_times_["vehicle_1"]["leaving_time"] < 
               intersection_times_["vehicle_2"]["leaving_time"]){
        std::cout << "VEHICLE 2 MUST WAIT" << std::endl;
        intersection_case_ = VEHICLE_2_MUST_WAIT;
    } else {
        std::cout << "VEHICLE 1 MUST WAIT" << std::endl;
        intersection_case_ = VEHICLE_1_MUST_WAIT;
    }
};

std::map<std::string, double> DynamicIntersectionManager::GetTimeEnteringAndLeavingIntersection(MotionPlanner& planner){
    Point2D<double> pos;
    double curr_time = 0;
    Trajectory traj = planner.GetLastSolution();
    // std::cout << "traj: " << traj.ToJson() << std::endl;
    std::vector<double> px = traj.Px();
    std::vector<double> py = traj.Py();
    std::vector<double> t = traj.T();
    int nb_samples = traj.NbSamples();

    std::map<std::string, double> result;

    // wait until a part of the vehicle is in the intersection
    int sample_ptr = 0;
    Corridor o;
    Corridor vehicle_footprint(px[sample_ptr]-planner.GetParameters().GetVehWidth()/2,
                               px[sample_ptr]+planner.GetParameters().GetVehWidth()/2,
                               py[sample_ptr]-planner.GetParameters().GetVehHeight()/2,
                               py[sample_ptr]+planner.GetParameters().GetVehHeight()/2);
    // pos.SetValues(px[sample_ptr], py[sample_ptr]);
    // while (!intersection_.ContainsPoint(pos)){
    while (!intersection_.GetOverlap(vehicle_footprint, o)){
        sample_ptr++;
        if (sample_ptr >= nb_samples){
            std::cout << "Cannot find time for vehicle to enter intersection" << std::endl;
            // throw std::runtime_error("Cannot find time for vehicle to enter intersection");
            break;
        }
        pos.SetValues(px[sample_ptr], py[sample_ptr]);
        vehicle_footprint.SetXmin(px[sample_ptr]-planner.GetParameters().GetVehWidth()/2);
        vehicle_footprint.SetXmax(px[sample_ptr]+planner.GetParameters().GetVehWidth()/2);
        vehicle_footprint.SetYmin(py[sample_ptr]-planner.GetParameters().GetVehHeight()/2);
        vehicle_footprint.SetYmax(py[sample_ptr]+planner.GetParameters().GetVehHeight()/2);
    }

    if (sample_ptr >= nb_samples){
        result["entering_time"] = -1;
        result["leaving_time"] = -1;
        return result;
    } else {
        std::cout << "entered intersection at t[" << sample_ptr << "]: " << t[sample_ptr] << std::endl;
    }
    result["entering_time"] = t[sample_ptr];

    // wait until the vehicle is fully out of the intersection
    while (intersection_.GetOverlap(vehicle_footprint, o)){
        sample_ptr++;
        if (sample_ptr >= nb_samples){
            std::cout << "Cannot find time for vehicle to leave intersection" << std::endl;
            // throw std::runtime_error("Cannot find time for vehicle to leave intersection");
        }
        pos.SetValues(px[sample_ptr], py[sample_ptr]);
        vehicle_footprint.SetXmin(px[sample_ptr]-planner.GetParameters().GetVehWidth()/2);
        vehicle_footprint.SetXmax(px[sample_ptr]+planner.GetParameters().GetVehWidth()/2);
        vehicle_footprint.SetYmin(py[sample_ptr]-planner.GetParameters().GetVehHeight()/2);
        vehicle_footprint.SetYmax(py[sample_ptr]+planner.GetParameters().GetVehHeight()/2);
    }

    if (sample_ptr >= nb_samples){
        result["leaving_time"] = -1;
        return result;
    } else {
        std::cout << "left intersection at t[" << sample_ptr << "]: " << t[sample_ptr] << std::endl;
    }

    result["leaving_time"] = t[sample_ptr];

    return result;
};