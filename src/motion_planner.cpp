#include <iostream>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>
// #include <pybind11/pybind11.h>

#include <thread>
#include <future>
#include <chrono>

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"
#include "core/trajectory.hpp"

using namespace casadi;
using json = nlohmann::json;

MotionPlanner::MotionPlanner(PlannerMethod method, Parameters const &params, 
                             Environment &environment) :
        params_(params),
        environment_(environment),
        corridor_sequence_(environment_, params),
        parametrization_(corridor_sequence_, params),
        ocp_solver_(corridor_sequence_, params){
	method_ = method;

    // SetSolver("ipopt");
    // SetSolver("fatrop", true);
    SetSolver(solver_name_, true);

	// P2P method attributes
	int max_nb_corridors = corridor_sequence_.MaxNbCorridors();
	p2p_waypoints_ = std::vector<Point2D<double>>(max_nb_corridors + 1);
	coarse_samples_position_ = 
		std::vector<Point2D<double>>(1 + 3*max_nb_corridors);
	coarse_samples_velocity_ =
		std::vector<Point2D<double>>(1 + 3*max_nb_corridors);
	coarse_samples_acceleration_ =
		std::vector<Point2D<double>>(1 + 3*max_nb_corridors);
	coarse_samples_time_ = std::vector<double>(1 + 3*max_nb_corridors);
}

void MotionPlanner::SetStart(Point2D<double> start){
    if (!environment_.isValidPosition(start)){
        std::cout << start << std::endl;
        throw InvalidPositionInEnvironmentException("Invalid starting position");
    }
    start_ = start;
}

void MotionPlanner::SetRandomStart(){
    environment_.GetRandomFreeVehiclePosition(start_, params_.GetVehWidth(),
                                              params_.GetVehHeight(),
                                              params_.GetMargin());
}

void MotionPlanner::SetDest(Point2D<double> dest){
    if (!environment_.isValidPosition(dest)){
        throw InvalidPositionInEnvironmentException("Invalid destination");
    }
    dest_ = dest;
}

void MotionPlanner::SetRandomDest(){
    environment_.GetRandomFreeVehiclePosition(dest_, params_.GetVehWidth(),
                                              params_.GetVehHeight(),
                                              params_.GetMargin());
}

void MotionPlanner::UpdateCorridorSequence(){
    logger_.LogEvent(UpdatedCorridorsEvent(corridor_sequence_));
    corridor_sequence_.UpdateSequence(start_, dest_, start_vel_, params_,
                                      sequence_update_token_);
}

void MotionPlanner::UpdateCorridorSequence(const Point2D<double> &start,
                                           const Point2D<double> &dest){
    SetStart(start);
    SetDest(dest);
    UpdateCorridorSequence();
}

void MotionPlanner::Plan(){
    // Determine which function to invoke based on the selected method of the 
    // planner
    if (!silent_mode_){
        std::cout << std::endl << "=== STARTING PLANNER ===" << std::endl;
        std::cout << "Planning from " << start_ << " to " << dest_ << " with start velocity " << start_vel_ << std::endl;
    }
    logger_.LogEvent(PlannerCalledEvent(start_, dest_, start_vel_));

    // store the current solution into the previous solution
    previous_solution_ = last_solution_;

    // Start the clock
    auto planning_computation_time_start = std::chrono::high_resolution_clock::now();

    // Update the corridor sequence
    if (corridor_sequence_.CurrentlyConsideringFullSequence()){
        UpdateCorridorSequence();
        if (!silent_mode_){ std::cout << "Done updating the corridor sequence" << std::endl;}
        if (!corridor_sequence_.SequenceAvailable()){
            if (!silent_mode_){ std::cout << "No corridor sequence found to plan through." << std::endl;}
            last_solution_.Reset(start_);
            return;
        }
        if (!silent_mode_){ PrintCorridorSequence();}
    }
    switch(method_){
        case P2P:
            PlanP2P();
            break;
        case OCP:
            PlanOCP();
            break;
        case ARENA:
            PlanARENA();
            break;
        default:
            std::cout << "Invalid method selected" << std::endl;
    }

    // Stop the clock
    auto planning_computation_time_end = std::chrono::high_resolution_clock::now();

    // print out the computation time in milliseconds
    std::chrono::duration<double, std::milli> planning_computation_time = 
        planning_computation_time_end - planning_computation_time_start;
    if (!silent_mode_){ std::cout << "Planning computation time: " << planning_computation_time.count() << " ms" << std::endl;}
    last_solution_.SetTotalComputationTime(planning_computation_time.count());

    if (last_solution_.TotalComputationTime() < 0 || 
        last_solution_.SolverTime() < 0 ||
        (last_solution_.CorridorInfeasibilitiesDetected() && method_ != OCP)){

        // must be uncommented in benchmark mode
        // emergency_mode_ = true;
        // last_solution_ = previous_solution_;
        logger_.LogEvent(PlannerFailedEvent());
        throw std::runtime_error("Planner failed to find a (feasible) solution");
    } else {
        emergency_mode_ = false;
        sample_ptr_ = 0;
    }

    logger_.LogEvent(PlannerExitedEvent());
    if (!silent_mode_){std::cout << "==== ENDING PLANNER ====" << std::endl << std::endl;}
}

void MotionPlanner::Plan(const Point2D<double> &start, 
                         const Point2D<double> &dest, 
                         const Point2D<double> &start_vel){
    SetStart(start);
    SetDest(dest);
    SetStartVel(start_vel);
    Plan();
}

void MotionPlanner::PlanSafely(int max_allowed_ms){
    logger_.LogEvent(PlannerCalledSafelyEvent(start_, start_vel_, dest_));
    bool USE_THREADED_PLANNING = false;
    if (USE_THREADED_PLANNING){

        bool current_emergency_mode = emergency_mode_;

        std::packaged_task<void()> task([this](){ Plan();});
        std::future<void> result = task.get_future();

        // start planning
        std::thread planning_thread(std::move(task));

        // wait for the result
        if (result.wait_for(std::chrono::milliseconds(max_allowed_ms)) == 
                std::future_status::timeout){
            // timeout reached
            planning_thread.detach(); // This should be done in a more controlled way
            std::cerr << "WARNING: Planning took too long. Switching to emergency mode" << std::endl;
            emergency_mode_ = true;
            ComputeEmergencyBrakingTrajectory();
        } else {
            try{
                result.get();
            } catch (std::exception &e){
                std::cerr << "Caught exception: " << e.what() << std::endl;
                emergency_mode_ = true;
                ComputeEmergencyBrakingTrajectory();
            }
        }

        if (planning_thread.joinable()){
            planning_thread.join();
        }

        return;
    } else {
        bool current_emergency_mode = emergency_mode_;
        auto start = std::chrono::high_resolution_clock::now();
        try{
            // make sure this doen't take too long
            Plan();
        } catch (std::exception &e){
            if (!silent_mode_){ std::cerr << "Caught exception: " << e.what() << std::endl;}
            logger_.LogEvent(PlannerExceptionCaught(e.what()));
            // PrintPythonImplementationInfo();
            // std::cout << "parametrization:" << std::endl;
            // std::cout << parametrization_ << std::endl;

            if (start_vel_.Norm() <= 1.0e-10){
                // unable to plan with low starting velocity
                PlanConcatenatedSections();
            } else {
                // deal with issues
                if (current_emergency_mode){
                    // we were unable to recover from emergency mode
                    throw std::runtime_error("Unable to recover from emergency mode");
                }

                emergency_mode_ = true;
                ComputeEmergencyBrakingTrajectory();
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> planning_time = end - start;
        if (emergency_mode_){
            emergency_solution_.SetTotalComputationTime(planning_time.count());
        } else {
            last_solution_.SetTotalComputationTime(planning_time.count());
        }
    }

    logger_.LogEvent(SafelyPlannerExitedEvent());
}

void MotionPlanner::GetSample(double &time, Point2D<double> &pos, 
                              Point2D<double> &vel, Point2D<double> &acc){
    if (emergency_mode_){
        emergency_solution_.GetSample(emergency_sample_ptr_, time, pos, vel, acc);
        emergency_sample_ptr_++;
    } else {
        last_solution_.GetSample(sample_ptr_, time, pos, vel, acc);
        sample_ptr_++;
    }
}

int MotionPlanner::GetCurrentSampleIdx() const {
    if (emergency_mode_){
        return std::min(emergency_sample_ptr_, emergency_solution_.NbSamples());
    } else {
        return std::min(sample_ptr_, last_solution_.NbSamples());
    }
}

void MotionPlanner::SetSolver(std::string solver_name, 
                              bool update_prepared_opti_instances){
    assert (solver_name == "ipopt" || solver_name == "fatrop");
    // update_prepared_opti_instances = update_prepared_opti_instances && 
    //                                  solver_name != solver_name_;

    opts_casadi_.clear();
    opts_solver_.clear();

    solver_name_ = solver_name;
    opts_casadi_["expand"] = true;

    if (solver_name_ == "ipopt"){
        opts_solver_["linear_solver"] = "ma57";
    } else {
        opts_casadi_["structure_detection"] = "auto";
        opts_casadi_["debug"] = true;
        opts_solver_["mu_init"] = 1.0e-1;
    }
	opts_solver_["print_level"] = print_level_;
	opts_solver_["max_iter"] = method_ == OCP ? 1000 : max_iter_;
    // if (silent_mode_){ opts_casadi_["print_time"] = false;}

    std::cout << "jit: " << just_in_time_preparation_mode_ << std::endl;
    if (!just_in_time_preparation_mode_ && update_prepared_opti_instances){
        parametrization_.PrepareOptiInstances(parametrization_update_token_,
                                            solver_name_, opts_casadi_,
                                            opts_solver_);
        ocp_solver_.PrepareOptiInstances(ocp_solver_update_token_,
                                        solver_name_, opts_casadi_,
                                        opts_solver_);
    }
}

void MotionPlanner::SetJustInTimePreparationMode(bool set){
    if (!set && just_in_time_preparation_mode_){
        parametrization_.PrepareOptiInstances(parametrization_update_token_,
                                             solver_name_, opts_casadi_,
                                             opts_solver_);
        ocp_solver_.PrepareOptiInstances(ocp_solver_update_token_,
                                         solver_name_, opts_casadi_,
                                         opts_solver_);
    }
    just_in_time_preparation_mode_ = set;
}

json MotionPlanner::ToJson() const {
    json motion_planner_json;
    motion_planner_json["environment"] = environment_.ToJson();
    motion_planner_json["parameters"] = params_.ToJson();
    motion_planner_json["corridor_sequence"] = corridor_sequence_.ToJson();
    motion_planner_json["planner_method"] = PlannerMethodToString();
    if (method_ == ARENA){
        motion_planner_json["parametrization"] = parametrization_.ToJson();
        if (emergency_mode_){
            motion_planner_json["trajectory"] = emergency_solution_.ToJson();
        } else {
            motion_planner_json["trajectory"] = last_solution_.ToJson();
        }
    } else {
        if (emergency_mode_){
            motion_planner_json["trajectory"] = emergency_solution_.ToJson();
        } else {
            motion_planner_json["trajectory"] = last_solution_.ToJson();
        }
    }

    return motion_planner_json;
}

void MotionPlanner::DumpToJson(const std::string &filename, 
                               bool create_output_folder) const {
    if (create_output_folder){
        // Create output directory if it doesn't exist
        std::filesystem::create_directories("output");
    }

    // Define the full path
    std::string full_path;
    if (create_output_folder){
        full_path = "output/" + filename;
    } else {
        full_path = filename;
    }        

    json j = ToJson();

    // Write the updated content back to the file, creating it if it doesn't exist
    std::ofstream outFile(full_path);
    outFile << j.dump(4);  // Automatically creates the file if it doesn't exist
    outFile.close();
}

void MotionPlanner::PlanP2P(){
    logger_.LogEvent(MethodSpecificPlanningStarted("P2P"));
    if (start_vel_.x() != 0.0 || start_vel_.y() != 0.0){
        std::runtime_error("P2P method does not support planning with an initial velocity");
    }

    // Get the waypoints
    p2p_waypoints_ = corridor_sequence_.GetCorridorOverlapCenters();
    if (!silent_mode_){ std::cout << "Planning using P2P method from " << p2p_waypoints_[0] << " to " << p2p_waypoints_[p2p_waypoints_.size()-1] << std::endl;}
    
    for (int i = 0; i < corridor_sequence_.NbCorridors(); i++){
        PlanP2PLine(i);
    }

    int coarse_sample_idx = 3*corridor_sequence_.NbCorridors();
    coarse_samples_position_[coarse_sample_idx].CopyValues(
        p2p_waypoints_[corridor_sequence_.NbCorridors()]);
    coarse_samples_velocity_[coarse_sample_idx].SetX(0);
    coarse_samples_velocity_[coarse_sample_idx].SetY(0);
    coarse_samples_acceleration_[coarse_sample_idx].SetX(0);
    coarse_samples_acceleration_[coarse_sample_idx].SetY(0);

    last_solution_.Update(corridor_sequence_.NbCorridors(),
                          p2p_waypoints_,
                          coarse_samples_position_, 
                          coarse_samples_velocity_, 
                          coarse_samples_acceleration_, 
                          coarse_samples_time_,
                          0.0);
}

void MotionPlanner::PlanP2PLine(int start_waypoint_idx){
    int coarse_sample_idx = 3*start_waypoint_idx;

    curr_pos_.CopyValues(p2p_waypoints_[start_waypoint_idx]);
    next_pos_.CopyValues(p2p_waypoints_[start_waypoint_idx + 1]);
    curr_vel_.SetX(0.0); curr_vel_.SetY(0.0);
    curr_acc_.SetX(sign(next_pos_.x() - curr_pos_.x()) * params_.GetAmax());
    curr_acc_.SetY(sign(next_pos_.y() - curr_pos_.y()) * params_.GetAmax());

    double dist_x = std::abs(next_pos_.x() - curr_pos_.x());
    double dist_y = std::abs(next_pos_.y() - curr_pos_.y());
    double dist_bd;
    int bd = (dist_x > dist_y) ? 0 : 1;
    if (bd == 0){
        dist_bd = dist_x;
        curr_acc_.SetY(curr_acc_.y() * dist_y/dist_x);
    } else {
        dist_bd = dist_y;
        curr_acc_.SetX(curr_acc_.x() * dist_x/dist_y);
    }

    // compute distance covered while accelerating to maximum velocity
    double dist_accel = std::pow(params_.GetVmax(), 2) / 
                        (2.0 * params_.GetAmax());
    
    if (2*dist_accel > dist_bd){
        // unable to reach max velocity
        
        // accelerate to half the distance
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].CopyValues(curr_acc_);
        coarse_samples_time_[coarse_sample_idx] = 
            std::sqrt(dist_bd / params_.GetAmax());

        // integrate
        curr_pos_ += curr_acc_*
                    std::pow(coarse_samples_time_[coarse_sample_idx], 2)*0.5;
        curr_vel_ += curr_acc_*coarse_samples_time_[coarse_sample_idx];
        coarse_sample_idx++;
    
        // Add point after first acceleration to the samples
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].SetX(0.0);
        coarse_samples_acceleration_[coarse_sample_idx].SetY(0.0);
        coarse_samples_time_[coarse_sample_idx] = 0.0;
        coarse_sample_idx++;
        
        // Add point after zero coasting time
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].SetX(-curr_acc_.x());
        coarse_samples_acceleration_[coarse_sample_idx].SetY(-curr_acc_.y());
        coarse_samples_time_[coarse_sample_idx] = 
            coarse_samples_time_[coarse_sample_idx - 2];
    } else {
        // able to reach maximum velocity

        // accelerate to maximum velocity
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].CopyValues(curr_acc_);
        coarse_samples_time_[coarse_sample_idx] = 
            params_.GetVmax() / params_.GetAmax();

        curr_pos_ += curr_acc_*
                    std::pow(coarse_samples_time_[coarse_sample_idx], 2)*0.5;
        curr_vel_ += curr_acc_*coarse_samples_time_[coarse_sample_idx];
        coarse_sample_idx++;
    
        // Add point after first acceleration to the samples
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].SetX(0.0);
        coarse_samples_acceleration_[coarse_sample_idx].SetY(0.0);
        double coasting_time = (dist_bd - 2*dist_accel)/params_.GetVmax();
        coarse_samples_time_[coarse_sample_idx] = coasting_time;

        curr_pos_ += curr_vel_*coasting_time;
        coarse_sample_idx++;

        // Add point after coasting to the samples
        coarse_samples_position_[coarse_sample_idx].CopyValues(curr_pos_);
        coarse_samples_velocity_[coarse_sample_idx].CopyValues(curr_vel_);
        coarse_samples_acceleration_[coarse_sample_idx].SetX(-curr_acc_.x());
        coarse_samples_acceleration_[coarse_sample_idx].SetY(-curr_acc_.y());
        coarse_samples_time_[coarse_sample_idx] = 
            params_.GetVmax() / params_.GetAmax();
    }
}

void MotionPlanner::PlanOCP(){
    logger_.LogEvent(MethodSpecificPlanningStarted("OCP"));
    if (!silent_mode_){ std::cout << "Planning using OCP method" << std::endl;}
    ocp_solver_.Solve(ocp_solver_update_token_, solver_name_, opts_solver_,
                      opts_casadi_, just_in_time_preparation_mode_);

    std::map<std::string, DM> latest_solution = ocp_solver_.GetLatestSolution();

    DM xx_sol = latest_solution["xx"];
    DM uu_sol = latest_solution["uu"];
    DM tt_sol = latest_solution["tt"];

    double solver_time;

    solver_time = ocp_solver_.GetLatestSolverTime();
    solver_time *= 1000; // convert to milliseconds

    if (ocp_solver_.GetLatestSuccessStatus() != 1){
        solver_time = -1.0;
    }

    // Construct a time-grid for the current samples
    int N = corridor_sequence_.NbCorridors() * ocp_solver_.GetNbPointsPerCorridor();
    std::vector<double> t(N+1);
    double accumulated_time = 0.0;
    double local_dt = 0.0;
    for (int s = 0; s < corridor_sequence_.NbCorridors(); s++){
        local_dt = double(tt_sol(s*nb_points_per_corridor_)) / nb_points_per_corridor_;
        for (int i = 0; i < nb_points_per_corridor_; i++){
            t[s*nb_points_per_corridor_ + i] = accumulated_time + local_dt * i;
        }
        accumulated_time += double(tt_sol(s*nb_points_per_corridor_));
    }
    t[N] = accumulated_time;

    // Duplicate last controls
    DM last_controls = DM::zeros(2, 1);
    for (int i = 0; i < 2; i++){
        last_controls(i) = uu_sol(i, uu_sol.size2() - 1);
    }
    uu_sol = horzcat(uu_sol, last_controls);

    // Construct trajectory
    last_solution_.Update(xx_sol, uu_sol, t, solver_time, corridor_sequence_, 
                          params_);
}

void MotionPlanner::PlanARENA(){
    logger_.LogEvent(MethodSpecificPlanningStarted("ARENA"));
    if (!silent_mode_){ std::cout << "Planning using ARENA method" << std::endl;}
    
    // Try to solve a single arc
    double solver_time = 0.0;
    double sampling_time = 0.0;
    parametrization_.OptimizeSingleArc(parametrization_update_token_);
    std::set<int> problematic_corridors = CheckOutOfCorridor(solver_time);
    sampling_time += last_solution_.SamplingTime();

    // only continue if that didn't work
    if (problematic_corridors.size() > 0){
        // Initialize the parametrization
        parametrization_.UpdateParametrization(parametrization_update_token_);
        // std::cout << parametrization_ << std::endl;

        // Start the optimization loop
        bool made_modification = true;
        bool use_warm_start = false;
        while (made_modification){
            made_modification = false;

            // Solve the parametrization
            parametrization_.Solve(parametrization_update_token_, solver_name_,
                                   opts_casadi_, opts_solver_,
                                   just_in_time_preparation_mode_,
                                   use_warm_start);
            // parametrization_.OptimizeParametrization(
            //     parametrization_update_token_, solver_name_, opts_casadi_, 
            //     opts_solver_, use_warm_start);

            // Extract the solver time
            if (parametrization_.GetSolverTime() < 0){ solver_time = -1;
            } else { solver_time += parametrization_.GetSolverTime();}
                
            // Sample the trajectory and check if extra constraints are needed
            add_constraints_list_ = CheckOutOfCorridor(solver_time);
            sampling_time += last_solution_.SamplingTime();
                       
            bool added_new_constraints = add_constraints_list_.size() > 0;
            while (added_new_constraints && solver_time > 0){
                // Add the extra constraints
                if (!silent_mode_){std::cout << "adding constraints at: " << add_constraints_list_ << std::endl;}
                logger_.LogEvent(AddedAdditionalConstraintsEvent(add_constraints_list_));
                added_new_constraints = 
                    parametrization_.AddOvershootingConstraints(
                        add_constraints_list_, solver_name_, opts_casadi_, 
                        opts_solver_, just_in_time_preparation_mode_);
                // added_new_constraints = parametrization_.AddOvershootingConstraintsOld(add_constraints_list_);

                if (added_new_constraints){
                    // Extract the solver time
                    if (parametrization_.GetSolverTime() < 0){ solver_time = -1;
                    } else { solver_time += parametrization_.GetSolverTime();}

                    // Sample the trajectory and check if extra constraints are needed
                    add_constraints_list_ = CheckOutOfCorridor(solver_time);
                    sampling_time += last_solution_.SamplingTime();
                } else {
                    // If no new constraints were added, there is no change
                    // in the solution either, so we're done
                }
            }
            // If no modification was made, check if the parametrization is 
            // still sub-optimal
            if (eliminate_suboptimalities_ && solver_time > 0){
                made_modification = EliminateSubOptimalParametrization();
                use_warm_start = true;
            } 
        }
    }
    last_solution_.SetSamplingTime(sampling_time);
}


void MotionPlanner::ComputeEmergencyBrakingTrajectory(double T_scaling_factor){
    logger_.LogEvent(EmergencyBrakingPlanningStarted(start_, start_vel_));
    if (!silent_mode_){ std::cout << "Planning an emergency braking trajectory from " << start_ << " with start velocity " << start_vel_ << std::endl;}
    auto planning_computation_time_start = std::chrono::high_resolution_clock::now();
    double T_x = std::abs(start_vel_.x()) / params_.GetAmax();
    double T_y = std::abs(start_vel_.y()) / params_.GetAmax();

    // find the starting position and velocity of the free direction
    double T_bottleneck = std::max(T_x, T_y);
    double T = T_scaling_factor*T_bottleneck;
    auto GetBottlekneckPosition = +[](Point2D<double>& p){return p.x();};
    auto GetFreePosition = +[](Point2D<double>& p){return p.y();};
    auto SetBottleneckPosition = +[](Point2D<double>& p, double val){};
    auto SetFreePosition = +[](Point2D<double>& p, double val){};
    if (T_x <= T_y){
        GetBottlekneckPosition = +[](Point2D<double>& p){return p.y();};
        SetBottleneckPosition = [](Point2D<double>& p, double val){p.SetY(val);};
        GetFreePosition = +[](Point2D<double>& p){return p.x();};
        SetFreePosition = [](Point2D<double>& p, double val){p.SetX(val);};
    } else {
        GetBottlekneckPosition = +[](Point2D<double>& p){return p.x();};
        SetBottleneckPosition = [](Point2D<double>& p, double val){p.SetX(val);};
        GetFreePosition = +[](Point2D<double>& p){return p.y();};
        SetFreePosition = [](Point2D<double>& p, double val){p.SetY(val);};
    }

    double tau = 0.5*(T-std::abs(GetFreePosition(start_vel_))/params_.GetAmax()); // duration of switched arc (acceleration instead of braking)
    double p0 = GetFreePosition(start_);
    double v0 = GetFreePosition(start_vel_);

    // sample the extreme trajectories
    // std::cout << "\t(emergency): sampling the extreme trajectories" << std::endl;
    double dt = 0.001;
    std::vector<Point2D<double>> p1_samples(int(T/dt)+1);
    std::vector<Point2D<double>> p2_samples(int(T/dt)+1);

    double t = 0.0;
    int a_bottleneck = GetBottlekneckPosition(start_vel_) > 0 ? params_.GetAmax()/T_scaling_factor : -params_.GetAmax()/T_scaling_factor;
    int a_free = GetFreePosition(start_vel_) > 0 ? params_.GetAmax() : -params_.GetAmax();
    double x_min = 10^5;
    double x_max = -10^5;
    double y_min = 10^5;
    double y_max = -10^5;
    double p1, p2;
    double init_p1_accel = GetFreePosition(start_vel_) > 0 ? -a_free : a_free;
    for (int i = 0; i < p1_samples.size(); i++){
        t = i*dt;
        
        // double t_b = std::min(t, T_bottleneck);
        double t_b = t;
        SetBottleneckPosition(p1_samples[i], 
            GetBottlekneckPosition(start_) + 
            GetBottlekneckPosition(start_vel_)*t_b -
            0.5*a_bottleneck*std::pow(t_b, 2));
        SetBottleneckPosition(p2_samples[i], 
            GetBottlekneckPosition(start_) + 
            GetBottlekneckPosition(start_vel_)*t_b -
            0.5*a_bottleneck*std::pow(t_b, 2));
        
        // accelerate first before braking
        if (t < tau){
            p1 = p0 + v0*t + a_free*std::pow(t, 2)/2;
        } else {
            p1 = p0 + v0*tau + a_free*std::pow(tau, 2)/2 +
                 (v0 + a_free*tau)*(t - tau) - a_free*std::pow(t - tau, 2)/2;
            // p1 = p0 + v0*tau + a_free*std::pow(tau, 2)/2 +
            //      (v0 + a_free*tau)*(t - tau) + a_free*std::pow(t - tau, 2)/2;
        }

        // brake first before accelerating
        if (t < T - tau){
            p2 = p0 + v0*t - a_free*std::pow(t, 2)/2;
        } else {
            p2 = p0 + v0*(T-tau) - a_free*std::pow(T-tau, 2)/2 +
                (v0 - a_free*(T-tau))*(t - T + tau) + a_free*std::pow(t - T + tau, 2)/2;
        }

        SetFreePosition(p1_samples[i], std::min(p1, p2));
        SetFreePosition(p2_samples[i], std::max(p1, p2));

        // update min and max values
        x_min = std::min(x_min, std::min(p1_samples[i].x(), p2_samples[i].x()));
        x_max = std::max(x_max, std::max(p1_samples[i].x(), p2_samples[i].x()));
        y_min = std::min(y_min, std::min(p1_samples[i].y(), p2_samples[i].y()));
        y_max = std::max(y_max, std::max(p1_samples[i].y(), p2_samples[i].y()));
    }
    // std::cout << "\t\tdone" << std::endl;
    x_min -= params_.GetWidthOffset(); x_max += params_.GetWidthOffset();
    y_min -= params_.GetHeightOffset(); y_max += params_.GetHeightOffset();
    Point2D<double> bottom_left_point = Point2D<double>(x_min, y_min);
    Point2D<double> top_right_point = Point2D<double>(x_max, y_max);


    // list all obstacles to consider
    // std::cout << "\t(emergency): listing all obstacles" << std::endl;
    Point2D<int> bottom_left_cell = bottom_left_point.ConvertWorldToCell(environment_.CellWidth(), environment_.CellHeight());
    Point2D<int> top_right_cell = top_right_point.ConvertWorldToCell(environment_.CellWidth(), environment_.CellHeight());
    std::vector<Point2D<double>> obstacle_centers = {};
    std::vector<double> obstacle_widths = {};
    std::vector<double> obstacle_heights = {};
    for (int x_cell = bottom_left_cell.x(); x_cell <= top_right_cell.x(); x_cell++){
        for (int y_cell = bottom_left_cell.y(); y_cell <= top_right_cell.y(); y_cell++){
            if (!environment_.IsFree(x_cell, y_cell)){
                Point2D<double> obstacle_center = Point2D<int>(x_cell, y_cell).ConvertCellToWorld(environment_.CellWidth(), environment_.CellHeight());
                obstacle_centers.push_back(obstacle_center);
                obstacle_widths.push_back(environment_.CellWidth());
                obstacle_heights.push_back(environment_.CellHeight());
            }
        }
    }
    auto GetObstacleBottleneckSize = 
        [T_x, T_y, obstacle_widths, obstacle_heights](int i)
        { return (T_x <= T_y) ? obstacle_heights[i] : obstacle_widths[i];};
    auto GetObstacleFreeSize = 
        [T_x, T_y, obstacle_widths, obstacle_heights](int i)
        { return (T_x <= T_y) ? obstacle_widths[i] : obstacle_heights[i];};
    auto GetBottleneckOffset = [T_x, T_y, this]()
        { return (T_x <= T_y) ? params_.GetHeightOffset() : params_.GetWidthOffset();};
    auto GetFreeOffset = [T_x, T_y, this]()
        { return (T_x <= T_y) ? params_.GetWidthOffset() : params_.GetHeightOffset();};
    // std::cout << "\t\tdone" << std::endl;

    std::vector<std::vector<double>> safe_alpha_intervals = {{0, 1}};

    // std::cout << "\t(emergency): checking safe alpha values" << std::endl;
    // loop over relevant bottleneck positions and check the free position
    double free1, free2, obs1, obs2, obs_alpha_min, obs_alpha_max, alpha_min, alpha_max;
    std::vector<int> empty_intervals = {};
    std::vector<std::vector<double>> new_intervals = {};
    bool found_safe_alpha = true;
    for (int i = 0; i < p1_samples.size(); i++){
        for (int j = 0; j < obstacle_centers.size(); j++){

            // check if collision occurs
            double tolerance = 1.0e-5;
            if (std::abs(GetBottlekneckPosition(obstacle_centers[j]) - 
                         GetBottlekneckPosition(p1_samples[i])) + tolerance < 
                    GetObstacleBottleneckSize(j)/2 + GetBottleneckOffset()){
                // obstacle limits
                obs1 = GetFreePosition(obstacle_centers[j]) - GetObstacleFreeSize(j)/2 - GetFreeOffset();
                obs2 = GetFreePosition(obstacle_centers[j]) + GetObstacleFreeSize(j)/2 + GetFreeOffset();

                // check valid free positions
                free1 = std::min(GetFreePosition(p1_samples[i]), GetFreePosition(p2_samples[i]));
                free2 = std::max(GetFreePosition(p1_samples[i]), GetFreePosition(p2_samples[i]));

                // check alpha values that are in collision
                obs_alpha_min = (obs1 - free1) / (free2 - free1);
                obs_alpha_max = (obs2 - free1) / (free2 - free1);

                // update the current safe alpha intervals
                new_intervals.clear();
                for (int k = 0; k < safe_alpha_intervals.size(); k++){
                    // get interval edges in alpha-coordinates
                    alpha_min = safe_alpha_intervals[k][0];
                    alpha_max = safe_alpha_intervals[k][1];

                    // construct interval on left side of obstacle
                    if (alpha_min < obs_alpha_min){
                        // interval on left side exists
                        new_intervals.push_back({alpha_min, std::min(alpha_max, obs_alpha_min)});
                    }

                    // construct interval on right side of obstacle
                    if (alpha_max > obs_alpha_max){
                        // interval on right side exists
                        new_intervals.push_back({std::max(alpha_min, obs_alpha_max), alpha_max});
                    }
                }

                // Check if we can still continue
                if (new_intervals.size() == 0 && T_scaling_factor < 2.0){
                    // std::cout << "WARNING: No safe alpha intervals found. Increasing T_scaling_factor" << std::endl;
                    // LogEmergencyBrakingComputation(false, p1_samples, 
                    //     p2_samples, safe_alpha_intervals, 0.5, 
                    //     obstacle_centers, obstacle_widths, obstacle_heights);
                    // T_scaling_factor *= 1.2;
                    // return ComputeEmergencyBrakingTrajectory(T_scaling_factor);
                    found_safe_alpha = false;
                }

                // inefficient update of intervals
                safe_alpha_intervals.clear();
                for (int k = 0; k < new_intervals.size(); k++){
                    safe_alpha_intervals.push_back(new_intervals[k]);
                }
            }
        }
    }
    // std::cout << "\t\tdone" << std::endl;
    if (safe_alpha_intervals.size() == 0){
        std::cerr << "WARNING: No safe alpha intervals found. Using full interval" << std::endl;
        // std::cout << environment_ << std::endl;
    }

    // pick the alpha in the middle of the largest interval
    double alpha = 0.5;
    double max_interval_size = -1;
    for (int k = 0; k < safe_alpha_intervals.size(); k++){
        if (safe_alpha_intervals[k][1] - safe_alpha_intervals[k][0] > max_interval_size){
            alpha = 0.5*(safe_alpha_intervals[k][0] + safe_alpha_intervals[k][1]);
            max_interval_size = safe_alpha_intervals[k][1] - safe_alpha_intervals[k][0];
        }
    }

    // sample the trajectory
    // std::cout << "\t(emergency): preparing to sample the chosen trajectory" << std::endl;
    std::vector<double> t_x = {tau, T - 2*tau, tau};
    std::vector<double> t_y = {tau, T - 2*tau, tau};
    std::vector<double> accel_x(3, -a_bottleneck);
    std::vector<double> accel_y(3, -a_bottleneck);

    if (T_x <= T_y){
        accel_x[0] = init_p1_accel - 2*alpha*init_p1_accel;
        accel_x[1] = -a_free;
        accel_x[2] = -accel_x[0];
    } else {
        accel_y[0] = init_p1_accel - 2*alpha*init_p1_accel;
        accel_y[1] = -a_free;
        accel_y[2] = -accel_y[0];
    }
    // std::cout << "\t(emergency): sampling the chosen trajectories" << std::endl;
    // std::cout << "\t\talpha:     " << alpha << std::endl;
    // std::cout << "\t\tstart:     " << start_ << std::endl;
    // std::cout << "\t\tstart_vel: " << start_vel_ << std::endl;
    // std::cout << "\t\taccel_x:   " << accel_x << std::endl;
    // std::cout << "\t\taccel_y:   " << accel_y << std::endl;
    // std::cout << "\t\tt_x:       " << t_x << std::endl;
    // std::cout << "\t\tt_y:       " << t_y << std::endl;
    emergency_solution_.Update(start_, start_vel_, accel_x, accel_y, t_x, t_y);
    emergency_sample_ptr_ = 0;

    // print out the computation time in milliseconds
    auto planning_computation_time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> planning_computation_time = 
        planning_computation_time_end - planning_computation_time_start;
    if (!silent_mode_){std::cout << "Planning computation time: " << planning_computation_time.count() << " ms" << std::endl;}
    emergency_solution_.SetTotalComputationTime(planning_computation_time.count());

    // LogEmergencyBrakingComputation(true, p1_samples, p2_samples, 
    //     safe_alpha_intervals, alpha, obstacle_centers, obstacle_widths, 
    //     obstacle_heights);

    if (!found_safe_alpha){
        emergency_mode_ = false;
        logger_.LogEvent(EmergencyBrakingPlanningFailed("No safe alpha intervals found"));
        throw UnableToPlanEmergencyBrakingTrajectoryException();
    }
}

bool MotionPlanner::AreSequencesSeparable(MotionPlanner const &other, 
                            Point2D<double> const &collision_point, 
                            double& angle) const {
    
    CorridorSequence seq_this = GetCorridorSequence();
    CorridorSequence seq_other = other.GetCorridorSequence();

    // get indices of corridors that contain the collision point
    int corridor_idx_this = seq_this.GetIdxOfCorridorThatContainsPoint(collision_point);
    int corridor_idx_other = seq_other.GetIdxOfCorridorThatContainsPoint(collision_point);
    if (corridor_idx_this == -1 || corridor_idx_other == -1){
        throw std::runtime_error("Collision point not in any corridor");
    }

    std::cout << std::endl;
    std::cout << "checking separation of corridor " << 
        seq_this.GetCorridor(corridor_idx_this) << " and " << 
        seq_other.GetCorridor(corridor_idx_other) << std::endl;
    
    // get the edge points of the line segments to check intersection for
    Point2D<double> p1_this, p2_this, p1_other, p2_other;
    if (GetMethod() == ARENA){
        p1_this = parametrization_.GetWaypoint(corridor_idx_this);
        p2_this = parametrization_.GetWaypoint(corridor_idx_this + 1);
    } else {
        std::vector<Point2D<double>> segment_points_this = seq_this.GetCorridorOverlapCenters();
        p1_this = segment_points_this[corridor_idx_this];
        p2_this = segment_points_this[corridor_idx_this + 1];

    }
    if (other.GetMethod() == ARENA){
        p1_other = other.parametrization_.GetWaypoint(corridor_idx_other);
        p2_other = other.parametrization_.GetWaypoint(corridor_idx_other + 1);
    } else {
        std::vector<Point2D<double>> segment_points_other = seq_other.GetCorridorOverlapCenters();
        p1_other = segment_points_other[corridor_idx_other];
        p2_other = segment_points_other[corridor_idx_other + 1];
    }

    double dist_1 = (p1_this - p1_other).Norm() + (p2_this - p2_other).Norm();
    double dist_2 = (p1_this - p2_other).Norm() + (p2_this - p1_other).Norm();
    Point2D<double> a, b, v;
    if (dist_1 < dist_2){
        a = (p1_this + p1_other)*0.5;
        b = (p2_this + p2_other)*0.5;
    } else {
        a = (p1_this + p2_other)*0.5;
        b = (p2_this + p1_other)*0.5;
    }
    v = b - a;
    angle = std::atan2(v.y(), v.x());

    std::cout << "checking separation of lines " << p1_this << " - " << p2_this << " and " << p1_other << " - " << p2_other << std::endl;
    std::cout << "seperation? : " << !LineSegmentsIntersect(p1_this, p2_this, p1_other, p2_other) << std::endl;
    std::cout << "separation angle: " << angle << std::endl;
    std::cout << std::endl;

    // check if the line segments intersect
    return !LineSegmentsIntersect(p1_this, p2_this, p1_other, p2_other);
};

void MotionPlanner::SeparateVehicleFreeSpace(MotionPlanner &other,
                            Point2D<double> const &collision_point,
                            Point2D<double> const &pos_this_at_collision,
                            Point2D<double> const &pos_other_at_collision,
                            double separation_angle){
    // get the potential obstacle locations (neighbours of collision cell)
    double w = environment_.CellWidth();
    double h = environment_.CellHeight();
    Point2D<int> collision_cell = collision_point.ConvertWorldToCell(w, h);
    Point2D<double> collision_cell_center = collision_cell.ConvertCellToWorld(w, h);

    int x = collision_cell.x(); int y = collision_cell.y();
    std::vector<Point2D<int>> collision_cell_neighbours = {
        Point2D<int>(x - 1, y), // left
        Point2D<int>(x + 1, y), // right
        Point2D<int>(x, y - 1), // down
        Point2D<int>(x, y + 1), // up
    };

    // select the correct neighbour
    std::vector<double> distance_to_neighbour_edge = {
        std::abs(collision_point.x() - (collision_cell_neighbours[0].ConvertCellToWorld(w, h).x() + w/2)),
        std::abs(collision_point.x() - (collision_cell_neighbours[1].ConvertCellToWorld(w, h).x() - w/2)),
        std::abs(collision_point.y() - (collision_cell_neighbours[2].ConvertCellToWorld(w, h).y() + h/2)),
        std::abs(collision_point.y() - (collision_cell_neighbours[3].ConvertCellToWorld(w, h).y() - h/2))
    };
    auto min_it = std::min_element(distance_to_neighbour_edge.begin(), distance_to_neighbour_edge.end());
    int neighbour_idx = std::distance(distance_to_neighbour_edge.begin(), min_it);
    
    std::cout << "received separation angle: " << separation_angle << std::endl;
    while (separation_angle <= 0){ separation_angle += 3.1415926535;}
    while (separation_angle >= 3.1415926535){ separation_angle -= 3.1415926535;}
    std::cout << "modified separation angle: " << separation_angle << std::endl;
    if (separation_angle <= 0.25* 3.1415926535 || separation_angle >= 3*0.25* 3.1415926535){
        // trajectories should be pulled apart vertically
        std::cout << "pulling apart vertically" << std::endl;
        if (distance_to_neighbour_edge[2] < distance_to_neighbour_edge[3]){
            neighbour_idx = 2;
        } else {
            neighbour_idx = 3;
        }
    } else {
        // trajectories should be pulled apart horizontally
        std::cout << "pulling apart horizontally" << std::endl;
        if (distance_to_neighbour_edge[0] < distance_to_neighbour_edge[1]){
            neighbour_idx = 0;
        } else {
            neighbour_idx = 1;
        }
    }
    Point2D<double> neighbour_center = collision_cell_neighbours[neighbour_idx].ConvertCellToWorld(w, h);

    // add the obstacles
    double dist_to_collision_this = (pos_this_at_collision - neighbour_center).Norm();
    double dist_to_collision_other = (pos_other_at_collision - neighbour_center).Norm();
    if (dist_to_collision_this <= dist_to_collision_other){
       environment_.AddVirtualObstacle(collision_cell);
       other.environment_.AddVirtualObstacle(collision_cell_neighbours[neighbour_idx]);
    } else {
        environment_.AddVirtualObstacle(collision_cell_neighbours[neighbour_idx]);
        other.environment_.AddVirtualObstacle(collision_cell);
    }
};

void MotionPlanner::LogEmergencyBrakingComputation(
        bool print,
        std::vector<Point2D<double>>& p1_samples, 
        std::vector<Point2D<double>>& p2_samples,
        std::vector<std::vector<double>>& safe_alpha_intervals, double alpha,
        std::vector<Point2D<double>>& obstacle_centers,
        std::vector<double>& obstacle_widths, 
        std::vector<double>& obstacle_heights){
    emergency_trajs_1_.push_back(p1_samples);
    emergency_trajs_2_.push_back(p2_samples);
    emergency_safe_intervals_.push_back(safe_alpha_intervals);
    emergency_alphas_.push_back(alpha);
    emergency_obstacle_centers_.push_back(obstacle_centers);
    emergency_obstacle_widths_.push_back(obstacle_widths);
    emergency_obstacle_heights_.push_back(obstacle_heights);

    if (!print){return;}

    std::cout << "p1_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_trajs_1_[k].size(); i++){
            std::cout << emergency_trajs_1_[k][i];
            if (i < emergency_trajs_1_[k].size() - 1){std::cout << ", ";}
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << "p2_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_trajs_2_[k].size(); i++){
            std::cout << emergency_trajs_2_[k][i];
            if (i < emergency_trajs_2_[k].size() - 1){std::cout << ", ";}
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    // print the safe alpha intervals
    std::cout << "safe_alpha_intervals_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_safe_intervals_[k].size(); i++){
            std::cout << "[" << emergency_safe_intervals_[k][i][0] << ", " << emergency_safe_intervals_[k][i][1] << "]";
            if (i < emergency_safe_intervals_[k].size() - 1){
                std::cout << ", ";
            }
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << "alpha_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << emergency_alphas_[k];
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << "obs_center_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_obstacle_centers_[k].size(); i++){
            std::cout << emergency_obstacle_centers_[k][i];
            if (i < emergency_obstacle_centers_[k].size() - 1){std::cout << ", ";}
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << "obs_width_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_obstacle_widths_[k].size(); i++){
            std::cout << emergency_obstacle_widths_[k][i];
            if (i < emergency_obstacle_widths_[k].size() - 1){std::cout << ", ";}
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << "obs_height_list = [";
    for (int k = 0; k < emergency_trajs_1_.size(); k++){
        std::cout << "[";
        for (int i = 0; i < emergency_obstacle_heights_[k].size(); i++){
            std::cout << emergency_obstacle_heights_[k][i];
            if (i < emergency_obstacle_heights_[k].size() - 1){std::cout << ", ";}
        }
        std::cout << "]";
        if (k < emergency_trajs_1_.size() - 1){std::cout << ", ";}
    }
    std::cout << "]" << std::endl;

    std::cout << std::endl << std::endl;
}

std::set<int> MotionPlanner::CheckOutOfCorridor(double solver_time){
    last_solution_.Reset(start_);
    return last_solution_.Update(corridor_sequence_,
                                 parametrization_.GetTxSol(), 
                                 parametrization_.GetTySol(), 
                                 parametrization_.GetAlphaXSol(), 
                                 parametrization_.GetAlphaYSol(), 
                                 parametrization_.GetWaypointsSol(),
                                 parametrization_.GetWaypointVelocitiesSol(),
                                 params_,
                                 solver_time);
}

void MotionPlanner::PlanConcatenatedSections(bool recursive){
    logger_.LogEvent(ConcatenatedSectionsPlanningStarted(start_, start_vel_, dest_));
    if (!recursive){
        UpdateCorridorSequence();
        corridor_sequence_.ResetCorridorIdxs();
    }

    if (corridor_sequence_.NbCorridors() == 1){
        if (!silent_mode_){std::cout << "Planning concatenated sections: planning in a single corridor." << std::endl;}
        PlanP2P();
        return;
    }
    
    // Plan a trajectory, skipping the first corridor
    corridor_sequence_.IncrementFirstCorridorIdx();
    if (!silent_mode_){std::cout << "Planning concatenated sections: planning second part from " << corridor_sequence_.GetStart() << " to " << corridor_sequence_.GetDest() << std::endl;}
    try{ Plan();}
    catch (std::exception& e){
        if (!silent_mode_){std::cout << "Planning concatenated sections: caught exception: " << e.what() << std::endl;}
        PlanConcatenatedSections(true);
    }
    corridor_sequence_.DecrementFirstCorridorIdx();

    // store the trajectory
    Trajectory second_part_of_traj = last_solution_;

    // Plan first part using P2P
    int current_last_corridor_idx = corridor_sequence_.GetLastCorridorIdx();
    corridor_sequence_.SetLastCorridorIdx(corridor_sequence_.GetFirstCorridorIdx());
    if (!silent_mode_){std::cout << "Planning concatenated sections: planning first part from " << corridor_sequence_.GetStart() << " to " << corridor_sequence_.GetDest() << std::endl;}
    PlanP2P();
    corridor_sequence_.SetLastCorridorIdx(current_last_corridor_idx);

    // Concatenate the two trajectories
    if (!silent_mode_){std::cout << "Planning concatenated sections: concatenating trajectories." << std::endl;}
    last_solution_.Concatenate(second_part_of_traj);
}

bool MotionPlanner::EliminateSubOptimalParametrization(){
    // return false;

    bool made_modification = false;
    double tolerance = 1e-6;

    bool x_flip, y_flip;
    for (int w = 0; w < corridor_sequence_.NbCorridors()-1; w++){
        x_flip = 
            // coasting through the waypoint
            (parametrization_.GetTxSol()[w][2]   < tolerance && // no final acceleration
             parametrization_.GetTxSol()[w+1][0] < tolerance && // no first acceleration
             parametrization_.GetTxSol()[w][1]   > tolerance && // some coasting
             parametrization_.GetTxSol()[w+1][1] > tolerance)   // some coasting
            // ||
            // // unable to keep accelerating
            // (parametrization_.GetTxSol()[w][1]   < tolerance &&
            //  parametrization_.GetTxSol()[w][2]   < tolerance &&
            //  parametrization_.GetAlphaX(w) != parametrization_.GetAlphaX(w+1))
             ;
        y_flip =
            // coasting through the waypoint
            (parametrization_.GetTySol()[w][2]   < tolerance && // no final acceleration
             parametrization_.GetTySol()[w+1][0] < tolerance && // no first acceleration
             parametrization_.GetTySol()[w][1]   > tolerance && // some coasting
             parametrization_.GetTySol()[w+1][1] > tolerance)   // some coasting
            // ||
            // // unable to keep accelerating
            // (parametrization_.GetTySol()[w][1]   < tolerance &&
            //  parametrization_.GetTySol()[w][2]   < tolerance &&
            //  parametrization_.GetAlphaY(w) != parametrization_.GetAlphaY(w+1))
             ;

        // std::cout << "w: " << w << " - x_flip: " << x_flip << " - y_flip: " << y_flip << std::endl;
        if (x_flip || y_flip){           
            if (!silent_mode_){std::cout << "Flipping acceleration at waypoint " << w << " (" << x_flip << "-" << y_flip << ")" << std::endl;}
            made_modification = parametrization_.FlipAccelerationAtWaypoint(
                    parametrization_update_token_, w+1, x_flip, y_flip) ||
                    made_modification;
        }
    }

    logger_.LogEvent(EliminatedSubOptimalParametrizationEvent(made_modification));
    return made_modification;
}

std::string MotionPlanner::PlannerMethodToString() const {
    switch(method_){
        case P2P:
            return "P2P";
        case OCP:
            return "OCP";
        case ARENA:
            return "ARENA";
        default:
            return "Invalid";
    }
}

void MotionPlanner::PrintPythonImplementationInfo() const {
    std::cout << "=================================================" << std::endl;
    std::cout << "Information for python implementation" << std::endl;
    std::cout << "\tcorridors = [";
    for (int i = 0; i < GetCorridorSequence().NbCorridors(); i++){
        std::cout << "[" << GetCorridorSequence().GetCorridor(i).Xmin() << ", ";
        std::cout << GetCorridorSequence().GetCorridor(i).Xmax() << ", ";
        std::cout << GetCorridorSequence().GetCorridor(i).Ymin() << ", ";
        std::cout << GetCorridorSequence().GetCorridor(i).Ymax() << "]";
        if (i < GetCorridorSequence().NbCorridors() - 1){
            std::cout << ", ";
        }
    } std::cout << "]" << std::endl;
    std::cout << "\tcorridor_meta_data = ['nominal']*len(corridors)" << std::endl;
    std::cout << "\tp0 = [" << start_.x() << ", " << start_.y() << "]" << std::endl;
    std::cout << "\tpf = [" << dest_.x() << ", " << dest_.y() << "]" << std::endl;
    std::cout << "\tv0 = [" << start_vel_.x() << ", " << start_vel_.y() << "]" << std::endl;
    std::cout << "\tparams = {'a_max': " << params_.GetAmax() << ", 'v_max': " << params_.GetVmax() << ", 'veh_width': " << params_.GetVehWidth() << ", 'veh_height': " << params_.GetVehHeight() << ", 'M': " << params_.GetMargin() << "}" << std::endl;
    std::cout << "=================================================" << std::endl;   
}

bool MotionPlanner::LineSegmentsIntersect(Point2D<double> const &p1, 
                                          Point2D<double> const &p2, 
                                          Point2D<double> const &q1, 
                                          Point2D<double> const &q2) const {
    // Check if the line segments intersect
    double x1 = p1.x(); double y1 = p1.y();
    double x2 = p2.x(); double y2 = p2.y();
    double x3 = q1.x(); double y3 = q1.y();
    double x4 = q2.x(); double y4 = q2.y();

    double denominator = (x2 - x1)*(y4 - y3) - (y2 - y1)*(x4 - x3);
    if (denominator == 0){
        return false;
    }

    double t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) / denominator;
    double u = -((x1 - x2)*(y1 - y3) - (y1 - y2)*(x1 - x3)) / denominator;

    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
};