#include <iostream>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>

#include "core/motion_planner.hpp"
#include "core/corridor.hpp"
#include "core/trajectory.hpp"

using namespace casadi;
using json = nlohmann::json;

MotionPlanner::MotionPlanner(PlannerMethod method, Parameters const &params, 
                             Environment const &environment) :
        params_(params),
        environment_(environment),
        corridor_sequence_(environment_, params),
        parametrization_(corridor_sequence_, params) {

	method_ = method;
	opts_solver_["print_level"] = 0;
	// opts_solver_["max_iter"] = 50;
	InitializeRK4();

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
        throw InvalidPositionInEnvironmentException("Invalid starting position");
    }
    start_ = start;
}

void MotionPlanner::SetRandomStart(){
    environment_.GetRandomFreeVehiclePosition(start_, params_.GetVehWidth(),
                                              params_.GetVehHeight());
}

void MotionPlanner::SetDest(Point2D<double> dest){
    if (!environment_.isValidPosition(dest)){
        throw InvalidPositionInEnvironmentException("Invalid destination");
    }
    dest_ = dest;
}

void MotionPlanner::SetRandomDest(){
    environment_.GetRandomFreeVehiclePosition(dest_, params_.GetVehWidth(),
                                              params_.GetVehHeight());
}

void MotionPlanner::UpdateCorridorSequence(){
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

    std::cout << "Planning from " << start_ << " to " << dest_ << " with start velocity " << start_vel_ << std::endl;
    
    // Start the clock
    auto planning_computation_time_start = std::chrono::high_resolution_clock::now();

    // Update the corridor sequence
    UpdateCorridorSequence();
    if (!corridor_sequence_.SequenceAvailable()){
        std::cout << "No corridor sequence found to plan through." << std::endl;
        last_solution_.Reset(start_);
        return;
    }
    PrintCorridorSequence();
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
    std::cout << "Planning computation time: " << planning_computation_time.count() << " ms" << std::endl;
    last_solution_.SetTotalComputationTime(planning_computation_time.count());

    // std::cout << "Solution obtained:" << std::endl;
    // std::cout << last_solution_ << std::endl;
}

void MotionPlanner::Plan(const Point2D<double> &start, 
                         const Point2D<double> &dest, 
                         const Point2D<double> &start_vel){
    SetStart(start);
    SetDest(dest);
    SetStartVel(start_vel);
    Plan();
}

json MotionPlanner::ToJson() const {
    json motion_planner_json;
    motion_planner_json["environment"] = environment_.ToJson();
    motion_planner_json["parameters"] = params_.ToJson();
    motion_planner_json["corridor_sequence"] = corridor_sequence_.ToJson();
    motion_planner_json["planner_method"] = PlannerMethodToString();
    if (method_ == ARENA){
        motion_planner_json["parametrization"] = parametrization_.ToJson();
        motion_planner_json["trajectory"] = last_solution_.ToJson();
    } else {
        motion_planner_json["trajectory"] = last_solution_.ToJson();
    }

    return motion_planner_json;
}

void MotionPlanner::DumpToJson(const std::string &filename) const {
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

void MotionPlanner::PlanP2P(){
    std::cout << "Planning using P2P method" << std::endl;
    
    if (start_vel_.x() != 0.0 || start_vel_.y() != 0.0){
        std::runtime_error("P2P method does not support planning with an initial velocity");
    }

    // Get the waypoints
    p2p_waypoints_ = 
        helper_.GetCorridorOverlapCenters(corridor_sequence_, start_, dest_);
    
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
    std::cout << "Planning using OCP method" << std::endl;

    int nb_points_per_corridor = 30;
    int N = corridor_sequence_.NbCorridors() * nb_points_per_corridor;

    // Construct OCP
    Opti opti = Opti(); 
    MX xx = opti.variable(4, N+1);
    MX uu = opti.variable(2, N);
    MX tt = opti.variable(1, corridor_sequence_.NbCorridors());

    // initial and terminal constraints
    opti.subject_to(xx(0, 0) == start_.x());
    opti.subject_to(xx(1, 0) == start_.y());
    opti.subject_to(xx(2, 0) == start_vel_.x());
    opti.subject_to(xx(3, 0) == start_vel_.y());
    
    opti.subject_to(xx(0, N) == dest_.x());
    opti.subject_to(xx(1, N) == dest_.y());
    opti.subject_to(xx(2, N) == 0);
    opti.subject_to(xx(3, N) == 0);

    // basic box constraints
    opti.subject_to(-params_.GetAmax() <= (uu <= params_.GetAmax()));
    opti.subject_to(tt > 0);

    // Prepare looping over corridors
    MX obj = 0;
    int k_offset = 0;
    std::vector<Point2D<MX>> corners(4);
    Corridor current_corridor;
    std::vector<Point2D<double>> initialization_waypoints = 
        helper_.GetCorridorOverlapCenters(corridor_sequence_, start_, dest_);
    double initialization_distance = 0;
    double width_offset = params_.GetVehWidth()/2.0 + params_.GetMargin();
    double height_offset = params_.GetVehHeight()/2.0 + params_.GetMargin();

    // Start looping over corridors
    for (int s = 0; s < corridor_sequence_.NbCorridors(); s++){
        obj += tt(s);
        current_corridor = corridor_sequence_.GetCorridor(s);

        // initialize the time of the corridor
        initialization_distance = initialization_waypoints[s].Distance(
            initialization_waypoints[s+1]);
        opti.set_initial(tt(s), initialization_distance/params_.GetVmax());

        // loop over time-steps
        k_offset = s * nb_points_per_corridor;
        for (int k = k_offset; k < k_offset + nb_points_per_corridor; k++){
            // add dynamics
            rk4_arguments_[0] = xx(Slice(), k);
            rk4_arguments_[1] = uu(Slice(), k);
            rk4_arguments_[2] = tt(s)/nb_points_per_corridor;
            rk4_outputs_ = rk4_(rk4_arguments_);
            opti.subject_to(xx(Slice(), k + 1) == rk4_outputs_[0]);
            
            // enforce corner points to be inside the current corridor
            corners[0].SetValues(xx(0, k) - width_offset, 
                                 xx(1, k) - height_offset);
            corners[1].SetValues(xx(0, k) + width_offset,
                                 xx(1, k) - height_offset);
            corners[2].SetValues(xx(0, k) + width_offset,
                                 xx(1, k) + height_offset);
            corners[3].SetValues(xx(0, k) - width_offset,
                                 xx(1, k) + height_offset);
            for (Point2D<MX> corner : corners){
                opti.subject_to(current_corridor.Xmin() <= 
                        (corner.x() <= current_corridor.Xmax()));
                opti.subject_to(current_corridor.Ymin() <= 
                        (corner.y() <= current_corridor.Ymax()));
            }

            // Add max velocity constraint
            opti.subject_to(-params_.GetVmax() <= 
                            (xx(Slice(2,4), k) <= params_.GetVmax()));

            // Add initial guess
            opti.set_initial(xx(0, k), 
                initialization_waypoints[s].x() + 
                (k - k_offset)*(initialization_waypoints[s+1].x() - 
                initialization_waypoints[s].x())/nb_points_per_corridor);
            opti.set_initial(xx(1, k), 
                initialization_waypoints[s].y() + 
                (k - k_offset)*(initialization_waypoints[s+1].y() - 
                initialization_waypoints[s].y())/nb_points_per_corridor);
        }

        // the final point of a corridor should also be enforced to be 
        // within the next corridor to prevent corner cutting (if there 
        // exists a next corridor)
        if (s < corridor_sequence_.NbCorridors() - 1){
            int k = k_offset + nb_points_per_corridor;
            // current_corridor = corridor_sequence_.GetCorridor(s+1);
            corners[0].SetValues(xx(0, k) - width_offset, 
                                 xx(1, k) - height_offset);
            corners[1].SetValues(xx(0, k) + width_offset,
                                 xx(1, k) - height_offset);
            corners[2].SetValues(xx(0, k) + width_offset,
                                 xx(1, k) + height_offset);
            corners[3].SetValues(xx(0, k) - width_offset,
                                 xx(1, k) + height_offset);
            for (Point2D<MX> corner : corners){
                opti.subject_to(current_corridor.Xmin() <= 
                        (corner.x() <= current_corridor.Xmax()));
                opti.subject_to(current_corridor.Ymin() <= 
                        (corner.y() <= current_corridor.Ymax()));
            }
        }

    }

    opti.minimize(obj);
    opti.solver("ipopt", opts_casadi_, opts_solver_);
    
    DM xx_sol, uu_sol, tt_sol;
    double solver_time;
    try {
        OptiSol sol = opti.solve();
        
        // Extract solution
        xx_sol = sol.value(xx);
        uu_sol = sol.value(uu);
        tt_sol = sol.value(tt);

        solver_time = sol.stats()["t_wall_total"];
        solver_time *= 1000; // convert to milliseconds

    } catch (std::exception &e){
        std::cout << "An error occurred: " << e.what() << std::endl;
        xx_sol = opti.debug().value(xx);
        uu_sol = opti.debug().value(uu);
        tt_sol = opti.debug().value(tt);

        solver_time = -1;
    }

    // Construct a time-grid for the current samples
    std::vector<double> t(N+1);
    double accumulated_time = 0.0;
    double local_dt = 0.0;
    for (int s = 0; s < corridor_sequence_.NbCorridors(); s++){
        local_dt = double(tt_sol(s)) / nb_points_per_corridor;
        for (int i = 0; i < nb_points_per_corridor; i++){
            t[s*nb_points_per_corridor + i] = accumulated_time + local_dt * i;
        }
        accumulated_time += double(tt_sol(s));
    }
    t[N] = accumulated_time;

    // Duplicate last controls
    DM last_controls = DM::zeros(2, 1);
    for (int i = 0; i < 2; i++){
        last_controls(i) = uu_sol(i, uu_sol.size2() - 1);
    }
    uu_sol = horzcat(uu_sol, last_controls);

    // Construct trajectory
    last_solution_.Update(xx_sol, uu_sol, t, solver_time);
}

void MotionPlanner::PlanARENA(){
    std::cout << "Planning using ARENA method" << std::endl;    
    
    // Try to solve a single arc
    double solver_time = 0.0;
    parametrization_.OptimizeSingleArc(parametrization_update_token_);
    std::set<int> problematic_corridors = CheckOutOfCorridor(solver_time);

    // only continue if that didn't work
    if (problematic_corridors.size() > 0){
        // Initialize the parametrization
        parametrization_.UpdateParametrization(parametrization_update_token_);
        std::cout << parametrization_ << std::endl;

        // Start the optimization loop
        bool made_modification = true;
        // while (made_modification){
            // Solve the parametrization
            parametrization_.OptimizeParametrization(
                parametrization_update_token_, opts_casadi_, opts_solver_);
            solver_time += parametrization_.GetSolverTime();

            add_constraints_list_ = CheckOutOfCorridor(solver_time);

            // Check if additional constraints are required
            if (add_constraints_list_.size() > 0 && solver_time > 0){
                parametrization_.AddOvershootingConstraints(add_constraints_list_);
                solver_time += parametrization_.GetSolverTime();
                add_constraints_list_ = CheckOutOfCorridor(solver_time);
            }

            // Check if the parametrization is still sub-optimal
            made_modification = EliminateSubOptimalParametrization();
        // }
    }

    // Update the solution
    // std::cout << "updating solution " << std::endl;
    // last_solution_.Update(corridor_sequence_,
    //                       parametrization_.GetTxSol(), 
    //                       parametrization_.GetTySol(), 
    //                       parametrization_.GetAlphaXSol(), 
    //                       parametrization_.GetAlphaYSol(), 
    //                       parametrization_.GetWaypointsSol(),
    //                       parametrization_.GetWaypointVelocitiesSol(),
    //                       params_);
}

void MotionPlanner::InitializeRK4(){
    MX xk = MX::sym("xk", 4);
    MX uk = MX::sym("uk", 2);
    MX dt = MX::sym("dt");

    Function rhs = Function("rhs", {xk, uk}, {vertcat(xk(2), xk(3), uk(0), uk(1))});
    std::vector<MX> rhs_arguments = {xk, uk};
    MX k1 = dt*rhs(rhs_arguments)[0];

    rhs_arguments[0] = xk + 0.5*k1;
    MX k2 = dt*rhs(rhs_arguments)[0];

    rhs_arguments[0] = xk + 0.5*k2;
    MX k3 = dt*rhs(rhs_arguments)[0];

    rhs_arguments[0] = xk + k3;
    MX k4 = dt*rhs(rhs_arguments)[0];

    rk4_ = Function("rk4", {xk, uk, dt}, {xk + (k1 + 2*k2 + 2*k3 + k4)/6});
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

bool MotionPlanner::EliminateSubOptimalParametrization(){
    return false;
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