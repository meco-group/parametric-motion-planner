#include <iostream>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>
// #include <pybind11/pybind11.h>

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
        parametrization_(corridor_sequence_, params),
        ocp_solver_(corridor_sequence_, params){
	method_ = method;

    // SetSolver("ipopt");
    SetSolver("fatrop");
	InitializeRK4();

    // ocp_solver_.PrepareOptiInstances(ocp_solver_update_token_,
    //                                  solver_name_, opts_casadi_,
    //                                  opts_solver_);
    // parametrization_.PrepareOptiInstances(parametrization_update_token_,
    //                                       solver_name_, opts_casadi_,
    //                                       opts_solver_);

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
    std::cout << "Done updating the corridor sequence" << std::endl;
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

void MotionPlanner::SetSolver(std::string solver_name){
    assert (solver_name == "ipopt" || solver_name == "fatrop");

    // if (solver_name == solver_name_){return;}
    opts_casadi_.clear();
    opts_solver_.clear();

    solver_name_ = solver_name;
    opts_casadi_["expand"] = true;

    if (solver_name_ == "ipopt"){
        opts_solver_["linear_solver"] = "ma57";
    } else {
        opts_casadi_["structure_detection"] = "auto";
        opts_casadi_["debug"] = false;
        opts_solver_["mu_init"] = 1.0e-1;
    }
	opts_solver_["print_level"] = 0;
	// opts_solver_["max_iter"] = 50;

    ocp_solver_.PrepareOptiInstances(ocp_solver_update_token_,
                                     solver_name_, opts_casadi_,
                                     opts_solver_);
    parametrization_.PrepareOptiInstances(parametrization_update_token_,
                                          solver_name_, opts_casadi_,
                                          opts_solver_);
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
    ocp_solver_.Solve(ocp_solver_update_token_);

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
    std::cout << "Planning using ARENA method" << std::endl;    
    
    // Try to solve a single arc
    double solver_time = 0.0;
    parametrization_.OptimizeSingleArc(parametrization_update_token_);
    std::set<int> problematic_corridors = CheckOutOfCorridor(solver_time);

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
            parametrization_.Solve(parametrization_update_token_, 
                                   use_warm_start);
            // parametrization_.OptimizeParametrization(
            //     parametrization_update_token_, solver_name_, opts_casadi_, 
            //     opts_solver_, use_warm_start);

            // Extract the solver time
            if (parametrization_.GetSolverTime() < 0){ solver_time = -1;
            } else { solver_time += parametrization_.GetSolverTime();}
                
            // Sample the trajectory and check if extra constraints are needed
            add_constraints_list_ = CheckOutOfCorridor(solver_time);

            bool added_new_constraints = add_constraints_list_.size() > 0;
            while (added_new_constraints && solver_time > 0){
                // Add the extra constraints
                std::cout << "adding constraints at: " << add_constraints_list_ << std::endl;
                added_new_constraints = 
                    parametrization_.AddOvershootingConstraints(
                                                    add_constraints_list_);

                if (added_new_constraints){
                    // Extract the solver time
                    if (parametrization_.GetSolverTime() < 0){ solver_time = -1;
                    } else { solver_time += parametrization_.GetSolverTime();}

                    // Sample the trajectory and check if extra constraints are needed
                    add_constraints_list_ = CheckOutOfCorridor(solver_time);
                } else {
                    // If no new constraints were added, there is no change
                    // in the solution either, so we're done
                }
            }

            // If no modification was made, check if the parametrization is 
            // still sub-optimal
            if (eliminate_suboptimalities_){
                made_modification = EliminateSubOptimalParametrization();
                use_warm_start = true;
            } 
        }
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


void MotionPlanner::ComputeEmergencyBrakingTrajectory(){
    double T_x = std::abs(start_vel_.x()) / params_.GetAmax();
    double T_y = std::abs(start_vel_.y()) / params_.GetAmax();

    double T = std::max(T_x, T_y);
    double v0; double p0;
    if (T_x <= T_y){
        p0 = start_.x();
        v0 = start_vel_.x();
    } else {
        p0 = start_.y();
        v0 = start_vel_.y();   
    }

    double tau = 0.5*(T-std::abs(v0)/params_.GetAmax()); // duration of switched arc
    int s = v0 >= 0 ? 1 : -1;
    double a = s*params_.GetAmax();

    double p1 = p0 + v0*(T - tau) - a*std::pow(T - tau, 2)/2 + 
                (v0 - a*(T - tau))*tau + a*std::pow(tau, 2)/2;
    double p2 = p0 + v0*tau + a*std::pow(tau, 2)/2 +
                (v0 + a*tau)*(T - tau) - a*std::pow(T - tau, 2)/2;
    
    double p_min = std::min(p1, p2);
    double p_max = std::max(p1, p2);

    // g can be computed using the formula g = (p - C)/B where
    double B = -2*a*T*tau;
    double C = p0 + v0*tau + a*std::pow(tau, 2)/2 + (v0 + a*tau)*(T - tau) - 
               a*std::pow(T - tau, 2)/2;

    // select a feasible final point
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
            std::cout << "Flipping acceleration at waypoint " << w << std::endl;
            made_modification = made_modification ||
                parametrization_.FlipAccelerationAtWaypoint(
                    parametrization_update_token_, w+1, x_flip, y_flip);
        }
    }

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