#include <cmath>
#include <iostream>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>
#include <bitset>

#include "core/parametrization.hpp"
#include "core/trajectory.hpp"

using namespace casadi;
using json = nlohmann::json;

Parametrization::Parametrization(CorridorSequence const &corridor_sequence,
								 Parameters const &params)
    : corridor_sequence_(corridor_sequence),
	  params_(params),
      max_nb_corridors_(corridor_sequence.MaxNbCorridors()),
	  opti_(Opti())
	  {
	// true parameter variables
	alpha_x_ = std::vector<double>(max_nb_corridors_ + 1),
	alpha_y_ = std::vector<double>(max_nb_corridors_ + 1);
	waypoints_ = std::vector<Point2D<double>>(max_nb_corridors_ + 1); 
	max_waypoint_offsets_ = std::vector<Point2D<double>>(max_nb_corridors_ + 1);
	movable_waypoints_ = std::vector<bool>(max_nb_corridors_ + 1, false);
	waypoint_locations_ = std::vector<WaypointLocation>(max_nb_corridors_ + 1);

	// mx objects used in the optimization
	alpha_x_mx_ = MX(max_nb_corridors_ + 1, 1);
	alpha_y_mx_ = MX(max_nb_corridors_ + 1, 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

	// initialization containers
	t_x_init_ = std::vector<std::vector<double>>(max_nb_corridors_);
	t_y_init_ = std::vector<std::vector<double>>(max_nb_corridors_);
	waypoint_velocities_init_ = std::vector<Point2D<double>>(max_nb_corridors_ + 1);
	for (int i = 0; i < max_nb_corridors_; i++){
		t_x_init_[i] = std::vector<double>(3);
		t_y_init_[i] = std::vector<double>(3);
	}

	// optimized values
	alpha_x_sol_ = std::vector<double>(max_nb_corridors_ + 1);
	alpha_y_sol_ = std::vector<double>(max_nb_corridors_ + 1);
	waypoints_sol_ = std::vector<Point2D<double>>(max_nb_corridors_ + 1);
	waypoint_velocities_sol_ = std::vector<Point2D<double>>(max_nb_corridors_ + 1);
	t_x_sol_ = std::vector<std::vector<double>>(max_nb_corridors_);
	t_y_sol_ = std::vector<std::vector<double>>(max_nb_corridors_);
	for (int i = 0; i < max_nb_corridors_; i++){
		t_x_sol_[i] = std::vector<double>(3);
		t_y_sol_[i] = std::vector<double>(3);
	}

	// scratch space
	candidate_waypoints_ = std::vector<Point2D<double>>(4);
	candidate_valid_ = std::vector<bool>(4);
	candidate_alpha_x_ = std::vector<double>(4);
	candidate_alpha_y_ = std::vector<double>(4);

	intermediate_positions_ = std::vector<Point2D<MX>>(3 + 2 + 2*nb_fine_grid_samples_);
	intermediate_velocities_ = std::vector<Point2D<MX>>(3);
}

void Parametrization::PrepareOptiInstances(const UpdateToken&, 
						  std::string& solver_name_,
						  casadi::Dict& opts_casadi, 
						  casadi::Dict& opts_solver){
	std::cout << "preparing ARENA opti instances..." << std::endl;
	std::string movable_points_code;
	for (int i = 1; i < 9; i++){
		// std::cout << "preparing opti instances for " << i << " corridors" << std::endl;
		for (int nb_movable_waypoints = 0; nb_movable_waypoints < std::pow(2,i-1); nb_movable_waypoints++){
			movable_points_code = std::bitset<32>(nb_movable_waypoints).to_string();
			movable_points_code = movable_points_code.substr(32 - (i-1));
			movable_points_code = "0" + movable_points_code + "0";
			// std::cout << "\t" << movable_points_code << std::endl;
			PrepareSingleOptiInstance(i, solver_name_, opts_casadi, opts_solver, movable_points_code);
		}
	}
	std::cout << "\t\tDone!" << std::endl;
}

void Parametrization::Solve(const UpdateToken&, std::string& solver_name, 
							Dict& opts_casadi, Dict& opts_solver,
							bool just_in_time_preparation_mode,
							bool use_warm_start){
	int n = corridor_sequence_.NbCorridors();

	// create the code describing the movable waypoints
	std::string code = "";
	for (int i = 0; i < n+1; i++){
		if (movable_waypoints_[i]){ code += "1";
		} else { code += "0";}
	}

	if (use_warm_start){
		active_opti_inputs_["x_init"] = latest_solution_["opti_x"];
	} else {
		// Check if the opti_instance is prepared
		if (just_in_time_preparation_mode || 
			prepared_opti_instances_[n].find(code) == prepared_opti_instances_[n].end()){
			PrepareSingleOptiInstance(n, solver_name, opts_casadi, opts_solver, code);
			// std::cout << "Prepared opti instance for " << n << " corridors with code " << code << std::endl;
		}
		active_opti_instance_ = prepared_opti_instances_[n][code];
		active_opti_inputs_ = opti_inputs_[n][code];
		active_opti_code_ = code;

		// apply initial guess
		InitializeOptimization();
		int var_ptr = 0;
		for (int i = 0; i < n; i++){
			active_opti_inputs_["x_init"](var_ptr) = waypoint_velocities_init_[i].x();
			active_opti_inputs_["x_init"](var_ptr+1) = waypoint_velocities_init_[i].y();
			// active_opti_inputs_["x_init"](var_ptr) = waypoint_velocities_init_[n-1].x();
			// active_opti_inputs_["x_init"](var_ptr+1) = waypoint_velocities_init_[n-1].y();
			var_ptr += 2;

			if (i > 0 && code[i] == '1'){
				active_opti_inputs_["x_init"](Slice(var_ptr, var_ptr+2)) = 0;
				var_ptr += 2;
			}

			active_opti_inputs_["x_init"](Slice(var_ptr, var_ptr+3)) = t_x_init_[i];
			active_opti_inputs_["x_init"](Slice(var_ptr+3, var_ptr+6)) = t_y_init_[i];
			var_ptr += 6;

			if (i == 0){
				active_opti_inputs_["x_init"](var_ptr) = 0;
				active_opti_inputs_["x_init"](var_ptr+1) = alpha_0_init_;
				var_ptr += 2;
			}

			if (i == n-1){
				active_opti_inputs_["x_init"](var_ptr) = 0;
				active_opti_inputs_["x_init"](var_ptr+1) = alpha_f_init_;
				var_ptr += 2;
			}
		}
		active_opti_inputs_["x_init"](var_ptr) = waypoint_velocities_init_[n].x();
		active_opti_inputs_["x_init"](var_ptr+1) = waypoint_velocities_init_[n].y();
		// active_opti_inputs_["x_init"](var_ptr) = waypoint_velocities_init_[n-1].x();
		// active_opti_inputs_["x_init"](var_ptr+1) = waypoint_velocities_init_[n-1].y();

		// Set parameter values
		int offset_ptr = 0;
		for (int i = 0; i < n+1; i++){
			active_opti_inputs_["waypoints"](0,i) = waypoints_[i].x();
			active_opti_inputs_["waypoints"](1,i) = waypoints_[i].y();

			active_opti_inputs_["alpha"](0,i) = alpha_x_[i];
			active_opti_inputs_["alpha"](1,i) = alpha_y_[i];

			if (i < n){
				active_opti_inputs_["corridors"](0,i) = corridor_sequence_.GetCorridor(i).Xmin();
				active_opti_inputs_["corridors"](1,i) = corridor_sequence_.GetCorridor(i).Xmax();
				active_opti_inputs_["corridors"](2,i) = corridor_sequence_.GetCorridor(i).Ymin();
				active_opti_inputs_["corridors"](3,i) = corridor_sequence_.GetCorridor(i).Ymax();

				active_opti_inputs_["parabolic_slacks"](0,i) = 1.0e100;
				active_opti_inputs_["parabolic_slacks"](1,i) = 1.0e100;
				active_opti_inputs_["parabolic_slacks"](2,i) = 1.0e100;
				active_opti_inputs_["parabolic_slacks"](3,i) = 1.0e100;
			}

			if (i > 0 && i < n && code[i] == '1'){
				if (max_waypoint_offsets_[i].x() > 0){
					active_opti_inputs_["movable_distances"](0,offset_ptr) = 0;
					active_opti_inputs_["movable_distances"](1,offset_ptr) = max_waypoint_offsets_[i].x();
				} else {
					active_opti_inputs_["movable_distances"](0,offset_ptr) = max_waypoint_offsets_[i].x();
					active_opti_inputs_["movable_distances"](1,offset_ptr) = 0;
				}
				if (max_waypoint_offsets_[i].y() > 0){
					active_opti_inputs_["movable_distances"](2,offset_ptr) = 0;
					active_opti_inputs_["movable_distances"](3,offset_ptr) = max_waypoint_offsets_[i].y();
				} else {
					active_opti_inputs_["movable_distances"](2,offset_ptr) = max_waypoint_offsets_[i].y();
					active_opti_inputs_["movable_distances"](3,offset_ptr) = 0;
				}
				offset_ptr++;
			}
		}	

		if (std::abs(waypoints_[0].x() - waypoints_[1].x()) <
			std::abs(waypoints_[0].y() - waypoints_[1].y())){
			active_opti_inputs_["initial_bottleneck"] = 0;
		} else {
			active_opti_inputs_["initial_bottleneck"] = 1;
		}
			
		if (std::abs(waypoints_[n].x() - waypoints_[n-1].x()) <
			std::abs(waypoints_[n].y() - waypoints_[n-1].y())){
			active_opti_inputs_["final_bottleneck"] = 0;
		} else {
			active_opti_inputs_["final_bottleneck"] = 1;
		}

		active_opti_inputs_["vmax"] = DM(params_.GetVmax());
		active_opti_inputs_["amax"] = DM(params_.GetAmax());
		active_opti_inputs_["veh_width"] = DM(params_.GetVehWidth());
		active_opti_inputs_["veh_height"] = DM(params_.GetVehHeight());
		active_opti_inputs_["margin"] = DM(params_.GetMargin());
		active_opti_inputs_["start_vel"](0) = corridor_sequence_.GetStartVel().x();
		active_opti_inputs_["start_vel"](1) = corridor_sequence_.GetStartVel().y();

		// overshooting-prevention constraint
		double local_alpha, dist_waypoints, starting_vel_bd;
		if (std::abs(waypoints_[0].x() - waypoints_[1].x()) > 
			std::abs(waypoints_[0].y() - waypoints_[1].y())){
			// x is bottleneck
			local_alpha = alpha_x_[0];
			dist_waypoints = waypoints_[1].x() - waypoints_[0].x();
			starting_vel_bd = corridor_sequence_.GetStartVel().x();
		} else {
			// y is bottleneck
			local_alpha = alpha_y_[0];
			dist_waypoints = waypoints_[1].y() - waypoints_[0].y();
			starting_vel_bd = corridor_sequence_.GetStartVel().y();
		}
		bool cond_correct_direction = (starting_vel_bd > 0 ? 1.0 : -1.0) == 
			 					  	  (dist_waypoints > 0 ? 1.0 : -1.0);
		bool cond_braking = (local_alpha > 0 ? 1.0 : -1.0) == 
							(starting_vel_bd > 0 ? -1.0 : 1.0);
		bool cond_braking_distance = 0.5*std::pow(starting_vel_bd, 2)/
									 params_.GetAmax() > std::abs(dist_waypoints);

		active_opti_inputs_["t_init_limits"](0) = 1000;
		active_opti_inputs_["t_init_limits"](1) = 1000;
		if (cond_correct_direction && cond_braking && cond_braking_distance){
			double t_limit = (std::abs(starting_vel_bd) - 
						  	  std::sqrt(std::pow(starting_vel_bd, 2) - 
							  			2*params_.GetAmax()*
										std::abs(dist_waypoints)))/
						 				(params_.GetAmax());
			if (std::abs(waypoints_[0].x() - waypoints_[1].x()) > 
				std::abs(waypoints_[0].y() - waypoints_[1].y())){
				active_opti_inputs_["t_init_limits"](0) = t_limit;
			} else {
				active_opti_inputs_["t_init_limits"](1) = t_limit;
			}
		}
	}


	std::vector<DM> input = {
		active_opti_inputs_["x_init"],
		active_opti_inputs_["vmax"],
		active_opti_inputs_["amax"],
		active_opti_inputs_["veh_width"],
		active_opti_inputs_["veh_height"],
		active_opti_inputs_["margin"],
		active_opti_inputs_["waypoints"],
		active_opti_inputs_["alpha"],
		active_opti_inputs_["initial_bottleneck"],
		active_opti_inputs_["final_bottleneck"],
		active_opti_inputs_["start_vel"],
		active_opti_inputs_["corridors"],
		active_opti_inputs_["parabolic_slacks"],
		active_opti_inputs_["movable_distances"],
		active_opti_inputs_["t_init_limits"]
	};

	// std::cout << "solving opti instance " << active_opti_instance_.name() << std::endl;
	std::vector<DM> output = active_opti_instance_(input);
	latest_solution_.clear();
	latest_solution_["t_x"] = output[0];
	latest_solution_["t_y"] = output[1];
	latest_solution_["v_x"] = output[2];
	latest_solution_["v_y"] = output[3];
	latest_solution_["alpha_x"] = output[4];
	latest_solution_["alpha_y"] = output[5];
	latest_solution_["opti_x"] = output[7];
	latest_solution_["offsets"] = output[9];

	// std::cout << "Return status: " << active_opti_instance_.stats()["return_status"] << std::endl;

	latest_success_status_ = active_opti_instance_.stats()["success"];
	int nb_iterations = active_opti_instance_.stats()["iter_count"];
	// temp_.push_back(latest_success_status_);
	temp_.push_back(nb_iterations);
	// std::cout << "success statusses: " << std::endl;
	// std::cout << "number of iterations: " << std::endl;
	// for (int i = 0; i < temp_.size(); i++){
	// 	std::cout << temp_[i] << " ";
	// }
	// std::cout << std::endl;
	// if (nb_iterations == 0){
	// 	casadi::Dict d = active_opti_instance_.stats();
	// 	// print d keys one by one
	// 	for (auto const& element : d){
	// 		std::cout << element.first << " = " << element.second << std::endl;
	// 	}
	// }

	// std::cout << "Tx_sol: "	<< latest_solution_["t_x"] << std::endl;
	// std::cout << "Ty_sol: "	<< latest_solution_["t_y"] << std::endl;

	////////////////////////
	/// Extract solution ///
	////////////////////////
	ExtractSolution();
}

void Parametrization::UpdateParametrization(const UpdateToken&){
	// Check if the corridor sequence has changed
	if (use_smart_update_ &&
			corridor_sequence_.GetVersion() == latest_sequence_version_){
		std::cout << "NOTE: skipped update of parametrization." << std::endl;
		return;
	}

	// take care of the first and the last waypoint (start and dest)
	waypoints_[0] = corridor_sequence_.GetStart();
	waypoints_[corridor_sequence_.NbCorridors()] = corridor_sequence_.GetDest();
	waypoint_locations_[0] = WaypointLocation::Start;
	waypoint_locations_[corridor_sequence_.NbCorridors()] = 
		WaypointLocation::Dest;

	movable_waypoints_[0] = false;
	movable_waypoints_[corridor_sequence_.NbCorridors()] = false;

	// Compute the rest of the waypoints
	for (int i = 1; i < corridor_sequence_.NbCorridors(); i++){
		ComputeSingleWaypoint(i);
	}

	// perform second sweep
	for (int i = 1; i < corridor_sequence_.NbCorridors(); i++){
		ComputeSingleWaypoint(i, true);
	}

	// Update accelerations at start and dest based on distance only
	alpha_x_[0] = sign(waypoints_[1].x() - waypoints_[0].x());
	alpha_y_[0] = sign(waypoints_[1].y() - waypoints_[0].y());
	int final_idx = corridor_sequence_.NbCorridors();
	alpha_x_[final_idx] = -sign(waypoints_[final_idx].x() - 
							    waypoints_[final_idx - 1].x());
	alpha_y_[final_idx] = -sign(waypoints_[final_idx].y() -
							    waypoints_[final_idx - 1].y());

	// For the initial acceleration, consider the single arc solution
	OptimizeSingleArc1D(t_x_sol_[0], alpha_x_sol_, 
						waypoints_[0].x(), waypoints_[1].x(),
						corridor_sequence_.GetStartVel().x());
	OptimizeSingleArc1D(t_y_sol_[0], alpha_y_sol_,
						waypoints_[0].y(), waypoints_[1].y(),
						corridor_sequence_.GetStartVel().y());
	double t_x = t_x_sol_[0][0] + t_x_sol_[0][1] + t_x_sol_[0][2];
	double t_y = t_y_sol_[0][0] + t_y_sol_[0][1] + t_y_sol_[0][2];
	if (t_x > t_y){
		initial_bottleneck_direction_ = 0;
		alpha_x_[0] = alpha_x_sol_[0];
	} else {
		initial_bottleneck_direction_ = 1;
		alpha_y_[0] = alpha_y_sol_[0];
	}

	// Update the parametrization based on line of sights
	nb_movable_waypoints_ = 0;
	for (int i = 1; i < corridor_sequence_.NbCorridors(); i++){
		UpdateParametrizationWithLineOfSight(i);
		if (movable_waypoints_[i]){ nb_movable_waypoints_++;}
	}

	flipped_acceleration_x_ = std::vector<bool>(corridor_sequence_.NbCorridors() + 1, false);
	flipped_acceleration_y_ = std::vector<bool>(corridor_sequence_.NbCorridors() + 1, false);
	added_constraints_list_first_arc_.clear();
	added_constraints_list_second_arc_.clear();

	// Update version
	latest_sequence_version_ = corridor_sequence_.GetVersion();
};

void Parametrization::OptimizeParametrization(const UpdateToken&,
											  std::string& solver_name,
											  Dict& opts_casadi,
											  Dict& opts_solver,
											  bool use_prev_sol_as_init_guess){		

	if (optimization_problem_name_ == "original"){
	// prepare initial guess
	DM prev_sol;
	if (use_prev_sol_as_init_guess){
		prev_sol = sol_.value().value(opti_.x());
		// opti_.set_initial(opti_.x(), sol_.value().value(opti_.x()));
	} else {
		InitializeOptimization();
	}
	// ShowInitialization();

	// reset mx containers
	alpha_x_mx_ = MX(max_nb_corridors_ + 1, 1);
	alpha_y_mx_ = MX(max_nb_corridors_ + 1, 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

	// start the optimization
	opti_ = Opti();

	////////////////////////////////////////////
	/// Definition of optimization variables ///
	////////////////////////////////////////////
	//	time durations	
	t_x_ = opti_.variable(3, corridor_sequence_.NbCorridors());
	t_y_ = opti_.variable(3, corridor_sequence_.NbCorridors());
	for (int i = 0; i < corridor_sequence_.NbCorridors(); i++){
		opti_.set_initial(t_x_(Slice(), i), t_x_init_[i]);
		opti_.set_initial(t_y_(Slice(), i), t_y_init_[i]);
	}
	
	//	basic box constraints
	opti_.subject_to(0 <= (t_x_ <= 100));
	opti_.subject_to(0 <= (t_y_ <= 100));

	// Initialize parametrization MX objects and
	// Create movable waypoints
	MX offsets = opti_.variable(2, nb_movable_waypoints_);
	int offset_idx = 0;
	for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
		alpha_x_mx_(i) = alpha_x_[i]; 
		alpha_y_mx_(i) = alpha_y_[i];
		waypoints_mx_[i].CopyValues(waypoints_[i]);
	
		// if the waypoint is movable, make it so
		if (movable_waypoints_[i] && i > 0 && i <= corridor_sequence_.NbCorridors() - 1){
			// Check horizontal moving range
			if (max_waypoint_offsets_[i].x() > 0){
				opti_.subject_to(0 <= (offsets(0, offset_idx) <= 
									 max_waypoint_offsets_[i].x()));
			} else {
				opti_.subject_to(max_waypoint_offsets_[i].x() <= 
								(offsets(0, offset_idx) <= 0));
			}

			// Check vertical moving range
			if (max_waypoint_offsets_[i].y() > 0){
				opti_.subject_to(0 <= (offsets(1, offset_idx) <= 
									 max_waypoint_offsets_[i].y()));
			} else {
				opti_.subject_to(max_waypoint_offsets_[i].y() <= 
								(offsets(1, offset_idx) <= 0));
			}

			// Update waypoint
			waypoints_mx_[i].SetX(waypoints_mx_[i].x() + 
								  alpha_x_mx_(i)*offsets(0, offset_idx));
			waypoints_mx_[i].SetY(waypoints_mx_[i].y() + 
								  alpha_y_mx_(i)*offsets(1, offset_idx));

			offset_idx++;
		}
	}

	// corridor entry velocities
	// NOTE: in theory, only n velocities are needed (instead of n+1) but it 
	// makes the implementation slightly easier
	v_x_ = opti_.variable(corridor_sequence_.NbCorridors() + 0*1);
	v_y_ = opti_.variable(corridor_sequence_.NbCorridors() + 0*1);
	MX curr_v_x = corridor_sequence_.GetStartVel().x();
	MX curr_v_y = corridor_sequence_.GetStartVel().y();
	for (int i = 0; i < corridor_sequence_.NbCorridors(); i++){
		opti_.set_initial(v_x_, waypoint_velocities_init_[i].x());
		opti_.set_initial(v_y_, waypoint_velocities_init_[i].y());
	}

	// free initial acceleration
	MX s_0 = opti_.variable();
	if (std::abs(waypoints_[0].x() - waypoints_[1].x()) < 
			std::abs(waypoints_[0].y() - waypoints_[1].y())){
		alpha_x_mx_(0) = opti_.variable();
		opti_.set_initial(alpha_x_mx_(0), alpha_0_init_);
		if (RELAX_INITIAL_ACCELERATION_){
			opti_.subject_to(-1 - pow(s_0, 2) <= 
							(alpha_x_mx_(0) <= 1 + pow(s_0, 2)));
		} else {
			opti_.subject_to(alpha_x_mx_(0) == alpha_0_init_);
		}
	} else {
		alpha_y_mx_(0) = opti_.variable();
		opti_.set_initial(alpha_y_mx_(0), alpha_0_init_);
		if (RELAX_INITIAL_ACCELERATION_){
			opti_.subject_to(-1 - pow(s_0, 2) <= 
							(alpha_y_mx_(0) <= 1 + pow(s_0, 2)));
		} else {
			opti_.subject_to(alpha_y_mx_(0) == alpha_0_init_);
		}
	}

	// free final acceleration
	MX s_N = opti_.variable();
	if (std::abs(waypoints_[corridor_sequence_.NbCorridors()].x() - 
				waypoints_[corridor_sequence_.NbCorridors() - 1].x()) < 
			std::abs(waypoints_[corridor_sequence_.NbCorridors()].y() - 
				waypoints_[corridor_sequence_.NbCorridors() - 1].y())){
		alpha_x_mx_(corridor_sequence_.NbCorridors()) = opti_.variable();
		opti_.set_initial(alpha_x_mx_(corridor_sequence_.NbCorridors()), 
						 alpha_f_init_);
		if (RELAX_FINAL_ACCELERATION_){
			opti_.subject_to(-1 - pow(s_N, 2) <= 
							(alpha_x_mx_(corridor_sequence_.NbCorridors()) <= 
							 1 + pow(s_N, 2)));
		} else {
			opti_.subject_to(alpha_x_mx_(corridor_sequence_.NbCorridors()) == 
							 alpha_f_init_);
		}
	} else {
		alpha_y_mx_(corridor_sequence_.NbCorridors()) = opti_.variable();
		opti_.set_initial(alpha_y_mx_(corridor_sequence_.NbCorridors()), 
						 alpha_f_init_);
		if (RELAX_FINAL_ACCELERATION_){
			opti_.subject_to(-1 - pow(s_N, 2) <= 
							(alpha_y_mx_(corridor_sequence_.NbCorridors()) <= 
							 1 + pow(s_N, 2)));
		} else {
			opti_.subject_to(alpha_y_mx_(corridor_sequence_.NbCorridors()) == 
							 alpha_f_init_);
		}
	}
	MX obj = 0;
	if (RELAX_INITIAL_ACCELERATION_){ obj += 1.0e3*pow(s_0, 2);}
	if (RELAX_FINAL_ACCELERATION_){ obj += 1.0e3*pow(s_N, 2);}

	// add free initial velocity
	// if (RELAX_INITIAL_VELOCITY_ && 
	// 		std::abs(corridor_sequence_.GetStartVel().x()) > 1.0e-3 &&
	// 		std::abs(corridor_sequence_.GetStartVel().y()) > 1.0e-3){
	MX s_x = opti_.variable(); MX s_y = opti_.variable();
	if (RELAX_INITIAL_VELOCITY_){
		curr_v_x += s_x; curr_v_y += s_y;
		obj += 1.0e2*(casadi::sq(s_x) + casadi::sq(s_y));

		opti_.set_initial(s_x, waypoint_velocities_init_[0].x() - 
							  corridor_sequence_.GetStartVel().x());
		opti_.set_initial(s_y, waypoint_velocities_init_[0].y() -
							  corridor_sequence_.GetStartVel().y());
	}
	
	
	////////////////////////////////
	/// Definiton of constraints ///
	////////////////////////////////
	// constraint to prevent initial overshooting
	ApplyOvershootingPreventionConstraint(opti_, t_x_, t_y_);
	
	double width_offset = params_.GetWidthOffset();
	double height_offset = params_.GetHeightOffset();
	Point2D<MX> curr_waypoint;
	Point2D<MX> next_waypoint;
	Corridor curr_corridor;
	double position_tolerance = 1.0e-5;
	for (int w = 0; w < corridor_sequence_.NbCorridors(); w++){
		auto planning_computation_time_start = std::chrono::high_resolution_clock::now();
		curr_waypoint = waypoints_mx_[w];
		next_waypoint = waypoints_mx_[w+1];
		corridor_sequence_.GetCorridor(w, curr_corridor);

		// add gap-closing constraints on velocity
		opti_.subject_to(v_x_(w) == curr_v_x);
		opti_.subject_to(v_y_(w) == curr_v_y);

		// this assignment is needed to "cut the single shooting chain"
		curr_v_x = v_x_(w);
		curr_v_y = v_y_(w);

		// get some positions of relevance
		IntegrateOverCorridor(curr_waypoint, Point2D<MX>(curr_v_x, curr_v_y), 
							  t_x_(Slice(), w), t_y_(Slice(), w),
							  alpha_x_mx_(w), alpha_y_mx_(w),
							  alpha_x_mx_(w+1), alpha_y_mx_(w+1), params_.GetAmax());
		
		// add gap-closing constraints on positions
		opti_.subject_to(next_waypoint.x() - position_tolerance <=
					   (intermediate_positions_[2].x() <=
						next_waypoint.x() + position_tolerance));
		opti_.subject_to(next_waypoint.y() - position_tolerance <=
					   (intermediate_positions_[2].y() <=
						next_waypoint.y() + position_tolerance));

		// constrain equal time
		opti_.subject_to(t_x_(0, w) + t_x_(1, w) + t_x_(2, w) == 
						 t_y_(0, w) + t_y_(1, w) + t_y_(2, w));

		// constrain positions and velocities
		for (int i : {0, 1}){
			// NOTE: for some unknown reason, the double-sided inequalities
			// can make the solver fail.
			opti_.subject_to(curr_corridor.Xmin() + width_offset <= intermediate_positions_[i].x());
			opti_.subject_to(intermediate_positions_[i].x() <= curr_corridor.Xmax() - width_offset);
			opti_.subject_to(curr_corridor.Ymin()  + height_offset <= intermediate_positions_[i].y());
			opti_.subject_to(intermediate_positions_[i].y() <= curr_corridor.Ymax() - height_offset);
		}

		opti_.subject_to(-params_.GetVmax() <= 
			(intermediate_velocities_[0].x() <= params_.GetVmax()));
		opti_.subject_to(-params_.GetVmax() <= 
			(intermediate_velocities_[0].y() <= params_.GetVmax()));

		// integrate the velocity over this corridor
		curr_v_x = intermediate_velocities_[2].x();
		curr_v_y = intermediate_velocities_[2].y();

		if (w < corridor_sequence_.NbCorridors() - 1){
			opti_.subject_to(-params_.GetVmax() <= 
				(curr_v_x <= params_.GetVmax()));
			opti_.subject_to(-params_.GetVmax() <=
				(curr_v_y <= params_.GetVmax()));
		}
		
		// constrain exit velocity
		// TODO (is this needed?)

		// update objective term
		obj += t_x_(0, w) + t_x_(1, w) + t_x_(2, w);
	}

	// Add terminal velocity constraint
	double velocity_relaxation_tolerance = 1.0e-5;
	opti_.subject_to(-velocity_relaxation_tolerance <= 
					(curr_v_x <= velocity_relaxation_tolerance));
	opti_.subject_to(-velocity_relaxation_tolerance <=
					(curr_v_y <= velocity_relaxation_tolerance));


	//////////////////////////////////
	/// Finish problem formulation ///
	//////////////////////////////////
	opti_.minimize(obj);
	opti_.solver(solver_name, opts_casadi, opts_solver);

	//////////////////
	/// Warm-start ///
	//////////////////
	if (use_prev_sol_as_init_guess){
		opti_.set_initial(opti_.x(), prev_sol);
	}

	////////////////////////
	/// Extract solution ///
	////////////////////////
	ExtractSolutionOld();
	}

	if (optimization_problem_name_ == "new formulation"){
	// prepare initial guess
	DM prev_sol;
	if (use_prev_sol_as_init_guess){
		prev_sol = sol_.value().value(opti_.x());
		// opti_.set_initial(opti_.x(), sol_.value().value(opti_.x()));
	} else {
		InitializeOptimization();
	}
	// ShowInitialization();

	// reset mx containers
	alpha_x_mx_ = MX(max_nb_corridors_ + 1, 1);
	alpha_y_mx_ = MX(max_nb_corridors_ + 1, 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

	// start the optimization
	opti_ = Opti();
	// my_slack_parameter_ = opti_.parameter();

	////////////////////////////////////////////
	/// Definition of optimization variables ///
	////////////////////////////////////////////
	int n = corridor_sequence_.NbCorridors();
	std::vector<MX> v_x_MX(n+1);
	std::vector<MX> v_y_MX(n+1);
	std::vector<MX> t_x_MX(n);
	std::vector<MX> t_y_MX(n);
	std::vector<MX> offsets_MX(n);

	MX s_0, alpha_0, s_N, alpha_N;
	for (int k = 0; k < n; k++){
		// Update MX objects
		alpha_x_mx_(k) = alpha_x_[k]; 
		alpha_y_mx_(k) = alpha_y_[k];
		waypoints_mx_[k].CopyValues(waypoints_[k]);

		// Create variables
		v_x_MX[k] = opti_.variable();
		v_y_MX[k] = opti_.variable();
		t_x_MX[k] = opti_.variable(3, 1);
		t_y_MX[k] = opti_.variable(3, 1);
		if (k == 0 && RELAX_INITIAL_ACCELERATION_){
			s_0 = opti_.variable();
			alpha_0 = opti_.variable();
			opti_.set_initial(alpha_0, alpha_0_init_);
			if (std::abs(waypoints_[0].x() - waypoints_[1].x()) < 
				std::abs(waypoints_[0].y() - waypoints_[1].y())){
				alpha_x_mx_(0) = alpha_0;
			} else {
				alpha_y_mx_(0) = alpha_0;
			}
		}
		if (k == n - 1 && RELAX_FINAL_ACCELERATION_){
			s_N = opti_.variable();
			alpha_N = opti_.variable();
			opti_.set_initial(alpha_N, alpha_f_init_);
			if (std::abs(waypoints_[n].x() - waypoints_[n-1].x()) < 
				std::abs(waypoints_[n].y() - waypoints_[n-1].y())){
				alpha_x_mx_(n) = alpha_N;
				alpha_y_mx_(n) = alpha_y_[n];
			} else {
				alpha_y_mx_(n) = alpha_N;
				alpha_x_mx_(n) = alpha_x_[n];
			}
		}

		// Apply initial guesses
		opti_.set_initial(t_x_MX[k], t_x_init_[k]);
		opti_.set_initial(t_y_MX[k], t_y_init_[k]);
		opti_.set_initial(v_x_MX[k], waypoint_velocities_init_[n-1].x());
		opti_.set_initial(v_y_MX[k], waypoint_velocities_init_[n-1].y());

		// Deal with moving waypoints
		if (movable_waypoints_[k] && k > 0){
			offsets_MX[k] = opti_.variable(2, 1);

			// Update waypoint
			waypoints_mx_[k].SetX(waypoints_mx_[k].x() + 
								 alpha_x_mx_(k)*offsets_MX[k](0));
			waypoints_mx_[k].SetY(waypoints_mx_[k].y() + 
								  alpha_y_mx_(k)*offsets_MX[k](1));
		}
	}
	v_x_MX[n] = opti_.variable();
	v_y_MX[n] = opti_.variable();
	waypoints_mx_[n].CopyValues(waypoints_[n]);

	v_x_ = vertcat(v_x_MX);
	v_y_ = vertcat(v_y_MX);
	t_x_ = horzcat(t_x_MX);
	t_y_ = horzcat(t_y_MX);

	MX obj = 0;
	if (RELAX_INITIAL_ACCELERATION_){ obj += 1.0e3*pow(s_0, 2);}
	if (RELAX_FINAL_ACCELERATION_){ obj += 1.0e3*pow(s_N, 2);}
	
	////////////////////////////////
	/// Definiton of constraints ///
	////////////////////////////////
	double width_offset = params_.GetWidthOffset();
	double height_offset = params_.GetHeightOffset();
	Point2D<MX> curr_waypoint;
	Point2D<MX> next_waypoint;
	Corridor curr_corridor;
	MX curr_v_x = corridor_sequence_.GetStartVel().x();
	MX curr_v_y = corridor_sequence_.GetStartVel().y();
	MX off_x, off_y;
	double position_tolerance = 1.0e-5;
	for (int w = 0; w < n; w++){
		auto planning_computation_time_start = std::chrono::high_resolution_clock::now();
		curr_waypoint = waypoints_mx_[w];
		next_waypoint = waypoints_mx_[w+1];
		corridor_sequence_.GetCorridor(w, curr_corridor);

		// this assignment is needed to "cut the single shooting chain"
		curr_v_x = v_x_(w);
		curr_v_y = v_y_(w);

		// get some positions of relevance
		IntegrateOverCorridor(curr_waypoint, Point2D<MX>(curr_v_x, curr_v_y), 
							  t_x_(Slice(), w), t_y_(Slice(), w),
							  alpha_x_mx_(w), alpha_y_mx_(w),
							  alpha_x_mx_(w+1), alpha_y_mx_(w+1), params_.GetAmax());

		// add gap-closing constraints on velocity
		// if (w < n - 1){
			opti_.subject_to(v_x_(w+1) == intermediate_velocities_[2].x());
			opti_.subject_to(v_y_(w+1) == intermediate_velocities_[2].y());
		// }

		//	basic box constraints
		opti_.subject_to(0 <= (t_x_(Slice(), w) <= 100));
		opti_.subject_to(0 <= (t_y_(Slice(), w) <= 100));

		// deal with moving waypoints
		if (movable_waypoints_[w]){
			off_x = offsets_MX[w](0);
			if (max_waypoint_offsets_[w].x() > 0){
				opti_.subject_to(0 <= (off_x <= max_waypoint_offsets_[w].x()));
			} else {
				opti_.subject_to(max_waypoint_offsets_[w].x() <= (off_x <= 0));
			}

			// Check vertical moving range
			off_y = offsets_MX[w](1);
			if (max_waypoint_offsets_[w].y() > 0){
				opti_.subject_to(0 <= (off_y <= max_waypoint_offsets_[w].y()));
			} else {
				opti_.subject_to(max_waypoint_offsets_[w].y() <= (off_y <= 0));
			}
		}

		if (w == 0){
			opti_.subject_to(v_x_(w) == corridor_sequence_.GetStartVel().x());
			opti_.subject_to(v_y_(w) == corridor_sequence_.GetStartVel().y());

			// constraint to prevent initial overshooting
			// ApplyOvershootingPreventionConstraint(opti_, t_x_, t_y_);

			// Free initial acceleration
			if (std::abs(waypoints_[0].x() - waypoints_[1].x()) < 
				std::abs(waypoints_[0].y() - waypoints_[1].y())){
	
				if (RELAX_INITIAL_ACCELERATION_){
					opti_.subject_to(-1 - pow(s_0, 2) <= 
									(alpha_x_mx_(0) <= 1 + pow(s_0, 2)));
				} else {
					opti_.subject_to(alpha_x_mx_(0) == alpha_0_init_);
				}
			} else {
				if (RELAX_INITIAL_ACCELERATION_){
					opti_.subject_to(-1 - pow(s_0, 2) <= 
									(alpha_y_mx_(0) <= 1 + pow(s_0, 2)));
				} else {
					opti_.subject_to(alpha_y_mx_(0) == alpha_0_init_);
				}
			}
		}

		if (w == n-1){
			// free final acceleration
			if (std::abs(waypoints_[n].x() - waypoints_[n - 1].x()) < 
				std::abs(waypoints_[n].y() - waypoints_[n - 1].y())){
				
				if (RELAX_FINAL_ACCELERATION_){
					opti_.subject_to(-1 - pow(s_N, 2) <= 
									(alpha_x_mx_(n) <= 1 + pow(s_N, 2)));
				} else {
					opti_.subject_to(alpha_x_mx_(n) == alpha_f_init_);
				}
			} else {
				if (RELAX_FINAL_ACCELERATION_){
					opti_.subject_to(-1 - pow(s_N, 2) <= 
									(alpha_y_mx_(n) <= 
									1 + pow(s_N, 2)));
				} else {
					opti_.subject_to(alpha_y_mx_(n) == 
									alpha_f_init_);
				}
			}
		}
		
		// add gap-closing constraints on positions
		opti_.subject_to(next_waypoint.x() - position_tolerance <=
					   (intermediate_positions_[2].x() <=
						next_waypoint.x() + position_tolerance));
		opti_.subject_to(next_waypoint.y() - position_tolerance <=
					   (intermediate_positions_[2].y() <=
						next_waypoint.y() + position_tolerance));

		// constrain equal time
		opti_.subject_to(t_x_(0, w) + t_x_(1, w) + t_x_(2, w) == 
						 t_y_(0, w) + t_y_(1, w) + t_y_(2, w));

		// constrain positions and velocities
		for (int i : {0, 1}){
			// NOTE: for some unknown reason, the double-sided inequalities
			// can make the solver fail.
			opti_.subject_to(curr_corridor.Xmin() + width_offset <= intermediate_positions_[i].x());
			opti_.subject_to(intermediate_positions_[i].x() <= curr_corridor.Xmax() - width_offset);
			opti_.subject_to(curr_corridor.Ymin()  + height_offset <= intermediate_positions_[i].y());
			opti_.subject_to(intermediate_positions_[i].y() <= curr_corridor.Ymax() - height_offset);
		}

		opti_.subject_to(-params_.GetVmax() <= 
			(intermediate_velocities_[0].x() <= params_.GetVmax()));
		opti_.subject_to(-params_.GetVmax() <= 
			(intermediate_velocities_[0].y() <= params_.GetVmax()));

		// integrate the velocity over this corridor
		curr_v_x = intermediate_velocities_[2].x();
		curr_v_y = intermediate_velocities_[2].y();

		if (w < n - 1){
			opti_.subject_to(-params_.GetVmax() <= 
				(curr_v_x <= params_.GetVmax()));
			opti_.subject_to(-params_.GetVmax() <=
				(curr_v_y <= params_.GetVmax()));
		}

		// update objective term
		obj += t_x_(0, w) + t_x_(1, w) + t_x_(2, w);
	}

	// Add terminal velocity constraint
	double velocity_relaxation_tolerance = 1.0e-5;
	opti_.subject_to(-velocity_relaxation_tolerance <= 
					(curr_v_x <= velocity_relaxation_tolerance));
	opti_.subject_to(-velocity_relaxation_tolerance <=
					(curr_v_y <= velocity_relaxation_tolerance));


	//////////////////////////////////
	/// Finish problem formulation ///
	//////////////////////////////////
	opti_.minimize(obj);
	opti_.solver(solver_name, opts_casadi, opts_solver);

	//////////////////
	/// Warm-start ///
	//////////////////
	if (use_prev_sol_as_init_guess){
		opti_.set_initial(opti_.x(), prev_sol);
	}

	////////////////////////
	/// Extract solution ///
	////////////////////////
	ExtractSolutionOld();
	}

};
bool Parametrization::AddOvershootingConstraintsOld(std::set<int> &add_list){
	bool added_something = false;

	MX t_extreme;
	MX p_extreme;
	double lb;
	double ub;
	p_extremes_ = {};

	for (int w : add_list){
		// check in which direction constraints need to be added
		if (std::abs(waypoints_[w+1].x() - waypoints_[w].x()) >
				std::abs(waypoints_[w+1].y() - waypoints_[w].y())){

			// Determine bounds for vertical direction
			lb = corridor_sequence_.GetCorridor(w).Ymin() +
				 params_.GetHeightOffset();
			ub = corridor_sequence_.GetCorridor(w).Ymax() -
				 params_.GetHeightOffset();

			// check if constraint on first arc is new and needed
			t_extreme = -v_y_(w) / (alpha_y_mx_(w)*params_.GetAmax());
			if (added_constraints_list_first_arc_.count(w) == 0 && 
					double(sol_.value().value(t_extreme)) < t_y_sol_[w][0] &&
					w > 0){
				// std::cout << "Adding constraint on first arc for corridor " << w << " in y" << std::endl;
				added_constraints_list_first_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				p_extreme = waypoints_mx_[w].y() + v_y_(w)*t_extreme + 
					0.5*alpha_y_mx_(w)*params_.GetAmax()*pow(t_extreme, 2);
				opti_.subject_to(lb <= (p_extreme <= ub));
			}

			// check if constraint on second arc is new and needed
			MX next_vel = w+1 < corridor_sequence_.NbCorridors() ? v_y_(w+1) : MX(0.0);
			t_extreme = next_vel / (alpha_y_mx_(w+1)*params_.GetAmax());
			if (added_constraints_list_second_arc_.count(w) == 0 &&
					double(sol_.value().value(t_extreme)) < t_y_sol_[w][2] &&
					w+1 < corridor_sequence_.NbCorridors()){
				// std::cout << "Adding constraint on second arc for corridor " << w << " in y" << std::endl;
				added_constraints_list_second_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				p_extreme = waypoints_mx_[w+1].y() - next_vel*t_extreme + 
					0.5*alpha_y_mx_(w+1)*params_.GetAmax()*pow(t_extreme, 2);
				opti_.subject_to(lb <= (p_extreme <= ub));
			}

		} else {

			// Determine bounds for horizontal direction
			lb = corridor_sequence_.GetCorridor(w).Xmin() +
				 params_.GetWidthOffset();
			ub = corridor_sequence_.GetCorridor(w).Xmax() -
				 params_.GetWidthOffset();
			
			// check if constraint on first arc is new and needed
			t_extreme = -v_x_(w) / (alpha_x_mx_(w)*params_.GetAmax());
			if (added_constraints_list_first_arc_.count(w) == 0 && 
					double(sol_.value().value(t_extreme)) < t_x_sol_[w][0] &&
					w > 0){
				// std::cout << "Adding constraint on first arc for corridor " << w << " in x" << std::endl;
				added_constraints_list_first_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				p_extreme = waypoints_mx_[w].x() + v_x_(w)*t_extreme + 
					0.5*alpha_x_mx_(w)*params_.GetAmax()*pow(t_extreme, 2);
				opti_.subject_to(lb <= (p_extreme <= ub));
			}
			
			// check if constraint on second arc is new and needed
			MX next_vel = w+1 < corridor_sequence_.NbCorridors() ? v_x_(w+1) : MX(0.0);
			t_extreme = next_vel / (alpha_x_mx_(w+1)*params_.GetAmax());
			if (added_constraints_list_second_arc_.count(w) == 0 &&
					double(sol_.value().value(t_extreme)) < t_x_sol_[w][2] &&
					w+1 < corridor_sequence_.NbCorridors()){
				// std::cout << "Adding constraint on second arc for corridor " << w << " in x" << std::endl;
				added_constraints_list_second_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				p_extreme = waypoints_mx_[w+1].x() - next_vel*t_extreme + 
							0.5*alpha_x_mx_(w+1)*params_.GetAmax()*pow(t_extreme, 2);
				opti_.subject_to(lb <= (p_extreme <= ub));
			}
		}
	}

	if (added_something){
		ExtractSolutionOld();
	}

	return added_something;
}

bool Parametrization::AddOvershootingConstraints(std::set<int> &add_list,
		std::string& solver_name, casadi::Dict& opts_casadi,
		casadi::Dict& opts_solver, bool just_in_time_preparation_mode){
	bool added_something = false;

	double t_extreme;
	for (int w : add_list){
		// check in which direction constraints need to be added
		if (std::abs(waypoints_[w+1].x() - waypoints_[w].x()) >
				std::abs(waypoints_[w+1].y() - waypoints_[w].y())){
			// check if constraint on first arc is new and needed
			t_extreme = -waypoint_velocities_sol_[w].y() / 
						(alpha_y_sol_[w]*params_.GetAmax());
			if (added_constraints_list_first_arc_.count(w) == 0 && 
					t_extreme < t_y_sol_[w][0] && w > 0){
				// std::cout << "Adding constraint on first arc for corridor " << w << " in y" << std::endl;
				// std::cout << "t_extreme: " << t_extreme << std::endl;
				added_constraints_list_first_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				active_opti_inputs_["parabolic_slacks"](2, w) = 0;
			}

			// check if constraint on second arc is new and needed
			t_extreme = waypoint_velocities_sol_[w+1].y() / (alpha_y_sol_[w+1]*params_.GetAmax());
			if (added_constraints_list_second_arc_.count(w) == 0 &&
					t_extreme < t_y_sol_[w][2] &&
					w+1 < corridor_sequence_.NbCorridors()){
				// std::cout << "Adding constraint on second arc for corridor " << w << " in y" << std::endl;
				added_constraints_list_second_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				active_opti_inputs_["parabolic_slacks"](3, w) = 0;
			}

		} else {			
			// check if constraint on first arc is new and needed
			t_extreme = -waypoint_velocities_sol_[w].x() / 
						(alpha_x_sol_[w]*params_.GetAmax());
			if (added_constraints_list_first_arc_.count(w) == 0 && 
					t_extreme < t_x_sol_[w][0] && w > 0){
				// std::cout << "Adding constraint on first arc for corridor " << w << " in x" << std::endl;
				added_constraints_list_first_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				active_opti_inputs_["parabolic_slacks"](0, w) = 0;
			}
			
			// check if constraint on second arc is new and needed
			t_extreme = waypoint_velocities_sol_[w+1].x() / (alpha_x_sol_[w+1]*params_.GetAmax());
			if (added_constraints_list_second_arc_.count(w) == 0 &&
					t_extreme < t_x_sol_[w][2] &&
					w+1 < corridor_sequence_.NbCorridors()){
				// std::cout << "Adding constraint on second arc for corridor " << w << " in x" << std::endl;
				added_constraints_list_second_arc_.insert(w);
				added_something = true;

				// If so, add the constraint
				active_opti_inputs_["parabolic_slacks"](1, w) = 0;
			}
		}
	}

	if (added_something){
		Solve(UpdateToken(), solver_name, opts_casadi, opts_solver, 
			  just_in_time_preparation_mode, true);
		// ExtractSolutionOld();
	}

	return added_something;
}


void Parametrization::OptimizeSingleArc(const UpdateToken&){
	waypoints_[0].CopyValues(corridor_sequence_.GetStart());
	waypoints_[1].CopyValues(corridor_sequence_.GetDest());
	waypoints_sol_[0].CopyValues(corridor_sequence_.GetStart());
	waypoints_sol_[1].CopyValues(corridor_sequence_.GetDest());
	waypoint_velocities_sol_[0].CopyValues(corridor_sequence_.GetStartVel());

	OptimizeSingleArc1D(t_x_sol_[0], alpha_x_sol_, 
						corridor_sequence_.GetStart().x(),
						corridor_sequence_.GetDest().x(),
						corridor_sequence_.GetStartVel().x());
	OptimizeSingleArc1D(t_y_sol_[0], alpha_y_sol_,
						corridor_sequence_.GetStart().y(),
						corridor_sequence_.GetDest().y(),
						corridor_sequence_.GetStartVel().y());

	// empty the solution of the subsequent corridors
	for (int i = 1; i < corridor_sequence_.NbCorridors()+1; i++){
		for (int j = 0; j < 3; j++){
			t_x_sol_[i][j] = 0.0;
			t_y_sol_[i][j] = 0.0;
		}
		// waypoints_[i].CopyValues(corridor_sequence_.GetDest());
		waypoints_sol_[i].CopyValues(corridor_sequence_.GetDest());
		waypoint_velocities_sol_[i].CopyValues(Point2D<double>(0.0, 0.0));
	}
	// waypoints_[corridor_sequence_.NbCorridors()].CopyValues(
	// 	corridor_sequence_.GetDest());
	waypoints_sol_[corridor_sequence_.NbCorridors()].CopyValues(
		corridor_sequence_.GetDest());
	waypoint_velocities_sol_[corridor_sequence_.NbCorridors()].CopyValues(
		Point2D<double>(0.0, 0.0));
}

bool Parametrization::FlipAccelerationAtWaypoint(const UpdateToken&,
												 int waypoint_idx,
												 bool x_flip,
												 bool y_flip){
	if (waypoint_idx < 1 || waypoint_idx >= corridor_sequence_.NbCorridors()){
		throw std::runtime_error("Invalid waypoint index for flipping the acceleration.");
	}

	bool made_modification = false;

	// If this one has been flipped already, ignore this request
	if (x_flip && !flipped_acceleration_x_[waypoint_idx]){
		// flip the acceleration
		alpha_x_[waypoint_idx] = -alpha_x_[waypoint_idx];
		if (active_opti_code_ != ""){
		active_opti_inputs_["alpha"](0, waypoint_idx) = alpha_x_[waypoint_idx];
		}

		// flip the movable distances
		if (movable_waypoints_[waypoint_idx]){
			max_waypoint_offsets_[waypoint_idx].SetX(-max_waypoint_offsets_[waypoint_idx].x());
			if (active_opti_code_ != ""){
			int count = 0;
			for (int i = 0; i < waypoint_idx; i++){ if (movable_waypoints_[i]){ count++;}}
			if (max_waypoint_offsets_[waypoint_idx].x() > 0){
				active_opti_inputs_["movable_distances"](0, count) = 0;
				active_opti_inputs_["movable_distances"](1, count) = max_waypoint_offsets_[waypoint_idx].x();
			} else {
				active_opti_inputs_["movable_distances"](0, count) = max_waypoint_offsets_[waypoint_idx].x();
				active_opti_inputs_["movable_distances"](1, count) = 0;
			}
			}
		}

		flipped_acceleration_x_[waypoint_idx] = true;
		made_modification = true;
	}

	// If this one has been flipped already, ignore this request
	if (y_flip && !flipped_acceleration_y_[waypoint_idx]){
		// flip the acceleration
		alpha_y_[waypoint_idx] = -alpha_y_[waypoint_idx];
		if (active_opti_code_ != ""){
		active_opti_inputs_["alpha"](1, waypoint_idx) = alpha_y_[waypoint_idx];
		}

		// flip the movable distances
		if (movable_waypoints_[waypoint_idx]){
			max_waypoint_offsets_[waypoint_idx].SetY(-max_waypoint_offsets_[waypoint_idx].y());
			if (active_opti_code_ != ""){
			int count = 0;
			for (int i = 0; i < waypoint_idx; i++){ if (movable_waypoints_[i]){ count++;}}
			if (max_waypoint_offsets_[waypoint_idx].y() > 0){
				active_opti_inputs_["movable_distances"](2, count) = 0, 
				active_opti_inputs_["movable_distances"](3, count) = max_waypoint_offsets_[waypoint_idx].y();
			} else {
				active_opti_inputs_["movable_distances"](2, count) = max_waypoint_offsets_[waypoint_idx].y();
				active_opti_inputs_["movable_distances"](3, count) = 0;
			}
			}
		}

		flipped_acceleration_y_[waypoint_idx] = true;
		made_modification = true;
	}

	// Reset the add constraint lists since they won't be valid anymore
	if (made_modification){
		added_constraints_list_first_arc_.clear();
		added_constraints_list_second_arc_.clear();
	}

	return made_modification;
}

void Parametrization::FilterAddConstraintsList(const UpdateToken&, 
										std::set<int> &add_list) const {
	for (int w : added_constraints_list_first_arc_){
		if (added_constraints_list_second_arc_.count(w) > 0){
			add_list.erase(w);
		}
	}
}

Point2D<double> Parametrization::GetWaypoint(int idx) const {
	return waypoints_[idx].Copy();
};

Point2D<double> Parametrization::GetWaypointOffset(int idx) const {
            return max_waypoint_offsets_[idx].Copy();
};

WaypointLocation Parametrization::GetWaypointLocation(int idx) const {
            return waypoint_locations_[idx];
};

bool Parametrization::IsWaypointMovable(int idx) const { 
            return movable_waypoints_[idx];
};

const std::vector<Point2D<double>>& Parametrization::GetWaypointsSol() const {
	return waypoints_sol_;
};

std::vector<Point2D<double>>& Parametrization::GetWaypointVelocitiesSol(){
	return waypoint_velocities_sol_;
};

std::ostream& operator<<(std::ostream &out, 
						 Parametrization const &parametrization){
	Point2D<double> waypoint;
	Point2D<double> waypoint_offset;
	WaypointLocation waypoint_location;

	out << "Parametrization (" << parametrization.NbCorridors() << "/" 
		<< parametrization.MaxNbCorridors() << ")" << std::endl;

	for (int i = 0; i < parametrization.NbCorridors() + 1; i++){
		waypoint = parametrization.GetWaypoint(i);
		waypoint_location = parametrization.GetWaypointLocation(i);
		out << "Waypoint " << i << std::endl;
		out << "\tPosition: \t" << waypoint << "\t (" << waypoint_location << ")" << std::endl;
		out << "\tAcceleration: \t(" << parametrization.GetAlphaX(i) << ", " << 
			parametrization.GetAlphaY(i) << ")" << std::endl;
		if (parametrization.IsWaypointMovable(i)){
			waypoint_offset = parametrization.GetWaypointOffset(i);
			out << "\tMovable distance: \t(" << waypoint_offset.x() << ", "
				<<  waypoint_offset.y()<< ")" << std::endl;
		} else {
			out << "\t(not movable)" << std::endl;
		}
	}

	return out;
};

json Parametrization::ToJson() const {
	json j;

	int curr_nb_corridors = std::max(max_nb_corridors_, 
									 corridor_sequence_.NbCorridors());

	std::vector<json> waypoints_json = 					std::vector<json>(curr_nb_corridors + 1);
	std::vector<json> max_waypoint_offsets_json = 		std::vector<json>(curr_nb_corridors + 1);
	std::vector<json> waypoints_sol_json = 				std::vector<json>(curr_nb_corridors + 1);
	std::vector<json> waypoint_velocities_sol_json = 	std::vector<json>(curr_nb_corridors + 1);

	for (int i = 0; i < curr_nb_corridors + 1; i++){
		waypoints_json[i] = waypoints_[i].ToJson();
		max_waypoint_offsets_json[i] = max_waypoint_offsets_[i].ToJson();
		waypoints_sol_json[i] = waypoints_sol_[i].ToJson();
		waypoint_velocities_sol_json[i] = waypoint_velocities_sol_[i].ToJson();
	}

	std::vector<std::vector<double>> t_x_sol_json = 
		std::vector<std::vector<double>>(curr_nb_corridors);
	std::vector<std::vector<double>> t_y_sol_json =
		std::vector<std::vector<double>>(curr_nb_corridors);
	for (int i = 0; i < curr_nb_corridors; i++){
		t_x_sol_json[i] = t_x_sol_[i];
		t_y_sol_json[i] = t_y_sol_[i];
	}

	j["nb_corridors"] = corridor_sequence_.NbCorridors();
	j["alpha_x"] = std::vector<double>(alpha_x_.begin(), alpha_x_.begin() + 
									   curr_nb_corridors + 1);
	j["alpha_y"] = std::vector<double>(alpha_y_.begin(), alpha_y_.begin() + 
									   curr_nb_corridors + 1);
	j["waypoints"] = waypoints_json;
	j["movable_waypoints"] = std::vector<double>(movable_waypoints_.begin(),
												  movable_waypoints_.begin() + 
												  curr_nb_corridors + 1);
	j["max_waypoint_offsets"] = max_waypoint_offsets_json;

	j["waypoints_sol"] = waypoints_sol_json;
	j["waypoint_velocities_sol"] = waypoint_velocities_sol_json;
	j["alpha_x_sol"] = std::vector<double>(alpha_x_sol_.begin(), 
										   alpha_x_sol_.begin() + 
										   curr_nb_corridors + 1);
	j["alpha_y_sol"] = std::vector<double>(alpha_y_sol_.begin(), 
										   alpha_y_sol_.begin() + 
										   curr_nb_corridors + 1);
	j["t_x_sol"] = t_x_sol_json;
	j["t_y_sol"] = t_y_sol_json;


	return j;
};


void Parametrization::PrepareSingleOptiInstance(int nbCorridors,
										  		std::string& solver_name_,
										  		Dict& opts_casadi,
										  		Dict& opts_solver,
												std::string movable_waypoints_code){		
	// reset mx containers
	alpha_x_mx_ = MX(max_nb_corridors_ + 1, 1);
	alpha_y_mx_ = MX(max_nb_corridors_ + 1, 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

	// check code structure
	if (movable_waypoints_code.size() != nbCorridors + 1 || 
		movable_waypoints_code[0] != '0' ||
		movable_waypoints_code[nbCorridors] != '0'){
		throw std::runtime_error("Invalid movable_waypoints_code.");
	};
	for (int i = 0; i < nbCorridors; i++){
		if (movable_waypoints_code[i] != '0' && movable_waypoints_code[i] != '1'){
			throw std::runtime_error("Invalid element in movable_waypoints_code.");
		}
	}

	int nb_movable_waypoints = std::count(movable_waypoints_code.begin(), 
										  movable_waypoints_code.end(), '1');

	// start the optimization
	opti_ = Opti();
	int n = nbCorridors;

	////////////////////////////////
	/// Definition of parameters ///
	////////////////////////////////
	// vmax, amax, widht, height, margin
	MX vmax_p = opti_.parameter();
	MX amax_p = opti_.parameter();
	MX veh_width_p = opti_.parameter();
	MX veh_height_p = opti_.parameter();
	MX margin_p = opti_.parameter();

	// corridors
	MX corridor_p = opti_.parameter(4, n); // [xmin, xmax, ymin, ymax]

	// waypoints
	MX waypoints_p = opti_.parameter(2, n+1);

	// alpha_values
	MX alpha_p = opti_.parameter(2, n+1);
	MX init_bottleneck = opti_.parameter(); 	// 0 for x, 1 for y
	MX final_bottleneck = opti_.parameter();	// 0 for x, 1 for y

	// start_vel
	MX start_vel_p = opti_.parameter(2, 1);

	// movable waypoints: moving limits
	MX movable_distances_p;
	movable_distances_p = opti_.parameter(4, nb_movable_waypoints); 	// [x_lb, x_ub, y_lb, y_ub]

	// bounds on parabolic extremes
	MX parabolic_slacks_p = opti_.parameter(4, n); 		// [x_first, x_last, y_first, y_last]
	// std::cout << "\t\tdefined parameters" << std::endl;

	// bounds on duration of the initial acceleration
	MX t_init_limits = opti_.parameter(2, 1);

	////////////////////////////////////////////
	/// Definition of optimization variables ///
	////////////////////////////////////////////
	std::vector<MX> v_x_MX(n+1);
	std::vector<MX> v_y_MX(n+1);
	std::vector<MX> t_x_MX(n);
	std::vector<MX> t_y_MX(n);
	std::vector<MX> offsets_MX;
	offsets_MX = std::vector<MX>(nb_movable_waypoints);
	int offset_ptr = 0;
	std::map<int, int> offset_map; // waypoint_index to offset_index

	MX s_0, alpha_0, s_N, alpha_N;
	for (int k = 0; k < n; k++){
		// Update MX objects
		alpha_x_mx_(k) = alpha_p(0, k); 
		alpha_y_mx_(k) = alpha_p(1, k);
		waypoints_mx_[k].SetX(waypoints_p(0, k));
		waypoints_mx_[k].SetY(waypoints_p(1, k));

		// Create variables
		v_x_MX[k] = opti_.variable();
		v_y_MX[k] = opti_.variable();
		if (k > 0 && movable_waypoints_code[k] == '1') {
			offsets_MX[offset_ptr] = opti_.variable(2, 1);	
		}
		t_x_MX[k] = opti_.variable(3, 1);
		t_y_MX[k] = opti_.variable(3, 1);
		if (k == 0 && RELAX_INITIAL_ACCELERATION_){
			s_0 = opti_.variable();
			alpha_0 = opti_.variable();
			alpha_x_mx_(0) = (1-init_bottleneck)*alpha_0 + init_bottleneck*alpha_p(0, 0);
			alpha_y_mx_(0) = init_bottleneck*alpha_0 + (1-init_bottleneck)*alpha_p(1, 0);
		}
		if (k == n - 1 && RELAX_FINAL_ACCELERATION_){
			s_N = opti_.variable();
			alpha_N = opti_.variable();
			alpha_x_mx_(n) = (1-final_bottleneck)*alpha_N + final_bottleneck*alpha_p(0, n);
			alpha_y_mx_(n) = final_bottleneck*alpha_N + (1-final_bottleneck)*alpha_p(1, n);
		}

		if (k > 0 && movable_waypoints_code[k] == '1'){
			// Update waypoint
			waypoints_mx_[k].SetX(waypoints_mx_[k].x() + 
									alpha_x_mx_(k)*offsets_MX[offset_ptr](0));
			waypoints_mx_[k].SetY(waypoints_mx_[k].y() + 
									alpha_y_mx_(k)*offsets_MX[offset_ptr](1));
			offset_map[k] = offset_ptr;
			offset_ptr++;
		}
	}
	v_x_MX[n] = opti_.variable();
	v_y_MX[n] = opti_.variable();
	waypoints_mx_[n].SetX(waypoints_p(0, n));
	waypoints_mx_[n].SetY(waypoints_p(1, n));

	v_x_ = vertcat(v_x_MX);
	v_y_ = vertcat(v_y_MX);
	t_x_ = horzcat(t_x_MX);
	t_y_ = horzcat(t_y_MX);
	MX offsets = horzcat(offsets_MX);

	MX obj = 0;
	if (RELAX_INITIAL_ACCELERATION_){ obj += 1.0e3*pow(s_0, 2);}
	if (RELAX_FINAL_ACCELERATION_){ obj += 1.0e3*pow(s_N, 2);}
	// std::cout << "\t\tdefined variables" << std::endl;
	
	////////////////////////////////
	/// Definiton of constraints ///
	////////////////////////////////
	MX width_offset = veh_width_p/2 + margin_p;
	MX height_offset = veh_height_p/2 + margin_p;
	Point2D<MX> curr_waypoint;
	Point2D<MX> next_waypoint;
	MX curr_v_x, curr_v_y, off_x, off_y, t_extreme, p_extreme, lb, ub;
	Corridor_MX curr_corridor, prev_corridor;
	double position_tolerance = 1.0e-5;
	MX temp_mx(6, n);
	for (int w = 0; w < n; w++){
		// std::cout << "\t\tw = " << w << std::endl;
		auto planning_computation_time_start = std::chrono::high_resolution_clock::now();
		curr_waypoint = waypoints_mx_[w];
		next_waypoint = waypoints_mx_[w+1];
		curr_corridor.SetXmin(corridor_p(0, w));
		curr_corridor.SetXmax(corridor_p(1, w));
		curr_corridor.SetYmin(corridor_p(2, w));
		curr_corridor.SetYmax(corridor_p(3, w));
		if (w > 0){
			prev_corridor.SetXmin(corridor_p(0, w-1));
			prev_corridor.SetXmax(corridor_p(1, w-1));
			prev_corridor.SetYmin(corridor_p(2, w-1));
			prev_corridor.SetYmax(corridor_p(3, w-1));
		}

		// this assignment is needed to "cut the single shooting chain"
		curr_v_x = v_x_(w);
		curr_v_y = v_y_(w);

		// get some positions of relevance
		IntegrateOverCorridor(curr_waypoint, Point2D<MX>(curr_v_x, curr_v_y), 
							  t_x_(Slice(), w), t_y_(Slice(), w),
							  alpha_x_mx_(w), alpha_y_mx_(w),
							  alpha_x_mx_(w+1), alpha_y_mx_(w+1), amax_p);

		// add gap-closing constraints on velocity
		opti_.subject_to(v_x_(w+1) == intermediate_velocities_[2].x());
		opti_.subject_to(v_y_(w+1) == intermediate_velocities_[2].y());

		// add gap-closing constraints on positions
		if (w == n-1){
			// opti_.subject_to(next_waypoint.x() == intermediate_positions_[2].x());
			// opti_.subject_to(next_waypoint.y() == intermediate_positions_[2].y());
			opti_.subject_to(next_waypoint.x() - position_tolerance <= (intermediate_positions_[2].x() <= next_waypoint.x() + position_tolerance));
			opti_.subject_to(next_waypoint.y() - position_tolerance <= (intermediate_positions_[2].y() <= next_waypoint.y() + position_tolerance));
		} else {
			if (movable_waypoints_code[w+1] == '1'){
				opti_.subject_to(offsets_MX[offset_map[w+1]](0) == (intermediate_positions_[2].x() - waypoints_p(0, w+1))/alpha_p(0, w+1));
				opti_.subject_to(offsets_MX[offset_map[w+1]](1) == (intermediate_positions_[2].y() - waypoints_p(1, w+1))/alpha_p(1, w+1));
			} else {
				// opti_.subject_to(intermediate_positions_[2].x() == waypoints_p(0, w+1));
				// opti_.subject_to(intermediate_positions_[2].y() == waypoints_p(1, w+1));
				opti_.subject_to(waypoints_p(0,w+1) - position_tolerance <= (intermediate_positions_[2].x() <= waypoints_p(0,w+1) + position_tolerance));
				opti_.subject_to(waypoints_p(1,w+1) - position_tolerance <= (intermediate_positions_[2].y() <= waypoints_p(1,w+1) + position_tolerance));
			}
		}

		//	basic box constraints
		opti_.subject_to(0 <= (t_x_(Slice(), w) <= 100));
		opti_.subject_to(0 <= (t_y_(Slice(), w) <= 100));

		// initial overshooting prevention constraint
		if (w == 0){
			opti_.subject_to(t_x_(0,0) <= t_init_limits(0));
			opti_.subject_to(t_y_(0,0) <= t_init_limits(1));
		}

		// deal with moving waypoints
		if (movable_waypoints_code[w] == '1'){
			opti_.subject_to(movable_distances_p(0, offset_map[w]) <= offsets_MX[offset_map[w]](0));
			opti_.subject_to(offsets_MX[offset_map[w]](0) <= movable_distances_p(1, offset_map[w]));
			opti_.subject_to(movable_distances_p(2, offset_map[w]) <= offsets_MX[offset_map[w]](1));
			opti_.subject_to(offsets_MX[offset_map[w]](1) <= movable_distances_p(3, offset_map[w]));
		}

		// first corridor: special cases
		if (w == 0){
			opti_.subject_to(v_x_(w) == start_vel_p(0));
			opti_.subject_to(v_y_(w) == start_vel_p(1));

			// constraint to prevent initial overshooting
			// TODO: how do we deal with this?
			// ApplyOvershootingPreventionConstraint(opti_, t_x_, t_y_);

			// Free initial acceleration
			opti_.subject_to(-1 - pow(s_0, 2) <= (alpha_0 <= 1 + pow(s_0, 2)));
		}

		// last corridor: special cases
		if (w == n-1){
			// free final acceleration
			opti_.subject_to(-1 - pow(s_N, 2) <= (alpha_N <= 1 + pow(s_N, 2)));
		}

		// constrain equal time
		opti_.subject_to(t_x_(0, w) + t_x_(1, w) + t_x_(2, w) == 
						 t_y_(0, w) + t_y_(1, w) + t_y_(2, w));

		// constrain positions
		for (int i : {0, 1}){
			// NOTE: for some unknown reason, the double-sided inequalities
			// can make the solver fail.
			opti_.subject_to(curr_corridor.Xmin() + width_offset <= intermediate_positions_[i].x());
			opti_.subject_to(intermediate_positions_[i].x() <= curr_corridor.Xmax() - width_offset);
			opti_.subject_to(curr_corridor.Ymin()  + height_offset <= intermediate_positions_[i].y());
			opti_.subject_to(intermediate_positions_[i].y() <= curr_corridor.Ymax() - height_offset);
		}

		// add parabolic overshooting prevention constraints
		// x - direction
		// 		first arc
		if (w > 0){
		lb = curr_corridor.Xmin() + width_offset;
		ub = curr_corridor.Xmax() - width_offset;
		t_extreme = -v_x_(w) / (alpha_x_mx_(w)*amax_p);
		p_extreme = waypoints_mx_[w].x() + v_x_(w)*t_extreme + 
					0.5*alpha_x_mx_(w)*amax_p*pow(t_extreme, 2);
		opti_.subject_to(lb - parabolic_slacks_p(0, w) <= (p_extreme <= ub + parabolic_slacks_p(0, w)));
		}
		// 		second arc of previous (!) corridor
		if (w > 0){
		lb = prev_corridor.Xmin() + width_offset;
		ub = prev_corridor.Xmax() - width_offset;
		t_extreme = v_x_(w) / (alpha_x_mx_(w)*amax_p);
		p_extreme = waypoints_mx_[w].x() - v_x_(w)*t_extreme + 
					0.5*alpha_x_mx_(w)*amax_p*pow(t_extreme, 2);
		opti_.subject_to(lb - parabolic_slacks_p(1, w-1) <= (p_extreme <= ub + parabolic_slacks_p(1, w-1)));
		}

		// y - direction
		// 		first arc
		if (w > 0){
		lb = curr_corridor.Ymin() + height_offset;
		ub = curr_corridor.Ymax() - height_offset;
		t_extreme = -v_y_(w) / (alpha_y_mx_(w)*amax_p);
		p_extreme = waypoints_mx_[w].y() + v_y_(w)*t_extreme + 
					0.5*alpha_y_mx_(w)*amax_p*pow(t_extreme, 2);
		opti_.subject_to(lb - parabolic_slacks_p(2, w) <= p_extreme);
		opti_.subject_to(p_extreme <= ub + parabolic_slacks_p(2, w));
		}
		// 		second arc of previous (!) corridor
		if (w > 0){
		lb = prev_corridor.Ymin() + height_offset;
		ub = prev_corridor.Ymax() - height_offset;
		t_extreme = v_y_(w) / (alpha_y_mx_(w)*amax_p);
		p_extreme = waypoints_mx_[w].y() - v_y_(w)*t_extreme + 
					0.5*alpha_y_mx_(w)*amax_p*pow(t_extreme, 2);
		opti_.subject_to(lb - parabolic_slacks_p(3, w-1) <= p_extreme);
		opti_.subject_to(p_extreme <= ub + parabolic_slacks_p(3, w-1));
		}

		// constrain velocities
		opti_.subject_to(-vmax_p <= (intermediate_velocities_[0].x() <= vmax_p));
		opti_.subject_to(-vmax_p <= (intermediate_velocities_[0].y() <= vmax_p));

		// integrate the velocity over this corridor
		curr_v_x = intermediate_velocities_[2].x();
		curr_v_y = intermediate_velocities_[2].y();

		if (w < n - 1){
			opti_.subject_to(-vmax_p <= (curr_v_x <= vmax_p));
			opti_.subject_to(-vmax_p <= (curr_v_y <= vmax_p));
		}

		// update objective term
		obj += t_x_(0, w) + t_x_(1, w) + t_x_(2, w);
	}

	// Add terminal velocity constraint
	double velocity_relaxation_tolerance = 1.0e-5;
	opti_.subject_to(-velocity_relaxation_tolerance <= 
					(curr_v_x <= velocity_relaxation_tolerance));
	opti_.subject_to(-velocity_relaxation_tolerance <=
					(curr_v_y <= velocity_relaxation_tolerance));
	


	//////////////////////////////////
	/// Finish problem formulation ///
	//////////////////////////////////
	opti_.minimize(obj);
	opti_.solver(solver_name_, opts_casadi, opts_solver);

	// Create function object
	Dict opts;
	opts["error_on_fail"] = true;
	std::string name = "opti_ARENA_" + std::to_string(n) + "_" + movable_waypoints_code;

	std::vector<MX> inputs_to_function = {
		opti_.x(), vmax_p, amax_p, veh_width_p, veh_height_p, margin_p,
		waypoints_p, alpha_p, init_bottleneck, final_bottleneck, start_vel_p,
		corridor_p, parabolic_slacks_p, movable_distances_p, t_init_limits};
	std::vector<std::string> input_names_to_function = {
		"x_init", "v_max", "a_max", "veh_width", "veh_height", "margin", 
		"waypoints", "alpha", "initial_bottleneck", "final_bottleneck", 
		"start_vel", "corridors", "parabolic_slacks", "movable_distances",
		"t_init_limits"};
	std::vector<MX> outputs_to_function = {
		t_x_, t_y_, v_x_, v_y_, alpha_x_mx_, alpha_y_mx_, opti_.g(),
		opti_.x(), temp_mx, offsets};
	std::vector<std::string> output_names_to_function = {
		"t_x", "t_y", "v_x", "v_y", "alpha_x", "alpha_y", "constraints", 
		"opti_x", "temp_mx", "offsets"};

	Function opti_f = opti_.to_function(name, inputs_to_function, 
		outputs_to_function, input_names_to_function,
		output_names_to_function, opts);

	prepared_opti_instances_[n][movable_waypoints_code] = opti_f;

	std::map<std::string, DM> inputs;
	inputs["x_init"] = DM(2*(n+1) + 6*n + 4 + 2*nb_movable_waypoints, 1);
	inputs["vmax"] = DM(params_.GetVmax());
	inputs["amax"] = DM(params_.GetAmax());
	inputs["veh_width"] = DM(params_.GetVehWidth());
	inputs["veh_height"] = DM(params_.GetVehHeight());
	inputs["margin"] = DM(params_.GetMargin());
	inputs["waypoints"] = DM(2, n+1);
	inputs["alpha"] = DM(2, n+1);
	inputs["initial_bottleneck"] = DM(0);
	inputs["final_bottleneck"] = DM(0);
	inputs["start_vel"] = DM(2, 1);
	inputs["corridors"] = DM(4, n);
	inputs["parabolic_slacks"] = DM(4, n);
	inputs["movable_distances"] = DM(4, nb_movable_waypoints);
	inputs["t_init_limits"] = DM(2, 1);
	opti_inputs_[n][movable_waypoints_code] = inputs;
};


void Parametrization::ComputeSingleWaypoint(int waypoint_idx, 
											bool second_sweep){
	if (waypoint_idx < 1 || waypoint_idx >= corridor_sequence_.NbCorridors()){
		throw std::out_of_range("Invalid waypoint index");
	}

	// Retrieve the relevant corridors
	corridor_sequence_.GetCorridor(waypoint_idx-1, curr_corridor_);
	corridor_sequence_.GetCorridor(waypoint_idx, next_corridor_);
	curr_corridor_.GetOverlap(next_corridor_, overlap_);

	// Get the candidate waypoints
	ComputeCandidateWaypoints();

	// select the best candidate waypoint and update the default acceleration
	ApplyHeuristic(waypoint_idx, second_sweep);
};

void Parametrization::ComputeCandidateWaypoints(){
	double width_offset = params_.GetWidthOffset();
	double height_offset = params_.GetHeightOffset();

	// Get the four corners points as potential candidates
	// bottom left  -  bottom right  -  top right  -  top left
	candidate_waypoints_[0] = Point2D<double>(overlap_.Xmin() + width_offset,
											  overlap_.Ymin() + height_offset);
	candidate_waypoints_[1] = Point2D<double>(overlap_.Xmax() - width_offset,
											  overlap_.Ymin() + height_offset);
	candidate_waypoints_[2] = Point2D<double>(overlap_.Xmax() - width_offset,
											  overlap_.Ymax() - height_offset);
	candidate_waypoints_[3] = Point2D<double>(overlap_.Xmin() + width_offset,
											  overlap_.Ymax() - height_offset);
	
	// so far, all candidates are valid
	candidate_valid_[0] = true;
	candidate_valid_[1] = true;
	candidate_valid_[2] = true;
	candidate_valid_[3] = true;

	// based on the location of the waypoint within the overlap, compute the
	// default acceleration vector
	candidate_alpha_x_ = {-1, 1, 1, -1};
	candidate_alpha_y_ = {-1, -1, 1, 1};

	// remove candidates that are not at an edge of a corridor
	double tolerance = 1.0e-4;
	double distance_to_current;
	double distance_to_next;
	for (int i = 0; i < candidate_waypoints_.size(); i++){
		if (candidate_valid_[i]){
			distance_to_current = std::min(
				std::min( // horizontal distance to current corridor
					std::abs(candidate_waypoints_[i].x() - 
							 curr_corridor_.Xmin() - width_offset),
					std::abs(curr_corridor_.Xmax() - 
							 candidate_waypoints_[i].x() - width_offset)),
				std::min( // vertical distance to current corridor
					std::abs(candidate_waypoints_[i].y() - 
							 curr_corridor_.Ymin() - height_offset),
					std::abs(curr_corridor_.Ymax() - 
							 candidate_waypoints_[i].y() - height_offset)));
			distance_to_next = std::min(
				std::min( // horizontal distance to next corridor
					std::abs(candidate_waypoints_[i].x() - 
							 next_corridor_.Xmin() - width_offset),
					std::abs(next_corridor_.Xmax() - 
							 candidate_waypoints_[i].x() - width_offset)),
				std::min( // vertical distance to next corridor
					std::abs(candidate_waypoints_[i].y() - 
							 next_corridor_.Ymin() - height_offset),
					std::abs(next_corridor_.Ymax() - 
							 candidate_waypoints_[i].y() - height_offset)));
			if (distance_to_current > tolerance ||
				distance_to_next > tolerance){
				candidate_valid_[i] = false;
			}
		}
	}
}

void Parametrization::ApplyHeuristic(int waypoint_idx, 
									 bool second_sweep){
	// Find the next point to aim at
	int nb_corridors = corridor_sequence_.NbCorridors();
	if (second_sweep){
		// if it's the second sweep, aim at the destination
		next_point_ = waypoints_[waypoint_idx + 1];
	} else if (waypoint_idx == nb_corridors - 1){
		// if you're at the end, aim at the destination
		next_point_ = corridor_sequence_.GetDest();
	} else {
		// aim at the center of the next overlap
		corridor_sequence_.GetCorridor(waypoint_idx + 1).GetOverlap(next_corridor_, next_overlap_);
		next_overlap_.GetCenter(next_point_);
	}

	// Compute the previous point
	prev_point_ = waypoints_[waypoint_idx - 1];

	// Compute line of sight between previous and next point
	Point2D<double> line_of_sight = next_point_ - prev_point_;
	double angle = std::atan2(line_of_sight.y(), line_of_sight.x());

	// rotate center of the current overlap around angle
	Point2D<double> transformed_center = overlap_.GetCenter();
	transformed_center -= prev_point_;
	transformed_center.Rotate(-angle);

	// rotate line of sight 90 degrees
	double temp;
	if (transformed_center.y() < 0.0){ // left turn
		line_of_sight.Rotate(3.1415926535 /2);
	} else { // right turn
		line_of_sight.Rotate(-3.1415926535/2);
	}

	double norm = std::sqrt(line_of_sight.x()*line_of_sight.x() + 
							line_of_sight.y()*line_of_sight.y());
	double length = 20.0;
	Point2D<double> overlap_center = overlap_.GetCenter();
	line_of_sight.SetX(overlap_center.x() + length*line_of_sight.x()/norm);
	line_of_sight.SetY(overlap_center.y() + length*line_of_sight.y()/norm);

	int best_candidate_idx = 0;
	double best_distance = 1.0e10;
	for (int i = 0; i < candidate_waypoints_.size(); i++){
		if (candidate_valid_[i]){
			if (line_of_sight.Distance(candidate_waypoints_[i]) < best_distance){
				best_candidate_idx = i;
				best_distance = line_of_sight.Distance(candidate_waypoints_[i]);
			}
		}
	}

	// Store the best candidate waypoint
	waypoints_[waypoint_idx].CopyValues(candidate_waypoints_[best_candidate_idx]);
	waypoint_locations_[waypoint_idx] = WaypointLocation(best_candidate_idx);

	// Based on the best candidate waypoint, compute the acceleration
	alpha_x_[waypoint_idx] = candidate_alpha_x_[best_candidate_idx];
	alpha_y_[waypoint_idx] = candidate_alpha_y_[best_candidate_idx];
	
}

void Parametrization::UpdateParametrizationWithLineOfSight(int waypoint_idx){
	// check inputs
	if (waypoint_idx < 1 || 
			waypoint_idx >= corridor_sequence_.NbCorridors()){
		throw std::out_of_range("Invalid index of corridor to update");
	}
	
	// get the previous and next waypoints
	prev_point_ = waypoints_[waypoint_idx - 1];
	curr_point_ = waypoints_[waypoint_idx];
	next_point_ = waypoints_[waypoint_idx + 1];

	// get the correct corridors
	corridor_sequence_.GetCorridor(waypoint_idx - 1, curr_corridor_);
	corridor_sequence_.GetCorridor(waypoint_idx, next_corridor_);

	if (LineOfSightInCorridors(prev_point_, next_point_, curr_corridor_, 
							   next_corridor_)){
		// compute the distance to the line of sight
		double distance = curr_point_.DistanceToLine(prev_point_, next_point_);

		movable_waypoints_[waypoint_idx] = true;
		curr_corridor_.GetOverlap(next_corridor_, overlap_);

		if (distance > 2.0e-3){
			// flip the acceleration vector
			alpha_x_[waypoint_idx] = -alpha_x_[waypoint_idx];
			alpha_y_[waypoint_idx] = -alpha_y_[waypoint_idx];

			// allow the waypoint to move up until the acceleration vector
			// (not further since that would lead to an incorrect acceleration
			// vector)
			
			// compute the movable distances
			if (alpha_x_[waypoint_idx] > 0){
				max_waypoint_offsets_[waypoint_idx].SetX(overlap_.Xmax() - 
					params_.GetVehWidth()/2 - params_.GetMargin() - 
					curr_point_.x());
			} else {
				max_waypoint_offsets_[waypoint_idx].SetX(curr_point_.x() - 
					overlap_.Xmin() - params_.GetVehWidth()/2 - 
					params_.GetMargin());
			}

			if (alpha_y_[waypoint_idx] > 0){
				max_waypoint_offsets_[waypoint_idx].SetY(overlap_.Ymax() - 
					params_.GetVehHeight()/2 - params_.GetMargin() - 
					curr_point_.y());
			} else {
				max_waypoint_offsets_[waypoint_idx].SetY(curr_point_.y() - 
					overlap_.Ymin() - params_.GetVehHeight()/2 - 
					params_.GetMargin());
			}

		} else {
			// if there is no gap between the line of sight and the waypoint,
			// allow the waypoint to move (within the overlap region of the
			// movable waypoint), and flip the acceleration vector only if 
			// the waypoint cannot really move

			// compute the movable distances
			if (alpha_x_[waypoint_idx] < 0){
				max_waypoint_offsets_[waypoint_idx].SetX(curr_point_.x() -
					overlap_.Xmax() + params_.GetVehWidth()/2 + 
					params_.GetMargin());
			} else {
				max_waypoint_offsets_[waypoint_idx].SetX(overlap_.Xmin() - 
					curr_point_.x() + params_.GetVehWidth()/2 + 
					params_.GetMargin());
			}

			if (alpha_y_[waypoint_idx] < 0){
				max_waypoint_offsets_[waypoint_idx].SetY(curr_point_.y() - 
					overlap_.Ymax() + params_.GetVehHeight()/2 +
					params_.GetMargin());
			} else {
				max_waypoint_offsets_[waypoint_idx].SetY(overlap_.Ymin() - 
					curr_point_.y() + params_.GetVehHeight()/2 + 
					params_.GetMargin());
			}

			// flip the vector if the waypoint cannot move
            //
            // SUBOPTIMAL (replanning): 
            // the code below improves the parametrization, but makes it 
            // much more difficult to replan in a feasible way
            //
			if (false && 
					std::max(max_waypoint_offsets_[waypoint_idx].x(),
							 max_waypoint_offsets_[waypoint_idx].y()) < 5.1e-3){
				alpha_x_[waypoint_idx] = -alpha_x_[waypoint_idx];
				alpha_y_[waypoint_idx] = -alpha_y_[waypoint_idx];
				movable_waypoints_[waypoint_idx] = false;
			}
		}
	} else {
		// if there is no line of sight, the waypoint cannot move
		movable_waypoints_[waypoint_idx] = false;
	}
}

bool Parametrization::LineOfSightInCorridors(Point2D<double> const &point1, 
											 Point2D<double> const &point2, 
											 Corridor const &corridor1,
											 Corridor const &corridor2) const {
	// Construct the normalized line of sight
	Point2D<double> line_of_sight = point2 - point1;

	// starting from point 1, find the last point in the line of sight that
	// is still in the corridor
	double x_margin;
	double width_offset = params_.GetVehWidth()/2 + params_.GetMargin();
	if (line_of_sight.x() > 0){
		x_margin = corridor1.Xmax() - point1.x() - width_offset;
	} else {
		x_margin = point1.x() - corridor1.Xmin() - width_offset;
	}
 	double y_margin;
	double height_offset = params_.GetVehHeight()/2 + params_.GetMargin();
	if (line_of_sight.y() > 0){
		y_margin = corridor1.Ymax() - point1.y() - height_offset;
	} else {
		y_margin = point1.y() - corridor1.Ymin() - height_offset;
	}

	double t_x = std::abs(x_margin/line_of_sight.x());
	double t_y = std::abs(y_margin/line_of_sight.y());
	double t1 = std::min(t_x, t_y);

	// starting from point 2, find the last point in the line of sight that
	// is still in the corridor
	if (line_of_sight.x() < 0){
		x_margin = corridor2.Xmax() - point2.x() - width_offset;
	} else {
		x_margin = point2.x() - corridor2.Xmin() - width_offset;
	}
	if (line_of_sight.y() < 0){
		y_margin = corridor2.Ymax() - point2.y() - height_offset;
	} else {
		y_margin = point2.y() - corridor2.Ymin() - height_offset;
	}

	t_x = std::abs(x_margin/line_of_sight.x());
	t_y = std::abs(y_margin/line_of_sight.y());
	double t2 = std::min(t_x, t_y);

	return t1 + t2 >= 1.0;
}

void Parametrization::IntegrateOverCorridor(Point2D<MX> const &start, 
											Point2D<MX> const &start_vel, 
											MX const &t_x, MX const &t_y,
											MX const &alpha_x, 
											MX const &alpha_y, 
											MX const &alpha_x_next, 
											MX const &alpha_y_next,
											MX const &a_max){
	Point2D<MX> t;
	Point2D<MX> accel;

	// First arc
	t.SetX(t_x(0)); t.SetY(t_y(0));
	accel.SetX(a_max*alpha_x); accel.SetY(a_max*alpha_y);
	intermediate_positions_[0].CopyValues(start + t*start_vel + 
										  t*t*accel*0.5);
	intermediate_velocities_[0].CopyValues(start_vel + t*accel);

	// First arc - half
	t.SetX(0.5*t_x(0)); t.SetY(0.5*t_y(0));
	intermediate_positions_[3].CopyValues(start + t*start_vel + 
										  t*t*accel*0.5);
	
	// Second arc
	t.SetX(t_x(1)); t.SetY(t_y(1));
	accel.SetX(0); accel.SetY(0);
	intermediate_positions_[1].CopyValues(intermediate_positions_[0] + 
										  t*intermediate_velocities_[0]);
	intermediate_velocities_[1].CopyValues(intermediate_velocities_[0]);

	// Third arc
	t.SetX(t_x(2)); t.SetY(t_y(2));
	accel.SetX(a_max*alpha_x_next); accel.SetY(a_max*alpha_y_next);
	intermediate_positions_[2].CopyValues(intermediate_positions_[1] + 
										  t*intermediate_velocities_[1] + 
										  t*t*accel*0.5);
	intermediate_velocities_[2].CopyValues(intermediate_velocities_[1] + 
										   t*accel);
}

void Parametrization::ApplyOvershootingPreventionConstraint(
									Opti &opti, MX &t_x, MX &t_y){
	double local_alpha;
	double dist_waypoints;
	double starting_vel_bd;
	MX t0;
	
	if (waypoints_[0].x() - waypoints_[1].x() > 
		waypoints_[0].y() - waypoints_[1].y()){
		// x is bottleneck
		local_alpha = alpha_x_[0];
		dist_waypoints = waypoints_[1].x() - waypoints_[0].x();
		starting_vel_bd = corridor_sequence_.GetStartVel().x();
		t0 = t_x(0, 0);
	} else {
		// y is bottleneck
		local_alpha = alpha_y_[0];
		dist_waypoints = waypoints_[1].y() - waypoints_[0].y();
		starting_vel_bd = corridor_sequence_.GetStartVel().y();
		t0 = t_y(0, 0);
	}
	bool cond_correct_direction = (starting_vel_bd > 0 ? 1.0 : -1.0) == 
			 					  (dist_waypoints > 0 ? 1.0 : -1.0);
	bool cond_braking = (local_alpha > 0 ? 1.0 : -1.0) == 
						(starting_vel_bd > 0 ? -1.0 : 1.0);
	bool cond_braking_distance = 0.5*std::pow(starting_vel_bd, 2)/
						params_.GetAmax() > std::abs(dist_waypoints);

	if (cond_correct_direction && cond_braking && cond_braking_distance){
		// compute maximum duration to not overshoot the first waypoint
		double t_limit = (std::abs(starting_vel_bd) - 
						  std::sqrt(std::pow(starting_vel_bd, 2) - 
						  			2*params_.GetAmax()*
										std::abs(dist_waypoints)))/
						 (params_.GetAmax());
		opti.subject_to(t0 <= t_limit);
	} 
}

void Parametrization::ExtractSolutionOld(){
	try {
		sol_ = opti_.solve();
		// ShowOptiDebugInfo();
		solver_time_ = sol_.value().stats()["t_wall_total"];
		solver_time_ *= 1000; // convert to milliseconds

		for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
			alpha_x_sol_[i] = double(sol_.value().value(alpha_x_mx_(i)));
			alpha_y_sol_[i] = double(sol_.value().value(alpha_y_mx_(i)));
			waypoints_sol_[i].SetX(double(sol_.value().value(waypoints_mx_[i].x())));
			waypoints_sol_[i].SetY(double(sol_.value().value(waypoints_mx_[i].y())));

			if (i < corridor_sequence_.NbCorridors()){
				waypoint_velocities_sol_[i].SetX(double(sol_.value().value(v_x_(i))));
				waypoint_velocities_sol_[i].SetY(double(sol_.value().value(v_y_(i))));
				
				for (int j = 0; j < 3; j++){
					t_x_sol_[i][j] = double(sol_.value().value(t_x_(j, i)));
					t_y_sol_[i][j] = double(sol_.value().value(t_y_(j, i)));
				}
			} else {
				waypoint_velocities_sol_[i].SetX(double(
					sol_.value().value(v_x_(i-1)) + 
					alpha_x_sol_[i]*params_.GetAmax()*t_x_sol_[i][0] + 
					alpha_x_sol_[i+1]*params_.GetAmax()*t_x_sol_[i][2]));
				waypoint_velocities_sol_[i].SetY(double(
					sol_.value().value(v_y_(i-1)) + 
					alpha_y_sol_[i]*params_.GetAmax()*t_y_sol_[i][0] + 
					alpha_y_sol_[i+1]*params_.GetAmax()*t_y_sol_[i][2]));
			}
		}

	} catch (std::exception &e){
		ShowOptiDebugInfoFailed();
		solver_time_ = -1.0;
		std::cout << "An error occured: " << e.what() << std::endl;
		for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
			alpha_x_sol_[i] = double(opti_.debug().value(alpha_x_mx_(i)));
			alpha_y_sol_[i] = double(opti_.debug().value(alpha_y_mx_(i)));
			waypoints_sol_[i].SetX(double(opti_.debug().value(waypoints_mx_[i].x())));
			waypoints_sol_[i].SetY(double(opti_.debug().value(waypoints_mx_[i].y())));
			waypoint_velocities_sol_[i].SetX(double(opti_.debug().value(v_x_(i))));
			waypoint_velocities_sol_[i].SetY(double(opti_.debug().value(v_y_(i))));

			if (i < corridor_sequence_.NbCorridors()){
				for (int j = 0; j < 3; j++){
					t_x_sol_[i][j] = double(opti_.debug().value(t_x_(j, i)));
					t_y_sol_[i][j] = double(opti_.debug().value(t_y_(j, i)));
				}
			}
		}
	}

	std::cout << "Tx: " << t_x_sol_ << std::endl;
	std::cout << "Ty: " << t_y_sol_ << std::endl;
}

void Parametrization::ExtractSolution(){

	if (latest_success_status_ != 1){
		solver_time_ = -1.0;
	} else {
		solver_time_ = active_opti_instance_.stats()["t_wall_total"];
		solver_time_ *= 1000; // convert to milliseconds
	}

	int offset_ptr = 0;
	for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
		alpha_x_sol_[i] = double(latest_solution_["alpha_x"](i));
		alpha_y_sol_[i] = double(latest_solution_["alpha_y"](i));

		if (active_opti_code_[i] == '0'){
			waypoints_sol_[i].SetX(waypoints_[i].x());
			waypoints_sol_[i].SetY(waypoints_[i].y());
		} else {
			waypoints_sol_[i].SetX(waypoints_[i].x() + alpha_x_[i]*double(latest_solution_["offsets"](0, offset_ptr)));
			waypoints_sol_[i].SetY(waypoints_[i].y() + alpha_y_[i]*double(latest_solution_["offsets"](1, offset_ptr)));
			offset_ptr++;
		}

		waypoint_velocities_sol_[i].SetX(double(latest_solution_["v_x"](i)));
		waypoint_velocities_sol_[i].SetY(double(latest_solution_["v_y"](i)));

		if (i < corridor_sequence_.NbCorridors()){			
			for (int j = 0; j < 3; j++){
				t_x_sol_[i][j] = double(latest_solution_["t_x"](j, i));
				t_y_sol_[i][j] = double(latest_solution_["t_y"](j, i));
			}
		}
	}
}

void Parametrization::ShowOptiDebugInfo(){
	std::vector<double> my_x(opti_.x().size1(), 0.5);
	DM my_x_DM = DM(my_x);

	casadi::Function J = casadi::Function("J", {opti_.x()}, {jacobian(opti_.g(), opti_.x())});
	casadi::DM J_val = J(my_x_DM)[0];
	std::vector<double> jac_values = {};
	for (int i = 0; i < J_val.size1(); i++){
		for (int j = 0; j < J_val.size2(); j++){
			if (J_val.has_nz(i,j)){
				jac_values.push_back(double(J_val(i, j)));
			}
		}
	}
	std::cout << "jac_values_cpp = [";
	for (auto val : jac_values){
		std::cout << val << ", ";
	}
	std::cout << "]" << std::endl;
	J_val = J(sol_.value().value(opti_.x()))[0];
	casadi::Sparsity J_sparsity = J_val.sparsity();
	for (int i = 0; i < J_sparsity.size1(); i++){
		for (int j = 0; j < J_sparsity.size2(); j++){
			if (J_sparsity.has_nz(i, j) != 0){
				std::cout << "X ";
			} else {
				std::cout << ". ";
			}
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	std::cout << "dimensions: " << J_sparsity.size1() << " x " << J_sparsity.size2() << std::endl;

	casadi::Function g = casadi::Function("g", {opti_.x()}, {opti_.g()});
	casadi::DM g_val = g(my_x_DM)[0];
	std::cout << "g_values_cpp = [";
	for (int i = 0; i < g_val.size1(); i++){
		std::cout << double(g_val(i)) << ", ";
	}
	std::cout << "]" << std::endl;
}

void Parametrization::ShowOptiDebugInfoFailed(){
	std::vector<double> my_x(opti_.x().size1(), 0.5);
	DM my_x_DM = DM(my_x);

	casadi::Function J = casadi::Function("J", {opti_.x()}, {jacobian(opti_.g(), opti_.x())});
	casadi::DM J_val = J(my_x_DM)[0];
	std::vector<double> jac_values = {};
	for (int i = 0; i < J_val.size1(); i++){
		for (int j = 0; j < J_val.size2(); j++){
			if (J_val.has_nz(i,j)){
				jac_values.push_back(double(J_val(i, j)));
			}
		}
	}
	std::cout << "jac_values_cpp = [";
	for (auto val : jac_values){
		std::cout << val << ", ";
	}
	std::cout << "]" << std::endl;
	J_val = J(opti_.debug().value(opti_.x()))[0];
	casadi::Sparsity J_sparsity = J_val.sparsity();
	for (int i = 0; i < J_sparsity.size1(); i++){
		for (int j = 0; j < J_sparsity.size2(); j++){
			if (J_sparsity.has_nz(i, j) != 0){
				std::cout << "X ";
			} else {
				std::cout << ". ";
			}
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	std::cout << "dimensions: " << J_sparsity.size1() << " x " << J_sparsity.size2() << std::endl;

	casadi::Function g = casadi::Function("g", {opti_.x()}, {opti_.g()});
	casadi::DM g_val = g(my_x_DM)[0];
	std::cout << "g_values_cpp = [";
	for (int i = 0; i < g_val.size1(); i++){
		std::cout << double(g_val(i)) << ", ";
	}
	std::cout << "]" << std::endl;
}

void Parametrization::InitializeFirstArcNew(bool invert){
	double v_des = params_.GetVmax();

	int corridor_idx = invert ? corridor_sequence_.NbCorridors()-1 : 0;
	Point2D<double> start_vel = invert ? Point2D<double>(0,0) : 
										corridor_sequence_.GetStartVel();
	Point2D<double> p0 = invert ? Point2D<double>(waypoints_[corridor_idx+1]) :
								  Point2D<double>(waypoints_[corridor_idx]);
	Point2D<double> pf = invert ? Point2D<double>(waypoints_[corridor_idx]) :
								  Point2D<double>(waypoints_[corridor_idx+1]);

	double p0_bd, pf_bd, v0_bd, alpha_bd, alpha_next_bd, t1_bd, t2_bd, t3_bd;
	double p0_fd, pf_fd, v0_fd, alpha_fd, alpha_next_fd, t1_fd, t2_fd, t3_fd;

	t1_bd = 0; t2_bd = 0; t3_bd = 0;
	t1_fd = 0; t2_fd = 0; t3_fd = 0;

	double a_max = params_.GetAmax();

	// Compute the bottleneck direction
	if (corridor_idx == 0 && initial_bottleneck_direction_ == 1 ||
	  		corridor_idx > 0 && std::abs(p0.x() - pf.x()) < std::abs(p0.y() - pf.y())){
		p0_bd = p0.y(); pf_bd = pf.y(); v0_bd = start_vel.y();
		alpha_bd = alpha_y_[corridor_idx];
		alpha_next_bd = alpha_y_[corridor_idx + 1];

		p0_fd = p0.x(); pf_fd = pf.x(); v0_fd = start_vel.x();
		alpha_fd = alpha_x_[corridor_idx];
		alpha_next_fd = alpha_x_[corridor_idx + 1];
	} else {
		p0_bd = p0.x(); pf_bd = pf.x(); v0_bd = start_vel.x();
		alpha_bd = alpha_x_[corridor_idx];
		alpha_next_bd = alpha_x_[corridor_idx + 1];

		p0_fd = p0.y(); pf_fd = pf.y(); v0_fd = start_vel.y();
		alpha_fd = alpha_y_[corridor_idx];
		alpha_next_fd = alpha_y_[corridor_idx + 1];
	}

	// Compute timings in the bottleneck direction
	int sign = (pf_bd - p0_bd > 0) ? 1 : -1;
	t1_bd = std::abs(v_des * sign - v0_bd) / a_max;
	t2_bd = (std::abs(pf_bd - p0_bd) - 
			 std::abs(v0_bd*t1_bd + 
			 		  0.5*alpha_bd*a_max*std::pow(t1_bd, 2))) / v_des;
	double T = t1_bd + t2_bd;


	double t_accel = std::sqrt(2*std::abs(p0_bd - pf_bd)/a_max);
	if (t1_bd >= t_accel){
		t1_bd = t_accel;
		t2_bd = 0;
	}

	T = t1_bd + t2_bd;

	// in the first corridor, use the same time durations in the free
	// direction as in the bottleneck direction and select an appropriate
	// acceleration
	t1_fd = t1_bd;
	t2_fd = t2_bd;

	if (invert){
		if (t1_bd > 1.0e-15){
			alpha_f_init_ = 1.0/a_max * 
							std::min(a_max,
							std::max(-a_max, 
										(pf_fd - p0_fd - v0_fd*T) / 
										(0.5*std::pow(t1_bd,2) + t1_bd*t2_bd)));
		} else {
			alpha_f_init_ = pf_fd - p0_fd > 0 ? 1 : -1;
		}

		alpha_fd = alpha_f_init_;
	} else {
		if (t1_bd > 1.0e-15){
			alpha_0_init_ = 1.0/a_max * 
							std::min(a_max,
							std::max(-a_max, 
										(pf_fd - p0_fd - v0_fd*T) / 
										(0.5*std::pow(t1_bd,2) + t1_bd*t2_bd)));
		} else {
			alpha_0_init_ = pf_fd - p0_fd > 0 ? 1 : -1;
		}

		alpha_fd = alpha_0_init_;
	}
	
	// write the values in the initialization containers
	if (std::abs(p0.x() - pf.x()) < std::abs(p0.y() - pf.y())){
		t_x_init_[corridor_idx][0] = invert ? t3_fd : t1_fd;
		t_x_init_[corridor_idx][1] = t2_fd;
		t_x_init_[corridor_idx][2] = invert ? t1_fd : t3_fd;

		t_y_init_[corridor_idx][0] = invert ? t3_bd : t1_bd;
		t_y_init_[corridor_idx][1] = t2_bd;
		t_y_init_[corridor_idx][2] = invert ? t1_bd : t3_bd;

		if (invert){
			waypoint_velocities_init_[corridor_idx].SetX(
				-alpha_x_[corridor_idx]*params_.GetAmax()*t_x_init_[corridor_idx][0]
				-alpha_fd*params_.GetAmax()*t_x_init_[corridor_idx][2]
			);
			waypoint_velocities_init_[corridor_idx].SetY(
				-alpha_y_[corridor_idx]*params_.GetAmax()*t_y_init_[corridor_idx][0]
				-alpha_y_[corridor_idx+1]*params_.GetAmax()*t_y_init_[corridor_idx][2]
			);
			waypoint_velocities_init_[corridor_idx + 1].SetX(0);
			waypoint_velocities_init_[corridor_idx + 1].SetY(0);
		} else {
			waypoint_velocities_init_[0].SetX(0);
			waypoint_velocities_init_[0].SetY(0);
		}
	} else {
		t_x_init_[corridor_idx][0] = invert ? t3_bd : t1_bd;
		t_x_init_[corridor_idx][1] = t2_bd;
		t_x_init_[corridor_idx][2] = invert ? t1_bd : t3_bd;

		t_y_init_[corridor_idx][0] = invert ? t3_fd : t1_fd;
		t_y_init_[corridor_idx][1] = t2_fd;
		t_y_init_[corridor_idx][2] = invert ? t1_fd : t3_fd;

		if (invert){
			waypoint_velocities_init_[corridor_idx].SetX(
				-alpha_x_[corridor_idx]*params_.GetAmax()*t_x_init_[corridor_idx][0]
				-alpha_x_[corridor_idx+1]*params_.GetAmax()*t_x_init_[corridor_idx][2]
			);
			waypoint_velocities_init_[corridor_idx].SetY(
				-alpha_y_[corridor_idx]*params_.GetAmax()*t_y_init_[corridor_idx][0]
				-alpha_fd*params_.GetAmax()*t_y_init_[corridor_idx][2]
			);
			waypoint_velocities_init_[corridor_idx + 1].SetX(0);
			waypoint_velocities_init_[corridor_idx + 1].SetY(0);
		} else {
			waypoint_velocities_init_[0].SetX(0);
			waypoint_velocities_init_[0].SetY(0);
		}
	}
}

void Parametrization::InitializeOptimizationNew(){
	InitializeFirstArcNew(false);
	// 		t0 = u, t1 = K*u, y2 = L*u
	// 		v_0(u) = (A + Bu^2)/u
	double Ax, Bx, Ay, By, u;
	double K = 7;
	double L = 0.2;
	for (int w = 1; w < corridor_sequence_.NbCorridors()-1; w++){
		Ax = (waypoints_[w+1].x() - waypoints_[w].x())/(1+K+L);
		Bx = ((L+K)*alpha_x_[w]*params_.GetAmax() - 
			0.5*params_.GetAmax()*(alpha_x_[w] + alpha_x_[w+1]*L*L))/(1+K+L);

		Ay = (waypoints_[w+1].y() - waypoints_[w].y())/(1+K+L);
		By = ((L+K)*alpha_y_[w]*params_.GetAmax() - 
			0.5*params_.GetAmax()*(alpha_y_[w] + alpha_y_[w+1]*L*L))/(1+K+L);
		
		if (Ax < Ay){
			u = std::max(0.05, std::sqrt(std::abs(Ax/Bx)));
		} else {
			u = std::max(0.05, std::sqrt(std::abs(Ay/By)));
		}
		
		t_x_init_[w][0] = u;
		t_x_init_[w][1] = K*u;
		t_x_init_[w][2] = L*u;
		waypoint_velocities_init_[w].SetX((Ax + Bx*u*u)/u);
		t_y_init_[w][0] = u;
		t_y_init_[w][1] = K*u;
		t_y_init_[w][2] = L*u;
		waypoint_velocities_init_[w].SetY((Ay + By*u*u)/u);
	}

	InitializeFirstArcNew(true);
}

void Parametrization::InitializeOptimization(){
	InitializeOptimizationNew();
	return;
	double v_des;
	if (params_.GetAmax() > params_.GetVmax()){
		v_des = std::min(0.2, params_.GetVmax());
	} else {
		v_des = std::min(0.4, params_.GetVmax());
	}
	// v_des = std::max(std::abs(corridor_sequence_.GetStartVel().x()),
	// 				 std::abs(corridor_sequence_.GetStartVel().y()));
	// v_des = std::max(v_des, 0.4);
	// double v_des_original = v_des;
	v_des = params_.GetVmax();

	bool initialization_complete = false;
	Point2D<double> curr_vel;

	while (!initialization_complete){
		// set-up the initialization loop
		curr_vel = corridor_sequence_.GetStartVel();
		if (std::abs(curr_vel.x()) <= v_des &&
				std::abs(curr_vel.y()) <= v_des){
			waypoint_velocities_init_[0].CopyValues(curr_vel);
		} else {
			// double norm = curr_vel.Norm();
			// waypoint_velocities_init_[0].SetX(curr_vel.x()*v_des/norm);
			// waypoint_velocities_init_[0].SetY(curr_vel.y()*v_des/norm);
			double scaling = v_des/std::max(std::abs(curr_vel.x()), 
											std::abs(curr_vel.y()));
			waypoint_velocities_init_[0].SetX(curr_vel.x()*scaling);
			waypoint_velocities_init_[0].SetY(curr_vel.y()*scaling);
		}

		// start initializing
		bool success;
		for (int w = 0; w < corridor_sequence_.NbCorridors(); w++){
			success = InitializeArc(w, v_des, curr_vel);	
			if (!success){
				if (v_des <= 0.1){
					success = true;
				} else {
					// try again with lower velocity
					std::cout << "Starting over!" << std::endl;
					v_des *= 0.7;
					break;
				}
			}

			curr_vel.CopyValues(waypoint_velocities_init_[w+1]);
		}

		// check if initialization was successful
		if (success){
			initialization_complete = true;
		}
	}

	// modify initialization to limit the coasting time (considering the true
	// initial velocity)
	double dist_x = std::abs(waypoints_[1].x() - waypoints_[0].x());
	double dist_y = std::abs(waypoints_[1].y() - waypoints_[0].y());
	double t_x = dist_x / std::abs(waypoint_velocities_init_[0].x() + 1.0e-5);
	double t_y = dist_y / std::abs(waypoint_velocities_init_[0].y() + 1.0e-5);
	t_x_init_[0][1] = std::min(t_x_init_[0][1], t_x);
	t_y_init_[0][1] = std::min(t_y_init_[0][1], t_y);
}

bool Parametrization::InitializeArc(int corridor_idx, double v_des, 
									Point2D<double> const &start_vel){
	bool succesfull_initialization = true;

	Point2D<double> p0(waypoints_[corridor_idx]);
	Point2D<double> pf(waypoints_[corridor_idx + 1]);

	double p0_bd, pf_bd, v0_bd, alpha_bd, alpha_next_bd, t1_bd, t2_bd, t3_bd;
	double p0_fd, pf_fd, v0_fd, alpha_fd, alpha_next_fd, t1_fd, t2_fd, t3_fd;

	t1_bd = 0; t2_bd = 0; t3_bd = 0;
	t1_fd = 0; t2_fd = 0; t3_fd = 0;

	double a_max = params_.GetAmax();

	// Compute the bottleneck direction
	if (corridor_idx == 0 && initial_bottleneck_direction_ == 1 ||
	  		corridor_idx > 0 && std::abs(p0.x() - pf.x()) < std::abs(p0.y() - pf.y())){
		p0_bd = p0.y(); pf_bd = pf.y(); v0_bd = start_vel.y();
		alpha_bd = alpha_y_[corridor_idx];
		alpha_next_bd = alpha_y_[corridor_idx + 1];

		p0_fd = p0.x(); pf_fd = pf.x(); v0_fd = start_vel.x();
		alpha_fd = alpha_x_[corridor_idx];
		alpha_next_fd = alpha_x_[corridor_idx + 1];
	} else {
		p0_bd = p0.x(); pf_bd = pf.x(); v0_bd = start_vel.x();
		alpha_bd = alpha_x_[corridor_idx];
		alpha_next_bd = alpha_x_[corridor_idx + 1];

		p0_fd = p0.y(); pf_fd = pf.y(); v0_fd = start_vel.y();
		alpha_fd = alpha_y_[corridor_idx];
		alpha_next_fd = alpha_y_[corridor_idx + 1];
	}

	// Compute timings in the bottleneck direction
	int sign = (pf_bd - p0_bd > 0) ? 1 : -1;
	t1_bd = std::abs(v_des * sign - v0_bd) / a_max;
	t2_bd = (std::abs(pf_bd - p0_bd) - 
			 std::abs(v0_bd*t1_bd + 
			 		  0.5*alpha_bd*a_max*std::pow(t1_bd, 2))) / v_des;
	double T = t1_bd + t2_bd;

	// std::cout << "Step 1: " << t1_bd << ", " << t2_bd << " , " << T << std::endl;

	// Compute timings in the free direction
	if (corridor_idx == 0){
		double t_accel = std::sqrt(2*std::abs(p0_bd - pf_bd)/a_max);
		if (t1_bd >= t_accel){
			t1_bd = t_accel;
			t2_bd = 0;
		}

		T = t1_bd + t2_bd;

		// in the first corridor, use the same time durations in the free
		// direction as in the bottleneck direction and select an appropriate
		// acceleration
		t1_fd = t1_bd;
		t2_fd = t2_bd;

		if (t1_bd > 1.0e-15){
			alpha_0_init_ = 1.0/a_max * 
							std::min(a_max,
							std::max(-a_max, 
									 (pf_fd - p0_fd - v0_fd*T) / 
									 (0.5*std::pow(t1_bd,2) + t1_bd*t2_bd)));
		} else {
			alpha_0_init_ = pf_fd - p0_fd > 0 ? 1 : -1;
		}

		alpha_fd = alpha_0_init_;

		// std::cout << "Step 2: " << t1_fd << ", " << t2_fd << " , " << T << ", " << alpha_fd << std::endl;
	} else {
		// in subsequent corridors, make sure the next waypoint is reached at 
		// the exact same time as in the bottleneck direction
		double A = -0.5*alpha_fd*a_max;
		double B = alpha_fd*a_max*T;
		double C = p0_fd + v0_fd*T - pf_fd;
		double D = std::pow(B, 2) - 4*A*C;

		if (D >= 0){
			std::vector<double> roots = {(-B - std::sqrt(D))/(2*A),
										(-B + std::sqrt(D))/(2*A)};

			// remove negative roots
			if (roots[1] < 0){ roots.erase(roots.begin() + 1);}
			if (roots[0] < 0){ roots.erase(roots.begin());}

			if (roots.size() == 0){ roots = {0};}

			double t1_fd_max;
			if (alpha_fd > 0){ t1_fd_max = (v_des - v0_fd) / a_max;}
			else { t1_fd_max = (v0_fd + v_des) / a_max;}

			// get the smallest element from roots
			double smallest_root = *std::min_element(roots.begin(), roots.end());

			t1_fd = std::min(smallest_root, t1_fd_max);
			t2_fd = T - t1_fd;
		} else {
			// the free direction runs out of time
			t1_fd = t1_bd;
			t2_fd = t2_bd;

			succesfull_initialization = false;
		}
	}

	// If this is the final corridor, make sure we come to a stop
	if (corridor_idx == corridor_sequence_.NbCorridors() - 1){
		// include braking time in the bottleneck direction
		double v_bd_reached = v0_bd + alpha_bd*a_max*t1_bd;
		t3_bd = std::abs(v_bd_reached) / a_max;

		double dist_accel = std::abs(v0_bd*t1_bd + 
									0.5*alpha_bd*a_max*std::pow(t1_bd, 2));
		double dist_brake = 0.5*a_max*std::pow(t3_bd, 2);
		double dist_coast = std::abs(pf_bd - p0_bd) - dist_accel - dist_brake;
		t2_bd = dist_coast/v_des;
		T = t1_bd + t2_bd + t3_bd;

		// start braking at the same time in the free direction
		t3_fd = t3_bd;
		t2_fd = std::max(0.0, T - t1_fd - t3_fd);

		// compute point where the braking starts
		double p_brake_bd = p0_bd;
		double p_brake_fd = p0_fd;
		double v_brake_bd = v0_bd;
		double v_brake_fd = v0_fd;

		// 		integrate initial accelerations
		p_brake_bd += v_brake_bd*t1_bd + 0.5*alpha_bd*a_max*std::pow(t1_bd, 2);
		p_brake_fd += v_brake_fd*t1_fd + 0.5*alpha_fd*a_max*std::pow(t1_fd, 2);
		v_brake_bd += alpha_bd*a_max*t1_bd;
		v_brake_fd += alpha_fd*a_max*t1_fd;

		// 		integrate coasting
		p_brake_bd += v_brake_bd*t2_bd;
		p_brake_fd += v_brake_fd*t2_fd;

		alpha_f_init_ = pf_fd - p_brake_fd > 0 ? -1 : 1;
		alpha_f_init_ *= std::min(1.0, std::max(-1.0, 
							std::abs(v_brake_fd/v_brake_bd)));
		alpha_next_fd = alpha_f_init_;
	}

	bool negative_value_detected = t1_fd < 0 || t2_fd < 0 || t3_fd < 0 ||
								   t1_bd < 0 || t2_bd < 0 || t3_bd < 0;
	bool equal_timings = (t1_fd + t2_fd + t3_fd) == (t1_bd + t2_bd + t3_bd);
	succesfull_initialization = succesfull_initialization && 
								!negative_value_detected &&
								equal_timings;

	bool next_waypoint_reached = false;
	
	// write the values in the initialization containers
	if (std::abs(p0.x() - pf.x()) < std::abs(p0.y() - pf.y())){
		t_x_init_[corridor_idx][0] = t1_fd;
		t_x_init_[corridor_idx][1] = t2_fd;
		t_x_init_[corridor_idx][2] = t3_fd;

		t_y_init_[corridor_idx][0] = t1_bd;
		t_y_init_[corridor_idx][1] = t2_bd;
		t_y_init_[corridor_idx][2] = t3_bd;

		waypoint_velocities_init_[corridor_idx + 1].SetX(v0_fd + 
			alpha_fd*a_max*t1_fd + alpha_next_fd*a_max*t3_fd);
		waypoint_velocities_init_[corridor_idx + 1].SetY(v0_bd +
			alpha_bd*a_max*t1_bd + alpha_next_bd*a_max*t3_bd);

		double px = p0.x() + v0_fd*(t1_fd + t2_fd + t3_fd) + 
					a_max*alpha_fd*t1_fd*t2_fd + 
					0.5*a_max*alpha_fd*std::pow(t1_fd, 2);
		double py = p0.y() + v0_bd*(t1_bd + t2_bd + t3_bd) +
					a_max*alpha_bd*t1_bd*t2_bd + 
					0.5*a_max*alpha_bd*std::pow(t1_bd, 2);
		next_waypoint_reached = std::abs(px - pf.x()) < 1.0e-3 &&
								std::abs(py - pf.y()) < 1.0e-3;
	} else {
		t_x_init_[corridor_idx][0] = t1_bd;
		t_x_init_[corridor_idx][1] = t2_bd;
		t_x_init_[corridor_idx][2] = t3_bd;

		t_y_init_[corridor_idx][0] = t1_fd;
		t_y_init_[corridor_idx][1] = t2_fd;
		t_y_init_[corridor_idx][2] = t3_fd;

		waypoint_velocities_init_[corridor_idx + 1].SetX(v0_bd + 
			alpha_bd*a_max*t1_bd + alpha_next_bd*a_max*t3_bd);
		waypoint_velocities_init_[corridor_idx + 1].SetY(v0_fd +
			alpha_fd*a_max*t1_fd + alpha_next_fd*a_max*t3_fd);

		double px = p0.x() + v0_bd*(t1_bd + t2_bd + t3_bd) + 
					a_max*alpha_bd*t1_bd*t2_bd + 
					0.5*a_max*alpha_bd*std::pow(t1_bd, 2);
		double py = p0.y() + v0_fd*(t1_fd + t2_fd + t3_fd) +
					a_max*alpha_fd*t1_fd*t2_fd + 
					0.5*a_max*alpha_fd*std::pow(t1_fd, 2);
		next_waypoint_reached = std::abs(px - pf.x()) < 1.0e-3 &&
								std::abs(py - pf.y()) < 1.0e-3;
	}

	succesfull_initialization = succesfull_initialization && next_waypoint_reached;

	return succesfull_initialization;	
}

void Parametrization::ShowInitialization(){
	InitializeOptimization();

	double accumulator = 0.0;
	std::cout << "t_x_init_ (acc): " << std::endl;
	for (int i = 0; i < corridor_sequence_.NbCorridors(); i++){
		for (auto f : t_x_init_[i]){
			std::cout << accumulator + f << " ";
			accumulator += f;
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	
	accumulator = 0;
	std::cout << "t_y_init_ (acc): " << std::endl;
	for (int i = 0; i < corridor_sequence_.NbCorridors(); i++){
		for (auto f : t_y_init_[i]){
			std::cout << accumulator + f << " ";
			accumulator += f;
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	std::cout << "alpha_0_init_: " << alpha_0_init_ << std::endl;
	std::cout << "alpha_f_init_: " << alpha_f_init_ << std::endl;

	std::cout << "waypoint_velocities_init_: " << std::endl;
	for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
		std::cout << waypoint_velocities_init_[i] << std::endl;
	}

	// update free accelerations with initialized values
	std::vector<double> alpha_x_temp(alpha_x_);
	std::vector<double> alpha_y_temp(alpha_y_);
	if (std::abs(waypoints_[0].x() - waypoints_[1].x()) < 
			std::abs(waypoints_[0].y() - waypoints_[1].y())){
		alpha_x_temp[0] = alpha_0_init_;
	} else { alpha_y_temp[0] = alpha_0_init_;}

	if (std::abs(waypoints_[corridor_sequence_.NbCorridors()].x() - 
				waypoints_[corridor_sequence_.NbCorridors() - 1].x()) < 
			std::abs(waypoints_[corridor_sequence_.NbCorridors()].y() - 
					 waypoints_[corridor_sequence_.NbCorridors() - 1].y())){
		alpha_x_temp[corridor_sequence_.NbCorridors()] = alpha_f_init_;
	} else { alpha_y_temp[corridor_sequence_.NbCorridors()] = alpha_f_init_;}

	// Trajectory initialized_trajectory = Trajectory();
	initialized_trajectory_.Update(corridor_sequence_,
								   t_x_init_, t_y_init_,
								   alpha_x_temp, alpha_y_temp,
								   waypoints_, waypoint_velocities_init_,
								   params_, 0.0);
		
	// std::cout << "Initialized trajectory" << std::endl;
	// std::cout << initialized_trajectory << std::endl;
}

void Parametrization::OptimizeSingleArc1D(std::vector<double> &t_sol_vector,
										  std::vector<double> &alpha_sol_vector,
										  double p0, double pf, double v0){
	double pf_rel = std::abs(pf - p0);
	double v_max = params_.GetVmax();
	double a_max = params_.GetAmax();

	if (pf_rel == 0){
		t_sol_vector[0] = 0; t_sol_vector[1] = 0; t_sol_vector[2] = 0;
		alpha_sol_vector[0] = 0; alpha_sol_vector[1] = 0;
		return;
	}
	
	if (pf - p0 < 0){
		v0 = -v0;
	}

	double T = 0;
	double ts1 = 0;
	double ts2 = 0;
	double ts3 = 0;

	// first, check if the velocity limit is being exceeded
	if (v0 >= params_.GetVmax() + 1.0e-7){
		double t1 = (v0 - v_max)/a_max;
		double t2 = 1.0/v_max*(pf_rel - 1.0/a_max*(2*v0*v_max - 
									 			   0.5*std::pow(v0, 2) - 
												   v_max));
		double t3 = v_max/a_max;
		T = t1 + t2 + t3;

		ts1 = t1;
		ts2 = t1 + t2;
		alpha_sol_vector[0] = -1;
		alpha_sol_vector[1] = -1;			

	// if not, proceed as normal
	} else {
		// if v0 exceeds this threshold velocity, you need to start braking
		// immediately, because you will overshoot the target otherwise
		double V1 = std::sqrt(2*a_max*pf_rel);

		double ts = 0;
		if (v0 >= V1){
			alpha_sol_vector[0] = -1;
			alpha_sol_vector[1] = 1;
			T = (v0 + std::sqrt(std::max(0.0, 2*std::pow(v0, 2) - 
										 4*a_max*pf_rel))) / a_max;
			ts = 0.5*(T + v0/a_max);
		} else {
			alpha_sol_vector[0] = 1;
			alpha_sol_vector[1] = -1;
			T = (-v0 + std::sqrt(std::max(0.0, 2*std::pow(v0, 2) + 
										 4*a_max*pf_rel))) / a_max;
			ts = 0.5*(T - v0/a_max);
		}

		// check velocity limit
		// if we accelerate beyond the velocity limit, increase the coasting \
		// time
		if (a_max*(T - ts) > v_max){
			double delta_t = T - ts - v_max/a_max;
			ts1 = ts - delta_t;
			ts2 = ts + a_max/v_max*std::pow(delta_t, 2) + delta_t;
			T = ts2 + v_max/a_max;
		// if the velocitiy limit is not exceeded, keep the solution
		} else {
			ts1 = ts;
			ts2 = ts;
		}
	}

	// write solution
	if (pf - p0 < 0){
		alpha_sol_vector[0] = -alpha_sol_vector[0];
		alpha_sol_vector[1] = -alpha_sol_vector[1];
	}

	t_sol_vector[0] = ts1;
	t_sol_vector[1] = ts2 - ts1;
	t_sol_vector[2] = T - ts2;
}