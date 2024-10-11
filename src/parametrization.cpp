#include <cmath>
#include <iostream>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>

#include "core/parametrization.hpp"
#include "core/trajectory.hpp"

using namespace casadi;
using json = nlohmann::json;

Parametrization::Parametrization(CorridorSequence const &corridor_sequence,
								 Parameters const &params)
    : corridor_sequence_(corridor_sequence),
	  params_(params),
      max_nb_corridors_(corridor_sequence.MaxNbCorridors()),
	  opti_(Opti("nlp"))
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

	InitializeParabolicSegmentConstraintFunction();

	// scratch space
	candidate_waypoints_ = std::vector<Point2D<double>>(4);
	candidate_valid_ = std::vector<bool>(4);
	candidate_alpha_x_ = std::vector<double>(4);
	candidate_alpha_y_ = std::vector<double>(4);

	intermediate_positions_ = std::vector<Point2D<MX>>(3 + 2*nb_fine_grid_samples_);
	intermediate_velocities_ = std::vector<Point2D<MX>>(3);
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

	// Update accelerations at start and dest
	alpha_x_[0] = sign(waypoints_[1].x() - waypoints_[0].x());
	alpha_y_[0] = sign(waypoints_[1].y() - waypoints_[0].y());
	int final_idx = corridor_sequence_.NbCorridors();
	alpha_x_[final_idx] = -sign(waypoints_[final_idx].x() - 
							    waypoints_[final_idx - 1].x());
	alpha_y_[final_idx] = -sign(waypoints_[final_idx].y() -
							    waypoints_[final_idx - 1].y());

	// Update the parametrization based on line of sights
	nb_movable_waypoints_ = 0;
	for (int i = 1; i < corridor_sequence_.NbCorridors(); i++){
		UpdateParametrizationWithLineOfSight(i);
		if (movable_waypoints_[i]){ nb_movable_waypoints_++;}
	}

	// Update version
	latest_sequence_version_ = corridor_sequence_.GetVersion();
};

void Parametrization::OptimizeParametrization(const UpdateToken&,
											  Dict const &opts_casadi,
											  Dict const &opts_solver){		
	// prepare initial guess
	std::cout << "initializing" << std::endl;
	InitializeOptimization();
	// ShowInitialization();
	std::cout << "done" << std::endl;

	// reset mx containers
	alpha_x_mx_ = MX(max_nb_corridors_ + 1, 1);
	alpha_y_mx_ = MX(max_nb_corridors_ + 1, 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

	// start the optimization
	opti_ = Opti();

	////////////////////////////////////////////
	/// Definition of opti_mization variables ///
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
		if (movable_waypoints_[i]){
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

	//  corridor entry velocities
	v_x_ = opti_.variable(corridor_sequence_.NbCorridors() + 1);
	v_y_ = opti_.variable(corridor_sequence_.NbCorridors() + 1);
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
		opti_.subject_to(-1 - pow(s_0, 2) <= 
						(alpha_x_mx_(0) <= 1 + pow(s_0, 2)));
	} else {
		alpha_y_mx_(0) = opti_.variable();
		opti_.set_initial(alpha_y_mx_(0), alpha_0_init_);
		opti_.subject_to(-1 - pow(s_0, 2) <= 
						(alpha_y_mx_(0) <= 1 + pow(s_0, 2)));	
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
		opti_.subject_to(-1 - pow(s_N, 2) <= 
						(alpha_x_mx_(corridor_sequence_.NbCorridors()) <= 
						 1 + pow(s_N, 2)));
	} else {
		alpha_y_mx_(corridor_sequence_.NbCorridors()) = opti_.variable();
		opti_.set_initial(alpha_y_mx_(corridor_sequence_.NbCorridors()), 
						 alpha_f_init_);
		opti_.subject_to(-1 - pow(s_N, 2) <= 
						(alpha_y_mx_(corridor_sequence_.NbCorridors()) <= 
						 1 + pow(s_N, 2)));
	}

	// add free initial velocity
	MX obj = 1.0e3*pow(s_0, 2) + 1.0e3*pow(s_N, 2);
	if (RELAX_INITIAL_VELOCITY_){
		MX s_x = opti_.variable(); MX s_y = opti_.variable();
		curr_v_x += s_x; curr_v_y += s_y;
		obj += 1.0e2*(pow(s_x, 2) + pow(s_y, 2));

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
							  alpha_x_mx_(w+1), alpha_y_mx_(w+1));
		
		// add gap-closing constraints on position s
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
			opti_.subject_to(curr_corridor.Xmin() + width_offset <= 
						   (intermediate_positions_[i].x() <= 
						   	curr_corridor.Xmax() - width_offset));
			opti_.subject_to(curr_corridor.Ymin()  + height_offset <=
						   (intermediate_positions_[i].y() <= 
						   curr_corridor.Ymax() - height_offset));
		}
		for (int i : {1, 2}){
			opti_.subject_to(-params_.GetVmax() <= 
				(intermediate_velocities_[i].x() <= params_.GetVmax()));
			opti_.subject_to(-params_.GetVmax() <= 
				(intermediate_velocities_[i].y() <= params_.GetVmax()));
		}

		// integrate the velocity over this corridor
		curr_v_x = intermediate_velocities_[2].x();
		curr_v_y = intermediate_velocities_[2].y();
		
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
	opti_.solver("ipopt", opts_casadi, opts_solver);

	////////////////////////
	/// Extract solution ///
	////////////////////////
	Solve();
};

void Parametrization::AddOvershootingConstraints(std::set<int> &add_list){
	// set the previous solution as initial guess
	opti_.set_initial(opti_.x(), sol_.value().value(opti_.x()));
	
	MX t_extreme;
	MX p_extreme;
	double lb;
	double ub;
	p_extremes_ = {};
	for (int w : add_list){
		if (std::abs(waypoints_[w+1].x() - waypoints_[w].x()) >
				std::abs(waypoints_[w+1].y() - waypoints_[w].y())){
			lb = corridor_sequence_.GetCorridor(w).Ymin() +
				 params_.GetHeightOffset();
			ub = corridor_sequence_.GetCorridor(w).Ymax() -
				 params_.GetHeightOffset();

			// add constraint on first arc
			t_extreme = -v_y_(w) / (alpha_y_mx_(w)*params_.GetAmax());
			p_extreme = waypoints_mx_[w].y() + v_y_(w)*t_extreme + 
						0.5*alpha_y_mx_(w)*params_.GetAmax()*pow(t_extreme, 2);
			opti_.subject_to(lb <= (p_extreme <= ub));

			// add constraint on second arc
			t_extreme = v_y_(w+1) / (alpha_y_mx_(w+1)*params_.GetAmax());
			p_extreme = waypoints_mx_[w+1].y() - v_y_(w+1)*t_extreme + 
						0.5*alpha_y_mx_(w+1)*params_.GetAmax()*pow(t_extreme, 2);
			opti_.subject_to(lb <= (p_extreme <= ub));

		} else {
			lb = corridor_sequence_.GetCorridor(w).Xmin() +
				 params_.GetWidthOffset();
			ub = corridor_sequence_.GetCorridor(w).Xmax() -
				 params_.GetWidthOffset();
			
			// add constraint on first arc
			t_extreme = -v_x_(w) / (alpha_x_mx_(w)*params_.GetAmax());
			p_extreme = waypoints_mx_[w].x() + v_x_(w)*t_extreme + 
						0.5*alpha_x_mx_(w)*params_.GetAmax()*pow(t_extreme, 2);
			opti_.subject_to(lb <= (p_extreme <= ub));
			
			// add constraint on second arc
			t_extreme = v_x_(w+1) / (alpha_x_mx_(w+1)*params_.GetAmax());
			p_extreme = waypoints_mx_[w+1].x() - v_x_(w+1)*t_extreme + 
						0.5*alpha_x_mx_(w+1)*params_.GetAmax()*pow(t_extreme, 2);
			opti_.subject_to(lb <= (p_extreme <= ub));
		}
	}

	Solve();
}

void Parametrization::OptimizeSingleArc(const UpdateToken&){
	waypoints_[0].CopyValues(corridor_sequence_.GetStart());
	// waypoints_[1].CopyValues(corridor_sequence_.GetDest());
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
	for (int i = 2; i < corridor_sequence_.NbCorridors(); i++){
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
	// j["t_x_sol"] = std::vector<double>(t_x_sol_.begin(), 
	// 								   t_x_sol_.begin() + curr_nb_corridors);
	// j["t_y_sol"] = std::vector<double>(t_y_sol_.begin(), 
	// 								   t_y_sol_.begin() + curr_nb_corridors);
	j["t_x_sol"] = t_x_sol_json;
	j["t_y_sol"] = t_y_sol_json;


	return j;
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

	// STRAIGH_LINE_OF_OVERLAPS code
	// Moet ik dat implementeren?

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
											MX const &alpha_y_next){
	Point2D<MX> t;
	Point2D<MX> accel;
	
	// First arc
	t.SetX(t_x(0)); t.SetY(t_y(0));
	accel.SetX(alpha_x); accel.SetY(alpha_y);
	accel *= params_.GetAmax();
	intermediate_positions_[0].CopyValues(start + t*start_vel + 
										  t*t*accel*0.5);
	intermediate_velocities_[0].CopyValues(start_vel + t*accel);
	
	// Second arc
	t.SetX(t_x(1)); t.SetY(t_y(1));
	accel.SetX(0); accel.SetY(0);
	intermediate_positions_[1].CopyValues(intermediate_positions_[0] + 
										  t*intermediate_velocities_[0]);
	intermediate_velocities_[1].CopyValues(intermediate_velocities_[0]);

	// Third arc
	t.SetX(t_x(2)); t.SetY(t_y(2));
	accel.SetX(alpha_x_next); accel.SetY(alpha_y_next);
	accel *= params_.GetAmax();
	intermediate_positions_[2].CopyValues(intermediate_positions_[1] + 
										  t*intermediate_velocities_[1] + 
										  t*t*accel*0.5);
	intermediate_velocities_[2].CopyValues(intermediate_velocities_[1] + 
										   t*accel);

	// some fine grid samples
	int sample_ptr = 3;
	for (int i = 1; i <= nb_fine_grid_samples_; i++){
		t.SetY(t_y(0)*i/(nb_fine_grid_samples_ + 1));
		t.SetX(t_x(0)*i/(nb_fine_grid_samples_ + 1));
		accel.SetX(alpha_x); accel.SetY(alpha_y);
		accel *= params_.GetAmax();
		intermediate_positions_[sample_ptr].CopyValues(start + t*start_vel + 
													  t*t*accel*0.5);
		sample_ptr++;

		t.SetX(t_x(2)*i/(nb_fine_grid_samples_ + 1));
		t.SetY(t_y(2)*i/(nb_fine_grid_samples_ + 1));
		accel.SetX(alpha_x_next); accel.SetY(alpha_y_next);
		accel *= params_.GetAmax();
		intermediate_positions_[sample_ptr].CopyValues(intermediate_positions_[1] + 
													  t*intermediate_velocities_[1] + 
													  t*t*accel*0.5);
		sample_ptr++;
	}
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

void Parametrization::InitializeParabolicSegmentConstraintFunction(){
	// function inputs
	MX p0 = MX::sym("p0");
	MX v0 = MX::sym("v0");
	MX alpha = MX::sym("alpha");
	MX T = MX::sym("T");
	
	// define the signmoids
	MX x = MX::sym("x");
	double lmbd = 5.0;
	Function s = Function("s", {x}, {1.0/(1.0 + exp(-lmbd*x/0.01))});
	Function s1 = Function("s1", {x}, {s(-x + 0.0)[0]});
	Function s2 = Function("s2", {x}, {s(x - 0.0)[0] * s(T - x)[0]});
	Function s3 = Function("s3", {x}, {s(x - T)[0]});

	// get the smooth extreme point of the parabolic segment
	MX t_extreme = -v0/(params_.GetAmax()*alpha);
	MX p_extreme = s1(t_extreme)[0]*p0 + 
				   s2(t_extreme)[0]*
						(p0 + v0*t_extreme + 0.5*alpha*params_.GetAmax()*pow(t_extreme, 2)) + 
				   s3(t_extreme)[0]*
				   		(p0 + v0*T + 0.5*alpha*params_.GetAmax()*pow(T, 2));
	// MX p_extreme = p0 + v0*t_extreme + 0.5*alpha*params_.GetAmax()*pow(t_extreme, 2);

	parabolic_segment_extremum_ = Function("parabolic_segment_extremum", 
											{p0, v0, alpha, T}, {p_extreme});
}

void Parametrization::ConstrainParabolicSegment(Opti &opti, MX T, MX p0,
												MX v0, MX alpha,
												double min_val, 
												double max_val, 
												double offset,
												MX &obj){
	// return;
	int approach_selector = 2;

	if (approach_selector == 0){			// constraint extremum (suboptimal)
		MX t_extreme = -v0/(params_.GetAmax()*alpha);
		MX p_extreme = p0 + v0*t_extreme + 0.5*alpha*params_.GetAmax()*pow(t_extreme, 2);

		opti.subject_to(min_val + offset <= 
						(p_extreme <= max_val - offset));

	} else if (approach_selector == 1){		// use soft extremum formulation
		MX p_extreme = parabolic_segment_extremum_({p0, v0, alpha, T})[0];
		// p_extremes_.push_back(p_extreme);

		double relaxation = 0.001;
		opti.subject_to(min_val + offset - relaxation <= 
						(p_extreme <= max_val - offset + relaxation));
	
	} else if (approach_selector == 2){		// use slack-based approach
		MX s = opti.variable();
		MX t_extreme = -v0/(params_.GetAmax()*alpha) + s;
		opti.subject_to(0 <= (t_extreme <= T));
		MX p_extreme = p0 + v0*t_extreme + 0.5*alpha*params_.GetAmax()*pow(t_extreme, 2);
		opti.subject_to(min_val + offset <= (p_extreme <= max_val - offset));
		obj += 1.0e1*T*s*s;

		p_extremes_.push_back(p_extreme);
	}
}

void Parametrization::Solve(){
	try {
		sol_ = opti_.solve();
		solver_time_ = sol_.value().stats()["t_wall_total"];
		solver_time_ *= 1000; // convert to milliseconds

		for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
			alpha_x_sol_[i] = double(sol_.value().value(alpha_x_mx_(i)));
			alpha_y_sol_[i] = double(sol_.value().value(alpha_y_mx_(i)));
			waypoints_sol_[i].SetX(double(sol_.value().value(waypoints_mx_[i].x())));
			waypoints_sol_[i].SetY(double(sol_.value().value(waypoints_mx_[i].y())));
			waypoint_velocities_sol_[i].SetX(double(sol_.value().value(v_x_(i))));
			waypoint_velocities_sol_[i].SetY(double(sol_.value().value(v_y_(i))));

			if (i < corridor_sequence_.NbCorridors()){
				for (int j = 0; j < 3; j++){
					t_x_sol_[i][j] = double(sol_.value().value(t_x_(j, i)));
					t_y_sol_[i][j] = double(sol_.value().value(t_y_(j, i)));
				}
			}
		}
		// std::cout << "p_extremes: [";
		// for (auto p : p_extremes_){
		// 	std::cout << sol_.value().value(p);
		// 	std::cout << ", ";
		// }
		// std::cout << "]" << std::endl;

	} catch (std::exception &e){
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
}


void Parametrization::InitializeOptimization(){
	double v_des;
	if (params_.GetAmax() > params_.GetVmax()){
		v_des = std::min(0.2, params_.GetVmax());
	} else {
		v_des = std::min(0.4, params_.GetVmax());
	}

	bool initialization_complete = false;
	Point2D<double> curr_vel;

	while (!initialization_complete){
		// set-up the initialization loop
		curr_vel = corridor_sequence_.GetStartVel();
		if (std::abs(curr_vel.x()) <= v_des &&
				std::abs(curr_vel.y()) <= v_des){
			waypoint_velocities_init_[0].CopyValues(curr_vel);
		} else {
			double norm = curr_vel.Norm();
			waypoint_velocities_init_[0].SetX(curr_vel.x()*v_des/norm);
			waypoint_velocities_init_[0].SetY(curr_vel.y()*v_des/norm);
		}

		// start initializing
		bool success;
		for (int w = 0; w < corridor_sequence_.NbCorridors(); w++){
			success = InitializeArc(w, v_des, curr_vel);			
			if (!success){
				// try again with lower velocity
				v_des *= 0.7;
				break;
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
	if (std::abs(p0.x() - pf.x()) < std::abs(p0.y() - pf.y())){
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

	// Compute timings in the free direction
	if (corridor_idx == 0){
		double t_accel = std::sqrt(2*std::abs(p0_fd - pf_fd)/a_max);
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
	succesfull_initialization = succesfull_initialization && 
								!negative_value_detected;

	
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
	}

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