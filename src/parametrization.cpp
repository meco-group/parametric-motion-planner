#include <cmath>
#include <iostream>
#include <algorithm>
#include <casadi/casadi.hpp>

#include "parametrization.hpp"

using namespace casadi;

Parametrization::Parametrization(CorridorSequence const &corridor_sequence,
								 Parameters const &params)
    : corridor_sequence_(corridor_sequence),
	  params_(params),
      max_nb_corridors_(corridor_sequence.MaxNbCorridors()){

	// true parameter variables
	alpha_x_ = std::vector<double>(max_nb_corridors_); 
	alpha_y_ = std::vector<double>(max_nb_corridors_);
	waypoints_ = std::vector<Point2D<double>>(max_nb_corridors_); 
	max_waypoint_offsets_ = std::vector<Point2D<double>>(max_nb_corridors_);
	movable_waypoints_ = std::vector<bool>(max_nb_corridors_, false);
	waypoint_locations_ = std::vector<WaypointLocation>(max_nb_corridors_);

	// mx objects used in the optimization
	alpha_x_mx_ = MX::zeros(max_nb_corridors_ + 1);
	alpha_y_mx_ = MX::zeros(max_nb_corridors_ + 1);
	waypoints_mx_ = std::vector<Point2D<MX>>(max_nb_corridors_ + 1);

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

	intermediate_positions_ = std::vector<Point2D<MX>>(3);
	intermediate_velocities_ = std::vector<Point2D<MX>>(3);
}

void Parametrization::UpdateParametrization(const UpdateToken&){
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

	// Update accelerations at start and dest
	if (waypoints_[1].x() - waypoints_[0].x() > 0){ alpha_x_[0] = 1;} 
	else {alpha_x_[0] = -1;}
	if (waypoints_[1].y() - waypoints_[0].y() > 0){ alpha_y_[0] = 1;} 
	else {alpha_y_[0] = -1;}
	if (waypoints_[corridor_sequence_.NbCorridors()].x() - 
		waypoints_[corridor_sequence_.NbCorridors() - 1].x() > 0){
		alpha_x_[corridor_sequence_.NbCorridors()] = 1;
	} else {alpha_x_[corridor_sequence_.NbCorridors()] = -1;}
	if (waypoints_[corridor_sequence_.NbCorridors()].y() - 
		waypoints_[corridor_sequence_.NbCorridors() - 1].y() > 0){
		alpha_y_[corridor_sequence_.NbCorridors()] = 1;
	} else {alpha_y_[corridor_sequence_.NbCorridors()] = -1;}

	// perform second sweep
	for (int i = 1; i < corridor_sequence_.NbCorridors(); i++){
		ComputeSingleWaypoint(i, true);
	}

	// Update the parametrization based on line of sights
	nb_movable_waypoints_ = 0;
	for (int i = 1; i < corridor_sequence_.NbCorridors()-1; i++){
		UpdateParametrizationWithLineOfSight(i);
		if (movable_waypoints_[i]){ nb_movable_waypoints_++;}
	}
};

void Parametrization::OptimizeParametrization(const UpdateToken&){
	Opti opti = Opti();

	////////////////////////////////////////////
	/// Definition of optimization variables ///
	////////////////////////////////////////////
	//	time durations	
	MX t_x = opti.variable(3, corridor_sequence_.NbCorridors());
	MX t_y = opti.variable(3, corridor_sequence_.NbCorridors());
	
	//	basic box constraints
	opti.subject_to(0 <= (t_x <= 100));
	opti.subject_to(0 <= (t_y <= 100));

	// Initialize parametrization MX objects and
	// Create movable waypoints
	MX offsets = opti.variable(2, nb_movable_waypoints_);
	int offset_idx = 0;
	for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
		alpha_x_mx_(i) = alpha_x_[i]; 
		alpha_y_mx_(i) = alpha_y_[i];
		waypoints_mx_[i].CopyValues(waypoints_[i]);
	
		// if the waypoint is movable, make it so
		if (movable_waypoints_[i]){
			// Check horizontal moving range
			if (max_waypoint_offsets_[i].x() > 0){
				opti.subject_to(0 <= (offsets(0, offset_idx) <= 
									 max_waypoint_offsets_[i].x()));
			} else {
				opti.subject_to(-max_waypoint_offsets_[i].x() <= 
								(offsets(0, offset_idx) <= 0));
			}

			// Check vertical moving range
			if (max_waypoint_offsets_[i].y() > 0){
				opti.subject_to(0 <= (offsets(1, offset_idx) <= 
									 max_waypoint_offsets_[i].y()));
			} else {
				opti.subject_to(-max_waypoint_offsets_[i].y() <= 
								(offsets(1, offset_idx) <= 0));
			}

			// Update waypoint
			waypoints_mx_[i].SetX(waypoints_mx_[i].x() + 
								  alpha_x_mx_(i)*max_waypoint_offsets_[i].x());
			waypoints_mx_[i].SetY(waypoints_mx_[i].y() + 
								  alpha_y_mx_(i)*max_waypoint_offsets_[i].y());

			offset_idx++;
		}
	}

	//  corridor entry velocities
	MX v_x = opti.variable(corridor_sequence_.NbCorridors() + 1);
	MX v_y = opti.variable(corridor_sequence_.NbCorridors() + 1);
	MX curr_v_x = corridor_sequence_.GetStartVel().x();
	MX curr_v_y = corridor_sequence_.GetStartVel().y();

	// free initial acceleration
	MX alpha_0 = opti.variable();
	MX s_0 = opti.variable();
	opti.subject_to(-1 - pow(s_0, 2) <= (alpha_0 <= 1 + pow(s_0, 2)));

	// free final acceleration
	MX alpha_N = opti.variable();
	MX s_N = opti.variable();
	opti.subject_to(-1 - pow(s_N, 2) <= (alpha_N <= 1 + pow(s_N, 2)));
	
	MX obj = 1.0e3*pow(s_0, 2) + pow(s_N, 2);
	if (RELAX_INITIAL_VELOCITY_){
		MX s_x = opti.variable(); MX s_y = opti.variable();
		curr_v_x += s_x; curr_v_y += s_y;
		obj += 1.0e2*(pow(s_x, 2) + pow(s_y, 2));
	}
	
	
	////////////////////////////////
	/// Definiton of constraints ///
	////////////////////////////////
	
	// constraint to prevent initial overshooting
	// TODO

	double width_offset = params_.GetWidthOffset();
	double height_offset = params_.GetHeightOffset();
	Point2D<MX> curr_waypoint;
	Point2D<MX> next_waypoint;
	Corridor curr_corridor;
	double position_tolerance = 1.0e-5;
	for (int w = 0; w < corridor_sequence_.NbCorridors(); w++){
		curr_waypoint = waypoints_mx_[w];
		next_waypoint = waypoints_mx_[w];
		corridor_sequence_.GetCorridor(w, curr_corridor);

		// add gap-closing constraints on velocity
		opti.subject_to(v_x(w) == curr_v_x);
		opti.subject_to(v_y(w) == curr_v_y);

		// get some positions of relevance
		IntegrateOverCorridor(curr_waypoint, Point2D<MX>(curr_v_x, curr_v_y), 
							  w, t_x(Slice(), w), t_y(Slice(), w));
		
		// add gap-closing constraints on position 
		opti.subject_to(next_waypoint.x() - position_tolerance <=
					   (intermediate_positions_[2].x() <=
						next_waypoint.x() + position_tolerance));
		opti.subject_to(next_waypoint.y() - position_tolerance <=
					   (intermediate_positions_[2].y() <=
						next_waypoint.y() + position_tolerance));

		// constraint equal time
		opti.subject_to(t_x(0, w) + t_x(1, w) + t_x(2, w) == 
						t_y(0, w) + t_y(1, w) + t_y(2, w));

		// constrain positions and velocities
		for (int i = 0; i < 2; i++){
			opti.subject_to(curr_corridor.Xmin() + width_offset <= 
						   (intermediate_positions_[i].x() <= 
						   	curr_corridor.Xmax() - width_offset));
			opti.subject_to(curr_corridor.Ymin()  + height_offset <=
						   (intermediate_positions_[i].y() <= 
						   curr_corridor.Ymax() - height_offset));
			opti.subject_to(-params_.GetVmax() <= 
				(intermediate_velocities_[i].x() <= params_.GetVmax()));
			opti.subject_to(-params_.GetVmax() <= 
				(intermediate_velocities_[i].y() <= params_.GetVmax()));
		}

		// integrate the velocity over this corridor
		curr_v_x = intermediate_velocities_[2].x();
		curr_v_y = intermediate_velocities_[2].y();
		
		// constraint exit velocity
		// TODO (is this needed?)

		// update objective term
		obj += t_x(0, w) + t_x(1, w) + t_x(2, w);
	}

	// Add terminal velocity constraint
	double velocity_relaxation_tolerance = 1.0e-5;
	opti.subject_to(-velocity_relaxation_tolerance <= 
					(curr_v_x <= velocity_relaxation_tolerance));
	opti.subject_to(-velocity_relaxation_tolerance <=
					(curr_v_y <= velocity_relaxation_tolerance));


	//////////////////////////////////
	/// Finish problem formulation ///
	//////////////////////////////////
	opti.minimize(obj);
	opti.solver("ipopt", {}, {});

	OptiSol sol = opti.solve();

	////////////////////////
	/// Extract solution ///
	////////////////////////
	for (int i = 0; i < corridor_sequence_.NbCorridors() + 1; i++){
		alpha_x_sol_[i] = double(sol.value(alpha_x_mx_(i)));
		alpha_y_sol_[i] = double(sol.value(alpha_y_mx_(i)));
		waypoints_sol_[i].SetX(double(sol.value(waypoints_mx_[i].x())));
		waypoints_sol_[i].SetY(double(sol.value(waypoints_mx_[i].y())));
		waypoint_velocities_sol_[i].SetX(double(sol.value(v_x(i))));
		waypoint_velocities_sol_[i].SetY(double(sol.value(v_y(i))));

		if (i < corridor_sequence_.NbCorridors()){
			for (int j = 0; j < 3; j++){
				t_x_sol_[i][j] = double(sol.value(t_x(j, i)));
				t_y_sol_[i][j] = double(sol.value(t_y(j, i)));
			}
		}
	}
};

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

std::vector<Point2D<double>>& Parametrization::GetWaypointsSol(){
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
	double width_offset = params_.GetVehWidth()/2 + params_.GetMargin();
	double height_offset = params_.GetVehHeight()/2 + params_.GetMargin();

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
	double horizontal_distance;
	double vertical_distance;
	for (int i = 0; i < candidate_waypoints_.size(); i++){
		if (candidate_valid_[i]){
			horizontal_distance = std::min(
				std::min(
					std::abs(candidate_waypoints_[i].x() - 
							 curr_corridor_.Xmin() - width_offset),
					std::abs(curr_corridor_.Xmax() - 
							 candidate_waypoints_[i].x() - width_offset)),
				std::min(
					std::abs(candidate_waypoints_[i].x() - 
							 next_corridor_.Xmin() - width_offset),
					std::abs(next_corridor_.Xmax() - 
							 candidate_waypoints_[i].x() - width_offset)));
			vertical_distance = std::min(
				std::min(
					std::abs(candidate_waypoints_[i].y() - 
							 curr_corridor_.Ymin() - height_offset),
					std::abs(curr_corridor_.Ymax() - 
							 candidate_waypoints_[i].y() - height_offset)),
				std::min(
					std::abs(candidate_waypoints_[i].y() - 
							 next_corridor_.Ymin() - height_offset),
					std::abs(next_corridor_.Ymax() - 
							 candidate_waypoints_[i].y() - height_offset)));
			if (horizontal_distance > tolerance &&
				vertical_distance > tolerance){
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
		next_point_.SetX(overlap_.Xmin() + overlap_.Width()/2);
		next_point_.SetY(overlap_.Ymin() + overlap_.Height()/2);
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
		line_of_sight.Rotate(M_PI/2);
	} else { // right turn
		line_of_sight.Rotate(-M_PI/2);
	}

	double norm = std::sqrt(line_of_sight.x()*line_of_sight.x() + 
							line_of_sight.y()*line_of_sight.y());
	double length = 20;
	line_of_sight.SetX(line_of_sight.x()*length/norm);
	line_of_sight.SetY(line_of_sight.y()*length/norm);
	line_of_sight += transformed_center;

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
			waypoint_idx >= corridor_sequence_.NbCorridors() - 1){
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

	double t_x = x_margin/line_of_sight.x();
	double t_y = y_margin/line_of_sight.y();
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

	t_x = x_margin/line_of_sight.x();
	t_y = y_margin/line_of_sight.y();
	double t2 = std::min(t_x, t_y);

	return t1 + t2 >= 1.0;
}

void Parametrization::IntegrateOverCorridor(Point2D<MX> const &start, 
											Point2D<MX> const &start_vel, 
											int corridor_idx,
											MX const &t_x, MX const &t_y){
	Point2D<MX> t;
	Point2D<MX> accel;
	
	// First arc
	t.SetX(t_x(0)); t.SetY(t_y(0));
	accel.SetX(alpha_x_[corridor_idx]); accel.SetY(alpha_y_[corridor_idx]);
	intermediate_positions_[0].CopyValues(start + t*start_vel + 
										  t*t*accel*0.5);
	intermediate_velocities_[0].CopyValues(start_vel + t*accel);

	// Second arc
	t.SetX(t_x(1)); t.SetY(t_y(1));
	accel.SetX(0); accel.SetY(0);
	intermediate_positions_[1].CopyValues(intermediate_positions_[0] + 
										  t*intermediate_velocities_[0] + 
										  t*t*accel*0.5);
	intermediate_velocities_[1].CopyValues(start_vel + t*accel);

	// Third arc
	t.SetX(t_x(2)); t.SetY(t_y(2));
	accel.SetX(alpha_x_[corridor_idx+1]); accel.SetY(alpha_y_[corridor_idx+1]);
	intermediate_positions_[2].CopyValues(intermediate_positions_[1] + 
										  t*intermediate_velocities_[1] + 
										  t*t*accel*0.5);
	intermediate_velocities_[2].CopyValues(start_vel + t*accel);
}