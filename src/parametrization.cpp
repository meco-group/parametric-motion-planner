#include <cmath>
#include <iostream>

#include "parametrization.hpp"

Parametrization::Parametrization(CorridorSequence const &corridor_sequence,
								 Parameters const &params)
    : corridor_sequence_(corridor_sequence),
	  params_(params),
      max_nb_corridors_(corridor_sequence.MaxNbCorridors()){

	alpha_x_ = std::vector<double>(max_nb_corridors_); 
	alpha_y_ = std::vector<double>(max_nb_corridors_);
	waypoints_ = std::vector<Point2D<double>>(max_nb_corridors_); 
	waypoint_offsets_ = std::vector<Point2D<double>>(max_nb_corridors_);
	movable_waypoints_ = std::vector<bool>(max_nb_corridors_, false);

	t_x_ = std::vector<std::vector<double>>(max_nb_corridors_); 
	t_y_ = std::vector<std::vector<double>>(max_nb_corridors_);
    for (int i = 0; i < max_nb_corridors_; i++){
        t_x_[i] = std::vector<double>(3);
        t_y_[i] = std::vector<double>(3);
    };

	candidate_waypoints_ = std::vector<Point2D<double>>(4);
	candidate_valid_ = std::vector<bool>(4);
	candidate_alpha_x_ = std::vector<double>(4);
	candidate_alpha_y_ = std::vector<double>(4);
}

void Parametrization::UpdateParametrization(){
	// take care of the first and the last waypoint (start and dest)
	waypoints_[0] = corridor_sequence_.GetStart();
	waypoints_[max_nb_corridors_-1] = corridor_sequence_.GetDest();

	alpha_x_[0] = 1;
	alpha_y_[0] = 1;
	alpha_x_[max_nb_corridors_-1] = 1;
	alpha_y_[max_nb_corridors_-1] = 1;

	movable_waypoints_[0] = false;
	movable_waypoints_[max_nb_corridors_-1] = false;

	// Compute the rest of the waypoints
	for (int i = 1; i < corridor_sequence_.NbCorridors()-1; i++){
		ComputeSingleWaypoint(i);
	}

	// perform second sweep
	for (int i = corridor_sequence_.NbCorridors()-2; i > 0; i--){
		ComputeSingleWaypoint(i, true);
	}

	// Update the parametrization based on line of sights
	for (int i = 1; i < corridor_sequence_.NbCorridors()-1; i++){
		UpdateParametrizationWithLineOfSight(i);
	}
};

std::ostream& operator<<(std::ostream &out, 
						 Parametrization const &parametrization){
	Point2D<double> waypoint;
	Point2D<double> waypoint_offset;

	out << "Parametrization (" << parametrization.NbCorridors() << "/" 
		<< parametrization.MaxNbCorridors() << ")" << std::endl;

	for (int i = 0; i < parametrization.NbCorridors() + 1; i++){
		waypoint = parametrization.GetWaypoint(i);
		out << "Waypoint " << i << std::endl;
		out << "\tPosition: \t" << waypoint << std::endl;
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
	if (waypoint_idx < 1 || waypoint_idx >= corridor_sequence_.NbCorridors() - 1){
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
				std::abs(candidate_waypoints_[i].x() - 
						 overlap_.Xmin() + width_offset),
				std::abs(candidate_waypoints_[i].x() - 
						 overlap_.Xmax() - width_offset));
			vertical_distance = std::min(
				std::abs(candidate_waypoints_[i].y() - 
						 overlap_.Ymin() + height_offset),
				std::abs(candidate_waypoints_[i].y() - 
						 overlap_.Ymax() - height_offset));
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

	transformed_center.SetX(transformed_center.x()*std::cos(-angle) - 
							transformed_center.y()*std::sin(-angle));
	transformed_center.SetY(transformed_center.x()*std::sin(-angle) +
							transformed_center.y()*std::cos(-angle));
	
	// rotate line of sight 90 degrees
	double temp;
	if (transformed_center.y() < 0.0){ // left turn
		temp = line_of_sight.x();
		line_of_sight.SetX(-line_of_sight.y());
		line_of_sight.SetY(temp);
	} else { // right turn
		temp = line_of_sight.x();
		line_of_sight.SetX(line_of_sight.y());
		line_of_sight.SetY(-temp);
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
				waypoint_offsets_[waypoint_idx].SetX(overlap_.Xmax() - 
					params_.GetVehWidth()/2 - params_.GetMargin() - 
					curr_point_.x());
			} else {
				waypoint_offsets_[waypoint_idx].SetX(curr_point_.x() - 
					overlap_.Xmin() - params_.GetVehWidth()/2 - 
					params_.GetMargin());
			}

			if (alpha_y_[waypoint_idx] > 0){
				waypoint_offsets_[waypoint_idx].SetY(overlap_.Ymax() - 
					params_.GetVehHeight()/2 - params_.GetMargin() - 
					curr_point_.y());
			} else {
				waypoint_offsets_[waypoint_idx].SetY(curr_point_.y() - 
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
				waypoint_offsets_[waypoint_idx].SetX(curr_point_.x() -
					overlap_.Xmax() + params_.GetVehWidth()/2 + 
					params_.GetMargin());
			} else {
				waypoint_offsets_[waypoint_idx].SetX(overlap_.Xmin() - 
					curr_point_.x() + params_.GetVehWidth()/2 + 
					params_.GetMargin());
			}

			if (alpha_y_[waypoint_idx] < 0){
				waypoint_offsets_[waypoint_idx].SetY(curr_point_.y() - 
					overlap_.Ymax() + params_.GetVehHeight()/2 +
					params_.GetMargin());
			} else {
				waypoint_offsets_[waypoint_idx].SetY(overlap_.Ymin() - 
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
					std::max(waypoint_offsets_[waypoint_idx].x(),
							 waypoint_offsets_[waypoint_idx].y()) < 5.1e-3){
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