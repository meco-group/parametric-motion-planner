#include <iostream>
#include <casadi/casadi.hpp>

#include "motion_planner.hpp"

using namespace casadi;

MotionPlanner::MotionPlanner(){
    environment_ = Environment();
    params_ = new Parameters();
    method_ = ARENA;
}

void MotionPlanner::UpdateCorridorSequence(){
    if (!environment_.isValidPosition(start_) || 
        !environment_.isValidPosition(dest_)){
        throw InvalidPositionInEnvironmentException("Invalid starting position or destination");
    }
    environment_.GetCorridorSequence(start_, dest_, params_->GetVehWidth(), 
                                     params_->GetVehHeight(), 
                                     corridor_sequence_);
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

    switch(method_){
        case P2P:
            return PlanP2P();
        case OCP:
            return PlanOCP();
        case ARENA:
            return PlanARENA();
        default:
            std::cout << "Invalid method selected" << std::endl;
    }
}

void MotionPlanner::Plan(const Point2D<double> &start, 
                         const Point2D<double> &dest, 
                         const Point2D<double> &start_vel){
    SetStart(start);
    SetDest(dest);
    SetStartVel(start_vel);
    Plan();
}

void MotionPlanner::PlanP2P(){
    std::cout << "Planning using P2P method" << std::endl;
}

void MotionPlanner::PlanOCP(){
    std::cout << "Planning using OCP method" << std::endl;

    // Update the corridor sequence
    UpdateCorridorSequence();
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
    opti.subject_to(-params_->GetAmax() <= uu <= params_->GetAmax());
    opti.subject_to(tt > 0);

    // Loop over corridors
    MX obj = 0;
    int k_offset = 0;
    std::vector<Point2D<MX>> corners(4);
    Corridor current_corridor;
    for (int s = 0; s < corridor_sequence_.NbCorridors(); s++){
        obj += tt(s);
        current_corridor = corridor_sequence_.GetCorridor(s);

        // loop over time-steps
        k_offset = s * nb_points_per_corridor;
        for (int k = k_offset; k < k_offset + nb_points_per_corridor; k++){
            // add dynamics
            rk4_arguments_[0] = xx(Slice(), k);
            rk4_arguments_[1] = uu(Slice(), k);
            rk4_outputs_ = rk4_(rk4_arguments_);
            opti.subject_to(xx(Slice(), k + 1) == rk4_outputs_[0]);
            
            // enforce corner points to be inside the current corridor
            corners[0].SetValues(xx(0, k) - params_->GetVehWidth()/2, 
                                 xx(1, k) - params_->GetVehHeight()/2);
            corners[1].SetValues(xx(0, k) + params_->GetVehWidth()/2,
                                 xx(1, k) - params_->GetVehHeight()/2);
            corners[2].SetValues(xx(0, k) + params_->GetVehWidth()/2,
                                 xx(1, k) + params_->GetVehHeight()/2);
            corners[3].SetValues(xx(0, k) - params_->GetVehWidth()/2,
                                 xx(1, k) + params_->GetVehHeight()/2);
            for (Point2D<MX> corner in corners){
                opti.subject_to()

        }
    }


}

void MotionPlanner::PlanARENA(){
    std::cout << "Planning using ARENA method" << std::endl;
}