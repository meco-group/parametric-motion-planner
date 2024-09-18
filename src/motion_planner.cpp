#include <iostream>
#include <casadi/casadi.hpp>

#include "motion_planner.hpp"
#include "trajectory.hpp"

using namespace casadi;

MotionPlanner::MotionPlanner(){
    environment_ = Environment();
    params_ = new Parameters();
    method_ = ARENA;

    opts_solver_["print_level"] = 0;

    InitializeRK4();
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

    // Prepare looping over corridors
    MX obj = 0;
    int k_offset = 0;
    std::vector<Point2D<MX>> corners(4);
    Corridor current_corridor;
    std::vector<Point2D<double>> initialization_waypoints = 
        helper_.GetCorridorOverlapCenters(corridor_sequence_, start_, dest_);
    double initialization_distance = 0;

    // Start looping over corridors
    for (int s = 0; s < corridor_sequence_.NbCorridors(); s++){
        obj += tt(s);
        current_corridor = corridor_sequence_.GetCorridor(s);

        // initialize the time of the corridor
        initialization_distance = initialization_waypoints[s].Distance(
            initialization_waypoints[s+1]);
        opti.set_initial(tt(s), initialization_distance/params_->GetVmax());

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
            corners[0].SetValues(xx(0, k) - params_->GetVehWidth()/2.0, 
                                 xx(1, k) - params_->GetVehHeight()/2.0);
            corners[1].SetValues(xx(0, k) + params_->GetVehWidth()/2.0,
                                 xx(1, k) - params_->GetVehHeight()/2.0);
            corners[2].SetValues(xx(0, k) + params_->GetVehWidth()/2.0,
                                 xx(1, k) + params_->GetVehHeight()/2.0);
            corners[3].SetValues(xx(0, k) - params_->GetVehWidth()/2.0,
                                 xx(1, k) + params_->GetVehHeight()/2.0);
            for (Point2D<MX> corner : corners){
                opti.subject_to(current_corridor.Xmin() <= 
                        (corner.x() <= current_corridor.Xmax()));
                opti.subject_to(current_corridor.Ymin() <= 
                        (corner.y() <= current_corridor.Ymax()));
            }

            // Add max velocity constraint
            opti.subject_to(-params_->GetVmax() <= 
                            (xx(Slice(2,4), k) <= params_->GetVmax()));

            // Add initial guess
            opti.set_initial(xx(0, k), 
                initialization_waypoints[s].x() + 
                (k - k_offset)*(initialization_waypoints[s+1].x() - 
                initialization_waypoints[s].x())/nb_points_per_corridor);
        }

        // the final point of a corridor should also be enforced to be 
        // within the next corridor to prevent corner cutting (if there 
        // exists a next corridor)
        if (s < corridor_sequence_.NbCorridors() - 1){
            int k = k_offset + nb_points_per_corridor;
            current_corridor = corridor_sequence_.GetCorridor(s+1);
            corners[0].SetValues(xx(0, k) - params_->GetVehWidth()/2, 
                                 xx(1, k) - params_->GetVehHeight()/2);
            corners[1].SetValues(xx(0, k) + params_->GetVehWidth()/2,
                                 xx(1, k) - params_->GetVehHeight()/2);
            corners[2].SetValues(xx(0, k) + params_->GetVehWidth()/2,
                                 xx(1, k) + params_->GetVehHeight()/2);
            corners[3].SetValues(xx(0, k) - params_->GetVehWidth()/2,
                                 xx(1, k) + params_->GetVehHeight()/2);
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
    OptiSol sol = opti.solve();
    
    // Extract solution
    DM xx_sol = sol.value(xx);
    DM uu_sol = sol.value(uu);
    DM tt_sol = sol.value(tt);

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

    std::cout << "t: " << t << std::endl;

    // Construct trajectory
    last_solution_.Update(xx_sol, uu_sol, t);

    std::cout << "Solution obtained:" << std::endl;
    std::cout << last_solution_ << std::endl;

}

void MotionPlanner::PlanARENA(){
    std::cout << "Planning using ARENA method" << std::endl;
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