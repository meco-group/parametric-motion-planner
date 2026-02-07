#include "core/ocp_solver.hpp"
#include "core/corridor.hpp"
#include "core/helper_types.hpp"

OCPSolver::OCPSolver(CorridorSequence const &CorridorSequence,
                     Parameters const &params) :
        corridor_sequence_(CorridorSequence),
        params_(params),
        max_nb_corridors_(CorridorSequence.MaxNbCorridors()){};

void OCPSolver::PrepareOptiInstances(const UpdateToken& token,
                                     std::string& solver_name_,
                                     casadi::Dict& opts_casadi, 
                                     casadi::Dict& opts_solver){
    std::cout << "preparing OCP opti instances..." << std::endl;
    for (int i = 1; i < 0*7; i++){
        PrepareSingleOptiInstance(i, solver_name_, opts_casadi, opts_solver);
    }
    std::cout << "\t\tDone!" << std::endl;
}

void OCPSolver::PrepareSingleOptiInstance(int nbCorridors,
                                          std::string& solver_name,
                                          casadi::Dict& opts_casadi,
                                          casadi::Dict& opts_solver){
    int n = nbCorridors;
    int N = n * nb_points_per_corridor_;

    /////////////////////
    /// Construct OCP ///
    /////////////////////

    Opti opti = Opti(); 

    // Define parameters
    MX vmax_p = opti.parameter();
    MX amax_p = opti.parameter();
    MX veh_width_p = opti.parameter();
    MX veh_height_p = opti.parameter();
    MX margin_p = opti.parameter();
    MX start = opti.parameter(2, 1);
    MX dest = opti.parameter(2, 1);
    MX start_vel = opti.parameter(2, 1);
    MX corridor_p = opti.parameter(4, n);

    // Define variables
    std::vector<MX> xx_MX(N+1);
    std::vector<MX> tt_MX(N);
    std::vector<MX> uu_MX(N);
    for (int i = 0; i < N; i++){
        xx_MX[i] = opti.variable(4, 1);
        tt_MX[i] = opti.variable(1, 1);
        uu_MX[i] = opti.variable(2, 1);
    }
    xx_MX[N] = opti.variable(4);
    MX xx = horzcat(xx_MX);
    MX tt = horzcat(tt_MX);
    MX uu = horzcat(uu_MX);

    // Prepare looping over corridors
    MX obj = 0;
    int k_offset = 0;
    std::vector<Point2D<MX>> corners(4);
    Corridor_MX current_corridor;
    MX width_offset = veh_width_p/2.0 + margin_p;
    MX height_offset = veh_width_p/2.0 + margin_p;
    MX dt;
    // Start looping over corridors
    for (int s = 0; s < n; s++){
        obj += tt(s*nb_points_per_corridor_);
        // current_corridor = corridor_sequence_.GetCorridor(s);
        current_corridor.SetXmin(corridor_p(0, s));
        current_corridor.SetXmax(corridor_p(1, s));
        current_corridor.SetYmin(corridor_p(2, s));
        current_corridor.SetYmax(corridor_p(3, s));

        // loop over time-steps
        k_offset = s * nb_points_per_corridor_;
        for (int k = k_offset; k < k_offset + nb_points_per_corridor_; k++){
            // add dynamics
            dt = tt(k)/nb_points_per_corridor_;
            opti.subject_to(xx(0, k+1) == xx(0, k) + xx(2, k)*dt + 
                                          0.5*uu(0, k)*dt*dt);
            opti.subject_to(xx(1, k+1) == xx(1, k) + xx(3, k)*dt + 
                                          0.5*uu(1, k)*dt*dt);
            opti.subject_to(xx(2, k+1) == xx(2, k) + uu(0, k)*dt);
            opti.subject_to(xx(3, k+1) == xx(3, k) + uu(1, k)*dt);
            if (k < k_offset + nb_points_per_corridor_ - 1){
                opti.subject_to(tt(k+1) == tt(k));
            }

            // basic box constraints
            opti.subject_to(-amax_p <= (uu(Slice(), k) <= amax_p));
            opti.subject_to(tt(k) > 0);

            // Add initial constraints
            if (s == 0 && k == 0){
                opti.subject_to(xx(0, 0) == start(0));
                opti.subject_to(xx(1, 0) == start(1));
                opti.subject_to(xx(2, 0) == start_vel(0));
                opti.subject_to(xx(3, 0) == start_vel(1));
            }
            
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

            // the first point of a corridor should also be enforced to be 
            // within the previous corridor to prevent corner cutting (if there 
            // exists a previous corridor)
            if (s > 0 && k == k_offset){
                // current_corridor = corridor_sequence_.GetCorridor(s+1);
                corners[0].SetValues(xx(0, k) - width_offset, 
                                    xx(1, k) - height_offset);
                corners[1].SetValues(xx(0, k) + width_offset,
                                    xx(1, k) - height_offset);
                corners[2].SetValues(xx(0, k) + width_offset,
                                    xx(1, k) + height_offset);
                corners[3].SetValues(xx(0, k) - width_offset,
                                    xx(1, k) + height_offset);
                // Corridor previous_corridor = corridor_sequence_.GetCorridor(s-1);
                Corridor_MX previous_corridor;
                previous_corridor.SetXmin(corridor_p(0, s-1));
                previous_corridor.SetXmax(corridor_p(1, s-1));
                previous_corridor.SetYmin(corridor_p(2, s-1));
                previous_corridor.SetYmax(corridor_p(3, s-1));
                for (Point2D<MX> corner : corners){
                    opti.subject_to(corridor_p(0, s-1) <= 
                            (corner.x() <= corridor_p(1, s-1)));
                    opti.subject_to(corridor_p(2, s-1) <= 
                            (corner.y() <= corridor_p(3, s-1)));
                }
            }

            // Add max velocity constraint
            opti.subject_to(-vmax_p <= (xx(Slice(2,4), k) <= vmax_p));

            // Add final constraints
            if (s == n - 1 && 
                k == k_offset + nb_points_per_corridor_ - 1){
                opti.subject_to(xx(0, N) == dest(0));
                opti.subject_to(xx(1, N) == dest(1));
                opti.subject_to(xx(2, N) == 0);
                opti.subject_to(xx(3, N) == 0);
            }
        }
    }

    opti.minimize(obj);
    opti.solver(solver_name, opts_casadi, opts_solver);

    Dict opts;
    opts["error_on_fail"] = true;
    Function opti_f = opti.to_function("opti_OCP_" + std::to_string(n), 
        // inputs
        {opti.x(), vmax_p, amax_p, veh_width_p, veh_height_p, margin_p, start, 
         dest, start_vel, corridor_p},
        // outputs
        {tt, xx, uu},
        // input names
        {"x_init", "vmax", "amax", "veh_width", "veh_height", "margin", "start", 
         "dest", "start_vel", "corridors"},
        // output names
        {"tt", "xx", "uu"},
        opts);
    prepared_opti_instances_[nbCorridors] = opti_f;

    //////////////////////////////////
    /// Prepare inputs and outputs ///
    //////////////////////////////////
    std::map<std::string, DM> inputs;
    inputs["x_init"] = DM(4*(N+1) + 3*N, 1);
    inputs["vmax"] = DM(params_.GetVmax());
    inputs["amax"] = DM(params_.GetAmax());
    inputs["veh_width"] = DM(params_.GetVehWidth());
    inputs["veh_height"] = DM(params_.GetVehHeight());
    inputs["margin"] = DM(params_.GetMargin());
    inputs["start"] = DM(2, 1);
    inputs["dest"] = DM(2, 1);
    inputs["start_vel"] = DM(2, 1);
    inputs["corridors"] = DM(4, n);

    opti_inputs_[nbCorridors] = inputs;
};

void OCPSolver::Solve(const UpdateToken&, std::string& solver_name,
                      casadi::Dict& opts_solver,
                      casadi::Dict& opts_casadi,
                      bool just_in_time_preparation_mode){
    int n = corridor_sequence_.NbCorridors();
    int N = n * nb_points_per_corridor_;

    // Check if the desired opti instance is available
    if (just_in_time_preparation_mode ||
        prepared_opti_instances_.find(n) == prepared_opti_instances_.end()){
        PrepareSingleOptiInstance(n, solver_name, opts_casadi, opts_solver);
    }
    // PrepareSingleOptiInstance(n, solver_name, opts_casadi, opts_solver); // why does this resolve the OCP bug?

    // Prepare initialization
    std::vector<Point2D<double>> initialization_waypoints = 
        corridor_sequence_.GetCorridorOverlapCenters();

    opti_inputs_[n]["vmax"] = DM(params_.GetVmax());
    opti_inputs_[n]["amax"] = DM(params_.GetAmax());
    opti_inputs_[n]["veh_width"] = DM(params_.GetVehWidth());
    opti_inputs_[n]["veh_height"] = DM(params_.GetVehHeight());
    opti_inputs_[n]["margin"] = DM(params_.GetMargin());

    double initialization_distance;
    int k_offset;
    for (int s = 0; s < n; s++){
        initialization_distance = initialization_waypoints[s].Distance(
            initialization_waypoints[s+1]);
        
        k_offset = s * nb_points_per_corridor_;
        for (int k = k_offset; k < k_offset + nb_points_per_corridor_; k++){
            opti_inputs_[n]["x_init"](7*k) = initialization_waypoints[s].x() + 
                (k - k_offset)*(initialization_waypoints[s+1].x() - 
                initialization_waypoints[s].x())/nb_points_per_corridor_;
            opti_inputs_[n]["x_init"](7*k+1) = initialization_waypoints[s].y() +
                (k - k_offset)*(initialization_waypoints[s+1].y() - 
                initialization_waypoints[s].y())/nb_points_per_corridor_;
            opti_inputs_[n]["x_init"](7*k+2) = 0;
            opti_inputs_[n]["x_init"](7*k+3) = 0;
            opti_inputs_[n]["x_init"](7*k+4) = initialization_distance/params_.GetVmax();
            opti_inputs_[n]["x_init"](7*k+5) = 0;
            opti_inputs_[n]["x_init"](7*k+6) = 0;
        }

        opti_inputs_[n]["corridors"](0, s) = DM(corridor_sequence_.GetCorridor(s).Xmin());
        opti_inputs_[n]["corridors"](1, s) = DM(corridor_sequence_.GetCorridor(s).Xmax());
        opti_inputs_[n]["corridors"](2, s) = DM(corridor_sequence_.GetCorridor(s).Ymin());
        opti_inputs_[n]["corridors"](3, s) = DM(corridor_sequence_.GetCorridor(s).Ymax());
    }
    opti_inputs_[n]["x_init"](7*N) = initialization_waypoints[n].x();
    opti_inputs_[n]["x_init"](7*N+1) = initialization_waypoints[n].y();

    opti_inputs_[n]["start"](0) = initialization_waypoints[0].x();
    opti_inputs_[n]["start"](1) = initialization_waypoints[0].y();

    opti_inputs_[n]["dest"](0) = initialization_waypoints[n].x();
    opti_inputs_[n]["dest"](1) = initialization_waypoints[n].y();

    opti_inputs_[n]["start_vel"](0) = corridor_sequence_.GetStartVel().x();
    opti_inputs_[n]["start_vel"](1) = corridor_sequence_.GetStartVel().y();

    std::vector<DM> input = {
        opti_inputs_[n]["x_init"],
        opti_inputs_[n]["vmax"],
        opti_inputs_[n]["amax"],
        opti_inputs_[n]["veh_width"],
        opti_inputs_[n]["veh_height"],
        opti_inputs_[n]["margin"],
        opti_inputs_[n]["start"],
        opti_inputs_[n]["dest"],
        opti_inputs_[n]["start_vel"],
        opti_inputs_[n]["corridors"]
    };
    std::vector<DM> output = prepared_opti_instances_[n](input);
    latest_solution_.clear();
    latest_solution_["tt"] = output[0];
    latest_solution_["xx"] = output[1];
    latest_solution_["uu"] = output[2];
    // std::cout << prepared_opti_instances_[n].stats() << std::endl;
    latest_solver_time_ = prepared_opti_instances_[n].stats()["t_wall_total"];
    latest_success_status_ = prepared_opti_instances_[n].stats()["success"];
    std::cout << "OCP solver time: " << latest_solver_time_ << std::endl;
    std::cout << "OCP solver success: " << latest_success_status_ << std::endl;
}