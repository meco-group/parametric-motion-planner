#include <vector>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>

#include "core/trajectory.hpp"

using namespace casadi;
using json = nlohmann::json;

Trajectory::Trajectory() : 
        dt_(DT_DEFAULT), max_trajectory_time_(MAX_TRAJECTORY_TIME_DEFAULT), 
        max_nb_samples_(MAX_TRAJECTORY_TIME_DEFAULT / DT_DEFAULT + 1){
    t_ = std::vector<double>(max_nb_samples_);
    px_ = std::vector<double>(max_nb_samples_);
    py_ = std::vector<double>(max_nb_samples_);
    vx_ = std::vector<double>(max_nb_samples_);
    vy_ = std::vector<double>(max_nb_samples_);
    ax_ = std::vector<double>(max_nb_samples_);
    ay_ = std::vector<double>(max_nb_samples_);
}

void Trajectory::Update(int nb_corridors,
                        std::vector<Point2D<double>> const &waypoints,
                        std::vector<Point2D<double>> const &positions,
                        std::vector<Point2D<double>> const &velocities,
                        std::vector<Point2D<double>> const &accelerations,
                        std::vector<double> const &time_durations,
                        double solver_time){
    total_computation_time_ = -1;
    solver_time_ = solver_time;

    double total_time = 0.0;
    for (int i = 0; i < 3*nb_corridors+1; i++){
        total_time += time_durations[i];
    }
    tf_ = total_time;

    // compute the number of samples
    curr_nb_samples_ = total_time / dt_ + 1;
    if (curr_nb_samples_ > max_nb_samples_){
        // throw std::runtime_error("Trajectory  is too long to be updated");
        curr_nb_samples_ = 0;
        return;
    }

    // initialize time-grid
    for (int i = 0; i < curr_nb_samples_; i++){
        t_[i] = i * dt_;
    }

    int p2p_sol_idx = 0; // we will interpolate between idx and idx + 1
    double alpha, beta;
    double accumulated_time = 0.0; // sum of completely sampled durations
    for (int i = 0; i < curr_nb_samples_; i++){

        // for every sample, figure out the value of p2p_sol_idx
        while (t_[i] > accumulated_time + time_durations[p2p_sol_idx]){
            accumulated_time += time_durations[p2p_sol_idx];
            p2p_sol_idx++;
        }

        // Linearly interpolate
        alpha = (t_[i] - accumulated_time) / time_durations[p2p_sol_idx];
        beta = 1 - alpha;

        px_[i] = beta * positions[p2p_sol_idx].x() + 
                 alpha * positions[p2p_sol_idx + 1].x();
        py_[i] = beta * positions[p2p_sol_idx].y() +
                 alpha * positions[p2p_sol_idx + 1].y();
        vx_[i] = beta * velocities[p2p_sol_idx].x() +
                 alpha * velocities[p2p_sol_idx + 1].x();
        vy_[i] = beta * velocities[p2p_sol_idx].y() +
                 alpha * velocities[p2p_sol_idx + 1].y();

        ax_[i] = accelerations[p2p_sol_idx].x();
        ay_[i] = accelerations[p2p_sol_idx].y();
    }

    // The last sample should be steady-state
    vx_[curr_nb_samples_ - 1] = 0.0;
    vy_[curr_nb_samples_ - 1] = 0.0;
    ax_[curr_nb_samples_ - 1] = 0.0;
    ay_[curr_nb_samples_ - 1] = 0.0;
}

void Trajectory::Update(DM const &xx_ocp, DM const &uu_ocp, 
                        std::vector<double> const &tt_ocp,
                        double solver_time){
    total_computation_time_ = -1;
    solver_time_ = solver_time;
    tf_ = tt_ocp[tt_ocp.size() - 1];

    curr_nb_samples_ = tf_ / dt_ + 1;

    if (curr_nb_samples_ > max_nb_samples_){
        // throw std::runtime_error("Trajectory  is too long to be updated");
        curr_nb_samples_ = 0;
        return;
    }

    // initialize time-grid
    for (int i = 0; i < curr_nb_samples_; i++){
        t_[i] = i * dt_;
    }

    int ocp_sol_idx = 0;
    double alpha, beta;
    for (int i = 0; i < curr_nb_samples_; i++){
        // Figure out where to linearly interpolate
        while (ocp_sol_idx < tt_ocp.size() - 2 && 
                (tt_ocp[ocp_sol_idx]) < t_[i]){
            ocp_sol_idx++;
        }

        if (ocp_sol_idx == 0) {ocp_sol_idx = 1;}

        // Linearly interpolate
        alpha = (t_[i] - tt_ocp[ocp_sol_idx - 1]) / 
                (tt_ocp[ocp_sol_idx] - double(tt_ocp[ocp_sol_idx - 1]));
        beta = 1 - alpha;

        px_[i] = beta * double(xx_ocp(0, ocp_sol_idx - 1)) + 
                 alpha * double(xx_ocp(0, ocp_sol_idx));
        py_[i] = beta * double(xx_ocp(1, ocp_sol_idx - 1)) +
                 alpha * double(xx_ocp(1, ocp_sol_idx));
        vx_[i] = beta * double(xx_ocp(2, ocp_sol_idx - 1)) +
                 alpha * double(xx_ocp(2, ocp_sol_idx));
        vy_[i] = beta * double(xx_ocp(3, ocp_sol_idx - 1)) +
                 alpha * double(xx_ocp(3, ocp_sol_idx));
        ax_[i] = beta * double(uu_ocp(0, ocp_sol_idx - 1)) +
                 alpha * double(uu_ocp(0, ocp_sol_idx));
        ay_[i] = beta * double(uu_ocp(1, ocp_sol_idx - 1)) +
                 alpha * double(uu_ocp(1, ocp_sol_idx));
    }

    // The last sample should be steady-state
    vx_[curr_nb_samples_ - 1] = 0.0;
    vy_[curr_nb_samples_ - 1] = 0.0;
    ax_[curr_nb_samples_ - 1] = 0.0;
    ay_[curr_nb_samples_ - 1] = 0.0;
}

std::set<int> Trajectory::Update(CorridorSequence const &corridor_sequence,
                                 std::vector<std::vector<double>> const &t_x,
                                 std::vector<std::vector<double>> const &t_y,
                                 std::vector<double> const &alpha_x,
                                 std::vector<double> const &alpha_y,
                                 std::vector<Point2D<double>> const &waypoints,
                                 std::vector<Point2D<double>> const 
                                        &waypoint_velocities,
                                 Parameters const &params,
                                 double solver_time){
    total_computation_time_ = -1;
    solver_time_ = solver_time;
    
    double a_max = params.GetAmax();
    std::set<int> out_of_corridor_list = {};
    
    // compute the total time
    double total_time_x = 0.0;
    for (int i = 0; i < corridor_sequence.NbCorridors(); i++){
        total_time_x += t_x[i][0] + t_x[i][1] + t_x[i][2];
    }
    double total_time_y = 0.0;
    for (int i = 0; i < corridor_sequence.NbCorridors(); i++){
        total_time_y += t_y[i][0] + t_y[i][1] + t_y[i][2];
    }
    tf_ = std::max(total_time_x, total_time_y);

    // compute the number of samples
    curr_nb_samples_ = tf_ / dt_ + 1;

    if (curr_nb_samples_ > max_nb_samples_){
        // throw std::runtime_error("Trajectory  is too long to be updated");
        curr_nb_samples_ = 0;
        return out_of_corridor_list;
    }

    // initialize time-grid
    for (int i = 0; i < curr_nb_samples_; i++){
        t_[i] = i * dt_;
    }

    // initialize the trajectory
    int sample_ptr = 0;
    double local_corridor_time = 0.0;
    double tx_arc1, tx_arc2, tx_arc3, ty_arc1, ty_arc2, ty_arc3;
    Point2D<double> p0;
    Point2D<double> v0;
    Point2D<double> point_to_check;

    int corridor_idx = 0;

    // loop over corridors
    for (int w = 0; w < corridor_sequence.NbCorridors(); w++){
        corridor_idx = w;
        local_corridor_time = std::fmod(local_corridor_time, dt_);

        // set initial conditions for this corridor
        p0.CopyValues(waypoints[w]);
        v0.CopyValues(waypoint_velocities[w]);

        // sample this corridor
        while (local_corridor_time < t_x[w][0] + t_x[w][1] + t_x[w][2] ||
               local_corridor_time < t_y[w][0] + t_y[w][1] + t_y[w][2]){
            
            // update timings for every arc
            tx_arc1 = std::max(0.0, std::min(t_x[w][0], local_corridor_time));
            tx_arc2 = std::max(0.0, std::min(t_x[w][1], local_corridor_time - 
                                             t_x[w][0]));
            tx_arc3 = std::max(0.0, std::min(t_x[w][2], local_corridor_time - 
                                             t_x[w][0] - t_x[w][1]));
            ty_arc1 = std::max(0.0, std::min(t_y[w][0], local_corridor_time));
            ty_arc2 = std::max(0.0, std::min(t_y[w][1], local_corridor_time - 
                                             t_y[w][0]));
            ty_arc3 = std::max(0.0, std::min(t_y[w][2], local_corridor_time -
                                             t_y[w][0] - t_y[w][1]));

            // Update position
            px_[sample_ptr] = p0.x() + v0.x()*tx_arc1 +
                              0.5*alpha_x[w]*a_max*std::pow(tx_arc1, 2) +
                              (v0.x() + alpha_x[w]*a_max*tx_arc1)*
                                (tx_arc2 + tx_arc3) +
                              0.5*alpha_x[w+1]*a_max*std::pow(tx_arc3, 2);
            py_[sample_ptr] = p0.y() + v0.y()*ty_arc1 +
                              0.5*alpha_y[w]*a_max*std::pow(ty_arc1, 2) +
                              (v0.y() + alpha_y[w]*a_max*ty_arc1)*
                                (ty_arc2 + ty_arc3) +
                              0.5*alpha_y[w+1]*a_max*std::pow(ty_arc3, 2);
            
            // Check if position sample is within the corridor
            point_to_check.SetX(px_[sample_ptr]);
            point_to_check.SetY(py_[sample_ptr]);
            if (!corridor_sequence.GetCorridor(corridor_idx).ContainsVehicle(
                    point_to_check, params)){
                // if the point is not in this corridor, check to see if it is
                // in the next corridor
                corridor_idx++;
                if (corridor_idx >= corridor_sequence.NbCorridors() || 
                    !corridor_sequence.GetCorridor(corridor_idx).ContainsVehicle(
                        point_to_check, params)){
                    // if the point is also not in the next corridor, add
                    // it to the
                    // out_of_corridor_list
                    corridor_idx--;
                    if (out_of_corridor_list.count(w) == 0){
                        out_of_corridor_list.insert(w);
                        Corridor c = corridor_sequence.GetCorridor(w);
                        std::cout << "Point " << point_to_check << " is out of corridor " << c << std::endl;
                    }
                }
            }

            // Update velocity
            vx_[sample_ptr] = v0.x() + alpha_x[w]*a_max*tx_arc1 +
                              alpha_x[w+1]*a_max*tx_arc3;
            vy_[sample_ptr] = v0.y() + alpha_y[w]*a_max*ty_arc1 +
                              alpha_y[w+1]*a_max*ty_arc3;
            
            // Update acceleration
            if (local_corridor_time <= t_x[w][0]){ 
                ax_[sample_ptr] = alpha_x[w]*a_max;
            } else if (local_corridor_time <= t_x[w][0] + t_x[w][1]){ 
                ax_[sample_ptr] = 0;
            } else {
                ax_[sample_ptr] = alpha_x[w+1]*a_max;
            }
            if (local_corridor_time <= t_y[w][0]){ 
                ay_[sample_ptr] = alpha_y[w]*a_max;
            } else if (local_corridor_time <= t_y[w][0] + t_y[w][1]){
                ay_[sample_ptr] = 0;
            } else {
                ay_[sample_ptr] = alpha_y[w+1]*a_max;
            }

            sample_ptr++;
            local_corridor_time += dt_;
        }
    }

    // The last sample should be steady-state
    px_[curr_nb_samples_ - 1] = px_[curr_nb_samples_ - 2];
    py_[curr_nb_samples_ - 1] = py_[curr_nb_samples_ - 2];
    vx_[curr_nb_samples_ - 1] = 0.0;
    vy_[curr_nb_samples_ - 1] = 0.0;
    ax_[curr_nb_samples_ - 1] = 0.0;
    ay_[curr_nb_samples_ - 1] = 0.0;

    return out_of_corridor_list;
}

void Trajectory::Reset(Point2D<double> const &start){
    total_computation_time_ = -1;
    solver_time_ = -1;
    tf_ = 0.0;
    curr_nb_samples_ = 1;

    // initialize the trajectory
    t_[0] = 0.0;
    px_[0] = start.x();
    py_[0] = start.y();
    vx_[0] = 0.0;
    vy_[0] = 0.0;
    ax_[0] = 0.0;
    ay_[0] = 0.0;
}

void Trajectory::Append(double t, double px, double py, double vx, double vy, 
                        double ax, double ay){
    if (curr_nb_samples_ >= max_nb_samples_){
        throw std::runtime_error("Trajectory is too long to append");
    }

    t_[curr_nb_samples_] = t;
    px_[curr_nb_samples_] = px;
    py_[curr_nb_samples_] = py;
    vx_[curr_nb_samples_] = vx;
    vy_[curr_nb_samples_] = vy;
    ax_[curr_nb_samples_] = ax;
    ay_[curr_nb_samples_] = ay;

    tf_ = t;

    curr_nb_samples_++;
}

std::ostream& operator<<(std::ostream &out, Trajectory &trajectory){
    out << "t\t\tpx\t\tpy\t\tvx\t\tvy\t\tax\t\tay" << std::endl;
    for (int i = 0; i < trajectory.NbSamples(); i++){
        out << trajectory.T()[i] << "\t\t" << trajectory.Px()[i] << "\t\t" << trajectory.Py()[i] << "\t\t" << trajectory.Vx()[i] << "\t\t" << trajectory.Vy()[i] << "\t\t" << trajectory.Ax()[i] << "\t\t" << trajectory.Ay()[i] << std::endl;
    }
    // for (int i = 0; i < trajectory.NbSamples(); i++){
    //     out << trajectory.T()[i] << "\t\t" << trajectory.Ax()[i] << "\t\t" << trajectory.Ay()[i] << std::endl;
    // }

    return out;
}

json Trajectory::ToJson() const {
    json j;

    j["nb_samples"] = curr_nb_samples_;
    j["dt"] = dt_;
    j["t"] = std::vector<double>(t_.begin(), t_.begin() + curr_nb_samples_);
    j["px"] = std::vector<double>(px_.begin(), px_.begin() + curr_nb_samples_);
    j["py"] = std::vector<double>(py_.begin(), py_.begin() + curr_nb_samples_);
    j["vx"] = std::vector<double>(vx_.begin(), vx_.begin() + curr_nb_samples_);
    j["vy"] = std::vector<double>(vy_.begin(), vy_.begin() + curr_nb_samples_);
    j["ax"] = std::vector<double>(ax_.begin(), ax_.begin() + curr_nb_samples_);
    j["ay"] = std::vector<double>(ay_.begin(), ay_.begin() + curr_nb_samples_);
    j["total_computation_time"] = total_computation_time_;
    j["solver_time"] = solver_time_;
    j["Tf"] = Tf();

    return j;
}