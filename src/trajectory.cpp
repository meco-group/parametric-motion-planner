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
    corridor_infeasibilities_detected_ = false;
    solver_time_ = solver_time;
    emergency_braking_ = false;

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

        // px_[i] = beta * positions[p2p_sol_idx].x() + 
        //          alpha * positions[p2p_sol_idx + 1].x();
        // py_[i] = beta * positions[p2p_sol_idx].y() +
        //          alpha * positions[p2p_sol_idx + 1].y();
        px_[i] = positions[p2p_sol_idx].x() + 
                 velocities[p2p_sol_idx].x()*(t_[i] - accumulated_time) + 
                 0.5*(t_[i] - accumulated_time)*(t_[i] - accumulated_time)*accelerations[p2p_sol_idx].x();
        py_[i] = positions[p2p_sol_idx].y() + 
                 velocities[p2p_sol_idx].y()*(t_[i] - accumulated_time) + 
                 0.5*(t_[i] - accumulated_time)*(t_[i] - accumulated_time)*accelerations[p2p_sol_idx].y();
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
                        double solver_time, 
                        CorridorSequence const &corridor_sequence,
                        Parameters const &params){
    total_computation_time_ = -1;
    corridor_infeasibilities_detected_ = false;
    solver_time_ = solver_time;
    emergency_braking_ = false;
    tf_ = tt_ocp[tt_ocp.size() - 1];

    curr_nb_samples_ = tf_ / dt_ + 2;

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

    Point2D<double> point_to_check;
    int corridor_idx = 0;
    
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

        // px_[i] = beta * double(xx_ocp(0, ocp_sol_idx - 1)) + 
        //          alpha * double(xx_ocp(0, ocp_sol_idx));
        // py_[i] = beta * double(xx_ocp(1, ocp_sol_idx - 1)) +
        //          alpha * double(xx_ocp(1, ocp_sol_idx));
        if (i == 0){
            px_[i] = double(xx_ocp(0, ocp_sol_idx - 1));
            py_[i] = double(xx_ocp(1, ocp_sol_idx - 1));
        } else {
            px_[i] = double(xx_ocp(0, ocp_sol_idx-1) + 
                     xx_ocp(2, ocp_sol_idx-1)*(t_[i] - tt_ocp[ocp_sol_idx-1]) + 
                     0.5*uu_ocp(0, ocp_sol_idx-1)*std::pow(t_[i] - tt_ocp[ocp_sol_idx-1], 2));
            py_[i] = double(xx_ocp(1, ocp_sol_idx-1) + 
                     xx_ocp(3, ocp_sol_idx-1)*(t_[i] - tt_ocp[ocp_sol_idx-1]) + 
                     0.5*uu_ocp(1, ocp_sol_idx-1)*std::pow(t_[i] - tt_ocp[ocp_sol_idx-1], 2));
        }
        vx_[i] = beta * double(xx_ocp(2, ocp_sol_idx - 1)) +
                 alpha * double(xx_ocp(2, ocp_sol_idx));
        vy_[i] = beta * double(xx_ocp(3, ocp_sol_idx - 1)) +
                 alpha * double(xx_ocp(3, ocp_sol_idx));
        ax_[i] = beta * double(uu_ocp(0, ocp_sol_idx - 1)) +
                 alpha * double(uu_ocp(0, ocp_sol_idx));
        ay_[i] = beta * double(uu_ocp(1, ocp_sol_idx - 1)) +
                 alpha * double(uu_ocp(1, ocp_sol_idx));

        // Check to update the corridor index
        point_to_check.SetX(px_[i]);
        point_to_check.SetY(py_[i]);
        if (corridor_idx < corridor_sequence.NbCorridors() - 1 && 
            corridor_sequence.GetCorridor(corridor_idx + 1).ContainsVehicle(point_to_check, params)){
            corridor_idx++;
        }

        // Check if position sample is within the corridor
        if (!CheckPointInCorridors(point_to_check, corridor_sequence, 
                                   corridor_idx, params)){
            corridor_infeasibilities_detected_ = true;
        }
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
    corridor_infeasibilities_detected_ = false;
    solver_time_ = solver_time;
    emergency_braking_ = false;

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
    curr_nb_samples_ = tf_ / dt_ + 2;

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
    double prev_corridor_time = 0.0;
    double tx_arc1, tx_arc2, tx_arc3, ty_arc1, ty_arc2, ty_arc3;
    Point2D<double> p0;
    Point2D<double> v0;
    Point2D<double> point_to_check;

    int corridor_idx = 0;
    bool out_of_corridor = false;

    // loop over corridors
    for (int w = 0; w < corridor_sequence.NbCorridors(); w++){
        corridor_idx = w;
        local_corridor_time -= prev_corridor_time;

        // set initial conditions for this corridor
        p0.CopyValues(waypoints[w]);
        v0.CopyValues(waypoint_velocities[w]);

        // sample this corridor
        double local_x_timing = t_x[w][0] + t_x[w][1] + t_x[w][2];
        double local_y_timing = t_y[w][0] + t_y[w][1] + t_y[w][2];
        while (local_corridor_time < local_x_timing ||
               local_corridor_time < local_y_timing){
            
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

            if (local_x_timing == 0){
                tx_arc1 = -1.0e-16; 
                tx_arc2 = local_corridor_time; 
                tx_arc3 = local_corridor_time + 1.0e-16;
            }
            if (local_y_timing == 0){
                ty_arc1 = -1.0e-16; 
                ty_arc2 = local_corridor_time; 
                ty_arc3 = local_corridor_time + 1.0e-16;
            }

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
            out_of_corridor = !CheckPointInCorridors(point_to_check, 
                                                     corridor_sequence, 
                                                     corridor_idx, params);
            if (out_of_corridor){
                corridor_infeasibilities_detected_ = true;
                if (out_of_corridor_list.count(w) == 0){
                    out_of_corridor_list.insert(w);
                    Corridor c = corridor_sequence.GetCorridor(w);
                    // std::cout << "Point " << point_to_check << " is out of corridor " << c << std::endl;
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
        prev_corridor_time = std::max(t_x[w][0] + t_x[w][1] + t_x[w][2],
                                      t_y[w][0] + t_y[w][1] + t_y[w][2]);
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

void Trajectory::Update(Point2D<double> const &start,
                        Point2D<double> const &start_vel,
                        std::vector<double> const &accel_x,
                        std::vector<double> const &accel_y,
                        std::vector<double> const &t_x,
                        std::vector<double> const &t_y){       
    emergency_braking_ = true;
    
    // compute the total time
    double tf_ = t_x[0] + t_x[1] + t_x[2];

    // compute the number of samples
    curr_nb_samples_ = tf_ / dt_ + 3;

    // initialize time-grid
    for (int i = 0; i < curr_nb_samples_; i++){
        t_[i] = i * dt_;
    }

    // initialize the trajectory
    int sample_ptr = 0;
    double tx_arc1, tx_arc2, tx_arc3, ty_arc1, ty_arc2, ty_arc3;
    Point2D<double> p0 = start;
    Point2D<double> v0 = start_vel;
    double t = 0.0;
    for (int i = 0; i < curr_nb_samples_; i++){  
        t = t_[i];

        // update timings for every arc
        tx_arc1 = std::max(0.0, std::min(t_x[0], t));
        tx_arc2 = std::max(0.0, std::min(t_x[1], t - t_x[0]));
        tx_arc3 = std::max(0.0, std::min(t_x[2], t - t_x[0] - t_x[1]));
        ty_arc1 = std::max(0.0, std::min(t_y[0], t));
        ty_arc2 = std::max(0.0, std::min(t_y[1], t - t_y[0]));
        ty_arc3 = std::max(0.0, std::min(t_y[2], t - t_y[0] - t_y[1]));

        // Update x
        px_[sample_ptr] = p0.x() + v0.x()*tx_arc1 + 0.5*accel_x[0]*std::pow(tx_arc1, 2);
        vx_[sample_ptr] = v0.x() + accel_x[0]*tx_arc1;

        px_[sample_ptr] += vx_[sample_ptr]*tx_arc2 + 0.5*accel_x[1]*std::pow(tx_arc2, 2);
        vx_[sample_ptr] += accel_x[1]*tx_arc2;

        px_[sample_ptr] += vx_[sample_ptr]*tx_arc3 + 0.5*accel_x[2]*std::pow(tx_arc3, 2);
        vx_[sample_ptr] += accel_x[2]*tx_arc3;

        // Update y
        py_[sample_ptr] = p0.y() + v0.y()*ty_arc1 + 0.5*accel_y[0]*std::pow(ty_arc1, 2);
        vy_[sample_ptr] = v0.y() + accel_y[0]*ty_arc1;

        py_[sample_ptr] += vy_[sample_ptr]*ty_arc2 + 0.5*accel_y[1]*std::pow(ty_arc2, 2);
        vy_[sample_ptr] += accel_y[1]*ty_arc2;

        py_[sample_ptr] += vy_[sample_ptr]*ty_arc3 + 0.5*accel_y[2]*std::pow(ty_arc3, 2);
        vy_[sample_ptr] += accel_y[2]*ty_arc3;
            
        // Update acceleration
        if (t <= t_x[0]){ 
            ax_[sample_ptr] = accel_x[0];
        } else if (t <= t_x[0] + t_x[1]){ 
            ax_[sample_ptr] = accel_x[1];
        } else {
            ax_[sample_ptr] = accel_x[2];
        }
        if (t <= t_x[0]){ 
            ay_[sample_ptr] = accel_y[0];
        } else if (t <= t_y[0] + t_y[1]){
            ay_[sample_ptr] = accel_y[1];
        } else {
            ay_[sample_ptr] = accel_y[2];
        }

        sample_ptr++;
        t += dt_;
    }

    // The last sample should be steady-state
    px_[curr_nb_samples_ - 1] = px_[curr_nb_samples_ - 2];
    py_[curr_nb_samples_ - 1] = py_[curr_nb_samples_ - 2];
    vx_[curr_nb_samples_ - 1] = 0.0;
    vy_[curr_nb_samples_ - 1] = 0.0;
    ax_[curr_nb_samples_ - 1] = 0.0;
    ay_[curr_nb_samples_ - 1] = 0.0;
}

void Trajectory::Reset(Point2D<double> const &start){
    total_computation_time_ = -1;
    solver_time_ = -1;
    tf_ = 0.0;
    curr_nb_samples_ = 1;
    corridor_infeasibilities_detected_ = false;
    emergency_braking_ = false;

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
    if (std::isnan(px) || std::isnan(py) || std::isinf(px) || std::isinf(py)){
        throw std::runtime_error("nan or inf found!");
    }

    if (t > tf_){
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

void Trajectory::GetSample(int idx, double &time, Point2D<double> &pos, 
                           Point2D<double> &vel, Point2D<double> &acc) const {
    idx = std::max(0, std::min(idx, curr_nb_samples_ - 1));
    time = t_[idx];
    pos.SetX(px_[idx]);
    pos.SetY(py_[idx]);
    vel.SetX(vx_[idx]);
    vel.SetY(vy_[idx]);
    acc.SetX(ax_[idx]);
    acc.SetY(ay_[idx]);
}

void Trajectory::CheckCollision(Trajectory const &other, 
                                Parameters const &params_this, 
                                Parameters const &params_other,
                                Point2D<double>& collision_point){
    int sample_idx = 0;
    double distance_x, distance_y;
    double x_margin = params_this.GetWidthOffset() + params_other.GetWidthOffset();
    double y_margin = params_this.GetHeightOffset() + params_other.GetHeightOffset();

    Point2D<double> pos_this, vel_this, acc_this; double t_this;
    Point2D<double> pos_other, vel_other, acc_other; double t_other;

    std::cout << "Checking " << std::max(NbSamples(), other.NbSamples()) << " samples" << std::endl;

    while (sample_idx < std::max(NbSamples(), other.NbSamples())){
        GetSample(sample_idx, t_this, pos_this, vel_this, acc_this);
        other.GetSample(sample_idx, t_other, pos_other, vel_other, acc_other);
        distance_x = std::abs(pos_this.x() - pos_other.x()) - x_margin;
        distance_y = std::abs(pos_this.y() - pos_other.y()) - y_margin;

        // std::cout << pos_this << " - " << pos_other << std::endl;
        // std::cout << distance_x << " - " << distance_y << std::endl;
        // std::cout << std::endl;

        if (distance_x <= 0 && distance_y <= 0){
            // Collision detected!
            std::cout << "Collision detected at t = " << t_this << std::endl;
            std::cout << pos_this << " - " << pos_other << std::endl;
            collision_point = (pos_this + pos_other)*0.5;
            return;
        }

        // Increment the sample index cleverly
        sample_idx += std::max(1, std::min(
            int(distance_x/(params_this.GetVmax() + params_other.GetVmax())/dt_),
            int(distance_y/(params_this.GetVmax() + params_other.GetVmax())/dt_)
            )
        );
    }

    collision_point.SetX(-1.0);
    collision_point.SetY(-1.0);
}

void Trajectory::Concatenate(Trajectory const &other){
    if (curr_nb_samples_ + other.NbSamples() > max_nb_samples_){
        throw std::runtime_error("Trajectory is too long to concatenate");
    }

    if (dt_ != other.Dt()){
        throw std::runtime_error("Cannot concatenate trajectories with different timestep");
    }

    for (int i = 0; i < NbSamples(); i++){
        if (std::isnan(px_[i]) || std::isnan(py_[i]) || 
            std::isinf(px_[i]) || std::isinf(py_[i])){
            std::cerr << "nan found in this trajectory (" << i << "/" << NbSamples() << ")" << std::endl;
        }
    }
    for (int i = 0; i < other.NbSamples(); i++){
        if (std::isnan(other.Px()[i]) || std::isnan(other.Py()[i]) || 
            std::isinf(other.Px()[i]) || std::isinf(other.Py()[i])){
            std::cerr << "nan found in other trajectory (" << i << "/" << other.NbSamples() << ")" << std::endl;
        }
    }

    for (int i = 0; i < other.NbSamples(); i++){
        t_[curr_nb_samples_] = other.T()[i] + tf_;
        px_[curr_nb_samples_] = other.Px()[i];
        py_[curr_nb_samples_] = other.Py()[i];
        vx_[curr_nb_samples_] = other.Vx()[i];
        vy_[curr_nb_samples_] = other.Vy()[i];
        ax_[curr_nb_samples_] = other.Ax()[i];
        ay_[curr_nb_samples_] = other.Ay()[i];
        curr_nb_samples_++;
    }

    tf_ += other.Tf();
    solver_time_ += other.SolverTime();
    total_computation_time_ += other.TotalComputationTime();
    corridor_infeasibilities_detected_ = 
        corridor_infeasibilities_detected_ || 
        other.CorridorInfeasibilitiesDetected();
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
    j["corridor_infeasibilities_detected"] = corridor_infeasibilities_detected_;
    j["emergency_braking"] = emergency_braking_;

    return j;
}

bool Trajectory::CheckPointInCorridors(Point2D<double> const &point_to_check, 
                            CorridorSequence const &corridor_sequence,
                            int corridor_idx, Parameters const &params) const {
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
            // if (corridor_idx < corridor_sequence.NbCorridors() - 1){
            //     std::cout << "Point " << point_to_check << " is out of corridor " << corridor_sequence.GetCorridor(corridor_idx) << " and corridor " << corridor_sequence.GetCorridor(corridor_idx+1) << std::endl;
            // } else {
            //     std::cout << "Point " << point_to_check << " is out of corridor " << corridor_sequence.GetCorridor(corridor_idx) << std::endl;
            // }
            // std::cout << "min_x: " << point_to_check.x() - params.GetVehWidth()/2.0 - params.GetMargin() << std::endl;
            // std::cout << "max_x: " << point_to_check.x() + params.GetVehWidth()/2.0 + params.GetMargin() << std::endl;
            // std::cout << "min_y: " << point_to_check.y() - params.GetVehHeight()/2.0 - params.GetMargin() << std::endl;
            // std::cout << "max_y: " << point_to_check.y() + params.GetVehHeight()/2.0 + params.GetMargin() << std::endl;
            return false;
        }
        corridor_idx--;
    }

    return true;
}