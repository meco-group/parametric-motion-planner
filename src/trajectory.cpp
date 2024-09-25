#include <vector>
#include <casadi/casadi.hpp>
#include <nlohmann/json.hpp>

#include "trajectory.hpp"

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

void Trajectory::Update(DM const &xx_ocp, DM const &uu_ocp, 
                        std::vector<double> const &tt_ocp){
    curr_nb_samples_ = tt_ocp[tt_ocp.size() - 1] / dt_ + 1;

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

void Trajectory::Update(int nb_corridors,
                        std::vector<std::vector<double>> const &t_x,
                        std::vector<std::vector<double>> const &t_y,
                        std::vector<double> const &alpha_x,
                        std::vector<double> const &alpha_y,
                        std::vector<Point2D<double>> const &waypoints,
                        std::vector<Point2D<double>> const 
                            &waypoint_velocities,
                        double a_max){
    // compute the total time
    double total_time = 0.0;
    for (int i = 0; i < t_x.size(); i++){
        total_time += t_x[i][0] + t_x[i][1] + t_x[i][2];
    }

    // compute the number of samples
    curr_nb_samples_ = total_time / dt_ + 1;

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

    // loop over corridors
    for (int w = 0; w < nb_corridors; w++){
        local_corridor_time = std::fmod(local_corridor_time, dt_);

        // set initial conditions for this corridor
        p0.CopyValues(waypoints[w]);
        v0.CopyValues(waypoint_velocities[w]);

        // sample this corridor
        while (local_corridor_time < t_x[w][0] + t_x[w][1] + t_x[w][2]){
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
            vx_[sample_ptr] = v0.x() + alpha_x[w]*a_max*tx_arc1 +
                              alpha_x[w+1]*a_max*tx_arc3;
            vy_[sample_ptr] = v0.y() + alpha_y[w]*a_max*ty_arc1 +
                              alpha_y[w+1]*a_max*ty_arc3;
            
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

    return j;
}