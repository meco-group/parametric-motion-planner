#include <vector>
#include <casadi/casadi.hpp>

#include "trajectory.hpp"

using namespace casadi;

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

void Trajectory::Update(DM &xx_ocp, DM &uu_ocp, std::vector<double> &tt_ocp){
    curr_nb_samples_ = tt_ocp[tt_ocp.size() - 1] / dt_ + 1;
    
    // Duplicate last controls
    DM last_controls = DM::zeros(2, 1);
    for (int i = 0; i < 2; i++){
        last_controls(i) = uu_ocp(i, uu_ocp.size2() - 1);
    }
    uu_ocp = horzcat(uu_ocp, last_controls);

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

std::ostream& operator<<(std::ostream &out, Trajectory &trajectory){
    out << "t\tpx\tpy\tvx\tvy\tax\tay" << std::endl;
    for (int i = 0; i < trajectory.NbSamples(); i++){
        out << trajectory.T()[i] << "\t" << trajectory.Px()[i] << "\t" << trajectory.Py()[i] << "\t" << trajectory.Vx()[i] << "\t" << trajectory.Vy()[i] << "\t" << trajectory.Ax()[i] << "\t" << trajectory.Ay()[i] << std::endl;
    }
    return out;
}