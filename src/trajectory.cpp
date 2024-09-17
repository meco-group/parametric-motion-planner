#include <vector>
#include <casadi/casadi.hpp>

#include "trajectory.hpp"

using namespace casadi;

Trajectory::Trajectory(double dt, DM &xx_ocp, DM &uu_ocp, 
                       std::vector<double> &tt_ocp){
    // Initialize empty trajectory
    int nb_samples = tt_ocp[tt_ocp.size() - 1] / dt + 1;
    t_ = std::vector<double>(nb_samples);
    px_ = std::vector<double>(nb_samples);
    py_ = std::vector<double>(nb_samples);
    vx_ = std::vector<double>(nb_samples);
    vy_ = std::vector<double>(nb_samples);
    ax_ = std::vector<double>(nb_samples);
    ay_ = std::vector<double>(nb_samples);

    // Duplicate last controls
    DM last_controls = DM::zeros(2, 1);
    for (int i = 0; i < 2; i++){
        last_controls(i) = uu_ocp(i, uu_ocp.size2() - 1);
    }
    uu_ocp = horzcat(uu_ocp, last_controls);

    // initialize time-grid
    for (int i = 0; i < nb_samples; i++){
        t_[i] = i * dt;
    }

    int ocp_sol_idx = 0;
    double alpha, beta;
    for (int i = 0; i < nb_samples; i++){
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
    vx_[nb_samples - 1] = 0.0;
    vy_[nb_samples - 1] = 0.0;
    ax_[nb_samples - 1] = 0.0;
    ay_[nb_samples - 1] = 0.0;
}

std::ostream& operator<<(std::ostream &out, Trajectory &trajectory){
    out << "t\tpx\tpy\tvx\tvy\tax\tay" << std::endl;
    for (int i = 0; i < trajectory.T().size(); i++){
        out << trajectory.T()[i] << "\t" << trajectory.Px()[i] << "\t" << trajectory.Py()[i] << "\t" << trajectory.Vx()[i] << "\t" << trajectory.Vy()[i] << "\t" << trajectory.Ax()[i] << "\t" << trajectory.Ay()[i] << std::endl;
    }
    return out;
}