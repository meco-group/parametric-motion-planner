#ifndef __TRAJECTORY__
#define __TRAJECTORY__

#include <vector>
#include <casadi/casadi.hpp>

using namespace casadi;

class Trajectory{
    public:
        Trajectory(){};

        Trajectory(double dt, DM &xx_ocp, DM &uu_ocp, 
                   std::vector<double> &tt_ocp);

        // printing
        friend std::ostream& operator<<(std::ostream &out, Trajectory &trajectory);

        // Basic getters
        double Tf(){ return t_[t_.size() - 1];};
        std::vector<double> T(){ return t_;};
        std::vector<double> Px(){ return px_;};
        std::vector<double> Py(){ return py_;};
        std::vector<double> Vx(){ return vx_;};
        std::vector<double> Vy(){ return vy_;};
        std::vector<double> Ax(){ return ax_;};
        std::vector<double> Ay(){ return ay_;};



    private:
        double dt_;

        std::vector<double> t_;
        std::vector<double> px_;
        std::vector<double> py_;
        std::vector<double> vx_;
        std::vector<double> vy_;
        std::vector<double> ax_;
        std::vector<double> ay_;
};

#endif