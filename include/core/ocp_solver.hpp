#include "corridor.hpp"
#include "helper_types.hpp"

// forward decalaration
class MotionPlanner;

class OCPSolver{
    public:
        OCPSolver(CorridorSequence const &corridor_sequence,
                  Parameters const &params);

        class UpdateToken{
            public: 
                void Invalidate(){is_valid_ = false;};
                void Validate(){is_valid_ = true;};

            friend class MotionPlanner;
            friend class OCPSolver; 
            private: 
                UpdateToken() {};
                bool is_valid_ = true;
        };

        void PrepareOptiInstances(const UpdateToken&, std::string& solver_name_,
                                  casadi::Dict& opts_casadi, 
                                  casadi::Dict& opts_solver);

        void Solve(const UpdateToken&, std::string& solver_name,
                   casadi::Dict& opts_casadi,
                   casadi::Dict& opts_solver,
                   bool just_in_time_preparation_mode);

        // basic getters
        std::map<std::string, casadi::DM> GetLatestSolution() const { return latest_solution_;};
        double GetLatestSolverTime() const { return latest_solver_time_;};
        int GetLatestSuccessStatus() const { return latest_success_status_;};
        int GetNbPointsPerCorridor() const { return nb_points_per_corridor_;};
        void SetNbPointsPerCorridor(int n){ nb_points_per_corridor_ = n;};
        
    private:
        void PrepareSingleOptiInstance(int nbCorridors,
                                       std::string& solver_name,
                                       casadi::Dict& opts_casadi,
                                       casadi::Dict& opts_solver);
                                
        const CorridorSequence& corridor_sequence_;
        const Parameters& params_;
        const int max_nb_corridors_;

        int nb_points_per_corridor_ = 30;

        // list of prepared opti instances
        std::map<int, casadi::Function> prepared_opti_instances_;
        std::map<int, std::map<std::string, casadi::DM>> opti_inputs_;

        // solution objects
        std::map<std::string, casadi::DM> latest_solution_;
        double latest_solver_time_;
        int latest_success_status_;
};