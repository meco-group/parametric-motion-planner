#include "corridor.hpp"
#include "helper_types.hpp"
#include "helper_methods.hpp"

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

        void PrepareOptiInstance(const UpdateToken&, int nbCorridors,
                                 std::string& solver_name_,
                                 casadi::Dict const &opts_casadi,
                                 casadi::Dict const &opts_solver);        
    private:
        const CorridorSequence& corridor_sequence_;
        const Parameters& params_;
        const int max_nb_corridors_;

        int nb_points_per_corridor_ = 30;
};