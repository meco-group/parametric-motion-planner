
#include "parametrization.hpp"

Parametrization::Parametrization(CorridorSequence const &corridor_sequence)
    : corridor_sequence_(corridor_sequence),
      max_nb_corridors_(corridor_sequence.MaxNbCorridors()), 
      alpha_x_(corridor_sequence.MaxNbCorridors()), 
      alpha_y_(corridor_sequence.MaxNbCorridors()),
      waypoints_(corridor_sequence.MaxNbCorridors()), 
      waypoint_offsets_(corridor_sequence.MaxNbCorridors()),
      t_x_(corridor_sequence.MaxNbCorridors()), 
      t_y_(corridor_sequence.MaxNbCorridors()) {

    for (int i = 0; i < max_nb_corridors_; i++){
        t_x_[i] = std::vector<double>(3);
        t_y_[i] = std::vector<double>(3);
    };
}