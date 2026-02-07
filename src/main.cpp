#include <iostream>
#include <cmath>

#include "parametric_motion_planner.hpp"
#include "core/main_functions/basics.hpp"
#include "core/main_functions/tests.hpp"
#include "core/main_functions/initial_multi_mover_tests.hpp"
#include "core/main_functions/multi_mover_benchmarking.hpp"
#include "core/main_functions/deadlock_and_special_cases.hpp"

int main(int argc, char *argv[]){
    ///////////////////////////////////
    /// Prepare simulation scenario ///
    ///////////////////////////////////
    // prepare:
    // - environment
    // - station locations
    // - starting positions of agents
    // - sequence of tasks for each agent
    // - list of parameter objects for each agent

    // Create a default environment
    Environment env = Environment();
    double cell_size = env.CellWidth();

    int nb_stations = 30;      // number of stations in the environment
    int nb_movers = 5;         // number of movers in the environment
    int nb_task_per_agent = 5; // number of tasks per agent

    // some options on station locations
    bool avoid_narrow_passage = false;
    bool avoid_neighboring_stations = false; // this option limits the maximum number of stations available in the environment
        
    // define stations (random station names are just named "0", "1", "2", ...)
    std::map<std::string, Point2D<int>> stations = 
        GetRandomStations(nb_stations, env, avoid_neighboring_stations, 
                          avoid_narrow_passage);

    // set starting positions of agents
    std::vector<std::string> starting_positions;
    std::string random_starting_position;
    for (int i = 0; i < nb_movers; i++){
        // pick a random station as starting position for agent i
        random_starting_position = GetRandomStation(stations);

        // make sure to not put two movers at the same station
        while (std::find(starting_positions.begin(), starting_positions.end(), random_starting_position) != starting_positions.end()){
            random_starting_position = GetRandomStation(stations);
        }

        // select this starting station
        starting_positions.push_back(random_starting_position);
    }

    // create tasks for agents
    std::vector<MoverTask> tasks = {};
    for (int i = 0; i < nb_movers; i++){
        for (int j = 0; j < nb_task_per_agent; j++){
            std::string dest_station = GetRandomStation(stations);
            
            // create task to move agent i to dest_station after 
            // completing j previous tasks
            tasks.push_back(MoverTask(i, dest_station, j));

            // to reveal a task at a specific time, use constructor
            // MoverTask(i, dest_station, double time_to_reveal_this_task)
        }
    }

    // define agent parameters
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }






    //////////////////////////////////
    /// Create MultiMoverSimulator ///
    //////////////////////////////////
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);

    // enable Limited Update Mode if desired
    bool limited_update_time = false;
    if (limited_update_time){
        mms.SetMaxUpdateTrajectoryTime(1.0); // in ms
    }

    // for best solver performance, prepare casadi::Opti instances for each
    // agent in advance (avoiding online symbolic computation)
    bool prepare_offline_computation = true;
    if (prepare_offline_computation){
        mms.PrepareOptiInstances();
    }
    
    // Simulate
    try{
        mms.SimulateAllTasks();
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    
    // store logging files (used for visualization and post-processing)
    mms.DumpToJson("my_multi_mover_simulator_output.json");

    // print all tasks completed and basic profiler info
    mms.PrintLog();

    // for visualization, run python file
    // post-process/multi_mover_simulator/visualize_multi_mover_simulator.py
}
