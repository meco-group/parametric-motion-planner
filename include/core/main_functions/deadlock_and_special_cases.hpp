#ifndef __DEADLOC_AND_SPECIAL_CASES__
#define __DEADLOC_AND_SPECIAL_CASES__

#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>

#include "../motion_planner.hpp"
#include "../multimover/multi_mover_simulator.hpp"

void TestDeadlockScenario(){
    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(8, 17, cell_size, cell_size);
    std::vector<int> x_obs = {2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 
                              4, 4, 4, 4, 4, 4, 7, 7, 7, 7, 7, 7,
                              8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9,
                              12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13,
                              14, 14, 14, 14, 14, 14};
    std::vector<int> y_obs = {0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
                              0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
                              0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
                              0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
                              0, 1, 2, 3, 4, 5};
    for (int i = 0; i < x_obs.size(); i++){
        env.AddObstacle(Point2D<int>(x_obs[i], y_obs[i]));
    }

    bool RANDOMIZE = false;

    std::map<std::string, Point2D<int>> stations;
    if (RANDOMIZE){
        int nb_stations = 12;
        while (stations.size() < nb_stations){
            Point2D<double> candidate = env.GetRandomFreeCellPosition();
            Point2D<int> candidate_cell = 
                candidate.ConvertWorldToCell(env.CellWidth(), env.CellHeight());

            // check if candidate is not already in the map
            bool new_station = true;
            for (const auto& station : stations){
                if (station.second.Distance(candidate_cell) < 0.1){
                    new_station = false;
                    break;
                }
            }

            // check if candidate is not in main corridor
            bool in_main_corridor = candidate_cell.y() > 5;

            if (new_station && !in_main_corridor){
                stations[std::to_string(stations.size())] = candidate_cell;
            }
        }
    } else {
        stations = {{"0", Point2D<int>(0, 0)},
                    {"1", Point2D<int>(5, 2)},
                    {"2", Point2D<int>(5, 0)},
                    {"3", Point2D<int>(10, 2)},
                    {"4", Point2D<int>(10, 0)},
                    {"5", Point2D<int>(15, 5)},
                    {"6", Point2D<int>(15, 3)}};
    };


    Parameters params = Parameters();
    int nb_movers = 6;
    if (!RANDOMIZE){ nb_movers = 4;}

    std::vector<std::string> starting_positions;
    std::vector<MoverTask> tasks;
    if (RANDOMIZE){
        for (int i = 0; i < nb_movers; i++){
            starting_positions.push_back(std::to_string(i));

            // pick a random station that is not occupied
            int station_idx = rand() % stations.size();
            tasks.push_back(MoverTask(i, std::to_string(station_idx), 0.1));
        }
    } else {
        starting_positions = {"0", "4", "2", "5"};
        tasks = {
            MoverTask(0, "3", 0.1),
            // MoverTask(1, "1", 0.1),
            // MoverTask(2, "6", 0.1),
            MoverTask(1, "3", 0.1),
            MoverTask(2, "3", 0.1),
            MoverTask(3, "0", 0.1),
            MoverTask(3, "3", 1.0),
        };
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    try{
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    
    try{
        // Simulate for a bit
        std::cout << "Simulating..." << std::endl;
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    mms.DumpToJson("output/multi_mover_simulator.json");

    } catch (std::exception &e){
    std::cout << "something went wrong: " << e.what() << std::endl;
    std::cerr << e.what() << std::endl;
    return;
    }
}

void TestDeadlockScenario2(){
    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(8, 8, cell_size, cell_size);
    
    // Get stations at random boundary positions
    std::map<std::string, Point2D<int>> stations {
        {"0", Point2D<int>(1, 6)},
        {"1", Point2D<int>(6, 1)},
        {"2", Point2D<int>(6, 6)},
        {"3", Point2D<int>(1, 1)},
        {"4", Point2D<int>(6, 4)},
        {"5", Point2D<int>(1, 4)},
        {"6", Point2D<int>(3, 1)},
        {"7", Point2D<int>(4, 6)},
    };

    Parameters params = Parameters();
    int nb_movers = 4;

    std::vector<std::string> starting_positions = {"0", "2", "4", "6"};

    // Create tasks
    std::vector<MoverTask> tasks = {
        MoverTask(0, "1", 0.1),
        MoverTask(1, "3", 0.1),
        MoverTask(2, "5", 0.1),
        MoverTask(3, "7", 0.1),
    };

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    
    try{
        // Simulate for a bit
        std::cout << "Simulating..." << std::endl;
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
        mms.DumpToJson("output/multi_mover_simulator.json");
    }

    mms.PrintLog();
}

void TestDeadlockScenario3(){
    // scenario where two movers get into a deadlock by waiting for each other
    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(4, 4, cell_size, cell_size);
    
    // Get stations at random boundary positions
    std::map<std::string, Point2D<int>> stations {
        {"0", Point2D<int>(1, 0)},
        {"1", Point2D<int>(1, 3)},
        {"2", Point2D<int>(2, 3)},
        {"3", Point2D<int>(2, 0)},
        {"4", Point2D<int>(0, 0)},
        {"5", Point2D<int>(3, 3)},
        {"6", Point2D<int>(3, 0)},
        {"7", Point2D<int>(0, 3)},
    };

    Parameters params = Parameters();
    int nb_movers = 4;

    std::vector<std::string> starting_positions = {"0", "2", "4", "6"};

    // Create tasks
    std::vector<MoverTask> tasks = {
        MoverTask(0, "1", 0.0),
        MoverTask(1, "3", 0.0),
        MoverTask(2, "5", 0.4),
        MoverTask(3, "7", 0.41),
    };

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[0].SetAmax(0.4); params_list[0].SetVmax(0.2);
    params_list[1].SetAmax(0.4); params_list[1].SetVmax(0.2);
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    
    try{
        // Simulate for a bit
        std::cout << "Simulating..." << std::endl;
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
        mms.DumpToJson("output/multi_mover_simulator.json");
    }
    mms.PrintLog();
}

void TestDeadlockScenario4(){
    // Complex case where four movers get into a deadlock (with three other 
    // movers also moving around)
    // The environment consists more or less of four narrow corridors that all
    // end up on a central square
    try{
    int l = 6;
    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(2*l+2, 2*l+2, cell_size, cell_size);
    for (int i = 0; i < 2*l+2; i++){
        for (int j = 0; j < l+1; j++){
            if (i == l+1 || (i < l+1 && j == l)){continue;}
            // env.AddObstacle(Point2D<int>(i, j));
            env.DeleteCell(Point2D<int>(i, j));
        }
    }
    for (int i = 0; i < 2*l+2; i++){
        for (int j = l+1; j < 2*l+2; j++){
            if (i == l || (i >= l && j == l+1) || (i == 2 && j == l+1)){continue;}
            // env.AddObstacle(Point2D<int>(i, j));
            env.DeleteCell(Point2D<int>(i, j));
        }
    }
    for (int i = l-2; i < l+3; i++){
        env.AddCell(Point2D<int>(i, l+3));
    }
    env.AddCell(Point2D<int>(0, l+1));

    bool ENFORCE_REACHABILITY = true;
    if (ENFORCE_REACHABILITY){
        env.AddCell(Point2D<int>(1, l+1));
        for (int i = l-2; i < l; i++){
            env.AddCell(Point2D<int>(i, l+4));
        }
    }

    std::cout << env << std::endl;
    
    std::map<std::string, Point2D<int>> stations {
        {"0", Point2D<int>(0, l)},
        {"1", Point2D<int>(2, l+1)},
        {"2", Point2D<int>(l, 2*l+1)},
        {"3", Point2D<int>(l+1, 0)},
        {"4", Point2D<int>(2*l+1, l+1)},
        {"5", Point2D<int>(l-1, l+3)},
        {"6", Point2D<int>(l+2, l+3)},
        {"7", Point2D<int>(l-2, l+3)},
        {"8", Point2D<int>(0, l+1)},
    };

    Parameters params = Parameters();
    int nb_movers = 7;

    std::vector<std::string> starting_positions = {"2", "4", "3", "0", "5", "7", "8"};

    // Create tasks
    std::vector<MoverTask> tasks = {
        MoverTask(0, "1", 0.1),
        MoverTask(1, "2", 0.34),
        MoverTask(2, "4", 0.58),
        MoverTask(3, "3", 0.82),
        MoverTask(4, "6", 0.01),
        MoverTask(5, "5", 0.01),
        MoverTask(6, "0", 1.0),
    };

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_list[i].SetVmax(0.8);
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[4].SetVmax(0.1);
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    
    try{
        // Simulate for a bit
        std::cout << "Simulating..." << std::endl;
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
        mms.DumpToJson("output/multi_mover_simulator.json");
    }
    mms.PrintLog();
    } catch (std::exception &e){
        std::cout << "caught all: " << e.what() << std::endl;
    }
}

void TestDeadlockScenario5(){
   try{
    int nb_cycling_movers = 10;
    int nb_movers = nb_cycling_movers + 1;
    int e = 1;

    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(nb_cycling_movers+2+e+1, 2*nb_movers+1, cell_size, cell_size);
    for (int i = 0; i < env.NbCellCols(); i++){
        for (int j = env.NbCellRows()-1; j > env.NbCellRows()-2-e; j--){
            if (i == nb_movers){continue;}
            env.DeleteCell(Point2D<int>(i, j));
        }
    }
    env.AddObstacle(Point2D<int>(nb_movers, nb_cycling_movers));
    std::cout << env << std::endl;
    
    std::map<std::string, Point2D<int>> stations {
        {"special_first_station",  Point2D<int>(0, nb_cycling_movers)},
        {"special_second_station",  Point2D<int>(nb_movers, nb_cycling_movers+2+e)},
        {"special_third_station",  Point2D<int>(2*nb_movers, nb_cycling_movers)},
    };
    for (int i = 0; i < nb_cycling_movers; i++){
        stations["first_operation_" + std::to_string((i+1)%nb_cycling_movers)] = Point2D<int>(1+i, nb_cycling_movers);
        stations["second_operation_" + std::to_string(i)] = Point2D<int>(nb_movers+1+nb_cycling_movers-1-i, nb_cycling_movers);
        stations["third_operation_" + std::to_string(i)] = Point2D<int>(nb_movers+1, i);
    }

    std::vector<std::string> starting_positions = {};
    for (int i = 0; i < nb_cycling_movers; i++){
        starting_positions.push_back("first_operation_" + std::to_string(i));
    }
    starting_positions.push_back("special_first_station");
    

    // Create tasks
    int nb_task_waves = 5;
    std::vector<MoverTask> tasks = {};
    for (int i = 0; i < nb_cycling_movers; i++){
        double movement_duration = 1.0;
        int idx = i;
        int task_nb = 0;
        for (int j = 0; j < nb_task_waves; j++){
            tasks.push_back(MoverTask(i, 
                "second_operation_" + std::to_string(idx), task_nb));
            tasks[tasks.size()-1].SetVMax(0.4);
            tasks.push_back(MoverTask(i, 
                "third_operation_" + std::to_string(idx), task_nb + 1));
            tasks[tasks.size()-1].SetVMax(2.0);
            idx = (idx + 1) % nb_cycling_movers;
            tasks.push_back(MoverTask(i, 
                "first_operation_" + std::to_string(idx), task_nb + 2));
            tasks[tasks.size()-1].SetVMax(2.0);
            task_nb += 3;
        }
    }
    std::cout << "tasks:\n" << tasks << std::endl;
    // add special route
    tasks.push_back(MoverTask(nb_cycling_movers, "special_second_station", 0, 4.0));
    tasks.push_back(MoverTask(nb_cycling_movers, "special_third_station", 1));
    tasks[tasks.size()-1].SetAMax(1.0);

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    
    try{
        // Simulate for a bit
        std::cout << "Simulating..." << std::endl;
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
        mms.DumpToJson("output/multi_mover_simulator.json");
    }
    mms.PrintLog();
    } catch (std::exception &e){
        std::cout << "caught all: " << e.what() << std::endl;
    }
}

void SolveRandomProblemsForIllustration(){
    Parameters params = Parameters();
    Environment env = Environment(10, 12, 0.12, 0.12);
    std::vector<int> xx = {0, 0, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 11};
    std::vector<int> yy = {3, 6, 9, 3, 5, 3, 4, 5, 3, 5, 8, 6, 8, 0, 1, 7, 4, 6, 8, 1, 4, 6, 7, 9, 0, 2, 4, 9, 3, 5, 3};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }

    MotionPlanner mp = MotionPlanner(params, env);
    mp.SetStart(Point2D<double>(0.0724476, 0.634078));
    mp.SetDest(Point2D<double>(1.32, 1.115));

    std::vector<std::vector<double>> ppx;
    std::vector<std::vector<double>> ppy;
    int nb_trajs = 5000;
    int nb_tries = 2*nb_trajs;
    int counter = 0;
    int nb_failures = 0;
    while (counter < nb_trajs && nb_failures < nb_tries){
        try{
            mp.Plan();
            ppx.push_back(std::vector<double>(mp.GetLastSolution()->NbSamples()));
            ppy.push_back(std::vector<double>(mp.GetLastSolution()->NbSamples()));
            for (int i = 0; i < mp.GetLastSolution()->NbSamples(); i++){
                ppx[counter].at(i) = mp.GetLastSolution()->Px().at(i);
                ppy[counter].at(i) = mp.GetLastSolution()->Py().at(i);
            }
            counter++;
        } catch (std::exception e){
            std::cout << "Failure: " << e.what() << std::endl;
            nb_failures++;
        }
    }
    
    json results;
    results["ppx"] = ppx;
    results["ppy"] = ppy;
    std::ofstream file("output/random_problem_solutions.json");
    if (file.is_open()){
        file << results.dump(4);
        file.close();
    } else {
        std::cerr << "Could not open file for writing." << std::endl;
    }
}

void SolveReviewersProblem(){
    Environment env = Environment(5, 5, 0.12, 0.12);
    std::vector<int> xx = {1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4};
    std::vector<int> yy = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }

    Parameters params = Parameters();
    MotionPlanner mp = MotionPlanner(params, env);

    Point2D<int> start_cell = Point2D<int>(0, 0);
    Point2D<int> dest_cell = Point2D<int>(4, 4);
    mp.SetStart(start_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    mp.SetDest(dest_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    mp.PlanSafely();
    mp.DumpToJson("reviewers_problem_solution_no_coupling.json");
};

void SolveReviewersDiagonalProblem(){
    Environment env = Environment(14, 14, 0.12, 0.12);
    std::vector<int> xx = {0, 1, 2, 3, 2, 2, 6, 6, 6, 6, 6, 9, 9, 9, 9, 9, 9};
    std::vector<int> yy = {4, 4, 4, 4, 0, 1, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }

    Parameters params = Parameters();
    MotionPlanner mp = MotionPlanner(params, env);
    Point2D<int> start_cell = Point2D<int>(0, 0);
    Point2D<int> dest_cell = Point2D<int>(13, 13);
    mp.SetStart(start_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    mp.SetDest(dest_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    mp.SetSolver("ipopt");
    mp.SetMethod(OCP);

    for (int i = 0; i < 8; i++){
        mp.SetMaxNbCorridorGrowingIterations(i);
        mp.PlanSafely();
        mp.DumpToJson("reviewers_problem_solution_diagonal_ocp_" + std::to_string(i) + ".json");
    }

}

void TestWaitingPointCase(){
    /////////////
    /// Setup ///
    /////////////
    Environment env = Environment(4, 3, 0.12, 0.12);
    std::map<std::string, Point2D<int>> stations {
        {"main_mover_start", Point2D<int>(1, 0)},
        {"main_mover_end", Point2D<int>(1, 3)},
    };
    std::vector<std::string> starting_positions = {"main_mover_start"};
    std::vector<MoverTask> tasks = {MoverTask(0, "main_mover_end", 0, 1.6)};
    std::vector<Parameters> params_list(starting_positions.size());
    std::vector<Parameters*> params_ptr_list;

    stations["main_mover_start"] = Point2D<int>(1, 0);
    starting_positions.push_back("blocking_mover_start_0");
    starting_positions.push_back("blocking_mover_start_1");
    // starting_positions.push_back("blocking_mover_start_2");
    // starting_positions.push_back("blocking_mover_start_3");

    stations["blocking_mover_start_0"] = Point2D<int>(0, 1);
    stations["blocking_mover_end_0"] = Point2D<int>(2, 1);
    stations["blocking_mover_start_1"] = Point2D<int>(2, 2);
    stations["blocking_mover_end_1"] = Point2D<int>(0, 2);
    // stations["blocking_mover_start_2"] = Point2D<int>(0, 3);
    // stations["blocking_mover_end_2"] = Point2D<int>(2, 3);
    // stations["blocking_mover_start_3"] = Point2D<int>(2, 5);
    // stations["blocking_mover_end_3"] = Point2D<int>(0, 5);

    std::string dest_name = "end";
    for (int i = 0; i < 10; i++){
        tasks.push_back(MoverTask(1, "blocking_mover_" + dest_name + "_0", i));
        tasks.push_back(MoverTask(2, "blocking_mover_" + dest_name + "_1", i, 0.5));
        // tasks.push_back(MoverTask(3, "blocking_mover_" + dest_name + "_2", i, 1.0));
        // tasks.push_back(MoverTask(4, "blocking_mover_" + dest_name + "_3", i, 1.5));
        if (dest_name == "start"){ dest_name = "end";} else { dest_name = "start";}
    }

    params_list.push_back(Parameters());
    params_list.push_back(Parameters());
    params_list.push_back(Parameters());
    // params_list.push_back(Parameters());
    params_list[1].SetVmax(0.2);
    params_list[2].SetVmax(0.2);
    // params_list[3].SetVmax(0.2);
    // params_list[4].SetVmax(0.2);
    for (int i = 0; i < starting_positions.size(); i++){
        params_ptr_list.push_back(&params_list[i]);
    }

    MultiMoverSimulator mms1 = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    mms1.SetIgnoreCollisions(true);
    mms1.SetIgnoreWaitingPoints(false);
    int nb_steps = 6000;
    std::string file_name = "output/multi_mover_simulator_special_case_ignore_collisions.json";
    std::string file_name_appendix = "";
    for (int i = 0; i < nb_steps; i++){
        std::cout << "Simulating step " << i << " ..." << std::endl;
        try{
            mms1.SimulateSteps(1, false);
        } catch (std::exception &e){
            std::cout << "something went wrong at step " << i << ": " << e.what() << std::endl;
            std::cerr << e.what() << std::endl;
            break;
        }

        if (mms1.AllTasksCompleted()){
            std::cout << "All tasks completed at step " << i << "!" << std::endl;
            break;
        }
    }
    mms1.DumpToJson(file_name + file_name_appendix);
    mms1.PrintLog();
    
    // Simulate
    MultiMoverSimulator mms2 = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    mms2.SetIgnoreCollisions(false);
    mms2.SetIgnoreWaitingPoints(true);
    bool abort = false;
    for (int i = 0; i < nb_steps; i++){
        std::cout << "Simulating step " << i << " ..." << std::endl;
        try{
            mms2.SimulateSteps(1, false);
        } catch (std::exception &e){
            std::cout << "something went wrong at step " << i << ": " << e.what() << std::endl;
            std::cerr << e.what() << std::endl;
            if (i < 6000){
                abort = true;
            }
            break;
        }

        if (mms2.AllTasksCompleted()){
            std::cout << "All tasks completed at step " << i << "!" << std::endl;
            break;
        }
    }
    file_name = "output/multi_mover_simulator_special_case_ignore_waiting_points.json";
    mms2.DumpToJson(file_name + file_name_appendix);
    mms2.PrintLog();
    if (abort){ return;}
    
    // Simulate
    MultiMoverSimulator mms3 = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    mms3.SetIgnoreCollisions(false);
    mms3.SetIgnoreWaitingPoints(false);
    for (int i = 0; i < nb_steps; i++){
        std::cout << "Simulating step " << i << " ..." << std::endl;
        try{
            mms3.SimulateSteps(1, false);
        } catch (std::exception &e){
            std::cout << "something went wrong at step " << i << ": " << e.what() << std::endl;
            std::cerr << e.what() << std::endl;
            break;
        }

        if (mms3.AllTasksCompleted()){
            std::cout << "All tasks completed at step " << i << "!" << std::endl;
            break;
        }
    }
    file_name = "output/multi_mover_simulator_special_case_normal_mode.json";
    mms3.DumpToJson(file_name + file_name_appendix);
    mms3.PrintLog();
}

void TestLivelockCase(){
    Environment env = Environment();
    env.AddObstacle(Point2D<int>(5, 0));
    env.AddObstacle(Point2D<int>(5, 1));
    env.AddObstacle(Point2D<int>(7, 0));
    env.AddObstacle(Point2D<int>(7, 1));
    std::map<std::string, Point2D<int>> stations {
        {"main_mover_start", Point2D<int>(6, 0)},
        {"main_mover_end", Point2D<int>(6, 3)},
    };
    std::vector<std::string> starting_positions = {"main_mover_start"};
    std::vector<MoverTask> tasks = {MoverTask(0, "main_mover_end", 0, 0.4)};
    std::vector<Parameters> params_list(starting_positions.size());
    params_list[0].SetVmax(0.05);

    int nb_blocking_movers = 4;
    int nb_waves = 10;
    for (int i = 0; i < nb_blocking_movers; i++){
        stations["blocking_mover_start_" + std::to_string(i)] = Point2D<int>(0, i);
        stations["blocking_mover_end_" + std::to_string(i)] = Point2D<int>(11, i);
        starting_positions.push_back("blocking_mover_start_" + std::to_string(i));

        params_list.push_back(Parameters());
        
        std::string name = "end";
        for (int w = 0; w < nb_waves; w++){
            tasks.push_back(MoverTask(i+1, "blocking_mover_" + name + "_" + std::to_string(i), w, w == 0 ? 0.4*i : 0));
            if (name == "start"){ name = "end";} else { name = "start";}
        }
    };

    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < starting_positions.size(); i++){
        params_ptr_list.push_back(&params_list[i]);
    }

    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    try {
        mms.SimulateAllTasks();
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    mms.DumpToJson("output/multi_mover_simulator_livelock_case.json");
}

void IllustrativeCase(){
    // Environment env = Environment();
    int extra_row = 0;
    int extra_col = 0;
    Environment env = Environment(3 + extra_row, 10 + extra_col, 0.12, 0.12);
    std::map<std::string, Point2D<int>> stations {
        {"S1", Point2D<int>(0, 2 + extra_row)},        
        {"S2", Point2D<int>(0, 0)},
        {"S3", Point2D<int>(0, 1 + extra_row)},

        {"S4", Point2D<int>(4 + extra_col, 0)},
        {"S8", Point2D<int>(3 + extra_col, 2 + extra_row)},
        {"S5", Point2D<int>(9 + extra_col, 2 + extra_row)},
        {"S6", Point2D<int>(4 + extra_col, 2 + extra_row)},
        {"S7", Point2D<int>(7 + extra_col, 2 + extra_row)},
    };
    std::vector<std::string> starting_positions = {"S1", "S4", "S2", "S5"};
    std::vector<MoverTask> tasks = {
        MoverTask(1, "S6", 0, 0),
        MoverTask(2, "S7", 0, 0.05),
        MoverTask(3, "S1", 0, 0.1),
    };

    std::vector<Parameters> params_list(starting_positions.size());
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < starting_positions.size(); i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[1].SetVmax(0.2);

    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    try {
        mms.SimulateAllTasks();
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    mms.DumpToJson("output/multi_mover_simulator_illustrative_case.json");
}

std::string GetRandomStation(std::map<std::string, Point2D<int>>& stations)
{
    static std::mt19937 rng{ std::random_device{}() };
    std::uniform_int_distribution<size_t> dist(0, stations.size() - 1);

    auto it = stations.begin();
    std::advance(it, dist(rng));
    return it->first;
}

void LargeCase(){
    Environment env = Environment(30, 30, 0.12, 0.12);
    std::vector<int> xx = {1, 1, 1};
    std::vector<int> yy = {0, 1, 2};
    for (int i = 5; i < 15; i++){
        for (int j = 5; j < 25; j++){
            if (i > 10 && (j > 7 && j < 12 || j > 16 && j < 22)){continue;}
            xx.push_back(i);
            yy.push_back(j);
        }
    }
    for (int i = 20; i < 25; i++){
        for (int j = 5; j < 25; j++){
            // if (j > 20 && j < 30){continue;}
            if (j > 5 && j < 10 || j > 15 && j < 20){continue;}
            xx.push_back(i);
            yy.push_back(j);
        }
    }
    for (int i = 0; i < xx.size(); i++){
        env.DeleteCell(Point2D<int>(xx[i], yy[i]));
    }
    env.AddCell(Point2D<int>(22, 20));
    env.AddCell(Point2D<int>(22, 21));

    std::map<std::string, Point2D<int>> stations;
    for (int i = 0; i < 29; i++){
        if (i != 1){
            stations["S" + std::to_string(stations.size())] = Point2D<int>(i, 0);
        }
        stations["S" + std::to_string(stations.size())] = Point2D<int>(i, 29);
    }
    for (int i = 4; i < 16; i++){
        for (int j = 4; j < 26; j++){
            if (i > 11 && (j > 7 && j < 12 || j > 16 || j < 22)){continue;}
            if (env.IsFree(Point2D<int>(i, j))){
                stations["S" + std::to_string(stations.size())] = Point2D<int>(i, j);
            }
        }
    }
    for (int i = 19; i < 26; i++){
        for (int j = 4; j < 26; j++){
            if (j > 6 && j < 9 || j > 16 && j < 19){continue;}
            if (i == 22 && j == 19){continue;}
            if (i == 22 && j == 20){continue;}
            if (env.IsFree(Point2D<int>(i, j))){
                stations["S" + std::to_string(stations.size())] = Point2D<int>(i, j);
            }
        }
    }

    int nb_movers = 100;
    std::vector<std::string> starting_positions = {};
    // pick random starting positions
    while (starting_positions.size() < nb_movers){
        std::string station_name = GetRandomStation(stations);
        if (std::find(starting_positions.begin(), starting_positions.end(), station_name) == starting_positions.end()){
            starting_positions.push_back(station_name);
        }
    }


    // Create tasks
    std::vector<MoverTask> tasks = {};
    for (int i = 0; i < nb_movers; i++){
        std::string dest_station = GetRandomStation(stations);
        tasks.push_back(MoverTask(i, dest_station, 0));
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    mms.SetPlanWhileMoving(false);

    try{
        // Simulate for a bit
        mms.SimulateAllTasks();
        // Dump to json
        mms.DumpToJson("output/multi_mover_simulator_large_case.json");
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
        mms.DumpToJson("output/multi_mover_simulator_large_case.json");
    }
}

void UnresolvableDeadlockCase(){
    /*
    Environment env = Environment(5, 9, 0.12, 0.12);
    std::vector<int> xx = {0, 1, 2, 3, 3, 5, 5, 6, 7, 8};
    std::vector<int> yy = {2, 2, 2, 2, 4, 2, 4, 2, 2, 2};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }

    std::map<std::string, Point2D<int>> stations {
        {"U0", Point2D<int>(0, 4)},
        {"U1", Point2D<int>(1, 4)},
        {"U2", Point2D<int>(2, 4)},
        {"U4", Point2D<int>(4, 4)},
        {"U6", Point2D<int>(6, 4)},
        {"U7", Point2D<int>(7, 4)},
        {"U8", Point2D<int>(8, 4)},
        {"L0", Point2D<int>(0, 0)},
        {"L1", Point2D<int>(1, 0)},
        {"L2", Point2D<int>(2, 0)},
        {"L3", Point2D<int>(3, 0)},
        {"L4", Point2D<int>(4, 0)},
        {"L5", Point2D<int>(5, 0)},
        {"L6", Point2D<int>(6, 0)},
        {"L7", Point2D<int>(7, 0)},
        {"L8", Point2D<int>(8, 0)},
    };

    std::vector<std::string> starting_positions = {
        "L4", "U2", "U6", "U1", "U7", "L5"
    };

    // Create tasks
    std::vector<MoverTask> tasks = {
        MoverTask(0, "U4", 0.0),
        MoverTask(1, "U8", 1.4),
        MoverTask(2, "U0", 1.41),
        MoverTask(3, "L1", 1.6),
        MoverTask(4, "L7", 1.61),
        MoverTask(5, "U2", 1.8),
    };

    std::vector<Parameters> params_list(starting_positions.size());
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < starting_positions.size(); i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[0].SetVmax(0.2);
    */

    Environment env = Environment(14, 13, 0.12, 0.12);
    std::vector<int> xx = {5, 6, 6, 6, 6, 6, 6, 7};
    std::vector<int> yy = {5, 8, 9, 10, 11, 12, 13, 5};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }

    std::map<std::string, Point2D<int>> stations {
        {"A", Point2D<int>(5, 13)},
        {"B", Point2D<int>(7, 13)},
        {"C", Point2D<int>(0, 8)},
        {"D", Point2D<int>(0, 7)},
        {"E", Point2D<int>(0, 6)},
        {"F", Point2D<int>(4, 5)},
        {"G", Point2D<int>(6, 7)},
        {"H", Point2D<int>(6, 0)},
        {"I", Point2D<int>(12, 8)},
        {"J", Point2D<int>(12, 7)},
        {"K", Point2D<int>(12, 6)},
        {"L", Point2D<int>(8, 5)},
        {"M", Point2D<int>(2, 5)},
        {"N", Point2D<int>(3, 4)},
        {"O", Point2D<int>(9, 4)},
        {"P", Point2D<int>(10, 5)},
    };
    std::vector<std::string> starting_positions = {
        "H", "E", "K", "A", "B"
    };

    std::vector<MoverTask> tasks = {
        MoverTask(0, "G", 0),
        MoverTask(1, "J", 3.0),
        MoverTask(2, "D", 3.1),
        MoverTask(3, "I", 3.6),
        MoverTask(4, "C", 3.8),
    };

    std::vector<Parameters> params_list(starting_positions.size());
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < starting_positions.size(); i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[0].SetVmax(0.2);
    params_list[3].SetAmax(9);
    params_list[4].SetAmax(9);

    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
        stations, starting_positions, tasks);
    try {
        mms.SimulateAllTasks();
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    mms.DumpToJson("output/multi_mover_simulator_unresolvable_deadlock_case.json");
}

#endif