#ifndef __MULTI_MOVER_TESTS__
#define __MULTI_MOVER_TESTS__

#include <iostream>
#include <chrono>
#include <cmath>
#include <map>
#include <fstream>
#include "../motion_planner.hpp"
#include "../multimover/multi_mover_simulator.hpp"

std::map<std::string, Point2D<int>> GetMaxNbStations(){
    return {
        {"0", Point2D<int>(0, 1)},
        {"1", Point2D<int>(0, 3)},
        {"2", Point2D<int>(0, 5)},
        {"3", Point2D<int>(0, 7)},
        {"4", Point2D<int>(0, 9)},
        {"5", Point2D<int>(1, 0)},
        {"6", Point2D<int>(2, 9)},
        {"7", Point2D<int>(3, 0)},
        {"8", Point2D<int>(3, 4)},
        {"9", Point2D<int>(3, 6)},
        {"10", Point2D<int>(3, 8)},
        {"11", Point2D<int>(4, 3)},
        {"12", Point2D<int>(5, 0)},
        {"13", Point2D<int>(6, 3)},
        {"14", Point2D<int>(7, 0)},
        {"15", Point2D<int>(8, 3)},
        {"16", Point2D<int>(9, 0)},
        {"17", Point2D<int>(10, 3)},
        {"18", Point2D<int>(11, 0)},
        {"19", Point2D<int>(11, 2)},
    };
}

std::map<std::string, Point2D<int>> GetRandomStations(int nb_stations, Environment& env,
        bool avoid_neighboring_stations, bool avoid_narrow_passage){
    std::map<std::string, Point2D<int>> stations;
    Point2D<int> candidate;
    int nb_stations_found = 0;
    while (nb_stations_found < nb_stations){
        candidate = env.GetRandomFreeCellPositionAtEnvironmentEdge();

        // check if candidate is not already in the map
        bool new_station = true;
        double min_distance = avoid_neighboring_stations ? 1.1 : 0.1;
        for (const auto& station : stations){
            if (station.second.Distance(candidate) < min_distance){
                new_station = false;
                break;
            }
        }

        // make sure candidate is not in a corner (only if neighbouring stations are allowed)
        bool corner = false;
        if (!avoid_neighboring_stations && 
                (candidate.x() == 0 || candidate.x() == env.NbCellCols() - 1) &&
                (candidate.y() == 0 || candidate.y() == env.NbCellRows() - 1) ||
                (candidate.x() == 11 && candidate.y() == 3)){
            corner = true;
        }

        // let's remove any candidates on row 8 and column > 3
        if (candidate.y() == 8 && candidate.x() >= 3){
            corner = true;
        }
        if (avoid_narrow_passage && (candidate.y() == 9 && candidate.x() >= 3)){
            corner = true;
        }

        if (new_station && !corner){
            stations[std::to_string(nb_stations_found)] = candidate;
            nb_stations_found++;
        }
    }

    return stations;
}

Environment GetLargeEnvironment(){
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

    return env;
}

std::map<std::string, Point2D<int>> GetLargeEnvironmentStations(Environment& env){
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
    return stations;
}

std::string GetLargeEnvironmentRandomStation(std::map<std::string, Point2D<int>>& stations)
{
    static std::mt19937 rng{ std::random_device{}() };
    std::uniform_int_distribution<size_t> dist(0, stations.size() - 1);

    auto it = stations.begin();
    std::advance(it, dist(rng));
    return it->first;
}

std::vector<MoverTask> GetLargeEnvironmentTasks(int nb_movers, int nb_waves,
        std::map<std::string, Point2D<int>>& stations){
    // Create tasks
    std::vector<MoverTask> tasks = {};
    for (int i = 0; i < nb_movers; i++){
        for (int j = 0; j < nb_waves; j++){
            std::string dest_station = GetLargeEnvironmentRandomStation(stations);
            tasks.push_back(MoverTask(i, dest_station, j));
        }
    }
    return tasks;
}

std::vector<Point2D<double>> GetLargeEnvironmentStartingPositions(int nb_movers,
        std::map<std::string, Point2D<int>>& stations,
        Environment& env){
    std::vector<Point2D<double>> starting_positions = {};
    std::vector<std::string> used_station_names = {};
    // pick random starting positions
    while (starting_positions.size() < nb_movers){
        std::string station_name = GetLargeEnvironmentRandomStation(stations);
        if (std::find(used_station_names.begin(), used_station_names.end(), station_name) == used_station_names.end()){
            used_station_names.push_back(station_name);
            starting_positions.push_back(stations[station_name].ConvertCellToWorld(
                env.CellWidth(), env.CellHeight()));
        }   
    }
    return starting_positions;
}

void GenerateTaskFile(int nb_agents, int nb_stations, int nb_motion_waves, 
                      int task_file_id, bool benchmarking_mode){
    // the task file is a json file that contains a list of all tasks
    // for each task, it must specify
    //      - agent index
    //      - task sequence number
    //      - destination (as a position)
    //
    // Additionally, the task file specifies
    //      - starting position of each agent (as a position)
    //      - number of agents
    Environment env;
    std::map<std::string, Point2D<int>> stations;
    std::vector<Point2D<double>> starting_positions;
    std::vector<MoverTask> tasks;
    double cell_size = 0.12;

    if (nb_agents > 15 || nb_stations > 25){
        env = GetLargeEnvironment();
        stations = GetLargeEnvironmentStations(env);
        starting_positions = GetLargeEnvironmentStartingPositions(nb_agents, stations, env);
        tasks = GetLargeEnvironmentTasks(nb_agents, nb_motion_waves, stations);
        nb_stations = stations.size();
    } else {

        // TODO: make sure to offer the options to prevent stations to be
        //      - right next to eachother
        //      - in the upper right corridor
        bool avoid_narrow_passage = true;
        bool avoid_neighboring_stations = true;

        env = Environment();
        double cell_size = env.CellWidth();

        // set random seed
        srand(time(0));
        
        // get the stations
        if (avoid_narrow_passage && avoid_neighboring_stations && nb_stations > 15){
            stations = GetMaxNbStations();
        } else {
            stations = GetRandomStations(nb_stations, env, 
                    avoid_neighboring_stations, avoid_narrow_passage);
        }

        // set starting positions
        std::vector<Point2D<double>> starting_positions(nb_agents);
        for (int i = 0; i < nb_agents; i++){
            Point2D<int> start_cell = stations[std::to_string(i)];
            starting_positions[i] = start_cell.ConvertCellToWorld(cell_size, cell_size);
        }

        // create tasks
        tasks = {};
        std::vector<bool> occupied(nb_stations, false);
        std::vector<bool> newly_selected(nb_stations, false);
        for (int i = 0; i < nb_agents; i++){ occupied[i] = true;}
        // wave of motions
        for (int i = 0; i < nb_motion_waves; i++){

            // Add task for each mover
            for (int j = 0; j < nb_agents; j++){
                // pick a random station that is not occupied and not yet selected
                int station_idx = rand() % nb_stations;
                while (occupied[station_idx] || newly_selected[station_idx]){
                    station_idx = rand() % nb_stations;
                }
                newly_selected[station_idx] = true;
                tasks.push_back(MoverTask(j, std::to_string(station_idx), i));

            }
            occupied = newly_selected;
            newly_selected = std::vector<bool>(nb_stations, false);        
        }
    }

    // generate file
    json task_file_json;
    task_file_json["nb_agents"] = nb_agents;
    task_file_json["starting_positions"] = json::array();
    for (int i = 0; i < nb_agents; i++){
        task_file_json["starting_positions"].push_back(starting_positions[i].ToJson());
    }
    task_file_json["tasks"] = json::array();
    for (const auto& task : tasks){
        json task_json = task.ToJson();
        std::string destination_name = task_json["destination_name"];
        task_json["destination"] = stations[destination_name].ConvertCellToWorld(cell_size, cell_size).ToJson();
        task_file_json["tasks"].push_back(task_json);
    }
    task_file_json["stations"] = json::array();
    for (const auto& station : stations){
        json station_json;
        station_json["first"] = station.first;
        station_json["second"] = station.second.ToJson();
        task_file_json["stations"].push_back(station_json);
    }

    // dump to file
    std::string file_name = "tasks_" + std::to_string(nb_agents) + "_agents_" +
                            std::to_string(nb_stations) + "_stations_" +
                            std::to_string(nb_motion_waves) + "_waves_" + 
                            std::to_string(task_file_id) + ".json";
    if (benchmarking_mode){
        file_name = "../build/" + file_name;
    }
    std::cout << "generating file " << file_name << " ..." << std::endl;
    std::ofstream file(file_name);
    file << task_file_json.dump(4);
    file.close();
}

json ReadTaskFile(int nb_agents, int nb_stations, int nb_motion_waves, 
                  int task_file_id, bool benchmarking_mode){
    std::string file_name = "tasks_" + std::to_string(nb_agents) + "_agents_" +
                            std::to_string(nb_stations) + "_stations_" +
                            std::to_string(nb_motion_waves) + "_waves_" +
                            std::to_string(task_file_id) + ".json";
    if (benchmarking_mode){
        file_name = "../build/" + file_name;
    }
    std::cout << "reading file " << file_name << " ..." << std::endl;
    
    std::ifstream file(file_name);
    std::cout << "file opened." << std::endl;
    json task_file_json;
    file >> task_file_json;
    std::cout << "file read." << std::endl;
    file.close();

    return task_file_json;
}

void TestMultiMoverTasksWithStations(int nb_stations=15, int nb_movers=6, int nb_motion_waves=3){
    // Set the stage
    double cell_size = 0.12;
    // Environment env = Environment(6, 6, cell_size, cell_size);
    Environment env = Environment(20, 11, cell_size, cell_size);
    for (int xi = 3; xi < 8; xi++){
        for (int yi = 4; yi < 8; yi++){
            env.DeleteCell(Point2D<int>(xi, yi));
        }
    }
    for (int xi = 3; xi < 4; xi++){
        for (int yi = 11; yi < 16; yi++){
            env.DeleteCell(Point2D<int>(xi, yi));
        }
    }
    for (int xi = 7; xi < 8; xi++){
        for (int yi = 11; yi < 16; yi++){
            env.DeleteCell(Point2D<int>(xi, yi));
        }
    }
    std::cout << env << std::endl;

    // set random seed
    srand(time(0));

    // Get stations at random boundary positions
    std::map<std::string, Point2D<int>> stations;
    Point2D<int> candidate;
    int nb_stations_found = 0;
    while (nb_stations_found < nb_stations){
        candidate = env.GetRandomFreeCellPositionAtEnvironmentEdge();
        // candidate = env.GetRandomFreeCellPosition().ConvertWorldToCell(
        //     env.CellWidth(), env.CellHeight());

        // check if candidate is not already in the map
        bool new_station = true;
        for (const auto& station : stations){
            if (station.second.Distance(candidate) < 0.1){
                new_station = false;
                break;
            }
        }

        // make sure candidate is not in a corner
        bool corner = false;
        if ((candidate.x() == 0 || candidate.x() == env.NbCellCols() - 1) &&
                (candidate.y() == 0 || candidate.y() == env.NbCellRows() - 1)){
            corner = true;
        }

        if (new_station && !corner){
            stations[std::to_string(nb_stations_found)] = candidate;
            nb_stations_found++;
        }
    }
    std::cout << env << std::endl;

    std::cout << "stations: " << std::endl;
    for (const auto& station : stations){
        std::cout << "Station " << station.first << ": " 
                  << station.second.ConvertCellToWorld(cell_size, cell_size) 
                  << std::endl;
    }

    Parameters params = Parameters();

    std::vector<std::string> starting_positions(nb_movers);
    for (int i = 0; i < nb_movers; i++){
        starting_positions[i] = std::to_string(i);
    }

    // Create tasks
    std::vector<MoverTask> tasks = {};
    std::vector<bool> occupied(nb_stations, false);
    std::vector<bool> newly_selected(nb_stations, false);
    for (int i = 0; i < nb_movers; i++){ occupied[i] = true;}

    // wave of motions
    for (int i = 0; i < nb_motion_waves; i++){

        // Add task for each mover
        for (int j = 0; j < nb_movers; j++){
            // pick a random station that is not occupied and not yet selected
            int station_idx = rand() % nb_stations;
            while (occupied[station_idx] || newly_selected[station_idx]){
                station_idx = rand() % nb_stations;
            }
            newly_selected[station_idx] = true;
            // tasks.push_back(MoverTask(j, std::to_string(station_idx), 2.0 * i + 0.05 * j));
            tasks.push_back(MoverTask(j, std::to_string(station_idx), i));

        }
        occupied = newly_selected;
        newly_selected = std::vector<bool>(nb_stations, false);        
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    params_list[0].SetAmax(0.5);
    params_list[1].SetVmax(0.5);
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
    // mms.PrintProfilerInfo();
}

void TestMultiMoverDeployment(int nb_stations=35, int nb_movers=10, 
        int nb_waves=3, bool add_file_name_appendix=false){
    // Set the stage
    Environment env = Environment();
    double cell_size = env.CellWidth();
    std::cout << env << std::endl;

    // set random seed
    // srand(time(0));
    srand(0);

    // Get stations at random boundary positions
    std::map<std::string, Point2D<int>> stations;
    Point2D<int> candidate;
    int nb_stations_found = 0;
    while (nb_stations_found < nb_stations){
        candidate = env.GetRandomFreeCellPositionAtEnvironmentEdge();
        // std::cout << "\ttesting " << candidate << std::endl;

        // check if candidate is not already in the map
        bool new_station = true;
        for (const auto& station : stations){
            if (station.second.Distance(candidate) < 0.1){
                new_station = false;
                break;
            }
        }

        // make sure candidate is not in a corner
        bool corner = false;
        if ((candidate.x() == 0 || candidate.x() == env.NbCellCols() - 1) &&
                (candidate.y() == 0 || candidate.y() == env.NbCellRows() - 1) ||
                (candidate.x() == 11 && candidate.y() == 3)){
            corner = true;
        }

        // let's remove any candidates on row 8 and column > 3
        if (candidate.y() == 8 && candidate.x() >= 3){
            corner = true;
        }

        if (new_station && !corner){
            stations[std::to_string(nb_stations_found)] = candidate;
            nb_stations_found++;
            std::cout << "added candidate station at " << candidate << std::endl;
        }
    }
    std::cout << env << std::endl;

    std::cout << "stations: " << std::endl;
    for (const auto& station : stations){
        std::cout << "Station " << station.first << ": " 
                  << station.second.ConvertCellToWorld(cell_size, cell_size) 
                  << std::endl;
    }

    Parameters params = Parameters();

    std::vector<std::string> starting_positions(nb_movers);
    for (int i = 0; i < nb_movers; i++){
        starting_positions[i] = std::to_string(i);
    }

    // Create tasks
    std::vector<MoverTask> tasks = {};
    std::vector<bool> occupied(nb_stations, false);
    std::vector<bool> newly_selected(nb_stations, false);
    for (int i = 0; i < nb_movers; i++){ occupied[i] = true;}

    // wave of motions
    for (int i = 0; i < nb_waves; i++){

        // Add task for each mover
        for (int j = 0; j < nb_movers; j++){
            // pick a random station that is not occupied and not yet selected
            int station_idx = rand() % nb_stations;
            while (occupied[station_idx] || newly_selected[station_idx]){
                station_idx = rand() % nb_stations;
            }
            newly_selected[station_idx] = true;
            // tasks.push_back(MoverTask(j, std::to_string(station_idx), 2.0 * i + 0.05 * j));
            // tasks.push_back(MoverTask(j, std::to_string(station_idx), i, 5.0 * i + 0.1 * j));
            tasks.push_back(MoverTask(j, std::to_string(station_idx), i));

        }
        occupied = newly_selected;
        newly_selected = std::vector<bool>(nb_stations, false);        
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }
    // params_list[0].SetAmax(1.0);
    // params_list[1].SetVmax(1.0);
    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    // mms.SetPerformanceMode(true);
    
    // Simulate
    int nb_steps = 3600;
    for (int i = 0; i < nb_steps; i++){
        std::cout << "Simulating step " << i << " ..." << std::endl;
        try{
            mms.SimulateSteps(1, false);
        } catch (std::exception &e){
            std::cout << "something went wrong at step " << i << ": " << e.what() << std::endl;
            std::cerr << e.what() << std::endl;
            // throw std::runtime_error("Multi-mover simulation failed.");
            break;
        }

        if (mms.AllTasksCompleted()){
            std::cout << "All tasks completed at step " << i << "!" << std::endl;
            break;
        }
    }

    std::string file_name = "output/multi_mover_simulator";
    if (add_file_name_appendix){
        file_name += "_" + std::to_string(nb_movers);
    }
    mms.DumpToJson(file_name + ".json");
    mms.PrintLog();
    // mms.PrintProfilerInfo();
}

void PerformMultiMoverScalingTest(){
    std::vector<int> nb_movers_list = {/*2, */5/*, 10, 15, 18*/};
    for (int nb_movers : nb_movers_list){
        std::cout << "Performing multi-mover deployment test with " 
                  << nb_movers << " movers..." << std::endl;
        TestMultiMoverDeployment(39, nb_movers, 20, true);
        // TestMultiMoverTasksWithStations(60, nb_movers, 4);
    }
}

void PerformRun(Environment& env, std::vector<Parameters*>& params_ptr_list,
                std::map<std::string, Point2D<int>>& stations,
                std::vector<std::string>& starting_positions,
                std::vector<MoverTask>& tasks,
                int nb_movers, int nb_stations, int nb_waves,
                int task_file_id, bool benchmarking_mode,
                bool ignore_collisions, bool ignore_waiting_points,
                bool limited_update_time,
                std::string file_name){

    std::cout << "Creating MultiMoverSimulator..." << std::endl;
    MultiMoverSimulator mms = MultiMoverSimulator(env, params_ptr_list, 
                                        stations, starting_positions, tasks);
    mms.SetIgnoreCollisions(ignore_collisions);
    mms.SetIgnoreWaitingPoints(ignore_waiting_points);
    if (limited_update_time){
        mms.SetMaxUpdateTrajectoryTime(1.0); // in ms
    }
    if (nb_movers > 15 || nb_stations > 25){
        mms.SetMaxNbCorridorGrowingIterations(5);
    }
    if (benchmarking_mode){
        mms.PrepareOptiInstances();
    }
    
    // Simulate
    int nb_steps = (nb_movers <= 15) ? 4000 : 4000;
    if (benchmarking_mode){file_name = "../build/" + file_name;}
    std::string file_name_appendix = "_" + std::to_string(nb_movers) + "_agents_" +
                                     std::to_string(nb_stations) + "_stations_" +
                                     std::to_string(nb_waves) + "_waves_" +
                                     std::to_string(task_file_id) + ".json";
    for (int i = 0; i < nb_steps; i++){
        if (i % 10 == 0) {std::cout << "Simulating step " << i << " ..." << std::endl;}
        try{
            mms.SimulateSteps(1, false);
        } catch (std::exception &e){
            std::cout << "something went wrong at step " << i << ": " << e.what() << std::endl;
            std::cerr << e.what() << std::endl;
            break;
        }

        if (mms.AllTasksCompleted()){
            std::cout << "All tasks completed at step " << i << "!" << std::endl;
            break;
        }
    }
    mms.DumpToJson(file_name + file_name_appendix);
    mms.PrintLog();
}

void PerformWaitingPointTest(int nb_movers=6, int nb_stations=23, 
                             int nb_waves=30, int task_file_id=0,
                             bool read_existing_task_file=false,
                             bool benchmarking_mode=false){
    Environment env = Environment();
    double cell_size = env.CellWidth();

    if (nb_movers > 15 || nb_stations > 25){
        env = GetLargeEnvironment();
        nb_stations = 168;
    }

    if (!read_existing_task_file){
        // prompt user to confirm
        std::cout << "Please confirm generation of a new task file by typing 'go'" << std::endl;
        std::string user_input;
        std::getline(std::cin, user_input);
        if (user_input != "go"){
            throw std::runtime_error("User did not confirm task file generation. Aborting.");
        }
        std::cout << "Generating new task file..." << std::endl;

        GenerateTaskFile(nb_movers, nb_stations, nb_waves, task_file_id, benchmarking_mode);
    }
    json taskFileJson = ReadTaskFile(nb_movers, nb_stations, nb_waves, task_file_id, benchmarking_mode);
    
    json stations_json = taskFileJson["stations"];
    std::map<std::string, Point2D<int>> stations;
    for (const auto& station_json : stations_json){
        std::string first = station_json["first"];
        Point2D<int> second = Point2D<int>(station_json["second"]["x"], station_json["second"]["y"]);
        stations[first] = second;
    }
    
    json tasks_json = taskFileJson["tasks"];
    std::vector<MoverTask> tasks = {};
    for (const auto& task_json : tasks_json){
        int agent_index = task_json["agent_idx"];
        std::string destination_name = task_json["destination_name"];
        int task_sequence_nb = task_json["task_sequence_nb"];
        tasks.push_back(MoverTask(agent_index, destination_name, task_sequence_nb));
    }

    json starting_positions_json = taskFileJson["starting_positions"];
    std::vector<std::string> starting_positions;
    for (const auto& pos_json : starting_positions_json){
        Point2D<double> pos = Point2D<double>(pos_json["x"], pos_json["y"]);
        Point2D<int> cell_pos = pos.ConvertWorldToCell(cell_size, cell_size);
        for (const auto& station : stations){
            if (station.second == cell_pos){
                starting_positions.push_back(station.first);
                break;
            }
        }
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<Parameters*> params_ptr_list;
    for (int i = 0; i < nb_movers; i++){
        params_list[i] = Parameters();
        params_ptr_list.push_back(&params_list[i]);
    }


    // write "0/4: to file
    std::string file_name = "output/benchmark_progress.txt";
    std::ofstream file(file_name, std::ios::app);
    file << "0/4" << std::endl;

    PerformRun(env, params_ptr_list, stations, starting_positions, tasks,
                nb_movers, nb_stations, nb_waves, task_file_id,
                benchmarking_mode, false, false, false,
                "output/multi_mover_simulator_normal_mode");
    file << "1/4" << std::endl;

    PerformRun(env, params_ptr_list, stations, starting_positions, tasks,
                nb_movers, nb_stations, nb_waves, task_file_id,
                benchmarking_mode, false, false, true,
                "output/multi_mover_simulator_one_by_one");
    file << "2/4" << std::endl;
    
    PerformRun(env, params_ptr_list, stations, starting_positions, tasks,
                nb_movers, nb_stations, nb_waves, task_file_id,
                benchmarking_mode, false, true, false,
                "output/multi_mover_simulator_ignore_waiting_points");
    file << "3/4" << std::endl;

    PerformRun(env, params_ptr_list, stations, starting_positions, tasks,
               nb_movers, nb_stations, nb_waves, task_file_id,
               benchmarking_mode, true, false, false,
               "output/multi_mover_simulator_ignore_collisions");
    file << "4/4" << std::endl;
    file.close();
}
#endif