#ifndef __TESTS__
#define __TESTS__
#include <iostream>
#include <cmath>

#include "../motion_planner.hpp"
#include "../dynamic_intersection_manager.hpp"

void TestRandomVehiclePositions(){
    int N = 100;

    Environment environment = Environment(10, 12, 0.12, 0.12);
    Parameters params = Parameters();

    environment.AddRandomObstacles(0.10);

    json random_positions_json;
    random_positions_json["Environment"] = environment.ToJson();
    random_positions_json["Positions"] = json::array();
    random_positions_json["Valid"] = json::array();
    
    Point2D<double> pos;
    for (int i = 0; i < N; i++){
        std::cout << std::endl << "Generating position i = " << i << std::endl;
        environment.GetRandomFreeVehiclePosition(pos, 0.115, 0.115, 0.001);
        random_positions_json["Positions"].push_back(pos.ToJson());
        random_positions_json["Valid"].push_back(environment.isValidVehiclePosition(pos, 0.115, 0.115, 0.001));
    }

    std::ofstream outFile("output/random_positions.json");
    outFile << random_positions_json.dump(4);
    outFile.close();
}

int GetMaxNbCollisions(int n){return n*n-n*(n+1)/2;};

void TestTrajectoryCollisionCheck(){
    Environment environment = Environment();
    Parameters params = Parameters();
    int nb_of_planners = 2;

    std::vector<std::unique_ptr<MotionPlanner>> planners;
    std::vector<Trajectory> trajectories(nb_of_planners);
    std::vector<Point2D<int>> starts = {Point2D<int>(5, 0), Point2D<int>(0, 9), Point2D<int>(7, 0), Point2D<int>(11, 0)};
    std::vector<Point2D<int>> dests = {Point2D<int>(5, 9), Point2D<int>(2, 0), Point2D<int>(11, 3), Point2D<int>(7, 3)};
    for (int i = 0; i < nb_of_planners; i++){
        // planners.push_back(std::make_unique<MotionPlanner>(MotionPlanner(params, environment)));
        planners.emplace_back(std::make_unique<MotionPlanner>(params, environment));
        planners[i]->SetStart(starts[i].ConvertCellToWorld(environment.CellWidth(), environment.CellHeight()));
        planners[i]->SetDest(dests[i].ConvertCellToWorld(environment.CellWidth(), environment.CellHeight()));
    }

    std::vector<Point2D<double>> points_of_collision(GetMaxNbCollisions(nb_of_planners));
    int point_ptr = 0;
    bool ready = false;
    int counter = 0;
    Point2D<double> pos_i, pos_j;
    double collision_time;

    json trajectory_collision_check;
    // every element is a list of vehicles (motion planners) at that iteration
    // trajectory_collision_check["vehicle_planners"] = json::array();
    // every element is a list of collision points at that iteration
    // trajectory_collision_check["point_of_collision"] = json::array(); 
    trajectory_collision_check["iterations"] = json::array();

    while (!ready && counter < 4){
        json iteration;
        // plan for all vehicles
        for (int i = 0; i < nb_of_planners; i++){
            if (counter == 3 && i == 3){
                planners[i]->PlanConcatenatedSections();
            } else {
                planners[i]->PlanSafely();
            }
            // std::cout << planners[i]->ToJson() << std::endl;
            trajectories[i] = *(planners[i]->GetLastSolution());
        }

        // store info in json       
        iteration["vehicle_planners"] = json::array();
        for (int i = 0; i < nb_of_planners; i++){
            iteration["vehicle_planners"].push_back(planners[i]->ToJson());
        }

        // check all possible collisions
        ready = true;
        point_ptr = 0;
        for (int i = 0; i < nb_of_planners; i++){
            for (int j = i+1; j < nb_of_planners; j++){
                trajectories[i].CheckCollision(trajectories[j], params, params, 
                    points_of_collision[point_ptr], pos_i, pos_j, collision_time);
                std::cout << "collision point: " << points_of_collision[point_ptr] << std::endl;
                if (environment.isValidPosition(points_of_collision[point_ptr])){
                    ready = false;
                    environment.AddObstacle(
                        points_of_collision[point_ptr].ConvertWorldToCell(
                            environment.CellWidth(), environment.CellHeight()));
                    std::cout << environment << std::endl;
                }
                point_ptr++;
            }
        }
        counter++;

        // store info in json       
        for (int i = 0; i < points_of_collision.size(); i++){
            iteration["point_of_collision"].push_back(points_of_collision[i].ToJson());
        }

        trajectory_collision_check["iterations"].push_back(iteration);
    }

    std::ofstream outFile("output/trajectory_collision_check.json");
    outFile << trajectory_collision_check.dump(4);
    outFile.close();
}

void TestCollisionResolution(){
    int nb_of_planners = 3;

    bool RANDOMIZE = true;

    std::vector<Parameters> params(nb_of_planners, Parameters());
    std::vector<std::shared_ptr<Environment>> environments;
    std::vector<std::unique_ptr<MotionPlanner>> planners;
    std::vector<Trajectory> trajectories(nb_of_planners);
    std::vector<Point2D<int>> starts = {Point2D<int>(6, 3), Point2D<int>(0, 9), 
        Point2D<int>(7, 0), Point2D<int>(11, 2), Point2D<int>(7, 8), Point2D<int>(0, 0)};
    std::vector<Point2D<int>> dests = {Point2D<int>(7, 9), Point2D<int>(2, 0), 
        Point2D<int>(11, 1), Point2D<int>(7, 1), Point2D<int>(1, 9), Point2D<int>(0, 3)};
    Point2D<double> start, dest;
    bool good_start_dest_found = false;
    Environment test_env = Environment();
    std::unordered_set<Point2D<int>, Point2DHash<int>> occupied_cells;
    double w, h, m;
    for (int i = 0; i < nb_of_planners; i++){
        environments.emplace_back(std::make_shared<Environment>());
        if (!RANDOMIZE){ environments[i]->AddObstacle(Point2D<int>(0, 2));}
        planners.emplace_back(std::make_unique<MotionPlanner>(params[i], *environments[i]));

        w = params[i].GetVehWidth(); h = params[i].GetVehHeight(); m = params[i].GetMargin();
        if (RANDOMIZE){
            good_start_dest_found = false;
            while (!good_start_dest_found){
                test_env.GetRandomFreeVehiclePosition(start, w, h, m);
                test_env.GetRandomFreeVehiclePosition(dest, w, h, m);
                good_start_dest_found = 
                    start.Distance(dest) > 0.6;
            }
            // Add occupied starting and ending positions
            planners[i]->SetStart(start); planners[i]->SetDest(dest);
            occupied_cells = test_env.GetOccupiedFootprintCells(start, w, h, m);
            for (auto &cell : occupied_cells){
                test_env.AddObstacle(cell);
            }
            occupied_cells = test_env.GetOccupiedFootprintCells(dest, w, h, m);
            for (auto &cell : occupied_cells){
                test_env.AddObstacle(cell);
            }
        } else {
            planners[i]->SetStart(starts[i].ConvertCellToWorld(
                environments[i]->CellWidth(), environments[i]->CellHeight()));
            planners[i]->SetDest(dests[i].ConvertCellToWorld(
                environments[i]->CellWidth(), environments[i]->CellHeight()));
        }
    }

    if (!RANDOMIZE){
        params[4].SetVmax(0.5);
        params[5].SetAmax(3);
    }

    std::vector<Point2D<double>> points_of_collision(GetMaxNbCollisions(nb_of_planners));
    int point_ptr = 0;
    bool ready = false;
    int counter = 0;
    json collision_info;
    Point2D<double> pos_at_collision_i, pos_at_collision_j, pos_temp;
    double collision_time, separation_angle, double_temp;
    int collision_veh_i, collision_veh_j;

    json trajectory_collision_check;
    trajectory_collision_check["iterations"] = json::array();
    std::vector<double> waiting_times(nb_of_planners, 0.0);

    while (!ready && counter < 4){
        json iteration;
        // plan for all vehicles
        for (int i = 0; i < nb_of_planners; i++){
            planners[i]->PlanSafely();
            planners[i]->InsertInitialWaitingTime(waiting_times[i]);
            trajectories[i] = *(planners[i]->GetLastSolution());
        }

        // store info in json       
        iteration["vehicle_planners"] = json::array();
        for (int i = 0; i < nb_of_planners; i++){
            iteration["vehicle_planners"].push_back(planners[i]->ToJson());
        }

        // let "collisions" be an array of dictionaries
        iteration["collisions"] = json::array();

        // check all possible collisions
        ready = true;
        point_ptr = 0;
        for (int i = 0; i < nb_of_planners; i++){
            for (int j = i+1; j < nb_of_planners; j++){
                if (trajectories[i].CheckCollision(
                        trajectories[j], params[i], params[j],
                        points_of_collision[point_ptr], pos_at_collision_i, 
                        pos_at_collision_j, collision_time)){
                    // collision between vehicle i and j is detected

                    if (planners[i]->AreSequencesSeparable(*planners[j], 
                            points_of_collision[point_ptr], separation_angle)){
                        // if the corridors are separable, resolve by restricting the free space

                        planners[i]->SeparateVehicleFreeSpace(*planners[j], 
                            points_of_collision[point_ptr], pos_at_collision_i,
                            pos_at_collision_j, separation_angle);
                        ready = false;
                    } else {
                        // resolve by waiting
                        double waiting_time_i = trajectories[i].GetWaitingTimeThis(trajectories[j], params[i], params[j]);
                        double waiting_time_j = trajectories[j].GetWaitingTimeThis(trajectories[i], params[j], params[i]);
                        std::cout << "case i: " << waiting_time_i << " " << trajectories[j].Tf() << std::endl;
                        std::cout << "case j: " << waiting_time_j << " " << trajectories[i].Tf() << std::endl;

                        // check if we're not waiting indefinetly
                        if (waiting_time_i >= trajectories[j].Tf() && waiting_time_j >= trajectories[i].Tf()){
                            // no point in waiting, we need to add obstacles
                            std::cout << "HERE!" << std::endl;
                            occupied_cells = environments[i]->GetOccupiedFootprintCells(planners[j]->GetDest(), w, h, m);
                            for (auto &cell : occupied_cells){
                                environments[i]->AddVirtualObstacle(cell);
                                
                            }
                            // occupied_cells = environments[j]->GetOccupiedFootprintCells(planners[i]->GetDest(), w, h, m);
                            // for (auto &cell : occupied_cells){
                            //     environments[j]->AddVirtualObstacle(cell);
                            // }
                            // waiting_times[i] = 0.0;
                            // waiting_times[j] = 0.0;

                        } else if (waiting_time_i < waiting_time_j){
                            waiting_times[i] += waiting_time_i;
                        } else {
                            waiting_times[i] += waiting_time_j;
                        }
                        ready = false;
                    }

                    // store collision info
                    collision_info.clear();
                    collision_info["point_of_collision"] = points_of_collision[point_ptr].ToJson();
                    collision_info["vehicle_positions"] = {pos_at_collision_i.ToJson(), pos_at_collision_j.ToJson()};
                    collision_info["collision_time"] = collision_time;
                    collision_info["vehicle_indices"] = {i, j};
                    iteration["collisions"].push_back(collision_info);
                }
                point_ptr++;
            }
        }
        trajectory_collision_check["iterations"].push_back(iteration);

        counter++;
    }


    // // add updated trajectories to json
    // json next_iteration;
    // next_iteration["vehicle_planners"] = json::array();
    // for (int i = 0; i < nb_of_planners; i++){
    //     next_iteration["vehicle_planners"].push_back(planners[i]->ToJson());
    // }
    // next_iteration["point_of_collision"] = json::array();
    // trajectory_collision_check["iterations"].push_back(next_iteration);

    std::ofstream outFile("output/trajectory_collision_check.json");
    outFile << trajectory_collision_check.dump(4);
    outFile.close();
}

void TestEmergencyBraking(){
    Environment environment = Environment(18, 26, 0.12, 0.12);
    std::vector<int> rr = {0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 17, 17, 17, 17, 18, 18, 19, 19, 19, 20, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25};
    std::vector<int> cc = {5, 6, 16, 3, 13, 14, 15, 17, 3, 4, 6, 13, 16, 2, 3, 8, 10, 13, 3, 4, 6, 7, 2, 3, 5, 6, 7, 11, 12, 4, 9, 10, 13, 15, 0, 4, 14, 17, 8, 9, 13, 16, 3, 4, 7, 9, 12, 6, 7, 10, 16, 5, 4, 10, 7, 11, 7, 10, 3, 7, 16, 1, 3, 5, 8, 6, 13, 7, 11, 14, 13, 0, 2, 15, 2, 4, 5, 7, 9, 12, 13, 17, 8, 11, 14, 9, 17, 8, 13};
    for (int i = 0; i < rr.size(); i++){ environment.AddObstacle(Point2D<int>(rr[i], cc[i]));}

    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

    my_motion_planner.SetStart(
        Point2D<double>(2.09169, 0.77861));
    my_motion_planner.SetStartVel(Point2D<double>(-0.343122, -0.0363521));
    my_motion_planner.ComputeEmergencyBrakingTrajectory();
}

void TestCorridorTimeDimension(){
    Environment environment = Environment(20, 20, 0.12, 0.12);
    environment.AddRandomObstacles(0.25);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    Point2D<double> start, dest;
    double max_obst_density = 0.25;

    int nb_runs = 500;
    for (int i = 0; i < nb_runs; i++){
        std::cout << "setting start, dest and start vel" << std::endl;

        // set the obstacle density as a random number between 0 and max_obst_density
        environment.AddRandomObstacles(
            ((double) rand() / (RAND_MAX)) * max_obst_density);
        params.SetAmax(2.0 + ((double) rand() / (RAND_MAX)) * 4.0);
        params.SetVmax(0.5 + ((double) rand() / (RAND_MAX)) * 1.5);

        environment.GetRandomFreeVehiclePosition(start,
            params.GetVehWidth(), params.GetVehHeight(), params.GetMargin());
        environment.GetRandomFreeVehiclePosition(dest,
            params.GetVehWidth(), params.GetVehHeight(), params.GetMargin());
        my_motion_planner.SetStart(start);
        my_motion_planner.SetDest(dest);
        my_motion_planner.SetStartVel(Point2D<double>(0,0));
        my_motion_planner.PlanSafely();
        my_motion_planner.DumpToJson("example_trajectory" + str(i) + ".json");
    }
}

void TestDynamicIntersections(){
    double cw = 0.12;
    double ch = 0.12;

    // // first mover
    // Environment env1 = Environment(8, 8, cw, ch);
    // std::vector<int> xx1 = {3, 4, 6, 6, 6, 6, 6, 7, 7};
    // std::vector<int> yy1 = {0, 1, 3, 4, 0, 1, 6, 4, 7};
    // for (int i = 0; i < xx1.size(); i++){
    //     env1.AddObstacle(Point2D<int>(xx1[i], yy1[i]));
    // }
    // Parameters params1 = Parameters();
    // // params1.SetAmax(1.1);
    // MotionPlanner planner1 = MotionPlanner(params1, env1);
    // planner1.SetMaxNbCorridorGrowingIterations(0);
    // Point2D<double> start1 = Point2D<int>(4, 0).ConvertCellToWorld(cw, ch);
    // Point2D<double> dest1 = Point2D<int>(7, 6).ConvertCellToWorld(cw, ch);
    // Point2D<double> start_vel1 = Point2D<double>(0, 0);

    // // second mover
    // Environment env2 = Environment(8, 8, cw, ch);
    // std::vector<int> xx2 = {3, 3, 4, 5, 5, 5, 6, 6, 7, 7};
    // std::vector<int> yy2 = {7, 6, 5, 7, 4, 3, 6, 2, 4, 1};
    // for (int i = 0; i < xx2.size(); i++){
    //     env2.AddObstacle(Point2D<int>(xx2[i], yy2[i]));
    // }
    // Parameters params2 = Parameters();
    // MotionPlanner planner2 = MotionPlanner(params2, env2);
    // planner2.SetMaxNbCorridorGrowingIterations(0);
    // std::cout << "env2: " << env2 << std::endl;
    // Point2D<double> start2 = Point2D<int>(4, 7).ConvertCellToWorld(cw, ch);
    // Point2D<double> dest2 = Point2D<int>(7, 2).ConvertCellToWorld(cw, ch);
    // Point2D<double> start_vel2 = Point2D<double>(0, 0);

    // first mover
    Environment env = Environment(8, 8, cw, ch);
    env.AddRandomObstacles(0.05);
    Parameters params = Parameters();
    
    MotionPlanner planner1 = MotionPlanner(params, env);
    planner1.SetMaxNbCorridorGrowingIterations(1);

    MotionPlanner planner2 = MotionPlanner(params, env);
    planner2.SetMaxNbCorridorGrowingIterations(1);

    Point2D<double> start1, dest1, start_vel1;
    Point2D<double> start2, dest2, start_vel2;

    env.GetRandomFreeCellPosition(start1);
    env.GetRandomFreeCellPosition(dest1);
    env.GetRandomFreeCellPosition(start2);
    env.GetRandomFreeCellPosition(dest2);
    start_vel1 = Point2D<double>(0, 0);
    start_vel2 = Point2D<double>(0, 0);   

    // Simulate movers and store results
    DynamicIntersectionManager intersection_manager = 
        DynamicIntersectionManager(planner1, planner2);
    try{
        intersection_manager.SimulateSafely(start1, dest1, start_vel1, 
            start2, dest2, start_vel2);
    } catch (std::exception &e){
        std::cout << "something went wrong: " << e.what() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    intersection_manager.DumpToJson("output/dynamic_intersection.json");
}

#endif