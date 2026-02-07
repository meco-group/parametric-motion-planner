#ifndef __INITIAL_MULTI_MOVER_TESTS__
#define __INITIAL_MULTI_MOVER_TESTS__

#include <iostream>
#include <chrono>
#include <cmath>
#include "../motion_planner.hpp"
#include "../dynamic_simulator.hpp"
#include "../dynamic_sampler.hpp"

void SolveDynamicProblem(){
    // Environment environment = Environment();
    // Environment environment = Environment(10, 12, 0.12, 0.12);
    // std::cout << "Created environment " << environment << std::endl;
    // Parameters params = Parameters();
    // MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    // DynamicSimulator dynamic_simulator = DynamicSimulator(environment, my_motion_planner);

    // params.SetVmax(1.0);

    // // Create moving obstacles
    // double width = environment.CellWidth();
    // double height = environment.CellHeight();
    // std::vector<Point2D<double>> starting_positions = {
    //     Point2D<int>(3, 9).ConvertCellToWorld(width, height), 
    //     Point2D<int>(8, 0).ConvertCellToWorld(width, height), 
    // };
    // std::vector<Point2D<double>> ending_positions = {
    //     Point2D<int>(3, -1).ConvertCellToWorld(width, height), 
    //     Point2D<int>(8, 5).ConvertCellToWorld(width, height), };
    // std::vector<double> durations = {0.8, 1.65, 1.6};
    // std::vector<bool> loops = {false, true, true};
    // std::shared_ptr<MovingObstacle> moving_obstacle_ptr;
    // for (int i = 0; i < starting_positions.size(); i++){
    //     moving_obstacle_ptr = std::make_shared<LinearMovingObstacle>(0.1, 0.1,
    //             starting_positions[i], ending_positions[i], durations[i], loops[i]);
    //     dynamic_simulator.AddMovingObstacle(moving_obstacle_ptr);
    // }

    // // Create appearing obstacle
    // std::vector<Point2D<double>> appearing_positions = {
    //     Point2D<int>(10, 4).ConvertCellToWorld(width, height), 
    //     Point2D<int>(7, 2).ConvertCellToWorld(width, height), 
    //     Point2D<int>(9, 0).ConvertCellToWorld(width, height), 
    //     Point2D<int>(9, 1).ConvertCellToWorld(width, height), 
    //     Point2D<int>(9, 2).ConvertCellToWorld(width, height), 
    //     Point2D<int>(9, 3).ConvertCellToWorld(width, height), 
    // };
    // std::vector<double> appearance_times = {2.45, 1.2, 1.4, 1.4, 1.4, 1.4};
    // std::vector<double> disappearance_times = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    // std::shared_ptr<MovingObstacle> appearing_obstacle_ptr;
    // for (int i = 0; i < appearing_positions.size(); i++){
    //     appearing_obstacle_ptr = 
    //         std::make_shared<AppearingStaticObstacle>(0.1, 0.1 + 0.6*(i==2),
    //             appearing_positions[i], appearance_times[i], 
    //             disappearance_times[i]);
    //     dynamic_simulator.AddMovingObstacle(appearing_obstacle_ptr);
    // }

    // my_motion_planner.SetMethod(OCP);
    // Point2D<double> start(0.15, 1.08);
    // Point2D<double> dest(1.33, 0.24);
    // Point2D<double> start_vel(0, 0);
    // dynamic_simulator.Plan(start, dest, start_vel);

    // dynamic_simulator.DumpToJson("dynamic_solution_ocp.json");

    Environment environment = Environment(5, 12, 0.12, 0.12);
    std::cout << "Created environment " << environment << std::endl;
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    DynamicSimulator dynamic_simulator = DynamicSimulator(environment, my_motion_planner);

    params.SetVmax(1.0);

    double width = environment.CellWidth();
    double height = environment.CellHeight();

    // Create moving obstacles
    // ...

    // Create appearing obstacle
    std::vector<Point2D<double>> appearing_positions = {
        Point2D<int>(4, 2).ConvertCellToWorld(width, height), 
        Point2D<double>(1.08, 0.12),
        // Point2D<int>(9, 2).ConvertCellToWorld(width, height),
        // Point2D<double>(1.14, 0.48),
        Point2D<double>(1.08, 0.36),
    };
    std::vector<double> appearance_times = {0.22, 0.5, 0.75}; //0.35
    std::vector<double> disappearance_times = {10000.0, 10000.0, 10000.0};
    std::shared_ptr<MovingObstacle> appearing_obstacle_ptr;
    for (int i = 0; i < appearing_positions.size(); i++){
        appearing_obstacle_ptr = 
            std::make_shared<AppearingStaticObstacle>(
                0.1*(i==0) + (2*width-0.01)*(i==1) + (2*width-0.01)*(i==2), 
                0.1*(i==0) + (2*width-0.01)*(i==1) + (2*width-0.01)*(i==2),
                appearing_positions[i], appearance_times[i], 
                disappearance_times[i]);
        dynamic_simulator.AddMovingObstacle(appearing_obstacle_ptr);
    }

    /// Simulate ///
    Point2D<double> start = Point2D<int>(0, 2).ConvertCellToWorld(width, height);
    Point2D<double> dest = Point2D<int>(11, 2).ConvertCellToWorld(width, height);
    Point2D<double> start_vel(0, 0);

    // ARENA
    my_motion_planner.SetMethod(ARENA);
    dynamic_simulator.Plan(start, dest, start_vel);
    dynamic_simulator.DumpToJson("dynamic_solution_ARENA.json");
    dynamic_simulator.DumpToJson("dynamic_solution.json");

    // OCP
    my_motion_planner.SetMethod(OCP);
    dynamic_simulator.Reset();
    dynamic_simulator.Plan(start, dest, start_vel);
    dynamic_simulator.DumpToJson("dynamic_solution_ocp.json");

}

void SwitchDestinationCarrotStyle(){
    Environment environment = Environment();
    environment.AddRandomObstacles(0.05);
    // Environment environment = Environment(18, 26, 0.12, 0.12);
    // environment.AddRandomObstacles(0.2);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    my_motion_planner.SetJustInTimePreparationMode(false);

    DynamicSimulator dynamic_simulator = DynamicSimulator(environment, my_motion_planner);
    Point2D<double> start;
    environment.GetRandomFreeVehiclePosition(start, params.GetVehWidth(), 
                                             params.GetVehHeight(), 
                                             params.GetMargin());
    Point2D<double> start_vel(0,0);

    try{
        dynamic_simulator.MoveDestination(start, start_vel, 15);
        dynamic_simulator.DumpToJson("dynamic_solution_movable_destination.json");
    } catch (std::exception &e){
        std::cerr << e.what() << std::endl;
        dynamic_simulator.PrintMotionPlannerLog();
    }
}

void SwitchDestinationCarrotStyleUsingSampler(){
    Environment environment = Environment();
    std::vector<int> rr_test = {};
    std::vector<int> cc_test = {};
    for (int i = 0; i < rr_test.size(); i++){
        environment.AddObstacle(Point2D<int>(rr_test[i], cc_test[i]));
    }
    // environment.AddRandomObstacles(0.1);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    my_motion_planner.SetMethod(OCP);
    // my_motion_planner.SetSilentMode(true);
    my_motion_planner.SetJustInTimePreparationMode(false);

    DynamicSampler dynamic_sampler = DynamicSampler(environment, my_motion_planner);
    dynamic_sampler.AddInitialization(true);
    dynamic_sampler.AddBasicReplanningTriggers();
    dynamic_sampler.MoveDestinationDemo(20, true);

    try{
    Point2D<double> curr_pos, curr_vel, curr_acc;
    bool finished = false;
    while (!finished){
        finished = dynamic_sampler.GetSample(curr_pos, curr_vel, curr_acc);
    }
    } catch (std::exception &e){
        std::cerr << e.what() << std::endl;
    }

    dynamic_sampler.DumpToJson("dynamic_solution_movable_destination_sampler.json");
}

void AvoidSuddenAndLateObstacle(){
    Environment environment = Environment();
    std::vector<int> rr_test = {};
    std::vector<int> cc_test = {};
    for (int i = 0; i < rr_test.size(); i++){
        environment.AddObstacle(Point2D<int>(rr_test[i], cc_test[i]));
    }
    // environment.AddRandomObstacles(0.1);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);
    // my_motion_planner.SetMethod(OCP);
    // my_motion_planner.SetSilentMode(true);
    my_motion_planner.SetJustInTimePreparationMode(false);

    DynamicSampler dynamic_sampler = DynamicSampler(environment, my_motion_planner);
    dynamic_sampler.AddInitialization();
    dynamic_sampler.AddBasicReplanningTriggers();
    dynamic_sampler.SuddenObstacleDemo(0.65, Point2D<int>(3, 7));

    try{
    Point2D<double> curr_pos, curr_vel, curr_acc;
    bool finished = false;
    int sample_counter = 0;
    int buffer_size = 15;
    while (!finished){
        auto start = std::chrono::high_resolution_clock::now();
        finished = dynamic_sampler.GetSample(curr_pos, curr_vel, curr_acc);
        sample_counter++;
        auto end = std::chrono::high_resolution_clock::now();
        while (sample_counter > buffer_size && std::chrono::duration<double, std::milli>(end - start).count() < 10.0){
            end = std::chrono::high_resolution_clock::now();
        }
    }
    } catch (std::exception &e){
        std::cerr << e.what() << std::endl;
    }

    dynamic_sampler.DumpToJson("dynamic_solution_sudden_obstacle_sampler.json");
}


#endif