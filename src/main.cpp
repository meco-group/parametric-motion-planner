#include <iostream>
#include <cmath>

// #include "core/motion_planner.hpp"
// #include "core/environment.hpp"
#include "parametric_motion_planner.hpp"

void SolveAllMethods(MotionPlanner &motion_planner, std::string const &filename){
    motion_planner.SetJustInTimePreparationMode(false);

	motion_planner.SetMethod(ARENA);
    motion_planner.SetCorridorExtendedMode(false);
    try{ motion_planner.Plan();} catch (std::exception &e){ std::cout << e.what() << std::endl; return;}
    motion_planner.DumpToJson(filename + "_arena.json");

    // double tf_arena = motion_planner.GetLastSolution().Tf();
    double tf_arena = motion_planner.GetTravelTime();
    double t_comp_total_arena = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_arena = motion_planner.GetLastSolution().SolverTime();
    double t_comp_corridors_arena = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_arena = motion_planner.GetLastSolution().SamplingTime();

    motion_planner.SetCorridorExtendedMode(true);
    motion_planner.SetMethod(OCP);
    try{ motion_planner.Plan();} catch (std::exception &e){ std::cout << e.what() << std::endl; return;}
    motion_planner.DumpToJson(filename + "_ocp.json");

    double tf_ocp = motion_planner.GetLastSolution().Tf();
    double t_comp_total_ocp = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_ocp = motion_planner.GetLastSolution().SolverTime();
    double t_comp_corridors_ocp = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_ocp = motion_planner.GetLastSolution().SamplingTime();

    motion_planner.SetMethod(P2P);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_p2p.json");

    double tf_p2p = motion_planner.GetLastSolution().Tf();
    double t_comp_total_p2p = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_p2p = motion_planner.GetLastSolution().SolverTime();
    double t_comp_corridors_p2p = motion_planner.GetCorridorSequenceConstructionTime();
    double t_sampling_p2p = motion_planner.GetLastSolution().SamplingTime();

    double t_comp_total_and_move_arena = 0.001*t_comp_total_arena + tf_arena;
    double t_comp_total_and_move_ocp = 0.001*t_comp_total_ocp + tf_ocp;

    std::cout << std::endl;
	std::cout << "==============================================================" << std::endl;
    std::cout << "ARENA summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_arena);
    printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_arena);
    printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_arena);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_arena);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_arena);

    std::cout << "OCP summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_ocp);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_ocp);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_ocp);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_ocp);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_ocp);

    std::cout << "P2P summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_p2p);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_p2p);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_p2p);
    printf("\tCorridor time: \t\t\t%.3f ms\n", t_comp_corridors_p2p);
    printf("\tSampling time: \t\t\t%.3f ms\n", t_sampling_p2p);
    std::cout << std::endl;

	std::cout << "Overall results:" << std::endl;
	printf("\tSuboptimality: \t%.3f %% (%.3f ms)\n", 100.0*(tf_arena - tf_ocp)/tf_arena, tf_arena - tf_ocp);
	printf("\tTotal speedup: \t%.3f\n", t_comp_total_ocp/t_comp_total_arena);
	printf("\tSolver speedup:\t%.3f\n", t_comp_solver_ocp/t_comp_solver_arena);
    if (t_comp_total_and_move_arena > t_comp_total_and_move_ocp){
        printf("\tARENA is %.3fs slower to destination\n", t_comp_total_and_move_arena - t_comp_total_and_move_ocp);
    } else {
        printf("\tARENA is %.3fs faster to destination\n", t_comp_total_and_move_ocp - t_comp_total_and_move_arena);
    }
	std::cout << "==============================================================" << std::endl;
	std::cout << std::endl;
}

void SolveRandomProblem(int nb_runs){
    // Environment environment = Environment();
    Environment environment = Environment(15, 15, 0.12, 0.12);
    // Environment environment = Environment(25, 25, 0.12, 0.12);
    Parameters params = Parameters();
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	environment.AddRandomObstacles(0.1);

    // std::vector<int> rr = {};
    // std::vector<int> cc = {};
    // for (int i = 0; i < environment.NbCellCols(); i++){
    //     for (int j = 0; j < environment.NbCellRows(); j++){
    //         if (!environment.IsFree(Point2D<int>(i, j))){
    //             rr.push_back(i);
    //             cc.push_back(j);
    //         }
    //     }
    // }
    // std::cout << "std::vector<int> rr_test = {";
    // for (int i = 0; i < rr.size(); i++){
    //     std::cout << rr[i];
    //     if (i < rr.size() - 1){
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "};" << std::endl;
    // std::cout << "std::vector<int> cc_test = {";
    // for (int i = 0; i < cc.size(); i++){
    //     std::cout << cc[i];
    //     if (i < cc.size() - 1){
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "};" << std::endl;

    std::cout << "Created motion planner in environment " << environment << std::endl;

    for (int i = 0; i < nb_runs; i++){
    environment.AddRandomObstacles(0.1);    
	// Point2D<double> start = Point2D<double>(1.74, 0.06);
    // Point2D<double> dest = Point2D<double>(0.06, 1.38);
    // Point2D<double> start = Point2D<double>(0.46368271444046694, 0.5541697302378621);
    // Point2D<double> dest = Point2D<double>(1.4481433377924342, 0.7777571510707014);
    // my_motion_planner.SetStart(start);
    // my_motion_planner.SetDest(dest);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetRandomStart();
    my_motion_planner.SetRandomDest();
    my_motion_planner.SetStartVel(start_vel);
    my_motion_planner.SetSuboptimalityEliminationFeature(true);

    SolveAllMethods(my_motion_planner, "solution");

    // wait for user to press spacebar
    if (i < nb_runs - 1){getchar();}
    }
}

void SolveFatropFailureCase(){
    // Environment environment = Environment();
    // Environment environment = Environment(10, 12, 0.12, 0.12);
    // Environment environment = Environment(20, 20, 0.12, 0.12);
    Environment environment = Environment(25, 25, 0.12, 0.12);
    // Parameters params = Parameters(1.7743775335572929, 2.2106078993268388, 0.115, 0.115, 0.001);
    Parameters params = Parameters(0.7696381510903181, 5.5568675872914195, 0.115, 0.115, 0.001);
    MotionPlanner my_motion_planner = MotionPlanner(params, environment);

	// environment.AddRandomObstacles(0.25);
    // std::vector<int> rr_test = {1, 2, 4, 6, 6, 7, 9, 9, 11, 12, 12, 13};
    // std::vector<int> cc_test = {1, 12, 10, 7, 13, 13, 11, 12, 0, 2, 6, 3};
    std::vector<int> rr_test = {1, 1, 6, 7, 9, 9, 10, 12};
    std::vector<int> cc_test = {0, 1, 5, 3, 10, 11, 9, 3};
    for (int i = 0; i < rr_test.size(); i++){
        environment.AddObstacle(Point2D<int>(rr_test[i], cc_test[i]));
    }

    std::vector<int> rr = {};
    std::vector<int> cc = {};
    for (int i = 0; i < environment.NbCellCols(); i++){
        for (int j = 0; j < environment.NbCellRows(); j++){
            if (!environment.IsFree(Point2D<int>(i, j))){
                rr.push_back(i);
                cc.push_back(j);
            }
        }
    }
    std::cout << "std::vector<int> rr_test = {";
    for (int i = 0; i < rr.size(); i++){
        std::cout << rr[i];
        if (i < rr.size() - 1){
            std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    std::cout << "std::vector<int> cc_test = {";
    for (int i = 0; i < cc.size(); i++){
        std::cout << cc[i];
        if (i < cc.size() - 1){
            std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;

    std::cout << "Created motion planner in environment " << environment << std::endl;

	// Point2D<double> start = Point2D<double>(1.74, 0.06);
    // Point2D<double> dest = Point2D<double>(0.06, 1.38);
    Point2D<double> start = Point2D<double>(0.46368271444046694, 0.5541697302378621);
    Point2D<double> dest = Point2D<double>(1.4481433377924342, 0.7777571510707014);
    Point2D<double> start_vel = Point2D<double>(0, 0);

    my_motion_planner.SetStart(start);
    my_motion_planner.SetDest(dest);
    my_motion_planner.SetStartVel(start_vel);
    my_motion_planner.SetSuboptimalityEliminationFeature(true);

    // my_motion_planner.SetRandomStart();
    // my_motion_planner.SetRandomDest();
    SolveAllMethods(my_motion_planner, "solution");
}

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
            trajectories[i] = planners[i]->GetLastSolution();
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
            trajectories[i] = planners[i]->GetLastSolution();
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
void TestMultiMoverTasksWithStations(){
    // Set the stage
    double cell_size = 0.12;
    Environment env = Environment(6, 6, cell_size, cell_size);
    // env.AddRandomObstacles(0.1);

    // set random seed
    srand(time(0));

    // Get stations at random boundary positions
    int nb_stations = 7;
    std::map<std::string, Point2D<int>> stations;
    Point2D<int> candidate;
    int nb_stations_found = 0;
    while (nb_stations_found < nb_stations){
        candidate = env.GetRandomFreeCellPositionAtEnvironmentEdge();

        // check if candidate is not already in the map
        bool new_station = true;
        for (const auto& station : stations){
            if (station.second.Distance(candidate) < 0.1){
                new_station = false;
                break;
            }
        }

        if (new_station){
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
    int nb_movers = 3;

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
    for (int i = 0; i < 4; i++){

        // Add task for each mover
        for (int j = 0; j < nb_movers; j++){
            // pick a random station that is not occupied and not yet selected
            int station_idx = rand() % nb_stations;
            while (occupied[station_idx] || newly_selected[station_idx]){
                station_idx = rand() % nb_stations;
            }
            newly_selected[station_idx] = true;
            tasks.push_back(MoverTask(j, std::to_string(station_idx), 1.0 * i + 0.05 * j));

        }
        occupied = newly_selected;
        newly_selected = std::vector<bool>(nb_stations, false);        
    }

    // Create simulator
    std::vector<Parameters> params_list(nb_movers);
    std::vector<const Parameters*> params_ptr_list;
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
    }
}

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
    std::vector<const Parameters*> params_ptr_list;
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
    std::vector<const Parameters*> params_ptr_list;
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


int main(int argc, char *argv[]){
    int nb_runs = 1;
    if (argc == 3 && std::strcmp(argv[1], "nb_runs") == 0){
        nb_runs = std::max(1, atoi(argv[2]));
    }
    // SolveRandomProblem(nb_runs);
    // SolveDynamicProblem();
    // TestRandomVehiclePositions();
    // SolveFatropFailureCase();
    // SwitchDestinationCarrotStyle();
    // SwitchDestinationCarrotStyleUsingSampler();
    // AvoidSuddenAndLateObstacle();
    // TestTrajectoryCollisionCheck();
    // TestCollisionResolution();
    // TestEmergencyBraking();
    // TestCorridorTimeDimension();
    // TestDynamicIntersections();
    // TestMultiMoverSimulator();
    // TestMultiMoverTasksWithStationsOld();
    TestMultiMoverTasksWithStations();
    // TestDeadlockScenario();
    // TestDeadlockScenario2();
}
