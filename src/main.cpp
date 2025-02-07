#include <iostream>
#include <cmath>

// #include "core/motion_planner.hpp"
// #include "core/environment.hpp"
#include "parametric_motion_planner.hpp"

void SolveAllMethods(MotionPlanner &motion_planner, std::string const &filename){
	motion_planner.SetMethod(ARENA);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_arena.json");

    // double tf_arena = motion_planner.GetLastSolution().Tf();
    double tf_arena = motion_planner.GetTravelTime();
    double t_comp_total_arena = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_arena = motion_planner.GetLastSolution().SolverTime();

    motion_planner.SetMethod(OCP);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_ocp.json");

    double tf_ocp = motion_planner.GetLastSolution().Tf();
    double t_comp_total_ocp = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_ocp = motion_planner.GetLastSolution().SolverTime();

    motion_planner.SetMethod(P2P);
    motion_planner.Plan();
    motion_planner.DumpToJson(filename + "_p2p.json");

    double tf_p2p = motion_planner.GetLastSolution().Tf();
    double t_comp_total_p2p = motion_planner.GetLastSolution().TotalComputationTime();
    double t_comp_solver_p2p = motion_planner.GetLastSolution().SolverTime();

    std::cout << std::endl;
	std::cout << "==============================================================" << std::endl;
    std::cout << "ARENA summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_arena);
    printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_arena);
    printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_arena);

    std::cout << "OCP summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_ocp);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_ocp);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_ocp);

    std::cout << "P2P summary: " << std::endl;
	printf("\tTf: \t\t\t\t%.3f s\n", tf_p2p);
	printf("\tTotal computation time: \t%.3f ms\n", t_comp_total_p2p);
	printf("\tSolver time: \t\t\t%.3f ms\n", t_comp_solver_p2p);
    std::cout << std::endl;

	std::cout << "Overall results:" << std::endl;
	printf("\tSuboptimality: \t%.3f %% (%.3f ms)\n", 100.0*(tf_arena - tf_ocp)/tf_arena, tf_arena - tf_ocp);
	printf("\tTotal speedup: \t%.3f\n", t_comp_total_ocp/t_comp_total_arena);
	printf("\tSolver speedup:\t%.3f\n", t_comp_solver_ocp/t_comp_solver_arena);
	std::cout << "==============================================================" << std::endl;
	std::cout << std::endl;
}

void SolveRandomProblem(){
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

int main(){
    // SolveRandomProblem();
    // SolveDynamicProblem();
    // TestRandomVehiclePositions();
    // SolveFatropFailureCase();
    // SwitchDestinationCarrotStyle();
    // SwitchDestinationCarrotStyleUsingSampler();
    // AvoidSuddenAndLateObstacle();
    // TestTrajectoryCollisionCheck();
    TestCollisionResolution();
    // TestEmergencyBraking();
}


/*
# TODO: fix these cases


// CASE THAT WORKS IN PYTHON AND SEEMS TO BE FIXED IF THE POSITION CONSTRAINT 
// RELAXATION IS REMOVED (INITIALIZATION SEEMS TO BE SLIGHTLY DIFFERENT...)

Created motion planner in environment 20 x 20 environment (2.4 x 2.4)
# . . . . . . # # . # . . . . . . . . . 
# # . . . . # # . . # . . # . . . . . . 
. . . . # # # . # . . . . . # . . # . # 
. . . . . . . # # # . . . . . . . . . . 
# # . # # . . . # . . . . . . . # # . # 
. . . . . . . . . . . . # . # # . . . # 
. . . . . . . . # . . . # # # # # # . . 
. . # . . . . # # . # . . . . . # # . . 
. . . . . # . . . . . # . # . . # . . . 
. . . . . . . . . # . # . # # . # . # . 
. # # . # . # # . . . . . . . . . . . # 
. . # # . . # # . . . . # # # . . . . . 
. . . . . . . . . . . # . . # . . # . . 
. . . . . . . . . # # . . . . . . # . . 
. . . . # . . # # . . . . . # . # # . # 
. . . # # . . . . . # . . . . . . . . . 
. . . . . . . . # . . . . # . . . . . . 
. . . . . . . . # . # # . . . # . . . . 
# . . . . . # . . . . # . # # . # . # # 
. . . . . . . # . . . . . . . # . . . . 

Planning from (0.561175, 1.50477) to (1.94371, 2.04797) with start velocity (0, 0)
0: [0.36, 0.84, 1.44, 1.8]
1: [0.48, 1.2, 1.68, 1.8]
2: [1.08, 1.44, 1.56, 1.92]
3: [1.2, 1.92, 1.8, 2.04]
4: [1.8, 2.04, 1.92, 2.28]

Planning using ARENA method
Point (0.813475, 1.75707) is out of corridor [0.36, 0.84, 1.44, 1.8]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  49.00us (  1.14us)  48.59us (  1.13us)        43
       nlp_g  | 485.00us ( 11.28us) 487.59us ( 11.34us)        43
  nlp_grad_f  | 102.00us (  2.43us) 102.92us (  2.45us)        42
  nlp_hess_l  | 745.00us ( 18.62us) 739.93us ( 18.50us)        40
   nlp_jac_g  |   1.44ms ( 34.21us)   1.43ms ( 34.16us)        42
       total  |   8.16ms (  8.16ms)   8.16ms (  8.16ms)         1
Point (0.79817, 1.74442) is out of corridor [0.48, 1.2, 1.68, 1.8]
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.09ms (  1.07us)   1.07ms (  1.05us)      1014
       nlp_g  |  11.85ms ( 11.68us)  11.83ms ( 11.66us)      1014
  nlp_grad_f  | 477.00us (  2.36us) 473.65us (  2.34us)       202
  nlp_hess_l  |   5.42ms ( 23.88us)   5.43ms ( 23.90us)       227
   nlp_jac_g  |   9.57ms ( 39.06us)   9.59ms ( 39.14us)       245
       total  |  81.83ms ( 81.83ms)  81.85ms ( 81.85ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
Point (0.8215, 1.74296) is out of corridor [0.48, 1.2, 1.68, 1.8]
Planning computation time: 123.2 ms
Planning from (0.561175, 1.50477) to (1.94371, 2.04797) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.36, 0.84, 1.44, 1.8]
1: [0.48, 1.2, 1.68, 1.8]
2: [1.08, 1.44, 1.56, 1.92]
3: [1.2, 1.92, 1.8, 2.04]
4: [1.8, 2.04, 1.92, 2.28]

Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  64.00us (  1.42us)  62.94us (  1.40us)        45
       nlp_g  |  12.52ms (278.24us)  12.53ms (278.43us)        45
  nlp_grad_f  | 124.00us (  2.88us) 123.34us (  2.87us)        43
  nlp_hess_l  |   8.80ms (214.59us)   8.81ms (214.85us)        41
   nlp_jac_g  |  33.47ms (778.30us)  33.48ms (778.71us)        43
       total  | 102.78ms (102.78ms) 102.78ms (102.78ms)         1
Planning computation time: 464.795 ms
Planning from (0.561175, 1.50477) to (1.94371, 2.04797) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.36, 0.84, 1.44, 1.8]
1: [0.48, 1.2, 1.68, 1.8]
2: [1.08, 1.44, 1.56, 1.92]
3: [1.2, 1.92, 1.8, 2.04]
4: [1.8, 2.04, 1.92, 2.28]

Planning using P2P method
Planning computation time: 0.078711 ms

==============================================================
ARENA summary: 
        Tf:                             1.521 s
        Total computation time:         123.200 ms
        Solver time:                    7.160 ms
OCP summary: 
        Tf:                             1.173 s
        Total computation time:         464.795 ms
        Solver time:                    102.782 ms
P2P summary: 
        Tf:                             2.144 s
        Total computation time:         0.079 ms
        Solver time:                    0.000 ms

Overall results:
        Suboptimality:  22.869 % (0.348 ms)
        Total speedup:  3.773
        Solver speedup: 14.356
==============================================================



















. . # . . . . . . # . . . . . . . . # . 
. . . . . . . . # . . . . . # # . . . . 
# # . . . . . . . . # . . . # # . . . . 
. # . # . # . . . . . # . . . . # . . # 
# # # # . . . . . . . . . . . . . . . . 
. # . . . . . . . . # . . # . . . . . # 
. # . . . . . . . . . . # . . # . . . . 
. . . . . . . . . . # . . . # . # . # # 
. . # . . . # . . . . . . . . # . . . . 
# . . . # # . . . # . . # # . . . . # . 
. . . . # . . . . . # . . . . . . . . . 
. . # . . # # . . # # # # . # # # # . # 
. # . . # . . . . . . . . . # . . . . . 
. . # . . . # . . . . . # . . # . . . . 
. . . . . . . . . . . . # . . . . . # . 
# . . # # . . . . . # . . . . . . . . # 
. . # . . . . . . . . . . # . . # . # . 
. . . . . . . . . . . . . . # . . # . . 
# . . # . . . . . . # . . . . . . # . . 
. . # . . . . . . . . # # . . . . . . . 

Planning from (1.44618, 2.27419) to (0.951939, 1.45279) with start velocity (0, 0)
0: [1.32, 1.68, 2.04, 2.4]
1: [1.08, 1.56, 2.16, 2.28]
2: [0.84, 1.2, 1.92, 2.28]
3: [0.72, 1.2, 1.44, 2.16]
4: [0.84, 1.08, 1.2, 1.56]

Planning using ARENA method
Point (1.36938, 2.19739) is out of corridor [1.32, 1.68, 2.04, 2.4]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 106.00us (  1.49us) 108.02us (  1.52us)        71
       nlp_g  |   1.15ms ( 16.21us)   1.15ms ( 16.23us)        71
  nlp_grad_f  | 182.00us (  3.37us) 182.32us (  3.38us)        54
  nlp_hess_l  |   1.17ms ( 22.50us)   1.16ms ( 22.38us)        52
   nlp_jac_g  |   2.90ms ( 53.65us)   2.90ms ( 53.61us)        54
       total  |  16.09ms ( 16.09ms)  16.09ms ( 16.09ms)         1
Flipping acceleration at waypoint 2
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  67.00us (  1.37us)  65.99us (  1.35us)        49
       nlp_g  | 742.00us ( 15.14us) 732.86us ( 14.96us)        49
  nlp_grad_f  | 133.00us (  2.89us) 133.60us (  2.90us)        46
  nlp_hess_l  |   1.03ms ( 23.32us)   1.02ms ( 23.29us)        44
   nlp_jac_g  |   2.30ms ( 50.00us)   2.30ms ( 49.93us)        46
       total  |  11.78ms ( 11.78ms)  11.78ms ( 11.78ms)         1
Point (1.35927, 2.21798) is out of corridor [1.08, 1.56, 2.16, 2.28]
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   4.64ms (  1.18us)   4.60ms (  1.17us)      3939
       nlp_g  |  53.41ms ( 13.56us)  53.38ms ( 13.55us)      3939
  nlp_grad_f  |   1.48ms (  2.81us)   1.46ms (  2.78us)       527
  nlp_hess_l  |  13.91ms ( 25.38us)  13.90ms ( 25.37us)       548
   nlp_jac_g  |  28.41ms ( 50.19us)  28.43ms ( 50.22us)       566
       total  | 279.55ms (279.55ms) 279.55ms (279.55ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
Point (1.35927, 2.21733) is out of corridor [1.08, 1.56, 2.16, 2.28]
Planning computation time: 374.8 ms
Planning from (1.44618, 2.27419) to (0.951939, 1.45279) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [1.32, 1.68, 2.04, 2.4]
1: [1.08, 1.56, 2.16, 2.28]
2: [0.84, 1.2, 1.92, 2.28]
3: [0.72, 1.2, 1.44, 2.16]
4: [0.84, 1.08, 1.2, 1.56]

Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 127.00us (  1.72us) 124.89us (  1.69us)        74
       nlp_g  |  24.61ms (332.62us)  24.62ms (332.67us)        74
  nlp_grad_f  | 274.00us (  3.75us) 266.43us (  3.65us)        73
  nlp_hess_l  |  19.08ms (268.73us)  19.10ms (268.95us)        71
   nlp_jac_g  |  67.25ms (921.22us)  67.30ms (921.93us)        73
       total  | 219.06ms (219.06ms) 219.07ms (219.07ms)         1
Planning computation time: 681.373 ms
Planning from (1.44618, 2.27419) to (0.951939, 1.45279) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [1.32, 1.68, 2.04, 2.4]
1: [1.08, 1.56, 2.16, 2.28]
2: [0.84, 1.2, 1.92, 2.28]
3: [0.72, 1.2, 1.44, 2.16]
4: [0.84, 1.08, 1.2, 1.56]

Planning using P2P method
Planning computation time: 0.109537 ms

==============================================================
ARENA summary: 
        Tf:                             1.472 s
        Total computation time:         374.800 ms
        Solver time:                    26.868 ms
OCP summary: 
        Tf:                             1.016 s
        Total computation time:         681.373 ms
        Solver time:                    219.066 ms
P2P summary: 
        Tf:                             1.761 s
        Total computation time:         0.110 ms
        Solver time:                    0.000 ms

Overall results:
        Suboptimality:  30.989 % (0.456 ms)
        Total speedup:  1.818
        Solver speedup: 8.153
==============================================================










Created motion planner in environment 20 x 20 environment (2.4 x 2.4)
. . . # . . . . # . . . . . # . . # . . 
. . . # . # # # . . . . # . # . . . # # 
# . . # . # . . . # # . # # . . . . . . 
# . # . . . . . . # # . . # . # . # . . 
. . . # # . . . # . . # # . . # . . . . 
# . . . # . # . # . # . . # . . . . . . 
. # . . . . . . . . . # . . . . # . . . 
. . . . # . . . . # . . . . # . # . # # 
. . # . # . # . . . . . . # . # . . . . 
# # # # . . . . . . # # . . . . . # . . 
. . . . . . . . . . # . # . . . # # . . 
. . . . . . . # . . . # . . . . . . . . 
# . . . . . . . # . . . . . # # # . # # 
# . . . . . . . # . . # # . # . . . . . 
. . . . # . . . . . . . . # # . # . # . 
. . . . . . . . # . . . . . . # # # . # 
. . . . . # . # . . . . . . . . . # # . 
. . . . . . . . . . # . . # # . . . # # 
. # . . . # . . # . . . . # . # . . # # 
. # # . . # # . . . . . . . . . # . . . 

Planning from (1.73982, 1.81923) to (0.242202, 0.732387) with start velocity (0, 0)
0: [1.68, 1.8, 1.56, 2.16]
1: [1.44, 1.8, 1.56, 1.68]
2: [1.44, 1.68, 1.44, 1.68]
3: [1.2, 1.56, 1.32, 1.56]
4: [0.84, 1.32, 1.32, 1.44]
5: [0.84, 1.08, 1.08, 1.56]
6: [0.48, 0.96, 0.84, 1.32]
7: [0.24, 0.6, 0.84, 1.2]
8: [0.12, 0.48, 0.48, 1.2]

Planning using ARENA method
Point (1.73712, 1.81653) is out of corridor [1.68, 1.8, 1.56, 2.16]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 191.00us (  2.62us) 192.37us (  2.64us)        73
       nlp_g  |   2.29ms ( 31.32us)   2.28ms ( 31.28us)        73
  nlp_grad_f  | 271.00us (  5.02us) 267.16us (  4.95us)        54
  nlp_hess_l  |   2.79ms ( 52.57us)   2.79ms ( 52.65us)        53
   nlp_jac_g  |   5.61ms ( 98.39us)   5.64ms ( 98.97us)        57
       total  |  30.10ms ( 30.10ms)  30.10ms ( 30.10ms)         1
Point (1.73349, 1.61745) is out of corridor [1.44, 1.8, 1.56, 1.68]
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 576.00us (  2.23us) 570.85us (  2.21us)       258
       nlp_g  |   8.36ms ( 32.39us)   8.34ms ( 32.32us)       258
  nlp_grad_f  | 273.00us (  5.15us) 273.54us (  5.16us)        53
  nlp_hess_l  |   4.11ms ( 59.64us)   4.12ms ( 59.74us)        69
   nlp_jac_g  |   7.95ms (103.29us)   7.96ms (103.36us)        77
       total  |  55.97ms ( 55.97ms)  55.97ms ( 55.97ms)         1
An error occured: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
Point (1.73349, 1.61734) is out of corridor [1.44, 1.8, 1.56, 1.68]
Planning computation time: 177.54 ms
Planning from (1.73982, 1.81923) to (0.242202, 0.732387) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [1.68, 1.8, 1.56, 2.16]
1: [1.44, 1.8, 1.56, 1.68]
2: [1.44, 1.68, 1.44, 1.68]
3: [1.2, 1.56, 1.32, 1.56]
4: [0.84, 1.32, 1.32, 1.44]
5: [0.84, 1.08, 1.08, 1.56]
6: [0.48, 0.96, 0.84, 1.32]
7: [0.24, 0.6, 0.84, 1.2]
8: [0.12, 0.48, 0.48, 1.2]

Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 147.00us (  2.23us) 148.34us (  2.25us)        66
       nlp_g  |  38.26ms (579.65us)  38.27ms (579.78us)        66
  nlp_grad_f  | 331.00us (  5.34us) 318.37us (  5.13us)        62
  nlp_hess_l  |  35.19ms (586.48us)  35.21ms (586.79us)        60
   nlp_jac_g  | 107.29ms (  1.73ms) 107.32ms (  1.73ms)        62
       total  | 311.24ms (311.24ms) 311.24ms (311.24ms)         1
Planning computation time: 1076.51 ms
Planning from (1.73982, 1.81923) to (0.242202, 0.732387) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [1.68, 1.8, 1.56, 2.16]
1: [1.44, 1.8, 1.56, 1.68]
2: [1.44, 1.68, 1.44, 1.68]
3: [1.2, 1.56, 1.32, 1.56]
4: [0.84, 1.32, 1.32, 1.44]
5: [0.84, 1.08, 1.08, 1.56]
6: [0.48, 0.96, 0.84, 1.32]
7: [0.24, 0.6, 0.84, 1.2]
8: [0.12, 0.48, 0.48, 1.2]

Planning using P2P method
Planning computation time: 0.05083 ms

==============================================================
ARENA summary: 
        Tf:                             1.910 s
        Total computation time:         177.540 ms
        Solver time:                    29.099 ms
OCP summary: 
        Tf:                             1.710 s
        Total computation time:         1076.512 ms
        Solver time:                    311.240 ms
P2P summary: 
        Tf:                             3.462 s
        Total computation time:         0.051 ms
        Solver time:                    0.000 ms

Overall results:
        Suboptimality:  10.462 % (0.200 ms)
        Total speedup:  6.064
        Solver speedup: 10.696
==============================================================














Created motion planner in environment 10 x 12 environment (1.2 x 1.44)
# . . . . . . . X X X X 
. . . . . . . . X X X X 
. . . . X X X X X X X X 
# . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . . X X X X X X X X 
. . . # . . . . . . . . 
. # . . . # . . . . . # 
. # . . . . . . . . . . 
. # . . . . . . . . . . 

Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using ARENA method
Point (0.516624, 0.305713) is out of corridor [0.24, 0.6] x [0, 0.36]

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  55.00us (  1.00us)  53.86us (979.31ns)        55
       nlp_g  | 353.00us (  6.42us) 353.26us (  6.42us)        55
  nlp_grad_f  |  91.00us (  2.22us)  87.54us (  2.14us)        41
  nlp_hess_l  | 466.00us ( 11.95us) 466.32us ( 11.96us)        39
   nlp_jac_g  | 807.00us ( 19.68us) 809.10us ( 19.73us)        41
       total  |   6.03ms (  6.03ms)   6.03ms (  6.03ms)         1
Planning computation time: 21.5785 ms
Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using OCP method
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |  47.00us (810.34ns)  46.73us (805.62ns)        58
       nlp_g  |  10.67ms (183.93us)  10.67ms (183.92us)        58
  nlp_grad_f  |  55.00us (  2.04us)  53.64us (  1.99us)        27
  nlp_hess_l  |  28.00ms (777.86us)  28.02ms (778.22us)        36
   nlp_jac_g  |  45.05ms (938.50us)  45.06ms (938.77us)        48
       total  | 116.01ms (116.01ms) 116.01ms (116.01ms)         1
An error occurred: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:159:
.../casadi/core/optistack_internal.cpp:997: Assertion "return_success(accept_limit)" failed:
Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Infeasible_Problem_Detected'
Planning computation time: 216.651 ms
Planning from (0.449124, 0.238788) to (0.869822, 0.35674) with start velocity (0, 0)
NOTE: skipped update of corridor sequence.
0: [0.24, 0.6] x [0, 0.36]
1: [0.36, 0.84] x [0, 0.24]
2: [0.72, 1.08] x [0, 0.48]

Planning using P2P method
Planning computation time: 0.027963 ms

==============================================================
ARENA summary: 
        Tf:                             0.627 s
        Total computation time:         21.578 ms
        Solver time:                    6.033 ms
OCP summary: 
        Tf:                             1.679 s
        Total computation time:         216.651 ms
        Solver time:                    -1.000 ms				----> Why does this fail?
P2P summary: 
        Tf:                             1.126 s
        Total computation time:         0.028 ms
        Solver time:                    0.000 ms

Overall results:
        Suboptimality:  -167.745 % (-1.052 ms)
        Total speedup:  10.040
        Solver speedup: -0.166
==============================================================
*/