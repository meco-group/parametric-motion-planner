#include <iostream>
#include <cmath>

#include "parametric_motion_planner.hpp"

int main(int argc, char *argv[]){
    bool RANDOMIZE = true;

    // create an environment
    Environment env = Environment();

    // add some obstacles
    std::vector<int> xx = {};
    std::vector<int> yy = {};
    for (int i = 0; i < xx.size(); i++){
        env.AddObstacle(Point2D<int>(xx[i], yy[i]));
    }
    if (RANDOMIZE){
        env.AddRandomObstacles(0.1);
    }

    std::cout << "Environment: " << std::endl;
    std::cout << env << std::endl;

    // select default parameters
    Parameters params = Parameters();

    // create motion planner
    MotionPlanner mp = MotionPlanner(params, env);

    // Plan from start to destination
    Point2D<int> start_cell = Point2D<int>(10, 3);
    Point2D<int> dest_cell = Point2D<int>(6, 8);
    mp.SetStart(start_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    mp.SetDest(dest_cell.ConvertCellToWorld(env.CellWidth(), env.CellHeight()));
    if (RANDOMIZE){
        mp.SetRandomStart();
        mp.SetRandomDest();
    }
    mp.PlanSafely();
    mp.DumpToJson("example_problem.json");
    std::cout << "stored output in output/example_problem.json" << std::endl;
}
