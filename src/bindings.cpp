#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "parametric_motion_planner.hpp"

PYBIND11_MODULE(parametric_motion_planner_module, m){
    m.doc() = "Parametric motion planner module";

    //////////////////////
    /// Planner method ///
    //////////////////////
    pybind11::enum_<PlannerMethod>(m, "PlannerMethod")
        .value("ARENA", ARENA)
        .value("OCP", OCP)
        .value("P2P", P2P)
        ;

    ///////////////
    /// Point2D ///
    ///////////////
    pybind11::class_<Point2D<double>>(m, "Point2Dd")
        .def(pybind11::init<double, double>())
        .def("SetX", &Point2D<double>::SetX)
        .def("SetY", &Point2D<double>::SetY)
        .def("x", &Point2D<double>::x)
        .def("y", &Point2D<double>::y)
        ;
    pybind11::class_<Point2D<int>>(m, "Point2Di")
        .def(pybind11::init<int, int>())
        .def("SetX", &Point2D<int>::SetX)
        .def("SetY", &Point2D<int>::SetY)
        .def("x", &Point2D<int>::x)
        .def("y", &Point2D<int>::y)
        ;

    //////////////////
    /// Parameters ///
    //////////////////
    pybind11::class_<Parameters>(m, "Parameters")
        .def(pybind11::init<double, double, double, double, double>())
        .def("GetVmax", &Parameters::GetVmax)
        .def("GetAmax", &Parameters::GetAmax)
        .def("GetVehWidth", &Parameters::GetVehWidth)
        .def("GetVehHeight", &Parameters::GetVehHeight)
        .def("GetMargin", &Parameters::GetMargin)
        .def("SetVmax", &Parameters::SetVmax)
        .def("SetAmax", &Parameters::SetAmax)
        .def("SetVehWidth", &Parameters::SetVehWidth)
        .def("SetVehHeight", &Parameters::SetVehHeight)
        .def("SetMargin", &Parameters::SetMargin)
        .def("ToJson", [](const Parameters& self){
            return self.ToJson().dump();
        })
        ;

    ///////////////////
    /// Environment ///
    ///////////////////
    pybind11::class_<Environment>(m, "Environment")
        .def(pybind11::init<>())
        .def(pybind11::init<int, int, double, double>())
        .def("AddObstacle", &Environment::AddObstacle)
        .def("AddRandomObstacles", &Environment::AddRandomObstacles)
        .def("GetRandomFreeVehiclePosition", &Environment::GetRandomFreeVehiclePosition)
        .def("GetRandomFreeCellPosition", &Environment::GetRandomFreeCellPosition)
        .def("ToJson", [](const Environment& self){
            return self.ToJson().dump();
        })

        // custom function needed in benchmarking
        .def("CopyObstacles", [](Environment& self, Environment& other){
            // sanity check
            assert (self.NbCellCols() == other.NbCellCols());
            assert (self.NbCellRows() == other.NbCellRows());
            assert (self.CellWidth() == other.CellWidth());
            assert (self.CellHeight() == other.CellHeight());

            // copy obstacles
            self.ClearAllObstacles();
            for (int i = 0; i < self.NbCellCols(); i++){
                for (int j = 0; j < self.NbCellRows(); j++){
                    if (!other.IsFree(i, j)){
                        self.AddObstacle(Point2D<int>(i, j));
                    }
                }
            }
        })
        ;

    //////////////////
    /// Trajectory ///
    //////////////////
    pybind11::class_<Trajectory>(m, "Trajectory")
        .def(pybind11::init<>())
        .def("Px", &Trajectory::Px)
        .def("Py", &Trajectory::Py)
        .def("nbSamples", &Trajectory::NbSamples)
        ;

    //////////////////
    /// MotionPlanner ///
    /////////////////////
    pybind11::class_<MotionPlanner>(m, "MotionPlanner")
        .def(pybind11::init<PlannerMethod, Parameters const &, Environment const &>())
        .def("SetStart", &MotionPlanner::SetStart)
        .def("SetDest", &MotionPlanner::SetDest)
        .def("SetStartVel", &MotionPlanner::SetStartVel)
        .def("SetMethod", &MotionPlanner::SetMethod)
        .def("SetOCPNumberOfPointsPerCorridor", &MotionPlanner::SetOCPNumberOfPointsPerCorridor)
        .def("Plan", pybind11::overload_cast<>(&MotionPlanner::Plan))
        .def("GetTotalComputationTime", &MotionPlanner::GetTotalComputationTime)
        .def("GetSolverTime", &MotionPlanner::GetSolverTime)
        .def("GetTravelTime", &MotionPlanner::GetTravelTime)
        .def("CorridorInfeasibilitiesDetected", &MotionPlanner::CorridorInfeasibilitiesDetected)
        .def("SetSuboptimalityEliminationFeature", &MotionPlanner::SetSuboptimalityEliminationFeature)
        .def("UpdateCorridorSequence", pybind11::overload_cast<>(&MotionPlanner::UpdateCorridorSequence))
        .def("GetCorridorSequence", [](const MotionPlanner& self){
            const CorridorSequence& sequence = self.GetCorridorSequence();
            std::vector<std::vector<double>> corridors(sequence.NbCorridors());
            Corridor c;
            for (int i = 0; i < sequence.NbCorridors(); i++){
                c = sequence.GetCorridor(i);
                corridors[i] = {c.Xmin(), c.Xmax(), c.Ymin(), c.Ymax()};
            }
            return corridors;
        })
        .def("GetLastSolution", &MotionPlanner::GetLastSolution)
        .def("DumpToJson", &MotionPlanner::DumpToJson)
        .def("PrintParametrization", &MotionPlanner::PrintParametrization)
        .def("ShowInitialization", &MotionPlanner::PrintInitialization)
        .def("SetPrintLevel", &MotionPlanner::SetPrintLevel)
        .def("SetMaxIter", &MotionPlanner::SetMaxIter)
        ;
}