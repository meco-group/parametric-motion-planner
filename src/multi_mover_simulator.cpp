#include "core/multi_mover_simulator.hpp"

Agent::Agent(MotionPlanner* planner) : planner_(planner), blocking_agent_(nullptr){
    state_ = IDLING;
    final_dest_ = planner->GetStart();
}

void Agent::SetFinalDestination(const Point2D<double>& final_dest){
    final_dest_ = final_dest;
    state_ = PLANNING_TO_FINAL_DESTINATION;
}

const Point2D<double>& Agent::GetFinalDestination() const{
    return final_dest_;
}

const Point2D<double>& Agent::GetCurrentPosition() const{
    return planner_->GetStart();
}

const AgentState& Agent::GetState() const{
    return state_;
}

const Point2D<double>& Agent::GetWaitingPosition() const{
    return waiting_position_;
}

const Agent& Agent::GetBlockingAgent() const{
    return *blocking_agent_;
}

const Corridor& Agent::GetIntersection() const{
    return intersection_;
}

void Agent::WaitForAgent(const Point2D<double>& waiting_position, 
                          Agent& blocking_agent, Corridor& intersection){
    state_ = MOVING_TO_WAITING_POINT;
    waiting_position_ = waiting_position;
    blocking_agent_ = &blocking_agent;
    intersection_ = intersection;
}

void Agent::SimulateStep(Point2D<double>& pos, 
                          Point2D<double>& vel, 
                          Point2D<double>& acc, 
                          double& time){
    // TODO
}

















MultiMoverSimulator::MultiMoverSimulator(std::vector<MotionPlanner>& planners) : agents_(){
    for (auto& planner : planners){
        agents_.emplace_back(&planner);
    }
}

void MultiMoverSimulator::InstructAgentToDestination(int agent_idx, 
                                                      const Point2D<double>& final_dest){
    if (agent_idx < 0 || agent_idx >= agents_.size()){
        throw std::out_of_range("Agent index out of range");
    }
    agents_[agent_idx].SetFinalDestination(final_dest);
}

void MultiMoverSimulator::SimulateSteps(int nb_steps){
    for (int i = 0; i < nb_steps; i++){
        for (auto& agent : agents_){
            Point2D<double> pos, vel, acc;
            double time;
            agent.SimulateStep(pos, vel, acc, time);
        }
    }

    // TODO: deal with agents updating their trajectories
}