#include "core/multimover/mover_task.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

json MoverTaskEvent::ToJson() const {
    json j;
    j["time_stamp"] = time_stamp_;
    j["event_type"] = TaskEventTypeToString(event_type_);
    j["optional_meta_data"] = optional_meta_data_;
    return j;
}

bool MoverTask::RevealTask(double current_time) {
    if (current_time >= (time_to_reveal_task_ + task_delay_)){
        if (!has_been_revealed_) {
            task_events_.emplace_back(current_time, TASK_REVEALED);
        }
        has_been_revealed_ = true;
        return true;
    }
    return false;
}

void MoverTask::PostponeTask(double time_to_wait, double curr_time) {
    if (completed_){ return;}
    task_delay_ += time_to_wait;
    has_been_revealed_ = false;
    task_events_.emplace_back(curr_time, TASK_POSTPONED, time_to_wait);
}

void MoverTask::NotifyCompleted(double current_time) {
    completed_ = true;
    task_events_.emplace_back(current_time, TASK_COMPLETED);
}

void MoverTask::NotifyStartedToMove(double current_time) {
    if (completed_){ return;}
    task_events_.emplace_back(current_time, MOVER_MOVING);
}

void MoverTask::NotifyStartedToWait(double current_time) {
    if (completed_){ return;}
    task_events_.emplace_back(current_time, MOVER_WAITING);
}

void MoverTask::NotifyPlanningOccured(double current_time, double planning_time) {
    if (completed_){ return;}
    task_events_.emplace_back(current_time, TASK_PLANNING_OCCURED, planning_time);
}

void MoverTask::NotifyAborted(double current_time){
    if (completed_){ return;}
    task_events_.emplace_back(current_time, TASK_ABORTED);
}

json MoverTask::ToJson() const {
    json j;
    j["agent_idx"] = agent_idx_;
    j["destination_name"] = destination_name_;
    j["time_to_reveal_task"] = time_to_reveal_task_;
    j["task_delay"] = task_delay_;
    j["task_events"] = json::array();
    for (const auto& event : task_events_) {
        j["task_events"].push_back(event.ToJson());
    }
    return j;
}