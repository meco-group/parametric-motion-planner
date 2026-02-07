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

bool MoverTask::RevealTask(double current_time, int nb_completed_tasks_by_agent) {
    if (current_time >= (time_to_reveal_task_ + task_delay_) &&
            (nb_completed_tasks_by_agent < 0 || task_sequence_nb_< 0 ||
             nb_completed_tasks_by_agent == task_sequence_nb_)) {
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
    // has_been_revealed_ = false;
}

void MoverTask::NotifyResumed(double current_time){
    if (completed_){ return;}
    task_events_.emplace_back(current_time, TASK_RESUMED);
}

json MoverTask::ToJson() const {
    json j;
    j["agent_idx"] = agent_idx_;
    j["task_sequence_nb"] = task_sequence_nb_;
    j["destination_name"] = destination_name_;
    j["time_to_reveal_task"] = time_to_reveal_task_;
    j["task_delay"] = task_delay_;
    j["task_events"] = json::array();
    for (const auto& event : task_events_) {
        j["task_events"].push_back(event.ToJson());
    }
    j["deadlock_resolution_task"] = deadlock_resolution_task_;
    return j;
}