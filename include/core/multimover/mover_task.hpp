#ifndef __MOVER_TASK_HPP__
#define __MOVER_TASK_HPP__

#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

enum TaskEventType {
    TASK_POSTPONED,
    TASK_REVEALED,
    TASK_PLANNING_OCCURED,
    TASK_COMPLETED,
    MOVER_WAITING,
    MOVER_MOVING,
    TASK_ABORTED,
};

inline std::string TaskEventTypeToString(TaskEventType type) {
    switch (type) {
        case TASK_POSTPONED: return "TASK_POSTPONED";
        case TASK_REVEALED: return "TASK_REVEALED";
        case TASK_PLANNING_OCCURED: return "TASK_PLANNING_OCCURED";
        case TASK_COMPLETED: return "TASK_COMPLETED";
        case MOVER_WAITING: return "MOVER_WAITING";
        case MOVER_MOVING: return "MOVER_MOVING";
        default: return "UNKNOWN_EVENT_TYPE";
    }
}

class MoverTaskEvent{
    public:
        MoverTaskEvent(double time_stamp, TaskEventType event_type, 
                       double optional_meta_data = 0.0) :
            time_stamp_(time_stamp), event_type_(event_type), 
            optional_meta_data_(optional_meta_data) {}

        json ToJson() const;

    private:
        double time_stamp_;
        TaskEventType event_type_;
        double optional_meta_data_;
};

class MoverTask{
    public:
        MoverTask(int agent_idx, const std::string& destination, 
                  double time_to_reveal_task, bool deadlock_resolution_task=false) :
            agent_idx_(agent_idx), destination_name_(destination), 
            time_to_reveal_task_(time_to_reveal_task), 
            deadlock_resolution_task_(deadlock_resolution_task) {};
        
        // check if now is the time to reveal the task
        bool RevealTask(double current_time);

        // if a task cannot be processed because the agent is not yet ready,
        // postpone the task
        void PostponeTask(double time_to_wait, double curr_time);

        void NotifyCompleted(double current_time);
        void NotifyStartedToWait(double current_time);
        void NotifyStartedToMove(double current_time);
        void NotifyPlanningOccured(double current_time, double planning_time);
        void NotifyAborted(double current_time);

        // Basic getters
        int GetAgentIdx() const { return agent_idx_;}
        const std::string& GetDestinationName() const { return destination_name_; }
        bool HasBeenRevealed() const { return has_been_revealed_; }

        // printing
        friend std::ostream& operator<<(std::ostream& os, const MoverTask& task) {
            os << "MoverTask(agent_idx: " << task.agent_idx_ 
               << ", destination: " << task.destination_name_ 
               << ", time_to_reveal_task: " << task.time_to_reveal_task_ 
               << ", task_delay: " << task.task_delay_ 
               << ", has_been_revealed: " << task.has_been_revealed_ << ")";
            return os;
        }

        json ToJson() const;

    private:
        int agent_idx_ = -1;
        std::string destination_name_;
        double time_to_reveal_task_;
        double task_delay_ = 0;
        bool has_been_revealed_ = false;
        bool completed_ = false; // whether the task has been completed
        bool deadlock_resolution_task_ = false; // whether this task is a deadlock resolution task

        std::vector<MoverTaskEvent> task_events_; // events related to this task
};

#endif