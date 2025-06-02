#include <iostream>

class MoverTask{
    public:
        MoverTask(int agent_idx, const std::string& destination, 
                  double time_to_reveal_task) :
            agent_idx_(agent_idx), destination_name_(destination), 
            time_to_reveal_task_(time_to_reveal_task) {};
        
        // check if now is the time to reveal the task
        bool RevealTask(double current_time) {
            if (current_time >= (time_to_reveal_task_ + task_delay_)){
                has_been_revealed_ = true;
                return true;
            }
            return false;
        }

        // if a task cannot be processed because the agent is not yet ready,
        // postpone the task
        void PostponeTask(double time_to_wait) {
            task_delay_ += time_to_wait;
            has_been_revealed_ = false;
        }

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

    private:
        int agent_idx_ = -1;
        std::string destination_name_;
        double time_to_reveal_task_;
        double task_delay_ = 0;
        bool has_been_revealed_ = false;
};