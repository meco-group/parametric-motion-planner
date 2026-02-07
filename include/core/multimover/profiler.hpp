#ifndef __PROFILER__
#define __PROFILER__

#include <vector>
#include <chrono>
#include <string>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Class to profile simulation
class Profiler {
    public:
        Profiler() = default;
        Profiler(std::initializer_list<const char*> simulation_step_names)
            : simulation_step_names_(simulation_step_names.begin(), simulation_step_names.end()) {};

        void StartSimulationStep(){
            if (current_intermediate_step_idx != -1){
                throw std::runtime_error("Previous simulation step has not correctly been profiled. You must call EndSimulationStep()");
            }
            time_durations_ms_.push_back(
                std::vector<double>(simulation_step_names_.size(), 0.0));
            current_intermediate_step_idx = 0;
            start_time_ = std::chrono::high_resolution_clock::now();
        };
        double RecordIntermediateSimulationStep(){
            if (current_intermediate_step_idx >= simulation_step_names_.size()){
                throw std::runtime_error("No more intermediate steps can be recorded. You must call EndSimulationStep() first.");
            }
            if (current_intermediate_step_idx == -1){
                throw std::runtime_error("You must call StartSimulationStep() before recording intermediate steps.");
            }
            curr_time_ = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_time_ - start_time_).count()/1000;
            time_durations_ms_.back()[current_intermediate_step_idx] = duration;
            current_intermediate_step_idx++;
            start_time_ = std::chrono::high_resolution_clock::now();
            return duration;
        };
        double EndSimulationStep(){
            if (current_intermediate_step_idx != simulation_step_names_.size() - 1){
                throw std::runtime_error("Not all simulation steps have been recorded. You must call RecordIntermediateSimulationStep() for each step.");
            }
            curr_time_ = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_time_ - start_time_).count()/1000;
            time_durations_ms_.back()[current_intermediate_step_idx] = duration;
            current_intermediate_step_idx++;
            current_intermediate_step_idx = -1;
            start_time_ = std::chrono::high_resolution_clock::now();
            return duration;
        };

        void AbortStep(){
            if (current_intermediate_step_idx == -1){
                throw std::runtime_error("No simulation step is being recorded. You must call StartSimulationStep() first.");
            }
            while (current_intermediate_step_idx < simulation_step_names_.size()-1){
                RecordIntermediateSimulationStep();
            }
            EndSimulationStep();
        };

        void PrintLastStepInfo() const {
            if (time_durations_ms_.size() == 0){
                std::cout << "No simulation steps have been recorded yet." << std::endl;
                return;
            }
            for (int i = 0; i < simulation_step_names_.size(); i++){
                double ms = time_durations_ms_.back()[i]/1000;
                if (ms > 1.0){
                    std::cout << simulation_step_names_[i] << ": " 
                            << time_durations_ms_.back()[i]/1000 << " ms" << std::endl;
                }
            }
        }

        json ToJson() const {
            json j;
            std::vector<double> total_durations(simulation_step_names_.size(), 0.0);
            for (int i = 0; i < time_durations_ms_.size(); i++){
                for (int j = 0; j < simulation_step_names_.size(); j++){
                    total_durations[j] += time_durations_ms_[i][j];
                }
            }
            
            for (int i = 0; i < simulation_step_names_.size(); i++){
                json j_step;
                j_step["total_ms"] = total_durations[i];
                j_step["average_ms"] = total_durations[i] / time_durations_ms_.size();
                j[simulation_step_names_[i]] = j_step;
            }

            return j;
        }

        json ToJsonDetailed() const {
            json j;
            for (int i = 0; i < simulation_step_names_.size(); i++){
                j[simulation_step_names_[i]] = json::array();
            }
            for (int step_idx = 0; step_idx < time_durations_ms_.size(); step_idx++){
                for (int sim_step_idx = 0; sim_step_idx < simulation_step_names_.size(); sim_step_idx++){
                    j[simulation_step_names_[sim_step_idx]].push_back(
                        time_durations_ms_[step_idx][sim_step_idx]);
                }
            }
            return j;
        }

    private:
        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point curr_time_;
        int current_intermediate_step_idx = -1;
        std::vector<std::string> simulation_step_names_;
        
        std::vector<std::vector<double>> time_durations_ms_;
};

#endif