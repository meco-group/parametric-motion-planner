#include "core/dynamic_sampler.hpp"

bool DynamicSampler::GetSample(Point2D<double> &pos, Point2D<double> &vel, 
                               Point2D<double> &acc){
    // std::cout << "Getting sample" << std::endl;
    if (record_sample_time_){ start = std::chrono::high_resolution_clock::now();}
    if (record_sample_request_times_){
        if (nb_samples_provided_ == 0){
            time_of_first_sample_request_ = std::chrono::high_resolution_clock::now();
        }
        auto now = std::chrono::high_resolution_clock::now();
        duration_of_request_since_first_sample_in_ms_.push_back(
            std::chrono::duration_cast<std::chrono::microseconds>(
                now - time_of_first_sample_request_).count()/1000.0);
    }
    // RecordTrigger("Sample requested");
    // First, check triggers and perform actions
    ExecuteTriggerActions();

    // Retrieve the current sample
    double local_time = 0.0;
    motion_planner_.GetSample(local_time, pos, vel, acc);
    curr_pos_ = pos; curr_vel_ = vel;
    travelled_positions_.push_back(curr_pos_);
    // std::cout << "sample provided: " << pos << std::endl;
    // RecordTrigger("Sample provided");

    // Update logs
    curr_time_ = last_succesfull_planning_time_ + local_time;
    nb_samples_provided_++;
    nb_samples_provided_since_last_replan_++;
    travelled_trajectory_.Append(curr_time_, curr_pos_.x(), curr_pos_.y(), 
                                 curr_vel_.x(), curr_vel_.y(), acc.x(), acc.y());

    if (finished_){
        std::cout << "event record:" << std::endl;
        for (const std::string& s : events_){
            std::cout << s << std::endl;
        }
        std::cout << std::endl << std::endl;

        // double max_d = -1;
        // for (int i = 0; i < travelled_positions_.size()-1; i++){
        //     double d = travelled_positions_[i].Distance(travelled_positions_[i+1]);
        //     // if (d > 2 * std::sqrt(2)*2.0*0.010){
        //     //     std::cout << "distance: " << d << std::endl;
        //     // }
        //     if (d > max_d){
        //         max_d = d;
        //     }
        // }
        // std::cout << "max distance: " << max_d << "(max expected: " << std::sqrt(2)*2.0*0.010 << ")" << std::endl;

        // std::cout << "nb_samples_provided:         " << nb_samples_provided_ << std::endl;
        // std::cout << "length travelled trajectory: " << travelled_trajectory_.NbSamples() << std::endl;
        // std::cout << "length travelled positions:  " << travelled_positions_.size() << std::endl;
    }

    if (record_sample_time_){
        stop = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.0;
        ms_to_retrieve_sample_.push_back(ms);
    }

    return finished_;
};

void DynamicSampler::AddTrigger(std::function<bool()> trigger,
                                std::function<void()> action){
    trigger_action_pairs_.push_back(std::make_pair(trigger, action));
};

void DynamicSampler::AddInitialization(bool with_obstacles){
    AddTrigger(
        [this](){ return GetTotalNbSamplesProvided() == 0;}, 
        [this, with_obstacles](){
            SetInitialStart();
            // SetRandomDestination();
            motion_planner_.SetDest(Point2D<int>(6, 8).ConvertCellToWorld(
                environment_.CellWidth(), environment_.CellHeight()));

            // add obstacle
            if (with_obstacles){
                for (int i = 0; i < movable_obstacle_position_.size(); i++){
                    environment_.AddObstacle(movable_obstacle_position_[i]);
                }
            }

            Plan();
            RecordTrigger("Initial plan");
    });
}

void DynamicSampler::AddBasicReplanningTriggers(){
    // If in emergency mode and reached resting position, relpan
    AddTrigger(
        [this](){return motion_planner_.EmergencyMode() && 
                        GetCurrentRemainingNbSamples() <= 0;
        },
        [this](){
            Plan(); RecordTrigger("Replanning after exiting emergency mode");
        }
    );

    // If planning failed and emergency trajectory failed as well, keep going 
    // for a while and try again
    AddTrigger(
        [this](){
            return !last_planning_succeeded_ && nb_samples_provided_since_last_replan_ > 3;
        },
        [this](){ 
            Plan(); RecordTrigger("Retrying to plan after failure");
        }
    );

    // If there is nothing left to do, finish
    AddTrigger(
        [this](){
            return !motion_planner_.EmergencyMode() &&
                   GetCurrentRemainingNbSamples() <= 0;
        },
        [this](){ 
            RecordTrigger("Finished (nothing left to do)");
            Finish();
        }
    );
};

void DynamicSampler::RecordTrigger(std::string&& trigger_statement){
    events_.push_back(trigger_statement);
};

void DynamicSampler::MoveDestinationDemo(int max_nb_replans,
                                         bool alternate_dest){
    if (alternate_dest){ max_nb_replans = 2*floor(max_nb_replans/2);}
    
    // make sure to start properly
    AddInitialization();

    // make sure to replan properly
    AddBasicReplanningTriggers();

    // change the destination if only 30% of the current trajectory remains
    std::vector<double> rand_fractions(max_nb_replans);
    for (int i = 0; i < max_nb_replans; i++){
        rand_fractions[i] = 0.05 + 0.3*(double)std::rand()/((double)RAND_MAX);
    }
    AddTrigger(
        [this, max_nb_replans, alternate_dest, rand_fractions](){
            double fraction = alternate_dest ? rand_fractions[nb_replans_] : 0.3;
            return nb_replans_ < max_nb_replans - 1 &&
                   last_planning_succeeded_ && 
                   !motion_planner_.EmergencyMode() &&
                   GetCurrentRemainingNbSamples() < fraction*GetCurrentNbSamples();
        },
        [this, alternate_dest](){
            // pick the destination
            if (alternate_dest){
                Point2D<double> dest = nb_replans_ % 2 == 0 ?
                    Point2D<int>(6, 8).ConvertCellToWorld(
                        environment_.CellWidth(), environment_.CellHeight()) :
                    Point2D<int>(10, 1).ConvertCellToWorld(
                        environment_.CellWidth(), environment_.CellHeight());
                motion_planner_.SetDest(dest);
            } else {
                SetRandomDestination();
            }

            // place a random obstacle
            if (!motion_planner_.EmergencyMode()){
                for (int i = 0; i < movable_obstacle_position_.size(); i++){
                    // remove the movable obstacle from the environment
                    environment_.RemoveObstacle(movable_obstacle_position_[i]);

                    // pick a new positions in the square (0, 4)-(3,4)-(3,8)-(0,8)
                    movable_obstacle_position_[i] = Point2D<int>(rand() % 4, 
                                                              4 + rand() % 4);
                    environment_.AddObstacle(movable_obstacle_position_[i]);
                }
            }

            Plan();
            RecordTrigger("Changing destination and replanning");
        }
    );

    // move back to the starting position for the final replanning
    AddTrigger(
        [this, max_nb_replans](){
            return nb_replans_ == max_nb_replans - 1 &&
                   last_planning_succeeded_ && 
                   !motion_planner_.EmergencyMode() &&
                   GetCurrentRemainingNbSamples() < 0.3*GetCurrentNbSamples();
        },
        [this](){
            motion_planner_.SetDest(
                Point2D<int>(10, 1).ConvertCellToWorld(
                    environment_.CellWidth(), environment_.CellHeight()));
            Plan();
            RecordTrigger("Changing destination and replanning");
        }
    );
};

void DynamicSampler::SuddenObstacleDemo(double time_of_sudden_obstacle, 
                                        Point2D<int> obstacle_position){
    // make sure to start properly
    AddInitialization();

    // make sure to replan properly
    AddBasicReplanningTriggers();

    // add the sudden obstacle
    AddTrigger(
        [this, time_of_sudden_obstacle, obstacle_position](){
            if (sudden_obstacle_deployed_){ return false;}
            // time based appraoch:
            auto now = std::chrono::high_resolution_clock::now();
            double moving_time = std::chrono::duration_cast<std::chrono::microseconds>(
                now - time_of_first_sample_request_).count()/(1.0e6);
            moving_time -= 0.001*mover_started_moving_since_first_sample_in_ms_;
            return moving_time >= time_of_sudden_obstacle;

            //// distance based appraoch:
            // double d = curr_pos_.Distance(obstacle_position.ConvertCellToWorld(
            //     environment_.CellWidth(), environment_.CellHeight()));
            // return d < 3*environment_.CellWidth();
        },
        [this, obstacle_position](){
            std::cout << environment_ << std::endl;
            environment_.AddObstacle(obstacle_position);
            sudden_obstacle_deployed_ = true;
            std::cout << environment_ << std::endl;
            Plan();
            RecordTrigger("Adding sudden obstacle and replanning");
        }
    );

    // go back to home position
    AddTrigger(
        [this]() {
            return (GetCurrentRemainingNbSamples() <= 1 &&
                curr_pos_.y() >= 0.5);
        },
        [this]() {
            motion_planner_.SetDest(Point2D<int>(10, 1).ConvertCellToWorld(
                environment_.CellWidth(), environment_.CellHeight()));
            Plan();
            RecordTrigger("Going back home");
        }
    );
};

int DynamicSampler::GetCurrentNbSamples() const {
    return motion_planner_.GetLastSolution().NbSamples();
};

int DynamicSampler::GetCurrentRemainingNbSamples() const {
    return motion_planner_.GetLastSolution().NbSamples() - motion_planner_.GetCurrentSampleIdx();
};

void DynamicSampler::SetInitialStart(){
    curr_pos_ = Point2D<int>(10, 1).ConvertCellToWorld(
                        environment_.CellWidth(), environment_.CellHeight());
    curr_vel_ = Point2D<double>(0, 0);

    motion_planner_.SetStart(curr_pos_);
    motion_planner_.SetStartVel(curr_vel_);
};

void DynamicSampler::Plan(){
    bool second_to_last_planning_succeeded = last_planning_succeeded_;
    auto now = std::chrono::high_resolution_clock::now();
    double replanning_time = std::chrono::duration_cast<std::chrono::microseconds>(
        now - time_of_first_sample_request_).count() / (1.0e6);
    try{
        motion_planner_.SetStart(curr_pos_);
        motion_planner_.SetStartVel(curr_vel_);
        motion_planner_.PlanSafely(10);
        last_planning_succeeded_ = true;
        last_succesfull_planning_time_ = curr_time_;
        if (!motion_planner_.EmergencyMode()){
            nb_replans_++;
        }

        if (curr_time_ > 0){
            // replanning_times_.push_back(curr_time_);
            if (second_to_last_planning_succeeded){
                //auto now = std::chrono::high_resolution_clock::now();
                //double replanning_time = std::chrono::duration_cast<std::chrono::microseconds>(
                //    now - time_of_first_sample_request_).count()/(1.0e6);
                replanning_times_.push_back(replanning_time);
            }

            // discard first sample of the trajectory
            double t; Point2D<double> a_temp;
            motion_planner_.GetSample(t, curr_pos_, curr_vel_, a_temp);
        }
        previous_trajectories_.push_back(motion_planner_.GetLastSolution());
        previous_corridor_sequences_.push_back(motion_planner_.GetCorridorSequence());
        previous_environments_.push_back(environment_.ToJson());
    } catch (std::exception &e){
        if (second_to_last_planning_succeeded){
            //auto now = std::chrono::high_resolution_clock::now();
            //double replanning_time = std::chrono::duration_cast<std::chrono::microseconds>(
            //    now - time_of_first_sample_request_).count()/(1.0e6);
            //    replanning_times_.push_back(replanning_time);
        }
        last_planning_succeeded_ = false;
    }
    nb_samples_provided_since_last_replan_ = 0;
};

json DynamicSampler::ToJson() const {
    json dynamic_sampler_json;
    dynamic_sampler_json["motion_planner"] = motion_planner_.ToJson();
    dynamic_sampler_json["environment"] = environment_.ToJson();
    dynamic_sampler_json["replanning_times"] = replanning_times_;
    dynamic_sampler_json["previous_trajectories"] = json::array();
    for (const auto& traj : previous_trajectories_){
        dynamic_sampler_json["previous_trajectories"].push_back(traj.ToJson());
    }
    dynamic_sampler_json["previous_corridor_sequences"] = json::array();
    for (const auto& corridor : previous_corridor_sequences_){
        dynamic_sampler_json["previous_corridor_sequences"].push_back(corridor.ToJson());
    }
    dynamic_sampler_json["previous_environments"] = json::array();
    for (const auto& env : previous_environments_){
        dynamic_sampler_json["previous_environments"].push_back(env);
    }
    dynamic_sampler_json["travelled_trajectory"] = travelled_trajectory_.ToJson();

    dynamic_sampler_json["record_sample_time"] = record_sample_time_;
    dynamic_sampler_json["ms_to_retrieve_sample"] = ms_to_retrieve_sample_;

    dynamic_sampler_json["record_sample_request_times"] = record_sample_request_times_;
    dynamic_sampler_json["duration_of_request_since_first_sample_in_ms"] = duration_of_request_since_first_sample_in_ms_;

    dynamic_sampler_json["mover_started_moving"] = mover_started_moving_since_first_sample_in_ms_;

    dynamic_sampler_json["travelled_positions"] = json::array();
    for (const auto& pos : travelled_positions_){
        dynamic_sampler_json["travelled_positions"].push_back(pos.ToJson());
    }

    return dynamic_sampler_json;
};

void DynamicSampler::DumpToJson(const std::string &filename) const {
    std::filesystem::create_directory("output");
    std::string full_path = "output/" + filename;
    
    std::ofstream outFile(full_path);
    outFile << ToJson().dump(4);
    outFile.close();
}

void DynamicSampler::FirstBufferDeployedToPLC(){
    auto now = std::chrono::high_resolution_clock::now();
    mover_started_moving_since_first_sample_in_ms_ = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            now - time_of_first_sample_request_).count()/1000.0;
};
 
void DynamicSampler::ExecuteTriggerActions(){
    int curr_nb_of_triggers_recorded;
    for (const auto& [trigger, action] : trigger_action_pairs_){
        if (trigger()){
            curr_nb_of_triggers_recorded = events_.size();
            action();
            if (events_.size() != curr_nb_of_triggers_recorded + 1){
                throw std::runtime_error("Action did not record its trigger");
            }
            // only do one trigger
            break;
        }
    }
};

void DynamicSampler::Finish(){
    finished_ = true;
    motion_planner_.PrintLog(0, true);
};