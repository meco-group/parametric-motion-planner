
#include "core/moving_obstacle.hpp"

json MovingObstacle::ToJson() const {
    json j;
    j["width"] = width_;
    j["height"] = height_;
    j["travelled_trajectory"] = travelled_trajectory_.ToJson();
    return j;
}

// Linearly moving obstacle
LinearMovingObstacle::LinearMovingObstacle(double width, double height,
                                           Point2D<double> start, 
                                           Point2D<double> end, 
                                           double movement_duration, 
                                           bool loop) : 
                                           MovingObstacle() {
    start_ = start;
    position_ = start;
    end_ = end;
    movement_duration_ = movement_duration;
    loop_ = loop;
    width_ = width;
    height_ = height;
}

void LinearMovingObstacle::Update(double dt) {
    // compute remainder of time divided by movement duration
    double t = current_time_;
    while (t > movement_duration_){
        t -= movement_duration_;
    }

    // Update time
    current_time_ += dt;
    t += dt;

    // looping logic
    if (t > movement_duration_){
        if (loop_){
            // revert the movement
            Point2D<double> temp = start_;
            start_ = end_;
            end_ = temp;
            t -= movement_duration_;
        } else {
            start_ = end_;
            position_ = end_;
        }
    }

    t = t/movement_duration_;
    position_ = start_ + (end_ - start_)*t;
    // Append the new position to the travelled trajectory
    AppendToTravelledTrajectory(current_time_, position_.x(), position_.y(), 
                                0, 0, 0, 0);
}

void LinearMovingObstacle::Reset() {
    if (!(start_ == original_start_)){
        end_ = start_;
        start_ = original_start_;
    }
    current_time_ = 0;

    Update(0);
}

// Appearing obstacle
AppearingStaticObstacle::AppearingStaticObstacle(double width, double height, 
                                                 Point2D<double> position, 
                                                 double appearance_time, 
                                                 double disappearance_time) : 
                                                 MovingObstacle() {
    position_ = Point2D<double>(-1000, -1000);
    width_ = width;
    height_ = height;
    position_to_appear_at_ = position;
    appearance_time_ = appearance_time;
    disappearance_time_ = disappearance_time;
}

void AppearingStaticObstacle::Update(double dt) {
    current_time_ += dt;
    if (current_time_ >= appearance_time_ && 
        current_time_ <= disappearance_time_){
        position_ = position_to_appear_at_;
    } else {
        position_.SetX(-1000);
        position_.SetY(-1000);
    }
    AppendToTravelledTrajectory(current_time_, position_.x(), 
                                position_.y(), 0, 0, 0, 0);
}