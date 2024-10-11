
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