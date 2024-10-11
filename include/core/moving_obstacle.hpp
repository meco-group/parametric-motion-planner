#ifndef __MOVING_OBSTACLE__
#define __MOVING_OBSTACLE__

#include "helper_types.hpp"
#include "environment.hpp"
#include "trajectory.hpp"

class MovingObstacle {
    public:
        virtual ~MovingObstacle() = default;

        virtual void Update(double dt) = 0;
        bool operator==(const MovingObstacle& other) const;

        // Basic getters
        Point2D<double> GetPosition() const { return position_;};
        double GetWidth() const { return width_;};
        double GetHeight() const { return height_;};

        void AppendToTravelledTrajectory(double t, double px, double py, 
                                         double vx, double vy, double ax, 
                                         double ay){
            travelled_trajectory_.Append(t, px, py, vx, vy, ax, ay);
        }

        json ToJson() const; 

    protected:
        Point2D<double> position_;
        double width_ = 0;
        double height_ = 0;
        double current_time_ = 0;

        Trajectory travelled_trajectory_;
};


// Linearly moving obstacle
class LinearMovingObstacle : public MovingObstacle {
    public:
        LinearMovingObstacle(double width, double height, 
                             Point2D<double> start, Point2D<double> end, 
                             double movement_duration, bool loop);

        void Update(double dt) override;

    private:
        Point2D<double> start_;
        Point2D<double> end_;
        double movement_duration_;
        bool loop_;
};


#endif