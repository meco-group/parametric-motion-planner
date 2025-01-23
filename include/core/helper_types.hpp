#ifndef __HELPER_TYPES__
#define __HELPER_TYPES__

#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Enumeration of supported planning methods
enum PlannerMethod{
    P2P = 0,
    OCP = 1,
    ARENA = 2
};

// Enumeration of environment grid cell occupancy states
enum CellOccupancy{
    FREE = 0,
    DELETED = 1,
    OCCUPIED_STATIC = 2,
    OCCUPIED_DYNAMIC = 3,
    OCCUPIED_STATIC_AND_DYNAMIC = 4
};

// Class for a 2D point (world coordinates (double) or grid cell (int))
template<typename T>
class Point2D {
    public:
        Point2D(){x_ = 0.0; y_ = 0.0;};
        Point2D(T x, T y){ x_ = x; y_ = y;};
        
        // getters
        T x() const { return x_;};
        T y() const { return y_;};
        
        Point2D<T> Copy() const { return Point2D(x_, y_);};

        // take the values of another point
        template <typename U>
        void CopyValues(const Point2D<U> &other){
            x_ = T(other.x()); y_ = T(other.y());
        }

        double Distance(const Point2D<T> &other) const {
            return sqrt(pow(x_ - other.x(), 2) + pow(y_ - other.y(), 2));
        }
        T ManhattanDistance(const Point2D<T> &other) const {
            return abs(x_ - other.x()) + abs(y_ - other.y());
        }
        double DistanceToLine(const Point2D<T> &line_start, 
                              const Point2D<T> &line_end) const {
            double num = std::abs((line_end.y() - line_start.y()) * x_ - 
                             (line_end.x() - line_start.x()) * y_ + 
                             line_end.x() * line_start.y() - 
                             line_end.y() * line_start.x());
            double den = sqrt(pow(line_end.y() - line_start.y(), 2) + 
                              pow(line_end.x() - line_start.x(), 2));
            return num / den;
        }

        template<typename U = T, typename = typename std::enable_if<std::is_same<U, double>::value>::type>
        void Rotate(double angle){
            double temp = x_;
            SetX(x_*std::cos(angle) - y_*std::sin(angle));
            SetY(temp*std::sin(angle) + y_*std::cos(angle));
        }

        double Norm() const {return sqrt(x_*x_ + y_*y_);}

        // setters
        void SetX(T x){ x_ = x;};
        void SetY(T y){ y_ = y;};
        void SetValues(T x, T y){ x_ = x; y_ = y;};

        // printing overloading
        friend std::ostream& operator<<(std::ostream &out, Point2D const &v) {
            out << "(" << v.x() << ", " << v.y() << ")";
            return out;
        }

        // operation overloading
        bool operator==(const Point2D<T> &other) const {
            return x_ == other.x() && y_ == other.y();
        }
        Point2D<T> operator-(const Point2D<T> &other) const {
            return Point2D<T>(x_ - other.x(), y_ - other.y());
        }
        Point2D<T>& operator-=(const Point2D<T> &other){
            x_ -= other.x(); y_ -= other.y(); return *this;
        }
        Point2D<T> operator+(const Point2D<T> &other) const {
            return Point2D<T>(x_ + other.x(), y_ + other.y());
        }
        Point2D<T>& operator+=(const Point2D<T> &other){
            x_ += other.x(); y_ += other.y(); return *this;
        }
        Point2D<T> operator*(double scalar) const {
            return Point2D<T>(x_ * scalar, y_ * scalar);
        }
        Point2D<T>& operator*=(double scalar){
            x_ *= scalar; y_ *= scalar; return *this;
        }
        Point2D<T> operator*(const Point2D<T> &other) const {
            return Point2D<T>(x_ * other.x(), y_ * other.y());
        }

        // Convert a cell to world coordinates
        // The middle point of the cell is converted
        template<typename U = T, typename = typename std::enable_if<std::is_same<U, int>::value>::type>
        Point2D<double> ConvertCellToWorld(double cell_width, double cell_height) const {
            return Point2D<double>((x_ + 0.5) * cell_width, (y_ + 0.5) * cell_height);
        }

        // Convert a world point to cell coordinates
        // The cell in which the world point is located is returned
        template<typename U = T, typename = typename std::enable_if<std::is_same<U, double>::value>::type>
        Point2D<int> ConvertWorldToCell(double cell_width, double cell_height) const {
            return Point2D<int>(int(x_ / cell_width) - (x_ < 0), 
                                int(y_ / cell_height) - (y_ < 0));
        }
        template<typename U = T, typename = typename std::enable_if<std::is_same<U, double>::value>::type>
        Point2D<int> ConvertWorldToCell(double cell_width, double cell_height, 
                                        Point2D<int>& cell) const {
            cell.SetX(int(x_ / cell_width) - (x_ < 0));
            cell.SetY(int(y_ / cell_height) - (y_ < 0));
            return cell;
        }

        bool operator<(const Point2D<T> &other) const {
            return x_ < other.x() || (x_ == other.x() && y_ < other.y());
        }

        json ToJson() const { return json{{"x", x_}, {"y", y_}};};
    
    private:
        T x_;
        T y_;
};

template <typename T>
struct Point2DHash {
    std::size_t operator()(const Point2D<T>& obj) const {
        // Example hash function combining x and y
        return std::hash<T>()(obj.x()) + 10000 * std::hash<T>()(obj.y());
    }
};

#endif