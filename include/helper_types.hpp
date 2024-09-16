#ifndef __HELPER_TYPES__
#define __HELPER_TYPES__

#include <iostream>

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
    OCCUPIED = 2
};

// Class for a 2D point (world coordinates (double) or grid cell (int))
template<typename T>
class Point2D {
    public:
        Point2D(){
            x_ = 0.0;
            y_ = 0.0;
        };

        Point2D(T x, T y){
            x_ = x;
            y_ = y;
        };
        
        // getters
        double x() const { return x_;};
        double y() const { return y_;};
        
        // setters
        void SetX(T x){ x_ = x;};
        void SetY(T y){ y_ = y;};
        void SetValues(T x, T y){ x_ = x; y_ = y;};
        
        void CopyValues(const Point2D<T> &other){
            x_ = other.x(); y_ = other.y();
        }

        // printing overloading
        friend std::ostream& operator<<(std::ostream &out, Point2D &v) {
            out << "(" << v.x() << ", " << v.y() << ")";
            return out;
        }

        // equality overloading
        bool operator==(const Point2D<T> &other) const {
            return x_ == other.x() && y_ == other.y();
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
            return Point2D<int>(int(x_ / cell_width), int(y_ / cell_height));
        }
        template<typename U = T, typename = typename std::enable_if<std::is_same<U, double>::value>::type>
        Point2D<int> ConvertWorldToCell(double cell_width, double cell_height, 
                                        Point2D<int>& cell) const {
            cell.SetX(int(x_ / cell_width));
            cell.SetY(int(y_ / cell_height));
            return cell;
        }

        bool operator<(const Point2D<T> &other) const {
            return x_ < other.x() || (x_ == other.x() && y_ < other.y());
        }
    
    private:
        T x_;
        T y_;
};

#endif