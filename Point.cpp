#include "point.hpp"

#include <cmath>
#include <ostream>
#include <vector>



    Point::Point(double x, double y) : x(x), y(y) {}

    double Point::distance(const Point& p) const {
        return hypot(x - p.x, y - p.y);
    }

    double Point::norm() const {
        return hypot(x, y);
    }

    bool Point::operator ==(const Point& rhs) const {
        return (std::fabs(x - rhs.x) < std::numeric_limits<double>::epsilon()) &&
               (std::fabs(y - rhs.y) < std::numeric_limits<double>::epsilon());
    }

    bool Point::operator !=(const Point& rhs) const {
        return !(rhs == *this);
    }


    bool Point::operator >(const Point& rhs) const {
        return this->norm() > rhs.norm();
    }

    bool Point::operator <(const Point& rhs) const {
        return rhs > *this;
    }