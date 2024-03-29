#include "point.hpp"

#include <cmath>
#include <tuple>


Point::Point(double x, double y)
: x(x)
, y(y) 
{}

double Point::distance(const Point& p) const 
{
    return hypot(x - p.x, y - p.y);
}

double Point::norm() const 
{
    return hypot(x, y);
}

double Point::getAngle(const Point& lhs, const Point& rhs) const
{
    Point ab = { x - lhs.x, y - lhs.y };
    Point cb = { x - rhs.x, y - rhs.y };

    double dot = (ab.x * cb.x + ab.y * cb.y); // dot product
    double cross = (ab.x * cb.y - ab.y * cb.x); // cross product

    double alpha = atan2(cross, dot);
    return alpha;
}


bool Point::operator ==(const Point& rhs) const 
{
    return (std::fabs(x - rhs.x) < 100 * __DBL_EPSILON__) &&
            (std::fabs(y - rhs.y) < 100 * __DBL_EPSILON__);
}

bool Point::operator !=(const Point& rhs) const 
{
    return !(*this == rhs);
}

bool Point::operator <(const Point& rhs) const 
{
    return std::tie(x, y) < std::tie(rhs.x, rhs.y);
}