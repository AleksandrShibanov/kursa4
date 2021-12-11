#pragma once

#include <cmath>
#include <ostream>
#include <vector>


struct Point {
    double x;
    double y;

    Point(double x = 0, double y = 0);

    double distance(const Point& p) const;

    double norm() const;

    bool operator ==(const Point& rhs) const;

    bool operator !=(const Point& rhs) const;


    bool operator >(const Point& rhs) const ;

    bool operator <(const Point& rhs) const;
};