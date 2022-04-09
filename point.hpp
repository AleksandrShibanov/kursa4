#pragma once

#include <cmath>
#include <ostream>
#include <vector>

#include <SFML/Graphics.hpp>

struct Point {
    double x;
    double y;
    mutable sf::Color color = sf::Color::White;

    Point(double x = 0, double y = 0);

    double distance(const Point& p) const;

    double norm() const;

    double getAngle(const Point& lhs, const Point& rhs) const;

    bool operator ==(const Point& rhs) const;

    bool operator !=(const Point& rhs) const;

    bool operator <(const Point& rhs) const;

    friend std::ostream& operator<<(std::ostream &os, const Point& sPoint)
    {
        os << sPoint.x << " " << sPoint.y;
        return os;
    }
};