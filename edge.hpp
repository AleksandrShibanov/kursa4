#pragma once

#include <cmath>
#include <ostream>
#include <vector>
#include "point.hpp"

constexpr double PI = 3.141592653589793238463;

struct Edge {
    Point p1;
    Point p2;
    bool bad = false;

    Edge(const Point& p1, const Point& p2);

    double distance(const Point& p) const;

    double length();

    void MakeBad();

    void MakeGood();

    bool isBad();

    double degree(const Edge& edge) const;

    bool hasCommonPoint(const Edge& edge) const ;

    bool operator ==(const Edge& rhs) const ;

    bool operator !=(const Edge &rhs) const ;

    bool operator <(const Edge& rhs) const ;

    bool operator>(const Edge &rhs) const ;

    friend std::ostream &operator<<(std::ostream &os, const Edge &edge);
};