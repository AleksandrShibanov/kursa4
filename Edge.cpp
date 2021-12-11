#include "edge.hpp"

#include <cmath>
#include <ostream>
#include <vector>
#include "point.hpp"


    Edge::Edge(const Point& p1, const Point& p2) : p1(p1), p2(p2) {}

    double Edge::distance(const Point& p) const {
        return std::fabs((p2.x - p1.x) * (p1.y - p.y) - (p1.x - p.x) * (p2.y - p1.y)) / std::hypot(p2.y - p1.y, p2.x - p1.x);
    }

    double Edge::length(){
        return std::hypot(p2.x - p1.x, p2.y - p1.y);
    }

    void Edge::MakeBad() {
        bad = true;
    }

    void Edge::MakeGood() {
        bad = false;
    }

    bool Edge::isBad() {
        return bad;
    }

    double Edge::degree(const Edge& edge) const {
        double deltaX1 = p1.x - p2.x;
        double deltaX2 = edge.p1.x - edge.p2.x;
        double deltaY1 = p1.y - p2.y;
        double deltaY2 = edge.p1.y - edge.p2.y;
        double n = deltaX1 * deltaX2 + deltaY1 * deltaY2;
        Point vec1{deltaX1, deltaY1};
        Point vec2{deltaX2, deltaY2};
        double d = vec1.norm() * vec2.norm();
        double rad = acos(n / d);
        double ret = rad * (180 / PI);
        return ret;
    }

    bool Edge::hasCommonPoint(const Edge& edge) const {
        return p1 == edge.p1 || p2 == edge.p2 || p1 == edge.p2 || p2 == edge.p1;
    }

    bool Edge::operator ==(const Edge& rhs) const {
        return (p1 == rhs.p1 && p2 == rhs.p2) || (p1 == rhs.p2 && p2 == rhs.p1);
    }

    bool Edge::operator !=(const Edge &rhs) const {
        return !(rhs == *this);
    }

    bool Edge::operator <(const Edge& rhs) const {
        Point ZERO(0, 0);
        return distance(ZERO) < rhs.distance(ZERO);
    }

    bool Edge::operator>(const Edge &rhs) const {
        return rhs < *this;
    }