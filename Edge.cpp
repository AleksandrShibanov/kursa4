#include "edge.hpp"

#include <cmath>
#include <iostream>
#include <vector>
#include <cassert>
#include <Eigen/Dense>
#include "utils.hpp"


Edge::Edge(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float precision) : p1(p1), p2(p2), precision(precision) {
    assert(!p1.isApprox(p2, precision));
}

double Edge::distance(const Eigen::Vector2f& p) const 
{
    return std::fabs((p2.x() - p1.x()) * (p1.y() - p.y()) - (p1.x() - p.x()) * (p2.y() - p1.y())) / std::hypot(p2.y() - p1.y(), p2.x() - p1.x());
}

double Edge::length() const
{
    return std::hypot(p2.x() - p1.x(), p2.y() - p1.y());
}

double Edge::cross(const Edge& edge) const
{
    std::pair<double, double> vec1(p2.x()-p1.x(), p2.y()-p1.y());
    std::pair<double, double> vec2(edge.p2.x()-edge.p1.x(), edge.p2.y()-edge.p1.y());
    return vec1.first * vec2.second - vec1.second * vec2.first;
}


void Edge::MakeBad() 
{
    bad = true;
}

void Edge::MakeGood() 
{
    bad = false;
}

bool Edge::isBad() 
{
    return bad;
}

bool Edge::hasCommonPoint(const Edge& edge) const 
{
    return p1.isApprox(edge.p1, precision) ||
           p1.isApprox(edge.p2, precision) || 
           p2.isApprox(edge.p1, precision) || 
           p2.isApprox(edge.p2, precision);
}

Eigen::Vector2f Edge::getCommonPoint(const Edge& edge) const
{
    assert(hasCommonPoint(edge) == true);

    if (p1.isApprox(edge.p1, precision) || p1.isApprox(edge.p2, precision))
        return p1;
    return p2;
}

Eigen::Vector2f Edge::getNotCommonPoint(const Edge& edge) const
{
    assert(hasCommonPoint(edge) == true);
    
    if (p1.isApprox(edge.p1, precision) || p1.isApprox(edge.p2, precision))
        return p2;
    return p1;
}

std::pair<Eigen::Vector2f, Eigen::Vector2f> Edge::getPointsOrderedByX() const
{
    if (p1.x() < p2.x())
        return std::make_pair(p1, p2);
    else 
        return std::make_pair(p2, p1);
}

std::pair<Eigen::Vector2f, Eigen::Vector2f> Edge::getPointsOrderedByY() const
{
    if (p1.y() < p2.y())
        return std::make_pair(p1, p2);
    else 
        return std::make_pair(p2, p1);
}

bool Edge::operator==(const Edge& rhs) const 
{
    return (p1.isApprox(rhs.p1, precision) && p2.isApprox(rhs.p2, precision)) 
            || 
            (p1.isApprox(rhs.p2, precision) && p2.isApprox(rhs.p1, precision)) ;
}

bool Edge::operator !=(const Edge &rhs) const 
{
    return !(rhs == *this);
}

bool Edge::operator <(const Edge &rhs) const 
{
    vecCompare cmp;
    if (cmp(p1, rhs.p1)) return true;
    return false;
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored.
std::optional<Eigen::Vector2f> Edge::isIntersects(const Edge& aEdge) const 
{
    double s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p2.x() - p1.x();
    s10_y = p2.y() - p1.y();
    s32_x = aEdge.p2.x() - aEdge.p1.x();
    s32_y = aEdge.p2.y() - aEdge.p1.y();

    denom = s10_x * s32_y - s32_x * s10_y;
    if (std::fabs(denom) < precision)
        return std::nullopt; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p1.x() - aEdge.p1.x();
    s02_y = p1.y() - aEdge.p1.y();
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return std::nullopt; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return std::nullopt; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return std::nullopt; // No collision
    
    // Collision detected
    t = t_numer / denom;
    return Eigen::Vector2f(p1.x() + (t * s10_x), p1.y() + (t * s10_y));
}

double Edge::clockwiseAngle(const Edge& aEdge) const
{
    double dot = (p1.x()-p2.x())*(aEdge.p1.x()-aEdge.p2.x()) + (p1.y()-p2.y())*(aEdge.p1.y()-aEdge.p2.y());      
    double det = (p1.x()-p2.x())*(aEdge.p1.y()-aEdge.p2.y()) - (p1.y()-p2.y())*(aEdge.p1.x()-aEdge.p2.x());      
    return atan2(det, dot);
}
