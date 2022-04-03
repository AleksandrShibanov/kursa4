#include "edge.hpp"

#include <cmath>
#include <iostream>
#include <vector>
#include <cassert>

#include "point.hpp"

Edge::Edge(const Point& p1, const Point& p2) : p1(p1), p2(p2) {}

double Edge::distance(const Point& p) const 
{
    return std::fabs((p2.x - p1.x) * (p1.y - p.y) - (p1.x - p.x) * (p2.y - p1.y)) / std::hypot(p2.y - p1.y, p2.x - p1.x);
}

double Edge::length()
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
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

double Edge::degree(const Edge& edge) const 
{
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

bool Edge::hasCommonPoint(const Edge& edge) const 
{
    return p1 == edge.p1 || p2 == edge.p2 || p1 == edge.p2 || p2 == edge.p1;
}

Point Edge::getCommonPoint(const Edge& edge) const
{
    assert(hasCommonPoint(edge) == true);

    if (p1 == edge.p1 || p1 == edge.p2)
        return p1;
    return p2;
}

Point Edge::getNotCommonPoint(const Edge& edge) const
{
    assert(hasCommonPoint(edge) == true);
    
    if (p1 == edge.p1 || p1 == edge.p2)
        return p2;
    return p1;
}

bool Edge::operator==(const Edge& rhs) const 
{
    return (p1 == rhs.p1 && p2 == rhs.p2) || (p1 == rhs.p2 && p2 == rhs.p1);
}

bool Edge::operator !=(const Edge &rhs) const 
{
    return !(rhs == *this);
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored.
bool Edge::isIntersects(const Edge& aEdge, Point& aIntersectionPoint) const  // код из интернетов
{
    double s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p2.x - p1.x;
    s10_y = p2.y - p1.y;
    s32_x = aEdge.p2.x - aEdge.p1.x;
    s32_y = aEdge.p2.y - aEdge.p1.y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return false; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p1.x - aEdge.p1.x;
    s02_y = p1.y - aEdge.p1.y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return false; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return false; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return false; // No collision
    
    // Collision detected
    t = t_numer / denom;
    aIntersectionPoint.x = p1.x + (t * s10_x);
    aIntersectionPoint.y = p1.y + (t * s10_y);
    return true;
}

double Edge::clockwiseAngle(const Edge& aEdge) const
{
    double dot = (p1.x-p2.x)*(aEdge.p1.x-aEdge.p2.x) + (p1.y-p2.y)*(aEdge.p1.y-aEdge.p2.y);      
    double det = (p1.x-p2.x)*(aEdge.p1.y-aEdge.p2.y) - (p1.y-p2.y)*(aEdge.p1.x-aEdge.p2.x);      
    return atan2(det, dot);
}