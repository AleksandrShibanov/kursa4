#pragma once

#include <cmath>
#include <ostream>
#include <vector>
#include <boost/functional/hash.hpp>
#include <optional>

#include "point.hpp"

constexpr double PI = 3.141592653589793238463;

struct Edge 
{
    Point p1;
    Point p2;
    bool bad = false;

    Edge() = default;
    Edge(const Point& p1, const Point& p2);

    double distance(const Point& p) const;

    double length();

    void MakeBad();

    void MakeGood();

    bool isBad();

    double degree(const Edge& edge) const;

    bool hasCommonPoint(const Edge& edge) const;
    Point getCommonPoint(const Edge& edge) const;
    Point getNotCommonPoint(const Edge& edge) const;

    std::pair<Point, Point> getPointsOrderedByX() const;
    std::pair<Point, Point> getPointsOrderedByY() const;

    double clockwiseAngle(const Edge& aEdge) const;

    std::optional<Point> isIntersects(const Edge& aEdge) const;

    bool operator ==(const Edge& rhs) const;

    bool operator !=(const Edge &rhs) const;

    friend std::ostream &operator<<(std::ostream &os, const Edge &edge)
    {
        os << "X1: " << edge.p1.x << " Y1: " << edge.p1.y << std::endl;
        os << "X2: " << edge.p2.x << " Y2: " << edge.p2.y << std::endl;
        return os;
    }
};

template <class T>
class MyHash;

template<>
struct MyHash<Edge>
{
    std::size_t operator()(const Edge& sEdge) const 
    {
        std::size_t res = 0;
        if (sEdge.p1 < sEdge.p2)
        {
            boost::hash_combine(res, sEdge.p1.x);
            boost::hash_combine(res, sEdge.p1.y);
            boost::hash_combine(res, sEdge.p2.x);
            boost::hash_combine(res, sEdge.p2.y);
        }
        else 
        {
            boost::hash_combine(res, sEdge.p2.x);
            boost::hash_combine(res, sEdge.p2.y);
            boost::hash_combine(res, sEdge.p1.x);
            boost::hash_combine(res, sEdge.p1.y);
        }
        return res;
    }
};