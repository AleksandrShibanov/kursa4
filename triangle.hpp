#pragma once

#include <vector>
#include "point.hpp"
#include "edge.hpp"


struct Triangle {
    Point A;
    Point B;
    Point C;
    std::vector<Edge> edges;
    bool bad = false;

    Triangle(const Point& A, const Point& B, const Point& C);

    bool hasCommonEdge(const Triangle &triangle) const;

    bool hasCommonPoint(const Triangle &triangle) const;

    void MakeBad() ;
    void MakeGood() ;

    bool isBad() ;

    Point getPoint(const Edge& edge);

    bool containsPoint(const Point& v) const;

    bool circumscribedCircleContains(const Point& D) const ;

    bool operator ==(const Triangle& rhs) const ;

    bool operator !=(const Triangle& rhs) const;

    bool containsEdge(const Edge &edge) const;
};
