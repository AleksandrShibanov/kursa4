#pragma once

#include <vector>
#include "edge.hpp"
#include "utils.hpp"


struct Triangle {
    Eigen::Vector2f A;
    Eigen::Vector2f B;
    Eigen::Vector2f C;
    std::vector<Edge> edges;
    bool bad = false;

    Triangle(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C);

    bool hasCommonEdge(const Triangle &triangle) const;

    bool hasCommonPoint(const Triangle &triangle) const;

    void MakeBad() ;
    void MakeGood() ;

    bool isBad() ;

    Eigen::Vector2f getPoint(const Edge& edge);

    bool containsPoint(const Eigen::Vector2f& v) const;

    bool circumscribedCircleContains(const Eigen::Vector2f& D) const ;

    // bool operator ==(const Triangle& rhs) const ;

    // bool operator !=(const Triangle& rhs) const;

    bool containsEdge(const Edge &edge) const;
};
