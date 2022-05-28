#pragma once

#include <cmath>
#include <ostream>
#include <vector>
#include <boost/functional/hash.hpp>
#include <optional>
#include <Eigen/Dense>

constexpr double PI = 3.141592653589793238463;

struct Edge 
{
    Eigen::Vector2f p1;
    Eigen::Vector2f p2;
    bool bad = false;

    Edge() = default;
    Edge(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);

    double distance(const Eigen::Vector2f& p) const;

    double length() const;

    double cross(const Edge& edge) const;

    void MakeBad();

    void MakeGood();

    bool isBad();

    bool hasCommonPoint(const Edge& edge) const;
    Eigen::Vector2f getCommonPoint(const Edge& edge) const;
    Eigen::Vector2f getNotCommonPoint(const Edge& edge) const;

    std::pair<Eigen::Vector2f, Eigen::Vector2f> getPointsOrderedByX() const;
    std::pair<Eigen::Vector2f, Eigen::Vector2f> getPointsOrderedByY() const;

    double clockwiseAngle(const Edge& aEdge) const;

    std::optional<Eigen::Vector2f> isIntersects(const Edge& aEdge) const;

    std::pair<double, double> getVector() const;

    bool operator ==(const Edge& rhs) const;

    bool operator !=(const Edge &rhs) const;

    friend std::ostream &operator<<(std::ostream &os, const Edge &edge)
    {
        os << "X1: " << edge.p1.x() << " Y1: " << edge.p1.y() << std::endl;
        os << "X2: " << edge.p2.x() << " Y2: " << edge.p2.y() << std::endl;
        return os;
    }
};