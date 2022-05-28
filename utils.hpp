#pragma once

#include <cmath>
#include <ostream>
#include <vector>
#include <boost/functional/hash.hpp>
#include <optional>
#include <Eigen/Dense>

constexpr float PRECISION = 1e-5;

template <class T>
class MyHash;

template<>
struct MyHash<Edge>
{
    std::size_t operator()(const Edge& sEdge) const 
    {
        std::size_t res = 0;
        if (sEdge.p1.x() < sEdge.p2.x())
        {
            boost::hash_combine(res, sEdge.p1.x());
            boost::hash_combine(res, sEdge.p1.y());
            boost::hash_combine(res, sEdge.p2.x());
            boost::hash_combine(res, sEdge.p2.y());
        }
        else 
        {
            boost::hash_combine(res, sEdge.p2.x());
            boost::hash_combine(res, sEdge.p2.y());
            boost::hash_combine(res, sEdge.p1.x());
            boost::hash_combine(res, sEdge.p1.y());
        }
        return res;
    }
};

struct vecCompare {
    bool operator() (Eigen::Vector2f v, Eigen::Vector2f w) const {
        for (int i = 0; i < 2; ++i) {
            if (v(i) < w(i)) return true;
            if (v(i) > w(i)) return false;
        }

        return false;
    }
};