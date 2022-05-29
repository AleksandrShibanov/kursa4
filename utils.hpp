#pragma once

#include <cmath>
#include <ostream>
#include <vector>
#include <boost/functional/hash.hpp>
#include <optional>
#include <Eigen/Dense>

constexpr double HEIGHT = 900.0;
constexpr double WIDTH = 1600.0;

constexpr uint8_t gNumberOfThreads = 8;

constexpr size_t N = 10000;  // number of points
constexpr size_t sZonesCount = 1;  // number of parallel calculated triangulation 'bricks'

constexpr size_t MAX_ITERATIONS = 10000;
constexpr size_t MAX_MERGE_ITERATIONS = 150;
constexpr double SPLIT = 0.03;

constexpr float ADF_PRECISION = 1e-7;
constexpr float INC_PRECISION = 1e-15;
constexpr float MERGE_PRECISION = 1e-30;


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
        Eigen::Vector2f b(v.x()-w.x(), v.y()-w.y());
        Eigen::Vector2f zero(0.0, 0.0);
        if (b.isApprox(zero, ADF_PRECISION))
            return false;
        for (int i = 0; i < 2; ++i) {
            if (v(i) < w(i)) return true;
            if (v(i) > w(i)) return false;
        }

        return false;
    }
};

template<class Container>
auto sinserter(Container& c){
    using std::end;
    return std::inserter(c, end(c));
}