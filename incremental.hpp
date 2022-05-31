#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include "triangle.hpp"
#include "edge.hpp"


/**
 * Bowyer Watson Algorithm https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
 */

struct Incremental {
    std::vector<Eigen::Vector2f> points;
    std::vector<Eigen::Vector2f> processed;


    explicit Incremental(std::vector<Eigen::Vector2f>& points);
    std::pair<Triangle, Triangle> createBigTriangle(Eigen::Vector2f& p1_1, Eigen::Vector2f& p2_1, Eigen::Vector2f& p3_1, Eigen::Vector2f& p4_1);
    std::vector<Triangle> triangulate();

};